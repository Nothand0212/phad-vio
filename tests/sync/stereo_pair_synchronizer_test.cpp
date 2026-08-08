#include "phad/sync/stereo_pair_synchronizer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

  using phad::common::Timestamp;
  using phad::sensor::CameraId;
  using phad::sensor::Image;
  using phad::sensor::ImageFrameEvent;
  using phad::sensor::ImuMeasurement;
  using phad::sensor::StereoFrame;
  using phad::sensor::StereoImuPacket;
  using phad::sync::PushStatus;
  using phad::sync::StereoPairSynchronizer;
  using phad::sync::StereoPairSynchronizerOptions;

  [[nodiscard]] Image makeTinyImage( std::uint8_t tag )
  {
    return Image{ 1, 1, 1, std::vector<std::uint8_t>{ tag } };
  }

  [[nodiscard]] ImageFrameEvent makeEvent( CameraId camera, std::int64_t ns,
                                           std::uint8_t tag )
  {
    return ImageFrameEvent{ camera, Timestamp{ ns }, makeTinyImage( tag ) };
  }

  [[nodiscard]] std::uint8_t pixelTag( const Image& image )
  {
    const auto pixels = image.pixels<std::uint8_t>();
    EXPECT_TRUE( pixels.has_value() );
    EXPECT_EQ( pixels->size(), 1U );
    return ( *pixels )[ 0 ];
  }

  void expectPushOk( StereoPairSynchronizer& sync, ImageFrameEvent event )
  {
    EXPECT_EQ( sync.pushImage( std::move( event ) ), PushStatus::kOk );
  }

  void expectImuPushOk( StereoPairSynchronizer& sync,
                        ImuMeasurement          measurement )
  {
    EXPECT_EQ( sync.pushImu( measurement ), PushStatus::kOk );
  }

  void pushPair( StereoPairSynchronizer& sync, std::int64_t ns,
                 std::uint8_t tag )
  {
    expectPushOk( sync, makeEvent( CameraId::kLeft, ns, tag ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, ns,
                                   static_cast<std::uint8_t>( 100 + tag ) ) );
  }

  /// IMU 样本:accel x = ax(可精确插值),其余分量非零以检测串位。
  [[nodiscard]] ImuMeasurement makeImu( std::int64_t ns, double ax )
  {
    ImuMeasurement m;
    m.timestamp         = Timestamp{ ns };
    m.accel_mps2[ 0 ]   = ax;
    m.accel_mps2[ 1 ]   = 1.0;
    m.accel_mps2[ 2 ]   = 2.0;
    m.gyro_radps[ 0 ]   = 0.5 * ax;
    m.gyro_radps[ 1 ]   = 3.0;
    m.gyro_radps[ 2 ]   = 4.0;
    return m;
  }

  /// 段内相邻样本时间戳差之和(非 imu_gap 段必须 ≡ 图像间隔)。
  [[nodiscard]] std::int64_t segmentDt( const StereoImuPacket& packet )
  {
    std::int64_t sum = 0;
    for ( std::size_t i = 1; i < packet.samples.size(); ++i )
    {
      sum += packet.samples[ i ].timestamp.nanoseconds() -
             packet.samples[ i - 1 ].timestamp.nanoseconds();
    }
    return sum;
  }

  TEST( StereoPairSynchronizerTest, EqualLengthExactPairsAll )
  {
    StereoPairSynchronizer sync;
    constexpr int          n = 5;
    for ( int i = 0; i < n; ++i )
    {
      const std::int64_t ns = 1000 + i;
      expectPushOk( sync, makeEvent( CameraId::kLeft, ns,
                                     static_cast<std::uint8_t>( i ) ) );
      expectPushOk( sync,
                    makeEvent( CameraId::kRight, ns,
                               static_cast<std::uint8_t>( 100 + i ) ) );
    }

    for ( int i = 0; i < n; ++i )
    {
      const auto frame = sync.tryPop();
      ASSERT_TRUE( frame.has_value() );
      EXPECT_EQ( frame->timestamp.nanoseconds(), 1000 + i );
      EXPECT_EQ( pixelTag( frame->left ), static_cast<std::uint8_t>( i ) );
      EXPECT_EQ( pixelTag( frame->right ),
                 static_cast<std::uint8_t>( 100 + i ) );
    }
    EXPECT_FALSE( sync.tryPop().has_value() );

    const auto& d = sync.diagnostics();
    EXPECT_EQ( d.emitted_stereo, static_cast<std::uint64_t>( n ) );
    EXPECT_EQ( d.dropped_left, 0U );
    EXPECT_EQ( d.dropped_right, 0U );
    EXPECT_LE( d.max_left_queue, 1U );
    EXPECT_LE( d.max_right_queue, 1U );
  }

  TEST( StereoPairSynchronizerTest, Mh04StyleLeftLeadingOrphan )
  {
    StereoPairSynchronizer sync;
    expectPushOk( sync, makeEvent( CameraId::kLeft, 1, 1 ) );
    expectPushOk( sync, makeEvent( CameraId::kLeft, 2, 2 ) );
    expectPushOk( sync, makeEvent( CameraId::kLeft, 3, 3 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 2, 12 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 3, 13 ) );

    const auto first = sync.tryPop();
    ASSERT_TRUE( first.has_value() );
    EXPECT_EQ( first->timestamp.nanoseconds(), 2 );
    EXPECT_EQ( pixelTag( first->left ), 2 );
    EXPECT_EQ( pixelTag( first->right ), 12 );

    const auto second = sync.tryPop();
    ASSERT_TRUE( second.has_value() );
    EXPECT_EQ( second->timestamp.nanoseconds(), 3 );

    EXPECT_FALSE( sync.tryPop().has_value() );
    EXPECT_EQ( sync.diagnostics().emitted_stereo, 2U );
    EXPECT_EQ( sync.diagnostics().dropped_left, 1U );
    EXPECT_EQ( sync.diagnostics().dropped_right, 0U );
  }

  TEST( StereoPairSynchronizerTest, V102StyleRightTrailingOrphanNeedsFlush )
  {
    StereoPairSynchronizer sync;
    expectPushOk( sync, makeEvent( CameraId::kLeft, 10, 1 ) );
    expectPushOk( sync, makeEvent( CameraId::kLeft, 20, 2 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 10, 11 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 20, 12 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 30, 13 ) );

    EXPECT_TRUE( sync.tryPop().has_value() );
    EXPECT_TRUE( sync.tryPop().has_value() );
    EXPECT_FALSE( sync.tryPop().has_value() );
    EXPECT_EQ( sync.diagnostics().dropped_right, 0U );

    sync.flush();
    EXPECT_EQ( sync.diagnostics().emitted_stereo, 2U );
    EXPECT_EQ( sync.diagnostics().dropped_left, 0U );
    EXPECT_EQ( sync.diagnostics().dropped_right, 1U );
  }

  TEST( StereoPairSynchronizerTest, V203StyleScatteredRightExtrasMatchIntersection )
  {
    // Left ∩ Right = {10,20,40}; right-only orphans scattered mid-span and tail.
    const std::vector<std::int64_t> left_stamps{ 10, 20, 40 };
    const std::vector<std::int64_t> right_stamps{ 10, 20, 30, 40, 50, 60, 70 };

    StereoPairSynchronizer sync;
    std::size_t            li = 0;
    std::size_t            ri = 0;
    while ( li < left_stamps.size() || ri < right_stamps.size() )
    {
      const bool push_left =
          li < left_stamps.size() &&
          ( ri >= right_stamps.size() ||
            left_stamps[ li ] <= right_stamps[ ri ] );
      if ( push_left )
      {
        expectPushOk(
            sync,
            makeEvent( CameraId::kLeft, left_stamps[ li ],
                       static_cast<std::uint8_t>( left_stamps[ li ] ) ) );
        ++li;
      }
      else
      {
        expectPushOk(
            sync,
            makeEvent( CameraId::kRight, right_stamps[ ri ],
                       static_cast<std::uint8_t>( 100 + right_stamps[ ri ] ) ) );
        ++ri;
      }
    }
    sync.flush();

    std::set<std::int64_t> emitted;
    while ( const auto frame = sync.tryPop() )
    {
      const std::int64_t ns = frame->timestamp.nanoseconds();
      emitted.insert( ns );
      EXPECT_EQ( pixelTag( frame->left ), static_cast<std::uint8_t>( ns ) );
      EXPECT_EQ( pixelTag( frame->right ),
                 static_cast<std::uint8_t>( 100 + ns ) );
    }

    EXPECT_EQ( emitted, ( std::set<std::int64_t>{ 10, 20, 40 } ) );
    EXPECT_EQ( sync.diagnostics().emitted_stereo, 3U );
    EXPECT_EQ( sync.diagnostics().dropped_left, 0U );
    EXPECT_EQ( sync.diagnostics().dropped_right, 4U );
  }

  TEST( StereoPairSynchronizerTest, InterleavedPushOrdersSameEmitSet )
  {
    const auto run = []( const std::string& order ) {
      StereoPairSynchronizer    sync;
      std::vector<std::int64_t> left_ns{ 1, 2 };
      std::vector<std::int64_t> right_ns{ 1, 2 };
      std::size_t               li = 0;
      std::size_t               ri = 0;
      for ( const char ch : order )
      {
        if ( ch == 'L' )
        {
          expectPushOk( sync, makeEvent( CameraId::kLeft, left_ns[ li ],
                                         static_cast<std::uint8_t>( li ) ) );
          ++li;
        }
        else
        {
          expectPushOk( sync, makeEvent( CameraId::kRight, right_ns[ ri ],
                                         static_cast<std::uint8_t>( 50 + ri ) ) );
          ++ri;
        }
      }
      sync.flush();
      std::set<std::int64_t> stamps;
      while ( const auto frame = sync.tryPop() )
      {
        stamps.insert( frame->timestamp.nanoseconds() );
      }
      return stamps;
    };

    const auto expected = std::set<std::int64_t>{ 1, 2 };
    EXPECT_EQ( run( "LLRR" ), expected );
    EXPECT_EQ( run( "RRLL" ), expected );
    EXPECT_EQ( run( "LRLR" ), expected );
  }

  TEST( StereoPairSynchronizerTest, ExactTolRejectsOneNanosecondSkew )
  {
    StereoPairSynchronizer sync;
    expectPushOk( sync, makeEvent( CameraId::kLeft, 100, 1 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 101, 2 ) );
    EXPECT_FALSE( sync.tryPop().has_value() );
    EXPECT_EQ( sync.diagnostics().dropped_left, 1U );
    EXPECT_EQ( sync.diagnostics().dropped_right, 0U );
    EXPECT_EQ( sync.diagnostics().emitted_stereo, 0U );

    // Remaining right waits until flush.
    sync.flush();
    EXPECT_EQ( sync.diagnostics().dropped_right, 1U );
  }

  TEST( StereoPairSynchronizerTest, SoftTolPairsWithinWindowUsesLeftStamp )
  {
    StereoPairSynchronizerOptions options;
    options.tol_ns = 3'000'000;  // 3 ms
    StereoPairSynchronizer sync{ options };

    expectPushOk( sync, makeEvent( CameraId::kLeft, 1'000'000, 1 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 3'500'000, 2 ) );

    const auto frame = sync.tryPop();
    ASSERT_TRUE( frame.has_value() );
    EXPECT_EQ( frame->timestamp.nanoseconds(), 1'000'000 );
    EXPECT_EQ( pixelTag( frame->left ), 1 );
    EXPECT_EQ( pixelTag( frame->right ), 2 );
    EXPECT_EQ( sync.diagnostics().emitted_stereo, 1U );
    EXPECT_EQ( sync.diagnostics().dropped_left, 0U );
    EXPECT_EQ( sync.diagnostics().dropped_right, 0U );
  }

  TEST( StereoPairSynchronizerTest, PerCameraOutOfOrderIsSticky )
  {
    StereoPairSynchronizer sync;
    expectPushOk( sync, makeEvent( CameraId::kLeft, 10, 1 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 10, 2 ) );
    ASSERT_TRUE( sync.tryPop().has_value() );

    EXPECT_EQ( sync.pushImage( makeEvent( CameraId::kLeft, 9, 3 ) ),
               PushStatus::kOutOfOrder );
    EXPECT_EQ( sync.stickyError(), PushStatus::kOutOfOrder );
    EXPECT_EQ( sync.pushImage( makeEvent( CameraId::kLeft, 11, 4 ) ),
               PushStatus::kOutOfOrder );
    EXPECT_EQ( sync.diagnostics().pushed_left, 1U );
  }

  TEST( StereoPairSynchronizerTest, PerCameraDuplicateIsSticky )
  {
    StereoPairSynchronizer sync;
    expectPushOk( sync, makeEvent( CameraId::kRight, 5, 1 ) );
    EXPECT_EQ( sync.pushImage( makeEvent( CameraId::kRight, 5, 2 ) ),
               PushStatus::kDuplicate );
    EXPECT_EQ( sync.stickyError(), PushStatus::kDuplicate );
    EXPECT_EQ( sync.pushImage( makeEvent( CameraId::kLeft, 5, 3 ) ),
               PushStatus::kDuplicate );
  }

  TEST( StereoPairSynchronizerTest, StickyStillAllowsTryPopOfPriorPairs )
  {
    StereoPairSynchronizer sync;
    expectPushOk( sync, makeEvent( CameraId::kLeft, 1, 1 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 1, 2 ) );
    EXPECT_EQ( sync.pushImage( makeEvent( CameraId::kLeft, 1, 3 ) ),
               PushStatus::kDuplicate );

    const auto frame = sync.tryPop();
    ASSERT_TRUE( frame.has_value() );
    EXPECT_EQ( frame->timestamp.nanoseconds(), 1 );
    EXPECT_EQ( sync.diagnostics().emitted_stereo, 1U );
  }

  TEST( StereoPairSynchronizerTest, BoundedQueueDropsOldestOnOverflow )
  {
    StereoPairSynchronizerOptions options;
    options.max_queue = 2;
    StereoPairSynchronizer sync{ options };

    expectPushOk( sync, makeEvent( CameraId::kLeft, 1, 1 ) );
    expectPushOk( sync, makeEvent( CameraId::kLeft, 2, 2 ) );
    expectPushOk( sync, makeEvent( CameraId::kLeft, 3, 3 ) );

    const auto& d = sync.diagnostics();
    EXPECT_EQ( d.pushed_left, 3U );
    EXPECT_EQ( d.dropped_left_overflow, 1U );
    EXPECT_EQ( d.max_left_queue, 2U );
    EXPECT_EQ( d.emitted_stereo, 0U );
  }

  TEST( StereoPairSynchronizerTest, FlushCountsAllRemainingNotJustFront )
  {
    StereoPairSynchronizer sync;
    expectPushOk( sync, makeEvent( CameraId::kLeft, 1, 1 ) );
    expectPushOk( sync, makeEvent( CameraId::kLeft, 2, 2 ) );
    expectPushOk( sync, makeEvent( CameraId::kLeft, 3, 3 ) );
    expectPushOk( sync, makeEvent( CameraId::kRight, 10, 10 ) );
    // Left 1,2,3 all earlier than right 10 → drain drops all three left.
    EXPECT_EQ( sync.diagnostics().dropped_left, 3U );
    EXPECT_EQ( sync.diagnostics().dropped_right, 0U );

    sync.flush();
    EXPECT_EQ( sync.diagnostics().dropped_right, 1U );
  }

  TEST( StereoPairSynchronizerTest, RejectsInvalidOptions )
  {
    StereoPairSynchronizerOptions negative_tol;
    negative_tol.tol_ns = -1;
    EXPECT_THROW( StereoPairSynchronizer{ negative_tol },
                  std::invalid_argument );

    StereoPairSynchronizerOptions zero_queue;
    zero_queue.max_queue = 0;
    EXPECT_THROW( StereoPairSynchronizer{ zero_queue },
                  std::invalid_argument );
  }

  // ---- M4.1: pushImu / StereoImuPacket 矩阵 ----

  TEST( StereoPairSynchronizerTest, FirstPacketIsZeroSegmentAndExpiredImuDropped )
  {
    StereoPairSynchronizer sync;
    for ( std::int64_t ns = 1000; ns <= 1004; ++ns )
    {
      expectImuPushOk( sync, makeImu( ns, 1.0 ) );
    }
    expectImuPushOk( sync, makeImu( 1005, 2.0 ) );  // 恰在首帧
    expectImuPushOk( sync, makeImu( 2005, 3.0 ) );
    pushPair( sync, 1005, 1 );
    pushPair( sync, 2005, 2 );

    const auto first = sync.tryPopPacket();
    ASSERT_TRUE( first.has_value() );
    EXPECT_EQ( first->t_prev.nanoseconds(), 1005 );  // 零段
    EXPECT_TRUE( first->samples.empty() );
    EXPECT_FALSE( first->imu_gap );

    const auto second = sync.tryPopPacket();
    ASSERT_TRUE( second.has_value() );
    EXPECT_EQ( second->t_prev.nanoseconds(), 1005 );
    EXPECT_EQ( second->frame.timestamp.nanoseconds(), 2005 );
    ASSERT_EQ( second->samples.size(), 2U );
    EXPECT_EQ( second->samples[ 0 ].timestamp.nanoseconds(), 1005 );
    EXPECT_EQ( second->samples[ 1 ].timestamp.nanoseconds(), 2005 );
    EXPECT_EQ( segmentDt( *second ), 1000 );
    EXPECT_FALSE( second->imu_gap );

    EXPECT_EQ( sync.diagnostics().pushed_imu, 7U );
    EXPECT_EQ( sync.diagnostics().dropped_imu, 5U );  // 1000..1004 越界
    EXPECT_EQ( sync.diagnostics().imu_gap_count, 0U );
  }

  TEST( StereoPairSynchronizerTest, ImuSamplesExactlyOnBoundariesKeepSumDtExact )
  {
    StereoPairSynchronizer sync;
    expectImuPushOk( sync, makeImu( 1000, 1.0 ) );
    expectImuPushOk( sync, makeImu( 1500, 2.0 ) );
    expectImuPushOk( sync, makeImu( 2000, 3.0 ) );
    expectImuPushOk( sync, makeImu( 3000, 4.0 ) );
    pushPair( sync, 1000, 1 );
    pushPair( sync, 2000, 2 );
    pushPair( sync, 3000, 3 );

    (void) sync.tryPopPacket();  // 零段

    const auto mid = sync.tryPopPacket();
    ASSERT_TRUE( mid.has_value() );
    ASSERT_EQ( mid->samples.size(), 3U );
    EXPECT_EQ( mid->samples[ 0 ].timestamp.nanoseconds(), 1000 );
    EXPECT_EQ( mid->samples[ 1 ].timestamp.nanoseconds(), 1500 );
    EXPECT_EQ( mid->samples[ 2 ].timestamp.nanoseconds(), 2000 );
    EXPECT_DOUBLE_EQ( mid->samples[ 0 ].accel_mps2[ 0 ], 1.0 );
    EXPECT_DOUBLE_EQ( mid->samples[ 2 ].accel_mps2[ 0 ], 3.0 );
    EXPECT_EQ( segmentDt( *mid ), 1000 );
    EXPECT_FALSE( mid->imu_gap );

    const auto last = sync.tryPopPacket();
    ASSERT_TRUE( last.has_value() );
    ASSERT_EQ( last->samples.size(), 2U );
    // 左端 = 上段右端原样本(共享边界),无插值。
    EXPECT_EQ( last->samples[ 0 ].timestamp.nanoseconds(), 2000 );
    EXPECT_DOUBLE_EQ( last->samples[ 0 ].accel_mps2[ 0 ], 3.0 );
    EXPECT_EQ( last->samples[ 1 ].timestamp.nanoseconds(), 3000 );
    EXPECT_EQ( segmentDt( *last ), 1000 );
    EXPECT_FALSE( last->imu_gap );
  }

  TEST( StereoPairSynchronizerTest, ImuMissingMiddleSamplesStillExactSegment )
  {
    StereoPairSynchronizer sync;
    expectImuPushOk( sync, makeImu( 1000, 1.0 ) );
    expectImuPushOk( sync, makeImu( 2000, 3.0 ) );
    expectImuPushOk( sync, makeImu( 3000, 5.0 ) );
    pushPair( sync, 1000, 1 );
    pushPair( sync, 2000, 2 );
    pushPair( sync, 3000, 3 );

    (void) sync.tryPopPacket();
    const auto mid = sync.tryPopPacket();
    ASSERT_TRUE( mid.has_value() );
    ASSERT_EQ( mid->samples.size(), 2U );  // 无中间样本,但边界成立
    EXPECT_EQ( segmentDt( *mid ), 1000 );
    EXPECT_FALSE( mid->imu_gap );

    const auto last = sync.tryPopPacket();
    ASSERT_TRUE( last.has_value() );
    EXPECT_EQ( segmentDt( *last ), 1000 );
    EXPECT_FALSE( last->imu_gap );
  }

  TEST( StereoPairSynchronizerTest, InterpolatedRightBoundaryChainsToNextSegment )
  {
    StereoPairSynchronizer sync;
    expectImuPushOk( sync, makeImu( 1000, 1.0 ) );
    expectImuPushOk( sync, makeImu( 1500, 2.0 ) );
    expectImuPushOk( sync, makeImu( 2500, 4.0 ) );
    expectImuPushOk( sync, makeImu( 3000, 5.0 ) );
    pushPair( sync, 1000, 1 );
    pushPair( sync, 2000, 2 );
    pushPair( sync, 3000, 3 );

    (void) sync.tryPopPacket();  // 零段

    const auto mid = sync.tryPopPacket();
    ASSERT_TRUE( mid.has_value() );
    ASSERT_EQ( mid->samples.size(), 3U );
    // 右端为 (1500, 2500) 在 t=2000 的插值: ax = 2.0 + 0.5*(4.0-2.0) = 3.0
    EXPECT_EQ( mid->samples[ 2 ].timestamp.nanoseconds(), 2000 );
    EXPECT_DOUBLE_EQ( mid->samples[ 2 ].accel_mps2[ 0 ], 3.0 );
    EXPECT_DOUBLE_EQ( mid->samples[ 2 ].gyro_radps[ 0 ], 1.5 );
    EXPECT_EQ( segmentDt( *mid ), 1000 );
    EXPECT_FALSE( mid->imu_gap );

    // 下段左端复用插值样本(链式共享边界)。
    const auto last = sync.tryPopPacket();
    ASSERT_TRUE( last.has_value() );
    ASSERT_EQ( last->samples.size(), 3U );
    EXPECT_EQ( last->samples[ 0 ].timestamp.nanoseconds(), 2000 );
    EXPECT_DOUBLE_EQ( last->samples[ 0 ].accel_mps2[ 0 ], 3.0 );
    EXPECT_EQ( last->samples[ 1 ].timestamp.nanoseconds(), 2500 );
    EXPECT_EQ( last->samples[ 2 ].timestamp.nanoseconds(), 3000 );
    EXPECT_EQ( segmentDt( *last ), 1000 );
    EXPECT_FALSE( last->imu_gap );
  }

  TEST( StereoPairSynchronizerTest, ImuDuplicateIsStickyOnImuPath )
  {
    StereoPairSynchronizer sync;
    expectImuPushOk( sync, makeImu( 1000, 1.0 ) );
    EXPECT_EQ( sync.pushImu( makeImu( 1000, 2.0 ) ),
               PushStatus::kDuplicate );
    EXPECT_EQ( sync.imuStickyError(), PushStatus::kDuplicate );
    EXPECT_FALSE( sync.stickyError().has_value() );  // 图像路径不受影响
    EXPECT_EQ( sync.diagnostics().pushed_imu, 1U );
  }

  TEST( StereoPairSynchronizerTest, ImuOutOfOrderIsStickyOnImuPath )
  {
    StereoPairSynchronizer sync;
    expectImuPushOk( sync, makeImu( 1000, 1.0 ) );
    EXPECT_EQ( sync.pushImu( makeImu( 999, 2.0 ) ),
               PushStatus::kOutOfOrder );
    EXPECT_EQ( sync.imuStickyError(), PushStatus::kOutOfOrder );
    EXPECT_EQ( sync.pushImu( makeImu( 1001, 3.0 ) ),
               PushStatus::kOutOfOrder );
  }

  TEST( StereoPairSynchronizerTest, ImuNonFiniteIsInvalidValue )
  {
    StereoPairSynchronizer sync;
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    ImuMeasurement bad    = makeImu( 1000, 1.0 );
    bad.accel_mps2[ 1 ]   = nan;
    EXPECT_EQ( sync.pushImu( bad ), PushStatus::kInvalidValue );
    EXPECT_EQ( sync.imuStickyError(), PushStatus::kInvalidValue );
    EXPECT_EQ( sync.diagnostics().pushed_imu, 0U );
  }

  TEST( StereoPairSynchronizerTest, ImuStickyIsIndependentOfImageSticky )
  {
    StereoPairSynchronizer sync;
    // IMU 先置 sticky,图像路径照常。
    expectImuPushOk( sync, makeImu( 1000, 1.0 ) );
    EXPECT_EQ( sync.pushImu( makeImu( 999, 2.0 ) ),
               PushStatus::kOutOfOrder );
    pushPair( sync, 1000, 1 );
    const auto frame = sync.tryPop();
    ASSERT_TRUE( frame.has_value() );
    EXPECT_EQ( frame->timestamp.nanoseconds(), 1000 );

    // 图像置 sticky,IMU 路径照常。
    StereoPairSynchronizer other;
    pushPair( other, 1000, 1 );
    EXPECT_EQ( other.pushImage( makeEvent( CameraId::kLeft, 1000, 2 ) ),
               PushStatus::kDuplicate );
    expectImuPushOk( other, makeImu( 1000, 1.0 ) );
    EXPECT_TRUE( other.tryPop().has_value() );
    EXPECT_FALSE( other.imuStickyError().has_value() );
  }

  TEST( StereoPairSynchronizerTest, ImuGapBeyondThresholdMarksPacket )
  {
    // 帧间隔 10 ms / 240 ms,阈值 200 ms:第二段不 gap,第三段 gap。
    StereoPairSynchronizerOptions options;
    options.imu_gap_ns = 200'000'000;
    StereoPairSynchronizer sync{ options };
    expectImuPushOk( sync, makeImu( 100'000'000, 1.0 ) );
    expectImuPushOk( sync, makeImu( 105'000'000, 2.0 ) );
    expectImuPushOk( sync, makeImu( 110'000'000, 3.0 ) );
    expectImuPushOk( sync, makeImu( 130'000'000, 4.0 ) );
    expectImuPushOk( sync, makeImu( 350'000'000, 5.0 ) );
    pushPair( sync, 100'000'000, 1 );
    pushPair( sync, 110'000'000, 2 );  // 10 ms < 200 ms
    pushPair( sync, 350'000'000, 3 );  // 240 ms > 200 ms

    (void) sync.tryPopPacket();  // 零段

    const auto ok = sync.tryPopPacket();
    ASSERT_TRUE( ok.has_value() );
    ASSERT_EQ( ok->samples.size(), 3U );
    EXPECT_EQ( segmentDt( *ok ), 10'000'000 );
    EXPECT_FALSE( ok->imu_gap );

    const auto gapped = sync.tryPopPacket();
    ASSERT_TRUE( gapped.has_value() );
    EXPECT_TRUE( gapped->imu_gap );
    EXPECT_EQ( sync.diagnostics().imu_gap_count, 1U );
  }

  TEST( StereoPairSynchronizerTest, ImuGapWhenBoundariesUnconstructible )
  {
    StereoPairSynchronizer sync;
    expectImuPushOk( sync, makeImu( 2500, 4.0 ) );  // 只落在第二段中间
    expectImuPushOk( sync, makeImu( 3000, 5.0 ) );
    pushPair( sync, 1000, 1 );
    pushPair( sync, 2000, 2 );
    pushPair( sync, 3000, 3 );

    (void) sync.tryPopPacket();  // 零段

    const auto second = sync.tryPopPacket();
    ASSERT_TRUE( second.has_value() );
    EXPECT_TRUE( second->samples.empty() );  // 左端右端均无法构造
    EXPECT_TRUE( second->imu_gap );

    // 第三段从 2500 恢复: 左端缺失(gap),但中间 + 右端成立。
    const auto third = sync.tryPopPacket();
    ASSERT_TRUE( third.has_value() );
    ASSERT_EQ( third->samples.size(), 2U );
    EXPECT_EQ( third->samples[ 0 ].timestamp.nanoseconds(), 2500 );
    EXPECT_EQ( third->samples[ 1 ].timestamp.nanoseconds(), 3000 );
    EXPECT_TRUE( third->imu_gap );
    EXPECT_EQ( sync.diagnostics().imu_gap_count, 2U );
  }

  TEST( StereoPairSynchronizerTest, ImuOverflowDropsOldestCounted )
  {
    StereoPairSynchronizerOptions options;
    options.max_imu_queue = 2;
    StereoPairSynchronizer sync{ options };
    expectImuPushOk( sync, makeImu( 1000, 1.0 ) );
    expectImuPushOk( sync, makeImu( 1001, 2.0 ) );
    expectImuPushOk( sync, makeImu( 1002, 3.0 ) );

    const auto& d = sync.diagnostics();
    EXPECT_EQ( d.pushed_imu, 3U );
    EXPECT_EQ( d.dropped_imu_overflow, 1U );
    EXPECT_EQ( d.max_imu_queue, 2U );

    // 被 drop 的最老样本(1000)已不在段内: 左端构造失败 → gap。
    pushPair( sync, 1001, 1 );
    pushPair( sync, 1002, 2 );
    (void) sync.tryPopPacket();  // 零段
    const auto packet = sync.tryPopPacket();
    ASSERT_TRUE( packet.has_value() );
    EXPECT_EQ( packet->samples[ 0 ].timestamp.nanoseconds(), 1001 );
    EXPECT_EQ( packet->samples[ 1 ].timestamp.nanoseconds(), 1002 );
  }

  TEST( StereoPairSynchronizerTest, FlushCountsRemainingImuAsDropped )
  {
    StereoPairSynchronizer sync;
    expectImuPushOk( sync, makeImu( 1000, 1.0 ) );
    expectImuPushOk( sync, makeImu( 2000, 2.0 ) );
    sync.flush();
    EXPECT_EQ( sync.diagnostics().dropped_imu, 2U );
  }

}  // namespace
