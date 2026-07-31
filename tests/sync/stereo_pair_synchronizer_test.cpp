#include "phad/sync/stereo_pair_synchronizer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
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
  using phad::sensor::StereoFrame;
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

}  // namespace
