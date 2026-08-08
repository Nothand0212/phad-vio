#include "phad/sync/stereo_pair_synchronizer.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

/**
 * @file stereo_pair_synchronizer.cpp
 * @brief StereoPairSynchronizer 实现。
 */

namespace phad::sync
{

  namespace
  {

    /// 线性插值:t 严格落在 a.t 与 b.t 之间(a.t < t < b.t)。
    [[nodiscard]] sensor::ImuMeasurement interpolateImu(
        const sensor::ImuMeasurement& a, const sensor::ImuMeasurement& b,
        common::Timestamp t )
    {
      const double dt    = static_cast<double>( b.timestamp.nanoseconds() -
                                                a.timestamp.nanoseconds() );
      const double alpha = static_cast<double>( t.nanoseconds() -
                                                a.timestamp.nanoseconds() ) /
                           dt;
      sensor::ImuMeasurement out;
      out.timestamp = t;
      for ( std::size_t i = 0; i < 3; ++i )
      {
        out.accel_mps2[ i ] =
            a.accel_mps2[ i ] + alpha * ( b.accel_mps2[ i ] - a.accel_mps2[ i ] );
        out.gyro_radps[ i ] =
            a.gyro_radps[ i ] + alpha * ( b.gyro_radps[ i ] - a.gyro_radps[ i ] );
      }
      return out;
    }

    [[nodiscard]] bool isFinite( const sensor::ImuMeasurement& m )
    {
      for ( std::size_t i = 0; i < 3; ++i )
      {
        if ( !std::isfinite( m.accel_mps2[ i ] ) ||
             !std::isfinite( m.gyro_radps[ i ] ) )
        {
          return false;
        }
      }
      return true;
    }

  }  // namespace

  StereoPairSynchronizer::StereoPairSynchronizer(
      StereoPairSynchronizerOptions options )
      : m_options( options )
  {
    if ( m_options.tol_ns < 0 )
    {
      throw std::invalid_argument(
          "StereoPairSynchronizerOptions.tol_ns must be >= 0" );
    }
    if ( m_options.max_queue == 0U )
    {
      throw std::invalid_argument(
          "StereoPairSynchronizerOptions.max_queue must be >= 1" );
    }
    if ( m_options.max_imu_queue == 0U )
    {
      throw std::invalid_argument(
          "StereoPairSynchronizerOptions.max_imu_queue must be >= 1" );
    }
    if ( m_options.imu_gap_ns < 0 )
    {
      throw std::invalid_argument(
          "StereoPairSynchronizerOptions.imu_gap_ns must be >= 0" );
    }
  }

  PushStatus StereoPairSynchronizer::pushImage( sensor::ImageFrameEvent event )
  {
    if ( m_sticky.has_value() )
    {
      return *m_sticky;
    }

    if ( event.camera != sensor::CameraId::kLeft &&
         event.camera != sensor::CameraId::kRight )
    {
      m_sticky = PushStatus::kInvalidStamp;
      return *m_sticky;
    }

    std::optional<common::Timestamp>& last =
        event.camera == sensor::CameraId::kLeft ? m_last_left : m_last_right;
    if ( last.has_value() )
    {
      if ( event.timestamp < *last )
      {
        m_sticky = PushStatus::kOutOfOrder;
        return *m_sticky;
      }
      if ( event.timestamp == *last )
      {
        m_sticky = PushStatus::kDuplicate;
        return *m_sticky;
      }
    }
    last = event.timestamp;

    if ( event.camera == sensor::CameraId::kLeft )
    {
      ++m_diag.pushed_left;
      m_left.push_back( std::move( event ) );
      while ( m_left.size() > m_options.max_queue )
      {
        m_left.pop_front();
        ++m_diag.dropped_left_overflow;
      }
      updateMaxQueue( sensor::CameraId::kLeft );
    }
    else
    {
      ++m_diag.pushed_right;
      m_right.push_back( std::move( event ) );
      while ( m_right.size() > m_options.max_queue )
      {
        m_right.pop_front();
        ++m_diag.dropped_right_overflow;
      }
      updateMaxQueue( sensor::CameraId::kRight );
    }

    drain();
    return PushStatus::kOk;
  }

  PushStatus StereoPairSynchronizer::pushImu(
      sensor::ImuMeasurement measurement )
  {
    if ( m_imu_sticky.has_value() )
    {
      return *m_imu_sticky;
    }
    if ( !isFinite( measurement ) )
    {
      m_imu_sticky = PushStatus::kInvalidValue;
      return *m_imu_sticky;
    }
    if ( m_last_imu.has_value() )
    {
      if ( measurement.timestamp < *m_last_imu )
      {
        m_imu_sticky = PushStatus::kOutOfOrder;
        return *m_imu_sticky;
      }
      if ( measurement.timestamp == *m_last_imu )
      {
        m_imu_sticky = PushStatus::kDuplicate;
        return *m_imu_sticky;
      }
    }
    m_last_imu = measurement.timestamp;

    ++m_diag.pushed_imu;
    m_imu.push_back( measurement );
    while ( m_imu.size() > m_options.max_imu_queue )
    {
      m_imu.pop_front();
      ++m_diag.dropped_imu_overflow;
    }
    if ( m_imu.size() > m_diag.max_imu_queue )
    {
      m_diag.max_imu_queue = m_imu.size();
    }
    return PushStatus::kOk;
  }

  std::optional<sensor::StereoFrame> StereoPairSynchronizer::tryPop()
  {
    auto packet = tryPopPacket();
    if ( !packet.has_value() )
    {
      return std::nullopt;
    }
    return packet->frame;
  }

  std::optional<sensor::StereoImuPacket> StereoPairSynchronizer::tryPopPacket()
  {
    if ( m_ready.empty() )
    {
      return std::nullopt;
    }
    sensor::StereoImuPacket packet = std::move( m_ready.front() );
    m_ready.pop_front();
    return packet;
  }

  void StereoPairSynchronizer::flush()
  {
    m_diag.dropped_left += static_cast<std::uint64_t>( m_left.size() );
    m_diag.dropped_right += static_cast<std::uint64_t>( m_right.size() );
    m_left.clear();
    m_right.clear();
    m_diag.dropped_imu += static_cast<std::uint64_t>( m_imu.size() );
    m_imu.clear();
  }

  const StereoPairDiagnostics& StereoPairSynchronizer::diagnostics()
      const noexcept
  {
    return m_diag;
  }

  std::optional<PushStatus> StereoPairSynchronizer::stickyError() const noexcept
  {
    return m_sticky;
  }

  std::optional<PushStatus> StereoPairSynchronizer::imuStickyError()
      const noexcept
  {
    return m_imu_sticky;
  }

  void StereoPairSynchronizer::drain()
  {
    while ( !m_left.empty() && !m_right.empty() )
    {
      const std::int64_t t_l    = m_left.front().timestamp.nanoseconds();
      const std::int64_t t_r    = m_right.front().timestamp.nanoseconds();
      const std::int64_t dt     = t_l - t_r;
      const std::int64_t abs_dt = dt < 0 ? -dt : dt;

      if ( abs_dt <= m_options.tol_ns )
      {
        sensor::ImageFrameEvent left  = std::move( m_left.front() );
        sensor::ImageFrameEvent right = std::move( m_right.front() );
        m_left.pop_front();
        m_right.pop_front();
        sensor::StereoFrame frame{ left.timestamp, std::move( left.image ),
                                   std::move( right.image ) };
        m_ready.push_back( makePacket( frame.timestamp, m_last_emitted_left,
                                       std::move( frame ) ) );
        m_last_emitted_left = m_ready.back().frame.timestamp;
        ++m_diag.emitted_stereo;
      }
      else if ( dt < 0 )
      {
        m_left.pop_front();
        ++m_diag.dropped_left;
      }
      else
      {
        m_right.pop_front();
        ++m_diag.dropped_right;
      }
    }
  }

  void StereoPairSynchronizer::dropExpiredImu( common::Timestamp t_prev )
  {
    while ( !m_imu.empty() && m_imu.front().timestamp < t_prev )
    {
      m_imu.pop_front();
      ++m_diag.dropped_imu;
    }
  }

  sensor::StereoImuPacket StereoPairSynchronizer::makePacket(
      common::Timestamp                  t_cur,
      std::optional<common::Timestamp>   t_prev_opt,
      sensor::StereoFrame                frame )
  {
    const common::Timestamp t_cur_stamp = frame.timestamp;
    // 聚合初始化: frame 需 move 构造(Image 无默认构造)。
    sensor::StereoImuPacket packet{ std::move( frame ), {}, t_cur_stamp, false };

    if ( !t_prev_opt.has_value() )
    {
      // 首个 packet: 不构造段;丢弃 < t_cur 的过期样本,恰在 t_cur 的保留为
      // 下一段左端(M4.3 前,早于首帧的 IMU 由消费方另行读取)。
      dropExpiredImu( t_cur );
      if ( !m_imu.empty() && m_imu.front().timestamp == t_cur )
      {
        m_last_boundary = m_imu.front();
      }
      return packet;
    }

    const common::Timestamp t_prev = *t_prev_opt;
    packet.t_prev = t_prev;
    dropExpiredImu( t_prev );

    // 1. 左端: 原样本优先,否则用上一段右端(恰在 t_prev 的插值样本)。
    if ( !m_imu.empty() && m_imu.front().timestamp == t_prev )
    {
      packet.samples.push_back( m_imu.front() );
      m_imu.pop_front();
    }
    else if ( m_last_boundary.has_value() &&
              m_last_boundary->timestamp == t_prev )
    {
      packet.samples.push_back( *m_last_boundary );
    }
    else
    {
      packet.imu_gap = true;  // 无法构造左端
    }

    // 2. 中间 + 右端: 取 < t_cur 的原样本;t_cur 处原样本优先作右端,
    //    否则用最靠近 t_cur 的样本与队列 front 线性插值。
    while ( !m_imu.empty() && m_imu.front().timestamp < t_cur )
    {
      packet.samples.push_back( m_imu.front() );
      m_imu.pop_front();
    }
    if ( !m_imu.empty() && m_imu.front().timestamp == t_cur )
    {
      packet.samples.push_back( m_imu.front() );
      m_imu.pop_front();
      m_last_boundary = packet.samples.back();
    }
    else if ( !m_imu.empty() && !packet.samples.empty() )
    {
      // 队列 front > t_cur,左侧有样本 → 用最靠近 t_cur 的样本插值右端。
      packet.samples.push_back(
          interpolateImu( packet.samples.back(), m_imu.front(), t_cur ) );
      m_last_boundary = packet.samples.back();
    }
    else
    {
      packet.imu_gap = true;  // 无法构造右端(无右侧样本)
    }

    // 3. 大间断判定: 段宽超过阈值或段内无样本。
    if ( t_cur.nanoseconds() - t_prev.nanoseconds() > m_options.imu_gap_ns ||
         packet.samples.empty() )
    {
      packet.imu_gap = true;
    }
    if ( packet.imu_gap )
    {
      ++m_diag.imu_gap_count;
    }
    return packet;
  }

  void StereoPairSynchronizer::updateMaxQueue( sensor::CameraId camera )
  {
    if ( camera == sensor::CameraId::kLeft )
    {
      if ( m_left.size() > m_diag.max_left_queue )
      {
        m_diag.max_left_queue = m_left.size();
      }
    }
    else if ( m_right.size() > m_diag.max_right_queue )
    {
      m_diag.max_right_queue = m_right.size();
    }
  }

}  // namespace phad::sync
