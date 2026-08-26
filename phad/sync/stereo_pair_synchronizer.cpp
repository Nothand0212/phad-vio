#include "phad/sync/stereo_pair_synchronizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

namespace phad::sync
{
  namespace
  {

    [[nodiscard]] bool isFinite( const sensor::ImuMeasurement& measurement )
    {
      for ( std::size_t axis = 0; axis < 3U; ++axis )
      {
        if ( !std::isfinite( measurement.accel_mps2[ axis ] ) ||
             !std::isfinite( measurement.gyro_radps[ axis ] ) )
        {
          return false;
        }
      }
      return true;
    }

    [[nodiscard]] bool subtractionOverflows( std::int64_t next,
                                             std::int64_t current )
    {
      return current < 0 &&
             next > std::numeric_limits<std::int64_t>::max() + current;
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
    if ( m_options.imu_continuity_limit_ns <= 0 )
    {
      throw std::invalid_argument(
          "StereoPairSynchronizerOptions.imu_continuity_limit_ns must be > 0" );
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

    drainStereo();
    if ( m_packet_mode.value_or( false ) )
    {
      drainPackets( false );
    }
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
      if ( subtractionOverflows( measurement.timestamp.nanoseconds(),
                                 m_last_imu->nanoseconds() ) )
      {
        m_imu_sticky = PushStatus::kInvalidStamp;
        return *m_imu_sticky;
      }
    }
    m_last_imu = measurement.timestamp;

    ++m_diag.pushed_imu;
    m_imu.push_back( std::move( measurement ) );
    while ( m_imu.size() > m_options.max_imu_queue )
    {
      m_imu.pop_front();
      ++m_diag.dropped_imu_overflow;
    }
    m_diag.max_imu_queue = std::max( m_diag.max_imu_queue, m_imu.size() );
    if ( m_packet_mode.value_or( false ) )
    {
      drainPackets( false );
    }
    return PushStatus::kOk;
  }

  std::optional<sensor::StereoFrame> StereoPairSynchronizer::tryPop()
  {
    if ( m_packet_mode.value_or( false ) )
    {
      if ( m_ready.empty() )
      {
        return std::nullopt;
      }
      sensor::StereoFrame frame = std::move( m_ready.front().m_frame );
      m_ready.pop_front();
      return frame;
    }
    m_packet_mode = false;
    if ( m_paired.empty() )
    {
      return std::nullopt;
    }
    sensor::StereoFrame frame = std::move( m_paired.front() );
    m_paired.pop_front();
    return frame;
  }

  std::optional<sensor::StereoImuPacket>
  StereoPairSynchronizer::tryPopPacket()
  {
    if ( m_packet_mode.has_value() && !*m_packet_mode )
    {
      return std::nullopt;
    }
    m_packet_mode = true;
    drainPackets( false );
    if ( m_ready.empty() )
    {
      return std::nullopt;
    }
    sensor::StereoImuPacket&               front = m_ready.front();
    std::optional<sensor::StereoImuPacket> packet;
    if ( auto* raw = std::get_if<sensor::RawImuInterval>( &front.m_imu ) )
    {
      packet.emplace( sensor::StereoImuPacket{
          .m_frame = std::move( front.m_frame ),
          .m_imu   = sensor::RawImuInterval{
                .m_t_begin = raw->m_t_begin,
                .m_t_end   = raw->m_t_end,
                .m_samples = std::move( raw->m_samples ) } } );
    }
    else
    {
      packet.emplace( sensor::StereoImuPacket{
          .m_frame = std::move( front.m_frame ),
          .m_imu   = std::get<sensor::MeasurementDiscontinuity>(
              front.m_imu ) } );
    }
    m_ready.pop_front();
    return packet;
  }

  void StereoPairSynchronizer::flush()
  {
    drainStereo();
    m_diag.dropped_left += static_cast<std::uint64_t>( m_left.size() );
    m_diag.dropped_right += static_cast<std::uint64_t>( m_right.size() );
    m_left.clear();
    m_right.clear();

    if ( m_packet_mode.value_or( false ) )
    {
      drainPackets( true );
      m_paired.clear();
    }
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

  void StereoPairSynchronizer::drainStereo()
  {
    while ( !m_left.empty() && !m_right.empty() )
    {
      const std::int64_t t_l = m_left.front().timestamp.nanoseconds();
      const std::int64_t t_r = m_right.front().timestamp.nanoseconds();
      if ( subtractionOverflows( std::max( t_l, t_r ),
                                 std::min( t_l, t_r ) ) )
      {
        m_sticky = PushStatus::kInvalidStamp;
        return;
      }
      const std::int64_t dt     = t_l - t_r;
      const std::int64_t abs_dt = dt < 0 ? -dt : dt;

      if ( abs_dt <= m_options.tol_ns )
      {
        sensor::ImageFrameEvent left  = std::move( m_left.front() );
        sensor::ImageFrameEvent right = std::move( m_right.front() );
        m_left.pop_front();
        m_right.pop_front();
        m_paired.push_back( sensor::StereoFrame{
            left.timestamp, std::move( left.image ), std::move( right.image ) } );
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

  void StereoPairSynchronizer::drainPackets( bool source_exhausted )
  {
    while ( !m_paired.empty() )
    {
      if ( !m_last_emitted_left.has_value() && !m_imu.empty() &&
           m_imu.front().timestamp >= m_paired.front().timestamp )
      {
        // IMU timestamps are strictly increasing, so no future sample can
        // provide positive-duration support before this leading stereo pair.
        m_paired.pop_front();
        continue;
      }
      std::optional<sensor::StereoImuPacket> packet =
          makePacket( m_paired.front(), source_exhausted );
      if ( !packet.has_value() )
      {
        return;
      }
      m_ready.push_back( std::move( *packet ) );
      m_paired.pop_front();
    }
  }

  std::optional<sensor::StereoImuPacket>
  StereoPairSynchronizer::makePacket( sensor::StereoFrame& frame,
                                      bool                 source_exhausted )
  {
    const common::Timestamp          t_end   = frame.timestamp;
    std::optional<common::Timestamp> t_begin = m_last_emitted_left;
    if ( !t_begin.has_value() )
    {
      if ( m_imu.empty() || m_imu.front().timestamp >= t_end )
      {
        return std::nullopt;
      }
      t_begin = m_imu.front().timestamp;
    }
    if ( *t_begin >= t_end )
    {
      return std::nullopt;
    }

    const auto lower = [ this ]( common::Timestamp timestamp ) {
      return std::lower_bound(
          m_imu.begin(), m_imu.end(), timestamp,
          []( const sensor::ImuMeasurement& sample,
              common::Timestamp             target ) {
            return sample.timestamp < target;
          } );
    };
    const auto end_right = lower( t_end );
    if ( end_right == m_imu.end() && !source_exhausted )
    {
      return std::nullopt;
    }

    const auto begin_right = lower( *t_begin );
    const bool begin_exact =
        begin_right != m_imu.end() && begin_right->timestamp == *t_begin;
    const bool end_exact =
        end_right != m_imu.end() && end_right->timestamp == t_end;
    const bool begin_bracketed =
        begin_exact || ( begin_right != m_imu.begin() &&
                         begin_right != m_imu.end() );
    const bool end_bracketed =
        end_exact || ( end_right != m_imu.begin() &&
                       end_right != m_imu.end() );
    bool continuous = begin_bracketed && end_bracketed;

    if ( continuous )
    {
      for ( auto it = m_imu.begin(); std::next( it ) != m_imu.end(); ++it )
      {
        const auto next = std::next( it );
        if ( it->timestamp < t_end && next->timestamp > *t_begin )
        {
          const std::int64_t delta_ns =
              next->timestamp.nanoseconds() - it->timestamp.nanoseconds();
          if ( delta_ns > m_options.imu_continuity_limit_ns )
          {
            continuous = false;
            break;
          }
        }
      }
    }

    if ( continuous )
    {
      const auto                          first = begin_exact ? begin_right : std::prev( begin_right );
      const auto                          last  = end_right;
      std::vector<sensor::ImuMeasurement> samples;
      samples.reserve(
          static_cast<std::size_t>( std::distance( first, last ) ) + 1U );
      samples.insert( samples.end(), first, std::next( last ) );
      m_last_emitted_left = t_end;
      pruneImu( t_end );
      return sensor::StereoImuPacket{
          .m_frame = std::move( frame ),
          .m_imu   = sensor::RawImuInterval{
                .m_t_begin = *t_begin,
                .m_t_end   = t_end,
                .m_samples = std::move( samples ) } };
    }

    ++m_diag.imu_discontinuity_count;
    m_last_emitted_left = t_end;
    pruneImu( t_end );
    return sensor::StereoImuPacket{
        .m_frame = std::move( frame ),
        .m_imu   = sensor::MeasurementDiscontinuity{
              .m_t_begin = *t_begin,
              .m_t_end   = t_end } };
  }

  void StereoPairSynchronizer::pruneImu( common::Timestamp t_end )
  {
    while ( m_imu.size() > 1U && m_imu[ 1U ].timestamp <= t_end )
    {
      m_imu.pop_front();
      ++m_diag.dropped_imu;
    }
  }

  void StereoPairSynchronizer::updateMaxQueue( sensor::CameraId camera )
  {
    if ( camera == sensor::CameraId::kLeft )
    {
      m_diag.max_left_queue = std::max( m_diag.max_left_queue, m_left.size() );
    }
    else
    {
      m_diag.max_right_queue = std::max( m_diag.max_right_queue, m_right.size() );
    }
  }

}  // namespace phad::sync
