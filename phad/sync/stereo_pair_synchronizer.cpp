#include "phad/sync/stereo_pair_synchronizer.hpp"

#include <stdexcept>
#include <utility>

/**
 * @file stereo_pair_synchronizer.cpp
 * @brief StereoPairSynchronizer 实现。
 */

namespace phad::sync
{

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

  std::optional<sensor::StereoFrame> StereoPairSynchronizer::tryPop()
  {
    if ( m_ready.empty() )
    {
      return std::nullopt;
    }
    sensor::StereoFrame frame = std::move( m_ready.front() );
    m_ready.pop_front();
    return frame;
  }

  void StereoPairSynchronizer::flush()
  {
    m_diag.dropped_left += static_cast<std::uint64_t>( m_left.size() );
    m_diag.dropped_right += static_cast<std::uint64_t>( m_right.size() );
    m_left.clear();
    m_right.clear();
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
        m_ready.push_back( sensor::StereoFrame{
            left.timestamp, std::move( left.image ),
            std::move( right.image ) } );
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
