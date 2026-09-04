#include "apps/internal/offline_vo_session_keyframe.hpp"

#include <algorithm>
#include <cmath>

namespace phad::apps::internal
{
  namespace
  {
    constexpr double      kKeyframeParallaxPx   = 30.0;
    constexpr std::size_t kKeyframeMinPnpTracks = 10U;

    void updateKeyframeSnapshot(
        const frontend::FrameTracks& tracks,
        common::Timestamp            timestamp,
        const Eigen::Matrix3d&       camera_rotation,
        KeyframeSelectorState&       state )
    {
      state.m_last_kf_pixels.clear();
      for ( const auto& observation : tracks.observations )
      {
        state.m_last_kf_pixels[ observation.id ] = observation.left_pixel;
      }
      state.m_last_kf_timestamp = timestamp;
      state.m_last_kf_rotation  = camera_rotation;
      ++state.m_total_keyframes;
    }
  }  // namespace

  bool isKeyframe(
      const frontend::FrameTracks&              tracks,
      const common::Timestamp                   current_ts,
      const KeyframeSelectorState&              state,
      const camera::RectifiedStereoCalibration& calibration )
  {
    if ( tracks.observations.empty() )
    {
      return false;
    }
    if ( state.m_total_keyframes < 2U )
    {
      return true;
    }
    if ( tracks.observations.size() < kKeyframeMinPnpTracks )
    {
      return true;
    }

    const std::int64_t dt_ns =
        current_ts.nanoseconds() - state.m_last_kf_timestamp.nanoseconds();
    if ( dt_ns > 500'000'000 )
    {
      return true;
    }

    std::size_t common_count = 0;
    for ( const auto& observation : tracks.observations )
    {
      if ( state.m_last_kf_pixels.count( observation.id ) > 0U )
      {
        ++common_count;
      }
    }
    const double survive_ratio =
        static_cast<double>( common_count ) /
        static_cast<double>(
            std::max( state.m_last_kf_pixels.size(), std::size_t{ 1 } ) );
    if ( survive_ratio < 0.6 )
    {
      return true;
    }

    const Eigen::Matrix3d r_kf_to_cur =
        state.m_last_accepted_rotation.transpose() * state.m_last_kf_rotation;
    const double fx = calibration.fxPixels();
    const double fy = calibration.fyPixels();
    const double cx = calibration.cxPixels();
    const double cy = calibration.cyPixels();

    double      parallax_sum   = 0.0;
    std::size_t parallax_count = 0;
    for ( const auto& observation : tracks.observations )
    {
      const auto previous = state.m_last_kf_pixels.find( observation.id );
      if ( previous == state.m_last_kf_pixels.end() )
      {
        continue;
      }
      const Eigen::Vector2d& last_pixel = previous->second;
      const Eigen::Vector3d  ray_kf( ( last_pixel.x() - cx ) / fx,
                                     ( last_pixel.y() - cy ) / fy, 1.0 );
      const Eigen::Vector3d  ray_cur = r_kf_to_cur * ray_kf;
      if ( ray_cur.z() <= 1e-6 )
      {
        continue;
      }
      const Eigen::Vector2d compensated_pixel(
          ray_cur.x() / ray_cur.z() * fx + cx,
          ray_cur.y() / ray_cur.z() * fy + cy );
      const double dx = observation.left_pixel.x() - compensated_pixel.x();
      const double dy = observation.left_pixel.y() - compensated_pixel.y();
      parallax_sum += std::sqrt( dx * dx + dy * dy );
      ++parallax_count;
    }

    return parallax_count > 0U &&
           parallax_sum / static_cast<double>( parallax_count ) >
               kKeyframeParallaxPx;
  }

  void applyAcceptedCameraState(
      const bool                                is_keyframe,
      const frontend::FrameTracks&              tracks,
      const common::Timestamp                   timestamp,
      const estimator::VioUpdateResult&         update,
      const camera::RectifiedStereoCalibration& calibration,
      KeyframeSelectorState&                    state )
  {
    if ( update.status != estimator::UpdateStatus::kOk ||
         !update.estimate.has_value() )
    {
      return;
    }

    const Eigen::Matrix3d camera_rotation =
        update.estimate->T_W_B.linear() *
        calibration.T_B_left_rectified().rotation();
    if ( is_keyframe )
    {
      updateKeyframeSnapshot( tracks, timestamp, camera_rotation, state );
    }
    state.m_last_accepted_rotation = camera_rotation;
  }

}  // namespace phad::apps::internal
