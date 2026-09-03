#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <unordered_map>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/estimator/types.hpp"
#include "phad/frontend/stereo_tracks.hpp"

namespace phad::apps::internal
{

  struct KeyframeSelectorState
  {
    std::unordered_map<common::LandmarkId, Eigen::Vector2d> m_last_kf_pixels;
    common::Timestamp                                       m_last_kf_timestamp{ 0 };
    std::uint32_t                                           m_total_keyframes        = 0;
    Eigen::Matrix3d                                         m_last_accepted_rotation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d                                         m_last_kf_rotation       = Eigen::Matrix3d::Identity();
  };

  [[nodiscard]] bool isKeyframe(
      const frontend::FrameTracks&              tracks,
      common::Timestamp                         current_ts,
      const KeyframeSelectorState&              state,
      const camera::RectifiedStereoCalibration& calibration );

  void applyAcceptedCameraState(
      bool                                      is_keyframe,
      const frontend::FrameTracks&              tracks,
      common::Timestamp                         timestamp,
      const estimator::VioUpdateResult&         update,
      const camera::RectifiedStereoCalibration& calibration,
      KeyframeSelectorState&                    state );

}  // namespace phad::apps::internal
