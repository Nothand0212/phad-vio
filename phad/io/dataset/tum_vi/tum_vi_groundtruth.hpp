#pragma once

#include <filesystem>

#include "phad/common/trajectory.hpp"
#include "phad/io/dataset/dataset_error.hpp"

namespace phad::io::dataset::tum_vi
{

  /**
   * @brief Load calibrated TUM-VI body ground truth from dso/gt_imu.csv.
   *
   * The calibrated export expresses the pose of the IMU body in the world
   * frame, so each returned sample is T_W_B.
   */
  [[nodiscard]] DatasetResult<common::Trajectory> openGroundtruth(
      const std::filesystem::path& sequence_root );

}  // namespace phad::io::dataset::tum_vi
