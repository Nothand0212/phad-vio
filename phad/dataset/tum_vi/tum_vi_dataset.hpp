#pragma once

#include <filesystem>

#include "phad/dataset/stereo_imu_dataset.hpp"

namespace phad::dataset::tum_vi
{

  [[nodiscard]] DatasetResult<StereoImuDataset> open(
      const std::filesystem::path& sequence_root );

}  // namespace phad::dataset::tum_vi
