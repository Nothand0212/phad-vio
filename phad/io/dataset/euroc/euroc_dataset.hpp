#pragma once

#include <filesystem>

#include "phad/io/dataset/dataset_error.hpp"
#include "phad/io/dataset/stereo_imu_dataset.hpp"

namespace phad::io::dataset::euroc
{

  [[nodiscard]] DatasetResult<StereoImuDataset> open(
      const std::filesystem::path& sequence_root );

}  // namespace phad::io::dataset::euroc
