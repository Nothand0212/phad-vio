#pragma once

#include <cstdint>
#include <filesystem>
#include <string_view>

#include "phad/common/trajectory.hpp"
#include "phad/io/dataset/dataset_error.hpp"
#include "phad/io/dataset/stereo_imu_dataset.hpp"

namespace phad::apps::internal
{

  enum class VoBenchDatasetFormat : std::uint8_t
  {
    kEuroc = 0,
    kTumVi = 1,
  };

  [[nodiscard]] io::dataset::DatasetResult<io::dataset::StereoImuDataset>
  openVoBenchDataset( VoBenchDatasetFormat         format,
                      const std::filesystem::path& sequence_root );

  [[nodiscard]] io::dataset::DatasetResult<common::Trajectory>
  openVoBenchGroundtruth( VoBenchDatasetFormat         format,
                          const std::filesystem::path& sequence_root );

  [[nodiscard]] std::string_view voBenchDatasetConfigToken(
      VoBenchDatasetFormat format ) noexcept;

}  // namespace phad::apps::internal
