#pragma once

#ifndef PHAD_REPLAY_DIAGNOSTIC_TESTING
#error "stereo_imu_dataset_reader_test_access.hpp is available only to replay diagnostic tests"
#endif

#include <string>

#include "phad/io/dataset/stereo_imu_dataset.hpp"

namespace phad::io::dataset::internal
{

  class StereoImuDatasetReaderTestAccess
  {
  public:
    [[nodiscard]] static std::string canonicalStateJson(
        const StereoImuDatasetReader& reader );
  };

  [[nodiscard]] inline std::string canonicalStateJson(
      const StereoImuDatasetReader& reader )
  {
    return StereoImuDatasetReaderTestAccess::canonicalStateJson( reader );
  }

}  // namespace phad::io::dataset::internal
