#pragma once

#include <utility>

#include "phad/dataset/stereo_imu_dataset.hpp"

namespace phad::dataset::internal
{

  class StereoImuDatasetBuilder
  {
  public:
    [[nodiscard]] static StereoImuDataset build(
        StereoImuCalibration                calibration,
        std::vector<sensor::ImuMeasurement> imu_measurements,
        std::vector<StereoFrameRef>         stereo_index,
        sensor::PixelType                   left_pixel_type,
        sensor::PixelType                   right_pixel_type )
    {
      return StereoImuDataset{
          std::move( calibration ), std::move( imu_measurements ),
          std::move( stereo_index ), left_pixel_type, right_pixel_type };
    }
  };

}  // namespace phad::dataset::internal
