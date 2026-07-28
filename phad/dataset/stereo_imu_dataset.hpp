#pragma once

#include <cstddef>
#include <filesystem>
#include <span>
#include <utility>
#include <vector>

#include "phad/dataset/dataset_error.hpp"
#include "phad/sensor/calibration.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_frame.hpp"

namespace phad::dataset
{

  struct StereoFrameRef
  {
    common::Timestamp     timestamp;
    std::filesystem::path left_path;
    std::filesystem::path right_path;
  };

  struct StereoImuCalibration
  {
    sensor::CameraCalibration left;
    sensor::CameraCalibration right;
    sensor::ImuCalibration    imu;
  };

  namespace internal
  {
    class StereoImuDatasetBuilder;
  }

  class StereoImuDataset
  {
  public:
    [[nodiscard]] const StereoImuCalibration& calibration() const noexcept
    {
      return m_calibration;
    }

    [[nodiscard]] std::span<const sensor::ImuMeasurement> imuMeasurements()
        const noexcept
    {
      return m_imu_measurements;
    }

    [[nodiscard]] std::span<const StereoFrameRef> stereoIndex() const noexcept
    {
      return m_stereo_index;
    }

    [[nodiscard]] DatasetResult<sensor::StereoFrame> loadStereo(
        std::size_t index ) const;

  private:
    friend class internal::StereoImuDatasetBuilder;

    StereoImuDataset( StereoImuCalibration                calibration,
                      std::vector<sensor::ImuMeasurement> imu_measurements,
                      std::vector<StereoFrameRef>         stereo_index,
                      sensor::PixelType                   left_pixel_type,
                      sensor::PixelType                   right_pixel_type )
        : m_calibration( std::move( calibration ) ),
          m_imu_measurements( std::move( imu_measurements ) ),
          m_stereo_index( std::move( stereo_index ) ),
          m_left_pixel_type( left_pixel_type ),
          m_right_pixel_type( right_pixel_type ) {}

    StereoImuCalibration                m_calibration;
    std::vector<sensor::ImuMeasurement> m_imu_measurements;
    std::vector<StereoFrameRef>         m_stereo_index;
    sensor::PixelType                   m_left_pixel_type;
    sensor::PixelType                   m_right_pixel_type;
  };

}  // namespace phad::dataset
