#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "phad/io/dataset/dataset_error.hpp"
#include "phad/sensor/calibration.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_frame.hpp"

namespace phad::io::dataset
{

  struct StereoFrameRef
  {
    common::Timestamp     timestamp;
    std::filesystem::path left_path;
    std::filesystem::path right_path;
  };

  using StereoImuCalibration = sensor::StereoImuCalibration;

  struct DatasetStreamSummary
  {
    std::size_t                      count = 0;
    std::optional<common::Timestamp> first_timestamp;
    std::optional<common::Timestamp> last_timestamp;
  };

  struct StereoImuDatasetSummary
  {
    DatasetStreamSummary imu;
    DatasetStreamSummary stereo;
  };

  struct DatasetReaderEnd
  {
    auto operator<=>( const DatasetReaderEnd& ) const = default;
  };

  enum class DatasetReaderErrorCode : std::uint8_t
  {
    kImageDecodeFailed   = 0,
    kImageFormatMismatch = 1,
  };

  struct DatasetReaderError
  {
    DatasetReaderErrorCode code;
    std::string            sensor_id;
    common::Timestamp      timestamp;
    std::size_t            record_number;
    std::string            cause;

    bool operator==( const DatasetReaderError& ) const = default;
  };

  template <typename T>
  using DatasetReaderResult =
      std::variant<T, DatasetReaderEnd, DatasetReaderError>;

  namespace internal
  {
    class StereoImuDatasetBuilder;
    class StereoImuDatasetImpl;
  }  // namespace internal

  class StereoImuDatasetReader
  {
  public:
    StereoImuDatasetReader( const StereoImuDatasetReader& )            = delete;
    StereoImuDatasetReader& operator=( const StereoImuDatasetReader& ) = delete;
    StereoImuDatasetReader( StereoImuDatasetReader&& ) noexcept        = default;
    StereoImuDatasetReader& operator=( StereoImuDatasetReader&& ) noexcept =
        default;

    [[nodiscard]] DatasetReaderResult<sensor::ImuMeasurement> takeImu();
    [[nodiscard]] DatasetReaderResult<common::Timestamp>
                                                           peekStereoTimestamp();
    [[nodiscard]] DatasetReaderResult<sensor::StereoFrame> takeStereo();

  private:
    friend class StereoImuDataset;

    explicit StereoImuDatasetReader(
        std::shared_ptr<const internal::StereoImuDatasetImpl> impl );

    std::shared_ptr<const internal::StereoImuDatasetImpl> m_impl;
    std::size_t                                           m_next_imu_index    = 0;
    std::size_t                                           m_next_stereo_index = 0;
    std::optional<DatasetReaderError>                     m_terminal_error;
  };

  class StereoImuDataset
  {
  public:
    [[nodiscard]] StereoImuCalibration calibration() const;

    [[nodiscard]] StereoImuDatasetSummary summary() const noexcept;
    [[nodiscard]] StereoImuDatasetReader  reader() const;

    [[nodiscard]] std::span<const sensor::ImuMeasurement> imuMeasurements()
        const noexcept;

    [[nodiscard]] std::span<const StereoFrameRef> stereoIndex() const noexcept;

    [[nodiscard]] DatasetResult<sensor::StereoFrame> loadStereo(
        std::size_t index ) const;

  private:
    friend class internal::StereoImuDatasetBuilder;

    StereoImuDataset( StereoImuCalibration                calibration,
                      std::vector<sensor::ImuMeasurement> imu_measurements,
                      std::vector<StereoFrameRef>         stereo_index,
                      sensor::PixelType                   left_pixel_type,
                      sensor::PixelType                   right_pixel_type );

    std::shared_ptr<const internal::StereoImuDatasetImpl> m_impl;
  };

}  // namespace phad::io::dataset
