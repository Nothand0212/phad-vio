#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>

#include "phad/sensor/calibration.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_frame.hpp"

namespace phad::io::dataset
{

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
    class StereoImuDatasetReaderImpl;
  }  // namespace internal

  class StereoImuDatasetReader
  {
  public:
    ~StereoImuDatasetReader();

    StereoImuDatasetReader( const StereoImuDatasetReader& )            = delete;
    StereoImuDatasetReader& operator=( const StereoImuDatasetReader& ) = delete;
    StereoImuDatasetReader( StereoImuDatasetReader&& ) noexcept;
    StereoImuDatasetReader& operator=( StereoImuDatasetReader&& ) noexcept;

    [[nodiscard]] DatasetReaderResult<sensor::ImuMeasurement> takeImu();
    [[nodiscard]] DatasetReaderResult<common::Timestamp>
                                                           peekStereoTimestamp();
    [[nodiscard]] DatasetReaderResult<sensor::StereoFrame> takeStereo();

  private:
    friend class StereoImuDataset;

    explicit StereoImuDatasetReader(
        std::shared_ptr<const internal::StereoImuDatasetImpl> impl );

    std::unique_ptr<internal::StereoImuDatasetReaderImpl> m_impl;
  };

  class StereoImuDataset
  {
  public:
    [[nodiscard]] StereoImuCalibration calibration() const;

    [[nodiscard]] StereoImuDatasetSummary summary() const noexcept;
    [[nodiscard]] StereoImuDatasetReader  reader() const;

  private:
    friend class internal::StereoImuDatasetBuilder;

    explicit StereoImuDataset(
        std::shared_ptr<const internal::StereoImuDatasetImpl> impl );

    std::shared_ptr<const internal::StereoImuDatasetImpl> m_impl;
  };

}  // namespace phad::io::dataset
