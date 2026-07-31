#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>

#include "phad/sensor/camera_id.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_imu_calibration.hpp"

namespace phad::io::dataset
{

  struct DatasetStreamSummary
  {
    std::size_t                      count = 0;
    std::optional<common::Timestamp> first_timestamp;
    std::optional<common::Timestamp> last_timestamp;
  };

  struct StereoImuDatasetSummary
  {
    DatasetStreamSummary imu;
    DatasetStreamSummary left;
    DatasetStreamSummary right;
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
    [[nodiscard]] DatasetReaderResult<common::Timestamp>      peekImageTimestamp(
             sensor::CameraId camera );
    [[nodiscard]] DatasetReaderResult<sensor::ImageFrameEvent> takeImage(
        sensor::CameraId camera );

  private:
    friend class StereoImuDataset;

    explicit StereoImuDatasetReader(
        std::shared_ptr<const internal::StereoImuDatasetImpl> impl );

    std::unique_ptr<internal::StereoImuDatasetReaderImpl> m_impl;
  };

  class StereoImuDataset
  {
  public:
    [[nodiscard]] sensor::StereoImuCalibration calibration() const;

    [[nodiscard]] StereoImuDatasetSummary summary() const noexcept;
    /// Left/right 时间戳 exact 交集大小（诊断用；不解码、不做容差配对）。
    [[nodiscard]] std::size_t exactTimestampIntersectionCount()
        const noexcept;
    [[nodiscard]] StereoImuDatasetReader reader() const;

  private:
    friend class internal::StereoImuDatasetBuilder;

    explicit StereoImuDataset(
        std::shared_ptr<const internal::StereoImuDatasetImpl> impl );

    std::shared_ptr<const internal::StereoImuDatasetImpl> m_impl;
  };

}  // namespace phad::io::dataset
