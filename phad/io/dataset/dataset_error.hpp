#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "phad/common/timestamp.hpp"

namespace phad::io::dataset
{

  /**
   * @brief DatasetErrorCode 枚举类型用于表示数据集加载过程中的错误类型。
   *
   * 每个枚举值对应一个特定的错误场景，用于标识数据集加载失败的原因。
   * 这些错误类型涵盖了从文件系统操作到数据解析的各种失败情况，便于调试和错误处理。
   */
  enum class DatasetErrorCode : std::uint8_t
  {
    kRootNotFound         = 0,
    kRequiredFileMissing  = 1,
    kIoError              = 2,
    kInvalidCsvHeader     = 3,
    kInvalidColumnCount   = 4,
    kInvalidField         = 5,
    kInvalidTimestamp     = 6,
    kTimestampOverflow    = 7,
    kDuplicateTimestamp   = 8,
    kOutOfOrderTimestamp  = 9,
    kNonFiniteMeasurement = 10,
    kUnsafeImagePath      = 11,
    kImageFileMissing     = 12,
    // 13 was kStereoTimestampMismatch; removed in M3.2 (pairing in phad::sync)
    kInvalidCalibration         = 14,
    kUnsupportedCameraModel     = 15,
    kUnsupportedDistortionModel = 16,
    kUnsupportedImuExtrinsics   = 17,
    kImageDecodeFailed          = 18,
    kImageFormatMismatch        = 19,
    kEmptyStream                = 20,
  };

  /**
   * @brief DatasetError 结构体用于表示数据集加载过程中的错误信息。
   *
   * 该结构体封装了错误代码、相关传感器标识、文件路径、时间戳等信息，
   * 提供对错误场景的详细描述和调试辅助功能。
   */
  struct DatasetError
  {
    DatasetErrorCode                 code = DatasetErrorCode::kInvalidField;
    std::string                      sensor_id;
    std::filesystem::path            source_path;
    std::optional<std::size_t>       line_or_record_index;
    std::optional<common::Timestamp> timestamp;
    std::string                      field;
    std::string                      cause;

    [[nodiscard]] std::string describe() const
    {
      std::ostringstream stream;
      stream << "dataset error " << static_cast<int>( code );
      if ( !sensor_id.empty() )
      {
        stream << ", sensor=" << sensor_id;
      }
      if ( !source_path.empty() )
      {
        stream << ", path=" << source_path.string();
      }
      if ( line_or_record_index.has_value() )
      {
        stream << ", line/record=" << *line_or_record_index;
      }
      if ( timestamp.has_value() )
      {
        stream << ", timestamp_ns=" << timestamp->nanoseconds();
      }
      if ( !field.empty() )
      {
        stream << ", field=" << field;
      }
      if ( !cause.empty() )
      {
        stream << ": " << cause;
      }
      return stream.str();
    }
  };

  template <typename T>
  class DatasetResult
  {
  public:
    DatasetResult( T value ) : m_value( std::move( value ) ) {}
    DatasetResult( DatasetError error ) : m_error( std::move( error ) ) {}

    [[nodiscard]] bool hasValue() const noexcept { return m_value.has_value(); }
    explicit           operator bool() const noexcept { return hasValue(); }

    [[nodiscard]] T&       value() & { return m_value.value(); }
    [[nodiscard]] const T& value() const& { return m_value.value(); }
    [[nodiscard]] T&&      value() && { return std::move( m_value ).value(); }

    [[nodiscard]] DatasetError&       error() & { return m_error.value(); }
    [[nodiscard]] const DatasetError& error() const& { return m_error.value(); }

  private:
    std::optional<T>            m_value;
    std::optional<DatasetError> m_error;
  };

}  // namespace phad::io::dataset
