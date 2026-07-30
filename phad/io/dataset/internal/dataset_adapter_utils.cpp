#include "phad/io/dataset/internal/dataset_adapter_utils.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <system_error>
#include <utility>

/**
 * @file dataset_adapter_utils.cpp
 * @brief 数据集加载过程中的辅助工具函数实现。
 *
 * 该文件实现了 phad::io::dataset::internal 命名空间中的辅助工具函数，
 * 用于数据集加载过程中的错误处理和数据解析。
 */

namespace phad::io::dataset::internal
{
  namespace
  {

    namespace fs = std::filesystem;

    constexpr std::string_view kCameraHeader = "#timestamp [ns],filename";
    constexpr std::string_view kImuHeader =
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
        "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
        "a_RS_S_z [m s^-2]";

    void removeCarriageReturn( std::string& line )
    {
      if ( !line.empty() && line.back() == '\r' )
      {
        line.pop_back();
      }
    }

    std::vector<std::string_view> splitCsv( const std::string& line )
    {
      std::vector<std::string_view> fields;
      std::size_t                   begin = 0;
      while ( true )
      {
        const std::size_t comma = line.find( ',', begin );
        if ( comma == std::string::npos )
        {
          fields.emplace_back( line.data() + begin, line.size() - begin );
          return fields;
        }
        fields.emplace_back( line.data() + begin, comma - begin );
        begin = comma + 1;
      }
    }

    DatasetResult<common::Timestamp> parseTimestamp(
        std::string_view text, const std::string& sensor_id, const fs::path& path,
        std::size_t line )
    {
      std::int64_t value = 0;
      const auto   result =
          std::from_chars( text.data(), text.data() + text.size(), value, 10 );
      if ( result.ec == std::errc::result_out_of_range )
      {
        return makeError( DatasetErrorCode::kTimestampOverflow, sensor_id, path,
                          "timestamp", "timestamp does not fit int64_t", line );
      }
      if ( result.ec != std::errc{} || result.ptr != text.data() + text.size() )
      {
        return makeError( DatasetErrorCode::kInvalidTimestamp, sensor_id, path,
                          "timestamp", "expected an integer nanosecond value",
                          line );
      }
      return common::Timestamp{ value };
    }

    DatasetResult<double> parseFiniteDouble(
        std::string_view text, const std::string& sensor_id, const fs::path& path,
        std::size_t line, std::string field )
    {
      double     value = 0.0;
      const auto result =
          std::from_chars( text.data(), text.data() + text.size(), value );
      if ( result.ec != std::errc{} || result.ptr != text.data() + text.size() )
      {
        return makeError( DatasetErrorCode::kInvalidField, sensor_id, path,
                          std::move( field ), "expected a floating-point value",
                          line );
      }
      if ( !std::isfinite( value ) )
      {
        return makeError( DatasetErrorCode::kNonFiniteMeasurement, sensor_id,
                          path, std::move( field ),
                          "measurement must be finite", line );
      }
      return value;
    }

    std::optional<DatasetError> checkIncreasing(
        common::Timestamp previous, common::Timestamp current,
        const std::string& sensor_id, const fs::path& path, std::size_t line )
    {
      if ( current == previous )
      {
        return makeError( DatasetErrorCode::kDuplicateTimestamp, sensor_id, path,
                          "timestamp", "duplicate timestamp", line, current );
      }
      if ( current < previous )
      {
        return makeError( DatasetErrorCode::kOutOfOrderTimestamp, sensor_id,
                          path, "timestamp",
                          "timestamp is earlier than previous record", line,
                          current );
      }
      return std::nullopt;
    }

    bool isWithinDirectory( const fs::path& child, const fs::path& directory )
    {
      const fs::path relative = child.lexically_relative( directory );
      return !relative.empty() && !relative.is_absolute() &&
             *relative.begin() != "..";
    }

  }  // namespace

  DatasetError makeError( DatasetErrorCode code, std::string sensor_id,
                          fs::path source_path, std::string field,
                          std::string cause, std::optional<std::size_t> line,
                          std::optional<common::Timestamp> timestamp )
  {
    return DatasetError{ code,
                         std::move( sensor_id ),
                         std::move( source_path ),
                         line,
                         timestamp,
                         std::move( field ),
                         std::move( cause ) };
  }

  DatasetResult<YAML::Node> loadYaml( const fs::path&    path,
                                      const std::string& sensor_id )
  {
    try
    {
      return YAML::LoadFile( path.string() );
    }
    catch ( const YAML::Exception& exception )
    {
      return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                        {}, exception.what() );
    }
  }

  DatasetResult<double> yamlScalar(
      const YAML::Node& root, std::string_view key,
      const std::string& sensor_id, const fs::path& path )
  {
    try
    {
      return root[ std::string( key ) ].as<double>();
    }
    catch ( const YAML::Exception& exception )
    {
      return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                        std::string( key ), exception.what() );
    }
  }

  DatasetError mapCalibrationError(
      const sensor::CalibrationError& error, std::string sensor_id,
      fs::path source_path, std::string field )
  {
    std::ostringstream cause;
    cause << "core calibration error " << static_cast<int>( error.code )
          << ", field=" << error.field_path << ": " << error.detail;
    return makeError( DatasetErrorCode::kInvalidCalibration,
                      std::move( sensor_id ), std::move( source_path ),
                      std::move( field ), std::move( cause ).str() );
  }

  sensor::CalibrationResult<sensor::RigidTransform> invertRigidTransform(
      const sensor::RigidTransform& transform )
  {
    Eigen::Matrix4d       inverse  = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d rotation = transform.rotation();
    inverse.block<3, 3>( 0, 0 )    = rotation.transpose();
    inverse.block<3, 1>( 0, 3 ) =
        -rotation.transpose() * transform.translation();
    return sensor::RigidTransform::create( std::move( inverse ) );
  }

  bool isIdentity( const sensor::RigidTransform& transform ) noexcept
  {
    constexpr double tolerance = 1e-12;
    return transform.rotation().isApprox( Eigen::Matrix3d::Identity(),
                                          tolerance ) &&
           transform.translation().isZero( tolerance );
  }

  std::optional<DatasetError> validateRequiredPath( const fs::path& path,
                                                    bool            directory )
  {
    std::error_code ec;
    const bool      valid = directory ? fs::is_directory( path, ec )
                                      : fs::is_regular_file( path, ec );
    if ( !valid || ec )
    {
      return makeError( DatasetErrorCode::kRequiredFileMissing, {}, path, {},
                        ec ? ec.message()
                           : ( directory ? "required directory is missing"
                                         : "required file is missing" ) );
    }
    return std::nullopt;
  }

  DatasetResult<std::vector<CameraRecord>> parseCameraCsv(
      const fs::path& csv_path, const fs::path& data_path,
      const std::string& sensor_id )
  {
    std::ifstream stream( csv_path );
    if ( !stream )
    {
      return makeError( DatasetErrorCode::kIoError, sensor_id, csv_path, {},
                        "failed to open camera CSV" );
    }

    std::string line_text;
    if ( !std::getline( stream, line_text ) )
    {
      return makeError( DatasetErrorCode::kInvalidCsvHeader, sensor_id, csv_path,
                        "header", "CSV is empty", 1 );
    }
    removeCarriageReturn( line_text );
    if ( line_text != kCameraHeader )
    {
      return makeError( DatasetErrorCode::kInvalidCsvHeader, sensor_id, csv_path,
                        "header", "unexpected camera CSV header", 1 );
    }

    std::error_code ec;
    const fs::path  canonical_data = fs::canonical( data_path, ec );
    if ( ec )
    {
      return makeError( DatasetErrorCode::kRequiredFileMissing, sensor_id,
                        data_path, {}, ec.message() );
    }

    std::vector<CameraRecord> records;
    std::size_t               line_number = 1;
    while ( std::getline( stream, line_text ) )
    {
      ++line_number;
      removeCarriageReturn( line_text );
      const auto fields = splitCsv( line_text );
      if ( fields.size() != 2U )
      {
        return makeError( DatasetErrorCode::kInvalidColumnCount, sensor_id,
                          csv_path, {}, "camera row must contain 2 columns",
                          line_number );
      }
      auto timestamp =
          parseTimestamp( fields[ 0 ], sensor_id, csv_path, line_number );
      if ( !timestamp )
      {
        return timestamp.error();
      }
      if ( !records.empty() )
      {
        if ( auto error = checkIncreasing( records.back().timestamp,
                                           timestamp.value(), sensor_id,
                                           csv_path, line_number ) )
        {
          return *std::move( error );
        }
      }

      const fs::path filename{ std::string( fields[ 1 ] ) };
      if ( filename.empty() || filename.is_absolute() ||
           filename.filename() != filename || filename == "." ||
           filename == ".." )
      {
        return makeError( DatasetErrorCode::kUnsafeImagePath, sensor_id,
                          csv_path, "filename",
                          "image reference must be a non-empty basename",
                          line_number, timestamp.value() );
      }
      const fs::path image_path      = data_path / filename;
      const fs::path canonical_image = fs::canonical( image_path, ec );
      if ( ec )
      {
        return makeError( DatasetErrorCode::kImageFileMissing, sensor_id,
                          image_path, "filename", ec.message(), line_number,
                          timestamp.value() );
      }
      if ( !isWithinDirectory( canonical_image, canonical_data ) )
      {
        return makeError( DatasetErrorCode::kUnsafeImagePath, sensor_id,
                          image_path, "filename",
                          "resolved image escapes the sensor data directory",
                          line_number, timestamp.value() );
      }
      if ( !fs::is_regular_file( canonical_image, ec ) || ec )
      {
        return makeError( DatasetErrorCode::kImageFileMissing, sensor_id,
                          image_path, "filename",
                          ec ? ec.message() : "image is not a regular file",
                          line_number, timestamp.value() );
      }
      records.push_back(
          CameraRecord{ timestamp.value(), canonical_image, line_number } );
    }
    if ( stream.bad() )
    {
      return makeError( DatasetErrorCode::kIoError, sensor_id, csv_path, {},
                        "failed while reading camera CSV" );
    }
    return records;
  }

  DatasetResult<std::vector<sensor::ImuMeasurement>> parseImuCsv(
      const fs::path& csv_path, const std::string& sensor_id )
  {
    std::ifstream stream( csv_path );
    if ( !stream )
    {
      return makeError( DatasetErrorCode::kIoError, sensor_id, csv_path, {},
                        "failed to open IMU CSV" );
    }

    std::string line_text;
    if ( !std::getline( stream, line_text ) )
    {
      return makeError( DatasetErrorCode::kInvalidCsvHeader, sensor_id, csv_path,
                        "header", "CSV is empty", 1 );
    }
    removeCarriageReturn( line_text );
    if ( line_text != kImuHeader )
    {
      return makeError( DatasetErrorCode::kInvalidCsvHeader, sensor_id, csv_path,
                        "header", "unexpected IMU CSV header", 1 );
    }

    const std::array<std::string, 6> field_names{
        "w_RS_S_x", "w_RS_S_y", "w_RS_S_z",
        "a_RS_S_x", "a_RS_S_y", "a_RS_S_z" };
    std::vector<sensor::ImuMeasurement> measurements;
    std::size_t                         line_number = 1;
    while ( std::getline( stream, line_text ) )
    {
      ++line_number;
      removeCarriageReturn( line_text );
      const auto fields = splitCsv( line_text );
      if ( fields.size() != 7U )
      {
        return makeError( DatasetErrorCode::kInvalidColumnCount, sensor_id,
                          csv_path, {}, "IMU row must contain 7 columns",
                          line_number );
      }
      auto timestamp =
          parseTimestamp( fields[ 0 ], sensor_id, csv_path, line_number );
      if ( !timestamp )
      {
        return timestamp.error();
      }
      if ( !measurements.empty() )
      {
        if ( auto error = checkIncreasing(
                 measurements.back().timestamp, timestamp.value(), sensor_id,
                 csv_path, line_number ) )
        {
          return *std::move( error );
        }
      }

      std::array<double, 6> values{};
      for ( std::size_t index = 0; index < values.size(); ++index )
      {
        auto value =
            parseFiniteDouble( fields[ index + 1 ], sensor_id, csv_path,
                               line_number, field_names[ index ] );
        if ( !value )
        {
          DatasetError error = value.error();
          error.timestamp    = timestamp.value();
          return error;
        }
        values[ index ] = value.value();
      }
      measurements.push_back( sensor::ImuMeasurement{
          timestamp.value(), { values[ 3 ], values[ 4 ], values[ 5 ] }, { values[ 0 ], values[ 1 ], values[ 2 ] } } );
    }
    if ( stream.bad() )
    {
      return makeError( DatasetErrorCode::kIoError, sensor_id, csv_path, {},
                        "failed while reading IMU CSV" );
    }
    return measurements;
  }

  DatasetResult<std::vector<StereoFrameManifestEntry>> joinStereo(
      const std::vector<CameraRecord>& left_records,
      const std::vector<CameraRecord>& right_records,
      const fs::path&                  source_path )
  {
    if ( left_records.size() != right_records.size() )
    {
      return makeError( DatasetErrorCode::kStereoTimestampMismatch,
                        "cam0/cam1", source_path, "timestamp",
                        "camera manifests have different record counts" );
    }

    std::vector<StereoFrameManifestEntry> stereo_manifest;
    stereo_manifest.reserve( left_records.size() );
    for ( std::size_t index = 0; index < left_records.size(); ++index )
    {
      const auto& left  = left_records[ index ];
      const auto& right = right_records[ index ];
      if ( left.timestamp != right.timestamp )
      {
        return makeError( DatasetErrorCode::kStereoTimestampMismatch,
                          "cam0/cam1", source_path, "timestamp",
                          "camera timestamps do not match exactly", index,
                          left.timestamp );
      }
      stereo_manifest.push_back( StereoFrameManifestEntry{
          left.timestamp, left.image_path, right.image_path } );
    }
    return stereo_manifest;
  }

}  // namespace phad::io::dataset::internal
