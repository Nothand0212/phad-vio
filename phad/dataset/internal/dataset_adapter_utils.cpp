#include "phad/dataset/internal/dataset_adapter_utils.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <system_error>
#include <utility>

namespace phad::dataset::internal
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

  DatasetResult<double> positiveYamlScalar(
      const YAML::Node& root, std::string_view key,
      const std::string& sensor_id, const fs::path& path )
  {
    try
    {
      const double value = root[ std::string( key ) ].as<double>();
      if ( !std::isfinite( value ) || value <= 0.0 )
      {
        return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                          std::string( key ),
                          "value must be finite and positive" );
      }
      return value;
    }
    catch ( const YAML::Exception& exception )
    {
      return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                        std::string( key ), exception.what() );
    }
  }

  DatasetResult<sensor::RigidTransform> validateRigidTransform(
      sensor::RigidTransform transform, const std::string& sensor_id,
      const fs::path& path, std::string field )
  {
    for ( const double value : transform.matrix )
    {
      if ( !std::isfinite( value ) )
      {
        return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                          field, "transform values must be finite" );
      }
    }

    constexpr double tolerance = 1e-6;
    const auto&      matrix    = transform.matrix;
    for ( std::size_t row = 0; row < 3; ++row )
    {
      for ( std::size_t other = 0; other < 3; ++other )
      {
        double dot = 0.0;
        for ( std::size_t column = 0; column < 3; ++column )
        {
          dot += matrix[ row * 4 + column ] *
                 matrix[ other * 4 + column ];
        }
        const double expected = row == other ? 1.0 : 0.0;
        if ( std::abs( dot - expected ) > tolerance )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id,
                            path, field + ".rotation",
                            "rotation is not orthonormal" );
        }
      }
    }

    const double determinant =
        matrix[ 0 ] * ( matrix[ 5 ] * matrix[ 10 ] -
                        matrix[ 6 ] * matrix[ 9 ] ) -
        matrix[ 1 ] * ( matrix[ 4 ] * matrix[ 10 ] -
                        matrix[ 6 ] * matrix[ 8 ] ) +
        matrix[ 2 ] * ( matrix[ 4 ] * matrix[ 9 ] -
                        matrix[ 5 ] * matrix[ 8 ] );
    if ( std::abs( determinant - 1.0 ) > tolerance )
    {
      return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                        field + ".rotation",
                        "rotation determinant must be +1" );
    }
    if ( std::abs( matrix[ 12 ] ) > tolerance ||
         std::abs( matrix[ 13 ] ) > tolerance ||
         std::abs( matrix[ 14 ] ) > tolerance ||
         std::abs( matrix[ 15 ] - 1.0 ) > tolerance )
    {
      return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                        field + ".bottom_row",
                        "homogeneous bottom row must be [0, 0, 0, 1]" );
    }
    return transform;
  }

  sensor::RigidTransform invertRigidTransform(
      const sensor::RigidTransform& transform ) noexcept
  {
    sensor::RigidTransform inverse;
    const auto&            source = transform.matrix;
    auto&                  target = inverse.matrix;
    for ( std::size_t row = 0; row < 3; ++row )
    {
      for ( std::size_t column = 0; column < 3; ++column )
      {
        target[ row * 4 + column ] = source[ column * 4 + row ];
      }
      target[ row * 4 + 3 ] =
          -( target[ row * 4 ] * source[ 3 ] +
             target[ row * 4 + 1 ] * source[ 7 ] +
             target[ row * 4 + 2 ] * source[ 11 ] );
    }
    target[ 12 ] = 0.0;
    target[ 13 ] = 0.0;
    target[ 14 ] = 0.0;
    target[ 15 ] = 1.0;
    return inverse;
  }

  bool isIdentity( const sensor::RigidTransform& transform ) noexcept
  {
    constexpr double tolerance = 1e-12;
    for ( std::size_t index = 0; index < transform.matrix.size(); ++index )
    {
      const bool diagonal =
          index == 0 || index == 5 || index == 10 || index == 15;
      const double expected = diagonal ? 1.0 : 0.0;
      if ( std::abs( transform.matrix[ index ] - expected ) > tolerance )
      {
        return false;
      }
    }
    return true;
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

  DatasetResult<std::vector<StereoFrameRef>> joinStereo(
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

    std::vector<StereoFrameRef> stereo_index;
    stereo_index.reserve( left_records.size() );
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
      stereo_index.push_back(
          StereoFrameRef{ left.timestamp, left.image_path, right.image_path } );
    }
    return stereo_index;
  }

}  // namespace phad::dataset::internal
