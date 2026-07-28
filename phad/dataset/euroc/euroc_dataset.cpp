#include "phad/dataset/euroc/euroc_dataset.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace phad::dataset
{
  namespace
  {

    namespace fs = std::filesystem;

    constexpr std::string_view kCameraHeader = "#timestamp [ns],filename";
    constexpr std::string_view kImuHeader =
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
        "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
        "a_RS_S_z [m s^-2]";

    struct CameraRecord
    {
      common::Timestamp timestamp;
      fs::path          image_path;
      std::size_t       line = 0;
    };

    DatasetError makeError( DatasetErrorCode code, std::string sensor_id,
                            fs::path source_path, std::string field,
                            std::string                      cause,
                            std::optional<std::size_t>       line = std::nullopt,
                            std::optional<common::Timestamp> timestamp =
                                std::nullopt )
    {
      return DatasetError{ code, std::move( sensor_id ), std::move( source_path ), line,
                           timestamp, std::move( field ), std::move( cause ) };
    }

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
                          "timestamp", "expected an integer nanosecond value", line );
      }
      return common::Timestamp{ value };
    }

    DatasetResult<double> parseFiniteDouble( std::string_view   text,
                                             const std::string& sensor_id,
                                             const fs::path& path, std::size_t line,
                                             std::string field )
    {
      double     value = 0.0;
      const auto result =
          std::from_chars( text.data(), text.data() + text.size(), value );
      if ( result.ec != std::errc{} || result.ptr != text.data() + text.size() )
      {
        return makeError( DatasetErrorCode::kInvalidField, sensor_id, path,
                          std::move( field ), "expected a floating-point value", line );
      }
      if ( !std::isfinite( value ) )
      {
        return makeError( DatasetErrorCode::kNonFiniteMeasurement, sensor_id, path,
                          std::move( field ), "measurement must be finite", line );
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
        return makeError( DatasetErrorCode::kOutOfOrderTimestamp, sensor_id, path,
                          "timestamp", "timestamp is earlier than previous record",
                          line, current );
      }
      return std::nullopt;
    }

    bool isWithinDirectory( const fs::path& child, const fs::path& directory )
    {
      const fs::path relative = child.lexically_relative( directory );
      return !relative.empty() && !relative.is_absolute() &&
             *relative.begin() != "..";
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
                                             timestamp.value(), sensor_id, csv_path,
                                             line_number ) )
          {
            return *std::move( error );
          }
        }

        const fs::path filename{ std::string( fields[ 1 ] ) };
        if ( filename.empty() || filename.is_absolute() ||
             filename.filename() != filename || filename == "." ||
             filename == ".." )
        {
          return makeError( DatasetErrorCode::kUnsafeImagePath, sensor_id, csv_path,
                            "filename",
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
        const fs::path& csv_path )
    {
      constexpr std::string_view sensor_id = "imu0";
      std::ifstream              stream( csv_path );
      if ( !stream )
      {
        return makeError( DatasetErrorCode::kIoError, std::string( sensor_id ),
                          csv_path, {}, "failed to open IMU CSV" );
      }

      std::string line_text;
      if ( !std::getline( stream, line_text ) )
      {
        return makeError( DatasetErrorCode::kInvalidCsvHeader,
                          std::string( sensor_id ), csv_path, "header", "CSV is empty",
                          1 );
      }
      removeCarriageReturn( line_text );
      if ( line_text != kImuHeader )
      {
        return makeError( DatasetErrorCode::kInvalidCsvHeader,
                          std::string( sensor_id ), csv_path, "header",
                          "unexpected IMU CSV header", 1 );
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
          return makeError( DatasetErrorCode::kInvalidColumnCount,
                            std::string( sensor_id ), csv_path, {},
                            "IMU row must contain 7 columns", line_number );
        }
        auto timestamp =
            parseTimestamp( fields[ 0 ], std::string( sensor_id ), csv_path, line_number );
        if ( !timestamp )
        {
          return timestamp.error();
        }
        if ( !measurements.empty() )
        {
          if ( auto error = checkIncreasing(
                   measurements.back().timestamp, timestamp.value(),
                   std::string( sensor_id ), csv_path, line_number ) )
          {
            return *std::move( error );
          }
        }
        std::array<double, 6> values{};
        for ( std::size_t index = 0; index < values.size(); ++index )
        {
          auto value = parseFiniteDouble( fields[ index + 1 ], std::string( sensor_id ),
                                          csv_path, line_number, field_names[ index ] );
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
        return makeError( DatasetErrorCode::kIoError, std::string( sensor_id ),
                          csv_path, {}, "failed while reading IMU CSV" );
      }
      return measurements;
    }

    DatasetResult<sensor::RigidTransform> parseTransform(
        const YAML::Node& root, const std::string& sensor_id,
        const fs::path& path )
    {
      try
      {
        const YAML::Node transform = root[ "T_BS" ];
        if ( !transform || transform[ "rows" ].as<int>() != 4 ||
             transform[ "cols" ].as<int>() != 4 )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "T_BS", "T_BS must be a 4x4 matrix" );
        }
        const YAML::Node data = transform[ "data" ];
        if ( !data.IsSequence() || data.size() != 16U )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "T_BS.data", "T_BS data must contain 16 values" );
        }
        sensor::RigidTransform result;
        for ( std::size_t index = 0; index < result.matrix.size(); ++index )
        {
          result.matrix[ index ] = data[ index ].as<double>();
          if ( !std::isfinite( result.matrix[ index ] ) )
          {
            return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                              "T_BS.data", "T_BS values must be finite" );
          }
        }

        constexpr double tolerance = 1e-6;
        for ( int row = 0; row < 3; ++row )
        {
          for ( int other = 0; other < 3; ++other )
          {
            double dot = 0.0;
            for ( int column = 0; column < 3; ++column )
            {
              dot += result.matrix[ static_cast<std::size_t>( row * 4 + column ) ] *
                     result.matrix[ static_cast<std::size_t>( other * 4 + column ) ];
            }
            const double expected = row == other ? 1.0 : 0.0;
            if ( std::abs( dot - expected ) > tolerance )
            {
              return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id,
                                path, "T_BS.rotation",
                                "rotation is not orthonormal" );
            }
          }
        }
        const auto&  m = result.matrix;
        const double determinant =
            m[ 0 ] * ( m[ 5 ] * m[ 10 ] - m[ 6 ] * m[ 9 ] ) -
            m[ 1 ] * ( m[ 4 ] * m[ 10 ] - m[ 6 ] * m[ 8 ] ) +
            m[ 2 ] * ( m[ 4 ] * m[ 9 ] - m[ 5 ] * m[ 8 ] );
        if ( std::abs( determinant - 1.0 ) > tolerance )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "T_BS.rotation",
                            "rotation determinant must be +1" );
        }
        if ( std::abs( m[ 12 ] ) > tolerance || std::abs( m[ 13 ] ) > tolerance ||
             std::abs( m[ 14 ] ) > tolerance || std::abs( m[ 15 ] - 1.0 ) > tolerance )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "T_BS.bottom_row",
                            "homogeneous bottom row must be [0, 0, 0, 1]" );
        }
        return result;
      }
      catch ( const YAML::Exception& exception )
      {
        return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                          "T_BS", exception.what() );
      }
    }

    DatasetResult<double> positiveYamlScalar( const YAML::Node&  root,
                                              const std::string& key,
                                              const std::string& sensor_id,
                                              const fs::path&    path )
    {
      try
      {
        const double value = root[ key ].as<double>();
        if ( !std::isfinite( value ) || value <= 0.0 )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            key, "value must be finite and positive" );
        }
        return value;
      }
      catch ( const YAML::Exception& exception )
      {
        return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                          key, exception.what() );
      }
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
        return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path, {},
                          exception.what() );
      }
    }

    DatasetResult<sensor::CameraCalibration> parseCameraCalibration(
        const fs::path& path, const std::string& sensor_id )
    {
      auto yaml = loadYaml( path, sensor_id );
      if ( !yaml )
      {
        return yaml.error();
      }
      const YAML::Node& root = yaml.value();
      try
      {
        if ( root[ "sensor_type" ].as<std::string>() != "camera" )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "sensor_type", "expected camera" );
        }
        if ( root[ "camera_model" ].as<std::string>() != "pinhole" )
        {
          return makeError( DatasetErrorCode::kUnsupportedCameraModel, sensor_id,
                            path, "camera_model", "only pinhole is supported" );
        }
        if ( root[ "distortion_model" ].as<std::string>() != "radial-tangential" )
        {
          return makeError( DatasetErrorCode::kUnsupportedDistortionModel, sensor_id,
                            path, "distortion_model",
                            "only radial-tangential is supported" );
        }
      }
      catch ( const YAML::Exception& exception )
      {
        return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path, {},
                          exception.what() );
      }

      auto transform = parseTransform( root, sensor_id, path );
      auto rate      = positiveYamlScalar( root, "rate_hz", sensor_id, path );
      if ( !transform )
      {
        return transform.error();
      }
      if ( !rate )
      {
        return rate.error();
      }

      sensor::CameraCalibration calibration;
      calibration.T_B_camera = transform.value();
      calibration.rate_hz    = rate.value();
      try
      {
        const YAML::Node resolution = root[ "resolution" ];
        const YAML::Node intrinsics = root[ "intrinsics" ];
        const YAML::Node distortion = root[ "distortion_coefficients" ];
        if ( !resolution.IsSequence() || resolution.size() != 2U )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "resolution", "resolution must contain width and height" );
        }
        calibration.resolution = { resolution[ 0 ].as<int>(),
                                   resolution[ 1 ].as<int>() };
        if ( calibration.resolution.width <= 0 ||
             calibration.resolution.height <= 0 )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "resolution", "dimensions must be positive" );
        }
        if ( !intrinsics.IsSequence() || intrinsics.size() != 4U )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "intrinsics", "intrinsics must contain 4 values" );
        }
        calibration.intrinsics = { intrinsics[ 0 ].as<double>(),
                                   intrinsics[ 1 ].as<double>(),
                                   intrinsics[ 2 ].as<double>(),
                                   intrinsics[ 3 ].as<double>() };
        const auto& values     = calibration.intrinsics;
        if ( !std::isfinite( values.fx_pixels ) ||
             !std::isfinite( values.fy_pixels ) ||
             !std::isfinite( values.cx_pixels ) ||
             !std::isfinite( values.cy_pixels ) ||
             values.fx_pixels <= 0.0 || values.fy_pixels <= 0.0 )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "intrinsics",
                            "intrinsics must be finite and focal lengths positive" );
        }
        if ( !distortion.IsSequence() || distortion.size() != 4U )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                            "distortion_coefficients",
                            "distortion must contain 4 values" );
        }
        for ( std::size_t index = 0;
              index < calibration.distortion_coefficients.size(); ++index )
        {
          calibration.distortion_coefficients[ index ] =
              distortion[ index ].as<double>();
          if ( !std::isfinite( calibration.distortion_coefficients[ index ] ) )
          {
            return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                              "distortion_coefficients",
                              "distortion values must be finite" );
          }
        }
      }
      catch ( const YAML::Exception& exception )
      {
        return makeError( DatasetErrorCode::kInvalidCalibration, sensor_id, path, {},
                          exception.what() );
      }
      return calibration;
    }

    DatasetResult<sensor::ImuCalibration> parseImuCalibration(
        const fs::path& path )
    {
      constexpr std::string_view sensor_id = "imu0";
      auto                       yaml      = loadYaml( path, std::string( sensor_id ) );
      if ( !yaml )
      {
        return yaml.error();
      }
      const YAML::Node& root = yaml.value();
      try
      {
        if ( root[ "sensor_type" ].as<std::string>() != "imu" )
        {
          return makeError( DatasetErrorCode::kInvalidCalibration,
                            std::string( sensor_id ), path, "sensor_type",
                            "expected imu" );
        }
      }
      catch ( const YAML::Exception& exception )
      {
        return makeError( DatasetErrorCode::kInvalidCalibration,
                          std::string( sensor_id ), path, "sensor_type",
                          exception.what() );
      }
      auto transform = parseTransform( root, std::string( sensor_id ), path );
      auto rate =
          positiveYamlScalar( root, "rate_hz", std::string( sensor_id ), path );
      auto accel_noise = positiveYamlScalar(
          root, "accelerometer_noise_density", std::string( sensor_id ), path );
      auto gyro_noise = positiveYamlScalar(
          root, "gyroscope_noise_density", std::string( sensor_id ), path );
      auto accel_walk = positiveYamlScalar(
          root, "accelerometer_random_walk", std::string( sensor_id ), path );
      auto gyro_walk = positiveYamlScalar(
          root, "gyroscope_random_walk", std::string( sensor_id ), path );
      if ( !transform )
      {
        return transform.error();
      }
      if ( !rate )
      {
        return rate.error();
      }
      if ( !accel_noise )
      {
        return accel_noise.error();
      }
      if ( !gyro_noise )
      {
        return gyro_noise.error();
      }
      if ( !accel_walk )
      {
        return accel_walk.error();
      }
      if ( !gyro_walk )
      {
        return gyro_walk.error();
      }
      return sensor::ImuCalibration{
          transform.value(), rate.value(), accel_noise.value(), gyro_noise.value(),
          accel_walk.value(), gyro_walk.value() };
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

    bool isIdentity( const sensor::RigidTransform& transform )
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

    DatasetResult<sensor::Image> decodeImage(
        const fs::path& path, const std::string& sensor_id, std::size_t index,
        common::Timestamp timestamp, const sensor::ImageResolution& expected )
    {
      cv::Mat decoded;
      try
      {
        decoded = cv::imread( path.string(), cv::IMREAD_UNCHANGED );
      }
      catch ( const cv::Exception& exception )
      {
        return makeError( DatasetErrorCode::kImageDecodeFailed, sensor_id, path,
                          "image", exception.what(), index, timestamp );
      }
      if ( decoded.empty() )
      {
        return makeError( DatasetErrorCode::kImageDecodeFailed, sensor_id, path,
                          "image", "OpenCV could not decode the image", index,
                          timestamp );
      }
      if ( decoded.cols != expected.width || decoded.rows != expected.height )
      {
        return makeError(
            DatasetErrorCode::kImageFormatMismatch, sensor_id, path, "resolution",
            "decoded dimensions do not match sensor calibration", index, timestamp );
      }
      if ( decoded.type() != CV_8UC1 )
      {
        return makeError( DatasetErrorCode::kImageFormatMismatch, sensor_id, path,
                          "pixel_type",
                          "decoded image must be 8-bit single-channel grayscale",
                          index, timestamp );
      }

      const std::size_t         row_bytes = static_cast<std::size_t>( decoded.cols );
      std::vector<std::uint8_t> pixels;
      pixels.reserve( row_bytes * static_cast<std::size_t>( decoded.rows ) );
      for ( int row = 0; row < decoded.rows; ++row )
      {
        const auto* begin = decoded.ptr<std::uint8_t>( row );
        pixels.insert( pixels.end(), begin, begin + row_bytes );
      }
      return sensor::Image{ decoded.cols, decoded.rows, decoded.channels(),
                            sensor::PixelType::kUint8, std::move( pixels ) };
    }

  }  // namespace

  DatasetResult<EurocDataset> EurocDataset::open(
      const fs::path& sequence_root )
  {
    std::error_code ec;
    if ( !fs::is_directory( sequence_root, ec ) || ec )
    {
      return makeError( DatasetErrorCode::kRootNotFound, {}, sequence_root, {},
                        ec ? ec.message() : "sequence root is not a directory" );
    }

    const fs::path mav0 = sequence_root / "mav0";
    for ( const auto* sensor_id : { "cam0", "cam1", "imu0" } )
    {
      const fs::path sensor_root = mav0 / sensor_id;
      for ( const auto& [ path, directory ] :
            std::array<std::pair<fs::path, bool>, 3>{
                std::pair{ sensor_root, true },
                std::pair{ sensor_root / "sensor.yaml", false },
                std::pair{ sensor_root / "data.csv", false } } )
      {
        if ( auto error = validateRequiredPath( path, directory ) )
        {
          error->sensor_id = sensor_id;
          return *std::move( error );
        }
      }
    }
    for ( const auto* sensor_id : { "cam0", "cam1" } )
    {
      if ( auto error =
               validateRequiredPath( mav0 / sensor_id / "data", true ) )
      {
        error->sensor_id = sensor_id;
        return *std::move( error );
      }
    }

    auto left_calibration =
        parseCameraCalibration( mav0 / "cam0" / "sensor.yaml", "cam0" );
    auto right_calibration =
        parseCameraCalibration( mav0 / "cam1" / "sensor.yaml", "cam1" );
    auto imu_calibration = parseImuCalibration( mav0 / "imu0" / "sensor.yaml" );
    if ( !left_calibration )
    {
      return left_calibration.error();
    }
    if ( !right_calibration )
    {
      return right_calibration.error();
    }
    if ( !imu_calibration )
    {
      return imu_calibration.error();
    }
    if ( !isIdentity( imu_calibration.value().T_B_imu ) )
    {
      return makeError(
          DatasetErrorCode::kUnsupportedImuExtrinsics, "imu0",
          mav0 / "imu0" / "sensor.yaml", "T_BS",
          "M1 requires identity IMU extrinsics so sensor-frame measurements "
          "already satisfy the body-frame ImuMeasurement contract" );
    }

    auto left_records     = parseCameraCsv( mav0 / "cam0" / "data.csv",
                                            mav0 / "cam0" / "data", "cam0" );
    auto right_records    = parseCameraCsv( mav0 / "cam1" / "data.csv",
                                            mav0 / "cam1" / "data", "cam1" );
    auto imu_measurements = parseImuCsv( mav0 / "imu0" / "data.csv" );
    if ( !left_records )
    {
      return left_records.error();
    }
    if ( !right_records )
    {
      return right_records.error();
    }
    if ( !imu_measurements )
    {
      return imu_measurements.error();
    }

    if ( left_records.value().size() != right_records.value().size() )
    {
      return makeError( DatasetErrorCode::kStereoTimestampMismatch, "cam0/cam1",
                        mav0, "timestamp",
                        "camera manifests have different record counts" );
    }
    std::vector<StereoFrameRef> stereo_index;
    stereo_index.reserve( left_records.value().size() );
    for ( std::size_t index = 0; index < left_records.value().size(); ++index )
    {
      const auto& left  = left_records.value()[ index ];
      const auto& right = right_records.value()[ index ];
      if ( left.timestamp != right.timestamp )
      {
        return makeError( DatasetErrorCode::kStereoTimestampMismatch,
                          "cam0/cam1", mav0, "timestamp",
                          "camera timestamps do not match exactly", index,
                          left.timestamp );
      }
      stereo_index.push_back(
          { left.timestamp, left.image_path, right.image_path } );
    }

    return EurocDataset{
        EurocCalibration{ left_calibration.value(), right_calibration.value(),
                          imu_calibration.value() },
        std::move( imu_measurements ).value(), std::move( stereo_index ) };
  }

  DatasetResult<sensor::StereoFrame> EurocDataset::loadStereo(
      std::size_t index ) const
  {
    if ( index >= m_stereo_index.size() )
    {
      return makeError( DatasetErrorCode::kIndexOutOfRange, "cam0/cam1", {},
                        "index", "stereo frame index is out of range", index );
    }
    const auto& reference = m_stereo_index[ index ];
    auto        left      = decodeImage( reference.left_path, "cam0", index,
                                         reference.timestamp, m_calibration.left.resolution );
    if ( !left )
    {
      return left.error();
    }
    auto right = decodeImage( reference.right_path, "cam1", index,
                              reference.timestamp, m_calibration.right.resolution );
    if ( !right )
    {
      return right.error();
    }
    return sensor::StereoFrame{ reference.timestamp, std::move( left ).value(),
                                std::move( right ).value() };
  }

}  // namespace phad::dataset
