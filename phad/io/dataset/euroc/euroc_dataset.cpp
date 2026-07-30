#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <array>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>

#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/io/dataset/internal/dataset_adapter_utils.hpp"
#include "phad/io/dataset/internal/stereo_imu_dataset_builder.hpp"

namespace phad::io::dataset
{
  namespace
  {

    namespace fs      = std::filesystem;
    namespace adapter = internal;

    struct ParsedCamera
    {
      sensor::CameraParameters parameters;
      sensor::RigidTransform   T_B_camera;
    };

    std::string transformSourceField(
        const sensor::CalibrationError& error, std::string_view base )
    {
      if ( error.field_path == "rigid_transform.rotation" )
      {
        return std::string{ base } + ".rotation";
      }
      if ( error.field_path == "rigid_transform.bottom_row" )
      {
        return std::string{ base } + ".bottom_row";
      }
      return std::string{ base };
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
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path, "T_BS",
              "T_BS must be a 4x4 matrix" );
        }
        const YAML::Node data = transform[ "data" ];
        if ( !data.IsSequence() || data.size() != 16U )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "T_BS.data", "T_BS data must contain 16 values" );
        }

        Eigen::Matrix4d matrix;
        for ( std::size_t index = 0; index < 16U; ++index )
        {
          const Eigen::Index row =
              static_cast<Eigen::Index>( index / 4U );
          const Eigen::Index column =
              static_cast<Eigen::Index>( index % 4U );
          matrix( row, column ) = data[ index ].as<double>();
        }
        auto result = sensor::RigidTransform::create( std::move( matrix ) );
        if ( !result )
        {
          return adapter::mapCalibrationError(
              result.error(), sensor_id, path,
              transformSourceField( result.error(), "T_BS" ) );
        }
        return std::move( result ).value();
      }
      catch ( const YAML::Exception& exception )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCalibration, sensor_id, path, "T_BS",
            exception.what() );
      }
    }

    DatasetResult<ParsedCamera> parseCameraCalibration(
        const fs::path& path, const std::string& sensor_id )
    {
      auto yaml = adapter::loadYaml( path, sensor_id );
      if ( !yaml )
      {
        return yaml.error();
      }
      const YAML::Node& root = yaml.value();
      try
      {
        if ( root[ "sensor_type" ].as<std::string>() != "camera" )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "sensor_type", "expected camera" );
        }
        if ( root[ "camera_model" ].as<std::string>() != "pinhole" )
        {
          return adapter::makeError(
              DatasetErrorCode::kUnsupportedCameraModel, sensor_id, path,
              "camera_model", "only pinhole is supported" );
        }
        if ( root[ "distortion_model" ].as<std::string>() !=
             "radial-tangential" )
        {
          return adapter::makeError(
              DatasetErrorCode::kUnsupportedDistortionModel, sensor_id, path,
              "distortion_model",
              "EuRoC requires radial-tangential distortion" );
        }
      }
      catch ( const YAML::Exception& exception )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCalibration, sensor_id, path, {},
            exception.what() );
      }

      auto transform = parseTransform( root, sensor_id, path );
      if ( !transform )
      {
        return transform.error();
      }

      try
      {
        const YAML::Node resolution = root[ "resolution" ];
        const YAML::Node intrinsics = root[ "intrinsics" ];
        const YAML::Node distortion = root[ "distortion_coefficients" ];
        if ( !resolution.IsSequence() || resolution.size() != 2U )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "resolution", "resolution must contain width and height" );
        }
        if ( !intrinsics.IsSequence() || intrinsics.size() != 4U )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "intrinsics", "intrinsics must contain 4 values" );
        }
        if ( !distortion.IsSequence() || distortion.size() != 4U )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "distortion_coefficients",
              "distortion must contain 4 values" );
        }
        auto model = sensor::PinholeRadialTangentialParameters::create(
            intrinsics[ 0 ].as<double>(), intrinsics[ 1 ].as<double>(),
            intrinsics[ 2 ].as<double>(), intrinsics[ 3 ].as<double>(),
            distortion[ 0 ].as<double>(), distortion[ 1 ].as<double>(),
            distortion[ 2 ].as<double>(), distortion[ 3 ].as<double>() );
        if ( !model )
        {
          const bool intrinsics_error =
              model.error().field_path.find( "fx_pixels" ) !=
                  std::string::npos ||
              model.error().field_path.find( "fy_pixels" ) !=
                  std::string::npos ||
              model.error().field_path.find( "cx_pixels" ) !=
                  std::string::npos ||
              model.error().field_path.find( "cy_pixels" ) !=
                  std::string::npos;
          return adapter::mapCalibrationError(
              model.error(), sensor_id, path,
              intrinsics_error ? "intrinsics"
                               : "distortion_coefficients" );
        }

        auto rate =
            adapter::yamlScalar( root, "rate_hz", sensor_id, path );
        if ( !rate )
        {
          return rate.error();
        }
        auto parameters = sensor::CameraParameters::create(
            sensor::CameraModelParameters{ std::move( model ).value() },
            resolution[ 0 ].as<int>(), resolution[ 1 ].as<int>(),
            rate.value() );
        if ( !parameters )
        {
          const std::string field =
              parameters.error().field_path == "camera.rate_hz"
                  ? "rate_hz"
                  : "resolution";
          return adapter::mapCalibrationError(
              parameters.error(), sensor_id, path, field );
        }
        return ParsedCamera{ std::move( parameters ).value(),
                             std::move( transform ).value() };
      }
      catch ( const YAML::Exception& exception )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCalibration, sensor_id, path, {},
            exception.what() );
      }
    }

    std::string imuSourceField( const sensor::CalibrationError& error )
    {
      if ( error.field_path == "imu.rate_hz" )
      {
        return "rate_hz";
      }
      if ( error.field_path == "imu.acc_nd" )
      {
        return "accelerometer_noise_density";
      }
      if ( error.field_path == "imu.gyr_nd" )
      {
        return "gyroscope_noise_density";
      }
      if ( error.field_path == "imu.acc_rw" )
      {
        return "accelerometer_random_walk";
      }
      return "gyroscope_random_walk";
    }

    DatasetResult<sensor::ImuParameters> parseImuCalibration(
        const fs::path& path )
    {
      const std::string sensor_id = "imu0";
      auto              yaml      = adapter::loadYaml( path, sensor_id );
      if ( !yaml )
      {
        return yaml.error();
      }
      const YAML::Node& root = yaml.value();
      try
      {
        if ( root[ "sensor_type" ].as<std::string>() != "imu" )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "sensor_type", "expected imu" );
        }
      }
      catch ( const YAML::Exception& exception )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCalibration, sensor_id, path,
            "sensor_type", exception.what() );
      }

      auto transform = parseTransform( root, sensor_id, path );
      if ( !transform )
      {
        return transform.error();
      }
      if ( !adapter::isIdentity( transform.value() ) )
      {
        return adapter::makeError(
            DatasetErrorCode::kUnsupportedImuExtrinsics, sensor_id, path,
            "T_BS",
            "EuRoC adapter requires identity IMU extrinsics so measurements "
            "already satisfy the body-frame contract" );
      }

      auto rate =
          adapter::yamlScalar( root, "rate_hz", sensor_id, path );
      auto accel_noise = adapter::yamlScalar(
          root, "accelerometer_noise_density", sensor_id, path );
      auto gyro_noise = adapter::yamlScalar(
          root, "gyroscope_noise_density", sensor_id, path );
      auto accel_walk = adapter::yamlScalar(
          root, "accelerometer_random_walk", sensor_id, path );
      auto gyro_walk = adapter::yamlScalar(
          root, "gyroscope_random_walk", sensor_id, path );
      for ( const auto* result :
            { &rate, &accel_noise, &gyro_noise, &accel_walk, &gyro_walk } )
      {
        if ( !*result )
        {
          return result->error();
        }
      }
      auto parameters = sensor::ImuParameters::create(
          rate.value(), accel_noise.value(), gyro_noise.value(),
          accel_walk.value(), gyro_walk.value() );
      if ( !parameters )
      {
        return adapter::mapCalibrationError(
            parameters.error(), sensor_id, path,
            imuSourceField( parameters.error() ) );
      }
      return std::move( parameters ).value();
    }

  }  // namespace

  DatasetResult<StereoImuDataset> euroc::open(
      const fs::path& sequence_root )
  {
    std::error_code ec;
    if ( !fs::is_directory( sequence_root, ec ) || ec )
    {
      return adapter::makeError(
          DatasetErrorCode::kRootNotFound, {}, sequence_root, {},
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
        if ( auto error =
                 adapter::validateRequiredPath( path, directory ) )
        {
          error->sensor_id = sensor_id;
          return *std::move( error );
        }
      }
    }
    for ( const auto* sensor_id : { "cam0", "cam1" } )
    {
      if ( auto error = adapter::validateRequiredPath(
               mav0 / sensor_id / "data", true ) )
      {
        error->sensor_id = sensor_id;
        return *std::move( error );
      }
    }

    auto left_calibration =
        parseCameraCalibration( mav0 / "cam0" / "sensor.yaml", "cam0" );
    auto right_calibration =
        parseCameraCalibration( mav0 / "cam1" / "sensor.yaml", "cam1" );
    auto imu_calibration =
        parseImuCalibration( mav0 / "imu0" / "sensor.yaml" );
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
    auto left_records = adapter::parseCameraCsv(
        mav0 / "cam0" / "data.csv", mav0 / "cam0" / "data", "cam0" );
    auto right_records = adapter::parseCameraCsv(
        mav0 / "cam1" / "data.csv", mav0 / "cam1" / "data", "cam1" );
    auto imu_measurements =
        adapter::parseImuCsv( mav0 / "imu0" / "data.csv", "imu0" );
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
    auto stereo_index = adapter::joinStereo(
        left_records.value(), right_records.value(), mav0 );
    if ( !stereo_index )
    {
      return stereo_index.error();
    }

    auto calibration = sensor::StereoImuCalibration::create(
        left_calibration.value().parameters,
        right_calibration.value().parameters, imu_calibration.value(),
        left_calibration.value().T_B_camera,
        right_calibration.value().T_B_camera );
    if ( !calibration )
    {
      return adapter::mapCalibrationError(
          calibration.error(), "cam0/cam1", mav0,
          "cam0.T_BS/cam1.T_BS" );
    }

    return adapter::StereoImuDatasetBuilder::build(
        std::move( calibration ).value(),
        std::move( imu_measurements ).value(),
        std::move( stereo_index ).value(), sensor::PixelType::kUint8,
        sensor::PixelType::kUint8 );
  }

}  // namespace phad::io::dataset
