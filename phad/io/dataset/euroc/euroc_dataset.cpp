#include "phad/io/dataset/euroc/euroc_dataset.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>

#include "phad/io/dataset/internal/dataset_adapter_utils.hpp"
#include "phad/io/dataset/internal/stereo_imu_dataset_builder.hpp"

namespace phad::io::dataset
{
  namespace
  {

    namespace fs      = std::filesystem;
    namespace adapter = internal;

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

        sensor::RigidTransform result;
        for ( std::size_t index = 0; index < result.matrix.size(); ++index )
        {
          result.matrix[ index ] = data[ index ].as<double>();
        }
        return adapter::validateRigidTransform( result, sensor_id, path,
                                                "T_BS" );
      }
      catch ( const YAML::Exception& exception )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCalibration, sensor_id, path, "T_BS",
            exception.what() );
      }
    }

    DatasetResult<sensor::CameraCalibration> parseCameraCalibration(
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
      auto rate =
          adapter::positiveYamlScalar( root, "rate_hz", sensor_id, path );
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
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "resolution", "resolution must contain width and height" );
        }
        calibration.resolution = { resolution[ 0 ].as<int>(),
                                   resolution[ 1 ].as<int>() };
        if ( calibration.resolution.width <= 0 ||
             calibration.resolution.height <= 0 )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "resolution", "dimensions must be positive" );
        }
        if ( !intrinsics.IsSequence() || intrinsics.size() != 4U )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "intrinsics", "intrinsics must contain 4 values" );
        }
        calibration.intrinsics = {
            intrinsics[ 0 ].as<double>(), intrinsics[ 1 ].as<double>(),
            intrinsics[ 2 ].as<double>(), intrinsics[ 3 ].as<double>() };
        const auto& values = calibration.intrinsics;
        if ( !std::isfinite( values.fx_pixels ) ||
             !std::isfinite( values.fy_pixels ) ||
             !std::isfinite( values.cx_pixels ) ||
             !std::isfinite( values.cy_pixels ) ||
             values.fx_pixels <= 0.0 || values.fy_pixels <= 0.0 )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "intrinsics",
              "intrinsics must be finite and focal lengths positive" );
        }
        if ( !distortion.IsSequence() || distortion.size() != 4U )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              "distortion_coefficients",
              "distortion must contain 4 values" );
        }
        for ( std::size_t index = 0;
              index < calibration.distortion_coefficients.size(); ++index )
        {
          calibration.distortion_coefficients[ index ] =
              distortion[ index ].as<double>();
          if ( !std::isfinite(
                   calibration.distortion_coefficients[ index ] ) )
          {
            return adapter::makeError(
                DatasetErrorCode::kInvalidCalibration, sensor_id, path,
                "distortion_coefficients",
                "distortion values must be finite" );
          }
        }
      }
      catch ( const YAML::Exception& exception )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCalibration, sensor_id, path, {},
            exception.what() );
      }
      return calibration;
    }

    DatasetResult<sensor::ImuCalibration> parseImuCalibration(
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
      auto rate =
          adapter::positiveYamlScalar( root, "rate_hz", sensor_id, path );
      auto accel_noise = adapter::positiveYamlScalar(
          root, "accelerometer_noise_density", sensor_id, path );
      auto gyro_noise = adapter::positiveYamlScalar(
          root, "gyroscope_noise_density", sensor_id, path );
      auto accel_walk = adapter::positiveYamlScalar(
          root, "accelerometer_random_walk", sensor_id, path );
      auto gyro_walk = adapter::positiveYamlScalar(
          root, "gyroscope_random_walk", sensor_id, path );
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
          transform.value(), rate.value(), accel_noise.value(),
          gyro_noise.value(), accel_walk.value(), gyro_walk.value() };
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
    if ( !adapter::isIdentity( imu_calibration.value().T_B_imu ) )
    {
      return adapter::makeError(
          DatasetErrorCode::kUnsupportedImuExtrinsics, "imu0",
          mav0 / "imu0" / "sensor.yaml", "T_BS",
          "EuRoC adapter requires identity IMU extrinsics so measurements "
          "already satisfy the body-frame contract" );
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

    return adapter::StereoImuDatasetBuilder::build(
        StereoImuCalibration{ left_calibration.value(),
                              right_calibration.value(),
                              imu_calibration.value() },
        std::move( imu_measurements ).value(),
        std::move( stereo_index ).value(), sensor::PixelType::kUint8,
        sensor::PixelType::kUint8 );
  }

}  // namespace phad::io::dataset
