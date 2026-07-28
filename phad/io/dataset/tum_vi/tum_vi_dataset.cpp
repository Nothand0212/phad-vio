#include "phad/io/dataset/tum_vi/tum_vi_dataset.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <string>
#include <utility>

#include "phad/io/dataset/internal/dataset_adapter_utils.hpp"
#include "phad/io/dataset/internal/stereo_imu_dataset_builder.hpp"

namespace phad::io::dataset::tum_vi
{
  namespace
  {

    namespace fs      = std::filesystem;
    namespace adapter = internal;

    constexpr double kCameraRateHz = 20.0;

    sensor::RigidTransform identityTransform()
    {
      sensor::RigidTransform transform;
      transform.matrix[ 0 ]  = 1.0;
      transform.matrix[ 5 ]  = 1.0;
      transform.matrix[ 10 ] = 1.0;
      transform.matrix[ 15 ] = 1.0;
      return transform;
    }

    DatasetResult<sensor::RigidTransform> parseKalibrTransform(
        const YAML::Node& node, const std::string& sensor_id,
        const fs::path& path, const std::string& field )
    {
      try
      {
        if ( !node.IsSequence() || node.size() != 4U )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path, field,
              "transform must contain four rows" );
        }
        sensor::RigidTransform transform;
        for ( std::size_t row = 0; row < 4; ++row )
        {
          if ( !node[ row ].IsSequence() || node[ row ].size() != 4U )
          {
            return adapter::makeError(
                DatasetErrorCode::kInvalidCalibration, sensor_id, path, field,
                "each transform row must contain four values" );
          }
          for ( std::size_t column = 0; column < 4; ++column )
          {
            transform.matrix[ row * 4 + column ] =
                node[ row ][ column ].as<double>();
          }
        }
        return adapter::validateRigidTransform( transform, sensor_id, path,
                                                field );
      }
      catch ( const YAML::Exception& exception )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCalibration, sensor_id, path, field,
            exception.what() );
      }
    }

    DatasetResult<sensor::CameraCalibration> parseCameraCalibration(
        const YAML::Node& root, const std::string& sensor_id,
        const fs::path& path )
    {
      const YAML::Node camera = root[ sensor_id ];
      try
      {
        if ( !camera )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidCalibration, sensor_id, path,
              sensor_id, "camera entry is missing" );
        }
        if ( camera[ "camera_model" ].as<std::string>() != "pinhole" )
        {
          return adapter::makeError(
              DatasetErrorCode::kUnsupportedCameraModel, sensor_id, path,
              "camera_model", "TUM VI adapter requires pinhole cameras" );
        }
        if ( camera[ "distortion_model" ].as<std::string>() != "equidistant" )
        {
          return adapter::makeError(
              DatasetErrorCode::kUnsupportedDistortionModel, sensor_id, path,
              "distortion_model",
              "TUM VI adapter requires equidistant distortion" );
        }
      }
      catch ( const YAML::Exception& exception )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCalibration, sensor_id, path, {},
            exception.what() );
      }

      auto T_camera_imu = parseKalibrTransform(
          camera[ "T_cam_imu" ], sensor_id, path, "T_cam_imu" );
      if ( !T_camera_imu )
      {
        return T_camera_imu.error();
      }

      sensor::CameraCalibration calibration;
      calibration.distortion_model = sensor::DistortionModel::kEquidistant;
      calibration.T_B_camera =
          adapter::invertRigidTransform( T_camera_imu.value() );
      calibration.rate_hz = kCameraRateHz;
      try
      {
        const YAML::Node resolution = camera[ "resolution" ];
        const YAML::Node intrinsics = camera[ "intrinsics" ];
        const YAML::Node distortion = camera[ "distortion_coeffs" ];
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
              "distortion_coeffs", "distortion must contain 4 values" );
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
                "distortion_coeffs", "distortion values must be finite" );
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
      auto              rate =
          adapter::positiveYamlScalar( root, "update_rate", sensor_id, path );
      auto accel_noise = adapter::positiveYamlScalar(
          root, "accelerometer_noise_density", sensor_id, path );
      auto gyro_noise = adapter::positiveYamlScalar(
          root, "gyroscope_noise_density", sensor_id, path );
      auto accel_walk = adapter::positiveYamlScalar(
          root, "accelerometer_random_walk", sensor_id, path );
      auto gyro_walk = adapter::positiveYamlScalar(
          root, "gyroscope_random_walk", sensor_id, path );
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
          identityTransform(), rate.value(), accel_noise.value(),
          gyro_noise.value(), accel_walk.value(), gyro_walk.value() };
    }

  }  // namespace

  DatasetResult<StereoImuDataset> open( const fs::path& sequence_root )
  {
    std::error_code ec;
    if ( !fs::is_directory( sequence_root, ec ) || ec )
    {
      return adapter::makeError(
          DatasetErrorCode::kRootNotFound, {}, sequence_root, {},
          ec ? ec.message() : "sequence root is not a directory" );
    }

    const fs::path mav0 = sequence_root / "mav0";
    const fs::path dso  = sequence_root / "dso";
    for ( const auto& [ path, directory ] :
          std::array<std::pair<fs::path, bool>, 7>{
              std::pair{ mav0 / "cam0", true },
              std::pair{ mav0 / "cam1", true },
              std::pair{ mav0 / "imu0", true },
              std::pair{ mav0 / "cam0" / "data.csv", false },
              std::pair{ mav0 / "cam1" / "data.csv", false },
              std::pair{ mav0 / "imu0" / "data.csv", false },
              std::pair{ dso / "camchain.yaml", false } } )
    {
      if ( auto error = adapter::validateRequiredPath( path, directory ) )
      {
        return *std::move( error );
      }
    }
    if ( auto error =
             adapter::validateRequiredPath( dso / "imu_config.yaml", false ) )
    {
      return *std::move( error );
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

    auto camchain = adapter::loadYaml( dso / "camchain.yaml", "cam0/cam1" );
    if ( !camchain )
    {
      return camchain.error();
    }
    auto left_calibration = parseCameraCalibration(
        camchain.value(), "cam0", dso / "camchain.yaml" );
    auto right_calibration = parseCameraCalibration(
        camchain.value(), "cam1", dso / "camchain.yaml" );
    auto imu_calibration =
        parseImuCalibration( dso / "imu_config.yaml" );
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

    return adapter::StereoImuDatasetBuilder::build(
        StereoImuCalibration{ left_calibration.value(),
                              right_calibration.value(),
                              imu_calibration.value() },
        std::move( imu_measurements ).value(),
        std::move( stereo_index ).value(), sensor::PixelType::kUint16,
        sensor::PixelType::kUint16 );
  }

}  // namespace phad::io::dataset::tum_vi
