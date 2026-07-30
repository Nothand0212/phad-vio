#include "phad/io/dataset/tum_vi/tum_vi_dataset.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <array>
#include <filesystem>
#include <string>
#include <string_view>
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
        Eigen::Matrix4d matrix;
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
            matrix( static_cast<Eigen::Index>( row ),
                    static_cast<Eigen::Index>( column ) ) =
                node[ row ][ column ].as<double>();
          }
        }
        auto transform =
            sensor::RigidTransform::create( std::move( matrix ) );
        if ( !transform )
        {
          return adapter::mapCalibrationError(
              transform.error(), sensor_id, path,
              transformSourceField( transform.error(), field ) );
        }
        return std::move( transform ).value();
      }
      catch ( const YAML::Exception& exception )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCalibration, sensor_id, path, field,
            exception.what() );
      }
    }

    DatasetResult<ParsedCamera> parseCameraCalibration(
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

      auto T_B_camera =
          adapter::invertRigidTransform( T_camera_imu.value() );
      if ( !T_B_camera )
      {
        return adapter::mapCalibrationError(
            T_B_camera.error(), sensor_id, path,
            transformSourceField( T_B_camera.error(), "T_cam_imu" ) );
      }
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
              "distortion_coeffs", "distortion must contain 4 values" );
        }
        auto model = sensor::PinholeEquidistantParameters::create(
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
              intrinsics_error ? "intrinsics" : "distortion_coeffs" );
        }

        auto parameters = sensor::CameraParameters::create(
            sensor::CameraModelParameters{ std::move( model ).value() },
            resolution[ 0 ].as<int>(), resolution[ 1 ].as<int>(),
            kCameraRateHz );
        if ( !parameters )
        {
          return adapter::mapCalibrationError(
              parameters.error(), sensor_id, path, "resolution" );
        }
        return ParsedCamera{ std::move( parameters ).value(),
                             std::move( T_B_camera ).value() };
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
        return "update_rate";
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
      auto              rate =
          adapter::yamlScalar( root, "update_rate", sensor_id, path );
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

    auto calibration = sensor::StereoImuCalibration::create(
        left_calibration.value().parameters,
        right_calibration.value().parameters, imu_calibration.value(),
        left_calibration.value().T_B_camera,
        right_calibration.value().T_B_camera );
    if ( !calibration )
    {
      return adapter::mapCalibrationError(
          calibration.error(), "cam0/cam1", dso / "camchain.yaml",
          "cam0.T_cam_imu/cam1.T_cam_imu" );
    }

    return adapter::StereoImuDatasetBuilder::build(
        std::move( calibration ).value(),
        std::move( imu_measurements ).value(),
        std::move( stereo_index ).value(), sensor::PixelType::kUint16,
        sensor::PixelType::kUint16 );
  }

}  // namespace phad::io::dataset::tum_vi
