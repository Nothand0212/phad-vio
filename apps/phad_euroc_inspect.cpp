#include <array>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

#include "phad/io/dataset/euroc/euroc_dataset.hpp"

namespace
{

  template <std::size_t Size>
  void printArray( std::string_view                name,
                   const std::array<double, Size>& values )
  {
    std::cout << name << ": [";
    for ( std::size_t index = 0; index < values.size(); ++index )
    {
      if ( index != 0 )
      {
        std::cout << ", ";
      }
      std::cout << values[ index ];
    }
    std::cout << "]\n";
  }

  void printTimestamp(
      std::string_view                              name,
      const std::optional<phad::common::Timestamp>& timestamp )
  {
    if ( timestamp.has_value() )
    {
      std::cout << name << ": " << timestamp->nanoseconds() << '\n';
    }
  }

  void printCameraCalibration(
      std::string_view                      side,
      const phad::sensor::CameraParameters& parameters,
      const phad::sensor::RigidTransform&   T_B_camera )
  {
    const auto& model =
        std::get<phad::sensor::PinholeRadialTangentialParameters>(
            parameters.modelParameters() );
    std::cout << side << "_camera_model: pinhole\n"
              << side << "_camera_distortion_model: radial-tangential\n"
              << side << "_camera_resolution: "
              << parameters.imageWidth() << 'x'
              << parameters.imageHeight() << '\n'
              << side << "_camera_rate_hz: " << parameters.rateHz() << '\n';
    printArray(
        std::string{ side } + "_camera_intrinsics",
        std::array{ model.fxPixels(), model.fyPixels(), model.cxPixels(),
                    model.cyPixels() } );
    printArray( std::string{ side } + "_camera_distortion_coefficients",
                std::array{ model.k1(), model.k2(), model.p1(), model.p2() } );

    std::array<double, 16> matrix{};
    const auto             rotation    = T_B_camera.rotation();
    const auto             translation = T_B_camera.translation();
    for ( std::size_t row = 0; row < 3; ++row )
    {
      for ( std::size_t column = 0; column < 3; ++column )
      {
        matrix[ row * 4 + column ] =
            rotation( static_cast<Eigen::Index>( row ),
                      static_cast<Eigen::Index>( column ) );
      }
      matrix[ row * 4 + 3 ] =
          translation( static_cast<Eigen::Index>( row ) );
    }
    matrix[ 15 ] = 1.0;
    printArray( std::string{ side } + "_camera_T_B_camera",
                matrix );
  }

}  // namespace

int main( int argc, char** argv )
{
  if ( argc != 2 )
  {
    std::cerr << "usage: phad_euroc_inspect <sequence-root>\n";
    return 2;
  }

  auto opened =
      phad::io::dataset::euroc::open( std::filesystem::path{ argv[ 1 ] } );
  if ( !opened )
  {
    std::cerr << opened.error().describe() << '\n';
    return 1;
  }

  const auto& dataset     = opened.value();
  const auto  calibration = dataset.calibration();
  const auto  summary     = dataset.summary();
  printCameraCalibration( "left", calibration.leftCamera(),
                          calibration.T_B_left_camera() );
  printCameraCalibration( "right", calibration.rightCamera(),
                          calibration.T_B_right_camera() );
  printArray( "imu_T_B_imu",
              std::array{ 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 } );
  std::cout << "imu_rate_hz: " << calibration.imu().rateHz() << '\n'
            << "imu_acc_nd: " << calibration.imu().accNd() << '\n'
            << "imu_gyr_nd: " << calibration.imu().gyrNd() << '\n'
            << "imu_acc_rw: " << calibration.imu().accRw() << '\n'
            << "imu_gyr_rw: " << calibration.imu().gyrRw() << '\n'
            << "cam0_frames: " << summary.left.count << '\n'
            << "cam1_frames: " << summary.right.count << '\n'
            << "stereo_intersection_frames: "
            << dataset.exactTimestampIntersectionCount() << '\n'
            << "imu_measurements: " << summary.imu.count << '\n';
  printTimestamp( "cam0_first_ns", summary.left.first_timestamp );
  printTimestamp( "cam0_last_ns", summary.left.last_timestamp );
  printTimestamp( "cam1_first_ns", summary.right.first_timestamp );
  printTimestamp( "cam1_last_ns", summary.right.last_timestamp );
  printTimestamp( "imu_first_ns", summary.imu.first_timestamp );
  printTimestamp( "imu_last_ns", summary.imu.last_timestamp );
  return 0;
}
