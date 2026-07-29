#include <array>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

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
      std::string_view                       side,
      const phad::sensor::CameraCalibration& calibration )
  {
    std::cout << side << "_camera_model: pinhole\n"
              << side << "_camera_distortion_model: radial-tangential\n"
              << side << "_camera_resolution: "
              << calibration.resolution.width << 'x'
              << calibration.resolution.height << '\n'
              << side << "_camera_rate_hz: " << calibration.rate_hz << '\n';
    printArray(
        std::string{ side } + "_camera_intrinsics",
        std::array{ calibration.intrinsics.fx_pixels,
                    calibration.intrinsics.fy_pixels,
                    calibration.intrinsics.cx_pixels,
                    calibration.intrinsics.cy_pixels } );
    printArray( std::string{ side } + "_camera_distortion_coefficients",
                calibration.distortion_coefficients );
    printArray( std::string{ side } + "_camera_T_B_camera",
                calibration.T_B_camera.matrix );
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
  printCameraCalibration( "left", calibration.left );
  printCameraCalibration( "right", calibration.right );
  printArray( "imu_T_B_imu", calibration.imu.T_B_imu.matrix );
  std::cout << "imu_rate_hz: " << calibration.imu.rate_hz << '\n'
            << "imu_acc_nd: " << calibration.imu.acc_nd << '\n'
            << "imu_gyr_nd: " << calibration.imu.gyr_nd << '\n'
            << "imu_acc_rw: " << calibration.imu.acc_rw << '\n'
            << "imu_gyr_rw: " << calibration.imu.gyr_rw << '\n'
            << "stereo_frames: " << summary.stereo.count << '\n'
            << "imu_measurements: " << summary.imu.count << '\n';
  printTimestamp( "stereo_first_ns", summary.stereo.first_timestamp );
  printTimestamp( "stereo_last_ns", summary.stereo.last_timestamp );
  printTimestamp( "imu_first_ns", summary.imu.first_timestamp );
  printTimestamp( "imu_last_ns", summary.imu.last_timestamp );
  return 0;
}
