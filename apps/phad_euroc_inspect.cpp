#include <array>
#include <cstddef>
#include <filesystem>
#include <iostream>

#include "phad/dataset/euroc/euroc_dataset.hpp"

int main( int argc, char** argv )
{
  if ( argc != 2 )
  {
    std::cerr << "usage: phad_euroc_inspect <sequence-root>\n";
    return 2;
  }

  auto opened =
      phad::dataset::EurocDataset::open( std::filesystem::path{ argv[ 1 ] } );
  if ( !opened )
  {
    std::cerr << opened.error().describe() << '\n';
    return 1;
  }

  const auto& dataset = opened.value();
  const auto  stereo  = dataset.stereoIndex();
  const auto  imu     = dataset.imuMeasurements();
  std::cout << "stereo_frames: " << stereo.size() << '\n'
            << "imu_measurements: " << imu.size() << '\n';
  if ( !stereo.empty() )
  {
    std::cout << "stereo_first_ns: " << stereo.front().timestamp.nanoseconds()
              << '\n'
              << "stereo_last_ns: " << stereo.back().timestamp.nanoseconds()
              << '\n';
    const std::array<std::size_t, 3> sample_indices{
        0, stereo.size() / 2, stereo.size() - 1 };
    for ( const std::size_t index : sample_indices )
    {
      auto loaded = dataset.loadStereo( index );
      if ( !loaded )
      {
        std::cerr << loaded.error().describe() << '\n';
        return 1;
      }
      const auto& frame = loaded.value();
      std::cout << "sample[" << index
                << "]: timestamp_ns=" << frame.timestamp.nanoseconds()
                << ", left=" << frame.left.width() << 'x'
                << frame.left.height() << "x" << frame.left.channels()
                << ", right=" << frame.right.width() << 'x'
                << frame.right.height() << "x" << frame.right.channels()
                << '\n';
    }
  }
  if ( !imu.empty() )
  {
    std::cout << "imu_first_ns: " << imu.front().timestamp.nanoseconds() << '\n'
              << "imu_last_ns: " << imu.back().timestamp.nanoseconds() << '\n';
  }
  return 0;
}
