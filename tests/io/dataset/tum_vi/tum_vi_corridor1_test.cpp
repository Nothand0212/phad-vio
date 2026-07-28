#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "phad/io/dataset/tum_vi/tum_vi_dataset.hpp"

namespace
{

  TEST( TumViCorridor1IntegrationTest, LoadsManifestCalibrationAndSampleImages )
  {
    const char* configured_path =
        std::getenv( "PHAD_TUMVI_CORRIDOR1_PATH" );
    if ( configured_path == nullptr ||
         std::string( configured_path ).empty() )
    {
      GTEST_SKIP() << "PHAD_TUMVI_CORRIDOR1_PATH is not set";
    }

    auto opened = phad::io::dataset::tum_vi::open(
        std::filesystem::path{ configured_path } );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    const auto& dataset = opened.value();
    const auto  stereo  = dataset.stereoIndex();
    const auto  imu     = dataset.imuMeasurements();

    ASSERT_EQ( stereo.size(), 5990U );
    ASSERT_EQ( imu.size(), 59721U );
    EXPECT_EQ( stereo.front().timestamp.nanoseconds(),
               1'520'531'829'251'142'058 );
    EXPECT_EQ( stereo.back().timestamp.nanoseconds(),
               1'520'532'128'710'396'829 );
    EXPECT_EQ( imu.front().timestamp.nanoseconds(),
               1'520'531'829'221'612'058 );
    EXPECT_EQ( imu.back().timestamp.nanoseconds(),
               1'520'532'128'752'735'058 );
    EXPECT_TRUE( std::ranges::is_sorted(
        stereo, {}, &phad::io::dataset::StereoFrameRef::timestamp ) );
    EXPECT_TRUE( std::ranges::is_sorted(
        imu, {}, &phad::sensor::ImuMeasurement::timestamp ) );

    for ( const std::size_t index :
          std::array<std::size_t, 3>{
              0, stereo.size() / 2, stereo.size() - 1 } )
    {
      auto loaded = dataset.loadStereo( index );
      ASSERT_TRUE( loaded.hasValue() ) << loaded.error().describe();
      EXPECT_EQ( loaded.value().left.width(), 512 );
      EXPECT_EQ( loaded.value().left.height(), 512 );
      EXPECT_EQ( loaded.value().left.channels(), 1 );
      EXPECT_EQ( loaded.value().left.pixelType(),
                 phad::sensor::PixelType::kUint16 );
      EXPECT_EQ( loaded.value().right.width(), 512 );
      EXPECT_EQ( loaded.value().right.height(), 512 );
      EXPECT_EQ( loaded.value().right.channels(), 1 );
      EXPECT_EQ( loaded.value().right.pixelType(),
                 phad::sensor::PixelType::kUint16 );
      EXPECT_TRUE(
          loaded.value().left.pixels<std::uint16_t>().has_value() );
      EXPECT_TRUE(
          loaded.value().right.pixels<std::uint16_t>().has_value() );
    }
  }

}  // namespace
