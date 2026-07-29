#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <variant>

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
    const auto  summary = dataset.summary();

    ASSERT_EQ( summary.stereo.count, 5990U );
    ASSERT_EQ( summary.imu.count, 59721U );
    ASSERT_TRUE( summary.stereo.first_timestamp.has_value() );
    ASSERT_TRUE( summary.stereo.last_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.first_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.last_timestamp.has_value() );
    EXPECT_EQ( summary.stereo.first_timestamp->nanoseconds(),
               1'520'531'829'251'142'058 );
    EXPECT_EQ( summary.stereo.last_timestamp->nanoseconds(),
               1'520'532'128'710'396'829 );
    EXPECT_EQ( summary.imu.first_timestamp->nanoseconds(),
               1'520'531'829'221'612'058 );
    EXPECT_EQ( summary.imu.last_timestamp->nanoseconds(),
               1'520'532'128'752'735'058 );

    auto       reader = dataset.reader();
    const auto taken  = reader.takeStereo();
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::StereoFrame>( taken ) );
    const auto& frame = std::get<phad::sensor::StereoFrame>( taken );
    EXPECT_EQ( frame.timestamp, *summary.stereo.first_timestamp );
    EXPECT_EQ( frame.left.width(), 512 );
    EXPECT_EQ( frame.left.height(), 512 );
    EXPECT_EQ( frame.left.channels(), 1 );
    EXPECT_EQ( frame.left.pixelType(), phad::sensor::PixelType::kUint16 );
    EXPECT_EQ( frame.right.width(), 512 );
    EXPECT_EQ( frame.right.height(), 512 );
    EXPECT_EQ( frame.right.channels(), 1 );
    EXPECT_EQ( frame.right.pixelType(), phad::sensor::PixelType::kUint16 );
    EXPECT_TRUE( frame.left.pixels<std::uint16_t>().has_value() );
    EXPECT_TRUE( frame.right.pixels<std::uint16_t>().has_value() );
    EXPECT_FALSE( frame.left.pixels<std::uint8_t>().has_value() );
    EXPECT_FALSE( frame.right.pixels<std::uint8_t>().has_value() );
  }

}  // namespace
