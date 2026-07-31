#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <variant>

#include "phad/io/dataset/tum_vi/tum_vi_dataset.hpp"
#include "phad/sensor/camera_id.hpp"

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

    ASSERT_EQ( summary.left.count, 5990U );
    ASSERT_EQ( summary.right.count, 5990U );
    ASSERT_EQ( summary.imu.count, 59721U );
    ASSERT_TRUE( summary.left.first_timestamp.has_value() );
    ASSERT_TRUE( summary.left.last_timestamp.has_value() );
    ASSERT_TRUE( summary.right.first_timestamp.has_value() );
    ASSERT_TRUE( summary.right.last_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.first_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.last_timestamp.has_value() );
    EXPECT_EQ( summary.left.first_timestamp->nanoseconds(),
               1'520'531'829'251'142'058 );
    EXPECT_EQ( summary.left.last_timestamp->nanoseconds(),
               1'520'532'128'710'396'829 );
    EXPECT_EQ( summary.right.first_timestamp->nanoseconds(),
               1'520'531'829'251'142'058 );
    EXPECT_EQ( summary.right.last_timestamp->nanoseconds(),
               1'520'532'128'710'396'829 );
    EXPECT_EQ( summary.imu.first_timestamp->nanoseconds(),
               1'520'531'829'221'612'058 );
    EXPECT_EQ( summary.imu.last_timestamp->nanoseconds(),
               1'520'532'128'752'735'058 );

    auto       reader = dataset.reader();
    const auto left_taken =
        reader.takeImage( phad::sensor::CameraId::kLeft );
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::ImageFrameEvent>( left_taken ) );
    const auto& left_event =
        std::get<phad::sensor::ImageFrameEvent>( left_taken );
    EXPECT_EQ( left_event.timestamp, *summary.left.first_timestamp );
    EXPECT_EQ( left_event.camera, phad::sensor::CameraId::kLeft );
    EXPECT_EQ( left_event.image.width(), 512 );
    EXPECT_EQ( left_event.image.height(), 512 );
    EXPECT_EQ( left_event.image.channels(), 1 );
    EXPECT_EQ( left_event.image.pixelType(), phad::sensor::PixelType::kUint16 );
    EXPECT_TRUE( left_event.image.pixels<std::uint16_t>().has_value() );
    EXPECT_FALSE( left_event.image.pixels<std::uint8_t>().has_value() );

    const auto right_taken =
        reader.takeImage( phad::sensor::CameraId::kRight );
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::ImageFrameEvent>( right_taken ) );
    const auto& right_event =
        std::get<phad::sensor::ImageFrameEvent>( right_taken );
    EXPECT_EQ( right_event.timestamp, *summary.right.first_timestamp );
    EXPECT_EQ( right_event.camera, phad::sensor::CameraId::kRight );
    EXPECT_EQ( right_event.image.width(), 512 );
    EXPECT_EQ( right_event.image.height(), 512 );
    EXPECT_EQ( right_event.image.channels(), 1 );
    EXPECT_EQ( right_event.image.pixelType(), phad::sensor::PixelType::kUint16 );
    EXPECT_TRUE( right_event.image.pixels<std::uint16_t>().has_value() );
    EXPECT_FALSE( right_event.image.pixels<std::uint8_t>().has_value() );
  }

}  // namespace
