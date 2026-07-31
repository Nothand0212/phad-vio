#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <variant>

#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/sensor/camera_id.hpp"

namespace
{

  TEST( EurocMh01IntegrationTest,
        LoadsAuditedSummaryAndFirstSequentialImageFrames )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || std::string( configured_path ).empty() )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }

    auto opened = phad::io::dataset::euroc::open(
        std::filesystem::path{ configured_path } );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    const auto summary = opened.value().summary();

    ASSERT_EQ( summary.left.count, 3682U );
    ASSERT_EQ( summary.right.count, 3682U );
    ASSERT_EQ( summary.imu.count, 36820U );
    ASSERT_TRUE( summary.left.first_timestamp.has_value() );
    ASSERT_TRUE( summary.left.last_timestamp.has_value() );
    ASSERT_TRUE( summary.right.first_timestamp.has_value() );
    ASSERT_TRUE( summary.right.last_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.first_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.last_timestamp.has_value() );
    EXPECT_EQ( summary.left.first_timestamp->nanoseconds(),
               1'403'636'579'763'555'584 );
    EXPECT_EQ( summary.left.last_timestamp->nanoseconds(),
               1'403'636'763'813'555'456 );
    EXPECT_EQ( summary.right.first_timestamp->nanoseconds(),
               1'403'636'579'763'555'584 );
    EXPECT_EQ( summary.right.last_timestamp->nanoseconds(),
               1'403'636'763'813'555'456 );
    EXPECT_EQ( summary.imu.first_timestamp->nanoseconds(),
               1'403'636'579'758'555'392 );
    EXPECT_EQ( summary.imu.last_timestamp->nanoseconds(),
               1'403'636'763'853'555'456 );

    auto       reader = opened.value().reader();
    const auto left_loaded =
        reader.takeImage( phad::sensor::CameraId::kLeft );
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::ImageFrameEvent>( left_loaded ) );
    const auto& left_event =
        std::get<phad::sensor::ImageFrameEvent>( left_loaded );
    EXPECT_EQ( left_event.timestamp, *summary.left.first_timestamp );
    EXPECT_EQ( left_event.camera, phad::sensor::CameraId::kLeft );
    EXPECT_EQ( left_event.image.width(), 752 );
    EXPECT_EQ( left_event.image.height(), 480 );
    EXPECT_EQ( left_event.image.channels(), 1 );
    EXPECT_EQ( left_event.image.pixelType(), phad::sensor::PixelType::kUint8 );
    EXPECT_TRUE( left_event.image.pixels<std::uint8_t>().has_value() );
    EXPECT_FALSE( left_event.image.pixels<std::uint16_t>().has_value() );

    const auto right_loaded =
        reader.takeImage( phad::sensor::CameraId::kRight );
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::ImageFrameEvent>( right_loaded ) );
    const auto& right_event =
        std::get<phad::sensor::ImageFrameEvent>( right_loaded );
    EXPECT_EQ( right_event.timestamp, *summary.right.first_timestamp );
    EXPECT_EQ( right_event.camera, phad::sensor::CameraId::kRight );
    EXPECT_EQ( right_event.image.width(), 752 );
    EXPECT_EQ( right_event.image.height(), 480 );
    EXPECT_EQ( right_event.image.channels(), 1 );
    EXPECT_EQ( right_event.image.pixelType(), phad::sensor::PixelType::kUint8 );
    EXPECT_TRUE( right_event.image.pixels<std::uint8_t>().has_value() );
    EXPECT_FALSE( right_event.image.pixels<std::uint16_t>().has_value() );
  }

}  // namespace
