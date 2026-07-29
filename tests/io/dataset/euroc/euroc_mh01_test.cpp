#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <variant>

#include "phad/io/dataset/euroc/euroc_dataset.hpp"

namespace
{

  TEST( EurocMh01IntegrationTest,
        LoadsAuditedSummaryAndFirstSequentialStereoFrame )
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

    ASSERT_EQ( summary.stereo.count, 3682U );
    ASSERT_EQ( summary.imu.count, 36820U );
    ASSERT_TRUE( summary.stereo.first_timestamp.has_value() );
    ASSERT_TRUE( summary.stereo.last_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.first_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.last_timestamp.has_value() );
    EXPECT_EQ( summary.stereo.first_timestamp->nanoseconds(),
               1'403'636'579'763'555'584 );
    EXPECT_EQ( summary.stereo.last_timestamp->nanoseconds(),
               1'403'636'763'813'555'456 );
    EXPECT_EQ( summary.imu.first_timestamp->nanoseconds(),
               1'403'636'579'758'555'392 );
    EXPECT_EQ( summary.imu.last_timestamp->nanoseconds(),
               1'403'636'763'853'555'456 );

    auto       reader = opened.value().reader();
    const auto loaded = reader.takeStereo();
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::StereoFrame>( loaded ) );
    const auto& frame = std::get<phad::sensor::StereoFrame>( loaded );
    EXPECT_EQ( frame.timestamp, *summary.stereo.first_timestamp );
    EXPECT_EQ( frame.left.width(), 752 );
    EXPECT_EQ( frame.left.height(), 480 );
    EXPECT_EQ( frame.left.channels(), 1 );
    EXPECT_EQ( frame.left.pixelType(), phad::sensor::PixelType::kUint8 );
    EXPECT_EQ( frame.right.width(), 752 );
    EXPECT_EQ( frame.right.height(), 480 );
    EXPECT_EQ( frame.right.channels(), 1 );
    EXPECT_EQ( frame.right.pixelType(), phad::sensor::PixelType::kUint8 );
    EXPECT_TRUE( frame.left.pixels<std::uint8_t>().has_value() );
    EXPECT_TRUE( frame.right.pixels<std::uint8_t>().has_value() );
    EXPECT_FALSE( frame.left.pixels<std::uint16_t>().has_value() );
    EXPECT_FALSE( frame.right.pixels<std::uint16_t>().has_value() );
  }

}  // namespace
