#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "phad/io/dataset/euroc/euroc_dataset.hpp"

namespace
{

  TEST( EurocMh01IntegrationTest, LoadsAuditedManifestAndSampleImages )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || std::string( configured_path ).empty() )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }

    auto opened =
        phad::io::dataset::EurocDataset::open( std::filesystem::path{ configured_path } );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    const auto& dataset = opened.value();
    const auto  stereo  = dataset.stereoIndex();
    const auto  imu     = dataset.imuMeasurements();

    ASSERT_EQ( stereo.size(), 3682U );
    ASSERT_EQ( imu.size(), 36820U );
    EXPECT_EQ( stereo.front().timestamp.nanoseconds(), 1'403'636'579'763'555'584 );
    EXPECT_EQ( stereo.back().timestamp.nanoseconds(), 1'403'636'763'813'555'456 );
    EXPECT_EQ( imu.front().timestamp.nanoseconds(), 1'403'636'579'758'555'392 );
    EXPECT_EQ( imu.back().timestamp.nanoseconds(), 1'403'636'763'853'555'456 );
    EXPECT_TRUE( std::ranges::is_sorted( stereo, {}, &phad::io::dataset::StereoFrameRef::timestamp ) );
    EXPECT_TRUE( std::ranges::is_sorted(
        imu, {}, &phad::sensor::ImuMeasurement::timestamp ) );

    for ( const std::size_t index :
          std::array<std::size_t, 3>{ 0, stereo.size() / 2, stereo.size() - 1 } )
    {
      auto loaded = dataset.loadStereo( index );
      ASSERT_TRUE( loaded.hasValue() ) << loaded.error().describe();
      EXPECT_EQ( loaded.value().left.width(), 752 );
      EXPECT_EQ( loaded.value().left.height(), 480 );
      EXPECT_EQ( loaded.value().left.channels(), 1 );
      EXPECT_EQ( loaded.value().left.pixelType(),
                 phad::sensor::PixelType::kUint8 );
      EXPECT_EQ( loaded.value().right.width(), 752 );
      EXPECT_EQ( loaded.value().right.height(), 480 );
      EXPECT_EQ( loaded.value().right.channels(), 1 );
      EXPECT_EQ( loaded.value().right.pixelType(),
                 phad::sensor::PixelType::kUint8 );
    }
  }

}  // namespace
