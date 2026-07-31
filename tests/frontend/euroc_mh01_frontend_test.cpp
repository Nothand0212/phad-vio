#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <string>
#include <variant>

#include "phad/camera/stereo_rectifier.hpp"
#include "phad/frontend/stereo_tracker.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/io/dataset/stereo_imu_dataset.hpp"

/**
 * @file euroc_mh01_frontend_test.cpp
 * @brief MH_01_easy 全序列 frontend 出口条件门控测试。
 *
 * 需要 PHAD_ENABLE_MH01_TESTS=ON 且设置 PHAD_EUROC_MH01_PATH。
 */

namespace
{

  namespace fs = std::filesystem;

  using phad::frontend::LandmarkId;
  using phad::frontend::StereoTracker;

  [[nodiscard]] fs::path sequenceRoot()
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr )
    {
      return {};
    }
    return fs::path{ configured_path };
  }

  class Mh01FrontendTest : public ::testing::Test
  {
  protected:
    void SetUp() override
    {
      m_root = sequenceRoot();
      if ( m_root.empty() )
      {
        GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
      }
    }

    fs::path m_root;
  };

  TEST_F( Mh01FrontendTest, FullSequenceKeepsTracksAndIds )
  {
    auto opened = phad::io::dataset::euroc::open( m_root );
    ASSERT_TRUE( opened ) << opened.error().describe();

    auto rectifier =
        phad::camera::StereoRectifier::create( opened.value().calibration() );
    ASSERT_TRUE( rectifier ) << rectifier.error().detail;

    StereoTracker tracker( rectifier.value().calibration() );
    auto          reader = opened.value().reader();

    std::set<LandmarkId> seen_ids;
    std::uint64_t        frame_count = 0;
    double               epi_sum     = 0.0;

    while ( true )
    {
      auto loaded = reader.takeStereo();
      if ( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
               loaded ) )
      {
        break;
      }
      ASSERT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>( loaded ) )
          << "stereo reader failed mid-sequence";

      const auto& raw = std::get<phad::sensor::StereoFrame>( loaded );
      auto rectified  = rectifier.value().rectify( raw );
      ASSERT_TRUE( rectified ) << rectified.error().detail;

      const auto tracks = tracker.process( rectified.value() );
      ASSERT_FALSE( tracks.observations.empty() )
          << "track count hit zero at frame " << frame_count;
      EXPECT_LT( tracks.stats.epipolar_median_px, 1.5 )
          << "frame " << frame_count;

      std::set<LandmarkId> frame_ids;
      for ( const auto& observation : tracks.observations )
      {
        ASSERT_TRUE( frame_ids.insert( observation.id ).second )
            << "duplicate LandmarkId " << observation.id << " in frame "
            << frame_count;
        seen_ids.insert( observation.id );
      }

      epi_sum += tracks.stats.epipolar_median_px;
      ++frame_count;
    }

    ASSERT_EQ( frame_count, 3682U );
    EXPECT_LT( epi_sum / static_cast<double>( frame_count ), 1.0 );

    // Monotonic ids starting at 1 with no reuse => max id == distinct count.
    LandmarkId max_id = 0;
    for ( LandmarkId id : seen_ids )
    {
      max_id = std::max( max_id, id );
    }
    EXPECT_EQ( max_id, static_cast<LandmarkId>( seen_ids.size() ) );
  }

}  // namespace
