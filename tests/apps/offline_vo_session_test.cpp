#include "apps/offline_vo_session.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace
{

  using phad::apps::OfflineVoSessionOptions;
  using phad::apps::runOfflineVoSession;
  using phad::apps::VoDiagRow;
  using phad::apps::writeDiagCsv;

  TEST( OfflineVoSessionTest, MissingSequenceReturnsError )
  {
    OfflineVoSessionOptions options;
    options.sequence_root =
        std::filesystem::temp_directory_path() / "phad_missing_euroc_seq";
    std::filesystem::remove_all( options.sequence_root );

    const auto result = runOfflineVoSession( options );
    ASSERT_TRUE( result.error.has_value() );
    EXPECT_FALSE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.image_frames, 0U );
  }

  TEST( OfflineVoSessionTest, WriteDiagCsvMatchesProbeContract )
  {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "phad_session_diag.csv";
    std::filesystem::remove( path );

    VoDiagRow row;
    row.timestamp_ns            = 1403636579763555584LL;
    row.status                  = "ok";
    row.num_observations        = 136;
    row.num_landmarks           = 0;
    row.num_shared              = 0;
    row.low_connectivity        = false;
    row.window_size             = 1;
    row.prior_key               = 0;
    row.reproj_rms_before_px    = 0.0;
    row.reproj_rms_after_px     = 0.0;
    row.num_cheirality          = 0;
    row.lm_iterations           = 0;
    row.max_window_pose_shift_m = 0.0;

    ASSERT_FALSE( writeDiagCsv( path, { row } ).has_value() );

    std::ifstream in( path );
    ASSERT_TRUE( in );
    std::ostringstream oss;
    oss << in.rdbuf();
    const std::string text = oss.str();
    EXPECT_NE( text.find( "timestamp_ns,status,num_obs," ), std::string::npos );
    EXPECT_NE(
        text.find(
            "1403636579763555584,ok,136,0,0,0,1,0,0.000000,0.000000,0,0,0.000000" ),
        std::string::npos );
    std::filesystem::remove( path );
  }

  TEST( OfflineVoSessionTest, MaxFramesLimitsCountsAndCoverageSpan )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }

    OfflineVoSessionOptions options;
    options.sequence_root = configured_path;
    options.max_frames    = 5;

    const auto result = runOfflineVoSession( options );
    ASSERT_FALSE( result.error.has_value() ) << result.error->detail;
    ASSERT_TRUE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.image_frames, 5U );
    EXPECT_EQ( result.counts.ok, result.trajectory->size() );
    EXPECT_EQ( result.diag.size(), 5U );
    EXPECT_LT( result.first_image_ts, result.last_image_ts );
    EXPECT_GT( result.wall_s, 0.0 );
  }

}  // namespace
