#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>

#include "apps/offline_vo_session.hpp"
#include "phad/io/dataset/tum_vi/tum_vi_dataset.hpp"

namespace
{

  TEST( TumViCorridor1ProductTest, ProcessesFixedHundredFramePrefix )
  {
    const char* configured_path = std::getenv( "PHAD_TUMVI_CORRIDOR1_PATH" );
    if ( configured_path == nullptr || std::string( configured_path ).empty() )
    {
      GTEST_SKIP() << "PHAD_TUMVI_CORRIDOR1_PATH is not set";
    }

    auto opened = phad::io::dataset::tum_vi::open(
        std::filesystem::path{ configured_path } );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();

    phad::apps::OfflineVoSessionOptions options;
    options.max_frames = 100U;
    // M4 canonical product config; this gate reuses the existing bootstrap.
    options.estimator.m_enable_moving_bootstrap = true;
    const auto result                           = phad::apps::runOfflineVoSession(
        std::move( opened ).value(), options );

    const std::uint64_t observations = std::accumulate(
        result.diag.begin(), result.diag.end(), std::uint64_t{ 0 },
        []( std::uint64_t sum, const phad::apps::VoDiagRow& row ) {
          return sum + row.num_observations;
        } );
    const std::uint64_t disparities = std::accumulate(
        result.diag.begin(), result.diag.end(), std::uint64_t{ 0 },
        []( std::uint64_t sum, const phad::apps::VoDiagRow& row ) {
          return sum + row.num_disparity;
        } );
    std::map<std::string, std::uint64_t> status_counts;
    for ( const auto& row : result.diag )
    {
      ++status_counts[ row.status ];
    }
    std::ostringstream status_histogram;
    for ( const auto& [ status, count ] : status_counts )
    {
      if ( status_histogram.tellp() > 0 )
      {
        status_histogram << ',';
      }
      status_histogram << status << '=' << count;
    }
    std::ostringstream trace;
    trace << "error="
          << ( result.error.has_value() ? result.error->detail : "none" )
          << "; image_frames=" << result.counts.image_frames
          << "; diag_size=" << result.diag.size()
          << "; emitted_stereo=" << result.sync.emitted_stereo
          << "; dropped_left=" << result.sync.dropped_left
          << "; dropped_right=" << result.sync.dropped_right
          << "; dropped_left_overflow=" << result.sync.dropped_left_overflow
          << "; dropped_right_overflow="
          << result.sync.dropped_right_overflow
          << "; failed=" << result.counts.failed
          << "; observations=" << observations
          << "; disparities=" << disparities
          << "; status_histogram=" << status_histogram.str();
    ::testing::Test::RecordProperty( "fixed_prefix_summary", trace.str() );
    SCOPED_TRACE( trace.str() );

    EXPECT_FALSE( result.error.has_value() );
    EXPECT_EQ( result.counts.image_frames, 100U );
    EXPECT_EQ( result.diag.size(), 100U );
    EXPECT_EQ( result.sync.emitted_stereo, 100U );
    EXPECT_EQ( result.sync.dropped_left, 0U );
    EXPECT_EQ( result.sync.dropped_right, 0U );
    EXPECT_EQ( result.sync.dropped_left_overflow, 0U );
    EXPECT_EQ( result.sync.dropped_right_overflow, 0U );
    EXPECT_EQ( result.counts.failed, 0U );
    EXPECT_GT( observations, 0U );
    EXPECT_GT( disparities, 0U );
  }

}  // namespace
