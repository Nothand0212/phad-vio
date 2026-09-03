#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
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
    // M5 Slice B product activation remains opt-in.
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
          << "; rejected=" << result.counts.rejected
          << "; segments=" << result.counts.segments
          << "; reanchors=" << result.counts.reanchors
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
    EXPECT_EQ( result.counts.rejected, 0U );
    EXPECT_EQ( result.counts.segments, 1U );
    EXPECT_EQ( result.counts.reanchors, 0U );
    EXPECT_GT( observations, 0U );
    EXPECT_GT( disparities, 0U );
    ASSERT_TRUE( result.trajectory.has_value() );
    EXPECT_EQ( result.counts.ok, result.trajectory->size() );

    const auto first_ok = std::find_if(
        result.diag.begin(), result.diag.end(),
        []( const phad::apps::VoDiagRow& row ) {
          return row.status == "ok";
        } );
    ASSERT_NE( first_ok, result.diag.end() );
    const auto first_ok_index = static_cast<std::size_t>(
        std::distance( result.diag.begin(), first_ok ) );
    EXPECT_GE( first_ok_index, 5U );
    EXPECT_GE( first_ok->timestamp_ns - result.diag.front().timestamp_ns,
               250'000'000 );
    for ( std::size_t index = 0U; index < result.diag.size(); ++index )
    {
      EXPECT_EQ( result.diag[ index ].status,
                 index < first_ok_index ? "initializing" : "ok" )
          << "diag_index=" << index;
    }

    const auto& poses      = result.trajectory->poses();
    std::size_t pose_index = 0U;
    for ( const auto& row : result.diag )
    {
      if ( row.status != "ok" )
      {
        continue;
      }
      ASSERT_LT( pose_index, poses.size() );
      EXPECT_EQ( poses[ pose_index ].timestamp.nanoseconds(),
                 row.timestamp_ns );
      EXPECT_TRUE( poses[ pose_index ].T_W_B.matrix().allFinite() );
      ++pose_index;
    }
    EXPECT_EQ( pose_index, poses.size() );
    EXPECT_EQ( poses.front().timestamp.nanoseconds(),
               first_ok->timestamp_ns );
    EXPECT_EQ( poses.back().timestamp.nanoseconds(),
               result.diag.back().timestamp_ns );
  }

}  // namespace
