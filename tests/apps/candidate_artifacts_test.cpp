#include "apps/candidate_artifacts.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "apps/candidate_pipeline.hpp"
#include "phad/estimator/types.hpp"

namespace
{

  using phad::apps::CandidateError;
  using phad::apps::CandidateErrorCode;
  using phad::apps::CandidateFrameDiagnostics;
  using phad::apps::CandidateResetReason;
  using phad::apps::CandidateRunResult;
  using phad::apps::CandidateState;
  using phad::apps::KeyframeRule;
  using phad::apps::writeCandidateDiagCsv;
  using phad::apps::writeCandidateMeta;
  using phad::common::Timestamp;
  using phad::estimator::FusionMode;
  using phad::estimator::UpdateStatus;

  [[nodiscard]] std::string readText( const std::filesystem::path& path )
  {
    std::ifstream      in( path );
    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
  }

  [[nodiscard]] CandidateFrameDiagnostics makeRow( std::uint64_t  frame_index,
                                                    std::int64_t   ts_ns,
                                                    UpdateStatus   status,
                                                    std::string    message )
  {
    CandidateFrameDiagnostics row;
    row.frame_index  = frame_index;
    row.timestamp    = Timestamp{ ts_ns };
    row.status       = status;
    row.message      = std::move( message );
    row.segment_id   = 2;
    row.track_count  = 20;
    row.fusion_mode  = FusionMode::kGyroVisual;
    return row;
  }

  TEST( CandidateArtifactsTest, WriteDiagCsvMatchesStrictSchema )
  {
    CandidateFrameDiagnostics ok_row = makeRow( 0, 100, UpdateStatus::kOk,
                                                "accepted" );
    ok_row.selected_keyframe = true;
    ok_row.keyframe_rule     = KeyframeRule::kBootstrap;
    ok_row.epoch_committed   = true;
    ok_row.keyframe_epoch    = 1;
    ok_row.reset_reason      = CandidateResetReason::kBootstrap;
    ok_row.observation_count = 20;
    ok_row.disparity_count   = 18;
    ok_row.shared_count      = 16;
    ok_row.landmark_count    = 15;
    ok_row.culled_count      = 2;
    ok_row.dropped_track_count = 2;
    CandidateState state;
    state.T_W_B = Eigen::Isometry3d( Eigen::Translation3d( 0.1, 0.2, 0.3 ) );
    state.T_W_B.linear() =
        Eigen::AngleAxisd( 0.5, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
    state.bias_gyro = Eigen::Vector3d( 0.1, -0.2, 0.3 );
    ok_row.state    = state;

    // message 含逗号 → CSV 引号转义路径。
    CandidateFrameDiagnostics fail_row =
        makeRow( 1, 200, UpdateStatus::kFailed, "boom, with \"quote\"" );

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "phad_candidate_diag.csv";
    std::filesystem::remove( path );

    const std::vector<CandidateFrameDiagnostics> rows{ ok_row, fail_row };
    ASSERT_FALSE( writeCandidateDiagCsv( path, rows ).has_value() );

    const std::string text = readText( path );
    EXPECT_NE( text.find(
                   "frame_index,ts_ns,status,message,selected_keyframe," ),
               std::string::npos );
    EXPECT_NE( text.find( "state_valid,state_tx," ), std::string::npos );
    EXPECT_NE( text.find( "graph_valid,graph_active_pose_count," ),
               std::string::npos );

    // ok 行：全部数值列 + state_valid=1；graph 无值 → graph_valid=0。
    EXPECT_NE( text.find( "0,100,ok,accepted,1,1,1,1,2,1,20,20,18,16,"
                          "15,2,2,1,1,0.1,0.2,0.3," ),
               std::string::npos )
        << text;
    // fail 行：state_valid=0；message 引号转义。
    EXPECT_NE( text.find( "1,200,failed,\"boom, with \"\"quote\"\"\",0,0," ),
               std::string::npos );

    std::filesystem::remove( path );
  }

  TEST( CandidateArtifactsTest, WriteMetaRecordsFullPipelineOwnership )
  {
    CandidateRunResult run;
    run.counts.frames            = 5;
    run.counts.ok                = 4;
    run.counts.rejected          = 0;
    run.counts.failed            = 1;
    run.counts.keyframes         = 2;
    run.counts.track_only_frames = 3;

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "phad_candidate_meta.json";
    std::filesystem::remove( path );

    ASSERT_FALSE( writeCandidateMeta( path, run, 0.125 ).has_value() );
    const std::string text = readText( path );
    EXPECT_NE( text.find( "\"ownership\":\"full_pipeline\"" ),
               std::string::npos );
    EXPECT_NE( text.find( "\"frames\":5" ), std::string::npos );
    EXPECT_NE( text.find( "\"failed\":1" ), std::string::npos );
    EXPECT_NE( text.find( "\"terminal\":0" ), std::string::npos );
    EXPECT_NE( text.find( "\"wall_s\":0.125" ), std::string::npos );

    // terminal error 变体：terminal=1 + code/detail。
    run.error = CandidateError{
        .code      = CandidateErrorCode::kInputContract,
        .frame_index = std::nullopt,
        .timestamp = Timestamp{ 200 },
        .detail    = "non-gap IMU interval must contain at least two samples",
    };
    ASSERT_FALSE( writeCandidateMeta( path, run, 0.2 ).has_value() );
    const std::string terminal_text = readText( path );
    EXPECT_NE( terminal_text.find( "\"terminal\":1" ),
               std::string::npos );
    EXPECT_NE( terminal_text.find( "\"terminal_code\":1" ),
               std::string::npos );
    EXPECT_NE( terminal_text.find( "\"terminal_detail\":\"non-gap" ),
               std::string::npos );

    std::filesystem::remove( path );
  }

  TEST( CandidateArtifactsTest, WriteDiagCsvFailsOnUnwritablePath )
  {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "phad_missing_parent" /
        "nested" / "candidate_diag.csv";
    std::filesystem::remove_all( path.parent_path().parent_path() );

    const auto error = writeCandidateDiagCsv( path, {} );
    ASSERT_TRUE( error.has_value() );
    EXPECT_NE( error->detail.find( "candidate diag" ), std::string::npos );
  }

}  // namespace
