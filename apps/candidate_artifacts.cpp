#include "apps/candidate_artifacts.hpp"

#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <string>

#include <nlohmann/json.hpp>

#include "apps/stereo_vo_glue.hpp"

namespace phad::apps
{

  namespace
  {

    /// 标准 CSV 字段转义：含逗号/引号/换行时双引号包裹，内部引号翻倍。
    [[nodiscard]] std::string csvEscape( const std::string& value )
    {
      if ( value.find_first_of( ",\"\n" ) == std::string::npos )
      {
        return value;
      }
      std::string escaped;
      escaped.reserve( value.size() + 2 );
      escaped.push_back( '"' );
      for ( const char c : value )
      {
        if ( c == '"' )
        {
          escaped.push_back( '"' );
        }
        escaped.push_back( c );
      }
      escaped.push_back( '"' );
      return escaped;
    }

  }  // namespace

  std::optional<SessionError> writeCandidateDiagCsv(
      const std::filesystem::path&                  path,
      const std::vector<CandidateFrameDiagnostics>& rows )
  {
    std::ofstream out( path );
    if ( !out )
    {
      return SessionError{ "failed to open candidate diag csv: " +
                           path.string() };
    }

    out << "frame_index,ts_ns,status,message,selected_keyframe,"
           "keyframe_rule,epoch_committed,keyframe_epoch,segment_id,"
           "reset_reason,track_count,observation_count,disparity_count,"
           "shared_count,landmark_count,culled_count,dropped_track_count,"
           "fusion_mode,state_valid,state_tx,state_ty,state_tz,"
           "state_qw,state_qx,state_qy,state_qz,"
           "bias_gyro_x,bias_gyro_y,bias_gyro_z,"
           "graph_valid,graph_active_pose_count,graph_active_factor_count,"
           "graph_marginalized_pose_count,graph_retired_landmark_count\n";

    for ( const CandidateFrameDiagnostics& row : rows )
    {
      out << row.frame_index << ',' << row.timestamp.nanoseconds() << ','
          << updateStatusName( row.status ) << ',' << csvEscape( row.message )
          << ',' << ( row.selected_keyframe ? 1 : 0 ) << ','
          << static_cast<int>( row.keyframe_rule ) << ','
          << ( row.epoch_committed ? 1 : 0 ) << ',' << row.keyframe_epoch
          << ',' << row.segment_id << ','
          << static_cast<int>( row.reset_reason ) << ',' << row.track_count
          << ',' << row.observation_count << ',' << row.disparity_count << ','
          << row.shared_count << ',' << row.landmark_count << ','
          << row.culled_count << ',' << row.dropped_track_count << ','
          << static_cast<int>( row.fusion_mode ) << ','
          << ( row.state.has_value() ? 1 : 0 );

      if ( row.state.has_value() )
      {
        const Eigen::Quaterniond quat( row.state->T_W_B.linear() );
        out << ',' << std::setprecision( 9 )
            << row.state->T_W_B.translation().x() << ','
            << row.state->T_W_B.translation().y() << ','
            << row.state->T_W_B.translation().z() << ',' << quat.w() << ','
            << quat.x() << ',' << quat.y() << ',' << quat.z() << ','
            << row.state->bias_gyro.x() << ',' << row.state->bias_gyro.y()
            << ',' << row.state->bias_gyro.z();
      }
      else
      {
        out << ",0,0,0,1,0,0,0,0,0,0";
      }

      out << ',' << ( row.graph.has_value() ? 1 : 0 );
      if ( row.graph.has_value() )
      {
        out << ',' << row.graph->active_pose_count << ','
            << row.graph->active_factor_count << ','
            << row.graph->marginalized_pose_count << ','
            << row.graph->retired_landmark_count;
      }
      else
      {
        out << ",0,0,0,0";
      }
      out << '\n';
    }

    if ( !out )
    {
      return SessionError{ "failed to write candidate diag csv: " +
                           path.string() };
    }
    return std::nullopt;
  }

  std::optional<SessionError> writeCandidateMeta(
      const std::filesystem::path& path, const CandidateRunResult& run,
      double candidate_wall_s )
  {
    nlohmann::json json;
    json[ "ownership" ]          = "full_pipeline";
    json[ "frames" ]             = run.counts.frames;
    json[ "ok" ]                 = run.counts.ok;
    json[ "rejected" ]           = run.counts.rejected;
    json[ "failed" ]             = run.counts.failed;
    json[ "keyframes" ]          = run.counts.keyframes;
    json[ "track_only_frames" ]  = run.counts.track_only_frames;
    json[ "terminal" ]           = run.error.has_value() ? 1 : 0;
    if ( run.error.has_value() )
    {
      json[ "terminal_code" ]   = static_cast<int>( run.error->code );
      json[ "terminal_detail" ] = run.error->detail;
    }
    json[ "wall_s" ] = candidate_wall_s;

    std::ofstream out( path );
    if ( !out )
    {
      return SessionError{ "failed to open candidate meta json: " +
                           path.string() };
    }
    out << json.dump() << '\n';
    if ( !out )
    {
      return SessionError{ "failed to write candidate meta json: " +
                           path.string() };
    }
    return std::nullopt;
  }

}  // namespace phad::apps
