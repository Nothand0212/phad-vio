#include "apps/probe_b_writer.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace phad::apps
{
  namespace
  {

    [[nodiscard]] std::string formatDouble( double value )
    {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision( 6 ) << value;
      return stream.str();
    }

    [[nodiscard]] std::string formatUint64Array(
        const std::vector<std::uint64_t>& values )
    {
      std::string out = "[";
      for ( std::size_t index = 0; index < values.size(); ++index )
      {
        if ( index > 0U )
        {
          out += ',';
        }
        out += std::to_string( values[ index ] );
      }
      out += ']';
      return out;
    }

    [[nodiscard]] std::string formatShiftTopArray(
        const std::vector<ProbeBShiftTop>& entries )
    {
      std::string out = "[";
      for ( std::size_t index = 0; index < entries.size(); ++index )
      {
        if ( index > 0U )
        {
          out += ',';
        }
        out += "{\"key\":" + std::to_string( entries[ index ].key ) + ",\"dt_m\":" + formatDouble( entries[ index ].dt_m ) + '}';
      }
      out += ']';
      return out;
    }

    [[nodiscard]] const char* pnpProbeResultName(
        const estimator::PnpProbeResult result )
    {
      switch ( result )
      {
        case estimator::PnpProbeResult::kNotAttempted:
          return "not_attempted";
        case estimator::PnpProbeResult::kRansacFailed:
          return "ransac_failed";
        case estimator::PnpProbeResult::kInsufficientInliers:
          return "insufficient_inliers";
        case estimator::PnpProbeResult::kProposalPoseNonFinite:
          return "proposal_pose_nonfinite";
        case estimator::PnpProbeResult::kProposalRmsInvalidCorrespondence:
          return "proposal_rms_invalid_correspondence";
        case estimator::PnpProbeResult::kProposalRmsCheirality:
          return "proposal_rms_cheirality";
        case estimator::PnpProbeResult::kProposalRmsNonFinite:
          return "proposal_rms_nonfinite";
        case estimator::PnpProbeResult::kProposalRmsRejected:
          return "proposal_rms_rejected";
        case estimator::PnpProbeResult::kAccepted:
          return "accepted";
        case estimator::PnpProbeResult::kAcceptedAfterGuessRetry:
          return "accepted_after_guess_retry";
      }
      throw std::logic_error( "unknown PnP probe result" );
    }

    [[nodiscard]] const char* pnpGuessRetryOutcomeName(
        const estimator::PnpGuessRetryOutcome outcome )
    {
      switch ( outcome )
      {
        case estimator::PnpGuessRetryOutcome::kNotAttempted:
          return "not_attempted";
        case estimator::PnpGuessRetryOutcome::kSolveFailed:
          return "solve_failed";
        case estimator::PnpGuessRetryOutcome::kProposalPoseNonFinite:
          return "proposal_pose_nonfinite";
        case estimator::PnpGuessRetryOutcome::kProposalRmsInvalidCorrespondence:
          return "proposal_rms_invalid_correspondence";
        case estimator::PnpGuessRetryOutcome::kProposalRmsCheirality:
          return "proposal_rms_cheirality";
        case estimator::PnpGuessRetryOutcome::kProposalRmsNonFinite:
          return "proposal_rms_nonfinite";
        case estimator::PnpGuessRetryOutcome::kProposalRmsRejected:
          return "proposal_rms_rejected";
        case estimator::PnpGuessRetryOutcome::kWouldPassExistingPnpGates:
          return "would_pass_existing_pnp_gates";
      }
      throw std::logic_error( "unknown PnP guess retry outcome" );
    }

    void appendCommaIfNeeded( std::string& json, bool& first )
    {
      if ( !first )
      {
        json += ',';
      }
      first = false;
    }

    [[nodiscard]] std::string serializeProbeBFrame( const ProbeBFrame& frame )
    {
      std::string json  = "{";
      bool        first = true;

      json += "\"i\":" + std::to_string( frame.i );
      first = false;

      appendCommaIfNeeded( json, first );
      json += "\"ts_ns\":" + std::to_string( frame.ts_ns );

      if ( frame.pnp_attempted.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_attempted\":" +
                std::string( *frame.pnp_attempted ? "true" : "false" );
      }
      if ( frame.pnp_result.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_result\":\"" +
                std::string( pnpProbeResultName( *frame.pnp_result ) ) + '"';
      }
      if ( frame.pnp_correspondences.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_correspondences\":" +
                std::to_string( *frame.pnp_correspondences );
      }
      if ( frame.pnp_inliers.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_inliers\":" + std::to_string( *frame.pnp_inliers );
      }
      if ( frame.pnp_proposal_rms_px.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_proposal_rms_px\":" +
                formatDouble( *frame.pnp_proposal_rms_px );
      }
      if ( frame.pnp_guess_rms_px.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_guess_rms_px\":" +
                formatDouble( *frame.pnp_guess_rms_px );
      }
      if ( frame.pnp_proposal_inlier_count.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_proposal_inlier_count\":" +
                std::to_string( *frame.pnp_proposal_inlier_count );
      }
      if ( frame.pnp_proposal_cheiral_inlier_count.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_proposal_cheiral_inlier_count\":" +
                std::to_string(
                    *frame.pnp_proposal_cheiral_inlier_count );
      }
      if ( frame.pnp_proposal_min_inlier_depth_m.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_proposal_min_inlier_depth_m\":" +
                formatDouble(
                    *frame.pnp_proposal_min_inlier_depth_m );
      }
      if ( frame.pnp_proposal_min_inlier_landmark_id.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_proposal_min_inlier_landmark_id\":" +
                std::to_string(
                    *frame.pnp_proposal_min_inlier_landmark_id );
      }
      if ( frame.pnp_proposal_guess_translation_delta_m.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_proposal_guess_translation_delta_m\":" +
                formatDouble(
                    *frame.pnp_proposal_guess_translation_delta_m );
      }
      if ( frame.pnp_proposal_guess_rotation_delta_rad.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_proposal_guess_rotation_delta_rad\":" +
                formatDouble(
                    *frame.pnp_proposal_guess_rotation_delta_rad );
      }
      if ( frame.pnp_guess_retry_outcome.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_guess_retry_outcome\":\"" +
                std::string( pnpGuessRetryOutcomeName(
                    *frame.pnp_guess_retry_outcome ) ) +
                '"';
      }
      if ( frame.pnp_guess_retry_proposal_rms_px.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_guess_retry_proposal_rms_px\":" +
                formatDouble( *frame.pnp_guess_retry_proposal_rms_px );
      }
      if ( frame.pnp_guess_retry_guess_rms_px.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_guess_retry_guess_rms_px\":" +
                formatDouble( *frame.pnp_guess_retry_guess_rms_px );
      }
      if ( frame.pnp_guess_retry_cheiral_inlier_count.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_guess_retry_cheiral_inlier_count\":" +
                std::to_string(
                    *frame.pnp_guess_retry_cheiral_inlier_count );
      }
      if ( frame.pnp_guess_retry_min_inlier_depth_m.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_guess_retry_min_inlier_depth_m\":" +
                formatDouble(
                    *frame.pnp_guess_retry_min_inlier_depth_m );
      }
      if ( frame.pnp_guess_retry_translation_delta_m.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_guess_retry_translation_delta_m\":" +
                formatDouble(
                    *frame.pnp_guess_retry_translation_delta_m );
      }
      if ( frame.pnp_guess_retry_rotation_delta_rad.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"pnp_guess_retry_rotation_delta_rad\":" +
                formatDouble(
                    *frame.pnp_guess_retry_rotation_delta_rad );
      }
      if ( frame.current_pose_translation_delta_m.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"current_pose_translation_delta_m\":" +
                formatDouble( *frame.current_pose_translation_delta_m );
      }
      if ( frame.current_pose_rotation_delta_rad.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"current_pose_rotation_delta_rad\":" +
                formatDouble( *frame.current_pose_rotation_delta_rad );
      }
      if ( frame.current_velocity_delta_mps.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"current_velocity_delta_mps\":" +
                formatDouble( *frame.current_velocity_delta_mps );
      }
      if ( frame.current_acc_bias_delta_mps2.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"current_acc_bias_delta_mps2\":" +
                formatDouble( *frame.current_acc_bias_delta_mps2 );
      }
      if ( frame.current_gyr_bias_delta_radps.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"current_gyr_bias_delta_radps\":" +
                formatDouble( *frame.current_gyr_bias_delta_radps );
      }
      if ( frame.primary_graph_error_seed.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"primary_graph_error_seed\":" +
                formatDouble( *frame.primary_graph_error_seed );
      }
      if ( frame.primary_graph_error_optimized.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"primary_graph_error_optimized\":" +
                formatDouble( *frame.primary_graph_error_optimized );
      }
      if ( frame.primary_imu_factor_error_seed.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"primary_imu_factor_error_seed\":" +
                formatDouble( *frame.primary_imu_factor_error_seed );
      }
      if ( frame.primary_imu_factor_error_optimized.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"primary_imu_factor_error_optimized\":" +
                formatDouble( *frame.primary_imu_factor_error_optimized );
      }

      if ( frame.culled_ids.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"culled_ids\":" + formatUint64Array( frame.culled_ids.value() );
      }
      if ( frame.zombie_track_n.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"zombie_track_n\":" + std::to_string( frame.zombie_track_n.value() );
      }
      if ( frame.rejected_block_n.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"rejected_block_n\":" + std::to_string( frame.rejected_block_n.value() );
      }
      if ( frame.new_lm.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"new_lm\":" + std::to_string( frame.new_lm.value() );
      }
      if ( frame.shared.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"shared\":" + std::to_string( frame.shared.value() );
      }
      if ( frame.num_obs.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"num_obs\":" + std::to_string( frame.num_obs.value() );
      }
      if ( frame.lm_iterations.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"lm_iterations\":" + std::to_string( frame.lm_iterations.value() );
      }
      if ( frame.shift_m.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"shift_m\":" + formatDouble( frame.shift_m.value() );
      }
      if ( frame.shift_top.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"shift_top\":" + formatShiftTopArray( frame.shift_top.value() );
      }
      if ( frame.res_mean_px.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"res_mean_px\":" + formatDouble( frame.res_mean_px.value() );
      }
      if ( frame.res_max_px.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"res_max_px\":" + formatDouble( frame.res_max_px.value() );
      }
      if ( frame.res_max_id.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"res_max_id\":" + std::to_string( frame.res_max_id.value() );
      }
      if ( frame.drops_skipped_this_frame.has_value() )
      {
        appendCommaIfNeeded( json, first );
        json += "\"drops_skipped_this_frame\":" + std::string( frame.drops_skipped_this_frame.value() ? "true"
                                                                                                      : "false" );
      }

      json += "}\n";
      return json;
    }

  }  // namespace

  ProbeBWriter::ProbeBWriter( const std::filesystem::path& path )
      : m_out( path, std::ios::out | std::ios::trunc )
  {
    if ( !m_out )
    {
      throw std::runtime_error(
          "failed to open probe_b jsonl: " + path.string() );
    }
  }

  ProbeBWriter::~ProbeBWriter()
  {
    if ( m_out.is_open() )
    {
      m_out.flush();
      m_out.close();
    }
  }

  void ProbeBWriter::write( const ProbeBFrame& frame )
  {
    m_out << serializeProbeBFrame( frame );
    if ( !m_out )
    {
      throw std::runtime_error( "failed to write probe_b jsonl line" );
    }
  }

}  // namespace phad::apps
