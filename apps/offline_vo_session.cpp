#include "apps/offline_vo_session.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
#include <openssl/evp.h>

#include <nlohmann/json.hpp>

#include "apps/internal/m5_51_prefix_diagnostic.hpp"
#include "apps/internal/stereo_pair_stream_test_access.hpp"
#include "phad/estimator/internal/vio_estimator_test_access.hpp"
#include "phad/frontend/internal/stereo_tracker_test_access.hpp"
#endif

#include "apps/internal/offline_vo_session_keyframe.hpp"
#include "apps/probe_b_writer.hpp"
#include "apps/stereo_pair_stream.hpp"
#include "apps/stereo_vo_glue.hpp"
#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/camera/stereo_rectifier.hpp"
#include "phad/common/landmark_id.hpp"
#include "phad/estimator/vio_estimator.hpp"
#include "phad/io/dataset/dataset_replay_source.hpp"
#include "phad/sensor/stereo_frame.hpp"

/**
 * @file offline_vo_session.cpp
 * @brief 离线 VO session：只跑 pipeline，错误经 result 返回。
 */

namespace phad::apps
{
  void internal::recordAcceptedSegment(
      const std::uint32_t           segment_id,
      std::optional<std::uint32_t>& last_segment_id,
      FrameCounts&                  counts )
  {
    const bool is_new_segment =
        !last_segment_id.has_value() || segment_id > *last_segment_id;
    if ( is_new_segment )
    {
      ++counts.segments;
      if ( last_segment_id.has_value() )
      {
        ++counts.reanchors;
      }
    }
    last_segment_id = segment_id;
  }

  void internal::appendSegmentSummaryWarning(
      const FrameCounts& counts, std::vector<std::string>& warnings )
  {
    if ( counts.reanchors == 0U && counts.seed_rejected == 0U )
    {
      return;
    }

    warnings.push_back(
        "vo segments summary: segments=" + std::to_string( counts.segments ) +
        " reanchors=" + std::to_string( counts.reanchors ) +
        " seed_rejected=" + std::to_string( counts.seed_rejected ) );
  }

  namespace
  {
    constexpr std::int64_t kImuContinuityLimitNs = 20'000'000;

    [[nodiscard]] std::optional<SessionError> gyroObserveError(
        std::string_view code )
    {
      return SessionError{ "gyro observe:" + std::string{ code } };
    }

    [[nodiscard]] bool checkedSubtract( std::int64_t  left,
                                        std::int64_t  right,
                                        std::int64_t& result )
    {
      constexpr std::int64_t kMin = std::numeric_limits<std::int64_t>::min();
      constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();
      if ( ( right > 0 && left < kMin + right ) ||
           ( right < 0 && left > kMax + right ) )
      {
        return false;
      }
      result = left - right;
      return true;
    }

    [[nodiscard]] bool checkedAdd( std::int64_t  left,
                                   std::int64_t  right,
                                   std::int64_t& result )
    {
      constexpr std::int64_t kMin = std::numeric_limits<std::int64_t>::min();
      constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();
      if ( ( right > 0 && left > kMax - right ) ||
           ( right < 0 && left < kMin - right ) )
      {
        return false;
      }
      result = left + right;
      return true;
    }

    [[nodiscard]] bool sizeToUint64( std::size_t    size,
                                     std::uint64_t& value )
    {
      if constexpr ( sizeof( std::size_t ) > sizeof( std::uint64_t ) )
      {
        if ( size > std::numeric_limits<std::uint64_t>::max() )
        {
          return false;
        }
      }
      value = static_cast<std::uint64_t>( size );
      return true;
    }

    [[nodiscard]] std::optional<SessionError> validateGyroPacketRow(
        const GyroPacketRow&           packet,
        std::span<const GyroSampleRow> samples )
    {
      std::uint64_t sample_count = 0;
      if ( !sizeToUint64( samples.size(), sample_count ) ||
           sample_count != packet.sample_count )
      {
        return gyroObserveError( "sample_count_mismatch" );
      }

      std::int64_t interval_ns = 0;
      if ( !checkedSubtract( packet.t_cur_ns, packet.t_prev_ns,
                             interval_ns ) )
      {
        return gyroObserveError( "interval_overflow" );
      }
      if ( packet.interval_ns != interval_ns )
      {
        return gyroObserveError( "interval_mismatch" );
      }

      std::int64_t sum_dt_ns = 0;
      for ( std::size_t index = 0; index < samples.size(); ++index )
      {
        const GyroSampleRow& sample = samples[ index ];
        if ( sample.packet_index != packet.packet_index ||
             sample.sample_index != static_cast<std::uint64_t>( index ) )
        {
          return gyroObserveError( "sample_index_mismatch" );
        }
        if ( !std::isfinite( sample.gyr_x_radps ) ||
             !std::isfinite( sample.gyr_y_radps ) ||
             !std::isfinite( sample.gyr_z_radps ) )
        {
          return gyroObserveError( "sample_nonfinite" );
        }
        if ( index == 0U )
        {
          continue;
        }
        const std::int64_t previous_ns = samples[ index - 1U ].timestamp_ns;
        if ( sample.timestamp_ns == previous_ns )
        {
          return gyroObserveError( "sample_duplicate" );
        }
        if ( sample.timestamp_ns < previous_ns )
        {
          return gyroObserveError( "sample_out_of_order" );
        }
        std::int64_t dt_ns = 0;
        if ( !checkedSubtract( sample.timestamp_ns, previous_ns, dt_ns ) )
        {
          return gyroObserveError( "sample_delta_overflow" );
        }
        if ( !checkedAdd( sum_dt_ns, dt_ns, sum_dt_ns ) )
        {
          return gyroObserveError( "sum_dt_overflow" );
        }
      }
      if ( packet.sum_dt_ns != sum_dt_ns )
      {
        return gyroObserveError( "interval_unclosed" );
      }

      if ( interval_ns <= 0 )
      {
        return gyroObserveError( "interval_nonpositive" );
      }
      GyroPacketStatus expected;
      if ( packet.imu_gap )
      {
        expected = GyroPacketStatus::kGap;
      }
      else if ( samples.empty() )
      {
        expected = GyroPacketStatus::kEmptyNonfirst;
      }
      else
      {
        expected = GyroPacketStatus::kValid;
      }
      if ( packet.status != expected )
      {
        return gyroObserveError( "status_mismatch" );
      }
      return std::nullopt;
    }

    // Must match phad::estimator::VioEstimator::update reject messages.
    constexpr std::string_view kSeedRejectedNewSegment =
        "insufficient observations to seed new segment";
    constexpr std::string_view kSeedRejectedFirstSegment =
        "insufficient observations to seed first segment";


    [[nodiscard]] double percentile( std::vector<double> values, double q )
    {
      if ( values.empty() )
      {
        return 0.0;
      }
      std::sort( values.begin(), values.end() );
      const double position = q * static_cast<double>( values.size() - 1U );
      const auto   lower    = static_cast<std::size_t>( std::floor( position ) );
      const auto   upper    = static_cast<std::size_t>( std::ceil( position ) );
      if ( lower == upper )
      {
        return values[ lower ];
      }
      const double weight = position - static_cast<double>( lower );
      return values[ lower ] * ( 1.0 - weight ) + values[ upper ] * weight;
    }

    [[nodiscard]] StageTiming summarizeStage(
        std::vector<double> samples )
    {
      StageTiming timing;
      if ( samples.empty() )
      {
        return timing;
      }
      double sum = 0.0;
      double max = samples.front();
      for ( const double sample : samples )
      {
        sum += sample;
        max = std::max( max, sample );
      }
      timing.mean = sum / static_cast<double>( samples.size() );
      timing.p95  = percentile( samples, 0.95 );
      timing.max  = max;
      return timing;
    }

  }  // namespace

  namespace
  {
    struct M5RpeDiagnosticRun
    {
#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
      internal::M5RpePrefixArm                arm;
      internal::M5RpePrefixDiagnosticReceipt* receipt = nullptr;
#endif
    };

#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
    using DiagnosticJson = nlohmann::ordered_json;

    [[nodiscard]] std::string fixedHex( std::uint64_t value,
                                        std::size_t   digits )
    {
      constexpr char kHex[] = "0123456789abcdef";
      std::string    encoded( digits, '0' );
      for ( std::size_t index = 0; index < digits; ++index )
      {
        encoded[ digits - index - 1U ] = kHex[ value & 0x0fU ];
        value >>= 4U;
      }
      return encoded;
    }

    [[nodiscard]] std::string doubleBits( const double value )
    {
      return fixedHex( std::bit_cast<std::uint64_t>( value ), 16U );
    }

    [[nodiscard]] std::string sha256( const std::string_view bytes )
    {
      std::unique_ptr<EVP_MD_CTX, decltype( &EVP_MD_CTX_free )> context(
          EVP_MD_CTX_new(), &EVP_MD_CTX_free );
      if ( !context || EVP_DigestInit_ex( context.get(), EVP_sha256(), nullptr ) != 1 ||
           EVP_DigestUpdate( context.get(), bytes.data(), bytes.size() ) != 1 )
      {
        throw std::runtime_error( "M5 #51 state SHA-256 initialization failed" );
      }
      unsigned int                               digest_size = 0U;
      std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
      if ( EVP_DigestFinal_ex( context.get(), digest.data(), &digest_size ) != 1 ||
           digest_size != 32U )
      {
        throw std::runtime_error( "M5 #51 state SHA-256 finalization failed" );
      }
      std::string encoded;
      encoded.reserve( digest_size * 2U );
      for ( unsigned int index = 0U; index < digest_size; ++index )
      {
        encoded += fixedHex( digest[ index ], 2U );
      }
      return encoded;
    }

    [[nodiscard]] DiagnosticJson vector3Json( const Eigen::Vector3d& value )
    {
      return DiagnosticJson::array(
          { doubleBits( value.x() ), doubleBits( value.y() ),
            doubleBits( value.z() ) } );
    }

    [[nodiscard]] DiagnosticJson matrix3Json( const Eigen::Matrix3d& value )
    {
      DiagnosticJson encoded = DiagnosticJson::array();
      for ( Eigen::Index row = 0; row < value.rows(); ++row )
      {
        DiagnosticJson encoded_row = DiagnosticJson::array();
        for ( Eigen::Index column = 0; column < value.cols(); ++column )
        {
          encoded_row.push_back( doubleBits( value( row, column ) ) );
        }
        encoded.push_back( std::move( encoded_row ) );
      }
      return encoded;
    }

    [[nodiscard]] DiagnosticJson poseJson( const Eigen::Isometry3d& value )
    {
      DiagnosticJson encoded;
      encoded[ "rotation" ]    = matrix3Json( value.linear() );
      encoded[ "translation" ] = vector3Json( value.translation() );
      return encoded;
    }

    template <typename Enum>
    [[nodiscard]] std::uint8_t enumValue( const Enum value )
    {
      return static_cast<std::uint8_t>( value );
    }

    [[nodiscard]] DiagnosticJson initializationJson(
        const estimator::InitializationDiagnostics& value )
    {
      const auto count = []( const estimator::InitializationCountSummary& item ) {
        return DiagnosticJson{ { "actual", item.m_actual },
                               { "required", item.m_required } };
      };
      const auto rank = []( const estimator::InitializationRankSummary& item ) {
        return DiagnosticJson{ { "actual", item.m_actual },
                               { "required", item.m_required } };
      };
      const auto fit = []( const estimator::InitializationFitSummary& item ) {
        return DiagnosticJson{ { "statistic", doubleBits( item.m_statistic ) },
                               { "limit", doubleBits( item.m_limit ) },
                               { "effective_dof", item.m_effective_dof } };
      };
      const auto threshold = [](
                                 const estimator::InitializationThresholdSummary& item ) {
        return DiagnosticJson{ { "value", doubleBits( item.m_value ) },
                               { "limit", doubleBits( item.m_limit ) } };
      };
      DiagnosticJson encoded;
      encoded[ "path" ]            = enumValue( value.m_path );
      encoded[ "phase" ]           = enumValue( value.m_phase );
      encoded[ "reason" ]          = enumValue( value.m_reason );
      encoded[ "outcome" ]         = enumValue( value.m_outcome );
      encoded[ "attempt_id" ]      = value.m_attempt_id.has_value()
                                         ? DiagnosticJson{ *value.m_attempt_id }
                                         : DiagnosticJson{ nullptr };
      encoded[ "connected_poses" ] = count( value.m_connected_poses );
      encoded[ "duration" ]        = DiagnosticJson{
                 { "actual_ns", value.m_duration.m_actual_ns },
                 { "required_ns", value.m_duration.m_required_ns } };
      encoded[ "connected_landmarks" ] = count( value.m_connected_landmarks );
      encoded[ "visual_support" ]      = value.m_visual_support.has_value()
                                             ? count( *value.m_visual_support )
                                             : DiagnosticJson{ nullptr };
      encoded[ "visual_fit" ]          = value.m_visual_fit.has_value()
                                             ? fit( *value.m_visual_fit )
                                             : DiagnosticJson{ nullptr };
      encoded[ "gyro_rank" ]           = value.m_gyro_rank.has_value()
                                             ? rank( *value.m_gyro_rank )
                                             : DiagnosticJson{ nullptr };
      encoded[ "gyro_normalized_condition" ] =
          value.m_gyro_normalized_condition.has_value()
              ? threshold( *value.m_gyro_normalized_condition )
              : DiagnosticJson{ nullptr };
      encoded[ "gravity_velocity_rank" ] =
          value.m_gravity_velocity_rank.has_value()
              ? rank( *value.m_gravity_velocity_rank )
              : DiagnosticJson{ nullptr };
      encoded[ "gravity_velocity_normalized_condition" ] =
          value.m_gravity_velocity_normalized_condition.has_value()
              ? threshold( *value.m_gravity_velocity_normalized_condition )
              : DiagnosticJson{ nullptr };
      encoded[ "joint_fit" ]     = value.m_joint_fit.has_value()
                                       ? fit( *value.m_joint_fit )
                                       : DiagnosticJson{ nullptr };
      encoded[ "current_graph" ] = enumValue( value.m_current_graph );
      encoded[ "transaction" ]   = enumValue( value.m_transaction );
      return encoded;
    }

    [[nodiscard]] DiagnosticJson selectorJson(
        const internal::KeyframeSelectorState& state )
    {
      std::vector<std::pair<common::LandmarkId, Eigen::Vector2d>> pixels(
          state.m_last_kf_pixels.begin(), state.m_last_kf_pixels.end() );
      std::sort( pixels.begin(), pixels.end(), []( const auto& lhs, const auto& rhs ) {
        return lhs.first < rhs.first;
      } );
      DiagnosticJson encoded_pixels = DiagnosticJson::array();
      for ( const auto& [ id, pixel ] : pixels )
      {
        encoded_pixels.push_back( DiagnosticJson{
            { "id", id },
            { "u", doubleBits( pixel.x() ) },
            { "v", doubleBits( pixel.y() ) } } );
      }
      DiagnosticJson encoded;
      encoded[ "last_kf_pixels" ] = std::move( encoded_pixels );
      encoded[ "last_kf_timestamp_ns" ] =
          state.m_last_kf_timestamp.nanoseconds();
      encoded[ "total_keyframes" ] = state.m_total_keyframes;
      encoded[ "last_accepted_rotation" ] =
          matrix3Json( state.m_last_accepted_rotation );
      encoded[ "last_kf_rotation" ] = matrix3Json( state.m_last_kf_rotation );
      return encoded;
    }

    [[nodiscard]] DiagnosticJson frameCountsJson( const FrameCounts& value )
    {
      return DiagnosticJson{
          { "image_frames", value.image_frames },
          { "ok", value.ok },
          { "rejected", value.rejected },
          { "failed", value.failed },
          { "low_connectivity", value.low_connectivity },
          { "segments", value.segments },
          { "reanchors", value.reanchors },
          { "seed_rejected", value.seed_rejected },
          { "pnp_successes", value.pnp_successes },
          { "pnp_fallbacks", value.pnp_fallbacks },
          { "outliers_culled", value.outliers_culled },
          { "outliers_culled_unique", value.outliers_culled_unique },
          { "outlier_reopts", value.outlier_reopts },
          { "drops_skipped", value.drops_skipped },
          { "deferred_drops", value.deferred_drops },
          { "deferred_drop_ids", value.deferred_drop_ids },
          { "evictable_marked", value.evictable_marked },
          { "tracks_evicted", value.tracks_evicted },
          { "zombie_age_drops", value.zombie_age_drops },
          { "zombie_age_drop_ids", value.zombie_age_drop_ids },
          { "total_keyframes", value.total_keyframes },
          { "total_track_only_frames", value.total_track_only_frames },
          { "non_keyframe_evictions", value.non_keyframe_evictions },
          { "imu_reintegrations", value.imu_reintegrations } };
    }

    [[nodiscard]] DiagnosticJson vioDiagnosticsJson(
        const estimator::VioDiagnostics& value )
    {
      DiagnosticJson encoded;
      encoded[ "nav_states" ]               = value.m_nav_states;
      encoded[ "imu_factors" ]              = value.m_imu_factors;
      encoded[ "bias_rw_factors" ]          = value.m_bias_rw_factors;
      encoded[ "visual_factors" ]           = value.m_visual_factors;
      encoded[ "root_prior_sets" ]          = value.m_root_prior_sets;
      encoded[ "integration_steps" ]        = value.m_integration_steps;
      encoded[ "integrated_duration_ns" ]   = value.m_integrated_duration_ns;
      encoded[ "visual_coast_duration_ns" ] = value.m_visual_coast_duration_ns;
      encoded[ "non_keyframe_evictions" ]   = value.m_non_keyframe_evictions;
      encoded[ "imu_reintegrations" ]       = value.m_imu_reintegrations;
      encoded[ "acc_cov_diag" ]             = vector3Json( value.m_acc_cov_diag );
      encoded[ "gyr_cov_diag" ]             = vector3Json( value.m_gyr_cov_diag );
      encoded[ "integration_cov_diag" ] =
          vector3Json( value.m_integration_cov_diag );
      DiagnosticJson bias_rw = DiagnosticJson::array();
      for ( Eigen::Index index = 0; index < value.m_bias_rw_sigmas.rows();
            ++index )
      {
        bias_rw.push_back( doubleBits( value.m_bias_rw_sigmas( index ) ) );
      }
      encoded[ "bias_rw_sigmas" ]       = std::move( bias_rw );
      encoded[ "completed_segment_id" ] = value.m_completed_segment_id.has_value()
                                              ? DiagnosticJson{ *value.m_completed_segment_id }
                                              : DiagnosticJson{ nullptr };
      return encoded;
    }

    [[nodiscard]] DiagnosticJson optionalDoubleJson(
        const std::optional<double>& value )
    {
      return value.has_value() ? DiagnosticJson{ doubleBits( *value ) }
                               : DiagnosticJson{ nullptr };
    }

    template <typename Integer>
    [[nodiscard]] DiagnosticJson optionalIntegerJson(
        const std::optional<Integer>& value )
    {
      return value.has_value() ? DiagnosticJson{ *value }
                               : DiagnosticJson{ nullptr };
    }

    [[nodiscard]] DiagnosticJson updateDiagnosticsJson(
        const estimator::UpdateDiagnostics& value )
    {
      DiagnosticJson encoded;
      encoded[ "num_observations" ]           = value.num_observations;
      encoded[ "num_retained_observations" ]  = value.num_retained_observations;
      encoded[ "num_seeded_landmarks" ]       = value.num_seeded_landmarks;
      encoded[ "num_current_visual_factors" ] = value.num_current_visual_factors;
      encoded[ "num_current_mono_visual_factors" ] =
          value.num_current_mono_visual_factors;
      encoded[ "unsupported_span_ns" ]     = value.unsupported_span_ns;
      encoded[ "num_landmarks" ]           = value.num_landmarks;
      encoded[ "num_shared" ]              = value.num_shared;
      encoded[ "num_mapped_observations" ] = value.num_mapped_observations;
      encoded[ "num_disparity" ]           = value.num_disparity;
      encoded[ "num_cheirality" ]          = value.num_cheirality;
      encoded[ "lm_iterations" ]           = value.lm_iterations;
      encoded[ "window_size" ]             = value.window_size;
      encoded[ "segment_id" ]              = value.segment_id;
      encoded[ "prior_key" ]               = value.prior_key;
      encoded[ "reproj_rms_before_px" ]    = doubleBits( value.reproj_rms_before_px );
      encoded[ "reproj_rms_after_px" ]     = doubleBits( value.reproj_rms_after_px );
      encoded[ "max_window_pose_shift_m" ] =
          doubleBits( value.max_window_pose_shift_m );
      encoded[ "low_connectivity" ]       = value.low_connectivity;
      encoded[ "pnp_success" ]            = value.pnp_success;
      encoded[ "pnp_inliers" ]            = value.pnp_inliers;
      encoded[ "outliers_culled" ]        = value.outliers_culled;
      encoded[ "outliers_culled_unique" ] = value.outliers_culled_unique;
      encoded[ "reproj_rms_after_cull_px" ] =
          doubleBits( value.reproj_rms_after_cull_px );
      encoded[ "outlier_reopt" ]             = value.outlier_reopt;
      encoded[ "outlier_reopt_failed" ]      = value.outlier_reopt_failed;
      encoded[ "outlier_reopt_rounds" ]      = value.outlier_reopt_rounds;
      std::vector<common::LandmarkId> culled = value.culled_landmark_ids;
      std::sort( culled.begin(), culled.end() );
      encoded[ "culled_landmark_ids" ]       = std::move( culled );
      encoded[ "probe_rejected_block_n" ]    = value.probe_rejected_block_n;
      encoded[ "probe_new_lm_n" ]            = value.probe_new_lm_n;
      encoded[ "probe_pnp_attempted" ]       = value.probe_pnp_attempted;
      encoded[ "probe_pnp_result" ]          = enumValue( value.probe_pnp_result );
      encoded[ "probe_pnp_correspondences" ] = value.probe_pnp_correspondences;
      encoded[ "probe_pnp_inliers" ]         = value.probe_pnp_inliers;
      encoded[ "probe_pnp_proposal_rms_px" ] =
          optionalDoubleJson( value.probe_pnp_proposal_rms_px );
      encoded[ "probe_pnp_guess_rms_px" ] =
          optionalDoubleJson( value.probe_pnp_guess_rms_px );
      encoded[ "probe_pnp_proposal_inlier_count" ] =
          optionalIntegerJson( value.probe_pnp_proposal_inlier_count );
      encoded[ "probe_pnp_proposal_cheiral_inlier_count" ] =
          optionalIntegerJson( value.probe_pnp_proposal_cheiral_inlier_count );
      encoded[ "probe_pnp_proposal_min_inlier_depth_m" ] =
          optionalDoubleJson( value.probe_pnp_proposal_min_inlier_depth_m );
      encoded[ "probe_pnp_proposal_min_inlier_landmark_id" ] =
          optionalIntegerJson( value.probe_pnp_proposal_min_inlier_landmark_id );
      encoded[ "probe_pnp_proposal_guess_translation_delta_m" ] =
          optionalDoubleJson( value.probe_pnp_proposal_guess_translation_delta_m );
      encoded[ "probe_pnp_proposal_guess_rotation_delta_rad" ] =
          optionalDoubleJson( value.probe_pnp_proposal_guess_rotation_delta_rad );
      encoded[ "probe_pnp_guess_retry_outcome" ] =
          enumValue( value.probe_pnp_guess_retry_outcome );
      encoded[ "probe_pnp_guess_retry_proposal_rms_px" ] =
          optionalDoubleJson( value.probe_pnp_guess_retry_proposal_rms_px );
      encoded[ "probe_pnp_guess_retry_guess_rms_px" ] =
          optionalDoubleJson( value.probe_pnp_guess_retry_guess_rms_px );
      encoded[ "probe_pnp_guess_retry_cheiral_inlier_count" ] =
          optionalIntegerJson( value.probe_pnp_guess_retry_cheiral_inlier_count );
      encoded[ "probe_pnp_guess_retry_min_inlier_depth_m" ] =
          optionalDoubleJson( value.probe_pnp_guess_retry_min_inlier_depth_m );
      encoded[ "probe_pnp_guess_retry_translation_delta_m" ] =
          optionalDoubleJson( value.probe_pnp_guess_retry_translation_delta_m );
      encoded[ "probe_pnp_guess_retry_rotation_delta_rad" ] =
          optionalDoubleJson( value.probe_pnp_guess_retry_rotation_delta_rad );
      encoded[ "probe_current_pose_translation_delta_m" ] =
          optionalDoubleJson( value.probe_current_pose_translation_delta_m );
      encoded[ "probe_current_pose_rotation_delta_rad" ] =
          optionalDoubleJson( value.probe_current_pose_rotation_delta_rad );
      encoded[ "probe_current_velocity_delta_mps" ] =
          optionalDoubleJson( value.probe_current_velocity_delta_mps );
      encoded[ "probe_current_acc_bias_delta_mps2" ] =
          optionalDoubleJson( value.probe_current_acc_bias_delta_mps2 );
      encoded[ "probe_current_gyr_bias_delta_radps" ] =
          optionalDoubleJson( value.probe_current_gyr_bias_delta_radps );
      encoded[ "probe_primary_graph_error_seed" ] =
          optionalDoubleJson( value.probe_primary_graph_error_seed );
      encoded[ "probe_primary_graph_error_optimized" ] =
          optionalDoubleJson( value.probe_primary_graph_error_optimized );
      encoded[ "probe_primary_imu_factor_error_seed" ] =
          optionalDoubleJson( value.probe_primary_imu_factor_error_seed );
      encoded[ "probe_primary_imu_factor_error_optimized" ] =
          optionalDoubleJson( value.probe_primary_imu_factor_error_optimized );
      DiagnosticJson shift_top = DiagnosticJson::array();
      for ( const auto& [ key, delta ] : value.probe_shift_top )
      {
        shift_top.push_back( DiagnosticJson{
            { "key", key }, { "translation_delta", doubleBits( delta ) } } );
      }
      encoded[ "probe_shift_top" ]    = std::move( shift_top );
      encoded[ "probe_res_mean_px" ]  = doubleBits( value.probe_res_mean_px );
      encoded[ "probe_res_max_px" ]   = doubleBits( value.probe_res_max_px );
      encoded[ "probe_res_max_id" ]   = value.probe_res_max_id;
      encoded[ "probe_detail_valid" ] = value.probe_detail_valid;
      encoded[ "vio" ]                = vioDiagnosticsJson( value.m_vio );
      encoded[ "initialization" ]     = initializationJson( value.m_initialization );
      return encoded;
    }

    [[nodiscard]] DiagnosticJson estimateJson(
        const estimator::VioEstimate& value )
    {
      DiagnosticJson encoded;
      encoded[ "timestamp_ns" ] = value.timestamp.nanoseconds();
      encoded[ "T_W_B" ]        = poseJson( value.T_W_B );
      encoded[ "v_W_B" ]        = vector3Json( value.m_v_W_B );
      encoded[ "acc_bias" ]     = vector3Json( value.m_bias.m_acc_mps2 );
      encoded[ "gyr_bias" ]     = vector3Json( value.m_bias.m_gyr_radps );
      encoded[ "segment_id" ]   = value.m_segment_id;
      return encoded;
    }

    [[nodiscard]] DiagnosticJson updateJson(
        const estimator::VioUpdateResult& value )
    {
      DiagnosticJson encoded;
      encoded[ "status" ]      = enumValue( value.status );
      encoded[ "estimate" ]    = value.estimate.has_value()
                                     ? estimateJson( *value.estimate )
                                     : DiagnosticJson{ nullptr };
      encoded[ "diagnostics" ] = updateDiagnosticsJson( value.diagnostics );
      encoded[ "message" ]     = value.message;
      return encoded;
    }

    [[nodiscard]] DiagnosticJson frameTracksJson(
        const frontend::FrameTracks& value )
    {
      DiagnosticJson observations = DiagnosticJson::array();
      for ( const auto& observation : value.observations )
      {
        observations.push_back( DiagnosticJson{
            { "id", observation.id },
            { "left_u", doubleBits( observation.left_pixel.x() ) },
            { "left_v", doubleBits( observation.left_pixel.y() ) },
            { "disparity_px", doubleBits( observation.disparity_px ) },
            { "status", enumValue( observation.status ) },
            { "length", observation.length } } );
      }
      DiagnosticJson stats{
          { "tracked", value.stats.tracked },
          { "detected", value.stats.detected },
          { "forward_backward_rejected", value.stats.forward_backward_rejected },
          { "epipolar_rejected", value.stats.epipolar_rejected },
          { "disparity_rejected", value.stats.disparity_rejected },
          { "depth_rejected", value.stats.depth_rejected },
          { "epipolar_median_px", doubleBits( value.stats.epipolar_median_px ) },
          { "epipolar_p95_px", doubleBits( value.stats.epipolar_p95_px ) },
          { "track_length_median", value.stats.track_length_median },
          { "track_length_max", value.stats.track_length_max },
          { "evicted", value.stats.evicted } };
      return DiagnosticJson{
          { "timestamp_ns", value.timestamp.nanoseconds() },
          { "observations", std::move( observations ) },
          { "stats", std::move( stats ) } };
    }

    [[nodiscard]] DiagnosticJson timedPoseJson(
        const common::TimedPose& value )
    {
      return DiagnosticJson{
          { "timestamp_ns", value.timestamp.nanoseconds() },
          { "T_W_B", poseJson( value.T_W_B ) } };
    }

    [[nodiscard]] DiagnosticJson diagRowJson( const VoDiagRow& value )
    {
      DiagnosticJson encoded;
      encoded[ "timestamp_ns" ]         = value.timestamp_ns;
      encoded[ "status" ]               = value.status;
      encoded[ "num_observations" ]     = value.num_observations;
      encoded[ "num_landmarks" ]        = value.num_landmarks;
      encoded[ "num_shared" ]           = value.num_shared;
      encoded[ "num_disparity" ]        = value.num_disparity;
      encoded[ "low_connectivity" ]     = value.low_connectivity;
      encoded[ "window_size" ]          = value.window_size;
      encoded[ "prior_key" ]            = value.prior_key;
      encoded[ "reproj_rms_before_px" ] = doubleBits( value.reproj_rms_before_px );
      encoded[ "reproj_rms_after_px" ]  = doubleBits( value.reproj_rms_after_px );
      encoded[ "num_cheirality" ]       = value.num_cheirality;
      encoded[ "lm_iterations" ]        = value.lm_iterations;
      encoded[ "max_window_pose_shift_m" ] =
          doubleBits( value.max_window_pose_shift_m );
      encoded[ "segment_id" ]      = value.segment_id;
      encoded[ "pnp_success" ]     = value.pnp_success;
      encoded[ "pnp_inliers" ]     = value.pnp_inliers;
      encoded[ "outliers_culled" ] = value.outliers_culled;
      encoded[ "reproj_rms_after_cull_px" ] =
          doubleBits( value.reproj_rms_after_cull_px );
      encoded[ "is_keyframe" ]         = value.is_keyframe;
      encoded[ "unsupported_span_ns" ] = value.unsupported_span_ns;
      encoded[ "num_retained_observations" ] =
          value.num_retained_observations;
      encoded[ "num_seeded_landmarks" ] = value.num_seeded_landmarks;
      encoded[ "num_current_visual_factors" ] =
          value.num_current_visual_factors;
      encoded[ "num_mapped_observations" ] =
          value.num_mapped_observations;
      encoded[ "num_current_mono_visual_factors" ] =
          value.num_current_mono_visual_factors;
      encoded[ "initialization" ] = initializationJson( value.initialization );
      return encoded;
    }

    template <typename Range, typename Encoder>
    [[nodiscard]] DiagnosticJson encodeRange( const Range& range,
                                              Encoder      encoder )
    {
      DiagnosticJson encoded = DiagnosticJson::array();
      for ( const auto& value : range )
      {
        encoded.push_back( encoder( value ) );
      }
      return encoded;
    }

    [[nodiscard]] DiagnosticJson syncDiagnosticsJson(
        const sync::StereoPairDiagnostics& value )
    {
      return DiagnosticJson{
          { "pushed_left", value.pushed_left },
          { "pushed_right", value.pushed_right },
          { "emitted_stereo", value.emitted_stereo },
          { "dropped_left", value.dropped_left },
          { "dropped_right", value.dropped_right },
          { "dropped_left_overflow", value.dropped_left_overflow },
          { "dropped_right_overflow", value.dropped_right_overflow },
          { "max_left_queue", value.max_left_queue },
          { "max_right_queue", value.max_right_queue },
          { "pushed_imu", value.pushed_imu },
          { "dropped_imu", value.dropped_imu },
          { "dropped_imu_overflow", value.dropped_imu_overflow },
          { "imu_discontinuity_count", value.imu_discontinuity_count },
          { "max_imu_queue", value.max_imu_queue } };
    }

    [[nodiscard]] DiagnosticJson gyroArtifactsJson(
        const GyroObserveArtifacts& value )
    {
      DiagnosticJson packets = DiagnosticJson::array();
      for ( const GyroPacketRow& packet : value.packets )
      {
        packets.push_back( DiagnosticJson{
            { "packet_index", packet.packet_index },
            { "t_prev_ns", packet.t_prev_ns },
            { "t_cur_ns", packet.t_cur_ns },
            { "vo_segment_id", packet.vo_segment_id },
            { "imu_gap", packet.imu_gap },
            { "sample_count", packet.sample_count },
            { "sum_dt_ns", packet.sum_dt_ns },
            { "interval_ns", packet.interval_ns },
            { "status", enumValue( packet.status ) } } );
      }
      DiagnosticJson samples = DiagnosticJson::array();
      for ( const GyroSampleRow& sample : value.samples )
      {
        samples.push_back( DiagnosticJson{
            { "packet_index", sample.packet_index },
            { "sample_index", sample.sample_index },
            { "timestamp_ns", sample.timestamp_ns },
            { "gyr_x_radps", doubleBits( sample.gyr_x_radps ) },
            { "gyr_y_radps", doubleBits( sample.gyr_y_radps ) },
            { "gyr_z_radps", doubleBits( sample.gyr_z_radps ) } } );
      }
      return DiagnosticJson{
          { "packets", std::move( packets ) },
          { "samples", std::move( samples ) } };
    }

    [[nodiscard]] DiagnosticJson sessionOptionsJson(
        const OfflineVoSessionOptions& value )
    {
      DiagnosticJson encoded;
      encoded[ "max_frames" ]           = value.max_frames.has_value()
                                              ? DiagnosticJson{ *value.max_frames }
                                              : DiagnosticJson{ nullptr };
      encoded[ "collect_timing" ]       = value.collect_timing;
      encoded[ "collect_gyro_observe" ] = value.collect_gyro_observe;
      encoded[ "drop_culled_tracks" ]   = value.drop_culled_tracks;
      encoded[ "skip_drop_min_culled" ] = value.skip_drop_min_culled;
      encoded[ "probe_b_path" ]         = value.probe_b_path.string();
      encoded[ "defer_drop_topk" ]      = value.defer_drop_topk;
      encoded[ "evict_skip_culled" ]    = value.evict_skip_culled;
      encoded[ "zombie_drop_age" ]      = value.zombie_drop_age;
      return encoded;
    }

    [[nodiscard]] DiagnosticJson sessionResultJson(
        const OfflineVoSessionResult& value )
    {
      DiagnosticJson error = nullptr;
      if ( value.error.has_value() )
      {
        error = DiagnosticJson{
            { "detail", value.error->detail },
            { "estimator_status",
              value.error->estimator_status.has_value()
                  ? DiagnosticJson{ enumValue( *value.error->estimator_status ) }
                  : DiagnosticJson{ nullptr } },
            { "timestamp_ns",
              value.error->timestamp.has_value()
                  ? DiagnosticJson{ value.error->timestamp->nanoseconds() }
                  : DiagnosticJson{ nullptr } } };
      }
      DiagnosticJson encoded;
      encoded[ "counts" ] = frameCountsJson( value.counts );
      encoded[ "diag" ]   = encodeRange(
          value.diag, []( const VoDiagRow& row ) { return diagRowJson( row ); } );
      encoded[ "gyro_observe" ]             = gyroArtifactsJson( value.gyro_observe );
      encoded[ "sync" ]                     = syncDiagnosticsJson( value.sync );
      encoded[ "warnings" ]                 = value.warnings;
      encoded[ "first_image_timestamp_ns" ] = value.first_image_ts.nanoseconds();
      encoded[ "last_image_timestamp_ns" ]  = value.last_image_ts.nanoseconds();
      encoded[ "reprojection" ]             = DiagnosticJson{
                      { "median_px", doubleBits( value.reproj.median_px ) },
                      { "p95_px", doubleBits( value.reproj.p95_px ) } };
      encoded[ "error" ] = std::move( error );
      return encoded;
    }

    void requireInactiveEstimatorTestSeams( const DiagnosticJson& estimator )
    {
      const auto& seams = estimator.at( "test_one_shot" );
      const bool  inactive =
          seams.at( "next_moving_candidate_result" ).is_null() &&
          seams.at( "next_current_graph_result" ).is_null() &&
          !seams.at( "next_current_graph_contract_error" ).get<bool>() &&
          seams.at( "next_initial_visual_depth" ).is_null() &&
          !seams.at( "next_initial_visual_whitening_error" ).get<bool>() &&
          seams.at( "next_pnp_proposal_failure" ).is_null() &&
          seams.at( "next_pnp_guess_retry_outcome" ).is_null();
      if ( !inactive )
      {
        throw std::logic_error(
            "M5 #51 replay state found an active estimator test one-shot" );
      }
    }

    [[nodiscard]] internal::M5RpeCanonicalState captureM5RpeState(
        const OfflineVoSessionOptions&                     options,
        const OfflineVoSessionResult&                      result,
        const internal::KeyframeSelectorState&             keyframe_selector,
        const frontend::StereoTracker&                     tracker,
        const estimator::VioEstimator&                     estimator,
        const StereoPairStream&                            stream,
        const std::unordered_set<common::LandmarkId>&      pending_drop,
        const std::unordered_map<common::LandmarkId, int>& zombie_age,
        const std::unordered_set<common::LandmarkId>&      lifetime_culled,
        const std::optional<std::uint32_t>&                last_segment_id,
        const std::vector<common::TimedPose>&              poses,
        const std::vector<common::TimedPose>&              keyframe_poses,
        const std::vector<double>&                         rms_after,
        const frontend::FrameTracks*                       current_tracks,
        const bool*                                        current_is_keyframe,
        const estimator::VioUpdateResult*                  current_update )
    {
      DiagnosticJson estimator_state = DiagnosticJson::parse(
          estimator::internal::VioEstimatorTestAccess::canonicalStateJson(
              estimator ) );
      requireInactiveEstimatorTestSeams( estimator_state );

      std::vector<common::LandmarkId> sorted_pending( pending_drop.begin(),
                                                      pending_drop.end() );
      std::sort( sorted_pending.begin(), sorted_pending.end() );
      std::vector<std::pair<common::LandmarkId, int>> sorted_zombies(
          zombie_age.begin(), zombie_age.end() );
      std::sort( sorted_zombies.begin(), sorted_zombies.end(),
                 []( const auto& lhs, const auto& rhs ) {
                   return lhs.first < rhs.first;
                 } );
      DiagnosticJson encoded_zombies = DiagnosticJson::array();
      for ( const auto& [ id, age ] : sorted_zombies )
      {
        encoded_zombies.push_back( DiagnosticJson{
            { "id", id }, { "age", age } } );
      }
      std::vector<common::LandmarkId> sorted_lifetime(
          lifetime_culled.begin(), lifetime_culled.end() );
      std::sort( sorted_lifetime.begin(), sorted_lifetime.end() );

      DiagnosticJson encoded_poses = encodeRange(
          poses, []( const common::TimedPose& pose ) {
            return timedPoseJson( pose );
          } );
      DiagnosticJson encoded_keyframe_poses = encodeRange(
          keyframe_poses, []( const common::TimedPose& pose ) {
            return timedPoseJson( pose );
          } );
      DiagnosticJson encoded_rms = DiagnosticJson::array();
      for ( const double value : rms_after )
      {
        encoded_rms.push_back( doubleBits( value ) );
      }

      DiagnosticJson current_frame = nullptr;
      if ( current_tracks != nullptr || current_is_keyframe != nullptr ||
           current_update != nullptr )
      {
        if ( current_tracks == nullptr || current_is_keyframe == nullptr ||
             current_update == nullptr )
        {
          throw std::logic_error(
              "M5 #51 current-frame state must be captured atomically" );
        }
        current_frame = DiagnosticJson{
            { "frame_index", result.counts.image_frames },
            { "tracks", frameTracksJson( *current_tracks ) },
            { "is_keyframe", *current_is_keyframe },
            { "update", updateJson( *current_update ) } };
      }

      DiagnosticJson session;
      session[ "options" ]         = sessionOptionsJson( options );
      session[ "result" ]          = sessionResultJson( result );
      session[ "pending_drop" ]    = std::move( sorted_pending );
      session[ "zombie_age" ]      = std::move( encoded_zombies );
      session[ "lifetime_culled" ] = std::move( sorted_lifetime );
      session[ "last_segment_id" ] = last_segment_id.has_value()
                                         ? DiagnosticJson{ *last_segment_id }
                                         : DiagnosticJson{ nullptr };
      session[ "poses" ]           = std::move( encoded_poses );
      session[ "keyframe_poses" ]  = std::move( encoded_keyframe_poses );
      session[ "rms_after" ]       = std::move( encoded_rms );
      session[ "current_frame" ]   = std::move( current_frame );

      DiagnosticJson root;
      root[ "schema" ]   = "phad.m5_51.future_runtime_state.v1";
      root[ "encoding" ] = DiagnosticJson{
          { "floating_point", "ieee754-raw-bits-big-endian-hex" },
          { "wall_clock_and_timing_samples", "excluded-non-control-state" },
          { "unordered_containers", "sorted-by-semantic-key" } };
      root[ "keyframe_selector" ] = selectorJson( keyframe_selector );
      root[ "tracker" ]           = DiagnosticJson::parse(
          frontend::internal::StereoTrackerTestAccess::canonicalStateJson(
              tracker ) );
      root[ "estimator" ] = std::move( estimator_state );
      root[ "session" ]   = std::move( session );
      root[ "stream" ]    = DiagnosticJson::parse(
          internal::StereoPairStreamTestAccess::canonicalStateJson( stream ) );

      internal::M5RpeCanonicalState captured;
      captured.m_canonical_json = root.dump();
      captured.m_sha256         = sha256( captured.m_canonical_json );
      return captured;
    }

    [[nodiscard]] bool isFormalMovingRootCommit(
        const bool                             is_keyframe,
        const estimator::VioUpdateResult&      update,
        const internal::KeyframeSelectorState& selector )
    {
      const auto& initialization = update.diagnostics.m_initialization;
      return is_keyframe && update.status == estimator::UpdateStatus::kOk &&
             update.estimate.has_value() &&
             initialization.m_path == estimator::InitializationPath::kMoving &&
             initialization.m_phase == estimator::InitializationPhase::kCommit &&
             initialization.m_outcome == estimator::InitializationOutcome::kCommitted &&
             initialization.m_attempt_id.has_value() &&
             initialization.m_current_graph ==
                 estimator::InitializationGraphResult::kValidated &&
             initialization.m_transaction ==
                 estimator::InitializationTransactionResult::kRootCommitted &&
             selector.m_total_keyframes == 1U;
    }

    [[nodiscard]] OfflineVoSessionResult diagnosticFailure(
        const std::string& detail )
    {
      OfflineVoSessionResult result;
      result.error = SessionError{ detail };
      return result;
    }
#endif
  }  // namespace

  namespace
  {
    OfflineVoSessionResult runOfflineVoSessionImpl(
        io::dataset::StereoImuDataset            dataset,
        const OfflineVoSessionOptions&           options,
        const internal::EstimatorUpdateFunction& estimator_update,
        M5RpeDiagnosticRun*                      diagnostic )
    {
      OfflineVoSessionResult result;

      const sensor::StereoImuCalibration calibration = dataset.calibration();
      auto                               rectifier   = camera::StereoRectifier::create( calibration );
      if ( !rectifier )
      {
        result.error =
            SessionError{ "stereo rectifier: " + rectifier.error().detail };
        return result;
      }

      const auto&             rectified_cal = rectifier.value().calibration();
      frontend::StereoTracker tracker( rectified_cal, options.tracker );

      // Probe B: CLI path only. Enable estimator side-channel when writing.
      std::unique_ptr<ProbeBWriter>          probe_b_writer;
      std::unordered_set<common::LandmarkId> lifetime_culled;
      estimator::EstimatorOptions            estimator_options = options.estimator;
      if ( !options.probe_b_path.empty() )
      {
        try
        {
          probe_b_writer =
              std::make_unique<ProbeBWriter>( options.probe_b_path );
        }
        catch ( const std::runtime_error& exception )
        {
          result.error = SessionError{ exception.what() };
          return result;
        }
        estimator_options.enable_probe_b = true;
      }
      estimator::VioEstimator estimator(
          rectified_cal, calibration.imu(), estimator_options );

      // Probe: optional deferred top-K drop after skip (see defer_drop_topk).
      std::unordered_set<common::LandmarkId> pending_drop;
      // Probe: consecutive survival age for skip-culled ids (zombie_drop_age).
      std::unordered_map<common::LandmarkId, int> zombie_age;

      const auto eraseZombieAge =
          [ &zombie_age ]( std::span<const common::LandmarkId> ids ) {
            for ( const common::LandmarkId id : ids )
            {
              zombie_age.erase( id );
            }
          };

      const auto flushPendingDrop = [ & ]() {
        if ( pending_drop.empty() )
        {
          return;
        }
        const std::vector<common::LandmarkId> ids( pending_drop.begin(),
                                                   pending_drop.end() );
        tracker.dropTracks( ids );
        eraseZombieAge( ids );
        ++result.counts.deferred_drops;
        result.counts.deferred_drop_ids +=
            static_cast<std::uint64_t>( ids.size() );
        pending_drop.clear();
      };

      const auto selectDeferDropTopk =
          []( const std::vector<common::LandmarkId>& culled,
              const int                              topk ) {
            std::vector<common::LandmarkId> selected;
            if ( topk <= 0 || culled.empty() )
            {
              return selected;
            }
            selected = culled;
            std::sort( selected.begin(), selected.end() );
            const auto keep = static_cast<std::size_t>( topk );
            if ( keep < selected.size() )
            {
              selected.resize( keep );
            }
            return selected;
          };

      internal::KeyframeSelectorState kf_state;

      std::vector<common::TimedPose> poses;
      std::vector<common::TimedPose> kf_poses;
      std::vector<double>            rms_after;
      std::vector<double>            rectify_s;
      std::vector<double>            frontend_s;
      std::vector<double>            estimator_s;
      std::vector<double>            total_s;

      std::optional<std::uint32_t> last_segment_id;

      const auto                       wall_begin = std::chrono::steady_clock::now();
      io::dataset::DatasetReplaySource source{ dataset };
      StereoPairStream                 stream{
          source, sync::StereoPairSynchronizerOptions{
                                      .imu_continuity_limit_ns = kImuContinuityLimitNs } };

      const auto finalizeSegmentsAndWarnings = [ &result, &stream ]() {
        result.warnings = stream.warnings();
        internal::appendSegmentSummaryWarning( result.counts, result.warnings );
        if ( result.counts.pnp_fallbacks > 0U )
        {
          result.warnings.push_back(
              "vo pnp summary: pnp_successes=" +
              std::to_string( result.counts.pnp_successes ) +
              " pnp_fallbacks=" +
              std::to_string( result.counts.pnp_fallbacks ) );
        }
      };

      while ( true )
      {
        if ( options.max_frames.has_value() &&
             result.counts.image_frames >= *options.max_frames )
        {
          break;
        }

        auto loaded = stream.nextPacket();
        if ( std::holds_alternative<io::EndOfStream>( loaded ) )
        {
          break;
        }
        if ( const auto* error = std::get_if<StreamError>( &loaded ) )
        {
          result.error = SessionError{ error->detail };
          result.sync  = stream.diagnostics();
          flushPendingDrop();
          finalizeSegmentsAndWarnings();
          return result;
        }

        const auto  frame_begin = std::chrono::steady_clock::now();
        const auto& packet      = std::get<sensor::StereoImuPacket>( loaded );
        const auto& raw         = packet.m_frame;

        const auto rectify_begin = std::chrono::steady_clock::now();
        auto       rectified     = rectifier.value().rectify( raw );
        const auto rectify_end   = std::chrono::steady_clock::now();
        if ( !rectified )
        {
          result.error =
              SessionError{ "rectify failed: " + rectified.error().detail };
          result.sync = stream.diagnostics();
          flushPendingDrop();
          finalizeSegmentsAndWarnings();
          return result;
        }

        flushPendingDrop();
        const auto                  frontend_begin = std::chrono::steady_clock::now();
        const frontend::FrameTracks tracks =
            tracker.process( rectified.value() );
        const auto frontend_end = std::chrono::steady_clock::now();

        // Candidate B: age skip-culled ids still present after process.
        if ( options.zombie_drop_age > 0 )
        {
          std::unordered_set<common::LandmarkId> present;
          present.reserve( tracks.observations.size() );
          for ( const auto& obs : tracks.observations )
          {
            present.insert( obs.id );
          }
          for ( auto it = zombie_age.begin(); it != zombie_age.end(); )
          {
            if ( present.count( it->first ) != 0U )
            {
              ++it->second;
              ++it;
            }
            else
            {
              it = zombie_age.erase( it );
            }
          }
        }

        // Zombie = frontend obs whose id was permanently culled in a prior frame.
        std::uint32_t zombie_track_n = 0;
        if ( probe_b_writer )
        {
          for ( const auto& obs : tracks.observations )
          {
            if ( lifetime_culled.count( obs.id ) != 0U )
            {
              ++zombie_track_n;
            }
          }
        }

        // Slice ⑤: keyframe selection.
        const bool is_kf = internal::isKeyframe(
            tracks, tracks.timestamp, kf_state, rectified_cal );
        const auto estimator_begin =
            std::chrono::steady_clock::now();
        const estimator::VioMeasurement measurement =
            toVioMeasurement( tracks, packet.m_imu );
        std::optional<RecoveryBridgeInput> recovery_input;
        if ( options.collect_recovery_bridge_log )
        {
          recovery_input = RecoveryBridgeInput{
              .m_processed_index = result.counts.image_frames,
              .m_timestamp_ns    = measurement.m_timestamp.nanoseconds(),
              .m_is_keyframe     = is_kf,
              .m_observations    = measurement.m_observations,
          };
        }
        const estimator::VioUpdateResult update =
            estimator_update ? estimator_update( measurement, is_kf )
                             : estimator.update( measurement, is_kf );
        const auto estimator_end = std::chrono::steady_clock::now();

        if ( options.collect_gyro_observe )
        {
          if ( const auto error = collectGyroObservePacket(
                   packet, update.diagnostics.segment_id,
                   result.gyro_observe ) )
          {
            result.error = *error;
            result.sync  = stream.diagnostics();
            flushPendingDrop();
            finalizeSegmentsAndWarnings();
            return result;
          }
        }

        internal::applyAcceptedCameraState(
            is_kf, tracks, tracks.timestamp, update, rectified_cal, kf_state );

#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
        if ( diagnostic != nullptr && result.counts.image_frames == 6U )
        {
          if ( diagnostic->receipt == nullptr )
          {
            throw std::logic_error( "M5 #51 diagnostic receipt is null" );
          }
          if ( !isFormalMovingRootCommit( is_kf, update, kf_state ) )
          {
            throw std::runtime_error(
                "M5 #51 hook frame is not the accepted moving-root commit" );
          }

          auto& receipt               = *diagnostic->receipt;
          receipt.m_pre_hook_state_v1 = captureM5RpeState(
              options, result, kf_state, tracker, estimator, stream,
              pending_drop, zombie_age, lifetime_culled, last_segment_id,
              poses, kf_poses, rms_after, &tracks, &is_kf, &update );

          if ( diagnostic->arm == internal::M5RpePrefixArm::kSelectorWarmup )
          {
            ++kf_state.m_total_keyframes;
          }

          receipt.m_post_hook_state_v1 = captureM5RpeState(
              options, result, kf_state, tracker, estimator, stream,
              pending_drop, zombie_age, lifetime_culled, last_segment_id,
              poses, kf_poses, rms_after, &tracks, &is_kf, &update );
          const DiagnosticJson pre = DiagnosticJson::parse(
              receipt.m_pre_hook_state_v1.m_canonical_json );
          const DiagnosticJson post = DiagnosticJson::parse(
              receipt.m_post_hook_state_v1.m_canonical_json );
          const DiagnosticJson difference = DiagnosticJson::diff( pre, post );
          receipt.m_hook_diff_json        = difference.dump();

          if ( diagnostic->arm == internal::M5RpePrefixArm::kBaseline )
          {
            if ( !difference.empty() )
            {
              throw std::logic_error(
                  "M5 #51 baseline hook changed runtime state" );
            }
          }
          else if ( difference.size() != 1U ||
                    difference.at( 0 ).at( "op" ) != "replace" ||
                    difference.at( 0 ).at( "path" ) !=
                        "/keyframe_selector/total_keyframes" ||
                    difference.at( 0 ).at( "value" ) != 2U )
          {
            throw std::logic_error(
                "M5 #51 intervention changed state outside the selector counter" );
          }
          receipt.m_hook_executed = true;
        }
#else
        static_cast<void>( diagnostic );
#endif
        const auto frame_end = estimator_end;

        // Composition-root feedback: drop frontend tracks for ids the
        // estimator permanently removed this frame. Default on (④c); two-level
        // gate (④f): drop_culled_tracks then skip when outliers_culled >= N.
        // Probe --defer-drop-topk: on skip, optionally queue sorted top-K ids
        // for flush before next process (K=0 keeps ④f; K=full ≈④g/④e).
        // Probe --zombie-drop-age: on skip, track consecutive presence; drop
        // when age >= N (see 2026-08-03-note-m3-3-zombie-drop-age-probe-design.md).
        // Does not enter warnings.
        bool drops_skipped_this_frame = false;
        if ( options.drop_culled_tracks &&
             !update.diagnostics.culled_landmark_ids.empty() )
        {
          const bool skip =
              options.skip_drop_min_culled > 0 &&
              update.diagnostics.outliers_culled >=
                  static_cast<std::uint32_t>( options.skip_drop_min_culled );
          if ( skip )
          {
            ++result.counts.drops_skipped;
            drops_skipped_this_frame = true;
            for ( const common::LandmarkId id : selectDeferDropTopk(
                      update.diagnostics.culled_landmark_ids,
                      options.defer_drop_topk ) )
            {
              pending_drop.insert( id );
            }
            if ( options.evict_skip_culled )
            {
              tracker.markEvictable( update.diagnostics.culled_landmark_ids );
              ++result.counts.evictable_marked;
            }
            if ( options.zombie_drop_age > 0 )
            {
              for ( const common::LandmarkId id :
                    update.diagnostics.culled_landmark_ids )
              {
                zombie_age.try_emplace( id, 1 );
              }
            }
          }
          else
          {
            tracker.dropTracks( update.diagnostics.culled_landmark_ids );
            eraseZombieAge( update.diagnostics.culled_landmark_ids );
          }
        }

        if ( options.zombie_drop_age > 0 )
        {
          std::vector<common::LandmarkId> aged_drop;
          for ( const auto& [ id, age ] : zombie_age )
          {
            if ( age >= options.zombie_drop_age )
            {
              aged_drop.push_back( id );
            }
          }
          if ( !aged_drop.empty() )
          {
            tracker.dropTracks( aged_drop );
            eraseZombieAge( aged_drop );
            ++result.counts.zombie_age_drops;
            result.counts.zombie_age_drop_ids +=
                static_cast<std::uint64_t>( aged_drop.size() );
          }
        }

        result.counts.tracks_evicted += tracks.stats.evicted;

        // Probe B jsonl: i matches diag row order (0-based before increment).
        if ( probe_b_writer )
        {
          const std::uint64_t frame_i = result.counts.image_frames;
          const auto&         d       = update.diagnostics;
          for ( const common::LandmarkId id : d.culled_landmark_ids )
          {
            lifetime_culled.insert( id );
          }

          const bool heavy =
              ( frame_i >= 420U && frame_i <= 450U ) ||
              !d.culled_landmark_ids.empty() || d.lm_iterations >= 100U ||
              d.max_window_pose_shift_m > 0.5;

          ProbeBFrame probe_frame;
          probe_frame.i                   = frame_i;
          probe_frame.ts_ns               = tracks.timestamp.nanoseconds();
          probe_frame.pnp_attempted       = d.probe_pnp_attempted;
          probe_frame.pnp_result          = d.probe_pnp_result;
          probe_frame.pnp_correspondences = d.probe_pnp_correspondences;
          probe_frame.pnp_inliers         = d.probe_pnp_inliers;
          probe_frame.pnp_proposal_rms_px = d.probe_pnp_proposal_rms_px;
          probe_frame.pnp_guess_rms_px    = d.probe_pnp_guess_rms_px;
          probe_frame.pnp_proposal_inlier_count =
              d.probe_pnp_proposal_inlier_count;
          probe_frame.pnp_proposal_cheiral_inlier_count =
              d.probe_pnp_proposal_cheiral_inlier_count;
          probe_frame.pnp_proposal_min_inlier_depth_m =
              d.probe_pnp_proposal_min_inlier_depth_m;
          probe_frame.pnp_proposal_min_inlier_landmark_id =
              d.probe_pnp_proposal_min_inlier_landmark_id;
          probe_frame.pnp_proposal_guess_translation_delta_m =
              d.probe_pnp_proposal_guess_translation_delta_m;
          probe_frame.pnp_proposal_guess_rotation_delta_rad =
              d.probe_pnp_proposal_guess_rotation_delta_rad;
          probe_frame.pnp_guess_retry_outcome =
              d.probe_pnp_guess_retry_outcome;
          probe_frame.pnp_guess_retry_proposal_rms_px =
              d.probe_pnp_guess_retry_proposal_rms_px;
          probe_frame.pnp_guess_retry_guess_rms_px =
              d.probe_pnp_guess_retry_guess_rms_px;
          probe_frame.pnp_guess_retry_cheiral_inlier_count =
              d.probe_pnp_guess_retry_cheiral_inlier_count;
          probe_frame.pnp_guess_retry_min_inlier_depth_m =
              d.probe_pnp_guess_retry_min_inlier_depth_m;
          probe_frame.pnp_guess_retry_translation_delta_m =
              d.probe_pnp_guess_retry_translation_delta_m;
          probe_frame.pnp_guess_retry_rotation_delta_rad =
              d.probe_pnp_guess_retry_rotation_delta_rad;
          probe_frame.current_pose_translation_delta_m =
              d.probe_current_pose_translation_delta_m;
          probe_frame.current_pose_rotation_delta_rad =
              d.probe_current_pose_rotation_delta_rad;
          probe_frame.current_velocity_delta_mps =
              d.probe_current_velocity_delta_mps;
          probe_frame.current_acc_bias_delta_mps2 =
              d.probe_current_acc_bias_delta_mps2;
          probe_frame.current_gyr_bias_delta_radps =
              d.probe_current_gyr_bias_delta_radps;
          probe_frame.primary_graph_error_seed =
              d.probe_primary_graph_error_seed;
          probe_frame.primary_graph_error_optimized =
              d.probe_primary_graph_error_optimized;
          probe_frame.primary_imu_factor_error_seed =
              d.probe_primary_imu_factor_error_seed;
          probe_frame.primary_imu_factor_error_optimized =
              d.probe_primary_imu_factor_error_optimized;
          if ( heavy )
          {
            probe_frame.culled_ids = std::vector<std::uint64_t>(
                d.culled_landmark_ids.begin(), d.culled_landmark_ids.end() );
            probe_frame.zombie_track_n   = zombie_track_n;
            probe_frame.rejected_block_n = d.probe_rejected_block_n;
            probe_frame.new_lm           = d.probe_new_lm_n;
            probe_frame.shared           = d.num_shared;
            probe_frame.num_obs          = d.num_observations;
            probe_frame.lm_iterations    = d.lm_iterations;
            probe_frame.shift_m          = d.max_window_pose_shift_m;
            std::vector<ProbeBShiftTop> shift_top;
            shift_top.reserve( d.probe_shift_top.size() );
            for ( const auto& entry : d.probe_shift_top )
            {
              shift_top.push_back(
                  ProbeBShiftTop{ .key = entry.first, .dt_m = entry.second } );
            }
            probe_frame.shift_top = std::move( shift_top );
            if ( d.probe_detail_valid )
            {
              probe_frame.res_mean_px = d.probe_res_mean_px;
              probe_frame.res_max_px  = d.probe_res_max_px;
              probe_frame.res_max_id  = static_cast<std::uint64_t>(
                  d.probe_res_max_id );
            }
            probe_frame.drops_skipped_this_frame = drops_skipped_this_frame;
          }
          else
          {
            // Light frame (design §4): keep index continuity without heavy fields.
            probe_frame.shared  = d.num_shared;
            probe_frame.num_obs = d.num_observations;
          }

          try
          {
            probe_b_writer->write( probe_frame );
          }
          catch ( const std::runtime_error& exception )
          {
            result.error = SessionError{ exception.what() };
            result.sync  = stream.diagnostics();
            flushPendingDrop();
            finalizeSegmentsAndWarnings();
            return result;
          }
        }

        if ( result.counts.image_frames == 0U )
        {
          result.first_image_ts = tracks.timestamp;
        }
        result.last_image_ts = tracks.timestamp;
        ++result.counts.image_frames;

        if ( is_kf )
        {
          ++result.counts.total_keyframes;
        }
        else
        {
          ++result.counts.total_track_only_frames;
        }

        bool terminal_estimator_result = false;
        switch ( update.status )
        {
          case estimator::UpdateStatus::kOk:
            ++result.counts.ok;
            break;
          case estimator::UpdateStatus::kInitializing:
          case estimator::UpdateStatus::kDiscontinuity:
          case estimator::UpdateStatus::kVisualOutage:
            break;
          case estimator::UpdateStatus::kInvalidInput:
          case estimator::UpdateStatus::kFailed:
            ++result.counts.failed;
            result.error = SessionError{
                .detail = "VIO estimator " +
                          std::string{ updateStatusName( update.status ) } +
                          ": " + update.message,
                .estimator_status = update.status,
                .timestamp        = tracks.timestamp,
            };
            terminal_estimator_result = true;
            break;
          case estimator::UpdateStatus::kRejected:
            ++result.counts.rejected;
            if ( update.message == kSeedRejectedNewSegment ||
                 update.message == kSeedRejectedFirstSegment )
            {
              ++result.counts.seed_rejected;
            }
            break;
        }
        if ( update.diagnostics.low_connectivity )
        {
          ++result.counts.low_connectivity;
        }

        const auto& d = update.diagnostics;
        result.counts.non_keyframe_evictions =
            d.m_vio.m_non_keyframe_evictions;
        result.counts.imu_reintegrations = d.m_vio.m_imu_reintegrations;
        result.counts.outliers_culled += d.outliers_culled;
        result.counts.outliers_culled_unique += d.outliers_culled_unique;
        // Slice ④e: accumulate successful reopt *rounds*, not frames-with-reopt.
        result.counts.outlier_reopts += d.outlier_reopt_rounds;
        if ( update.status == estimator::UpdateStatus::kOk )
        {
          // All accepted frames write est.tum (non-keyframes carry PnP pose);
          // keyframes additionally collect kf.tum.
          rms_after.push_back( d.reproj_rms_after_px );
          common::TimedPose pose;
          pose.timestamp = update.estimate->timestamp;
          pose.T_W_B     = update.estimate->T_W_B;
          poses.push_back( pose );
          if ( is_kf )
          {
            kf_poses.push_back( pose );
          }

          const std::uint32_t segment_id = d.segment_id;
          const bool          is_new_segment =
              !last_segment_id.has_value() || segment_id > *last_segment_id;
          internal::recordAcceptedSegment( segment_id, last_segment_id,
                                           result.counts );

          // Normal-path PnP counters exclude each segment root. A disabled PnP
          // initializer is not a
          // fallback — that path never attempted PnP.
          if ( d.pnp_success )
          {
            ++result.counts.pnp_successes;
          }
          if ( options.estimator.enable_pnp_init &&
               d.num_mapped_observations > 0U &&
               !d.pnp_success && !is_new_segment )
          {
            ++result.counts.pnp_fallbacks;
          }
        }

        result.diag.push_back( VoDiagRow{
            .timestamp_ns               = tracks.timestamp.nanoseconds(),
            .status                     = updateStatusName( update.status ),
            .num_observations           = d.num_observations,
            .num_landmarks              = d.num_landmarks,
            .num_shared                 = d.num_shared,
            .num_disparity              = d.num_disparity,
            .low_connectivity           = d.low_connectivity,
            .window_size                = d.window_size,
            .prior_key                  = d.prior_key,
            .reproj_rms_before_px       = d.reproj_rms_before_px,
            .reproj_rms_after_px        = d.reproj_rms_after_px,
            .num_cheirality             = d.num_cheirality,
            .lm_iterations              = d.lm_iterations,
            .max_window_pose_shift_m    = d.max_window_pose_shift_m,
            .segment_id                 = d.segment_id,
            .pnp_success                = d.pnp_success,
            .pnp_inliers                = d.pnp_inliers,
            .outliers_culled            = d.outliers_culled,
            .reproj_rms_after_cull_px   = d.reproj_rms_after_cull_px,
            .is_keyframe                = is_kf,
            .unsupported_span_ns        = d.unsupported_span_ns,
            .num_retained_observations  = d.num_retained_observations,
            .num_seeded_landmarks       = d.num_seeded_landmarks,
            .num_current_visual_factors = d.num_current_visual_factors,
            .num_mapped_observations    = d.num_mapped_observations,
            .num_current_mono_visual_factors =
                d.num_current_mono_visual_factors,
            .initialization    = d.m_initialization,
            .m_recovery_bridge = d.m_recovery_bridge,
            .m_recovery_input  = std::move( recovery_input ),
        } );

        if ( options.collect_timing )
        {
          const auto seconds = []( auto begin, auto end ) {
            return std::chrono::duration<double>( end - begin ).count();
          };
          rectify_s.push_back( seconds( rectify_begin, rectify_end ) );
          frontend_s.push_back( seconds( frontend_begin, frontend_end ) );
          estimator_s.push_back( seconds( estimator_begin, estimator_end ) );
          total_s.push_back( seconds( frame_begin, frame_end ) );
        }
        if ( terminal_estimator_result )
        {
          break;
        }
      }

      flushPendingDrop();

      const auto wall_end = std::chrono::steady_clock::now();
      result.wall_s =
          std::chrono::duration<double>( wall_end - wall_begin ).count();

      if ( options.collect_timing )
      {
        result.timings.rectify   = summarizeStage( std::move( rectify_s ) );
        result.timings.frontend  = summarizeStage( std::move( frontend_s ) );
        result.timings.estimator = summarizeStage( std::move( estimator_s ) );
        result.timings.total     = summarizeStage( std::move( total_s ) );
      }

      result.reproj.median_px = percentile( rms_after, 0.5 );
      result.reproj.p95_px    = percentile( rms_after, 0.95 );
      result.sync             = stream.diagnostics();
      finalizeSegmentsAndWarnings();

#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
      if ( diagnostic != nullptr )
      {
        if ( diagnostic->receipt == nullptr ||
             !diagnostic->receipt->m_hook_executed )
        {
          throw std::runtime_error(
              "M5 #51 diagnostic run ended before the moving-root hook" );
        }
        diagnostic->receipt->m_post_state_v1 = captureM5RpeState(
            options, result, kf_state, tracker, estimator, stream,
            pending_drop, zombie_age, lifetime_culled, last_segment_id,
            poses, kf_poses, rms_after, nullptr, nullptr, nullptr );
      }
#endif

      if ( !poses.empty() )
      {
        auto trajectory = common::Trajectory::create( std::move( poses ) );
        if ( !trajectory )
        {
          result.error = SessionError{ "trajectory create failed: " +
                                       trajectory.error().detail };
          return result;
        }
        result.trajectory = std::move( trajectory.value() );
      }

      // Keyframe-only trajectory (kf.tum).
      if ( !kf_poses.empty() )
      {
        auto kf_trajectory = common::Trajectory::create( std::move( kf_poses ) );
        if ( !kf_trajectory )
        {
          result.error = SessionError{ "kf trajectory create failed: " +
                                       kf_trajectory.error().detail };
          return result;
        }
        result.kf_trajectory = std::move( kf_trajectory.value() );
      }
      return result;
    }
  }  // namespace

  OfflineVoSessionResult internal::runOfflineVoSessionWithEstimatorUpdate(
      io::dataset::StereoImuDataset  dataset,
      const OfflineVoSessionOptions& options,
      const EstimatorUpdateFunction& estimator_update )
  {
    return runOfflineVoSessionImpl( std::move( dataset ), options,
                                    estimator_update, nullptr );
  }

#ifdef PHAD_REPLAY_DIAGNOSTIC_TESTING
  OfflineVoSessionResult internal::runM5RpePrefixDiagnostic(
      io::dataset::StereoImuDataset  dataset,
      const OfflineVoSessionOptions& options,
      const M5RpePrefixArm           arm,
      M5RpePrefixDiagnosticReceipt&  receipt )
  {
    receipt = M5RpePrefixDiagnosticReceipt{};
    if ( options.collect_timing )
    {
      receipt.m_failure_reason =
          "M5 #51 diagnostic requires collect_timing=false";
      return diagnosticFailure( receipt.m_failure_reason );
    }
    if ( !options.max_frames.has_value() || *options.max_frames < 8U )
    {
      receipt.m_failure_reason =
          "M5 #51 diagnostic requires a prefix of at least 8 frames";
      return diagnosticFailure( receipt.m_failure_reason );
    }
    if ( options.collect_gyro_observe || !options.probe_b_path.empty() ||
         options.estimator.enable_probe_b )
    {
      receipt.m_failure_reason =
          "M5 #51 diagnostic forbids mutable output side channels";
      return diagnosticFailure( receipt.m_failure_reason );
    }

    M5RpeDiagnosticRun diagnostic{ .arm = arm, .receipt = &receipt };
    try
    {
      OfflineVoSessionResult result = runOfflineVoSessionImpl(
          std::move( dataset ), options, {}, &diagnostic );
      if ( result.error.has_value() && receipt.m_failure_reason.empty() )
      {
        receipt.m_failure_reason = result.error->detail;
      }
      if ( !receipt.m_hook_executed && receipt.m_failure_reason.empty() )
      {
        receipt.m_failure_reason =
            "M5 #51 diagnostic did not execute the frame-6 hook";
      }
      if ( receipt.m_post_state_v1.m_canonical_json.empty() &&
           receipt.m_failure_reason.empty() )
      {
        receipt.m_failure_reason =
            "M5 #51 diagnostic did not capture the final prefix state";
      }
      if ( !receipt.m_failure_reason.empty() && !result.error.has_value() )
      {
        result.error = SessionError{ receipt.m_failure_reason };
      }
      return result;
    }
    catch ( const std::exception& exception )
    {
      receipt.m_failure_reason = exception.what();
      return diagnosticFailure( receipt.m_failure_reason );
    }
  }
#endif

  OfflineVoSessionResult runOfflineVoSession(
      io::dataset::StereoImuDataset  dataset,
      const OfflineVoSessionOptions& options )
  {
    return internal::runOfflineVoSessionWithEstimatorUpdate(
        std::move( dataset ), options, {} );
  }

  std::optional<SessionError> collectGyroObservePacket(
      const sensor::StereoImuPacket& packet,
      std::uint32_t                  vo_segment_id,
      GyroObserveArtifacts&          artifacts )
  {
    const auto* raw =
        std::get_if<sensor::RawImuInterval>( &packet.m_imu );
    const bool              imu_gap = raw == nullptr;
    const common::Timestamp t_begin =
        raw != nullptr
            ? raw->m_t_begin
            : std::get<sensor::MeasurementDiscontinuity>( packet.m_imu )
                  .m_t_begin;
    const common::Timestamp t_end =
        raw != nullptr
            ? raw->m_t_end
            : std::get<sensor::MeasurementDiscontinuity>( packet.m_imu )
                  .m_t_end;
    const std::vector<sensor::ImuMeasurement> empty_samples;
    const auto&                               samples = raw != nullptr ? raw->m_samples : empty_samples;

    std::uint64_t packet_index = 0;
    std::uint64_t sample_count = 0;
    if ( !sizeToUint64( artifacts.packets.size(), packet_index ) )
    {
      return gyroObserveError( "packet_index_overflow" );
    }
    if ( !sizeToUint64( samples.size(), sample_count ) )
    {
      return gyroObserveError( "sample_count_overflow" );
    }

    std::int64_t interval_ns = 0;
    if ( !checkedSubtract( t_end.nanoseconds(), t_begin.nanoseconds(),
                           interval_ns ) )
    {
      return gyroObserveError( "interval_overflow" );
    }

    std::vector<GyroSampleRow> rows;
    rows.reserve( samples.size() );
    for ( std::size_t index = 0; index < samples.size(); ++index )
    {
      std::uint64_t sample_index = 0;
      if ( !sizeToUint64( index, sample_index ) )
      {
        return gyroObserveError( "sample_index_overflow" );
      }
      const sensor::ImuMeasurement& sample = samples[ index ];
      rows.push_back( GyroSampleRow{
          .packet_index = packet_index,
          .sample_index = sample_index,
          .timestamp_ns = sample.timestamp.nanoseconds(),
          .gyr_x_radps  = sample.gyro_radps[ 0 ],
          .gyr_y_radps  = sample.gyro_radps[ 1 ],
          .gyr_z_radps  = sample.gyro_radps[ 2 ],
      } );
    }

    GyroPacketRow packet_row{
        .packet_index  = packet_index,
        .t_prev_ns     = t_begin.nanoseconds(),
        .t_cur_ns      = t_end.nanoseconds(),
        .vo_segment_id = vo_segment_id,
        .imu_gap       = imu_gap,
        .sample_count  = sample_count,
        .sum_dt_ns     = 0,
        .interval_ns   = interval_ns,
        .status        = GyroPacketStatus::kFirstZero,
    };

    std::int64_t sum_dt_ns = 0;
    for ( std::size_t index = 1; index < rows.size(); ++index )
    {
      const std::int64_t current_ns  = rows[ index ].timestamp_ns;
      const std::int64_t previous_ns = rows[ index - 1U ].timestamp_ns;
      if ( current_ns == previous_ns )
      {
        return gyroObserveError( "sample_duplicate" );
      }
      if ( current_ns < previous_ns )
      {
        return gyroObserveError( "sample_out_of_order" );
      }
      std::int64_t dt_ns = 0;
      if ( !checkedSubtract( current_ns, previous_ns, dt_ns ) )
      {
        return gyroObserveError( "sample_delta_overflow" );
      }
      if ( !checkedAdd( sum_dt_ns, dt_ns, sum_dt_ns ) )
      {
        return gyroObserveError( "sum_dt_overflow" );
      }
    }
    packet_row.sum_dt_ns = sum_dt_ns;

    if ( imu_gap )
    {
      packet_row.status = GyroPacketStatus::kGap;
    }
    else if ( rows.empty() )
    {
      packet_row.status = GyroPacketStatus::kEmptyNonfirst;
    }
    else
    {
      packet_row.status = GyroPacketStatus::kValid;
    }

    if ( const auto error = validateGyroPacketRow( packet_row, rows ) )
    {
      return error;
    }
    artifacts.packets.push_back( packet_row );
    artifacts.samples.insert( artifacts.samples.end(), rows.begin(),
                              rows.end() );
    return std::nullopt;
  }

  std::optional<SessionError> validateGyroObserveArtifacts(
      const GyroObserveArtifacts& artifacts )
  {
    std::size_t sample_begin = 0;
    for ( std::size_t index = 0; index < artifacts.packets.size(); ++index )
    {
      const GyroPacketRow& packet = artifacts.packets[ index ];
      if ( packet.packet_index != static_cast<std::uint64_t>( index ) )
      {
        return gyroObserveError( "packet_index_mismatch" );
      }
      if ( packet.sample_count >
           static_cast<std::uint64_t>( artifacts.samples.size() -
                                       sample_begin ) )
      {
        return gyroObserveError( "sample_count_mismatch" );
      }
      const std::size_t count =
          static_cast<std::size_t>( packet.sample_count );
      const std::span<const GyroSampleRow> all_samples{ artifacts.samples };
      const auto                           packet_samples = all_samples.subspan( sample_begin, count );
      if ( const auto error = validateGyroPacketRow( packet, packet_samples ) )
      {
        return error;
      }
      sample_begin += count;
    }
    if ( sample_begin != artifacts.samples.size() )
    {
      return gyroObserveError( "orphan_sample" );
    }
    return std::nullopt;
  }

  std::optional<SessionError> writeDiagCsv(
      const std::filesystem::path& path, const std::vector<VoDiagRow>& rows )
  {
    std::ofstream out( path );
    if ( !out )
    {
      return SessionError{ "failed to open diag csv: " + path.string() };
    }

    out << "timestamp_ns,status,num_obs,num_landmarks,num_shared,"
           "low_connectivity,window_size,prior_key,"
           "reproj_rms_before_px,reproj_rms_after_px,num_cheirality,"
           "lm_iterations,max_window_pose_shift_m,segment_id,"
           "pnp_success,pnp_inliers,outliers_culled,"
           "reproj_rms_after_cull_px,is_keyframe,num_disparity,"
           "unsupported_span_ns,num_retained_observations,"
           "num_seeded_landmarks,num_current_visual_factors,"
           "num_mapped_observations,num_current_mono_visual_factors\n";

    for ( const VoDiagRow& row : rows )
    {
      // Keep manipulator placement identical to the pre-migration probe so
      // MH_01 diag.csv stays byte-identical under std::fixed / setprecision.
      out << row.timestamp_ns << ',' << row.status << ','
          << row.num_observations << ',' << row.num_landmarks << ','
          << row.num_shared << ',' << ( row.low_connectivity ? 1 : 0 ) << ','
          << row.window_size << ',' << row.prior_key << ',' << std::fixed
          << std::setprecision( 6 ) << row.reproj_rms_before_px << ','
          << row.reproj_rms_after_px << ',' << row.num_cheirality << ','
          << row.lm_iterations << ',' << row.max_window_pose_shift_m << ','
          << row.segment_id << ',' << ( row.pnp_success ? 1 : 0 ) << ','
          << row.pnp_inliers << ',' << row.outliers_culled << ','
          << row.reproj_rms_after_cull_px << ','
          << ( row.is_keyframe ? 1 : 0 ) << ','
          << row.num_disparity << ',' << row.unsupported_span_ns << ','
          << row.num_retained_observations << ','
          << row.num_seeded_landmarks << ','
          << row.num_current_visual_factors << ','
          << row.num_mapped_observations << ','
          << row.num_current_mono_visual_factors << '\n';
    }

    if ( !out )
    {
      return SessionError{ "failed to write diag csv: " + path.string() };
    }
    return std::nullopt;
  }

}  // namespace phad::apps
