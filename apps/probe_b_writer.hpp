#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <vector>

#include "phad/estimator/types.hpp"

/**
 * @file probe_b_writer.hpp
 * @brief MH_05 Probe B 旁路 jsonl writer（默认关；session 在 path 非空时构造）。
 */

namespace phad::apps
{

  struct ProbeBShiftTop
  {
    std::uint64_t key  = 0;
    double        dt_m = 0.0;
  };

  /// 单行 probe_b.jsonl 记录；optional 字段表示轻/重帧差异。
  struct ProbeBFrame
  {
    std::uint64_t                                  i     = 0;
    std::int64_t                                   ts_ns = 0;
    std::optional<bool>                            pnp_attempted;
    std::optional<estimator::PnpProbeResult>       pnp_result;
    std::optional<std::uint32_t>                   pnp_correspondences;
    std::optional<std::uint32_t>                   pnp_inliers;
    std::optional<double>                          pnp_proposal_rms_px;
    std::optional<double>                          pnp_guess_rms_px;
    std::optional<std::uint32_t>                   pnp_proposal_inlier_count;
    std::optional<std::uint32_t>                   pnp_proposal_cheiral_inlier_count;
    std::optional<double>                          pnp_proposal_min_inlier_depth_m;
    std::optional<std::uint64_t>                   pnp_proposal_min_inlier_landmark_id;
    std::optional<double>                          pnp_proposal_guess_translation_delta_m;
    std::optional<double>                          pnp_proposal_guess_rotation_delta_rad;
    std::optional<estimator::PnpGuessRetryOutcome> pnp_guess_retry_outcome;
    std::optional<double>                          pnp_guess_retry_proposal_rms_px;
    std::optional<double>                          pnp_guess_retry_guess_rms_px;
    std::optional<std::uint32_t>                   pnp_guess_retry_cheiral_inlier_count;
    std::optional<double>                          pnp_guess_retry_min_inlier_depth_m;
    std::optional<double>                          pnp_guess_retry_translation_delta_m;
    std::optional<double>                          pnp_guess_retry_rotation_delta_rad;
    std::optional<double>                          current_pose_translation_delta_m;
    std::optional<double>                          current_pose_rotation_delta_rad;
    std::optional<double>                          current_velocity_delta_mps;
    std::optional<double>                          current_acc_bias_delta_mps2;
    std::optional<double>                          current_gyr_bias_delta_radps;
    // Stable keys retain "error" for this probe generation, but values are
    // GTSAM noise-weighted half-squared objectives with no physical unit.
    std::optional<double>                      primary_graph_error_seed;
    std::optional<double>                      primary_graph_error_optimized;
    std::optional<double>                      primary_imu_factor_error_seed;
    std::optional<double>                      primary_imu_factor_error_optimized;
    std::optional<std::vector<std::uint64_t>>  culled_ids;
    std::optional<std::uint32_t>               zombie_track_n;
    std::optional<std::uint32_t>               rejected_block_n;
    std::optional<std::uint32_t>               new_lm;
    std::optional<std::uint32_t>               shared;
    std::optional<std::uint32_t>               num_obs;
    std::optional<std::uint32_t>               lm_iterations;
    std::optional<double>                      shift_m;
    std::optional<std::vector<ProbeBShiftTop>> shift_top;
    std::optional<double>                      res_mean_px;
    std::optional<double>                      res_max_px;
    std::optional<std::uint64_t>               res_max_id;
    std::optional<bool>                        drops_skipped_this_frame;
  };

  class ProbeBWriter
  {
  public:
    explicit ProbeBWriter( const std::filesystem::path& path );
    ~ProbeBWriter();

    ProbeBWriter( const ProbeBWriter& )            = delete;
    ProbeBWriter& operator=( const ProbeBWriter& ) = delete;
    ProbeBWriter( ProbeBWriter&& )                 = delete;
    ProbeBWriter& operator=( ProbeBWriter&& )      = delete;

    void write( const ProbeBFrame& frame );

  private:
    std::ofstream m_out;
  };

}  // namespace phad::apps
