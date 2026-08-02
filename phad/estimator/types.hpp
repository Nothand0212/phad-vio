#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"

namespace phad::estimator
{

  using common::LandmarkId;

  struct StereoObservation
  {
    LandmarkId      id;
    Eigen::Vector2d left_pixel;    // rectified left
    double          disparity_px;  // must be > 0
  };

  struct KeyframeMeasurement
  {
    common::Timestamp              timestamp;
    std::vector<StereoObservation> observations;
  };

  struct EstimatorOptions
  {
    int    window_size                = 10;
    int    min_landmark_observations  = 2;
    int    min_seed_observations      = 10;  // init and re-anchor
    int    min_shared_landmarks       = 10;
    double stereo_sigma_px            = 1.0;
    double huber_k_px                 = 3.0;  // <= 0 disables Robust wrapper
    double prior_rotation_sigma_rad   = 1e-4;
    double prior_translation_sigma_m  = 1e-4;
    bool   use_constant_velocity_init = true;
    bool   enable_reanchor            = true;  // false reproduces M3.2 permanent reject
    bool   enable_pnp_init            = true;
    double pnp_reproj_px              = 2.0;
    double pnp_confidence             = 0.99;
    int    min_pnp_inliers            = 10;
    // enable_outlier_cull only gates mean-reproj cull; cheirality always
    // clears window observations for dropped landmarks.
    bool   enable_outlier_cull   = true;
    bool   enable_outlier_reopt  = true;  // false → 复现 b6fbcb6 只 cull
    double outlier_avg_reproj_px = 4.0;
    // After mean-cull / cheirality erase: refuse same LandmarkId backproject.
    // false → allow rebirth (Slice ④ pseudo-permanent; A/B only).
    bool block_culled_rebirth = true;
  };

  enum class UpdateStatus : std::uint8_t
  {
    kOk       = 0,
    kRejected = 1,
    kFailed   = 2
  };

  struct VioEstimate
  {
    common::Timestamp timestamp;
    Eigen::Isometry3d T_W_B;
  };

  struct UpdateDiagnostics
  {
    std::uint32_t num_observations        = 0;
    std::uint32_t num_landmarks           = 0;  // in the graph
    std::uint32_t num_shared              = 0;  // new frame ∩ window landmark table
    std::uint32_t num_cheirality          = 0;
    std::uint32_t lm_iterations           = 0;
    std::uint32_t window_size             = 0;
    std::uint32_t segment_id              = 0;  // increments on re-anchor; 0 is first segment
    std::uint64_t prior_key               = 0;  // Symbol('x', k) index k
    double        reproj_rms_before_px    = 0.0;
    double        reproj_rms_after_px     = 0.0;
    double        max_window_pose_shift_m = 0.0;
    bool          low_connectivity        = false;
    bool          pnp_success             = false;
    std::uint32_t pnp_inliers             = 0;
    std::uint32_t outliers_culled          = 0;
    std::uint32_t outliers_culled_unique   = 0;
    double        reproj_rms_after_cull_px = 0.0;
    bool          outlier_reopt            = false;  // 本帧是否成功跑了 LM₂
    bool          outlier_reopt_failed     = false;  // LM₂ 失败已回退；不进 diag.csv
    // 本帧永久移出地图的 id（mean-cull ∪ cheirality）；不进 diag.csv
    std::vector<common::LandmarkId> culled_landmark_ids;
  };

  struct VioUpdateResult
  {
    UpdateStatus               status = UpdateStatus::kFailed;
    std::optional<VioEstimate> estimate;  // only when kOk
    UpdateDiagnostics          diagnostics;
    std::string                message;  // non-empty when not kOk
  };

}  // namespace phad::estimator
