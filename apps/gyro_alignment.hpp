#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "phad/sensor/imu_measurement.hpp"

namespace phad::apps
{

  enum class GyroAlignmentEndpointStatus
  {
    kOk,
    kRejected,
    kFailed,
  };

  enum class GyroAlignmentPacketStatus
  {
    kFirstZero,
    kValid,
    kGap,
    kEmptyNonfirst,
  };

  enum class GyroAlignmentFitStatus
  {
    kSupportInsufficient,
    kRankDeficient,
    kIllConditioned,
    kObservabilityFailed,
    kSolved,
  };

  enum class GyroAlignmentStatus
  {
    kPass,
    kHypothesisFail,
    kInconclusive,
    kHardError,
  };

  enum class GyroAlignmentReason
  {
    kFitSupport,
    kValidationSupport,
    kFitRank,
    kFitCondition,
    kFitObservability,
    kPrefixNonlinearLoss,
    kValidationAggregate,
    kValidationFraction,
    kValidationAxis,
    kHalfStability,
  };

  enum class GyroAlignmentErrorCode
  {
    kInvalidProtocolDescriptor,
    kInputNotFound,
    kInputNotRegularFile,
    kInputHashMismatch,
    kInputIo,
    kInputGrammar,
    kInputSchema,
    kUnknownEnum,
    kDiagFailed,
    kIndexOrder,
    kCountMismatch,
    kDurationMismatch,
    kEndpointMismatch,
    kJoinMismatch,
    kTimestampOverflow,
    kNonfiniteInput,
    kInvalidQuaternion,
    kDiagPoseContradiction,
    kQ2IntegrationError,
    kNonfiniteComputation,
    kOutputExists,
    kOutputIo,
    kOutputPublish,
    kOutputRehashMismatch,
  };

  struct GyroAlignmentEndpoint
  {
    std::int64_t                      timestamp_ns{};
    GyroAlignmentEndpointStatus       status{};
    std::uint32_t                     segment_id{};
    std::optional<Eigen::Quaterniond> q_WB;
  };

  struct GyroAlignmentPacket
  {
    std::uint64_t                       packet_index{};
    std::int64_t                        t_prev_ns{};
    std::int64_t                        t_cur_ns{};
    std::uint32_t                       segment_id{};
    bool                                imu_gap{};
    GyroAlignmentPacketStatus           status{};
    std::vector<sensor::ImuMeasurement> samples;
    std::int64_t                        sum_dt_ns{};
    std::int64_t                        interval_ns{};
  };

  struct GyroAlignmentInput
  {
    std::vector<GyroAlignmentEndpoint> endpoints;
    std::vector<GyroAlignmentPacket>   packets;
  };

  struct GyroAlignmentRanges
  {
    std::int64_t                t0_ns{};
    std::int64_t                t_mid_ns{};
    std::int64_t                t_split_ns{};
    std::optional<std::int64_t> validation_last_end_ns;
  };

  struct GyroAlignmentIntervalExcludedCounts
  {
    std::uint64_t gap{};
    std::uint64_t empty_nonfirst{};
    std::uint64_t diag_rejected_no_pose{};
    std::uint64_t segment_boundary{};
    std::uint64_t near_pi{};
  };

  struct GyroAlignmentBlockExcludedCounts
  {
    std::optional<std::uint64_t> incomplete_calendar_block;
    std::optional<std::uint64_t> near_pi;
  };

  struct GyroAlignmentEligibility
  {
    std::uint64_t                       packet_total_nonfirst{};
    std::uint64_t                       packet_structurally_eligible{};
    std::uint64_t                       packet_fit_eligible{};
    GyroAlignmentIntervalExcludedCounts interval_excluded_counts;
    std::uint64_t                       validation_slots_total{};
    std::optional<std::uint64_t>        validation_blocks_eligible;
    GyroAlignmentBlockExcludedCounts    block_excluded_counts;
  };

  struct GyroAlignmentFit
  {
    GyroAlignmentFitStatus status{
        GyroAlignmentFitStatus::kSupportInsufficient };
    std::optional<Eigen::Vector3d> bias_radps;
    std::optional<Eigen::Vector3d> singular_values;
    std::optional<std::uint64_t>   rank;
    std::optional<double>          condition;
    std::int64_t                   eligible_duration_ns{};
    std::uint64_t                  packet_count{};
    std::optional<double>          loss_zero_rad2;
    std::optional<double>          loss_fit_rad2;
    std::optional<bool>            nonlinear_nonincrease;
  };

  struct GyroAlignmentSupport
  {
    std::int64_t                 full_duration_ns{};
    std::int64_t                 early_duration_ns{};
    std::int64_t                 late_duration_ns{};
    std::optional<std::uint64_t> validation_blocks;
    std::optional<bool>          sufficient;
  };

  struct GyroAlignmentGates
  {
    bool                           evaluated{};
    std::optional<double>          validation_loss_zero_rad2;
    std::optional<double>          validation_loss_fit_rad2;
    std::optional<std::uint64_t>   blocks_improved;
    std::optional<std::uint64_t>   blocks_total;
    std::optional<Eigen::Vector3d> axis_loss_zero_rad2;
    std::optional<Eigen::Vector3d> axis_loss_fit_rad2;
    std::optional<double>          half_bias_max_abs_diff_radps;
    std::optional<bool>            aggregate_reduction;
    std::optional<bool>            improved_fraction;
    std::optional<bool>            axis_nonworsening;
    std::optional<bool>            half_stability;
    std::optional<bool>            all_passed;
  };

  struct GyroAlignmentBlock
  {
    std::uint64_t   block_index{};
    std::int64_t    t_start_ns{};
    std::int64_t    t_end_ns{};
    std::uint32_t   segment_id{};
    std::uint64_t   packet_count{};
    Eigen::Vector3d r_zero_rad{ Eigen::Vector3d::Zero() };
    Eigen::Vector3d r_fit_rad{ Eigen::Vector3d::Zero() };
    double          loss_zero_rad2{};
    double          loss_fit_rad2{};
    bool            improved{};
  };

  struct GyroAlignmentAnalysis
  {
    GyroAlignmentRanges             ranges;
    GyroAlignmentEligibility        eligibility;
    GyroAlignmentFit                full;
    GyroAlignmentFit                early;
    GyroAlignmentFit                late;
    GyroAlignmentSupport            support;
    GyroAlignmentGates              gates;
    std::vector<GyroAlignmentBlock> blocks;
  };

  struct GyroAlignmentVerdict
  {
    GyroAlignmentStatus              status{ GyroAlignmentStatus::kHardError };
    int                              exit_code{ 1 };
    std::vector<GyroAlignmentReason> reason_codes;
    std::string                      detail;
  };

  struct GyroAlignmentResult
  {
    GyroAlignmentVerdict                  verdict;
    std::optional<GyroAlignmentErrorCode> error;
    std::optional<GyroAlignmentAnalysis>  analysis;
  };

  [[nodiscard]] GyroAlignmentResult analyzeGyroAlignment(
      const GyroAlignmentInput& input );

}  // namespace phad::apps
