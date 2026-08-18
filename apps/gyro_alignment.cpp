#include "apps/gyro_alignment.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "phad/estimator/gyro_rotation_predictor.hpp"

namespace phad::apps
{
  namespace
  {

    constexpr std::int64_t  kSecondNs          = 1'000'000'000;
    constexpr std::int64_t  kMidOffsetNs       = 50 * kSecondNs;
    constexpr std::int64_t  kSplitOffsetNs     = 100 * kSecondNs;
    constexpr std::int64_t  kFullSupportNs     = 90 * kSecondNs;
    constexpr std::int64_t  kHalfSupportNs     = 45 * kSecondNs;
    constexpr std::uint64_t kValidationSupport = 60U;
    constexpr double        kH                 = 1.0e-5;
    constexpr double        kNsToSeconds       = 1.0e-9;
    constexpr double        kRotationTolerance = 1.0e-10;
    const double            kEpsPi =
        std::sqrt( std::numeric_limits<double>::epsilon() );

    struct Failure
    {
      GyroAlignmentErrorCode code;
      std::string            detail;
    };

    struct LogValue
    {
      Eigen::Vector3d residual;
      double          theta;
    };

    struct IntervalRow
    {
      const GyroAlignmentPacket*     packet{};
      std::size_t                    prev_endpoint{};
      std::size_t                    cur_endpoint{};
      bool                           structural{};
      bool                           fit_eligible{};
      Eigen::Matrix3d                r_vis{ Eigen::Matrix3d::Identity() };
      Eigen::Vector3d                r_zero{ Eigen::Vector3d::Zero() };
      std::optional<Eigen::Matrix3d> jacobian;
    };

    struct Precomputed
    {
      GyroAlignmentAnalysis        analysis;
      std::vector<IntervalRow>     rows;
      std::vector<Eigen::Matrix3d> rotations_WB;
    };

    using FailureOrLog         = std::variant<Failure, LogValue>;
    using FailureOrRotation    = std::variant<Failure, Eigen::Matrix3d>;
    using FailureOrPrecomputed = std::variant<Failure, Precomputed>;
    using FailureOrFit         = std::variant<Failure, GyroAlignmentFit>;

    [[nodiscard]] GyroAlignmentResult hardError( Failure failure )
    {
      return {
          .verdict  = { .status       = GyroAlignmentStatus::kHardError,
                        .exit_code    = 1,
                        .reason_codes = {},
                        .detail       = std::move( failure.detail ) },
          .error    = failure.code,
          .analysis = std::nullopt,
      };
    }

    [[nodiscard]] bool addChecked( std::int64_t lhs, std::int64_t rhs,
                                   std::int64_t& result )
    {
      if ( ( rhs > 0 &&
             lhs > std::numeric_limits<std::int64_t>::max() - rhs ) ||
           ( rhs < 0 &&
             lhs < std::numeric_limits<std::int64_t>::min() - rhs ) )
      {
        return false;
      }
      result = lhs + rhs;
      return true;
    }

    [[nodiscard]] bool subtractChecked( std::int64_t  lhs,
                                        std::int64_t  rhs,
                                        std::int64_t& result )
    {
      if ( ( rhs > 0 &&
             lhs < std::numeric_limits<std::int64_t>::min() + rhs ) ||
           ( rhs < 0 &&
             lhs > std::numeric_limits<std::int64_t>::max() + rhs ) )
      {
        return false;
      }
      result = lhs - rhs;
      return true;
    }

    bool increment( std::uint64_t& value )
    {
      if ( value == std::numeric_limits<std::uint64_t>::max() )
      {
        return false;
      }
      ++value;
      return true;
    }

    [[nodiscard]] bool addChecked( std::uint64_t lhs, std::uint64_t rhs,
                                   std::uint64_t& result )
    {
      if ( lhs > std::numeric_limits<std::uint64_t>::max() - rhs )
      {
        return false;
      }
      result = lhs + rhs;
      return true;
    }

    [[nodiscard]] bool validEndpointStatus(
        GyroAlignmentEndpointStatus status )
    {
      switch ( status )
      {
        case GyroAlignmentEndpointStatus::kOk:
        case GyroAlignmentEndpointStatus::kRejected:
        case GyroAlignmentEndpointStatus::kFailed:
          return true;
      }
      return false;
    }

    [[nodiscard]] bool validPacketStatus( GyroAlignmentPacketStatus status )
    {
      switch ( status )
      {
        case GyroAlignmentPacketStatus::kFirstZero:
        case GyroAlignmentPacketStatus::kValid:
        case GyroAlignmentPacketStatus::kGap:
        case GyroAlignmentPacketStatus::kEmptyNonfirst:
          return true;
      }
      return false;
    }

    [[nodiscard]] FailureOrLog principalLog( const Eigen::Matrix3d& q )
    {
      if ( !q.allFinite() )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "gyro alignment residual rotation is non-finite" };
      }
      const Eigen::Matrix3d orthogonality =
          q.transpose() * q - Eigen::Matrix3d::Identity();
      const double orthogonality_error = orthogonality.norm();
      const double determinant         = q.determinant();
      const double determinant_error   = std::abs( determinant - 1.0 );
      if ( !orthogonality.allFinite() ||
           !std::isfinite( orthogonality_error ) ||
           !std::isfinite( determinant ) ||
           !std::isfinite( determinant_error ) ||
           orthogonality_error > kRotationTolerance ||
           determinant_error > kRotationTolerance )
      {
        return Failure{
            GyroAlignmentErrorCode::kNonfiniteComputation,
            "gyro alignment residual rotation violates the SO(3) contract" };
      }

      const double          trace = q.trace();
      const Eigen::Vector3d v{
          0.5 * ( q( 2, 1 ) - q( 1, 2 ) ),
          0.5 * ( q( 0, 2 ) - q( 2, 0 ) ),
          0.5 * ( q( 1, 0 ) - q( 0, 1 ) ),
      };
      const double s     = v.norm();
      const double c     = std::clamp( 0.5 * ( trace - 1.0 ), -1.0, 1.0 );
      const double theta = std::atan2( s, c );
      if ( !std::isfinite( trace ) || !v.allFinite() ||
           !std::isfinite( s ) || !std::isfinite( c ) ||
           !std::isfinite( theta ) )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "principal SO(3) Log computation is non-finite" };
      }
      if ( std::numbers::pi - theta <= kEpsPi )
      {
        return LogValue{ Eigen::Vector3d::Zero(), theta };
      }
      if ( theta <= kEpsPi )
      {
        return LogValue{ v, theta };
      }
      if ( !( s > 0.0 ) )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "principal SO(3) Log has zero sine away from zero" };
      }
      const double          scale    = theta / s;
      const Eigen::Vector3d residual = scale * v;
      if ( !std::isfinite( scale ) || !residual.allFinite() )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "principal SO(3) Log result is non-finite" };
      }
      return LogValue{ residual, theta };
    }

    [[nodiscard]] FailureOrRotation integrate(
        const GyroAlignmentPacket& packet, const Eigen::Vector3d& bias )
    {
      const estimator::GyroRotationResult result =
          estimator::integrateGyroRotation( packet.samples, bias );
      if ( const auto* error =
               std::get_if<estimator::GyroRotationError>( &result ) )
      {
        return Failure{
            GyroAlignmentErrorCode::kQ2IntegrationError,
            std::string{ "Q2 gyro integration failed: " } + error->detail };
      }
      const auto& prediction =
          std::get<estimator::GyroRotationPrediction>( result );
      if ( prediction.duration_ns != packet.interval_ns )
      {
        return Failure{ GyroAlignmentErrorCode::kQ2IntegrationError,
                        "Q2 gyro integration duration disagrees with packet" };
      }
      return prediction.delta_R_i_j;
    }

    [[nodiscard]] FailureOrLog packetResidual(
        const GyroAlignmentPacket& packet, const Eigen::Matrix3d& r_vis,
        const Eigen::Vector3d& bias )
    {
      FailureOrRotation rotation = integrate( packet, bias );
      if ( const auto* failure = std::get_if<Failure>( &rotation ) )
      {
        return *failure;
      }
      const Eigen::Matrix3d q =
          std::get<Eigen::Matrix3d>( rotation ).transpose() * r_vis;
      return principalLog( q );
    }

    [[nodiscard]] Failure validateSamples( const GyroAlignmentPacket& packet,
                                           bool&                      ok )
    {
      ok                     = false;
      std::int64_t sum_dt_ns = 0;
      for ( std::size_t i = 0; i < packet.samples.size(); ++i )
      {
        const auto& sample = packet.samples[ i ];
        if ( !std::isfinite( sample.gyro_radps[ 0 ] ) ||
             !std::isfinite( sample.gyro_radps[ 1 ] ) ||
             !std::isfinite( sample.gyro_radps[ 2 ] ) )
        {
          return { GyroAlignmentErrorCode::kNonfiniteInput,
                   "gyro packet contains a non-finite sample" };
        }
        if ( i == 0U )
        {
          continue;
        }
        const std::int64_t prev_ns =
            packet.samples[ i - 1U ].timestamp.nanoseconds();
        const std::int64_t cur_ns = sample.timestamp.nanoseconds();
        if ( cur_ns <= prev_ns )
        {
          return { GyroAlignmentErrorCode::kIndexOrder,
                   "gyro sample timestamps must be strictly increasing" };
        }
        std::int64_t dt_ns = 0;
        if ( !subtractChecked( cur_ns, prev_ns, dt_ns ) ||
             !addChecked( sum_dt_ns, dt_ns, sum_dt_ns ) )
        {
          return { GyroAlignmentErrorCode::kTimestampOverflow,
                   "gyro sample duration arithmetic overflow" };
        }
      }
      if ( sum_dt_ns != packet.sum_dt_ns )
      {
        return { GyroAlignmentErrorCode::kDurationMismatch,
                 "gyro sample durations disagree with packet sum" };
      }
      ok = true;
      return {};
    }

    [[nodiscard]] FailureOrPrecomputed precompute(
        const GyroAlignmentInput& input )
    {
      if ( input.packets.empty() || input.endpoints.empty() ||
           input.packets.size() != input.endpoints.size() )
      {
        return Failure{ GyroAlignmentErrorCode::kJoinMismatch,
                        "gyro packets and endpoints must form a non-empty exact join" };
      }

      Precomputed result;
      result.rows.reserve( input.packets.size() - 1U );
      result.rotations_WB.reserve( input.endpoints.size() );

      for ( std::size_t i = 0; i < input.endpoints.size(); ++i )
      {
        const auto& endpoint = input.endpoints[ i ];
        if ( !validEndpointStatus( endpoint.status ) )
        {
          return Failure{ GyroAlignmentErrorCode::kUnknownEnum,
                          "unknown gyro alignment endpoint status" };
        }
        if ( i > 0U &&
             endpoint.timestamp_ns <= input.endpoints[ i - 1U ].timestamp_ns )
        {
          return Failure{ GyroAlignmentErrorCode::kIndexOrder,
                          "gyro alignment endpoints must be strictly ordered" };
        }
        if ( endpoint.status == GyroAlignmentEndpointStatus::kFailed )
        {
          return Failure{ GyroAlignmentErrorCode::kDiagFailed,
                          "failed diagnostic endpoint cannot be analyzed" };
        }
        if ( endpoint.status == GyroAlignmentEndpointStatus::kRejected &&
             endpoint.q_WB.has_value() )
        {
          return Failure{ GyroAlignmentErrorCode::kDiagPoseContradiction,
                          "rejected diagnostic endpoint unexpectedly has a pose" };
        }
        if ( endpoint.q_WB.has_value() )
        {
          const Eigen::Quaterniond& q    = *endpoint.q_WB;
          const double              norm = q.norm();
          if ( !q.coeffs().allFinite() || !std::isfinite( norm ) )
          {
            return Failure{ GyroAlignmentErrorCode::kNonfiniteInput,
                            "endpoint quaternion is non-finite" };
          }
          if ( std::abs( norm - 1.0 ) > 1.0e-3 )
          {
            return Failure{ GyroAlignmentErrorCode::kInvalidQuaternion,
                            "endpoint quaternion is not unit length" };
          }
          result.rotations_WB.push_back( q.normalized().toRotationMatrix() );
        }
        else
        {
          result.rotations_WB.push_back( Eigen::Matrix3d::Zero() );
        }
      }

      const GyroAlignmentPacket& first = input.packets.front();
      if ( !validPacketStatus( first.status ) )
      {
        return Failure{ GyroAlignmentErrorCode::kUnknownEnum,
                        "unknown gyro alignment packet status" };
      }
      if ( first.packet_index != 0U ||
           first.status != GyroAlignmentPacketStatus::kFirstZero ||
           first.t_prev_ns != first.t_cur_ns || first.imu_gap ||
           !first.samples.empty() || first.sum_dt_ns != 0 ||
           first.interval_ns != 0 )
      {
        return Failure{ GyroAlignmentErrorCode::kInputSchema,
                        "packet zero must be the unique first_zero sentinel" };
      }
      if ( first.t_cur_ns != input.endpoints.front().timestamp_ns ||
           first.segment_id != input.endpoints.front().segment_id )
      {
        return Failure{ GyroAlignmentErrorCode::kJoinMismatch,
                        "first_zero does not join its endpoint" };
      }

      result.analysis.ranges.t0_ns = first.t_cur_ns;
      if ( !addChecked( first.t_cur_ns, kMidOffsetNs,
                        result.analysis.ranges.t_mid_ns ) ||
           !addChecked( first.t_cur_ns, kSplitOffsetNs,
                        result.analysis.ranges.t_split_ns ) )
      {
        return Failure{ GyroAlignmentErrorCode::kTimestampOverflow,
                        "sentinel-anchored split arithmetic overflow" };
      }

      std::int64_t max_nonfirst_t_cur_ns = first.t_cur_ns;
      for ( std::size_t i = 1U; i < input.packets.size(); ++i )
      {
        const auto& packet = input.packets[ i ];
        if ( !validPacketStatus( packet.status ) )
        {
          return Failure{ GyroAlignmentErrorCode::kUnknownEnum,
                          "unknown gyro alignment packet status" };
        }
        if ( packet.status == GyroAlignmentPacketStatus::kFirstZero )
        {
          return Failure{ GyroAlignmentErrorCode::kInputSchema,
                          "first_zero may only appear at packet zero" };
        }
        if ( packet.packet_index != static_cast<std::uint64_t>( i ) ||
             packet.t_cur_ns <= input.packets[ i - 1U ].t_cur_ns )
        {
          return Failure{ GyroAlignmentErrorCode::kIndexOrder,
                          "gyro packet index or timestamp order is invalid" };
        }
        if ( packet.t_cur_ns != input.endpoints[ i ].timestamp_ns ||
             packet.segment_id != input.endpoints[ i ].segment_id )
        {
          return Failure{ GyroAlignmentErrorCode::kJoinMismatch,
                          "gyro packet current endpoint join mismatch" };
        }
        if ( packet.status == GyroAlignmentPacketStatus::kValid &&
             packet.t_prev_ns != input.endpoints[ i - 1U ].timestamp_ns )
        {
          return Failure{
              GyroAlignmentErrorCode::kJoinMismatch,
              "valid gyro packet must join the immediately preceding endpoint" };
        }
        if ( packet.t_prev_ns >= packet.t_cur_ns )
        {
          return Failure{ GyroAlignmentErrorCode::kDurationMismatch,
                          "nonfirst gyro packet must have positive duration" };
        }
        std::int64_t interval_ns = 0;
        if ( !subtractChecked( packet.t_cur_ns, packet.t_prev_ns,
                               interval_ns ) )
        {
          return Failure{ GyroAlignmentErrorCode::kTimestampOverflow,
                          "gyro packet interval arithmetic overflow" };
        }
        if ( interval_ns != packet.interval_ns )
        {
          return Failure{ GyroAlignmentErrorCode::kDurationMismatch,
                          "gyro packet interval does not match endpoints" };
        }
        if ( packet.status == GyroAlignmentPacketStatus::kValid )
        {
          if ( packet.imu_gap || packet.samples.size() < 2U )
          {
            return Failure{ GyroAlignmentErrorCode::kCountMismatch,
                            "valid gyro packet has invalid gap/sample shape" };
          }
          if ( packet.samples.front().timestamp.nanoseconds() !=
                   packet.t_prev_ns ||
               packet.samples.back().timestamp.nanoseconds() !=
                   packet.t_cur_ns )
          {
            return Failure{ GyroAlignmentErrorCode::kEndpointMismatch,
                            "valid gyro packet samples do not match endpoints" };
          }
          if ( packet.sum_dt_ns != packet.interval_ns )
          {
            return Failure{ GyroAlignmentErrorCode::kDurationMismatch,
                            "valid gyro packet duration does not match interval" };
          }
        }
        else if ( packet.status == GyroAlignmentPacketStatus::kGap )
        {
          if ( !packet.imu_gap )
          {
            return Failure{ GyroAlignmentErrorCode::kInputSchema,
                            "gap packet must set imu_gap" };
          }
        }
        else if ( packet.imu_gap || !packet.samples.empty() ||
                  packet.sum_dt_ns != 0 )
        {
          return Failure{ GyroAlignmentErrorCode::kInputSchema,
                          "empty_nonfirst packet has invalid sample shape" };
        }
        bool    samples_ok     = false;
        Failure sample_failure = validateSamples( packet, samples_ok );
        if ( !samples_ok )
        {
          return sample_failure;
        }

        max_nonfirst_t_cur_ns = packet.t_cur_ns;
        if ( !increment( result.analysis.eligibility.packet_total_nonfirst ) )
        {
          return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                          "eligibility census overflow" };
        }

        IntervalRow row{ .packet = &packet, .jacobian = std::nullopt };
        if ( packet.status == GyroAlignmentPacketStatus::kGap )
        {
          increment( result.analysis.eligibility.interval_excluded_counts.gap );
          result.rows.push_back( row );
          continue;
        }
        if ( packet.status == GyroAlignmentPacketStatus::kEmptyNonfirst )
        {
          increment( result.analysis.eligibility.interval_excluded_counts
                         .empty_nonfirst );
          result.rows.push_back( row );
          continue;
        }

        const auto prev_it = std::lower_bound(
            input.endpoints.begin(), input.endpoints.end(), packet.t_prev_ns,
            []( const GyroAlignmentEndpoint& endpoint, std::int64_t value ) {
              return endpoint.timestamp_ns < value;
            } );
        if ( prev_it == input.endpoints.end() ||
             prev_it->timestamp_ns != packet.t_prev_ns )
        {
          return Failure{ GyroAlignmentErrorCode::kJoinMismatch,
                          "valid packet previous endpoint is missing" };
        }
        row.prev_endpoint = static_cast<std::size_t>(
            std::distance( input.endpoints.begin(), prev_it ) );
        row.cur_endpoint = i;
        const auto& prev = input.endpoints[ row.prev_endpoint ];
        const auto& cur  = input.endpoints[ row.cur_endpoint ];
        if ( prev.status == GyroAlignmentEndpointStatus::kRejected ||
             cur.status == GyroAlignmentEndpointStatus::kRejected ||
             !prev.q_WB.has_value() || !cur.q_WB.has_value() )
        {
          increment( result.analysis.eligibility.interval_excluded_counts
                         .diag_rejected_no_pose );
          result.rows.push_back( row );
          continue;
        }
        if ( prev.segment_id != cur.segment_id )
        {
          increment( result.analysis.eligibility.interval_excluded_counts
                         .segment_boundary );
          result.rows.push_back( row );
          continue;
        }

        row.structural = true;
        increment( result.analysis.eligibility.packet_structurally_eligible );
        row.r_vis =
            result.rotations_WB[ row.prev_endpoint ].transpose() *
            result.rotations_WB[ row.cur_endpoint ];
        FailureOrLog center = packetResidual(
            packet, row.r_vis, Eigen::Vector3d::Zero() );
        if ( const auto* failure = std::get_if<Failure>( &center ) )
        {
          return *failure;
        }
        const LogValue center_value = std::get<LogValue>( center );
        const double   duration_s =
            static_cast<double>( packet.interval_ns ) * kNsToSeconds;
        const double h_duration = kH * duration_s;
        const double margin     = std::nextafter(
            h_duration, std::numeric_limits<double>::infinity() );
        if ( !std::isfinite( duration_s ) || !( duration_s > 0.0 ) ||
             !std::isfinite( h_duration ) || !std::isfinite( margin ) )
        {
          return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                          "finite-difference guard computation is non-finite" };
        }
        if ( std::numbers::pi - center_value.theta <= kEpsPi + margin )
        {
          increment( result.analysis.eligibility.interval_excluded_counts
                         .near_pi );
          result.rows.push_back( row );
          continue;
        }

        row.r_zero = center_value.residual;
        row.jacobian.emplace( Eigen::Matrix3d::Zero() );
        for ( Eigen::Index axis = 0; axis < 3; ++axis )
        {
          Eigen::Vector3d plus     = Eigen::Vector3d::Zero();
          Eigen::Vector3d minus    = Eigen::Vector3d::Zero();
          plus( axis )             = kH;
          minus( axis )            = -kH;
          FailureOrLog plus_value  = packetResidual( packet, row.r_vis, plus );
          FailureOrLog minus_value = packetResidual( packet, row.r_vis, minus );
          if ( const auto* failure = std::get_if<Failure>( &plus_value ) )
          {
            return *failure;
          }
          if ( const auto* failure = std::get_if<Failure>( &minus_value ) )
          {
            return *failure;
          }
          const LogValue plus_log  = std::get<LogValue>( plus_value );
          const LogValue minus_log = std::get<LogValue>( minus_value );
          if ( std::numbers::pi - plus_log.theta <= kEpsPi ||
               std::numbers::pi - minus_log.theta <= kEpsPi )
          {
            return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                            "finite-difference arm reached the principal Log boundary" };
          }
          row.jacobian->col( axis ) =
              ( plus_log.residual - minus_log.residual ) / ( 2.0 * kH );
        }
        if ( !row.jacobian->allFinite() )
        {
          return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                          "finite-difference Jacobian is non-finite" };
        }
        row.fit_eligible = true;
        increment( result.analysis.eligibility.packet_fit_eligible );
        result.rows.push_back( row );
      }

      if ( input.packets.size() == 1U ||
           max_nonfirst_t_cur_ns <= result.analysis.ranges.t_split_ns )
      {
        result.analysis.eligibility.validation_slots_total = 0U;
      }
      else
      {
        std::int64_t delta_ns = 0;
        if ( !subtractChecked( max_nonfirst_t_cur_ns,
                               result.analysis.ranges.t_split_ns,
                               delta_ns ) )
        {
          return Failure{ GyroAlignmentErrorCode::kTimestampOverflow,
                          "validation range subtraction overflow" };
        }
        const std::int64_t slots = 1 + ( delta_ns - 1 ) / kSecondNs;
        result.analysis.eligibility.validation_slots_total =
            static_cast<std::uint64_t>( slots );
        if ( slots > std::numeric_limits<std::int64_t>::max() / kSecondNs ||
             !addChecked( result.analysis.ranges.t_split_ns,
                          slots * kSecondNs,
                          max_nonfirst_t_cur_ns ) )
        {
          return Failure{ GyroAlignmentErrorCode::kTimestampOverflow,
                          "validation range end arithmetic overflow" };
        }
        result.analysis.ranges.validation_last_end_ns =
            max_nonfirst_t_cur_ns;
      }


      std::uint64_t structural_excluded = 0U;
      const auto&   interval_counts =
          result.analysis.eligibility.interval_excluded_counts;
      if ( !addChecked( interval_counts.gap,
                        interval_counts.empty_nonfirst,
                        structural_excluded ) ||
           !addChecked( structural_excluded,
                        interval_counts.diag_rejected_no_pose,
                        structural_excluded ) ||
           !addChecked( structural_excluded,
                        interval_counts.segment_boundary,
                        structural_excluded ) ||
           !addChecked( structural_excluded,
                        result.analysis.eligibility
                            .packet_structurally_eligible,
                        structural_excluded ) ||
           structural_excluded !=
               result.analysis.eligibility.packet_total_nonfirst )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "structural eligibility census invariant failed" };
      }
      std::uint64_t fit_census = 0U;
      if ( !addChecked( result.analysis.eligibility.packet_fit_eligible,
                        interval_counts.near_pi, fit_census ) ||
           fit_census !=
               result.analysis.eligibility.packet_structurally_eligible )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "fit eligibility census invariant failed" };
      }
      return result;
    }

    [[nodiscard]] double orderedLoss( const Eigen::Vector3d& residual )
    {
      return ( residual.x() * residual.x() +
               residual.y() * residual.y() ) +
             residual.z() * residual.z();
    }

    [[nodiscard]] FailureOrFit solveFit(
        std::span<const IntervalRow* const> rows,
        std::int64_t                        minimum_duration_ns )
    {
      GyroAlignmentFit fit;
      for ( const IntervalRow* row : rows )
      {
        if ( !addChecked( fit.eligible_duration_ns,
                          row->packet->interval_ns,
                          fit.eligible_duration_ns ) )
        {
          return Failure{ GyroAlignmentErrorCode::kTimestampOverflow,
                          "fit duration accumulation overflow" };
        }
        if ( !increment( fit.packet_count ) )
        {
          return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                          "fit packet count overflow" };
        }
      }
      if ( fit.eligible_duration_ns < minimum_duration_ns )
      {
        fit.status = GyroAlignmentFitStatus::kSupportInsufficient;
        return fit;
      }

      if ( rows.size() >
           static_cast<std::size_t>(
               std::numeric_limits<Eigen::Index>::max() / 3 ) )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "fit matrix is too large" };
      }
      const Eigen::Index matrix_rows =
          static_cast<Eigen::Index>( rows.size() * 3U );
      Eigen::VectorXd residual_zero( matrix_rows );
      Eigen::MatrixXd jacobian( matrix_rows, 3 );
      for ( std::size_t i = 0; i < rows.size(); ++i )
      {
        const Eigen::Index offset          = static_cast<Eigen::Index>( i * 3U );
        residual_zero.segment<3>( offset ) = rows[ i ]->r_zero;
        if ( !rows[ i ]->jacobian.has_value() )
        {
          return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                          "fit-eligible row is missing its Jacobian" };
        }
        jacobian.block<3, 3>( offset, 0 ) = *rows[ i ]->jacobian;
      }
      if ( !residual_zero.allFinite() || !jacobian.allFinite() )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "fit matrix contains non-finite values" };
      }

      const Eigen::JacobiSVD<Eigen::MatrixXd> svd{
          jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV };
      const Eigen::VectorXd singular = svd.singularValues();
      if ( singular.size() != 3 || !singular.allFinite() ||
           !( singular( 0 ) > 0.0 ) )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "SVD produced invalid singular values" };
      }
      fit.singular_values = singular;
      const double cutoff = 3.0 * std::numeric_limits<double>::epsilon() *
                            singular( 0 );
      if ( !std::isfinite( cutoff ) )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "SVD rank cutoff is non-finite" };
      }
      std::uint64_t rank = 0U;
      for ( Eigen::Index i = 0; i < singular.size(); ++i )
      {
        if ( singular( i ) > cutoff )
        {
          ++rank;
        }
      }
      fit.rank = rank;
      if ( rank != 3U )
      {
        fit.status = GyroAlignmentFitStatus::kRankDeficient;
        return fit;
      }
      const double condition = singular( 0 ) / singular( 2 );
      if ( !std::isfinite( condition ) )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "SVD condition is non-finite" };
      }
      fit.condition = condition;
      if ( condition > 1.0e6 )
      {
        fit.status = GyroAlignmentFitStatus::kIllConditioned;
        return fit;
      }

      const Eigen::Vector3d bias = svd.solve( -residual_zero );
      if ( !bias.allFinite() )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "least-squares candidate bias is non-finite" };
      }
      double loss_zero = 0.0;
      double loss_fit  = 0.0;
      for ( const IntervalRow* row : rows )
      {
        const double zero = orderedLoss( row->r_zero );
        if ( !std::isfinite( zero ) ||
             !std::isfinite( loss_zero + zero ) )
        {
          return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                          "zero-bias prefix loss is non-finite" };
        }
        loss_zero += zero;
        FailureOrLog candidate =
            packetResidual( *row->packet, row->r_vis, bias );
        if ( const auto* failure = std::get_if<Failure>( &candidate ) )
        {
          return *failure;
        }
        const LogValue candidate_value = std::get<LogValue>( candidate );
        if ( std::numbers::pi - candidate_value.theta <= kEpsPi )
        {
          fit.status = GyroAlignmentFitStatus::kObservabilityFailed;
          return fit;
        }
        const double candidate_loss = orderedLoss( candidate_value.residual );
        if ( !std::isfinite( candidate_loss ) ||
             !std::isfinite( loss_fit + candidate_loss ) )
        {
          return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                          "candidate-bias prefix loss is non-finite" };
        }
        loss_fit += candidate_loss;
      }

      fit.bias_radps            = bias;
      fit.loss_zero_rad2        = loss_zero;
      fit.loss_fit_rad2         = loss_fit;
      fit.nonlinear_nonincrease = loss_fit <= loss_zero;
      fit.status                = GyroAlignmentFitStatus::kSolved;
      return fit;
    }

    [[nodiscard]] FailureOrLog blockResidual(
        std::span<const IntervalRow* const> rows,
        const std::vector<Eigen::Matrix3d>& rotations_WB,
        const Eigen::Vector3d&              bias )
    {
      if ( rows.empty() )
      {
        return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                        "cannot evaluate an empty validation block" };
      }
      Eigen::Matrix3d r_imu = Eigen::Matrix3d::Identity();
      for ( const IntervalRow* row : rows )
      {
        FailureOrRotation packet_rotation = integrate( *row->packet, bias );
        if ( const auto* failure =
                 std::get_if<Failure>( &packet_rotation ) )
        {
          return *failure;
        }
        r_imu = r_imu * std::get<Eigen::Matrix3d>( packet_rotation );
        if ( !r_imu.allFinite() )
        {
          return Failure{ GyroAlignmentErrorCode::kNonfiniteComputation,
                          "validation rotation composition is non-finite" };
        }
      }
      const Eigen::Matrix3d r_vis =
          rotations_WB[ rows.front()->prev_endpoint ].transpose() *
          rotations_WB[ rows.back()->cur_endpoint ];
      return principalLog( r_imu.transpose() * r_vis );
    }

    [[nodiscard]] bool hasStatus( const std::array<const GyroAlignmentFit*, 3>& fits,
                                  GyroAlignmentFitStatus                        status )
    {
      return std::any_of( fits.begin(), fits.end(),
                          [ status ]( const GyroAlignmentFit* fit ) {
                            return fit->status == status;
                          } );
    }

  }  // namespace

  GyroAlignmentResult analyzeGyroAlignment( const GyroAlignmentInput& input )
  {
    FailureOrPrecomputed precomputed = precompute( input );
    if ( const auto* failure = std::get_if<Failure>( &precomputed ) )
    {
      return hardError( *failure );
    }
    Precomputed state = std::get<Precomputed>( std::move( precomputed ) );

    std::vector<const IntervalRow*> full_rows;
    std::vector<const IntervalRow*> early_rows;
    std::vector<const IntervalRow*> late_rows;
    full_rows.reserve( state.rows.size() );
    early_rows.reserve( state.rows.size() );
    late_rows.reserve( state.rows.size() );
    for ( const IntervalRow& row : state.rows )
    {
      if ( !row.fit_eligible )
      {
        continue;
      }
      const GyroAlignmentPacket& packet = *row.packet;
      if ( packet.t_prev_ns >= state.analysis.ranges.t0_ns &&
           packet.t_cur_ns <= state.analysis.ranges.t_split_ns )
      {
        full_rows.push_back( &row );
      }
      if ( packet.t_prev_ns >= state.analysis.ranges.t0_ns &&
           packet.t_cur_ns <= state.analysis.ranges.t_mid_ns )
      {
        early_rows.push_back( &row );
      }
      if ( packet.t_prev_ns >= state.analysis.ranges.t_mid_ns &&
           packet.t_cur_ns <= state.analysis.ranges.t_split_ns )
      {
        late_rows.push_back( &row );
      }
    }

    FailureOrFit full = solveFit( full_rows, kFullSupportNs );
    if ( const auto* failure = std::get_if<Failure>( &full ) )
    {
      return hardError( *failure );
    }
    state.analysis.full = std::get<GyroAlignmentFit>( std::move( full ) );
    FailureOrFit early  = solveFit( early_rows, kHalfSupportNs );
    if ( const auto* failure = std::get_if<Failure>( &early ) )
    {
      return hardError( *failure );
    }
    state.analysis.early = std::get<GyroAlignmentFit>( std::move( early ) );
    FailureOrFit late    = solveFit( late_rows, kHalfSupportNs );
    if ( const auto* failure = std::get_if<Failure>( &late ) )
    {
      return hardError( *failure );
    }
    state.analysis.late = std::get<GyroAlignmentFit>( std::move( late ) );

    state.analysis.support.full_duration_ns =
        state.analysis.full.eligible_duration_ns;
    state.analysis.support.early_duration_ns =
        state.analysis.early.eligible_duration_ns;
    state.analysis.support.late_duration_ns =
        state.analysis.late.eligible_duration_ns;

    std::vector<std::vector<const IntervalRow*>> complete_blocks;
    complete_blocks.reserve( static_cast<std::size_t>(
        state.analysis.eligibility.validation_slots_total ) );
    std::uint64_t incomplete_blocks = 0U;
    for ( std::uint64_t block_index = 0U;
          block_index < state.analysis.eligibility.validation_slots_total;
          ++block_index )
    {
      const auto   block_index_ns = static_cast<std::int64_t>( block_index );
      std::int64_t t_start_ns     = 0;
      std::int64_t t_end_ns       = 0;
      if ( block_index_ns >
               std::numeric_limits<std::int64_t>::max() / kSecondNs ||
           !addChecked( state.analysis.ranges.t_split_ns,
                        block_index_ns * kSecondNs, t_start_ns ) ||
           !addChecked( t_start_ns, kSecondNs, t_end_ns ) )
      {
        return hardError(
            { GyroAlignmentErrorCode::kTimestampOverflow,
              "validation block boundary arithmetic overflow" } );
      }

      std::vector<const IntervalRow*> block_rows;
      for ( const IntervalRow& row : state.rows )
      {
        if ( row.packet->t_prev_ns >= t_start_ns &&
             row.packet->t_cur_ns <= t_end_ns )
        {
          block_rows.push_back( &row );
        }
      }
      bool complete = !block_rows.empty() &&
                      block_rows.front()->packet->t_prev_ns == t_start_ns &&
                      block_rows.back()->packet->t_cur_ns == t_end_ns;
      std::int64_t                 cursor = t_start_ns;
      std::optional<std::uint32_t> segment_id;
      for ( const IntervalRow* row : block_rows )
      {
        complete = complete && row->structural &&
                   row->packet->t_prev_ns == cursor;
        cursor = row->packet->t_cur_ns;
        if ( !segment_id.has_value() )
        {
          segment_id = row->packet->segment_id;
        }
        else
        {
          complete = complete && *segment_id == row->packet->segment_id;
        }
      }
      complete = complete && cursor == t_end_ns;
      if ( !complete )
      {
        if ( !increment( incomplete_blocks ) )
        {
          return hardError(
              { GyroAlignmentErrorCode::kNonfiniteComputation,
                "validation exclusion count overflow" } );
        }
        complete_blocks.emplace_back();
      }
      else
      {
        complete_blocks.push_back( std::move( block_rows ) );
      }
    }
    state.analysis.eligibility.block_excluded_counts
        .incomplete_calendar_block = incomplete_blocks;

    if ( state.analysis.full.status == GyroAlignmentFitStatus::kSolved )
    {
      const Eigen::Vector3d full_bias       = *state.analysis.full.bias_radps;
      std::uint64_t         near_pi_blocks  = 0U;
      std::uint64_t         eligible_blocks = 0U;
      for ( std::size_t block_index = 0U;
            block_index < complete_blocks.size(); ++block_index )
      {
        const auto& rows = complete_blocks[ block_index ];
        if ( rows.empty() )
        {
          continue;
        }
        FailureOrLog zero = blockResidual(
            rows, state.rotations_WB, Eigen::Vector3d::Zero() );
        if ( const auto* failure = std::get_if<Failure>( &zero ) )
        {
          return hardError( *failure );
        }
        FailureOrLog fitted =
            blockResidual( rows, state.rotations_WB, full_bias );
        if ( const auto* failure = std::get_if<Failure>( &fitted ) )
        {
          return hardError( *failure );
        }
        const LogValue zero_value = std::get<LogValue>( zero );
        const LogValue fit_value  = std::get<LogValue>( fitted );
        if ( std::numbers::pi - zero_value.theta <= kEpsPi ||
             std::numbers::pi - fit_value.theta <= kEpsPi )
        {
          if ( !increment( near_pi_blocks ) )
          {
            return hardError(
                { GyroAlignmentErrorCode::kNonfiniteComputation,
                  "validation near-pi count overflow" } );
          }
          continue;
        }
        const double loss_zero = orderedLoss( zero_value.residual );
        const double loss_fit  = orderedLoss( fit_value.residual );
        if ( !std::isfinite( loss_zero ) || !std::isfinite( loss_fit ) )
        {
          return hardError(
              { GyroAlignmentErrorCode::kNonfiniteComputation,
                "validation block loss is non-finite" } );
        }
        GyroAlignmentBlock block{
            .block_index    = static_cast<std::uint64_t>( block_index ),
            .t_start_ns     = rows.front()->packet->t_prev_ns,
            .t_end_ns       = rows.back()->packet->t_cur_ns,
            .segment_id     = rows.front()->packet->segment_id,
            .packet_count   = static_cast<std::uint64_t>( rows.size() ),
            .r_zero_rad     = zero_value.residual,
            .r_fit_rad      = fit_value.residual,
            .loss_zero_rad2 = loss_zero,
            .loss_fit_rad2  = loss_fit,
            .improved       = loss_fit <= loss_zero,
        };
        state.analysis.blocks.push_back( std::move( block ) );
        if ( !increment( eligible_blocks ) )
        {
          return hardError(
              { GyroAlignmentErrorCode::kNonfiniteComputation,
                "validation eligible block count overflow" } );
        }
      }
      state.analysis.eligibility.validation_blocks_eligible =
          eligible_blocks;
      state.analysis.eligibility.block_excluded_counts.near_pi =
          near_pi_blocks;
      state.analysis.support.validation_blocks = eligible_blocks;
      state.analysis.support.sufficient =
          state.analysis.full.eligible_duration_ns >= kFullSupportNs &&
          state.analysis.early.eligible_duration_ns >= kHalfSupportNs &&
          state.analysis.late.eligible_duration_ns >= kHalfSupportNs &&
          eligible_blocks >= kValidationSupport;
      std::uint64_t block_census = 0U;
      if ( !addChecked( incomplete_blocks, near_pi_blocks, block_census ) ||
           !addChecked( block_census, eligible_blocks, block_census ) ||
           block_census !=
               state.analysis.eligibility.validation_slots_total )
      {
        return hardError(
            { GyroAlignmentErrorCode::kNonfiniteComputation,
              "validation block census invariant failed" } );
      }
    }

    const std::array<const GyroAlignmentFit*, 3> fits{
        &state.analysis.full, &state.analysis.early, &state.analysis.late };
    const bool all_fits_solved = std::all_of(
        fits.begin(), fits.end(), []( const GyroAlignmentFit* fit ) {
          return fit->status == GyroAlignmentFitStatus::kSolved;
        } );
    if ( all_fits_solved && state.analysis.support.sufficient == true )
    {
      GyroAlignmentGates& gates = state.analysis.gates;
      gates.evaluated           = true;
      double          loss_zero = 0.0;
      double          loss_fit  = 0.0;
      Eigen::Vector3d axis_zero = Eigen::Vector3d::Zero();
      Eigen::Vector3d axis_fit  = Eigen::Vector3d::Zero();
      std::uint64_t   improved  = 0U;
      for ( const GyroAlignmentBlock& block : state.analysis.blocks )
      {
        if ( !std::isfinite( loss_zero + block.loss_zero_rad2 ) ||
             !std::isfinite( loss_fit + block.loss_fit_rad2 ) )
        {
          return hardError(
              { GyroAlignmentErrorCode::kNonfiniteComputation,
                "validation loss accumulation is non-finite" } );
        }
        loss_zero += block.loss_zero_rad2;
        loss_fit += block.loss_fit_rad2;
        for ( Eigen::Index axis = 0; axis < 3; ++axis )
        {
          const double zero_axis =
              block.r_zero_rad( axis ) * block.r_zero_rad( axis );
          const double fit_axis =
              block.r_fit_rad( axis ) * block.r_fit_rad( axis );
          if ( !std::isfinite( axis_zero( axis ) + zero_axis ) ||
               !std::isfinite( axis_fit( axis ) + fit_axis ) )
          {
            return hardError(
                { GyroAlignmentErrorCode::kNonfiniteComputation,
                  "validation axis loss accumulation is non-finite" } );
          }
          axis_zero( axis ) += zero_axis;
          axis_fit( axis ) += fit_axis;
        }
        if ( block.improved && !increment( improved ) )
        {
          return hardError(
              { GyroAlignmentErrorCode::kNonfiniteComputation,
                "validation improved count overflow" } );
        }
      }
      const double half_diff =
          ( *state.analysis.early.bias_radps -
            *state.analysis.late.bias_radps )
              .cwiseAbs()
              .maxCoeff();
      if ( !std::isfinite( loss_zero ) || !std::isfinite( loss_fit ) ||
           !axis_zero.allFinite() || !axis_fit.allFinite() ||
           !std::isfinite( half_diff ) )
      {
        return hardError(
            { GyroAlignmentErrorCode::kNonfiniteComputation,
              "gyro alignment gate metric is non-finite" } );
      }
      gates.validation_loss_zero_rad2 = loss_zero;
      gates.validation_loss_fit_rad2  = loss_fit;
      gates.blocks_improved           = improved;
      gates.blocks_total =
          static_cast<std::uint64_t>( state.analysis.blocks.size() );
      gates.axis_loss_zero_rad2          = axis_zero;
      gates.axis_loss_fit_rad2           = axis_fit;
      gates.half_bias_max_abs_diff_radps = half_diff;
      gates.aggregate_reduction =
          loss_zero > 0.0 && loss_fit <= 0.8 * loss_zero;
      if ( improved > std::numeric_limits<std::uint64_t>::max() / 3U ||
           *gates.blocks_total >
               std::numeric_limits<std::uint64_t>::max() / 2U )
      {
        return hardError(
            { GyroAlignmentErrorCode::kNonfiniteComputation,
              "validation fraction arithmetic overflow" } );
      }
      gates.improved_fraction =
          3U * improved >= 2U * *gates.blocks_total;
      gates.axis_nonworsening =
          ( axis_fit.array() <= axis_zero.array() ).all();
      gates.half_stability = half_diff <= 1.0e-3;
      gates.all_passed     = *gates.aggregate_reduction &&
                         *gates.improved_fraction &&
                         *gates.axis_nonworsening &&
                         *gates.half_stability;
    }

    GyroAlignmentVerdict verdict;
    verdict.reason_codes.clear();
    const bool fit_support =
        hasStatus( fits, GyroAlignmentFitStatus::kSupportInsufficient );
    const bool validation_support =
        all_fits_solved &&
        state.analysis.support.validation_blocks.has_value() &&
        *state.analysis.support.validation_blocks < kValidationSupport;
    const bool fit_rank =
        hasStatus( fits, GyroAlignmentFitStatus::kRankDeficient );
    const bool fit_condition =
        hasStatus( fits, GyroAlignmentFitStatus::kIllConditioned );
    const bool fit_observability =
        hasStatus( fits, GyroAlignmentFitStatus::kObservabilityFailed );
    const bool prefix_loss = std::any_of(
        fits.begin(), fits.end(), []( const GyroAlignmentFit* fit ) {
          return fit->status == GyroAlignmentFitStatus::kSolved &&
                 fit->nonlinear_nonincrease == false;
        } );
    if ( fit_support )
    {
      verdict.reason_codes.push_back( GyroAlignmentReason::kFitSupport );
    }
    if ( validation_support )
    {
      verdict.reason_codes.push_back(
          GyroAlignmentReason::kValidationSupport );
    }
    if ( fit_rank )
    {
      verdict.reason_codes.push_back( GyroAlignmentReason::kFitRank );
    }
    if ( fit_condition )
    {
      verdict.reason_codes.push_back( GyroAlignmentReason::kFitCondition );
    }
    if ( fit_observability )
    {
      verdict.reason_codes.push_back(
          GyroAlignmentReason::kFitObservability );
    }
    if ( prefix_loss )
    {
      verdict.reason_codes.push_back(
          GyroAlignmentReason::kPrefixNonlinearLoss );
    }
    if ( state.analysis.gates.evaluated )
    {
      if ( state.analysis.gates.aggregate_reduction == false )
      {
        verdict.reason_codes.push_back(
            GyroAlignmentReason::kValidationAggregate );
      }
      if ( state.analysis.gates.improved_fraction == false )
      {
        verdict.reason_codes.push_back(
            GyroAlignmentReason::kValidationFraction );
      }
      if ( state.analysis.gates.axis_nonworsening == false )
      {
        verdict.reason_codes.push_back(
            GyroAlignmentReason::kValidationAxis );
      }
      if ( state.analysis.gates.half_stability == false )
      {
        verdict.reason_codes.push_back(
            GyroAlignmentReason::kHalfStability );
      }
    }

    const bool inconclusive = fit_support || validation_support || fit_rank ||
                              fit_condition || fit_observability;
    const bool hypothesis_failure =
        prefix_loss ||
        ( state.analysis.gates.evaluated &&
          state.analysis.gates.all_passed == false );
    if ( inconclusive )
    {
      verdict.status    = GyroAlignmentStatus::kInconclusive;
      verdict.exit_code = 3;
      verdict.detail    = "gyro alignment support or observability is insufficient";
    }
    else if ( hypothesis_failure )
    {
      verdict.status    = GyroAlignmentStatus::kHypothesisFail;
      verdict.exit_code = 2;
      verdict.detail    = "gyro alignment scientific gates did not all pass";
    }
    else
    {
      verdict.status    = GyroAlignmentStatus::kPass;
      verdict.exit_code = 0;
      verdict.detail    = "all gyro alignment gates passed";
    }

    return { .verdict  = std::move( verdict ),
             .error    = std::nullopt,
             .analysis = std::move( state.analysis ) };
  }

}  // namespace phad::apps
