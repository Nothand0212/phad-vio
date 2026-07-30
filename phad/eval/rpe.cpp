#include "phad/eval/rpe.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <numbers>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

/**
 * @file rpe.cpp
 * @brief RPE 计算实现。
 */

namespace phad::eval
{
  namespace
  {

    /**
     * @brief 在 index 之后找时间戳最接近 target 的匹配对下标。
     *
     * 只向后搜索，因此间隔容差大于间隔本身时也不会把某个位姿与自身或与
     * 更早的位姿配对。timestamps 严格递增。
     */
    std::optional<std::size_t> findPartner(
        const std::vector<std::int64_t>& timestamps, std::size_t index,
        std::int64_t target, std::int64_t tolerance )
    {
      const auto begin =
          timestamps.begin() + static_cast<std::ptrdiff_t>( index + 1U );
      const auto upper = std::lower_bound( begin, timestamps.end(), target );
      auto       best  = upper;
      if ( upper == timestamps.end() )
      {
        if ( upper == begin )
        {
          return std::nullopt;
        }
        best = std::prev( upper );
      }
      else if ( upper != begin )
      {
        const auto lower = std::prev( upper );
        if ( target - *lower < *upper - target )
        {
          best = lower;
        }
      }
      if ( std::abs( *best - target ) > tolerance )
      {
        return std::nullopt;
      }
      return static_cast<std::size_t>(
          std::distance( timestamps.begin(), best ) );
    }

    double rotationAngleDeg( const Eigen::Isometry3d& transform )
    {
      const Eigen::AngleAxisd angle_axis{ transform.linear() };
      return angle_axis.angle() * 180.0 / std::numbers::pi;
    }

  }  // namespace

  EvalResult<RpeReport> computeRpe( const common::Trajectory& est,
                                    const common::Trajectory& gt,
                                    const RpeOptions&         options )
  {
    if ( options.delta_ns <= 0 || options.delta_tolerance_ns < 0 ||
         options.delta_tolerance_ns >= options.delta_ns )
    {
      std::ostringstream cause;
      cause << "delta must be positive and the tolerance must lie in [0, delta), "
               "got delta "
            << options.delta_ns << " ns and tolerance "
            << options.delta_tolerance_ns << " ns";
      return EvalError{ EvalErrorCode::kInvalidOptions, {}, std::nullopt, {}, std::move( cause ).str() };
    }

    auto association = associate( est, gt, options.association );
    if ( !association )
    {
      return association.error();
    }

    RpeReport report;
    report.delta_ns    = options.delta_ns;
    report.association = std::move( association ).value();

    std::vector<std::int64_t> timestamps;
    timestamps.reserve( report.association.pairs.size() );
    for ( const MatchedPair& pair : report.association.pairs )
    {
      timestamps.push_back(
          est.poses()[ pair.est_index ].timestamp.nanoseconds() );
    }

    std::vector<double> trans_errors;
    std::vector<double> rot_errors;
    for ( std::size_t index = 0; index < timestamps.size(); ++index )
    {
      const std::optional<std::size_t> partner =
          findPartner( timestamps, index, timestamps[ index ] + options.delta_ns,
                       options.delta_tolerance_ns );
      if ( !partner.has_value() )
      {
        ++report.dropped_no_partner;
        continue;
      }

      const MatchedPair&      from = report.association.pairs[ index ];
      const MatchedPair&      to   = report.association.pairs[ *partner ];
      const Eigen::Isometry3d est_motion =
          est.poses()[ from.est_index ].T_W_B.inverse() *
          est.poses()[ to.est_index ].T_W_B;
      const Eigen::Isometry3d gt_motion =
          gt.poses()[ from.gt_index ].T_W_B.inverse() *
          gt.poses()[ to.gt_index ].T_W_B;
      const Eigen::Isometry3d error = gt_motion.inverse() * est_motion;

      trans_errors.push_back( error.translation().norm() );
      rot_errors.push_back( rotationAngleDeg( error ) );
    }

    if ( trans_errors.empty() )
    {
      std::ostringstream cause;
      cause << "no matched pose pair is separated by " << options.delta_ns
            << " ns within a tolerance of " << options.delta_tolerance_ns
            << " ns; the matched estimate spans "
            << ( timestamps.back() - timestamps.front() ) << " ns";
      return EvalError{ EvalErrorCode::kNoDeltaPairs, {}, std::nullopt, {}, std::move( cause ).str() };
    }

    report.pair_count = trans_errors.size();
    report.trans_m    = computeStats( std::move( trans_errors ) );
    report.rot_deg    = computeStats( std::move( rot_errors ) );
    return report;
  }

}  // namespace phad::eval
