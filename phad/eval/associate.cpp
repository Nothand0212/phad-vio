#include "phad/eval/associate.hpp"

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <sstream>
#include <utility>

/**
 * @file associate.cpp
 * @brief 时间关联实现。
 */

namespace phad::eval
{
  namespace
  {

    /// 返回真值中与 timestamp 最近的样本下标。gt 非空且严格递增。
    std::size_t nearestIndex( const common::Trajectory& gt,
                              common::Timestamp         timestamp )
    {
      const std::vector<common::TimedPose>& poses = gt.poses();
      const auto                            upper = std::lower_bound(
          poses.begin(), poses.end(), timestamp,
          []( const common::TimedPose& pose, common::Timestamp value ) { return pose.timestamp < value; } );
      if ( upper == poses.begin() )
      {
        return 0;
      }
      if ( upper == poses.end() )
      {
        return poses.size() - 1U;
      }
      const auto         lower = std::prev( upper );
      const std::int64_t lower_gap =
          timestamp.nanoseconds() - lower->timestamp.nanoseconds();
      const std::int64_t upper_gap =
          upper->timestamp.nanoseconds() - timestamp.nanoseconds();
      return static_cast<std::size_t>(
          std::distance( poses.begin(), upper_gap < lower_gap ? upper
                                                              : lower ) );
    }

  }  // namespace

  double Association::matchRate() const noexcept
  {
    if ( est_total == 0 )
    {
      return 0.0;
    }
    return static_cast<double>( pairs.size() ) /
           static_cast<double>( est_total );
  }

  std::size_t Association::droppedTotal() const noexcept
  {
    return dropped_out_of_range + dropped_over_threshold;
  }

  EvalResult<Association> associate( const common::Trajectory& est,
                                     const common::Trajectory& gt,
                                     const AssociationOptions& options )
  {
    Association association;
    association.est_total = est.size();

    const std::int64_t gt_first = gt.firstTimestamp().nanoseconds();
    const std::int64_t gt_last  = gt.lastTimestamp().nanoseconds();
    for ( std::size_t index = 0; index < est.size(); ++index )
    {
      const common::Timestamp timestamp = est.poses()[ index ].timestamp;
      const std::size_t       gt_index  = nearestIndex( gt, timestamp );
      const std::int64_t      dt_ns =
          timestamp.nanoseconds() -
          gt.poses()[ gt_index ].timestamp.nanoseconds();
      if ( std::abs( dt_ns ) <= options.max_dt_ns )
      {
        association.pairs.push_back( MatchedPair{ index, gt_index, dt_ns } );
        continue;
      }
      if ( timestamp.nanoseconds() < gt_first ||
           timestamp.nanoseconds() > gt_last )
      {
        ++association.dropped_out_of_range;
      }
      else
      {
        ++association.dropped_over_threshold;
      }
    }

    if ( association.pairs.empty() )
    {
      std::ostringstream cause;
      cause << "no estimate pose falls within " << options.max_dt_ns
            << " ns of a groundtruth pose; estimate spans ["
            << est.firstTimestamp().nanoseconds() << ", "
            << est.lastTimestamp().nanoseconds() << "] ns, groundtruth spans ["
            << gt_first << ", " << gt_last << "] ns";
      return EvalError{ EvalErrorCode::kNoOverlap, {}, std::nullopt, {}, std::move( cause ).str() };
    }
    if ( association.pairs.size() < options.min_matches )
    {
      std::ostringstream cause;
      cause << "matched " << association.pairs.size() << " poses, at least "
            << options.min_matches << " are required";
      return EvalError{ EvalErrorCode::kTooFewMatches, {}, std::nullopt, {}, std::move( cause ).str() };
    }
    if ( association.matchRate() < options.min_match_rate )
    {
      std::ostringstream cause;
      cause << "matched " << association.pairs.size() << " of "
            << association.est_total << " estimate poses (rate "
            << association.matchRate() << "), minimum is "
            << options.min_match_rate << "; dropped "
            << association.dropped_out_of_range
            << " outside the groundtruth time span and "
            << association.dropped_over_threshold
            << " beyond the association threshold";
      return EvalError{ EvalErrorCode::kMatchRateTooLow, {}, std::nullopt, {}, std::move( cause ).str() };
    }
    return association;
  }

}  // namespace phad::eval
