#include "phad/eval/ate.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <utility>

/**
 * @file ate.cpp
 * @brief ATE 计算实现。
 */

namespace phad::eval
{
  namespace
  {

    ErrorStats computeStats( std::vector<double> errors )
    {
      ErrorStats stats;
      if ( errors.empty() )
      {
        return stats;
      }

      double sum         = 0.0;
      double squared_sum = 0.0;
      for ( const double error : errors )
      {
        sum += error;
        squared_sum += error * error;
      }
      const double count = static_cast<double>( errors.size() );
      stats.mean         = sum / count;
      stats.rmse         = std::sqrt( squared_sum / count );
      stats.stddev =
          std::sqrt( std::max( 0.0, squared_sum / count -
                                        stats.mean * stats.mean ) );
      stats.max = *std::max_element( errors.begin(), errors.end() );

      const std::size_t middle = errors.size() / 2U;
      std::nth_element( errors.begin(), errors.begin() + static_cast<std::ptrdiff_t>( middle ),
                        errors.end() );
      if ( errors.size() % 2U == 1U )
      {
        stats.median = errors[ middle ];
      }
      else
      {
        const double upper = errors[ middle ];
        const double lower = *std::max_element(
            errors.begin(), errors.begin() + static_cast<std::ptrdiff_t>( middle ) );
        stats.median = 0.5 * ( lower + upper );
      }
      return stats;
    }

    double rotationErrorDeg( const Eigen::Matrix3d& estimated,
                             const Eigen::Matrix3d& reference )
    {
      const Eigen::AngleAxisd error{ estimated.transpose() * reference };
      return error.angle() * 180.0 / std::numbers::pi;
    }

  }  // namespace

  EvalResult<AteReport> computeAte( const common::Trajectory& est,
                                    const common::Trajectory& gt,
                                    const AteOptions&         options )
  {
    auto association = associate( est, gt, options.association );
    if ( !association )
    {
      return association.error();
    }

    std::vector<Eigen::Vector3d> est_positions;
    std::vector<Eigen::Vector3d> gt_positions;
    est_positions.reserve( association.value().pairs.size() );
    gt_positions.reserve( association.value().pairs.size() );
    for ( const MatchedPair& pair : association.value().pairs )
    {
      est_positions.push_back(
          est.poses()[ pair.est_index ].T_W_B.translation() );
      gt_positions.push_back( gt.poses()[ pair.gt_index ].T_W_B.translation() );
    }

    auto alignment =
        alignSe3( est_positions, gt_positions, options.rank_tolerance );
    if ( !alignment )
    {
      return alignment.error();
    }

    AteReport report;
    report.T_align     = std::move( alignment ).value();
    report.association = std::move( association ).value();
    report.samples.reserve( report.association.pairs.size() );

    std::vector<double> trans_errors;
    std::vector<double> rot_errors;
    trans_errors.reserve( report.association.pairs.size() );
    rot_errors.reserve( report.association.pairs.size() );
    for ( const MatchedPair& pair : report.association.pairs )
    {
      const common::TimedPose& est_pose = est.poses()[ pair.est_index ];
      const common::TimedPose& gt_pose  = gt.poses()[ pair.gt_index ];
      const Eigen::Isometry3d  aligned  = report.T_align * est_pose.T_W_B;

      PoseErrorSample sample;
      sample.timestamp            = est_pose.timestamp;
      sample.dt_ns                = pair.dt_ns;
      sample.aligned_est_position = aligned.translation();
      sample.gt_position          = gt_pose.T_W_B.translation();
      sample.trans_m =
          ( sample.aligned_est_position - sample.gt_position ).norm();
      sample.rot_deg =
          rotationErrorDeg( aligned.linear(), gt_pose.T_W_B.linear() );

      trans_errors.push_back( sample.trans_m );
      rot_errors.push_back( sample.rot_deg );
      report.samples.push_back( sample );
    }

    report.trans_m = computeStats( std::move( trans_errors ) );
    report.rot_deg = computeStats( std::move( rot_errors ) );
    return report;
  }

}  // namespace phad::eval
