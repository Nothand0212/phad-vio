#include "phad/eval/ate.hpp"

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
