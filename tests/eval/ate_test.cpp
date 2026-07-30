#include "phad/eval/ate.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <utility>
#include <vector>

#include "tests/common/synthetic_trajectory.hpp"

namespace
{

  using phad::common::TimedPose;
  using phad::common::Trajectory;
  using phad::eval::AteOptions;
  using phad::eval::computeAte;
  using phad::eval::EvalErrorCode;
  using phad::testing::makeHelix;
  using phad::testing::makeTransform;
  using phad::testing::transformed;

  TEST( AteTest, SelfComparisonIsZero )
  {
    const Trajectory trajectory = makeHelix( 40 );
    const auto       report     = computeAte( trajectory, trajectory );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_NEAR( report.value().trans_m.rmse, 0.0, 1e-12 );
    EXPECT_NEAR( report.value().trans_m.max, 0.0, 1e-12 );
    EXPECT_NEAR( report.value().rot_deg.rmse, 0.0, 1e-9 );
    EXPECT_EQ( report.value().samples.size(), trajectory.size() );
  }

  TEST( AteTest, RecoversKnownRigidPerturbation )
  {
    const Trajectory        gt     = makeHelix( 40 );
    const Eigen::Isometry3d offset = makeTransform(
        1.1, Eigen::Vector3d{ -5.0, 2.0, 0.75 } );
    const Trajectory est = transformed( gt, offset );

    const auto report = computeAte( est, gt );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_NEAR( report.value().trans_m.rmse, 0.0, 1e-12 );
    EXPECT_NEAR( report.value().rot_deg.rmse, 0.0, 1e-9 );
    EXPECT_TRUE( report.value().T_align.isApprox( offset.inverse(), 1e-9 ) );
  }

  TEST( AteTest, IsInvariantUnderRigidTransformOfTheEstimate )
  {
    const Trajectory       gt          = makeHelix( 40 );
    std::vector<TimedPose> noisy_poses = gt.poses();
    for ( std::size_t index = 0; index < noisy_poses.size(); ++index )
    {
      noisy_poses[ index ].T_W_B.translation().z() +=
          0.01 * static_cast<double>( index % 3U );
    }
    const Trajectory est = Trajectory::create( std::move( noisy_poses ) ).value();

    const auto direct = computeAte( est, gt );
    ASSERT_TRUE( direct ) << direct.error().describe();
    const auto moved = computeAte(
        transformed( est, makeTransform( -0.6, Eigen::Vector3d{ 7.0, 8.0,
                                                                -9.0 } ) ),
        gt );
    ASSERT_TRUE( moved ) << moved.error().describe();
    EXPECT_NEAR( direct.value().trans_m.rmse, moved.value().trans_m.rmse,
                 1e-12 );
  }

  TEST( AteTest, ScaleErrorIsNotAbsorbedByAlignment )
  {
    // 双目尺度已知，因此对齐不估 scale：放大 1.1 倍的估计必须留下误差。
    // 单位正方形放大后，对齐只能吸收质心平移，四个点各残留
    // 0.05 * sqrt(2) 的误差。
    std::vector<TimedPose>             gt_poses;
    std::vector<TimedPose>             est_poses;
    const std::vector<Eigen::Vector3d> square{ { 0.0, 0.0, 0.0 },
                                               { 1.0, 0.0, 0.0 },
                                               { 0.0, 1.0, 0.0 },
                                               { 1.0, 1.0, 0.0 } };
    for ( std::size_t index = 0; index < square.size(); ++index )
    {
      const std::int64_t timestamp =
          phad::testing::kEurocEpochNs +
          static_cast<std::int64_t>( index ) * phad::testing::kStepNs;
      gt_poses.push_back( phad::testing::makePose( timestamp, square[ index ] ) );
      est_poses.push_back(
          phad::testing::makePose( timestamp, 1.1 * square[ index ] ) );
    }
    const Trajectory gt  = Trajectory::create( std::move( gt_poses ) ).value();
    const Trajectory est = Trajectory::create( std::move( est_poses ) ).value();

    const auto report = computeAte( est, gt );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_NEAR( report.value().trans_m.rmse, 0.05 * std::sqrt( 2.0 ), 1e-12 );
  }

  TEST( AteTest, ReportsRotationErrorInDegrees )
  {
    const Trajectory       gt        = makeHelix( 40 );
    std::vector<TimedPose> est_poses = gt.poses();
    const Eigen::Matrix3d  tilt =
        Eigen::AngleAxisd{ 10.0 * std::numbers::pi / 180.0,
                           Eigen::Vector3d::UnitY() }
            .toRotationMatrix();
    for ( TimedPose& pose : est_poses )
    {
      pose.T_W_B.linear() = pose.T_W_B.linear() * tilt;
    }
    const Trajectory est = Trajectory::create( std::move( est_poses ) ).value();

    const auto report = computeAte( est, gt );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_NEAR( report.value().trans_m.rmse, 0.0, 1e-12 );
    EXPECT_NEAR( report.value().rot_deg.rmse, 10.0, 1e-9 );
    EXPECT_NEAR( report.value().rot_deg.median, 10.0, 1e-9 );
  }

  TEST( AteTest, ReportsDroppedSamples )
  {
    const Trajectory       est = makeHelix( 10 );
    std::vector<TimedPose> gt_poses( est.poses().begin() + 1,
                                     est.poses().end() );
    const Trajectory       gt = Trajectory::create( std::move( gt_poses ) ).value();

    const auto report = computeAte( est, gt );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_EQ( report.value().association.dropped_out_of_range, 1U );
    EXPECT_EQ( report.value().samples.size(), 9U );
  }

  TEST( AteTest, PropagatesAssociationFailure )
  {
    const Trajectory est = makeHelix( 10 );
    const Trajectory gt =
        makeHelix( 10, phad::testing::kEurocEpochNs + 100 * phad::testing::kStepNs );

    const auto report = computeAte( est, gt );
    ASSERT_FALSE( report );
    EXPECT_EQ( report.error().code, EvalErrorCode::kNoOverlap );
  }

  TEST( AteTest, PropagatesDegenerateAlignment )
  {
    std::vector<TimedPose> poses;
    for ( std::size_t index = 0; index < 5U; ++index )
    {
      poses.push_back( phad::testing::makePose(
          phad::testing::kEurocEpochNs +
              static_cast<std::int64_t>( index ) * phad::testing::kStepNs,
          Eigen::Vector3d{ static_cast<double>( index ), 0.0, 0.0 } ) );
    }
    const Trajectory collinear =
        Trajectory::create( std::move( poses ) ).value();

    const auto report = computeAte( collinear, collinear );
    ASSERT_FALSE( report );
    EXPECT_EQ( report.error().code, EvalErrorCode::kDegenerateAlignment );
  }

}  // namespace
