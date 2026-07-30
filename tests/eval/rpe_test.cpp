#include "phad/eval/rpe.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
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
  using phad::eval::computeRpe;
  using phad::eval::EvalErrorCode;
  using phad::eval::RpeOptions;
  using phad::testing::kEurocEpochNs;
  using phad::testing::kStepNs;
  using phad::testing::makeHelix;
  using phad::testing::makePose;
  using phad::testing::makeTransform;
  using phad::testing::transformed;

  /// 每个位姿左乘一个随时间线性增长的平移，模拟恒定平移漂移。
  Trajectory withTranslationDrift( const Trajectory&      source,
                                   const Eigen::Vector3d& velocity_mps )
  {
    std::vector<TimedPose> poses    = source.poses();
    const std::int64_t     start_ns = source.firstTimestamp().nanoseconds();
    for ( TimedPose& pose : poses )
    {
      const double elapsed_s =
          static_cast<double>( pose.timestamp.nanoseconds() - start_ns ) * 1e-9;
      pose.T_W_B.translation() += velocity_mps * elapsed_s;
    }
    return Trajectory::create( std::move( poses ) ).value();
  }

  /// 直线路径，姿态为 identity；yaw_rate 非零时姿态随时间匀速偏转。
  Trajectory makeStraightLine( std::size_t count, double yaw_rate_deg_s )
  {
    std::vector<TimedPose> poses;
    poses.reserve( count );
    for ( std::size_t index = 0; index < count; ++index )
    {
      const std::int64_t       offset_ns = static_cast<std::int64_t>( index ) * kStepNs;
      const double             elapsed_s = static_cast<double>( offset_ns ) * 1e-9;
      const Eigen::Quaterniond rotation{ Eigen::AngleAxisd{
          yaw_rate_deg_s * elapsed_s * std::numbers::pi / 180.0,
          Eigen::Vector3d::UnitZ() } };
      poses.push_back( makePose( kEurocEpochNs + offset_ns,
                                 Eigen::Vector3d{ 0.1 * elapsed_s, 0.0, 0.0 },
                                 rotation ) );
    }
    return Trajectory::create( std::move( poses ) ).value();
  }

  TEST( RpeTest, SelfComparisonIsZero )
  {
    // 50 ms 采样、40 个位姿覆盖 1.95 s：只有前 20 个位姿存在相隔 1 s 的伙伴。
    const Trajectory trajectory = makeHelix( 40 );
    const auto       report     = computeRpe( trajectory, trajectory );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_EQ( report.value().delta_ns, 1'000'000'000 );
    EXPECT_EQ( report.value().pair_count, 20U );
    EXPECT_EQ( report.value().dropped_no_partner, 20U );
    EXPECT_NEAR( report.value().trans_m.rmse, 0.0, 1e-12 );
    EXPECT_NEAR( report.value().rot_deg.rmse, 0.0, 1e-9 );
  }

  TEST( RpeTest, IsInvariantUnderRigidTransformOfTheEstimate )
  {
    // RPE 只看相对运动，因此无需对齐：整体刚体偏移不改变任何数字。
    const Trajectory gt  = makeHelix( 60 );
    const Trajectory est = withTranslationDrift(
        gt, Eigen::Vector3d{ 0.1, -0.05, 0.02 } );

    const auto direct = computeRpe( est, gt );
    ASSERT_TRUE( direct ) << direct.error().describe();
    const auto moved = computeRpe(
        transformed( est,
                     makeTransform( 1.3, Eigen::Vector3d{ 4.0, -7.0, 11.0 } ) ),
        gt );
    ASSERT_TRUE( moved ) << moved.error().describe();
    EXPECT_NEAR( direct.value().trans_m.rmse, moved.value().trans_m.rmse,
                 1e-12 );
    EXPECT_NEAR( direct.value().rot_deg.rmse, moved.value().rot_deg.rmse, 1e-9 );
  }

  TEST( RpeTest, ConstantTranslationDriftEqualsDriftPerDelta )
  {
    const Trajectory gt = makeHelix( 60 );
    const Trajectory est =
        withTranslationDrift( gt, Eigen::Vector3d{ 0.1, 0.0, 0.0 } );

    const auto report = computeRpe( est, gt );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_NEAR( report.value().trans_m.rmse, 0.1, 1e-12 );
    EXPECT_NEAR( report.value().trans_m.max, 0.1, 1e-12 );
    EXPECT_NEAR( report.value().rot_deg.rmse, 0.0, 1e-9 );
  }

  TEST( RpeTest, ConstantYawDriftEqualsDriftPerDelta )
  {
    const Trajectory gt  = makeStraightLine( 60, 0.0 );
    const Trajectory est = makeStraightLine( 60, 5.0 );

    const auto report = computeRpe( est, gt );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_NEAR( report.value().rot_deg.rmse, 5.0, 1e-9 );
    EXPECT_NEAR( report.value().rot_deg.max, 5.0, 1e-9 );
  }

  TEST( RpeTest, PairsTimestampsOffTheDeltaGrid )
  {
    // 30 ms 采样时没有位姿恰好相隔 1 s：最近的伙伴偏离 10 ms，落在默认
    // 25 ms 容差内，误差按实际的 0.99 s 间隔计算。
    const Trajectory gt = makeHelix( 100, kEurocEpochNs, 30'000'000 );
    const Trajectory est =
        withTranslationDrift( gt, Eigen::Vector3d{ 0.1, 0.0, 0.0 } );

    const auto report = computeRpe( est, gt );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_EQ( report.value().pair_count, 67U );
    EXPECT_EQ( report.value().dropped_no_partner, 33U );
    EXPECT_NEAR( report.value().trans_m.rmse, 0.1 * 0.99, 1e-12 );
  }

  TEST( RpeTest, FailsWhenNoPairSpansTheDelta )
  {
    const Trajectory trajectory = makeHelix( 5 );
    const auto       report     = computeRpe( trajectory, trajectory );
    ASSERT_FALSE( report );
    EXPECT_EQ( report.error().code, EvalErrorCode::kNoDeltaPairs );
  }

  TEST( RpeTest, RejectsNonPositiveDelta )
  {
    const Trajectory trajectory = makeHelix( 40 );
    RpeOptions       options;
    options.delta_ns  = 0;
    const auto report = computeRpe( trajectory, trajectory, options );
    ASSERT_FALSE( report );
    EXPECT_EQ( report.error().code, EvalErrorCode::kInvalidOptions );
  }

  TEST( RpeTest, RejectsToleranceNotSmallerThanDelta )
  {
    // 容差不小于间隔时，最近的伙伴可能只相隔一帧，得到的数字与所声明的
    // 间隔无关。
    const Trajectory trajectory = makeHelix( 40 );
    RpeOptions       options;
    options.delta_tolerance_ns = options.delta_ns;
    const auto report          = computeRpe( trajectory, trajectory, options );
    ASSERT_FALSE( report );
    EXPECT_EQ( report.error().code, EvalErrorCode::kInvalidOptions );
  }

  TEST( RpeTest, PropagatesAssociationFailure )
  {
    const Trajectory est = makeHelix( 40 );
    const Trajectory gt =
        makeHelix( 40, kEurocEpochNs + 1'000 * kStepNs );

    const auto report = computeRpe( est, gt );
    ASSERT_FALSE( report );
    EXPECT_EQ( report.error().code, EvalErrorCode::kNoOverlap );
  }

}  // namespace
