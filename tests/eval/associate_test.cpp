#include "phad/eval/associate.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cstdint>
#include <utility>
#include <vector>

#include "tests/common/synthetic_trajectory.hpp"

namespace
{

  using phad::common::TimedPose;
  using phad::common::Trajectory;
  using phad::eval::associate;
  using phad::eval::AssociationOptions;
  using phad::eval::EvalErrorCode;
  using phad::testing::kEurocEpochNs;
  using phad::testing::kStepNs;
  using phad::testing::makeHelix;
  using phad::testing::makePose;

  /// 以 gt 为基准平移每个时间戳，模拟采样抖动。
  Trajectory shiftedTimestamps( const Trajectory& source, std::int64_t shift_ns )
  {
    std::vector<TimedPose> poses = source.poses();
    for ( TimedPose& pose : poses )
    {
      pose.timestamp =
          phad::common::Timestamp{ pose.timestamp.nanoseconds() + shift_ns };
    }
    return Trajectory::create( std::move( poses ) ).value();
  }

  TEST( AssociateTest, MatchesIdenticalTimestamps )
  {
    const Trajectory trajectory = makeHelix( 10 );
    const auto       result     = associate( trajectory, trajectory );
    ASSERT_TRUE( result ) << result.error().describe();
    EXPECT_EQ( result.value().pairs.size(), 10U );
    EXPECT_EQ( result.value().droppedTotal(), 0U );
    EXPECT_DOUBLE_EQ( result.value().matchRate(), 1.0 );
    for ( const auto& pair : result.value().pairs )
    {
      EXPECT_EQ( pair.est_index, pair.gt_index );
      EXPECT_EQ( pair.dt_ns, 0 );
    }
  }

  TEST( AssociateTest, MatchesJitterWithinThreshold )
  {
    const Trajectory gt     = makeHelix( 10 );
    const Trajectory est    = shiftedTimestamps( gt, 2'000'000 );
    const auto       result = associate( est, gt );
    ASSERT_TRUE( result ) << result.error().describe();
    EXPECT_EQ( result.value().pairs.size(), 10U );
    for ( const auto& pair : result.value().pairs )
    {
      EXPECT_EQ( pair.dt_ns, 2'000'000 );
    }
  }

  TEST( AssociateTest, CountsSamplesBeyondThresholdInsideGroundtruthSpan )
  {
    // 真值在中间缺一段：第 5 个样本被移除，估计仍然覆盖该时刻。
    std::vector<TimedPose> gt_poses = makeHelix( 10 ).poses();
    gt_poses.erase( gt_poses.begin() + 5 );
    const Trajectory gt  = Trajectory::create( std::move( gt_poses ) ).value();
    const Trajectory est = makeHelix( 10 );

    const auto result = associate( est, gt );
    ASSERT_TRUE( result ) << result.error().describe();
    EXPECT_EQ( result.value().pairs.size(), 9U );
    EXPECT_EQ( result.value().dropped_over_threshold, 1U );
    EXPECT_EQ( result.value().dropped_out_of_range, 0U );
  }

  TEST( AssociateTest, CountsSamplesOutsideGroundtruthSpan )
  {
    const Trajectory est = makeHelix( 10 );
    // 真值只覆盖中间 6 个时刻。
    std::vector<TimedPose> gt_poses( est.poses().begin() + 2,
                                     est.poses().begin() + 8 );
    const Trajectory       gt = Trajectory::create( std::move( gt_poses ) ).value();

    const auto result = associate( est, gt );
    ASSERT_TRUE( result ) << result.error().describe();
    EXPECT_EQ( result.value().pairs.size(), 6U );
    EXPECT_EQ( result.value().dropped_out_of_range, 4U );
    EXPECT_EQ( result.value().dropped_over_threshold, 0U );
    EXPECT_NEAR( result.value().matchRate(), 0.6, 1e-12 );
  }

  TEST( AssociateTest, ReportsNoOverlap )
  {
    const Trajectory gt     = makeHelix( 5 );
    const Trajectory est    = shiftedTimestamps( gt, 10 * kStepNs );
    const auto       result = associate( est, gt );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, EvalErrorCode::kNoOverlap );
  }

  TEST( AssociateTest, ReportsTooFewMatches )
  {
    const Trajectory       est = makeHelix( 10 );
    std::vector<TimedPose> gt_poses( est.poses().begin(),
                                     est.poses().begin() + 2 );
    const Trajectory       gt = Trajectory::create( std::move( gt_poses ) ).value();

    AssociationOptions options;
    options.min_match_rate = 0.0;
    const auto result      = associate( est, gt, options );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, EvalErrorCode::kTooFewMatches );
  }

  TEST( AssociateTest, ReportsMatchRateTooLow )
  {
    const Trajectory       est = makeHelix( 10 );
    std::vector<TimedPose> gt_poses( est.poses().begin(),
                                     est.poses().begin() + 4 );
    const Trajectory       gt = Trajectory::create( std::move( gt_poses ) ).value();

    const auto result = associate( est, gt );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, EvalErrorCode::kMatchRateTooLow );
  }

  TEST( AssociateTest, HonoursCustomThreshold )
  {
    const Trajectory   gt  = makeHelix( 10 );
    const Trajectory   est = shiftedTimestamps( gt, 4'000'000 );
    AssociationOptions options;
    options.max_dt_ns = 5'000'000;

    const auto relaxed = associate( est, gt, options );
    ASSERT_TRUE( relaxed ) << relaxed.error().describe();
    EXPECT_EQ( relaxed.value().pairs.size(), 10U );

    const auto strict = associate( est, gt );
    ASSERT_FALSE( strict );
    EXPECT_EQ( strict.error().code, EvalErrorCode::kNoOverlap );
  }

  TEST( AssociateTest, KeepsNearestWhenGroundtruthIsDenser )
  {
    const Trajectory gt = makeHelix( 20, kEurocEpochNs, kStepNs / 10 );
    const Trajectory est =
        makeHelix( 3, kEurocEpochNs + kStepNs / 10, kStepNs / 10 * 3 );

    const auto result = associate( est, gt );
    ASSERT_TRUE( result ) << result.error().describe();
    ASSERT_EQ( result.value().pairs.size(), 3U );
    EXPECT_EQ( result.value().pairs[ 0 ].gt_index, 1U );
    EXPECT_EQ( result.value().pairs[ 1 ].gt_index, 4U );
    EXPECT_EQ( result.value().pairs[ 2 ].gt_index, 7U );
  }

}  // namespace
