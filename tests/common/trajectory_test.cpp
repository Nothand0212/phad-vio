#include "phad/common/trajectory.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <limits>
#include <utility>
#include <vector>

#include "tests/eval/synthetic_trajectory.hpp"

namespace
{

  using phad::common::TimedPose;
  using phad::common::Trajectory;
  using phad::common::TrajectoryErrorCode;
  using phad::testing::kEurocEpochNs;
  using phad::testing::kStepNs;
  using phad::testing::makePose;

  std::vector<TimedPose> twoPoses()
  {
    return { makePose( kEurocEpochNs, Eigen::Vector3d{ 0.0, 0.0, 0.0 } ),
             makePose( kEurocEpochNs + kStepNs,
                       Eigen::Vector3d{ 1.0, 0.0, 0.0 } ) };
  }

  TEST( TrajectoryTest, RejectsEmptyPoseSequence )
  {
    const auto result = Trajectory::create( {} );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, TrajectoryErrorCode::kEmpty );
  }

  TEST( TrajectoryTest, AcceptsStrictlyIncreasingTimestamps )
  {
    auto result = Trajectory::create( twoPoses() );
    ASSERT_TRUE( result );
    const Trajectory trajectory = std::move( result ).value();
    EXPECT_EQ( trajectory.size(), 2U );
    EXPECT_EQ( trajectory.firstTimestamp().nanoseconds(), kEurocEpochNs );
    EXPECT_EQ( trajectory.lastTimestamp().nanoseconds(),
               kEurocEpochNs + kStepNs );
  }

  TEST( TrajectoryTest, RejectsDuplicateTimestamp )
  {
    std::vector<TimedPose> poses = twoPoses();
    poses[ 1 ].timestamp         = poses[ 0 ].timestamp;
    const auto result            = Trajectory::create( std::move( poses ) );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, TrajectoryErrorCode::kDuplicateTimestamp );
    EXPECT_EQ( result.error().index, 1U );
  }

  TEST( TrajectoryTest, RejectsOutOfOrderTimestamp )
  {
    std::vector<TimedPose> poses = twoPoses();
    std::swap( poses[ 0 ].timestamp, poses[ 1 ].timestamp );
    const auto result = Trajectory::create( std::move( poses ) );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code,
               TrajectoryErrorCode::kOutOfOrderTimestamp );
    EXPECT_EQ( result.error().index, 1U );
  }

  TEST( TrajectoryTest, RejectsNonFinitePose )
  {
    std::vector<TimedPose> poses = twoPoses();
    poses[ 1 ].T_W_B.translation().x() =
        std::numeric_limits<double>::quiet_NaN();
    const auto result = Trajectory::create( std::move( poses ) );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, TrajectoryErrorCode::kNonFinitePose );
    EXPECT_EQ( result.error().index, 1U );
  }

  TEST( TrajectoryTest, RejectsNonOrthogonalRotation )
  {
    std::vector<TimedPose> poses = twoPoses();
    poses[ 0 ].T_W_B.linear() *= 1.5;
    const auto result = Trajectory::create( std::move( poses ) );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, TrajectoryErrorCode::kInvalidRotation );
    EXPECT_EQ( result.error().index, 0U );
  }

  TEST( TrajectoryTest, RejectsMirroredRotation )
  {
    std::vector<TimedPose> poses      = twoPoses();
    poses[ 1 ].T_W_B.linear()( 0, 0 ) = -1.0;
    const auto result                 = Trajectory::create( std::move( poses ) );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, TrajectoryErrorCode::kInvalidRotation );
  }

}  // namespace
