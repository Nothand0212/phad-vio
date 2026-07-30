#include "phad/eval/align.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cstddef>
#include <numbers>
#include <vector>

#include "tests/common/synthetic_trajectory.hpp"

namespace
{

  using phad::eval::alignSe3;
  using phad::eval::EvalErrorCode;
  using phad::testing::makeHelix;
  using phad::testing::makeTransform;

  std::vector<Eigen::Vector3d> helixPositions( std::size_t count )
  {
    const phad::common::Trajectory trajectory = makeHelix( count );
    std::vector<Eigen::Vector3d>   positions;
    positions.reserve( count );
    for ( const auto& pose : trajectory.poses() )
    {
      positions.push_back( pose.T_W_B.translation() );
    }
    return positions;
  }

  std::vector<Eigen::Vector3d> transformedPositions(
      const std::vector<Eigen::Vector3d>& source,
      const Eigen::Isometry3d&            transform )
  {
    std::vector<Eigen::Vector3d> target;
    target.reserve( source.size() );
    for ( const Eigen::Vector3d& point : source )
    {
      target.push_back( transform * point );
    }
    return target;
  }

  TEST( AlignTest, RecoversKnownTransform )
  {
    const std::vector<Eigen::Vector3d> source   = helixPositions( 12 );
    const Eigen::Isometry3d            expected = makeTransform(
        0.7, Eigen::Vector3d{ 3.0, -2.0, 1.5 } );
    const std::vector<Eigen::Vector3d> target =
        transformedPositions( source, expected );

    const auto result = alignSe3( source, target );
    ASSERT_TRUE( result ) << result.error().describe();
    EXPECT_TRUE( result.value().isApprox( expected, 1e-9 ) );
  }

  TEST( AlignTest, RecoversIdentityForCoincidentPointSets )
  {
    const std::vector<Eigen::Vector3d> source = helixPositions( 12 );
    const auto                         result = alignSe3( source, source );
    ASSERT_TRUE( result ) << result.error().describe();
    EXPECT_TRUE(
        result.value().isApprox( Eigen::Isometry3d::Identity(), 1e-9 ) );
  }

  TEST( AlignTest, RecoversHalfTurnRotation )
  {
    const std::vector<Eigen::Vector3d> source = helixPositions( 12 );
    const Eigen::Isometry3d            expected =
        makeTransform( std::numbers::pi, Eigen::Vector3d::Zero() );
    const std::vector<Eigen::Vector3d> target =
        transformedPositions( source, expected );

    const auto result = alignSe3( source, target );
    ASSERT_TRUE( result ) << result.error().describe();
    EXPECT_TRUE( result.value().isApprox( expected, 1e-9 ) );
  }

  TEST( AlignTest, AlignsCoplanarPointSets )
  {
    const std::vector<Eigen::Vector3d> source{ { 0.0, 0.0, 0.0 },
                                               { 1.0, 0.0, 0.0 },
                                               { 0.0, 1.0, 0.0 },
                                               { 1.0, 1.0, 0.0 } };
    const Eigen::Isometry3d            expected =
        makeTransform( 0.3, Eigen::Vector3d{ 1.0, 2.0, 3.0 } );
    const std::vector<Eigen::Vector3d> target =
        transformedPositions( source, expected );

    const auto result = alignSe3( source, target );
    ASSERT_TRUE( result ) << result.error().describe();
    EXPECT_TRUE( result.value().isApprox( expected, 1e-9 ) );
  }

  TEST( AlignTest, RejectsCollinearPointSets )
  {
    const std::vector<Eigen::Vector3d> source{ { 0.0, 0.0, 0.0 },
                                               { 1.0, 0.0, 0.0 },
                                               { 2.0, 0.0, 0.0 },
                                               { 3.0, 0.0, 0.0 } };
    const auto                         result = alignSe3( source, source );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, EvalErrorCode::kDegenerateAlignment );
  }

  TEST( AlignTest, RejectsCoincidentPointSets )
  {
    const std::vector<Eigen::Vector3d> source( 5,
                                               Eigen::Vector3d{ 1.0, 1.0, 1.0 } );
    const auto                         result = alignSe3( source, source );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, EvalErrorCode::kDegenerateAlignment );
  }

  TEST( AlignTest, RejectsTooFewPoints )
  {
    const std::vector<Eigen::Vector3d> source{ { 0.0, 0.0, 0.0 },
                                               { 1.0, 0.0, 0.0 } };
    const auto                         result = alignSe3( source, source );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, EvalErrorCode::kTooFewMatches );
  }

  TEST( AlignTest, RejectsSizeMismatch )
  {
    const std::vector<Eigen::Vector3d> source = helixPositions( 5 );
    const std::vector<Eigen::Vector3d> target = helixPositions( 4 );
    const auto                         result = alignSe3( source, target );
    ASSERT_FALSE( result );
    EXPECT_EQ( result.error().code, EvalErrorCode::kTooFewMatches );
  }

}  // namespace
