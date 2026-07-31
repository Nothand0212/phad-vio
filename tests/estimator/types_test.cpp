#include "phad/estimator/types.hpp"

#include <gtest/gtest.h>

#include <type_traits>

#include "phad/common/landmark_id.hpp"

TEST( EstimatorTypes, LandmarkIdAliasesCommon )
{
  static_assert( std::is_same_v<phad::common::LandmarkId, std::uint64_t> );
  static_assert(
      std::is_same_v<phad::estimator::LandmarkId, phad::common::LandmarkId> );
}

TEST( EstimatorTypes, KeyframeMeasurementConstructs )
{
  phad::estimator::KeyframeMeasurement measurement;
  measurement.timestamp = phad::common::Timestamp{ 1'000'000'000 };
  measurement.observations.push_back( phad::estimator::StereoObservation{
      .id           = 7,
      .left_pixel   = Eigen::Vector2d( 320.5, 240.0 ),
      .disparity_px = 12.0,
  } );

  ASSERT_EQ( measurement.observations.size(), 1U );
  EXPECT_EQ( measurement.observations.front().id, 7U );
  EXPECT_DOUBLE_EQ( measurement.observations.front().disparity_px, 12.0 );

  phad::estimator::VioUpdateResult result;
  result.status   = phad::estimator::UpdateStatus::kOk;
  result.estimate = phad::estimator::VioEstimate{
      .timestamp = measurement.timestamp,
      .T_W_B     = Eigen::Isometry3d::Identity(),
  };
  EXPECT_TRUE( result.estimate.has_value() );
  EXPECT_TRUE( result.message.empty() );
}
