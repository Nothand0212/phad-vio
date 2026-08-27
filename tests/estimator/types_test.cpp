#include "phad/estimator/types.hpp"

#include <gtest/gtest.h>

#include <type_traits>

#include "phad/common/landmark_id.hpp"
#include "tests/estimator/vio_test_utils.hpp"

TEST( EstimatorTypes, LandmarkIdAliasesCommon )
{
  static_assert( std::is_same_v<phad::common::LandmarkId, std::uint64_t> );
  static_assert(
      std::is_same_v<phad::estimator::LandmarkId, phad::common::LandmarkId> );
}

TEST( EstimatorTypes, OutlierAvgReprojDefault )
{
  phad::estimator::EstimatorOptions options;
  EXPECT_DOUBLE_EQ( options.outlier_avg_reproj_px, 4.0 );
  EXPECT_TRUE( options.enable_outlier_cull );
  EXPECT_TRUE( options.block_culled_rebirth );
}

TEST( EstimatorTypes, VioMeasurementConstructs )
{
  phad::estimator::VioMeasurement measurement{
      .m_timestamp    = phad::common::Timestamp{ 1'000'000'000 },
      .m_observations = {},
      .m_imu          = phad::test_support::stationaryImuPayload(
          phad::common::Timestamp{ 1'000'000'000 } ) };
  measurement.m_observations.push_back( phad::estimator::StereoObservation{
      .id           = 7,
      .left_pixel   = Eigen::Vector2d( 320.5, 240.0 ),
      .disparity_px = 12.0,
  } );

  ASSERT_EQ( measurement.m_observations.size(), 1U );
  EXPECT_EQ( measurement.m_observations.front().id, 7U );
  EXPECT_DOUBLE_EQ( measurement.m_observations.front().disparity_px, 12.0 );

  phad::estimator::VioUpdateResult result;
  result.status   = phad::estimator::UpdateStatus::kOk;
  result.estimate = phad::estimator::VioEstimate{
      .timestamp    = measurement.m_timestamp,
      .T_W_B        = Eigen::Isometry3d::Identity(),
      .m_v_W_B      = Eigen::Vector3d::Zero(),
      .m_bias       = {},
      .m_segment_id = 0,
  };
  EXPECT_TRUE( result.estimate.has_value() );
  EXPECT_TRUE( result.message.empty() );
}
