#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/estimator/types.hpp"
#include "phad/estimator/vio_estimator.hpp"
#include "phad/sensor/rigid_transform.hpp"
#include "tests/estimator/vio_test_utils.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::common::LandmarkId;
  using phad::common::Timestamp;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::StereoObservation;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioEstimator;
  using phad::estimator::VioMeasurement;
  using phad::sensor::RigidTransform;

  [[nodiscard]] RectifiedStereoCalibration makeCalibration()
  {
    auto transform =
        RigidTransform::create( Eigen::Isometry3d::Identity().matrix() )
            .value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480,
               std::move( transform ) )
        .value();
  }

  [[nodiscard]] StereoObservation makeObservation( LandmarkId id,
                                                   double     disparity_px,
                                                   double     x_offset_px )
  {
    return StereoObservation{
        .id           = id,
        .left_pixel   = Eigen::Vector2d{ 280.0 + x_offset_px, 220.0 },
        .disparity_px = disparity_px };
  }

  [[nodiscard]] VioMeasurement makeMeasurement(
      std::int64_t                   timestamp_ns,
      std::vector<StereoObservation> observations )
  {
    const Timestamp timestamp{ timestamp_ns };
    return VioMeasurement{
        .m_timestamp    = timestamp,
        .m_observations = std::move( observations ),
        .m_imu          = phad::test_support::stationaryImuPayload( timestamp,
                                                                    100'000'000 ) };
  }

  [[nodiscard]] std::vector<StereoObservation> makeRootObservations()
  {
    std::vector<StereoObservation> observations;
    observations.reserve( 10U );
    for ( std::uint64_t index = 0; index < 10U; ++index )
    {
      observations.push_back( makeObservation(
          LandmarkId{ index + 1U }, 12.0,
          7.0 * static_cast<double>( index ) ) );
    }
    return observations;
  }

}  // namespace

TEST( MappedLandmarkBearingClassification,
      CountsMappedObservationsIndependentOfDisparity )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 2;
  options.min_landmark_observations       = 2;
  options.enable_pnp_init                 = false;
  options.enable_outlier_cull             = false;
  options.enable_outlier_reopt            = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto root = estimator.update(
      makeMeasurement( 100'000'000, makeRootObservations() ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  std::vector<StereoObservation> observations;
  observations.push_back(
      makeObservation( LandmarkId{ 1U }, 12.0, 0.0 ) );
  observations.push_back(
      makeObservation( LandmarkId{ 2U }, 0.0, 7.0 ) );
  observations.push_back(
      makeObservation( LandmarkId{ 101U }, 12.0, 14.0 ) );
  observations.push_back(
      makeObservation( LandmarkId{ 102U }, 0.0, 21.0 ) );

  const auto result = estimator.update(
      makeMeasurement( 200'000'000, std::move( observations ) ) );

  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  ASSERT_TRUE( result.estimate.has_value() );
  EXPECT_EQ( result.estimate->m_segment_id, 0U );
  EXPECT_EQ( result.diagnostics.num_observations, 4U );
  EXPECT_EQ( result.diagnostics.num_disparity, 2U );
  EXPECT_EQ( result.diagnostics.num_shared, 1U );
  EXPECT_EQ( result.diagnostics.num_mapped_observations, 2U );
  EXPECT_EQ( result.diagnostics.num_retained_observations, 4U );
  EXPECT_EQ( result.diagnostics.num_seeded_landmarks, 0U );
  EXPECT_EQ( result.diagnostics.num_current_visual_factors, 1U );
  EXPECT_FALSE( result.diagnostics.pnp_success );
  EXPECT_EQ( result.diagnostics.m_vio.m_visual_coast_duration_ns,
             100'000'000 );
}
