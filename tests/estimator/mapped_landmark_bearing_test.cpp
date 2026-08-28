#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
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

  [[nodiscard]] StereoObservation projectObservation(
      LandmarkId id, const Eigen::Vector3d& point_w,
      const Eigen::Vector3d& camera_w, bool mono )
  {
    const Eigen::Vector3d point_left = point_w - camera_w;
    EXPECT_GT( point_left.z(), 0.0 );
    return StereoObservation{
        .id = id,
        .left_pixel =
            Eigen::Vector2d{ 400.0 * point_left.x() / point_left.z() + 320.0,
                             400.0 * point_left.y() / point_left.z() + 240.0 },
        .disparity_px = mono ? 0.0 : 400.0 * 0.12 / point_left.z() };
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

  [[nodiscard]] std::vector<StereoObservation> makeDenseMixedObservations(
      const std::vector<LandmarkId>& mono_ids, double phase_px,
      double camera_x_m )
  {
    std::vector<StereoObservation> observations;
    observations.reserve( 48U );
    for ( int depth_index = 0;
          depth_index < 3 && observations.size() < 48U; ++depth_index )
    {
      for ( int y_index = -2;
            y_index <= 2 && observations.size() < 48U; ++y_index )
      {
        for ( int x_index = -2;
              x_index <= 2 && observations.size() < 48U; ++x_index )
        {
          const double x =
              0.15 * static_cast<double>( x_index ) - camera_x_m;
          const double      y = 0.12 * static_cast<double>( y_index );
          const double      z = 4.5 + 0.4 * static_cast<double>( depth_index );
          StereoObservation observation{
              .id = LandmarkId{ observations.size() + 1U },
              .left_pixel =
                  Eigen::Vector2d{ 400.0 * x / z + 320.0,
                                   400.0 * y / z + 240.0 },
              .disparity_px = 400.0 * 0.12 / z };
          if ( std::find( mono_ids.begin(), mono_ids.end(), observation.id ) !=
               mono_ids.end() )
          {
            observation.disparity_px = 0.0;
            const double sign =
                ( observation.id % 2U ) == 0U ? -1.0 : 1.0;
            observation.left_pixel.x() += sign * phase_px;
          }
          observations.push_back( observation );
        }
      }
    }
    return observations;
  }

}  // namespace

TEST( MappedLandmarkBearingAdmission, StereoControlHasNoMonoFactors )
{
  EstimatorOptions options;
  options.min_landmark_observations = 2;
  options.enable_pnp_init           = false;
  options.enable_outlier_cull       = false;
  options.enable_outlier_reopt      = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto root = estimator.update(
      makeMeasurement( 100'000'000, makeRootObservations() ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  const auto result = estimator.update(
      makeMeasurement( 200'000'000, makeRootObservations() ) );

  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  EXPECT_EQ( result.diagnostics.num_current_visual_factors, 10U );
  EXPECT_EQ( result.diagnostics.num_current_mono_visual_factors, 0U );
  EXPECT_EQ( result.diagnostics.m_vio.m_visual_factors, 20U );
}

TEST( MappedLandmarkBearingAdmission, MappedMonoAddsOneProjectionFactor )
{
  EstimatorOptions options;
  options.min_landmark_observations = 2;
  options.enable_pnp_init           = false;
  options.enable_outlier_cull       = false;
  options.enable_outlier_reopt      = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto root = estimator.update(
      makeMeasurement( 100'000'000, makeRootObservations() ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  std::vector<StereoObservation> observations;
  observations.push_back(
      makeObservation( LandmarkId{ 1U }, 0.0, 0.0 ) );
  const auto result = estimator.update(
      makeMeasurement( 200'000'000, std::move( observations ) ) );

  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  ASSERT_TRUE( result.estimate.has_value() );
  EXPECT_EQ( result.diagnostics.num_mapped_observations, 1U );
  EXPECT_EQ( result.diagnostics.num_shared, 0U );
  EXPECT_EQ( result.diagnostics.num_current_visual_factors, 1U );
  EXPECT_EQ( result.diagnostics.num_current_mono_visual_factors, 1U );
  EXPECT_EQ( result.diagnostics.m_vio.m_visual_factors, 2U );
}

TEST( MappedLandmarkBearingAdmission,
      EarlierMonoAttachesWhenLaterStereoSeedsLandmark )
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

  std::vector<StereoObservation> mono_observations;
  mono_observations.push_back(
      makeObservation( LandmarkId{ 101U }, 0.0, 14.0 ) );
  const auto mono = estimator.update(
      makeMeasurement( 200'000'000, std::move( mono_observations ) ) );
  ASSERT_EQ( mono.status, UpdateStatus::kOk ) << mono.message;
  EXPECT_EQ( mono.diagnostics.num_seeded_landmarks, 0U );
  EXPECT_EQ( mono.diagnostics.num_current_visual_factors, 0U );

  std::vector<StereoObservation> stereo_observations;
  stereo_observations.push_back(
      makeObservation( LandmarkId{ 101U }, 12.0, 14.0 ) );
  const auto stereo = estimator.update(
      makeMeasurement( 300'000'000, std::move( stereo_observations ) ) );

  ASSERT_EQ( stereo.status, UpdateStatus::kOk ) << stereo.message;
  EXPECT_EQ( stereo.diagnostics.num_shared, 0U );
  EXPECT_EQ( stereo.diagnostics.num_seeded_landmarks, 1U );
  EXPECT_EQ( stereo.diagnostics.num_landmarks, 1U );
  EXPECT_EQ( stereo.diagnostics.num_current_visual_factors, 1U );
  EXPECT_EQ( stereo.diagnostics.num_current_mono_visual_factors, 0U );
  EXPECT_EQ( stereo.diagnostics.m_vio.m_visual_factors, 2U );
}

TEST( MappedLandmarkBearingQuality, MixedRmsCountsMonoFactorOnce )
{
  EstimatorOptions options;
  options.min_landmark_observations = 2;
  options.enable_pnp_init           = false;
  options.enable_outlier_cull       = false;
  options.enable_outlier_reopt      = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto root = estimator.update(
      makeMeasurement( 100'000'000, makeRootObservations() ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  std::vector<StereoObservation> observations;
  observations.push_back(
      makeObservation( LandmarkId{ 1U }, 0.0, 3.0 ) );
  const auto result = estimator.update(
      makeMeasurement( 200'000'000, std::move( observations ) ) );

  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  EXPECT_NEAR( result.diagnostics.reproj_rms_before_px,
               std::sqrt( 9.0 / 2.0 ), 1e-9 );
}

TEST( MappedLandmarkBearingQuality, StereoOnlyRmsKeepsExistingDefinition )
{
  EstimatorOptions options;
  options.min_landmark_observations = 2;
  options.enable_pnp_init           = false;
  options.enable_outlier_cull       = false;
  options.enable_outlier_reopt      = false;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto root = estimator.update(
      makeMeasurement( 100'000'000, makeRootObservations() ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  std::vector<StereoObservation> observations;
  observations.push_back(
      makeObservation( LandmarkId{ 1U }, 12.0, 3.0 ) );
  const auto result = estimator.update(
      makeMeasurement( 200'000'000, std::move( observations ) ) );

  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  EXPECT_NEAR( result.diagnostics.reproj_rms_before_px, 3.0, 1e-9 );
}

TEST( MappedLandmarkBearingQuality, MixedMeanCullRequiresFourFactors )
{
  EstimatorOptions options;
  options.min_landmark_observations = 2;
  options.enable_pnp_init           = false;
  options.enable_outlier_cull       = true;
  options.enable_outlier_reopt      = false;
  options.huber_k_px                = 0.0;
  options.outlier_avg_reproj_px     = 1.0;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto root = estimator.update(
      makeMeasurement( 100'000'000,
                       makeDenseMixedObservations( {}, 0.0, 0.0 ) ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  const std::vector<LandmarkId> mono_ids{ LandmarkId{ 1U } };
  const auto                    second = estimator.update( makeMeasurement(
      200'000'000, makeDenseMixedObservations( mono_ids, 80.0, 0.05 ) ) );
  ASSERT_EQ( second.status, UpdateStatus::kOk ) << second.message;
  EXPECT_EQ( second.diagnostics.outliers_culled, 0U );

  const auto third = estimator.update( makeMeasurement(
      300'000'000, makeDenseMixedObservations( mono_ids, -80.0, 0.10 ) ) );
  ASSERT_EQ( third.status, UpdateStatus::kOk ) << third.message;
  EXPECT_EQ( third.diagnostics.outliers_culled, 0U );

  const auto fourth = estimator.update( makeMeasurement(
      400'000'000, makeDenseMixedObservations( mono_ids, 80.0, 0.15 ) ) );
  ASSERT_EQ( fourth.status, UpdateStatus::kOk ) << fourth.message;
  EXPECT_EQ( fourth.diagnostics.outliers_culled, 1U )
      << "visual=" << fourth.diagnostics.m_vio.m_visual_factors
      << " mono=" << fourth.diagnostics.num_current_mono_visual_factors
      << " rms_before=" << fourth.diagnostics.reproj_rms_before_px
      << " rms_after=" << fourth.diagnostics.reproj_rms_after_px
      << " probe_max=" << fourth.diagnostics.probe_res_max_px
      << " cheirality=" << fourth.diagnostics.num_cheirality
      << " landmarks=" << fourth.diagnostics.num_landmarks;
  EXPECT_NE( std::find( fourth.diagnostics.culled_landmark_ids.begin(),
                        fourth.diagnostics.culled_landmark_ids.end(),
                        LandmarkId{ 1U } ),
             fourth.diagnostics.culled_landmark_ids.end() );
}

TEST( MappedLandmarkBearingQuality,
      MixedMeanCullTriggersSuccessfulReoptimization )
{
  EstimatorOptions options;
  options.min_landmark_observations = 2;
  options.enable_pnp_init           = false;
  options.enable_outlier_cull       = true;
  options.enable_outlier_reopt      = true;
  options.huber_k_px                = 0.0;
  options.outlier_avg_reproj_px     = 1.0;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  const auto root = estimator.update(
      makeMeasurement( 100'000'000,
                       makeDenseMixedObservations( {}, 0.0, 0.0 ) ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  const std::vector<LandmarkId> mono_ids{
      LandmarkId{ 1U }, LandmarkId{ 2U }, LandmarkId{ 3U }, LandmarkId{ 4U } };
  const auto second = estimator.update( makeMeasurement(
      200'000'000, makeDenseMixedObservations( mono_ids, 80.0, 0.05 ) ) );
  ASSERT_EQ( second.status, UpdateStatus::kOk ) << second.message;
  EXPECT_EQ( second.diagnostics.outliers_culled, 0U );

  const auto third = estimator.update( makeMeasurement(
      300'000'000, makeDenseMixedObservations( mono_ids, -80.0, 0.10 ) ) );
  ASSERT_EQ( third.status, UpdateStatus::kOk ) << third.message;
  EXPECT_EQ( third.diagnostics.outliers_culled, 0U );

  const auto fourth = estimator.update( makeMeasurement(
      400'000'000, makeDenseMixedObservations( mono_ids, 80.0, 0.15 ) ) );
  ASSERT_EQ( fourth.status, UpdateStatus::kOk ) << fourth.message;
  EXPECT_GE( fourth.diagnostics.outliers_culled, mono_ids.size() );
  EXPECT_TRUE( fourth.diagnostics.outlier_reopt );
  EXPECT_FALSE( fourth.diagnostics.outlier_reopt_failed );
  EXPECT_GE( fourth.diagnostics.outlier_reopt_rounds, 1U );
  EXPECT_EQ( fourth.diagnostics.num_current_mono_visual_factors, 0U );
  for ( const LandmarkId id : mono_ids )
  {
    EXPECT_NE( std::find( fourth.diagnostics.culled_landmark_ids.begin(),
                          fourth.diagnostics.culled_landmark_ids.end(), id ),
               fourth.diagnostics.culled_landmark_ids.end() );
  }
}

TEST( MappedLandmarkBearingQuality, MonoCheiralityCullsMappedLandmark )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;
  options.min_landmark_observations       = 2;
  options.min_shared_landmarks            = 2;
  // Isolate the backend cheirality path from PnP's observation mask.
  options.enable_pnp_init      = false;
  options.enable_outlier_cull  = false;
  options.enable_outlier_reopt = false;
  options.huber_k_px           = 0.0;
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(), options );

  std::vector<Eigen::Vector3d> far_landmarks;
  far_landmarks.reserve( 48U );
  for ( int depth_index = 0;
        depth_index < 3 && far_landmarks.size() < 48U; ++depth_index )
  {
    for ( int y_index = -2;
          y_index <= 2 && far_landmarks.size() < 48U; ++y_index )
    {
      for ( int x_index = -2;
            x_index <= 2 && far_landmarks.size() < 48U; ++x_index )
      {
        far_landmarks.emplace_back(
            0.25 * static_cast<double>( x_index ),
            0.20 * static_cast<double>( y_index ),
            8.0 + 0.8 * static_cast<double>( depth_index ) );
      }
    }
  }
  const Eigen::Vector3d            near_landmark{ 0.15, 0.0, 1.5 };
  const LandmarkId                 near_id{ 99U };
  const std::vector<double>        camera_z{ 0.0, 0.3, 0.6, 0.9, 1.2, 3.0 };
  std::optional<StereoObservation> locked_near;

  for ( std::size_t frame_index = 0; frame_index < camera_z.size();
        ++frame_index )
  {
    const Eigen::Vector3d          camera_w{ 0.0, 0.0, camera_z[ frame_index ] };
    std::vector<StereoObservation> observations;
    observations.reserve( far_landmarks.size() + 1U );
    for ( std::size_t landmark_index = 0;
          landmark_index < far_landmarks.size(); ++landmark_index )
    {
      observations.push_back( projectObservation(
          LandmarkId{ landmark_index + 1U }, far_landmarks[ landmark_index ],
          camera_w, false ) );
    }
    if ( frame_index + 1U < camera_z.size() )
    {
      const StereoObservation near =
          projectObservation( near_id, near_landmark, camera_w, false );
      if ( frame_index + 2U == camera_z.size() )
      {
        locked_near = near;
      }
      observations.push_back( near );
    }
    else
    {
      ASSERT_TRUE( locked_near.has_value() );
      locked_near->disparity_px = 0.0;
      observations.push_back( *locked_near );
    }

    auto measurement = makeMeasurement(
        static_cast<std::int64_t>( frame_index + 1U ) * 100'000'000,
        std::move( observations ) );
    if ( frame_index + 1U == camera_z.size() )
    {
      auto& imu = std::get<phad::sensor::RawImuInterval>( measurement.m_imu );
      for ( phad::sensor::ImuMeasurement& sample : imu.m_samples )
      {
        // Seed the graph beyond the near landmark to trigger cheirality.
        sample.accel_mps2[ 2 ] += 400.0;
      }
    }

    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    if ( frame_index + 1U == camera_z.size() )
    {
      EXPECT_EQ( result.diagnostics.num_current_mono_visual_factors, 1U );
      ASSERT_TRUE( result.estimate.has_value() );
      EXPECT_GE( result.diagnostics.num_cheirality, 1U )
          << "pose_z=" << result.estimate->T_W_B.translation().z()
          << " rms_before=" << result.diagnostics.reproj_rms_before_px
          << " rms_after=" << result.diagnostics.reproj_rms_after_px
          << " visual=" << result.diagnostics.m_vio.m_visual_factors
          << " landmarks=" << result.diagnostics.num_landmarks;
      EXPECT_EQ( result.diagnostics.outliers_culled, 0U );
      EXPECT_NE( std::find( result.diagnostics.culled_landmark_ids.begin(),
                            result.diagnostics.culled_landmark_ids.end(),
                            near_id ),
                 result.diagnostics.culled_landmark_ids.end() );
    }
  }
}

TEST( MappedLandmarkBearingQuality, MonoPrimaryFailureRollsBackPacket )
{
  EstimatorOptions options;
  options.min_landmark_observations = 2;
  options.enable_pnp_init           = false;
  options.enable_outlier_cull       = false;
  options.enable_outlier_reopt      = false;
  VioEstimator subject( makeCalibration(),
                        phad::test_support::testImuParameters(), options );
  VioEstimator control( makeCalibration(),
                        phad::test_support::testImuParameters(), options );

  for ( VioEstimator* estimator : { &subject, &control } )
  {
    const auto root = estimator->update(
        makeMeasurement( 100'000'000, makeRootObservations() ) );
    ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;
    const auto supported = estimator->update(
        makeMeasurement( 200'000'000, makeRootObservations() ) );
    ASSERT_EQ( supported.status, UpdateStatus::kOk ) << supported.message;
  }

  std::vector<StereoObservation> explosive{
      makeObservation( LandmarkId{ 1U }, 0.0, 0.0 ) };
  explosive.front().left_pixel.x() =
      std::numeric_limits<double>::max() / 4.0;
  const auto failed = subject.update(
      makeMeasurement( 300'000'000, std::move( explosive ) ) );
  ASSERT_EQ( failed.status, UpdateStatus::kFailed ) << failed.message;
  EXPECT_FALSE( failed.estimate.has_value() );
  EXPECT_EQ( failed.diagnostics.num_current_visual_factors, 0U );
  EXPECT_EQ( failed.diagnostics.num_current_mono_visual_factors, 0U );
  EXPECT_EQ( subject.observationTimestamps( LandmarkId{ 1U } ),
             control.observationTimestamps( LandmarkId{ 1U } ) );

  const VioMeasurement legal = makeMeasurement(
      300'000'000,
      { makeObservation( LandmarkId{ 1U }, 0.0, 0.0 ) } );
  const auto after_failed = subject.update( legal );
  const auto direct       = control.update( legal );
  ASSERT_EQ( after_failed.status, UpdateStatus::kOk ) << after_failed.message;
  ASSERT_EQ( direct.status, UpdateStatus::kOk ) << direct.message;
  ASSERT_TRUE( after_failed.estimate.has_value() );
  ASSERT_TRUE( direct.estimate.has_value() );
  EXPECT_TRUE( after_failed.estimate->T_W_B.matrix().isApprox(
      direct.estimate->T_W_B.matrix(), 1e-12 ) );
  EXPECT_TRUE( after_failed.estimate->m_v_W_B.isApprox(
      direct.estimate->m_v_W_B, 1e-12 ) );
  EXPECT_EQ( after_failed.diagnostics.window_size,
             direct.diagnostics.window_size );
  EXPECT_EQ( after_failed.diagnostics.num_landmarks,
             direct.diagnostics.num_landmarks );
  EXPECT_EQ( after_failed.diagnostics.num_current_visual_factors,
             direct.diagnostics.num_current_visual_factors );
  EXPECT_EQ( after_failed.diagnostics.num_current_mono_visual_factors,
             direct.diagnostics.num_current_mono_visual_factors );
  EXPECT_EQ( after_failed.diagnostics.m_vio.m_visual_factors,
             direct.diagnostics.m_vio.m_visual_factors );
  EXPECT_EQ( after_failed.diagnostics.unsupported_span_ns,
             direct.diagnostics.unsupported_span_ns );
  EXPECT_EQ( subject.observationTimestamps( LandmarkId{ 1U } ),
             control.observationTimestamps( LandmarkId{ 1U } ) );
}

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
  EXPECT_EQ( result.diagnostics.num_current_visual_factors, 2U );
  EXPECT_EQ( result.diagnostics.num_current_mono_visual_factors, 1U );
  EXPECT_EQ( result.diagnostics.m_vio.m_visual_factors, 4U );
  EXPECT_FALSE( result.diagnostics.pnp_success );
  EXPECT_EQ( result.diagnostics.m_vio.m_visual_coast_duration_ns,
             100'000'000 );
}
