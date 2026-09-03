#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/internal/vio_estimator_test_access.hpp"
#include "phad/estimator/vio_estimator.hpp"
#include "phad/sensor/rigid_transform.hpp"
#include "tests/estimator/vio_test_utils.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::LandmarkId;
  using phad::estimator::PnpGuessRetryOutcome;
  using phad::estimator::PnpProbeResult;
  using phad::estimator::StereoObservation;
  using phad::estimator::UpdateStatus;
  using phad::estimator::VioEstimator;
  using phad::estimator::VioMeasurement;
  using phad::estimator::VioUpdateResult;
  using phad::estimator::internal::VioEstimatorTestAccess;
  using phad::sensor::RigidTransform;

  RectifiedStereoCalibration makeCalibration()
  {
    auto rigid = RigidTransform::create( Eigen::Isometry3d::Identity().matrix() )
                     .value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480, std::move( rigid ) )
        .value();
  }

  [[nodiscard]] StereoObservation projectLandmark(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d& T_W_B, LandmarkId id,
      const Eigen::Vector3d& point_W )
  {
    Eigen::Isometry3d T_B_C          = Eigen::Isometry3d::Identity();
    T_B_C.linear()                   = calibration.T_B_left_rectified().rotation();
    T_B_C.translation()              = calibration.T_B_left_rectified().translation();
    const Eigen::Vector3d point_left = ( T_W_B * T_B_C ).inverse() * point_W;
    const double          z          = point_left.z();
    EXPECT_GT( z, 0.0 );
    const double u_l =
        calibration.fxPixels() * point_left.x() / z + calibration.cxPixels();
    const double v =
        calibration.fyPixels() * point_left.y() / z + calibration.cyPixels();
    const double disparity =
        calibration.fxPixels() * calibration.baselineM() / z;
    return StereoObservation{ id, Eigen::Vector2d( u_l, v ), disparity };
  }

  VioMeasurement makeFrame(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d& T_W_B, std::int64_t timestamp_ns,
      const std::vector<Eigen::Vector3d>& landmarks_W,
      const std::vector<LandmarkId>&      ids )
  {
    VioMeasurement measurement;
    measurement.m_timestamp = phad::common::Timestamp{ timestamp_ns };
    measurement.m_imu       = phad::test_support::stationaryImuPayload(
        measurement.m_timestamp );
    for ( std::size_t index = 0; index < landmarks_W.size(); ++index )
    {
      measurement.m_observations.push_back( projectLandmark(
          calibration, T_W_B, ids[ index ], landmarks_W[ index ] ) );
    }
    return measurement;
  }

  std::vector<Eigen::Isometry3d> translatingPoses( int count, double step_m )
  {
    std::vector<Eigen::Isometry3d> poses;
    poses.reserve( static_cast<std::size_t>( count ) );
    for ( int index = 0; index < count; ++index )
    {
      Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
      T_W_B.translation() =
          Eigen::Vector3d( step_m * static_cast<double>( index ), 0.0, 0.0 );
      poses.push_back( T_W_B );
    }
    return poses;
  }

  std::vector<LandmarkId> sequentialIds( std::size_t count, LandmarkId start = 1 )
  {
    std::vector<LandmarkId> ids;
    ids.reserve( count );
    for ( std::size_t index = 0; index < count; ++index )
    {
      ids.push_back( start + static_cast<LandmarkId>( index ) );
    }
    return ids;
  }

  void offsetLeftPixel( VioMeasurement& measurement, LandmarkId id,
                        const Eigen::Vector2d& delta_px )
  {
    for ( StereoObservation& observation : measurement.m_observations )
    {
      if ( observation.id == id )
      {
        observation.left_pixel += delta_px;
        return;
      }
    }
    FAIL() << "landmark id " << id << " not found in measurement";
  }

  [[nodiscard]] bool hasTimestamp(
      const std::vector<phad::common::Timestamp>& stamps,
      std::int64_t                                timestamp_ns )
  {
    return std::find( stamps.begin(), stamps.end(),
                      phad::common::Timestamp{ timestamp_ns } ) != stamps.end();
  }

  // Values chosen to keep every landmark well within the frustum across the
  // small translations used below.
  const std::vector<Eigen::Vector3d> kLandmarksA{
      { 0.4, 0.1, 5.0 },
      { -0.3, 0.2, 4.5 },
      { 0.1, -0.25, 6.0 },
      { 0.6, -0.1, 5.5 },
      { -0.5, -0.2, 4.8 },
      { 0.0, 0.3, 5.2 },
      { 0.25, 0.15, 4.2 },
      { -0.2, -0.15, 5.8 },
      { 0.35, -0.05, 5.3 },
      { -0.15, 0.25, 4.6 },
  };

  const std::vector<Eigen::Vector3d> kLandmarksB{
      { 1.4, -0.3, 5.4 },
      { 0.9, 0.35, 4.9 },
      { 1.1, -0.15, 6.2 },
      { 1.6, 0.05, 5.1 },
      { 0.65, -0.4, 4.7 },
      { 1.0, 0.2, 5.6 },
      { 1.25, -0.2, 4.4 },
      { 0.8, 0.3, 5.9 },
      { 1.35, -0.05, 5.0 },
      { 0.85, 0.15, 4.8 },
  };

  // Close to the optical axis but spread in depth. A forward-shifted camera
  // changes disparity more than left-image position for this set, allowing a
  // deterministic left-PnP/stereo disagreement fixture.
  const std::vector<Eigen::Vector3d> kNearAxisLandmarks{
      { 0.02, 0.01, 4.0 },
      { -0.03, 0.02, 4.5 },
      { 0.01, -0.04, 5.0 },
      { 0.05, -0.01, 5.5 },
      { -0.04, -0.03, 6.0 },
      { 0.00, 0.04, 6.5 },
      { 0.03, 0.03, 4.2 },
      { -0.02, -0.05, 5.8 },
      { 0.04, -0.02, 6.8 },
      { -0.05, 0.01, 7.0 },
  };

  EstimatorOptions defaultPnpOptions()
  {
    EstimatorOptions options;
    options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.window_size                     = 5;
    options.min_shared_landmarks            = 3;
    options.min_pnp_inliers                 = 10;
    options.enable_pnp_init                 = true;
    return options;
  }

  void expectFiniteCurrentGraphProbe(
      const phad::estimator::UpdateDiagnostics& diagnostics )
  {
    ASSERT_TRUE(
        diagnostics.probe_current_pose_translation_delta_m.has_value() );
    EXPECT_TRUE( std::isfinite(
        *diagnostics.probe_current_pose_translation_delta_m ) );
    ASSERT_TRUE(
        diagnostics.probe_current_pose_rotation_delta_rad.has_value() );
    EXPECT_TRUE(
        std::isfinite( *diagnostics.probe_current_pose_rotation_delta_rad ) );
    ASSERT_TRUE( diagnostics.probe_current_velocity_delta_mps.has_value() );
    EXPECT_TRUE(
        std::isfinite( *diagnostics.probe_current_velocity_delta_mps ) );
    ASSERT_TRUE(
        diagnostics.probe_current_acc_bias_delta_mps2.has_value() );
    EXPECT_TRUE(
        std::isfinite( *diagnostics.probe_current_acc_bias_delta_mps2 ) );
    ASSERT_TRUE(
        diagnostics.probe_current_gyr_bias_delta_radps.has_value() );
    EXPECT_TRUE(
        std::isfinite( *diagnostics.probe_current_gyr_bias_delta_radps ) );
    ASSERT_TRUE( diagnostics.probe_primary_graph_error_seed.has_value() );
    EXPECT_TRUE(
        std::isfinite( *diagnostics.probe_primary_graph_error_seed ) );
    ASSERT_TRUE(
        diagnostics.probe_primary_graph_error_optimized.has_value() );
    EXPECT_TRUE(
        std::isfinite( *diagnostics.probe_primary_graph_error_optimized ) );
    ASSERT_TRUE(
        diagnostics.probe_primary_imu_factor_error_seed.has_value() );
    EXPECT_TRUE( std::isfinite(
        *diagnostics.probe_primary_imu_factor_error_seed ) );
    ASSERT_TRUE(
        diagnostics.probe_primary_imu_factor_error_optimized.has_value() );
    EXPECT_TRUE( std::isfinite(
        *diagnostics.probe_primary_imu_factor_error_optimized ) );
  }

  struct PnpFaultResults
  {
    VioUpdateResult m_fault;
    VioUpdateResult m_recovered;
  };

  struct PnpFaultScenario
  {
    PnpProbeResult                      m_failure;
    bool                                m_enable_product_retry = false;
    std::optional<PnpGuessRetryOutcome> m_retry_failure        = std::nullopt;
  };

  void expectGuessRetryNotAttempted(
      const phad::estimator::UpdateDiagnostics& diagnostics )
  {
    EXPECT_EQ( diagnostics.probe_pnp_guess_retry_outcome,
               PnpGuessRetryOutcome::kNotAttempted );
    EXPECT_FALSE(
        diagnostics.probe_pnp_guess_retry_proposal_rms_px.has_value() );
    EXPECT_FALSE(
        diagnostics.probe_pnp_guess_retry_guess_rms_px.has_value() );
    EXPECT_FALSE( diagnostics
                      .probe_pnp_guess_retry_cheiral_inlier_count.has_value() );
    EXPECT_FALSE(
        diagnostics.probe_pnp_guess_retry_min_inlier_depth_m.has_value() );
    EXPECT_FALSE(
        diagnostics.probe_pnp_guess_retry_translation_delta_m.has_value() );
    EXPECT_FALSE(
        diagnostics.probe_pnp_guess_retry_rotation_delta_rad.has_value() );
  }

  void expectExactPublicState( const VioUpdateResult& expected,
                               const VioUpdateResult& actual )
  {
    EXPECT_EQ( actual.status, expected.status );
    EXPECT_EQ( actual.message, expected.message );
    ASSERT_EQ( actual.estimate.has_value(), expected.estimate.has_value() );
    if ( expected.estimate.has_value() )
    {
      EXPECT_EQ( actual.estimate->timestamp, expected.estimate->timestamp );
      EXPECT_EQ( actual.estimate->m_segment_id,
                 expected.estimate->m_segment_id );
      EXPECT_TRUE( ( actual.estimate->T_W_B.matrix().array() ==
                     expected.estimate->T_W_B.matrix().array() )
                       .all() );
      EXPECT_TRUE( ( actual.estimate->m_v_W_B.array() ==
                     expected.estimate->m_v_W_B.array() )
                       .all() );
      EXPECT_TRUE( ( actual.estimate->m_bias.m_acc_mps2.array() ==
                     expected.estimate->m_bias.m_acc_mps2.array() )
                       .all() );
      EXPECT_TRUE( ( actual.estimate->m_bias.m_gyr_radps.array() ==
                     expected.estimate->m_bias.m_gyr_radps.array() )
                       .all() );
    }

    const auto& actual_diagnostics   = actual.diagnostics;
    const auto& expected_diagnostics = expected.diagnostics;
    EXPECT_EQ( actual_diagnostics.num_observations,
               expected_diagnostics.num_observations );
    EXPECT_EQ( actual_diagnostics.num_retained_observations,
               expected_diagnostics.num_retained_observations );
    EXPECT_EQ( actual_diagnostics.num_landmarks,
               expected_diagnostics.num_landmarks );
    EXPECT_EQ( actual_diagnostics.num_shared,
               expected_diagnostics.num_shared );
    EXPECT_EQ( actual_diagnostics.num_current_visual_factors,
               expected_diagnostics.num_current_visual_factors );
    EXPECT_EQ( actual_diagnostics.num_current_mono_visual_factors,
               expected_diagnostics.num_current_mono_visual_factors );
    EXPECT_EQ( actual_diagnostics.segment_id,
               expected_diagnostics.segment_id );
    EXPECT_EQ( actual_diagnostics.prior_key, expected_diagnostics.prior_key );
    EXPECT_EQ( actual_diagnostics.pnp_success,
               expected_diagnostics.pnp_success );
    EXPECT_EQ( actual_diagnostics.pnp_inliers,
               expected_diagnostics.pnp_inliers );
    EXPECT_EQ( actual_diagnostics.culled_landmark_ids,
               expected_diagnostics.culled_landmark_ids );
    EXPECT_EQ( actual_diagnostics.m_vio.m_root_prior_sets,
               expected_diagnostics.m_vio.m_root_prior_sets );
    EXPECT_EQ( actual_diagnostics.m_vio.m_completed_segment_id,
               expected_diagnostics.m_vio.m_completed_segment_id );
  }

  [[nodiscard]] PnpFaultResults runPnpProposalFailure(
      const PnpFaultScenario& scenario )
  {
    const auto calibration = makeCalibration();
    const auto ids         = sequentialIds( kNearAxisLandmarks.size(), 1 );

    EstimatorOptions options            = defaultPnpOptions();
    options.enable_pnp_cheirality_retry = scenario.m_enable_product_retry;
    options.enable_probe_b              = true;
    options.enable_outlier_cull         = false;
    options.enable_outlier_reopt        = false;
    VioEstimator estimator( calibration,
                            phad::test_support::testImuParameters(),
                            options );

    const Eigen::Isometry3d stationary = Eigen::Isometry3d::Identity();
    for ( std::int64_t frame = 1; frame <= 2; ++frame )
    {
      const auto seeded = estimator.update( makeFrame(
          calibration, stationary, frame * 50'000'000,
          kNearAxisLandmarks, ids ) );
      EXPECT_EQ( seeded.status, UpdateStatus::kOk ) << seeded.message;
    }

    std::vector<Eigen::Vector3d> landmarks_plus = kNearAxisLandmarks;
    std::vector<LandmarkId>      ids_plus       = ids;
    const LandmarkId             probe_id       = 99;
    landmarks_plus.emplace_back( 0.04, -0.03, 5.5 );
    ids_plus.push_back( probe_id );

    const auto introduced = estimator.update( makeFrame(
        calibration, stationary, 150'000'000, landmarks_plus, ids_plus ) );
    EXPECT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;
    EXPECT_EQ( introduced.diagnostics.num_landmarks,
               kNearAxisLandmarks.size() );

    auto fault_measurement = makeFrame(
        calibration, stationary, 200'000'000, landmarks_plus, ids_plus );
    offsetLeftPixel(
        fault_measurement, probe_id, Eigen::Vector2d( 80.0, 0.0 ) );
    VioEstimatorTestAccess::setNextPnpProposalFailure(
        estimator, scenario.m_failure );
    if ( scenario.m_retry_failure.has_value() )
    {
      VioEstimatorTestAccess::setNextPnpGuessRetryOutcome(
          estimator, *scenario.m_retry_failure );
    }
    VioUpdateResult fault = estimator.update( fault_measurement );
    EXPECT_EQ( fault.status, UpdateStatus::kOk ) << fault.message;
    EXPECT_TRUE( fault.estimate.has_value() );
    if ( fault.estimate.has_value() )
    {
      EXPECT_TRUE( fault.estimate->T_W_B.matrix().allFinite() );
    }
    EXPECT_FALSE( fault.diagnostics.pnp_success );
    EXPECT_EQ( fault.diagnostics.pnp_inliers, 0U );
    EXPECT_EQ( fault.diagnostics.num_landmarks, landmarks_plus.size() );
    EXPECT_TRUE( fault.diagnostics.probe_pnp_attempted );
    EXPECT_EQ( fault.diagnostics.probe_pnp_result, scenario.m_failure );
    EXPECT_EQ( fault.diagnostics.probe_pnp_correspondences,
               landmarks_plus.size() );
    // A shadow retry never applies its mask, so the current observation
    // remains in committed history and the probe landmark reaches the graph.
    const auto probe_timestamps = estimator.observationTimestamps( probe_id );
    EXPECT_EQ( probe_timestamps.size(), 2U );
    EXPECT_TRUE( hasTimestamp( probe_timestamps, 150'000'000 ) );
    EXPECT_TRUE( hasTimestamp( probe_timestamps, 200'000'000 ) );

    VioUpdateResult recovered = estimator.update( makeFrame(
        calibration, stationary, 250'000'000, landmarks_plus, ids_plus ) );
    EXPECT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
    EXPECT_TRUE( recovered.diagnostics.pnp_success );
    EXPECT_EQ( recovered.diagnostics.probe_pnp_result,
               PnpProbeResult::kAccepted );
    return PnpFaultResults{ .m_fault     = std::move( fault ),
                            .m_recovered = std::move( recovered ) };
  }

}  // namespace

TEST( StereoVoPnpTest, RejectsNonPositivePnpReproj )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.pnp_reproj_px                   = 0.0;
  EXPECT_THROW( ( VioEstimator{ makeCalibration(), phad::test_support::testImuParameters(), options } ),
                std::invalid_argument );
}

TEST( StereoVoPnpTest, RejectsInvalidPnpConfidence )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;    // tests seed at 2 frames
  options.pnp_confidence                  = 1.0;  // 要求 ∈ (0, 1)
  EXPECT_THROW( ( VioEstimator{ makeCalibration(), phad::test_support::testImuParameters(), options } ),
                std::invalid_argument );
}

TEST( StereoVoPnpTest, RejectsMinPnpInliersBelowFour )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.min_pnp_inliers                 = 3;
  EXPECT_THROW( ( VioEstimator{ makeCalibration(), phad::test_support::testImuParameters(), options } ),
                std::invalid_argument );
}

TEST( StereoVoPnpTest, ReportsProposalPoseNonFinite )
{
  const auto results = runPnpProposalFailure( PnpFaultScenario{
      .m_failure = PnpProbeResult::kProposalPoseNonFinite } );
  EXPECT_EQ( results.m_fault.diagnostics.probe_pnp_result,
             PnpProbeResult::kProposalPoseNonFinite );
  expectGuessRetryNotAttempted( results.m_fault.diagnostics );
}

TEST( StereoVoPnpTest, ReportsProposalRmsInvalidCorrespondence )
{
  const auto results = runPnpProposalFailure( PnpFaultScenario{
      .m_failure = PnpProbeResult::kProposalRmsInvalidCorrespondence } );
  EXPECT_EQ( results.m_fault.diagnostics.probe_pnp_result,
             PnpProbeResult::kProposalRmsInvalidCorrespondence );
  expectGuessRetryNotAttempted( results.m_fault.diagnostics );
}

TEST( StereoVoPnpTest, ReportsProposalRmsCheirality )
{
  const auto results = runPnpProposalFailure( PnpFaultScenario{
      .m_failure = PnpProbeResult::kProposalRmsCheirality } );
  EXPECT_FALSE( results.m_fault.diagnostics.pnp_success );
  EXPECT_EQ( results.m_fault.diagnostics.probe_pnp_result,
             PnpProbeResult::kProposalRmsCheirality );
  const auto& diagnostics = results.m_fault.diagnostics;
  EXPECT_EQ( diagnostics.probe_pnp_guess_retry_outcome,
             PnpGuessRetryOutcome::kWouldPassExistingPnpGates );
  ASSERT_TRUE(
      diagnostics.probe_pnp_guess_retry_proposal_rms_px.has_value() );
  EXPECT_TRUE( std::isfinite(
      *diagnostics.probe_pnp_guess_retry_proposal_rms_px ) );
  ASSERT_TRUE( diagnostics.probe_pnp_guess_retry_guess_rms_px.has_value() );
  EXPECT_TRUE( std::isfinite(
      *diagnostics.probe_pnp_guess_retry_guess_rms_px ) );
  ASSERT_TRUE( diagnostics
                   .probe_pnp_guess_retry_cheiral_inlier_count.has_value() );
  EXPECT_EQ(
      *diagnostics.probe_pnp_guess_retry_cheiral_inlier_count, 0U );
  ASSERT_TRUE(
      diagnostics.probe_pnp_guess_retry_min_inlier_depth_m.has_value() );
  EXPECT_GT( *diagnostics.probe_pnp_guess_retry_min_inlier_depth_m, 0.0 );
  ASSERT_TRUE(
      diagnostics.probe_pnp_guess_retry_translation_delta_m.has_value() );
  EXPECT_TRUE( std::isfinite(
      *diagnostics.probe_pnp_guess_retry_translation_delta_m ) );
  ASSERT_TRUE(
      diagnostics.probe_pnp_guess_retry_rotation_delta_rad.has_value() );
  EXPECT_TRUE( std::isfinite(
      *diagnostics.probe_pnp_guess_retry_rotation_delta_rad ) );
}

TEST( StereoVoPnpTest,
      ProductCheiralityRetryInstallsExactRansacInlierMaskWithoutProbe )
{
  const auto                   calibration = makeCalibration();
  std::vector<Eigen::Vector3d> landmarks   = kNearAxisLandmarks;
  landmarks.emplace_back( 0.04, -0.03, 5.5 );
  const auto              ids        = sequentialIds( landmarks.size(), 1 );
  const LandmarkId        outlier_id = ids.back();
  const Eigen::Isometry3d stationary = Eigen::Isometry3d::Identity();

  // A unique mono observation makes membership in the committed current-frame
  // mask observable through the public factor diagnostics. Repeat that tag for
  // every LandmarkId so a matching count cannot hide an identity swap.
  for ( const LandmarkId tagged_mono_id : ids )
  {
    SCOPED_TRACE( tagged_mono_id );
    EstimatorOptions options            = defaultPnpOptions();
    options.enable_pnp_cheirality_retry = true;
    options.enable_probe_b              = false;
    options.enable_outlier_cull         = false;
    options.enable_outlier_reopt        = false;
    VioEstimator estimator( calibration,
                            phad::test_support::testImuParameters(),
                            options );

    const auto root = estimator.update( makeFrame(
        calibration, stationary, 50'000'000, landmarks, ids ) );
    ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;
    const auto committed = estimator.update( makeFrame(
        calibration, stationary, 100'000'000, landmarks, ids ) );
    ASSERT_EQ( committed.status, UpdateStatus::kOk ) << committed.message;
    ASSERT_EQ( committed.diagnostics.num_landmarks, landmarks.size() );

    auto fault_measurement = makeFrame(
        calibration, stationary, 150'000'000, landmarks, ids );
    offsetLeftPixel(
        fault_measurement, outlier_id, Eigen::Vector2d( 80.0, 0.0 ) );
    for ( StereoObservation& observation : fault_measurement.m_observations )
    {
      if ( observation.id == tagged_mono_id )
      {
        observation.disparity_px = 0.0;
      }
    }
    VioEstimatorTestAccess::setNextPnpProposalFailure(
        estimator, PnpProbeResult::kProposalRmsCheirality );

    const auto recovered = estimator.update( fault_measurement );
    ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
    ASSERT_TRUE( recovered.estimate.has_value() );
    EXPECT_TRUE( recovered.diagnostics.pnp_success );
    EXPECT_FALSE( recovered.diagnostics.probe_pnp_attempted );
    EXPECT_EQ( recovered.diagnostics.pnp_inliers,
               kNearAxisLandmarks.size() );
    EXPECT_EQ( recovered.diagnostics.num_retained_observations,
               kNearAxisLandmarks.size() );
    EXPECT_EQ( recovered.diagnostics.num_current_visual_factors,
               kNearAxisLandmarks.size() );
    EXPECT_EQ( recovered.diagnostics.num_current_mono_visual_factors,
               tagged_mono_id == outlier_id ? 0U : 1U );

    for ( const LandmarkId id : ids )
    {
      const auto timestamps = estimator.observationTimestamps( id );
      ASSERT_EQ( timestamps.size(), 3U ) << "id=" << id;
      EXPECT_TRUE( hasTimestamp( timestamps, 50'000'000 ) ) << "id=" << id;
      EXPECT_TRUE( hasTimestamp( timestamps, 100'000'000 ) ) << "id=" << id;
      EXPECT_TRUE( hasTimestamp( timestamps, 150'000'000 ) ) << "id=" << id;
    }
  }
}

TEST( StereoVoPnpTest,
      FailedTypedCheiralityRetryIsExactlyEquivalentToGateOffFallback )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kNearAxisLandmarks.size(), 1 );

  const auto make_options = []( bool enable_retry ) {
    EstimatorOptions options            = defaultPnpOptions();
    options.enable_pnp_cheirality_retry = enable_retry;
    options.enable_probe_b              = false;
    options.enable_outlier_cull         = false;
    options.enable_outlier_reopt        = false;
    return options;
  };
  VioEstimator gate_off( calibration,
                         phad::test_support::testImuParameters(),
                         make_options( false ) );
  VioEstimator retry_failure( calibration,
                              phad::test_support::testImuParameters(),
                              make_options( true ) );

  const Eigen::Isometry3d stationary = Eigen::Isometry3d::Identity();
  for ( std::int64_t frame = 1; frame <= 2; ++frame )
  {
    const auto measurement = makeFrame(
        calibration, stationary, frame * 50'000'000,
        kNearAxisLandmarks, ids );
    const auto off_seeded = gate_off.update( measurement );
    const auto on_seeded  = retry_failure.update( measurement );
    ASSERT_EQ( off_seeded.status, UpdateStatus::kOk ) << off_seeded.message;
    ASSERT_EQ( on_seeded.status, UpdateStatus::kOk ) << on_seeded.message;
  }

  std::vector<Eigen::Vector3d> landmarks_plus = kNearAxisLandmarks;
  std::vector<LandmarkId>      ids_plus       = ids;
  const LandmarkId             probe_id       = 99;
  landmarks_plus.emplace_back( 0.04, -0.03, 5.5 );
  ids_plus.push_back( probe_id );

  const auto introduced_measurement = makeFrame(
      calibration, stationary, 150'000'000, landmarks_plus, ids_plus );
  const auto off_introduced = gate_off.update( introduced_measurement );
  const auto on_introduced  = retry_failure.update( introduced_measurement );
  ASSERT_EQ( off_introduced.status, UpdateStatus::kOk )
      << off_introduced.message;
  ASSERT_EQ( on_introduced.status, UpdateStatus::kOk )
      << on_introduced.message;

  auto fault_measurement = makeFrame(
      calibration, stationary, 200'000'000, landmarks_plus, ids_plus );
  offsetLeftPixel(
      fault_measurement, probe_id, Eigen::Vector2d( 80.0, 0.0 ) );
  VioEstimatorTestAccess::setNextPnpProposalFailure(
      gate_off, PnpProbeResult::kProposalRmsCheirality );
  VioEstimatorTestAccess::setNextPnpProposalFailure(
      retry_failure, PnpProbeResult::kProposalRmsCheirality );
  VioEstimatorTestAccess::setNextPnpGuessRetryOutcome(
      retry_failure, PnpGuessRetryOutcome::kSolveFailed );

  const auto off_fallback = gate_off.update( fault_measurement );
  const auto on_fallback  = retry_failure.update( fault_measurement );
  expectExactPublicState( off_fallback, on_fallback );
  EXPECT_FALSE( off_fallback.diagnostics.pnp_success );
  EXPECT_FALSE( on_fallback.diagnostics.pnp_success );
  EXPECT_FALSE( off_fallback.diagnostics.probe_pnp_attempted );
  EXPECT_FALSE( on_fallback.diagnostics.probe_pnp_attempted );
  for ( const LandmarkId id : ids_plus )
  {
    EXPECT_EQ( retry_failure.observationTimestamps( id ),
               gate_off.observationTimestamps( id ) );
  }

  const auto next_measurement = makeFrame(
      calibration, stationary, 250'000'000, landmarks_plus, ids_plus );
  const auto off_next = gate_off.update( next_measurement );
  const auto on_next  = retry_failure.update( next_measurement );
  expectExactPublicState( off_next, on_next );
  EXPECT_TRUE( off_next.diagnostics.pnp_success );
  EXPECT_TRUE( on_next.diagnostics.pnp_success );
  EXPECT_FALSE( off_next.diagnostics.probe_pnp_attempted );
  EXPECT_FALSE( on_next.diagnostics.probe_pnp_attempted );
}

TEST( StereoVoPnpTest, ReportsProposalRmsNonFinite )
{
  const auto results = runPnpProposalFailure( PnpFaultScenario{
      .m_failure = PnpProbeResult::kProposalRmsNonFinite } );
  EXPECT_EQ( results.m_fault.diagnostics.probe_pnp_result,
             PnpProbeResult::kProposalRmsNonFinite );
  expectGuessRetryNotAttempted( results.m_fault.diagnostics );
}

TEST( StereoVoPnpTest,
      ProductGateDoesNotRetryNonCheiralityProposalFailures )
{
  const std::vector<PnpProbeResult> failures{
      PnpProbeResult::kProposalPoseNonFinite,
      PnpProbeResult::kProposalRmsInvalidCorrespondence,
      PnpProbeResult::kProposalRmsNonFinite,
  };
  for ( const PnpProbeResult failure : failures )
  {
    SCOPED_TRACE( static_cast<int>( failure ) );
    const auto results = runPnpProposalFailure( PnpFaultScenario{
        .m_failure              = failure,
        .m_enable_product_retry = true } );
    EXPECT_FALSE( results.m_fault.diagnostics.pnp_success );
    EXPECT_EQ( results.m_fault.diagnostics.probe_pnp_result, failure );
    expectGuessRetryNotAttempted( results.m_fault.diagnostics );
  }
}

TEST( StereoVoPnpTest, CheiralityRetryFailureDoesNotRecurse )
{
  const auto results = runPnpProposalFailure( PnpFaultScenario{
      .m_failure              = PnpProbeResult::kProposalRmsCheirality,
      .m_enable_product_retry = true,
      .m_retry_failure        = PnpGuessRetryOutcome::kProposalRmsCheirality } );
  EXPECT_FALSE( results.m_fault.diagnostics.pnp_success );
  EXPECT_EQ( results.m_fault.diagnostics.probe_pnp_result,
             PnpProbeResult::kProposalRmsCheirality );
  EXPECT_EQ( results.m_fault.diagnostics.probe_pnp_guess_retry_outcome,
             PnpGuessRetryOutcome::kProposalRmsCheirality );
  EXPECT_TRUE( results.m_recovered.diagnostics.pnp_success );
  EXPECT_EQ( results.m_recovered.diagnostics.probe_pnp_result,
             PnpProbeResult::kAccepted );
  expectGuessRetryNotAttempted( results.m_recovered.diagnostics );
}

TEST( StereoVoPnpTest, ProposalFailureFaultArmRejectsNonFailureResult )
{
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(),
                          defaultPnpOptions() );
  EXPECT_THROW(
      VioEstimatorTestAccess::setNextPnpProposalFailure(
          estimator, PnpProbeResult::kAccepted ),
      std::invalid_argument );
}

TEST( StereoVoPnpTest, GuessRetryFaultArmRejectsNonFailureOutcomes )
{
  VioEstimator estimator( makeCalibration(),
                          phad::test_support::testImuParameters(),
                          defaultPnpOptions() );
  EXPECT_THROW(
      VioEstimatorTestAccess::setNextPnpGuessRetryOutcome(
          estimator, PnpGuessRetryOutcome::kNotAttempted ),
      std::invalid_argument );
  EXPECT_THROW(
      VioEstimatorTestAccess::setNextPnpGuessRetryOutcome(
          estimator, PnpGuessRetryOutcome::kWouldPassExistingPnpGates ),
      std::invalid_argument );
}

TEST( StereoVoPnpTest,
      ProductGateDoesNotChangePrimaryPnpSuccessPublicState )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 6, 0.05 );

  EstimatorOptions gate_off_options           = defaultPnpOptions();
  gate_off_options.enable_probe_b             = false;
  EstimatorOptions gate_on_options            = gate_off_options;
  gate_on_options.enable_pnp_cheirality_retry = true;
  VioEstimator gate_off( calibration,
                         phad::test_support::testImuParameters(),
                         gate_off_options );
  VioEstimator gate_on( calibration,
                        phad::test_support::testImuParameters(),
                        gate_on_options );

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t timestamp_ns =
        static_cast<std::int64_t>( index + 1U ) * 50'000'000;
    const auto measurement = makeFrame(
        calibration, poses[ index ], timestamp_ns, kLandmarksA, ids );
    const auto gate_off_result = gate_off.update( measurement );
    const auto gate_on_result  = gate_on.update( measurement );
    expectExactPublicState( gate_off_result, gate_on_result );

    if ( index == 0U )
    {
      EXPECT_FALSE( gate_off_result.diagnostics.pnp_success );
      EXPECT_FALSE( gate_on_result.diagnostics.pnp_success );
      continue;
    }
    EXPECT_TRUE( gate_off_result.diagnostics.pnp_success );
    EXPECT_TRUE( gate_on_result.diagnostics.pnp_success );
    EXPECT_FALSE( gate_off_result.diagnostics.probe_pnp_attempted );
    EXPECT_FALSE( gate_on_result.diagnostics.probe_pnp_attempted );
  }
}

TEST( StereoVoPnpTest, PnpSucceedsOnCleanMotion )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 6, 0.05 );

  EstimatorOptions options_pnp = defaultPnpOptions();
  options_pnp.enable_probe_b   = true;
  VioEstimator estimator_pnp( calibration, phad::test_support::testImuParameters(), options_pnp );

  EstimatorOptions options_cv = defaultPnpOptions();
  options_cv.enable_pnp_init  = false;
  VioEstimator estimator_cv( calibration, phad::test_support::testImuParameters(), options_cv );

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    const auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kLandmarksA, ids );

    const auto result_pnp = estimator_pnp.update( measurement );
    ASSERT_EQ( result_pnp.status, UpdateStatus::kOk ) << result_pnp.message;
    ASSERT_TRUE( result_pnp.estimate.has_value() );

    const auto result_cv = estimator_cv.update( measurement );
    ASSERT_EQ( result_cv.status, UpdateStatus::kOk ) << result_cv.message;
    ASSERT_TRUE( result_cv.estimate.has_value() );

    if ( index == 0 )
    {
      EXPECT_FALSE( result_pnp.diagnostics.pnp_success );
      EXPECT_FALSE( result_pnp.diagnostics.probe_pnp_attempted );
      EXPECT_EQ( result_pnp.diagnostics.probe_pnp_result,
                 PnpProbeResult::kNotAttempted );
      EXPECT_FALSE( result_cv.diagnostics.pnp_success );
      continue;
    }

    EXPECT_TRUE( result_pnp.diagnostics.pnp_success );
    EXPECT_GE( result_pnp.diagnostics.pnp_inliers, 10U );
    EXPECT_TRUE( result_pnp.diagnostics.probe_pnp_attempted );
    EXPECT_EQ( result_pnp.diagnostics.probe_pnp_result,
               PnpProbeResult::kAccepted );
    EXPECT_EQ( result_pnp.diagnostics.probe_pnp_correspondences,
               ids.size() );
    EXPECT_EQ( result_pnp.diagnostics.probe_pnp_inliers,
               result_pnp.diagnostics.pnp_inliers );
    ASSERT_TRUE(
        result_pnp.diagnostics.probe_pnp_proposal_rms_px.has_value() );
    EXPECT_TRUE( std::isfinite(
        *result_pnp.diagnostics.probe_pnp_proposal_rms_px ) );
    ASSERT_TRUE( result_pnp.diagnostics.probe_pnp_guess_rms_px.has_value() );
    EXPECT_TRUE(
        std::isfinite( *result_pnp.diagnostics.probe_pnp_guess_rms_px ) );
    ASSERT_TRUE( result_pnp.diagnostics
                     .probe_pnp_proposal_inlier_count.has_value() );
    EXPECT_EQ( *result_pnp.diagnostics.probe_pnp_proposal_inlier_count,
               result_pnp.diagnostics.pnp_inliers );
    ASSERT_TRUE( result_pnp.diagnostics
                     .probe_pnp_proposal_cheiral_inlier_count.has_value() );
    EXPECT_EQ(
        *result_pnp.diagnostics.probe_pnp_proposal_cheiral_inlier_count,
        0U );
    ASSERT_TRUE(
        result_pnp.diagnostics
            .probe_pnp_proposal_min_inlier_depth_m.has_value() );
    EXPECT_GT(
        *result_pnp.diagnostics.probe_pnp_proposal_min_inlier_depth_m,
        0.0 );
    ASSERT_TRUE(
        result_pnp.diagnostics
            .probe_pnp_proposal_min_inlier_landmark_id.has_value() );
    EXPECT_NE(
        std::find(
            ids.begin(), ids.end(),
            *result_pnp.diagnostics
                 .probe_pnp_proposal_min_inlier_landmark_id ),
        ids.end() );
    ASSERT_TRUE(
        result_pnp.diagnostics
            .probe_pnp_proposal_guess_translation_delta_m.has_value() );
    EXPECT_TRUE( std::isfinite(
        *result_pnp.diagnostics
             .probe_pnp_proposal_guess_translation_delta_m ) );
    ASSERT_TRUE(
        result_pnp.diagnostics
            .probe_pnp_proposal_guess_rotation_delta_rad.has_value() );
    EXPECT_TRUE( std::isfinite(
        *result_pnp.diagnostics
             .probe_pnp_proposal_guess_rotation_delta_rad ) );
    expectFiniteCurrentGraphProbe( result_pnp.diagnostics );
    EXPECT_FALSE( result_cv.diagnostics.pnp_success );

    // Clean synthetic motion + BA make both paths near-truth; gate on absolute
    // translation error (brief allows <0.05 m in lieu of a CV gap).
    const double err_pnp =
        ( result_pnp.estimate->T_W_B.translation() -
          poses[ index ].translation() )
            .norm();
    EXPECT_LT( err_pnp, 0.05 );
  }
}

TEST( StereoVoPnpTest, PnpSucceedsWithMappedLeftOnlyCorrespondences )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 2, 0.10 );

  EstimatorOptions options = defaultPnpOptions();
  VioEstimator     estimator( calibration, phad::test_support::testImuParameters(), options );

  const auto root = estimator.update( makeFrame(
      calibration, poses[ 0 ], 50'000'000, kLandmarksA, ids ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  auto left_only = makeFrame( calibration, poses[ 1 ], 100'000'000,
                              kLandmarksA, ids );
  for ( StereoObservation& observation : left_only.m_observations )
  {
    observation.disparity_px = 0.0;
  }

  const auto recovered = estimator.update( left_only );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  ASSERT_TRUE( recovered.estimate.has_value() );
  EXPECT_EQ( recovered.diagnostics.num_shared, 0U );
  EXPECT_EQ( recovered.diagnostics.num_mapped_observations, ids.size() );
  EXPECT_TRUE( recovered.diagnostics.pnp_success );
  EXPECT_EQ( recovered.diagnostics.pnp_inliers, ids.size() );
  EXPECT_EQ( recovered.diagnostics.num_current_mono_visual_factors,
             ids.size() );
  EXPECT_EQ( recovered.diagnostics.m_vio.m_visual_coast_duration_ns, 0 );
  EXPECT_EQ( recovered.diagnostics.unsupported_span_ns, 0 );
  EXPECT_LT( ( recovered.estimate->T_W_B.translation() -
               poses[ 1 ].translation() )
                 .norm(),
             0.05 );
}

TEST( StereoVoPnpTest, PnpMasksSharedOutliers )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 4, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  VioEstimator     estimator( calibration, phad::test_support::testImuParameters(), options );

  // Need shared > min_pnp_inliers so one outlier still leaves enough inliers.
  // Public surface has no window / landmarks_W accessors: prove mask via graph
  // count (probe stays at 1 window obs) + observationTimestamps + next-frame
  // num_shared (landmarks_W retained).
  const auto seed = estimator.update(
      makeFrame( calibration, poses[ 0 ], 50'000'000, kLandmarksA, ids ) );
  ASSERT_EQ( seed.status, UpdateStatus::kOk ) << seed.message;

  std::vector<Eigen::Vector3d> landmarks_plus = kLandmarksA;
  std::vector<LandmarkId>      ids_plus       = ids;
  const Eigen::Vector3d        probe_point{ 0.5, 0.0, 5.1 };
  const LandmarkId             probe_id = 99;
  landmarks_plus.push_back( probe_point );
  ids_plus.push_back( probe_id );

  const auto introduced = estimator.update( makeFrame(
      calibration, poses[ 1 ], 100'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;
  EXPECT_EQ( introduced.diagnostics.num_landmarks, kLandmarksA.size() );

  auto corrupted = makeFrame( calibration, poses[ 2 ], 150'000'000,
                              landmarks_plus, ids_plus );
  offsetLeftPixel( corrupted, probe_id, Eigen::Vector2d( 40.0, 0.0 ) );
  const auto masked = estimator.update( corrupted );
  ASSERT_EQ( masked.status, UpdateStatus::kOk ) << masked.message;
  EXPECT_TRUE( masked.diagnostics.pnp_success );
  EXPECT_GE( masked.diagnostics.pnp_inliers, 10U );
  EXPECT_LT( masked.diagnostics.pnp_inliers,
             static_cast<std::uint32_t>( landmarks_plus.size() ) );
  // Masked current obs → probe still has one window sighting → out of graph.
  EXPECT_EQ( masked.diagnostics.num_landmarks, kLandmarksA.size() );
  EXPECT_EQ( masked.diagnostics.num_observations, landmarks_plus.size() );

  const auto stamps = estimator.observationTimestamps( probe_id );
  EXPECT_EQ( stamps.size(), 2U );
  EXPECT_TRUE( hasTimestamp( stamps, 100'000'000 ) );
  EXPECT_TRUE( hasTimestamp( stamps, 150'000'000 ) );

  const auto recovered = estimator.update( makeFrame(
      calibration, poses[ 3 ], 200'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  EXPECT_EQ( recovered.diagnostics.num_shared, landmarks_plus.size() );
  EXPECT_EQ( recovered.diagnostics.num_landmarks, kLandmarksA.size() + 1U );
}

TEST( StereoVoPnpTest,
      PnpMaskDropsMappedStereoAndMonoOutliersButRetainsUnmappedHistory )
{
  const auto                   calibration = makeCalibration();
  std::vector<Eigen::Vector3d> landmarks   = kLandmarksA;
  landmarks.emplace_back( 0.55, 0.30, 5.7 );
  landmarks.emplace_back( -0.45, 0.35, 5.4 );
  const auto ids   = sequentialIds( landmarks.size(), 1 );
  const auto poses = translatingPoses( 2, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  VioEstimator     estimator( calibration,
                              phad::test_support::testImuParameters(),
                              options );

  const auto root = estimator.update( makeFrame(
      calibration, poses[ 0 ], 50'000'000, landmarks, ids ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  auto             mixed                 = makeFrame( calibration, poses[ 1 ], 100'000'000, landmarks,
                                                      ids );
  const LandmarkId good_mono_id          = ids[ 0 ];
  const LandmarkId stereo_outlier        = ids[ ids.size() - 2 ];
  const LandmarkId mono_outlier          = ids.back();
  const LandmarkId unmapped_id           = 99;
  mixed.m_observations[ 0 ].disparity_px = 0.0;
  offsetLeftPixel( mixed, stereo_outlier, Eigen::Vector2d( 40.0, 0.0 ) );
  offsetLeftPixel( mixed, mono_outlier, Eigen::Vector2d( 40.0, 0.0 ) );
  for ( StereoObservation& observation : mixed.m_observations )
  {
    if ( observation.id == mono_outlier )
    {
      observation.disparity_px = 0.0;
    }
  }
  StereoObservation unmapped = projectLandmark(
      calibration, poses[ 1 ], unmapped_id,
      Eigen::Vector3d( 0.2, -0.35, 5.0 ) );
  unmapped.disparity_px = 0.0;
  mixed.m_observations.push_back( unmapped );

  const auto masked = estimator.update( mixed );
  ASSERT_EQ( masked.status, UpdateStatus::kOk ) << masked.message;
  EXPECT_TRUE( masked.diagnostics.pnp_success );
  EXPECT_EQ( masked.diagnostics.pnp_inliers, 10U );
  EXPECT_EQ( masked.diagnostics.num_mapped_observations, ids.size() );
  EXPECT_EQ( masked.diagnostics.num_current_visual_factors, 10U );
  EXPECT_EQ( masked.diagnostics.num_current_mono_visual_factors, 1U );

  const auto good_mono_stamps =
      estimator.observationTimestamps( good_mono_id );
  const auto stereo_outlier_stamps =
      estimator.observationTimestamps( stereo_outlier );
  const auto mono_outlier_stamps =
      estimator.observationTimestamps( mono_outlier );
  const auto unmapped_stamps = estimator.observationTimestamps( unmapped_id );
  EXPECT_TRUE( hasTimestamp( good_mono_stamps, 100'000'000 ) );
  EXPECT_TRUE( hasTimestamp( stereo_outlier_stamps, 100'000'000 ) );
  EXPECT_TRUE( hasTimestamp( mono_outlier_stamps, 100'000'000 ) );
  EXPECT_TRUE( hasTimestamp( unmapped_stamps, 100'000'000 ) );
}

TEST( StereoVoPnpTest, FallsBackWhenPnpStereoRmsIsWorseThanGuess )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kNearAxisLandmarks.size(), 1 );

  EstimatorOptions options            = defaultPnpOptions();
  options.enable_pnp_cheirality_retry = true;
  options.enable_outlier_cull         = false;
  options.enable_outlier_reopt        = false;
  options.enable_probe_b              = true;
  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );

  const Eigen::Isometry3d stationary = Eigen::Isometry3d::Identity();
  for ( std::int64_t frame = 1; frame <= 2; ++frame )
  {
    const auto result = estimator.update( makeFrame(
        calibration, stationary, frame * 50'000'000, kNearAxisLandmarks,
        ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  std::vector<Eigen::Vector3d> landmarks_plus = kNearAxisLandmarks;
  std::vector<LandmarkId>      ids_plus       = ids;
  const LandmarkId             probe_id       = 99;
  landmarks_plus.emplace_back( 0.04, -0.03, 5.5 );
  ids_plus.push_back( probe_id );

  const auto introduced = estimator.update( makeFrame(
      calibration, stationary, 150'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;
  ASSERT_EQ( introduced.diagnostics.num_landmarks,
             kNearAxisLandmarks.size() );

  Eigen::Isometry3d wrong_pose = stationary;
  wrong_pose.translation().z() = 1.0;
  auto       inconsistent      = makeFrame( calibration, wrong_pose, 200'000'000,
                                            landmarks_plus, ids_plus );
  const auto guess_measurement = makeFrame(
      calibration, stationary, 200'000'000, landmarks_plus, ids_plus );
  for ( std::size_t index = 0; index < kNearAxisLandmarks.size(); ++index )
  {
    inconsistent.m_observations[ index ].disparity_px =
        guess_measurement.m_observations[ index ].disparity_px;
  }
  offsetLeftPixel( inconsistent, probe_id, Eigen::Vector2d( 80.0, 0.0 ) );

  const auto fallback = estimator.update( inconsistent );
  ASSERT_EQ( fallback.status, UpdateStatus::kOk ) << fallback.message;
  ASSERT_TRUE( fallback.estimate.has_value() );
  EXPECT_TRUE( fallback.estimate->T_W_B.matrix().allFinite() );
  EXPECT_FALSE( fallback.diagnostics.pnp_success );
  EXPECT_EQ( fallback.diagnostics.pnp_inliers, 0U );
  EXPECT_TRUE( fallback.diagnostics.probe_pnp_attempted );
  EXPECT_EQ( fallback.diagnostics.probe_pnp_result,
             PnpProbeResult::kProposalRmsRejected );
  EXPECT_EQ( fallback.diagnostics.probe_pnp_correspondences,
             landmarks_plus.size() );
  EXPECT_GE( fallback.diagnostics.probe_pnp_inliers,
             static_cast<std::uint32_t>( kNearAxisLandmarks.size() ) );
  ASSERT_TRUE(
      fallback.diagnostics.probe_pnp_proposal_rms_px.has_value() );
  ASSERT_TRUE( fallback.diagnostics.probe_pnp_guess_rms_px.has_value() );
  EXPECT_TRUE( std::isfinite(
      *fallback.diagnostics.probe_pnp_proposal_rms_px ) );
  EXPECT_TRUE(
      std::isfinite( *fallback.diagnostics.probe_pnp_guess_rms_px ) );
  EXPECT_GT( *fallback.diagnostics.probe_pnp_proposal_rms_px,
             *fallback.diagnostics.probe_pnp_guess_rms_px );
  expectGuessRetryNotAttempted( fallback.diagnostics );
  // Rejected proposal must not apply its mask: the probe's second sighting
  // reaches min_landmark_observations and therefore enters the graph.
  EXPECT_EQ( fallback.diagnostics.num_landmarks, landmarks_plus.size() );
}

TEST( StereoVoPnpTest, AcceptsPnpWithinStereoNoise )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kNearAxisLandmarks.size(), 1 );

  EstimatorOptions options = defaultPnpOptions();
  VioEstimator     estimator( calibration, phad::test_support::testImuParameters(), options );

  const Eigen::Isometry3d stationary = Eigen::Isometry3d::Identity();
  for ( std::int64_t frame = 1; frame <= 2; ++frame )
  {
    const auto result = estimator.update( makeFrame(
        calibration, stationary, frame * 50'000'000, kNearAxisLandmarks,
        ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  Eigen::Isometry3d slightly_wrong_pose = stationary;
  slightly_wrong_pose.translation().z() = 0.25;
  auto       noisy                      = makeFrame( calibration, slightly_wrong_pose, 150'000'000,
                                                     kNearAxisLandmarks, ids );
  const auto guess_measurement          = makeFrame(
      calibration, stationary, 150'000'000, kNearAxisLandmarks, ids );
  for ( std::size_t index = 0; index < kNearAxisLandmarks.size(); ++index )
  {
    noisy.m_observations[ index ].disparity_px =
        guess_measurement.m_observations[ index ].disparity_px;
  }

  const auto accepted = estimator.update( noisy );
  ASSERT_EQ( accepted.status, UpdateStatus::kOk ) << accepted.message;
  ASSERT_TRUE( accepted.estimate.has_value() );
  EXPECT_TRUE( accepted.estimate->T_W_B.matrix().allFinite() );
  EXPECT_TRUE( accepted.diagnostics.pnp_success );
  EXPECT_GE( accepted.diagnostics.pnp_inliers,
             static_cast<std::uint32_t>( kNearAxisLandmarks.size() ) );
}

TEST( StereoVoPnpTest, MixedScoreUsesOneL2NormPerInlierObservation )
{
  const auto              calibration = makeCalibration();
  const auto              ids         = sequentialIds( kNearAxisLandmarks.size(), 1 );
  const Eigen::Isometry3d stationary  = Eigen::Isometry3d::Identity();

  const auto run = [ & ]( std::size_t mono_count ) {
    EstimatorOptions options     = defaultPnpOptions();
    options.stereo_sigma_px      = 0.70;
    options.enable_outlier_cull  = false;
    options.enable_outlier_reopt = false;
    VioEstimator estimator( calibration,
                            phad::test_support::testImuParameters(),
                            options );
    for ( std::int64_t frame = 1; frame <= 2; ++frame )
    {
      const auto seeded = estimator.update( makeFrame(
          calibration, stationary, frame * 50'000'000,
          kNearAxisLandmarks, ids ) );
      EXPECT_EQ( seeded.status, UpdateStatus::kOk ) << seeded.message;
    }

    Eigen::Isometry3d proposal_pose = stationary;
    proposal_pose.translation().z() = 1.0;
    auto       mixed                = makeFrame( calibration, proposal_pose, 150'000'000,
                                                 kNearAxisLandmarks, ids );
    const auto guess                = makeFrame( calibration, stationary, 150'000'000,
                                                 kNearAxisLandmarks, ids );
    for ( std::size_t index = 0; index < mixed.m_observations.size(); ++index )
    {
      mixed.m_observations[ index ].disparity_px =
          index < mono_count ? 0.0
                             : guess.m_observations[ index ].disparity_px;
    }
    return estimator.update( mixed );
  };

  const auto three_mono = run( 3 );
  ASSERT_EQ( three_mono.status, UpdateStatus::kOk ) << three_mono.message;
  EXPECT_FALSE( three_mono.diagnostics.pnp_success );
  EXPECT_EQ( three_mono.diagnostics.pnp_inliers, 0U );

  const auto four_mono = run( 4 );
  ASSERT_EQ( four_mono.status, UpdateStatus::kOk ) << four_mono.message;
  EXPECT_TRUE( four_mono.diagnostics.pnp_success );
  EXPECT_EQ( four_mono.diagnostics.pnp_inliers, ids.size() );
}

TEST( StereoVoPnpTest, AcceptsFiniteProposalWhenPropagatedGuessCannotScore )
{
  const auto              calibration = makeCalibration();
  const auto              ids         = sequentialIds( kLandmarksA.size(), 1 );
  const Eigen::Isometry3d stationary  = Eigen::Isometry3d::Identity();

  EstimatorOptions options = defaultPnpOptions();
  VioEstimator     estimator( calibration,
                              phad::test_support::testImuParameters(),
                              options );
  const auto       root = estimator.update( makeFrame(
      calibration, stationary, 50'000'000, kLandmarksA, ids ) );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  auto  measurement = makeFrame( calibration, stationary, 100'000'000,
                                 kLandmarksA, ids );
  auto& imu         = std::get<phad::sensor::RawImuInterval>( measurement.m_imu );
  for ( phad::sensor::ImuMeasurement& sample : imu.m_samples )
  {
    sample.accel_mps2[ 2 ] += 8'000.0;
  }

  const auto recovered = estimator.update( measurement );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  EXPECT_TRUE( recovered.diagnostics.pnp_success );
  EXPECT_EQ( recovered.diagnostics.pnp_inliers, ids.size() );
  ASSERT_TRUE( recovered.estimate.has_value() );
  EXPECT_TRUE( recovered.estimate->T_W_B.matrix().allFinite() );
}

TEST( StereoVoPnpTest, BelowSupportThresholdSkipsPnpButRetainsVisualCadence )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 4, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  options.min_pnp_inliers  = 20;  // > available shared (10)
  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );

  for ( std::size_t index = 0; index < 2; ++index )
  {
    const auto result = estimator.update( makeFrame(
        calibration, poses[ index ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarksA,
        ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  // PnP remains gated by full support while graph admission follows map
  // membership and observation-count rules independently.
  std::vector<Eigen::Vector3d> landmarks_plus = kLandmarksA;
  std::vector<LandmarkId>      ids_plus       = ids;
  const Eigen::Vector3d        probe_point{ 0.5, 0.0, 5.1 };
  const LandmarkId             probe_id = 99;
  landmarks_plus.push_back( probe_point );
  ids_plus.push_back( probe_id );

  const auto introduced = estimator.update( makeFrame(
      calibration, poses[ 2 ], 150'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;
  EXPECT_FALSE( introduced.diagnostics.pnp_success );
  EXPECT_EQ( introduced.diagnostics.num_shared, ids.size() );
  EXPECT_EQ( introduced.diagnostics.num_seeded_landmarks, 1U );
  EXPECT_EQ( introduced.diagnostics.num_landmarks, ids.size() );
  EXPECT_EQ( introduced.diagnostics.num_current_visual_factors,
             ids.size() );
  EXPECT_EQ( introduced.diagnostics.m_vio.m_visual_factors, 30U );

  auto corrupted = makeFrame( calibration, poses[ 3 ], 200'000'000,
                              landmarks_plus, ids_plus );
  offsetLeftPixel( corrupted, probe_id, Eigen::Vector2d( 40.0, 0.0 ) );
  const auto fallback = estimator.update( corrupted );
  ASSERT_EQ( fallback.status, UpdateStatus::kOk ) << fallback.message;
  EXPECT_FALSE( fallback.diagnostics.pnp_success );
  EXPECT_EQ( fallback.diagnostics.pnp_inliers, 0U );
  EXPECT_EQ( fallback.diagnostics.num_shared, ids_plus.size() );
  EXPECT_EQ( fallback.diagnostics.num_landmarks, ids_plus.size() );
  EXPECT_EQ( fallback.diagnostics.num_current_visual_factors,
             ids_plus.size() );
  EXPECT_EQ( fallback.diagnostics.m_vio.m_visual_factors, 42U );
  EXPECT_EQ( fallback.diagnostics.m_vio.m_visual_coast_duration_ns,
             150'000'000 );
  EXPECT_EQ( estimator.observationTimestamps( probe_id ).size(), 2U );
}

TEST( StereoVoPnpTest, FallsBackWhenInliersBelowMin )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 4, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  options.min_pnp_inliers  = 10;
  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );

  for ( std::size_t index = 0; index < 2; ++index )
  {
    const auto result = estimator.update( makeFrame(
        calibration, poses[ index ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarksA,
        ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  std::vector<Eigen::Vector3d> landmarks_plus = kLandmarksA;
  std::vector<LandmarkId>      ids_plus       = ids;
  const Eigen::Vector3d        probe_point{ 0.5, 0.0, 5.1 };
  const LandmarkId             probe_id = 99;
  landmarks_plus.push_back( probe_point );
  ids_plus.push_back( probe_id );

  const auto introduced = estimator.update( makeFrame(
      calibration, poses[ 2 ], 150'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;

  // Corrupt most shared landmarks so RANSAC inliers stay below min.
  auto corrupted = makeFrame( calibration, poses[ 3 ], 200'000'000,
                              landmarks_plus, ids_plus );
  for ( std::size_t index = 0; index < 8; ++index )
  {
    offsetLeftPixel( corrupted, ids[ index ],
                     Eigen::Vector2d( 40.0 + 5.0 * static_cast<double>( index ),
                                      0.0 ) );
  }
  offsetLeftPixel( corrupted, probe_id, Eigen::Vector2d( 40.0, 0.0 ) );

  const auto fallback = estimator.update( corrupted );
  ASSERT_EQ( fallback.status, UpdateStatus::kOk ) << fallback.message;
  EXPECT_FALSE( fallback.diagnostics.pnp_success );
  EXPECT_EQ( fallback.diagnostics.pnp_inliers, 0U );
  // Fallback must not cull: probe reaches 2 observations and enters graph.
  EXPECT_EQ( fallback.diagnostics.num_landmarks, kLandmarksA.size() + 1U );
}

TEST( StereoVoPnpTest, DisabledUsesImuPropagation )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 5, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  options.enable_pnp_init  = false;
  VioEstimator estimator( calibration, phad::test_support::testImuParameters(), options );

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const auto result = estimator.update( makeFrame(
        calibration, poses[ index ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarksA,
        ids ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    EXPECT_FALSE( result.diagnostics.pnp_success );
    EXPECT_EQ( result.diagnostics.pnp_inliers, 0U );
  }
}

TEST( StereoVoPnpTest, RootAndVisualCoastSkipPnp )
{
  const auto calibration = makeCalibration();
  const auto ids_a       = sequentialIds( kLandmarksA.size(), 1 );
  const auto ids_b       = sequentialIds( kLandmarksB.size(), 1000 );

  EstimatorOptions options = defaultPnpOptions();
  VioEstimator     estimator( calibration, phad::test_support::testImuParameters(), options );

  const auto                     poses = translatingPoses( 4, 0.05 );
  std::vector<Eigen::Isometry3d> accepted;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const auto result = estimator.update( makeFrame(
        calibration, poses[ index ],
        static_cast<std::int64_t>( index + 1 ) * 50'000'000, kLandmarksA,
        ids_a ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    if ( index == 0 )
    {
      EXPECT_FALSE( result.diagnostics.pnp_success );
    }
    accepted.push_back( result.estimate->T_W_B );
  }

  const Eigen::Isometry3d& T_prev = accepted[ accepted.size() - 2 ];
  const Eigen::Isometry3d& T_last = accepted.back();
  const Eigen::Isometry3d  expected_anchor =
      T_last * ( T_prev.inverse() * T_last );

  const auto turnover = estimator.update( makeFrame(
      calibration, expected_anchor, 250'000'000, kLandmarksB, ids_b ) );
  ASSERT_EQ( turnover.status, UpdateStatus::kOk ) << turnover.message;
  EXPECT_EQ( turnover.diagnostics.segment_id, 0U );
  EXPECT_EQ( turnover.diagnostics.num_shared, 0U );
  EXPECT_FALSE( turnover.diagnostics.pnp_success );
  EXPECT_EQ( turnover.diagnostics.pnp_inliers, 0U );
  EXPECT_EQ( turnover.diagnostics.m_vio.m_visual_coast_duration_ns,
             50'000'000 );
}

TEST( StereoVoPnpTest, MaskedObsKeepsTrackTimesButMayDropFromGraph )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kLandmarksA.size(), 1 );
  const auto poses       = translatingPoses( 5, 0.05 );

  EstimatorOptions options = defaultPnpOptions();
  VioEstimator     estimator( calibration, phad::test_support::testImuParameters(), options );

  ASSERT_EQ( estimator
                 .update( makeFrame( calibration, poses[ 0 ], 50'000'000,
                                     kLandmarksA, ids ) )
                 .status,
             UpdateStatus::kOk );

  std::vector<Eigen::Vector3d> landmarks_plus = kLandmarksA;
  std::vector<LandmarkId>      ids_plus       = ids;
  const Eigen::Vector3d        probe_point{ 0.5, 0.0, 5.1 };
  const LandmarkId             probe_id = 99;
  landmarks_plus.push_back( probe_point );
  ids_plus.push_back( probe_id );

  const auto introduced = estimator.update( makeFrame(
      calibration, poses[ 1 ], 100'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( introduced.status, UpdateStatus::kOk ) << introduced.message;
  EXPECT_EQ( introduced.diagnostics.num_landmarks, kLandmarksA.size() );

  auto corrupted = makeFrame( calibration, poses[ 2 ], 150'000'000,
                              landmarks_plus, ids_plus );
  offsetLeftPixel( corrupted, probe_id, Eigen::Vector2d( 40.0, 0.0 ) );
  const auto masked = estimator.update( corrupted );
  ASSERT_EQ( masked.status, UpdateStatus::kOk ) << masked.message;
  EXPECT_TRUE( masked.diagnostics.pnp_success );
  // Masked current obs → still only one window sighting → out of graph.
  EXPECT_EQ( masked.diagnostics.num_landmarks, kLandmarksA.size() );

  const auto stamps_masked = estimator.observationTimestamps( probe_id );
  EXPECT_EQ( stamps_masked.size(), 2U );
  EXPECT_TRUE( hasTimestamp( stamps_masked, 100'000'000 ) );
  EXPECT_TRUE( hasTimestamp( stamps_masked, 150'000'000 ) );

  const auto recovered = estimator.update( makeFrame(
      calibration, poses[ 3 ], 200'000'000, landmarks_plus, ids_plus ) );
  ASSERT_EQ( recovered.status, UpdateStatus::kOk ) << recovered.message;
  EXPECT_TRUE( recovered.diagnostics.pnp_success );
  // Prior (unmasked) + recovered → count >= 2 → back in the graph.
  EXPECT_EQ( recovered.diagnostics.num_landmarks, kLandmarksA.size() + 1U );

  const auto stamps_recovered = estimator.observationTimestamps( probe_id );
  EXPECT_EQ( stamps_recovered.size(), 3U );
  EXPECT_TRUE( hasTimestamp( stamps_recovered, 200'000'000 ) );
}
