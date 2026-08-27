#include "phad/estimator/stereo_vo_estimator.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/sensor/imu_parameters.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::KeyframeMeasurement;
  using phad::estimator::StereoObservation;
  using phad::estimator::StereoVoEstimator;
  using phad::estimator::UpdateStatus;
  using phad::sensor::RigidTransform;

  [[nodiscard]] phad::sensor::ImuParameters makeImuParameters()
  {
    return phad::sensor::ImuParameters::create(
               200.0, 0.002, 0.00016968, 0.003, 1.9393e-05 )
        .value();
  }

  RectifiedStereoCalibration makeCalibration(
      const Eigen::Isometry3d& T_B_left = Eigen::Isometry3d::Identity() )
  {
    Eigen::Matrix4d matrix = T_B_left.matrix();
    auto            rigid  = RigidTransform::create( std::move( matrix ) ).value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480, std::move( rigid ) )
        .value();
  }

  [[nodiscard]] StereoObservation projectLandmark(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d&          T_W_B,
      phad::estimator::LandmarkId       id,
      const Eigen::Vector3d&            point_W )
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

  struct Scene
  {
    RectifiedStereoCalibration     calibration;
    std::vector<Eigen::Isometry3d> poses_W_B;
    std::vector<Eigen::Vector3d>   landmarks_W;
  };

  Scene makeTranslatingScene( int frame_count )
  {
    std::vector<Eigen::Isometry3d> poses_W_B;
    poses_W_B.reserve( static_cast<std::size_t>( frame_count ) );
    for ( int index = 0; index < frame_count; ++index )
    {
      Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
      T_W_B.translation() =
          Eigen::Vector3d( 0.05 * static_cast<double>( index ), 0.0, 0.0 );
      poses_W_B.push_back( T_W_B );
    }
    return Scene{
        makeCalibration(),
        std::move( poses_W_B ),
        {
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
        },
    };
  }

  KeyframeMeasurement makeMeasurement( const Scene& scene, int frame_index )
  {
    KeyframeMeasurement measurement;
    measurement.timestamp = phad::common::Timestamp{
        static_cast<std::int64_t>( frame_index + 1 ) * 50'000'000 };
    for ( std::size_t landmark_index = 0;
          landmark_index < scene.landmarks_W.size(); ++landmark_index )
    {
      measurement.observations.push_back( projectLandmark(
          scene.calibration, scene.poses_W_B[ static_cast<std::size_t>( frame_index ) ],
          static_cast<phad::estimator::LandmarkId>( landmark_index + 1 ),
          scene.landmarks_W[ landmark_index ] ) );
    }
    return measurement;
  }

  void expectExactlyEqual( const phad::estimator::VioUpdateResult& lhs,
                           const phad::estimator::VioUpdateResult& rhs )
  {
    EXPECT_EQ( lhs.status, rhs.status );
    EXPECT_EQ( lhs.message, rhs.message );
    ASSERT_EQ( lhs.estimate.has_value(), rhs.estimate.has_value() );
    if ( lhs.estimate.has_value() )
    {
      EXPECT_EQ( lhs.estimate->timestamp, rhs.estimate->timestamp );
      EXPECT_TRUE( ( lhs.estimate->T_W_B.matrix().array() ==
                     rhs.estimate->T_W_B.matrix().array() )
                       .all() );
    }

    const auto& a = lhs.diagnostics;
    const auto& b = rhs.diagnostics;
    EXPECT_EQ( a.num_observations, b.num_observations );
    EXPECT_EQ( a.num_landmarks, b.num_landmarks );
    EXPECT_EQ( a.num_shared, b.num_shared );
    EXPECT_EQ( a.num_disparity, b.num_disparity );
    EXPECT_EQ( a.num_cheirality, b.num_cheirality );
    EXPECT_EQ( a.lm_iterations, b.lm_iterations );
    EXPECT_EQ( a.window_size, b.window_size );
    EXPECT_EQ( a.segment_id, b.segment_id );
    EXPECT_EQ( a.prior_key, b.prior_key );
    EXPECT_DOUBLE_EQ( a.reproj_rms_before_px, b.reproj_rms_before_px );
    EXPECT_DOUBLE_EQ( a.reproj_rms_after_px, b.reproj_rms_after_px );
    EXPECT_DOUBLE_EQ( a.max_window_pose_shift_m,
                      b.max_window_pose_shift_m );
    EXPECT_EQ( a.low_connectivity, b.low_connectivity );
    EXPECT_EQ( a.pnp_success, b.pnp_success );
    EXPECT_EQ( a.pnp_inliers, b.pnp_inliers );
    EXPECT_EQ( a.outliers_culled, b.outliers_culled );
    EXPECT_EQ( a.outliers_culled_unique, b.outliers_culled_unique );
    EXPECT_DOUBLE_EQ( a.reproj_rms_after_cull_px,
                      b.reproj_rms_after_cull_px );
    EXPECT_EQ( a.outlier_reopt, b.outlier_reopt );
    EXPECT_EQ( a.outlier_reopt_failed, b.outlier_reopt_failed );
    EXPECT_EQ( a.outlier_reopt_rounds, b.outlier_reopt_rounds );
    EXPECT_EQ( a.culled_landmark_ids, b.culled_landmark_ids );
    EXPECT_EQ( a.probe_rejected_block_n, b.probe_rejected_block_n );
    EXPECT_EQ( a.probe_new_lm_n, b.probe_new_lm_n );
    EXPECT_EQ( a.probe_shift_top, b.probe_shift_top );
    EXPECT_DOUBLE_EQ( a.probe_res_mean_px, b.probe_res_mean_px );
    EXPECT_DOUBLE_EQ( a.probe_res_max_px, b.probe_res_max_px );
    EXPECT_EQ( a.probe_res_max_id, b.probe_res_max_id );
    EXPECT_EQ( a.probe_detail_valid, b.probe_detail_valid );
  }

}  // namespace

TEST( StereoVoEstimator, RecoversTranslationAndLowersReprojRms )
{
  const Scene      scene = makeTranslatingScene( 8 );
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size                     = 5;
  options.min_shared_landmarks            = 3;
  options.use_constant_velocity_init      = true;

  StereoVoEstimator estimator( scene.calibration, std::nullopt, options );
  double            last_after_rms = 0.0;
  for ( int frame_index = 0;
        frame_index < static_cast<int>( scene.poses_W_B.size() );
        ++frame_index )
  {
    const auto result =
        estimator.update( makeMeasurement( scene, frame_index ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    ASSERT_TRUE( result.estimate.has_value() );
    if ( frame_index >= 1 )
    {
      EXPECT_LT( result.diagnostics.reproj_rms_after_px,
                 result.diagnostics.reproj_rms_before_px + 1e-9 );
    }
    last_after_rms = result.diagnostics.reproj_rms_after_px;
    const Eigen::Vector3d gt =
        scene.poses_W_B[ static_cast<std::size_t>( frame_index ) ]
            .translation();
    const Eigen::Vector3d est = result.estimate->T_W_B.translation();
    EXPECT_NEAR( est.x(), gt.x(), 5e-3 );
    EXPECT_NEAR( est.y(), gt.y(), 5e-3 );
    EXPECT_NEAR( est.z(), gt.z(), 5e-3 );
  }
  EXPECT_LT( last_after_rms, 0.5 );
}

TEST( StereoVoEstimatorGyroOptions, ShadowRequiresImuParameters )
{
  EstimatorOptions options;
  options.gyro_mode = phad::estimator::GyroMode::kShadow;

  EXPECT_THROW(
      StereoVoEstimator( makeCalibration(), std::nullopt, options ),
      std::invalid_argument );
}

TEST( StereoVoEstimatorGyroOptions, RejectsNonPositiveAlignmentWindow )
{
  EstimatorOptions options;
  options.gyro_align_window_s = 0.0;

  EXPECT_THROW(
      StereoVoEstimator( makeCalibration(), std::nullopt, options ),
      std::invalid_argument );
}

TEST( StereoVoEstimatorGyroOptions, RejectsNonFiniteAlignmentWindow )
{
  for ( const double value :
        { std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::quiet_NaN() } )
  {
    EstimatorOptions options;
    options.gyro_align_window_s = value;
    EXPECT_THROW(
        StereoVoEstimator( makeCalibration(), std::nullopt, options ),
        std::invalid_argument );
  }
}

TEST( StereoVoEstimatorGyroOptions, FusedRequiresImuParameters )
{
  EstimatorOptions options;
  options.gyro_mode = phad::estimator::GyroMode::kFused;

  EXPECT_THROW(
      StereoVoEstimator( makeCalibration(), std::nullopt, options ),
      std::invalid_argument );
}

TEST( StereoVoEstimatorGyroOptions, ShadowAcceptsImuParameters )
{
  EstimatorOptions options;
  options.gyro_mode = phad::estimator::GyroMode::kShadow;

  EXPECT_NO_THROW(
      StereoVoEstimator( makeCalibration(), makeImuParameters(), options ) );
}

TEST( StereoVoEstimatorGyroOff, IgnoresMalformedImuSegmentExactly )
{
  const Scene      scene = makeTranslatingScene( 2 );
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;
  options.min_shared_landmarks            = 3;
  ASSERT_EQ( options.gyro_mode, phad::estimator::GyroMode::kOff );

  StereoVoEstimator                         baseline( scene.calibration, std::nullopt, options );
  StereoVoEstimator                         malformed( scene.calibration, std::nullopt, options );
  std::vector<phad::sensor::ImuMeasurement> bad_samples{
      { .timestamp  = phad::common::Timestamp{ 40'000'000 },
        .gyro_radps = { std::numeric_limits<double>::quiet_NaN(), 0.0,
                        0.0 } },
  };

  for ( int frame_index = 0; frame_index < 2; ++frame_index )
  {
    const KeyframeMeasurement plain = makeMeasurement( scene, frame_index );
    KeyframeMeasurement       bad   = plain;
    bad.imu                         = phad::estimator::ImuSegmentView{
                                .t_prev  = phad::common::Timestamp{ 999'000'000 },
                                .samples = bad_samples,
                                .gap     = false,
    };

    expectExactlyEqual( baseline.update( plain ), malformed.update( bad ) );
  }
}

TEST( StereoVoEstimator, PriorStaysOnOldestAndCapsWindow )
{
  const Scene      scene = makeTranslatingScene( 6 );
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size                     = 3;
  options.min_shared_landmarks            = 3;

  StereoVoEstimator estimator( scene.calibration, std::nullopt, options );
  std::uint64_t     expected_prior = 0;
  for ( int frame_index = 0; frame_index < 6; ++frame_index )
  {
    const auto result =
        estimator.update( makeMeasurement( scene, frame_index ) );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    EXPECT_LE( result.diagnostics.window_size, 3U );
    if ( frame_index < 3 )
    {
      expected_prior = 0;
      EXPECT_EQ( result.diagnostics.window_size,
                 static_cast<std::uint32_t>( frame_index + 1 ) );
    }
    else
    {
      expected_prior = static_cast<std::uint64_t>( frame_index - 2 );
      EXPECT_EQ( result.diagnostics.window_size, 3U );
    }
    EXPECT_EQ( result.diagnostics.prior_key, expected_prior );
    if ( frame_index == 0 )
    {
      EXPECT_EQ( result.diagnostics.num_landmarks, 0U );
    }
    else
    {
      EXPECT_EQ( result.diagnostics.num_landmarks, scene.landmarks_W.size() );
    }
  }
}

TEST( StereoVoEstimator, SingleObservationLandmarksStayOutOfGraph )
{
  const Scene      scene = makeTranslatingScene( 2 );
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size                     = 10;
  options.min_landmark_observations       = 2;
  options.min_shared_landmarks            = 1;

  StereoVoEstimator estimator( scene.calibration, std::nullopt, options );
  const auto        first = estimator.update( makeMeasurement( scene, 0 ) );
  ASSERT_EQ( first.status, UpdateStatus::kOk ) << first.message;
  EXPECT_EQ( first.diagnostics.num_landmarks, 0U );

  const auto second = estimator.update( makeMeasurement( scene, 1 ) );
  ASSERT_EQ( second.status, UpdateStatus::kOk ) << second.message;
  EXPECT_EQ( second.diagnostics.num_landmarks, scene.landmarks_W.size() );
}

TEST( StereoVoEstimator, HuberReducesOutlierPosePull )
{
  const Scene scene = makeTranslatingScene( 5 );

  auto run_with_outlier = [ & ]( double huber_k_px ) {
    EstimatorOptions options;
    options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.window_size                     = 5;
    options.min_shared_landmarks            = 3;
    options.huber_k_px                      = huber_k_px;
    StereoVoEstimator                           estimator( scene.calibration, std::nullopt, options );
    std::optional<phad::estimator::VioEstimate> last;
    for ( int frame_index = 0; frame_index < 5; ++frame_index )
    {
      KeyframeMeasurement measurement = makeMeasurement( scene, frame_index );
      if ( frame_index == 4 )
      {
        measurement.observations[ 0 ].left_pixel.x() += 40.0;
      }
      const auto result = estimator.update( measurement );
      EXPECT_EQ( result.status, UpdateStatus::kOk ) << result.message;
      last = result.estimate;
    }
    return *last;
  };

  const auto            with_huber    = run_with_outlier( 3.0 );
  const auto            without_huber = run_with_outlier( 0.0 );
  const Eigen::Vector3d gt            = scene.poses_W_B.back().translation();
  const double          err_huber     = ( with_huber.T_W_B.translation() - gt ).norm();
  const double          err_gauss =
      ( without_huber.T_W_B.translation() - gt ).norm();
  EXPECT_LT( err_huber, err_gauss );
}
