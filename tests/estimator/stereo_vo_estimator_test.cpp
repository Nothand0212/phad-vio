#include "phad/estimator/stereo_vo_estimator.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
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

}  // namespace

TEST( StereoVoEstimator, RecoversTranslationAndLowersReprojRms )
{
  const Scene      scene = makeTranslatingScene( 8 );
  EstimatorOptions options;
  options.window_size                = 5;
  options.min_shared_landmarks       = 3;
  options.use_constant_velocity_init = true;

  StereoVoEstimator estimator( scene.calibration, options );
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

TEST( StereoVoEstimator, PriorStaysOnOldestAndCapsWindow )
{
  const Scene      scene = makeTranslatingScene( 6 );
  EstimatorOptions options;
  options.window_size          = 3;
  options.min_shared_landmarks = 3;

  StereoVoEstimator estimator( scene.calibration, options );
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
  options.window_size               = 10;
  options.min_landmark_observations = 2;
  options.min_shared_landmarks      = 1;

  StereoVoEstimator estimator( scene.calibration, options );
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
    options.window_size          = 5;
    options.min_shared_landmarks = 3;
    options.huber_k_px           = huber_k_px;
    StereoVoEstimator                           estimator( scene.calibration, options );
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
