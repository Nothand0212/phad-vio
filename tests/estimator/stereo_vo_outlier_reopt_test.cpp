#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/estimator/types.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::KeyframeMeasurement;
  using phad::estimator::LandmarkId;
  using phad::estimator::StereoObservation;
  using phad::estimator::StereoVoEstimator;
  using phad::estimator::UpdateDiagnostics;
  using phad::estimator::UpdateStatus;
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

  KeyframeMeasurement makeFrame(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d& T_W_B, std::int64_t timestamp_ns,
      const std::vector<Eigen::Vector3d>& landmarks_W,
      const std::vector<LandmarkId>&      ids )
  {
    KeyframeMeasurement measurement;
    measurement.timestamp = phad::common::Timestamp{ timestamp_ns };
    for ( std::size_t index = 0; index < landmarks_W.size(); ++index )
    {
      measurement.observations.push_back( projectLandmark(
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

  void offsetLeftPixel( KeyframeMeasurement& measurement, LandmarkId id,
                        double delta_u_px )
  {
    for ( StereoObservation& observation : measurement.observations )
    {
      if ( observation.id == id )
      {
        observation.left_pixel.x() += delta_u_px;
        return;
      }
    }
    FAIL() << "landmark id " << id << " not found in measurement";
  }

  [[nodiscard]] std::vector<Eigen::Vector3d> makeDenseLandmarks()
  {
    std::vector<Eigen::Vector3d> landmarks;
    landmarks.reserve( 48 );
    for ( int iz = 0; iz < 3 && landmarks.size() < 48; ++iz )
    {
      for ( int iy = -2; iy <= 2 && landmarks.size() < 48; ++iy )
      {
        for ( int ix = -2; ix <= 2 && landmarks.size() < 48; ++ix )
        {
          landmarks.emplace_back(
              0.15 * static_cast<double>( ix ),
              0.12 * static_cast<double>( iy ),
              4.5 + 0.4 * static_cast<double>( iz ) );
        }
      }
    }
    return landmarks;
  }

  const std::vector<Eigen::Vector3d> kDenseLandmarks = makeDenseLandmarks();

  [[nodiscard]] std::vector<Eigen::Vector3d> makeSparseLandmarks()
  {
    return {
        Eigen::Vector3d( 0.15, 0.12, 4.5 ),
        Eigen::Vector3d( -0.15, 0.12, 4.5 ),
        Eigen::Vector3d( 0.15, -0.12, 4.9 ),
        Eigen::Vector3d( -0.15, -0.12, 4.9 ),
    };
  }

  EstimatorOptions defaultReoptOptions()
  {
    EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.window_size           = 8;
    options.min_shared_landmarks  = 3;
    options.min_seed_observations = 10;
    options.enable_pnp_init       = false;
    options.enable_outlier_cull   = true;
    options.enable_outlier_reopt  = true;
    options.outlier_avg_reproj_px = 3.0;
    // Reopt fixtures poison several ids across frames; allow rebirth so a
    // single frame can still mean-cull >= 4 (Slice ④ pseudo-permanent).
    options.block_culled_rebirth = false;
    return options;
  }

  [[nodiscard]] double poisonDeltaPx( std::size_t frame_index )
  {
    return ( frame_index % 2 == 0 ) ? 80.0 : -80.0;
  }

  // Per-id phase flip: identical deltas on several ids are absorbed as a
  // shared left bias (~1 px post-fit RMS) and never trip mean-cull.
  [[nodiscard]] double poisonDeltaPxFor( LandmarkId  id,
                                         std::size_t frame_index )
  {
    const double base = poisonDeltaPx( frame_index );
    return ( ( id % 2U ) == 0U ) ? -base : base;
  }

  void poisonIds( KeyframeMeasurement&           measurement,
                  const std::vector<LandmarkId>& ids, std::size_t frame_index )
  {
    for ( const LandmarkId id : ids )
    {
      offsetLeftPixel( measurement, id, poisonDeltaPxFor( id, frame_index ) );
    }
  }

}  // namespace

TEST( StereoVoOutlierReoptTest, DefaultsMaxReoptsThreeAndRoundsZero )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  EXPECT_EQ( options.max_outlier_reopts, 3 );
  UpdateDiagnostics d;
  EXPECT_EQ( d.outlier_reopt_rounds, 0U );
}

TEST( StereoVoOutlierReoptTest, RejectsNegativeMaxOutlierReopts )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.max_outlier_reopts = -1;
  EXPECT_THROW( StereoVoEstimator( makeCalibration(), options ),
                std::invalid_argument );
}

TEST( StereoVoOutlierReoptTest, ReoptsWhenAtLeastFourCulled )
{
  const auto                    calibration = makeCalibration();
  const auto                    ids         = sequentialIds( kDenseLandmarks.size(), 1 );
  const auto                    poses       = translatingPoses( 12, 0.05 );
  const std::vector<LandmarkId> poison_ids{ 1, 2, 3, 4 };

  // Huber down-weights several ±80 tracks so mean-cull never fires; disable it
  // so four poisoned ids stay above outlier_avg_reproj_px after LM₁.
  EstimatorOptions options_on      = defaultReoptOptions();
  options_on.huber_k_px            = 0.0;
  EstimatorOptions options_off     = options_on;
  options_off.enable_outlier_reopt = false;

  StereoVoEstimator est_on( calibration, options_on );
  StereoVoEstimator est_off( calibration, options_off );

  bool saw = false;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kDenseLandmarks, ids );
    if ( index >= 1 )
    {
      poisonIds( measurement, poison_ids, index );
    }
    const auto r_on  = est_on.update( measurement );
    const auto r_off = est_off.update( measurement );
    ASSERT_EQ( r_on.status, UpdateStatus::kOk ) << r_on.message;
    ASSERT_EQ( r_off.status, UpdateStatus::kOk ) << r_off.message;

    if ( r_on.diagnostics.outliers_culled >= 4U )
    {
      EXPECT_TRUE( r_on.diagnostics.outlier_reopt );
      EXPECT_FALSE( r_on.diagnostics.outlier_reopt_failed );
      EXPECT_EQ( r_on.diagnostics.outlier_reopt_rounds, 1U );
      EXPECT_GT( r_on.diagnostics.lm_iterations,
                 r_off.diagnostics.lm_iterations );
      EXPECT_FALSE( r_off.diagnostics.outlier_reopt );
      EXPECT_EQ( r_off.diagnostics.outlier_reopt_rounds, 0U );
      saw = true;
      break;
    }
  }
  EXPECT_TRUE( saw );
}

TEST( StereoVoOutlierReoptTest, MaxZeroSkipsReopt )
{
  const auto                    calibration = makeCalibration();
  const auto                    ids         = sequentialIds( kDenseLandmarks.size(), 1 );
  const auto                    poses       = translatingPoses( 12, 0.05 );
  const std::vector<LandmarkId> poison_ids{ 1, 2, 3, 4 };

  EstimatorOptions options    = defaultReoptOptions();
  options.huber_k_px          = 0.0;
  options.max_outlier_reopts  = 0;
  options.enable_outlier_reopt = true;
  StereoVoEstimator estimator( calibration, options );

  bool saw = false;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kDenseLandmarks, ids );
    if ( index >= 1 )
    {
      poisonIds( measurement, poison_ids, index );
    }
    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    if ( result.diagnostics.outliers_culled >= 4U )
    {
      EXPECT_EQ( result.diagnostics.outlier_reopt_rounds, 0U );
      EXPECT_FALSE( result.diagnostics.outlier_reopt );
      EXPECT_FALSE( result.diagnostics.outlier_reopt_failed );
      saw = true;
      break;
    }
  }
  EXPECT_TRUE( saw );
}

TEST( StereoVoOutlierReoptTest, MaxOneCapsReoptRounds )
{
  const auto                    calibration = makeCalibration();
  const auto                    ids         = sequentialIds( kDenseLandmarks.size(), 1 );
  const auto                    poses       = translatingPoses( 12, 0.05 );
  const std::vector<LandmarkId> poison_ids{ 1, 2, 3, 4 };

  EstimatorOptions options   = defaultReoptOptions();
  options.huber_k_px         = 0.0;
  options.max_outlier_reopts = 1;
  StereoVoEstimator estimator( calibration, options );

  bool saw = false;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kDenseLandmarks, ids );
    if ( index >= 1 )
    {
      poisonIds( measurement, poison_ids, index );
    }
    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    if ( result.diagnostics.outliers_culled >= 4U )
    {
      EXPECT_TRUE( result.diagnostics.outlier_reopt );
      EXPECT_EQ( result.diagnostics.outlier_reopt_rounds, 1U );
      EXPECT_FALSE( result.diagnostics.outlier_reopt_failed );
      saw = true;
      break;
    }
  }
  EXPECT_TRUE( saw );
}

TEST( StereoVoOutlierReoptTest, SkipsReoptWhenCulledOneToThree )
{
  const auto       calibration = makeCalibration();
  const auto       ids         = sequentialIds( kDenseLandmarks.size(), 1 );
  const auto       poses       = translatingPoses( 12, 0.05 );
  const LandmarkId poison_id   = 1;

  EstimatorOptions options_on      = defaultReoptOptions();
  EstimatorOptions options_off     = defaultReoptOptions();
  options_off.enable_outlier_reopt = false;

  StereoVoEstimator est_on( calibration, options_on );
  StereoVoEstimator est_off( calibration, options_off );

  bool saw = false;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kDenseLandmarks, ids );
    if ( index >= 1 )
    {
      offsetLeftPixel( measurement, poison_id, poisonDeltaPx( index ) );
    }
    const auto r_on  = est_on.update( measurement );
    const auto r_off = est_off.update( measurement );
    ASSERT_EQ( r_on.status, UpdateStatus::kOk ) << r_on.message;
    ASSERT_EQ( r_off.status, UpdateStatus::kOk ) << r_off.message;

    if ( r_on.diagnostics.outliers_culled >= 1U )
    {
      EXPECT_GE( r_on.diagnostics.outliers_culled, 1U );
      EXPECT_LE( r_on.diagnostics.outliers_culled, 3U );
      EXPECT_FALSE( r_on.diagnostics.outlier_reopt );
      EXPECT_FALSE( r_on.diagnostics.outlier_reopt_failed );
      EXPECT_EQ( r_on.diagnostics.lm_iterations,
                 r_off.diagnostics.lm_iterations );
      saw = true;
      break;
    }
  }
  EXPECT_TRUE( saw );
}

TEST( StereoVoOutlierReoptTest, DisabledSkipsReopt )
{
  const auto                    calibration = makeCalibration();
  const auto                    ids         = sequentialIds( kDenseLandmarks.size(), 1 );
  const auto                    poses       = translatingPoses( 12, 0.05 );
  const std::vector<LandmarkId> poison_ids{ 1, 2, 3, 4 };

  EstimatorOptions options     = defaultReoptOptions();
  options.enable_outlier_reopt = false;
  options.huber_k_px           = 0.0;
  StereoVoEstimator estimator( calibration, options );

  bool saw = false;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kDenseLandmarks, ids );
    if ( index >= 1 )
    {
      poisonIds( measurement, poison_ids, index );
    }
    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    if ( result.diagnostics.outliers_culled >= 4U )
    {
      EXPECT_FALSE( result.diagnostics.outlier_reopt );
      EXPECT_FALSE( result.diagnostics.outlier_reopt_failed );
      saw = true;
      break;
    }
  }
  EXPECT_TRUE( saw );
}

TEST( StereoVoOutlierReoptTest, NoCullSkipsReopt )
{
  const auto calibration = makeCalibration();
  const auto ids         = sequentialIds( kDenseLandmarks.size(), 1 );
  const auto poses       = translatingPoses( 8, 0.05 );

  EstimatorOptions options_on      = defaultReoptOptions();
  EstimatorOptions options_off     = defaultReoptOptions();
  options_off.enable_outlier_reopt = false;

  StereoVoEstimator est_on( calibration, options_on );
  StereoVoEstimator est_off( calibration, options_off );

  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    const auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kDenseLandmarks, ids );
    const auto r_on  = est_on.update( measurement );
    const auto r_off = est_off.update( measurement );
    ASSERT_EQ( r_on.status, UpdateStatus::kOk ) << r_on.message;
    ASSERT_EQ( r_off.status, UpdateStatus::kOk ) << r_off.message;
    EXPECT_EQ( r_on.diagnostics.outliers_culled, 0U );
    EXPECT_FALSE( r_on.diagnostics.outlier_reopt );
    EXPECT_FALSE( r_on.diagnostics.outlier_reopt_failed );
    EXPECT_EQ( r_on.diagnostics.lm_iterations,
               r_off.diagnostics.lm_iterations );
  }
}

TEST( StereoVoOutlierReoptTest, CheiralityOnlySkipsReopt )
{
  const auto       calibration = makeCalibration();
  EstimatorOptions options     = defaultReoptOptions();
  options.min_shared_landmarks = 2;
  options.huber_k_px           = 0.0;

  const std::vector<Eigen::Vector3d> far_landmarks{
      { 0.4, 0.1, 5.0 },
      { -0.3, 0.2, 4.5 },
      { 0.1, -0.25, 6.0 },
      { 0.6, -0.1, 5.5 },
      { -0.5, -0.2, 4.8 },
      { 0.0, 0.3, 5.2 },
      { 0.35, -0.05, 5.3 },
      { -0.15, 0.25, 4.6 },
      { 0.5, 0.05, 5.8 },
  };
  const Eigen::Vector3d near_landmark{ 0.15, 0.0, 1.0 };
  const auto            far_ids = sequentialIds( far_landmarks.size(), 1 );
  const LandmarkId      near_id = 99;

  StereoVoEstimator estimator( calibration, options );
  StereoObservation near_at_first{};
  bool              saw_cheirality = false;

  for ( int index = 0; index < 4; ++index )
  {
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    const double      z     = ( index == 0 ) ? 0.0 : 2.5;
    T_W_B.translation()     = Eigen::Vector3d( 0.0, 0.0, z );

    KeyframeMeasurement measurement;
    measurement.timestamp = phad::common::Timestamp{
        static_cast<std::int64_t>( index + 1 ) * 50'000'000 };

    for ( std::size_t landmark_index = 0; landmark_index < far_landmarks.size();
          ++landmark_index )
    {
      measurement.observations.push_back( projectLandmark(
          calibration, T_W_B, far_ids[ landmark_index ],
          far_landmarks[ landmark_index ] ) );
    }

    if ( index == 0 )
    {
      near_at_first =
          projectLandmark( calibration, T_W_B, near_id, near_landmark );
      measurement.observations.push_back( near_at_first );
    }
    else if ( index <= 2 )
    {
      measurement.observations.push_back( near_at_first );
    }

    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    EXPECT_EQ( result.diagnostics.outliers_culled, 0U );
    EXPECT_FALSE( result.diagnostics.outlier_reopt );
    EXPECT_FALSE( result.diagnostics.outlier_reopt_failed );
    if ( result.diagnostics.num_cheirality > 0 )
    {
      saw_cheirality = true;
    }
  }
  EXPECT_TRUE( saw_cheirality );
}

TEST( StereoVoOutlierReoptTest, Lm2FailureFallsBackToLm1Cull )
{
  // Sparse 4-landmark scene: lock geometry on clean frames, then poison every
  // id so mean-cull deletes the whole map → LM₂ graph is Prior-only / singular.
  const auto            calibration = makeCalibration();
  const auto            landmarks   = makeSparseLandmarks();
  const auto            ids         = sequentialIds( landmarks.size(), 1 );
  const auto            poses       = translatingPoses( 20, 0.05 );
  constexpr std::size_t kPoisonFrom = 4;

  EstimatorOptions options_on          = defaultReoptOptions();
  options_on.window_size               = 6;
  options_on.min_seed_observations     = 4;
  options_on.min_shared_landmarks      = 2;
  options_on.huber_k_px                = 0.0;
  options_on.min_landmark_observations = 2;
  options_on.outlier_avg_reproj_px     = 0.5;

  EstimatorOptions options_off     = options_on;
  options_off.enable_outlier_reopt = false;

  StereoVoEstimator est_on( calibration, options_on );
  StereoVoEstimator est_off( calibration, options_off );

  bool          saw        = false;
  std::uint32_t max_culled = 0;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, landmarks, ids );
    if ( index >= kPoisonFrom )
    {
      // Large per-id alternating offsets so post-fit mean stays above threshold
      // even when the sparse map tries to absorb the bias.
      for ( const LandmarkId id : ids )
      {
        const double base =
            ( index % 2U == 0U ) ? 200.0 : -200.0;
        const double delta = ( ( id % 2U ) == 0U ) ? -base : base;
        offsetLeftPixel( measurement, id, delta );
      }
    }
    const auto r_on  = est_on.update( measurement );
    const auto r_off = est_off.update( measurement );
    ASSERT_EQ( r_on.status, UpdateStatus::kOk )
        << "frame " << index << ": " << r_on.message;
    ASSERT_EQ( r_off.status, UpdateStatus::kOk )
        << "frame " << index << ": " << r_off.message;
    ASSERT_TRUE( r_on.estimate.has_value() );
    ASSERT_TRUE( r_off.estimate.has_value() );
    max_culled = std::max( max_culled, r_on.diagnostics.outliers_culled );

    if ( r_on.diagnostics.outliers_culled >= 4U )
    {
      EXPECT_FALSE( r_on.diagnostics.outlier_reopt );
      EXPECT_EQ( r_on.diagnostics.outlier_reopt_rounds, 0U );
      EXPECT_TRUE( r_on.diagnostics.outlier_reopt_failed );
      EXPECT_GE( r_on.diagnostics.outliers_culled, 4U );
      EXPECT_TRUE( r_on.estimate->T_W_B.matrix().isApprox(
          r_off.estimate->T_W_B.matrix(), 1e-12 ) );
      EXPECT_FALSE( r_off.diagnostics.outlier_reopt );
      EXPECT_FALSE( r_off.diagnostics.outlier_reopt_failed );
      saw = true;
      break;
    }
  }
  EXPECT_TRUE( saw ) << "max_culled=" << max_culled;
}

TEST( StereoVoOutlierReoptTest, CullsAfterReoptLm )
{
  // ④e allows mean-cull after a successful reopt LM (replaces NoSecondCull).
  const auto                    calibration = makeCalibration();
  const auto                    ids         = sequentialIds( kDenseLandmarks.size(), 1 );
  const auto                    poses       = translatingPoses( 12, 0.05 );
  const std::vector<LandmarkId> poison_ids{ 1, 2, 3, 4 };

  EstimatorOptions options_on      = defaultReoptOptions();
  options_on.huber_k_px            = 0.0;
  options_on.max_outlier_reopts    = 1;
  EstimatorOptions options_off     = options_on;
  options_off.enable_outlier_reopt = false;

  StereoVoEstimator est_on( calibration, options_on );
  StereoVoEstimator est_off( calibration, options_off );

  bool saw = false;
  for ( std::size_t index = 0; index < poses.size(); ++index )
  {
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( index + 1 ) * 50'000'000;
    auto measurement =
        makeFrame( calibration, poses[ index ], ts_ns, kDenseLandmarks, ids );
    if ( index >= 1 )
    {
      poisonIds( measurement, poison_ids, index );
    }
    const auto r_on  = est_on.update( measurement );
    const auto r_off = est_off.update( measurement );
    ASSERT_EQ( r_on.status, UpdateStatus::kOk ) << r_on.message;
    ASSERT_EQ( r_off.status, UpdateStatus::kOk ) << r_off.message;
    if ( r_on.diagnostics.outliers_culled >= 4U ||
         r_off.diagnostics.outliers_culled >= 4U )
    {
      EXPECT_EQ( r_on.status, UpdateStatus::kOk );
      EXPECT_TRUE( r_on.diagnostics.outlier_reopt );
      EXPECT_EQ( r_on.diagnostics.outlier_reopt_rounds, 1U );
      EXPECT_FALSE( r_on.diagnostics.outlier_reopt_failed );
      // Reopt-then-cull may cull at least as many as reopt-off (second wave
      // often 0 on this fixture — rounds==1 + kOk is the hard floor).
      EXPECT_GE( r_on.diagnostics.outliers_culled,
                 r_off.diagnostics.outliers_culled );
      saw = true;
      break;
    }
  }
  EXPECT_TRUE( saw );
}
