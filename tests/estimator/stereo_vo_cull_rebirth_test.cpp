#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
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

  const std::vector<Eigen::Vector3d> kLandmarks = makeDenseLandmarks();

  EstimatorOptions defaultCullOptions()
  {
    EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.window_size           = 8;
    options.min_shared_landmarks  = 3;
    options.min_seed_observations = 10;
    options.enable_pnp_init       = false;
    options.enable_outlier_cull   = true;
    options.outlier_avg_reproj_px = 3.0;
    return options;
  }

  [[nodiscard]] double poisonDeltaPx( std::size_t frame_index )
  {
    return ( frame_index % 2 == 0 ) ? 80.0 : -80.0;
  }

  [[nodiscard]] bool containsId( const std::vector<LandmarkId>& ids,
                                 LandmarkId                     id )
  {
    return std::find( ids.begin(), ids.end(), id ) != ids.end();
  }

}  // namespace

TEST( StereoVoCullRebirthTest, DefaultsBlockRebirthAndEmptyList )
{
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  EXPECT_TRUE( options.block_culled_rebirth );
  UpdateDiagnostics d;
  EXPECT_TRUE( d.culled_landmark_ids.empty() );
}

TEST( StereoVoCullRebirthTest, CulledIdNotRebackprojectedWhenBlocked )
{
  const auto       calibration = makeCalibration();
  const auto       ids         = sequentialIds( kLandmarks.size(), 1 );
  const auto       poses       = translatingPoses( 14, 0.05 );
  const LandmarkId poison_id   = 1;

  auto run = [ & ]( bool block_rebirth ) -> std::uint32_t {
    EstimatorOptions options      = defaultCullOptions();
    options.block_culled_rebirth  = block_rebirth;
    StereoVoEstimator estimator( calibration, options );

    int cull_frame = -1;
    for ( std::size_t index = 0; index < poses.size(); ++index )
    {
      const std::int64_t ts_ns =
          static_cast<std::int64_t>( index + 1 ) * 50'000'000;
      auto measurement =
          makeFrame( calibration, poses[ index ], ts_ns, kLandmarks, ids );
      if ( index >= 1 )
      {
        offsetLeftPixel( measurement, poison_id, poisonDeltaPx( index ) );
      }
      const auto result = estimator.update( measurement );
      EXPECT_EQ( result.status, UpdateStatus::kOk ) << result.message;
      if ( result.diagnostics.outliers_culled >= 1U )
      {
        EXPECT_TRUE( containsId( result.diagnostics.culled_landmark_ids,
                                 poison_id ) );
        cull_frame = static_cast<int>( index );
        break;
      }
    }
    EXPECT_GE( cull_frame, 0 );
    if ( cull_frame < 0 )
    {
      return 0;
    }

    // Clean observation of poison_id: rebirth candidate.
    const std::size_t rebirth = static_cast<std::size_t>( cull_frame + 1 );
    EXPECT_LT( rebirth + 1, poses.size() );
    {
      const std::int64_t ts_ns =
          static_cast<std::int64_t>( rebirth + 1 ) * 50'000'000;
      const auto result = estimator.update(
          makeFrame( calibration, poses[ rebirth ], ts_ns, kLandmarks, ids ) );
      EXPECT_EQ( result.status, UpdateStatus::kOk ) << result.message;
      EXPECT_EQ( result.diagnostics.num_shared, ids.size() - 1U );
    }

    // Next frame: shared count reveals whether poison_id was reborn.
    const std::size_t probe = rebirth + 1;
    const std::int64_t ts_ns =
        static_cast<std::int64_t>( probe + 1 ) * 50'000'000;
    const auto probe_result = estimator.update(
        makeFrame( calibration, poses[ probe ], ts_ns, kLandmarks, ids ) );
    EXPECT_EQ( probe_result.status, UpdateStatus::kOk ) << probe_result.message;
    return probe_result.diagnostics.num_shared;
  };

  EXPECT_EQ( run( true ), ids.size() - 1U );
  EXPECT_EQ( run( false ), ids.size() );
}

TEST( StereoVoCullRebirthTest, CheiralityIdsAppearInCulledLandmarkIds )
{
  // Approach with true near projections, then one step whose CV init lands
  // past the landmark while still attaching a stale stereo obs (PnP off so
  // the obs stays in-window for dropCheiralityLandmarks).
  const auto       calibration = makeCalibration();
  EstimatorOptions options;
  options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
  options.window_size                = 8;
  options.min_shared_landmarks       = 2;
  options.min_landmark_observations  = 2;
  options.huber_k_px                 = 0.0;
  options.enable_outlier_cull        = false;
  options.enable_pnp_init            = false;
  options.use_constant_velocity_init = true;

  const std::vector<Eigen::Vector3d> far_landmarks{
      { 0.4, 0.1, 8.0 },
      { -0.3, 0.2, 7.5 },
      { 0.1, -0.25, 9.0 },
      { 0.6, -0.1, 8.5 },
      { -0.5, -0.2, 7.8 },
      { 0.0, 0.3, 8.2 },
      { 0.35, -0.05, 8.3 },
      { -0.15, 0.25, 7.6 },
      { 0.5, 0.05, 8.8 },
  };
  const Eigen::Vector3d near_landmark{ 0.15, 0.0, 1.5 };
  const auto            far_ids = sequentialIds( far_landmarks.size(), 1 );
  const LandmarkId      near_id = 99;
  // z = 0.0, 0.7, 1.4 (in front), then 2.8 with CV predict ≈ 2.1 (past near).
  const std::vector<double> zs{ 0.0, 0.7, 1.4, 2.8 };

  StereoVoEstimator estimator( calibration, options );
  StereoObservation near_locked{};
  bool              saw_list = false;

  for ( std::size_t index = 0; index < zs.size(); ++index )
  {
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    T_W_B.translation()     = Eigen::Vector3d( 0.0, 0.0, zs[ index ] );

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

    if ( index + 1 < zs.size() )
    {
      const auto near_obs =
          projectLandmark( calibration, T_W_B, near_id, near_landmark );
      if ( index + 2 == zs.size() )
      {
        near_locked = near_obs;
      }
      measurement.observations.push_back( near_obs );
    }
    else
    {
      measurement.observations.push_back( near_locked );
    }

    const auto result = estimator.update( measurement );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    EXPECT_EQ( result.diagnostics.outliers_culled, 0U );
    if ( containsId( result.diagnostics.culled_landmark_ids, near_id ) )
    {
      saw_list = true;
      EXPECT_GE( result.diagnostics.num_cheirality, 1U );
    }
  }
  EXPECT_TRUE( saw_list );
}
