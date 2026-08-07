#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/estimator/types.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::common::LandmarkId;
  using phad::common::Timestamp;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::KeyframeMeasurement;
  using phad::estimator::StereoObservation;
  using phad::estimator::StereoVoEstimator;
  using phad::estimator::UpdateStatus;
  using phad::sensor::RigidTransform;

  RectifiedStereoCalibration makeCalibration()
  {
    auto rigid =
        RigidTransform::create( Eigen::Isometry3d::Identity().matrix() ).value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480, std::move( rigid ) )
        .value();
  }

  // Left-camera pixel of point_W seen from body pose T_W_B.
  [[nodiscard]] Eigen::Vector2d projectLeft(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d&          T_W_B,
      const Eigen::Vector3d&            point_W )
  {
    Eigen::Isometry3d T_B_C = Eigen::Isometry3d::Identity();
    T_B_C.linear()          = calibration.T_B_left_rectified().rotation();
    T_B_C.translation()     = calibration.T_B_left_rectified().translation();
    const Eigen::Vector3d point_left = ( T_W_B * T_B_C ).inverse() * point_W;
    const double          z          = point_left.z();
    EXPECT_GT( z, 0.0 );
    return Eigen::Vector2d(
        calibration.fxPixels() * point_left.x() / z + calibration.cxPixels(),
        calibration.fyPixels() * point_left.y() / z + calibration.cyPixels() );
  }

  [[nodiscard]] double stereoDisparity(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d&          T_W_B,
      const Eigen::Vector3d&            point_W )
  {
    Eigen::Isometry3d T_B_C = Eigen::Isometry3d::Identity();
    T_B_C.linear()          = calibration.T_B_left_rectified().rotation();
    T_B_C.translation()     = calibration.T_B_left_rectified().translation();
    const Eigen::Vector3d point_left = ( T_W_B * T_B_C ).inverse() * point_W;
    return calibration.fxPixels() * calibration.baselineM() / point_left.z();
  }

  // A scene of 12 stereo-stable landmarks plus one "target" landmark whose
  // stereo match fails (disparity_px = 0) — the Slice ⑦ zero-disparity path
  // (glue channel; the landmark is seeded when its stereo match returns).
  struct Scene
  {
    RectifiedStereoCalibration    calibration = makeCalibration();
    std::vector<LandmarkId>       stable_ids;
    std::vector<Eigen::Vector3d>  stable_W;
    LandmarkId                    target_id{ 100 };
    Eigen::Vector3d               target_W{ 0.3, 0.0, 5.0 };
    const std::int64_t            ts_step_ns = 100'000'000;

    Scene()
    {
      stable_ids.reserve( 12 );
      stable_W.reserve( 12 );
      int index = 0;
      for ( int iz = 0; iz < 3; ++iz )
      {
        for ( int iy = -1; iy <= 1; ++iy )
        {
          for ( int ix = -1; ix <= 1; ++ix )
          {
            if ( index >= 12 )
            {
              break;
            }
            stable_ids.push_back( LandmarkId{ static_cast<std::uint64_t>( index ) } );
            stable_W.emplace_back( 0.6 * static_cast<double>( ix ),
                                   -1.0 + 0.8 * static_cast<double>( iy ),
                                   4.0 + 1.5 * static_cast<double>( iz ) );
            ++index;
          }
        }
      }
    }

    // frame k (keyframe) at pose T_W_B; include_stereo_target selects whether
    // the target's right-eye match succeeds on this frame; include_target
    // omits the target entirely (realistic birth: the frontend drops tracks
    // of length 1, so the target's first estimator sighting is frame 1).
    [[nodiscard]] KeyframeMeasurement frame(
        const Eigen::Isometry3d& T_W_B, std::uint32_t k,
        bool include_stereo_target, bool include_target = true ) const
    {
      KeyframeMeasurement m;
      m.timestamp = Timestamp{ ts_step_ns * ( static_cast<std::int64_t>( k ) + 1 ) };
      for ( std::size_t i = 0; i < stable_ids.size(); ++i )
      {
        m.observations.push_back( StereoObservation{
            stable_ids[ i ], projectLeft( calibration, T_W_B, stable_W[ i ] ),
            stereoDisparity( calibration, T_W_B, stable_W[ i ] ) } );
      }
      if ( include_target )
      {
        m.observations.push_back( StereoObservation{
            target_id, projectLeft( calibration, T_W_B, target_W ),
            include_stereo_target
                ? stereoDisparity( calibration, T_W_B, target_W )
                : 0.0 } );
      }
      return m;
    }
  };

  [[nodiscard]] Eigen::Isometry3d translated( double x, double y = 0.0,
                                              double z = 0.0 )
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation()     = Eigen::Vector3d( x, y, z );
    return T;
  }

}  // namespace

TEST( MultiFrameTriangulationTest, BuildGraphSkipsZeroDisparity )
{
  // The target's zero-disparity observations must not count toward
  // min_landmark_observations: the landmark enters the BA graph only once it
  // has two real stereo observations (frame 4), never earlier.
  Scene             scene;
  StereoVoEstimator estimator( scene.calibration );

  auto r0 = estimator.update( scene.frame( translated( 0.0 ), 0,
                              /*include_stereo_target=*/false,
                              /*include_target=*/false ),
                              /*keyframe=*/true );
  EXPECT_EQ( r0.status, UpdateStatus::kOk );

  auto r1 = estimator.update( scene.frame( translated( 0.5 ), 1, false ),
                              /*keyframe=*/true );
  EXPECT_EQ( r1.status, UpdateStatus::kOk );

  // Zero-disparity observations only — never seeded, not in the graph.
  auto r2 = estimator.update( scene.frame( translated( 1.0 ), 2, false ),
                              /*keyframe=*/true );
  EXPECT_EQ( r2.status, UpdateStatus::kOk );
  EXPECT_EQ( r2.diagnostics.num_landmarks, 12U );

  // One stereo observation: counts = 1 < 2 — still out of the graph (had
  // zero-disparity observations been counted, this would already be 13).
  auto r3 = estimator.update( scene.frame( translated( 1.5 ), 3, true ),
                              /*keyframe=*/true );
  EXPECT_EQ( r3.status, UpdateStatus::kOk );
  EXPECT_EQ( r3.diagnostics.num_landmarks, 12U );

  // Second stereo observation: enters the graph with two real factors.
  auto r4 = estimator.update( scene.frame( translated( 2.0 ), 4, true ),
                              /*keyframe=*/true );
  EXPECT_EQ( r4.status, UpdateStatus::kOk );
  EXPECT_EQ( r4.diagnostics.num_landmarks, 13U );
}
