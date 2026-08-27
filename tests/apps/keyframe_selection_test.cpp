#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>

#include "apps/keyframe_epoch_gate.hpp"
#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/estimator/types.hpp"
#include "phad/frontend/stereo_tracks.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::apps::KeyframeDecision;
  using phad::apps::KeyframeEpochGate;
  using phad::apps::KeyframeEvent;
  using phad::apps::KeyframeFeedback;
  using phad::apps::KeyframeRule;
  using phad::camera::RectifiedStereoCalibration;
  using phad::common::LandmarkId;
  using phad::common::Timestamp;
  using phad::estimator::UpdateStatus;
  using phad::frontend::FrameTracks;
  using phad::frontend::StereoStatus;
  using phad::frontend::TrackObservation;
  using phad::sensor::RigidTransform;

  RectifiedStereoCalibration makeCalibration(
      Eigen::Matrix4d T_B_C = Eigen::Matrix4d::Identity() )
  {
    auto transform = RigidTransform::create( std::move( T_B_C ) );
    EXPECT_TRUE( transform );
    auto calibration = RectifiedStereoCalibration::create(
        400.0, 400.0, 320.0, 240.0, 0.1, 640, 480,
        std::move( transform ).value() );
    EXPECT_TRUE( calibration );
    return std::move( calibration ).value();
  }

  FrameTracks makeTracks( const std::int64_t    timestamp_ns,
                          const std::size_t     count,
                          const Eigen::Vector2d pixel,
                          const std::uint64_t   first_id = 0 )
  {
    FrameTracks tracks;
    tracks.timestamp = Timestamp{ timestamp_ns };
    tracks.stats     = {};
    tracks.observations.reserve( count );
    for ( std::size_t i = 0; i < count; ++i )
    {
      tracks.observations.push_back( TrackObservation{
          LandmarkId{ first_id + static_cast<std::uint64_t>( i ) },
          pixel,
          1.0,
          StereoStatus::kValid,
          1U } );
    }
    return tracks;
  }

  FrameTracks moveTracks( const FrameTracks&     source,
                          const std::int64_t     timestamp_ns,
                          const Eigen::Vector2d& delta )
  {
    FrameTracks tracks = source;
    tracks.timestamp   = Timestamp{ timestamp_ns };
    for ( auto& obs : tracks.observations )
    {
      obs.left_pixel += delta;
    }
    return tracks;
  }

  Eigen::Matrix3d rotationZ( const double angle_rad )
  {
    const double    c = std::cos( angle_rad );
    const double    s = std::sin( angle_rad );
    Eigen::Matrix3d rotation;
    rotation << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
    return rotation;
  }

  FrameTracks rotateTracks( const FrameTracks&     source,
                            const std::int64_t     timestamp_ns,
                            const Eigen::Matrix3d& R_source_to_current )
  {
    FrameTracks tracks = source;
    tracks.timestamp   = Timestamp{ timestamp_ns };
    for ( auto& obs : tracks.observations )
    {
      const Eigen::Vector3d source_ray(
          ( obs.left_pixel.x() - 320.0 ) / 400.0,
          ( obs.left_pixel.y() - 240.0 ) / 400.0, 1.0 );
      const Eigen::Vector3d current_ray =
          R_source_to_current * source_ray;
      obs.left_pixel = {
          current_ray.x() / current_ray.z() * 400.0 + 320.0,
          current_ray.y() / current_ray.z() * 400.0 + 240.0 };
    }
    return tracks;
  }

  KeyframeEvent resolve(
      KeyframeEpochGate& gate, const KeyframeDecision& decision,
      const UpdateStatus             status,
      std::optional<Eigen::Matrix3d> R_W_B = std::nullopt )
  {
    KeyframeFeedback feedback;
    feedback.status = status;
    feedback.R_W_B  = std::move( R_W_B );
    return gate.resolve( decision.ticket, feedback );
  }

  void accept( KeyframeEpochGate& gate, const FrameTracks& tracks,
               const Eigen::Matrix3d& R_W_B = Eigen::Matrix3d::Identity() )
  {
    const KeyframeDecision decision = gate.decide( tracks );
    ASSERT_TRUE( decision.selected );
    const KeyframeEvent event =
        resolve( gate, decision, UpdateStatus::kOk, R_W_B );
    EXPECT_TRUE( event.epoch_committed );
  }

  FrameTracks seedGate(
      KeyframeEpochGate&     gate,
      const Eigen::Matrix3d& first_rotation  = Eigen::Matrix3d::Identity(),
      const Eigen::Matrix3d& second_rotation = Eigen::Matrix3d::Identity() )
  {
    const FrameTracks first =
        makeTracks( 100'000'000, 10U, { 150.0, 100.0 } );
    accept( gate, first, first_rotation );

    const FrameTracks second =
        makeTracks( 200'000'000, 10U, { 160.0, 100.0 } );
    accept( gate, second, second_rotation );
    return second;
  }

  TEST( KeyframeEpochGateTest, EmptyVetoPrecedesBootstrap )
  {
    KeyframeEpochGate gate( makeCalibration() );
    FrameTracks       empty;
    empty.timestamp = Timestamp{ 100'000'000 };
    empty.stats     = {};

    const KeyframeDecision decision = gate.decide( empty );
    EXPECT_FALSE( decision.selected );
    EXPECT_EQ( decision.rule, KeyframeRule::kNone );
    EXPECT_TRUE( decision.evidence.triggers.empty );
    EXPECT_TRUE( decision.evidence.triggers.bootstrap );
    EXPECT_TRUE( decision.evidence.triggers.low_tracks );

    const KeyframeEvent event =
        resolve( gate, decision, UpdateStatus::kRejected );
    EXPECT_FALSE( event.epoch_committed );
    EXPECT_EQ( event.epoch, 0U );
  }

  TEST( KeyframeEpochGateTest, BootstrapCountsOnlyAcceptedKeyframes )
  {
    KeyframeEpochGate gate( makeCalibration() );
    const FrameTracks tracks =
        makeTracks( 100'000'000, 10U, { 100.0, 100.0 } );

    const KeyframeDecision rejected = gate.decide( tracks );
    EXPECT_EQ( rejected.rule, KeyframeRule::kBootstrap );
    EXPECT_FALSE(
        resolve( gate, rejected, UpdateStatus::kRejected ).epoch_committed );

    const KeyframeDecision first = gate.decide( tracks );
    EXPECT_EQ( first.evidence.epoch, 0U );
    EXPECT_EQ( first.rule, KeyframeRule::kBootstrap );
    EXPECT_EQ( resolve( gate, first, UpdateStatus::kOk ).epoch, 1U );

    const FrameTracks second_tracks =
        makeTracks( 200'000'000, 10U, { 100.0, 100.0 } );
    const KeyframeDecision second = gate.decide( second_tracks );
    EXPECT_EQ( second.evidence.epoch, 1U );
    EXPECT_EQ( second.rule, KeyframeRule::kBootstrap );
    EXPECT_EQ( resolve( gate, second, UpdateStatus::kOk ).epoch, 2U );

    const FrameTracks third_tracks =
        makeTracks( 300'000'000, 10U, { 100.0, 100.0 } );
    const KeyframeDecision third = gate.decide( third_tracks );
    EXPECT_FALSE( third.selected );
    EXPECT_EQ( third.evidence.epoch, 2U );
    resolve( gate, third, UpdateStatus::kOk );
  }

  TEST( KeyframeEpochGateTest, LowTrackRulePrecedesOtherTriggers )
  {
    KeyframeEpochGate gate( makeCalibration() );
    seedGate( gate );

    const FrameTracks tracks =
        makeTracks( 800'000'001, 9U, { 240.0, 100.0 }, 100U );
    const KeyframeDecision decision = gate.decide( tracks );
    EXPECT_EQ( decision.rule, KeyframeRule::kLowTracks );
    EXPECT_TRUE( decision.evidence.triggers.low_tracks );
    EXPECT_TRUE( decision.evidence.triggers.timeout );
    EXPECT_TRUE( decision.evidence.triggers.low_survival );
    EXPECT_FALSE( decision.evidence.triggers.parallax );
    resolve( gate, decision, UpdateStatus::kRejected );
  }

  TEST( KeyframeEpochGateTest, TimeoutUsesStrictHalfSecondBoundary )
  {
    KeyframeEpochGate gate( makeCalibration() );
    const FrameTracks last_keyframe = seedGate( gate );

    const FrameTracks at_boundary =
        moveTracks( last_keyframe, 700'000'000, { 0.0, 0.0 } );
    const KeyframeDecision boundary = gate.decide( at_boundary );
    EXPECT_FALSE( boundary.selected );
    EXPECT_FALSE( boundary.evidence.triggers.timeout );
    resolve( gate, boundary, UpdateStatus::kOk );

    const FrameTracks after_boundary =
        moveTracks( last_keyframe, 700'000'001, { 0.0, 0.0 } );
    const KeyframeDecision after = gate.decide( after_boundary );
    EXPECT_EQ( after.rule, KeyframeRule::kTimeout );
    EXPECT_TRUE( after.evidence.triggers.timeout );
    resolve( gate, after, UpdateStatus::kRejected );
  }

  TEST( KeyframeEpochGateTest, SurvivalUsesStrictSixtyPercentBoundary )
  {
    KeyframeEpochGate gate( makeCalibration() );
    seedGate( gate );

    FrameTracks at_boundary =
        makeTracks( 300'000'000, 6U, { 160.0, 100.0 }, 0U );
    FrameTracks new_tracks =
        makeTracks( 300'000'000, 4U, { 160.0, 100.0 }, 100U );
    at_boundary.observations.insert(
        at_boundary.observations.end(), new_tracks.observations.begin(),
        new_tracks.observations.end() );
    const KeyframeDecision boundary = gate.decide( at_boundary );
    EXPECT_DOUBLE_EQ( boundary.evidence.survival_ratio, 0.6 );
    EXPECT_FALSE( boundary.evidence.triggers.low_survival );
    EXPECT_FALSE( boundary.selected );
    resolve( gate, boundary, UpdateStatus::kOk );

    FrameTracks below =
        makeTracks( 400'000'000, 5U, { 160.0, 100.0 }, 0U );
    new_tracks =
        makeTracks( 400'000'000, 5U, { 160.0, 100.0 }, 100U );
    below.observations.insert( below.observations.end(),
                               new_tracks.observations.begin(),
                               new_tracks.observations.end() );
    const KeyframeDecision decision = gate.decide( below );
    EXPECT_EQ( decision.rule, KeyframeRule::kLowSurvival );
    EXPECT_TRUE( decision.evidence.triggers.low_survival );
    resolve( gate, decision, UpdateStatus::kRejected );
  }

  TEST( KeyframeEpochGateTest, ParallaxUsesStrictThirtyPixelBoundary )
  {
    KeyframeEpochGate gate( makeCalibration() );
    const FrameTracks last_keyframe = seedGate( gate );

    const FrameTracks at_boundary =
        moveTracks( last_keyframe, 300'000'000, { 30.0, 0.0 } );
    const KeyframeDecision boundary = gate.decide( at_boundary );
    EXPECT_DOUBLE_EQ( boundary.evidence.raw_parallax_px, 30.0 );
    EXPECT_DOUBLE_EQ(
        boundary.evidence.compensated_parallax_px, 30.0 );
    EXPECT_FALSE( boundary.evidence.triggers.parallax );
    EXPECT_FALSE( boundary.selected );
    resolve( gate, boundary, UpdateStatus::kOk );

    const FrameTracks over_boundary =
        moveTracks( last_keyframe, 400'000'000, { 30.001, 0.0 } );
    const KeyframeDecision over = gate.decide( over_boundary );
    EXPECT_EQ( over.rule, KeyframeRule::kParallax );
    EXPECT_TRUE( over.evidence.triggers.parallax );
    resolve( gate, over, UpdateStatus::kRejected );
  }

  TEST( KeyframeEpochGateTest, AcceptedSnapshotKeepsPreviousCausalPose )
  {
    KeyframeEpochGate     gate( makeCalibration() );
    const Eigen::Matrix3d first_rotation = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d second_rotation =
        rotationZ( 30.0 * std::numbers::pi_v<double> / 180.0 );
    const FrameTracks last_keyframe =
        seedGate( gate, first_rotation, second_rotation );

    const FrameTracks current = rotateTracks(
        last_keyframe, 300'000'000, second_rotation.transpose() * first_rotation );
    const KeyframeDecision decision = gate.decide( current );
    EXPECT_GT( decision.evidence.raw_parallax_px, 30.0 );
    EXPECT_NEAR( decision.evidence.compensated_parallax_px, 0.0, 1e-10 );
    EXPECT_FALSE( decision.evidence.triggers.parallax );
    EXPECT_FALSE( decision.selected );
    resolve( gate, decision, UpdateStatus::kOk, second_rotation );
  }

  TEST( KeyframeEpochGateTest, AcceptedNonKeyframeUpdatesCausalPose )
  {
    KeyframeEpochGate     gate( makeCalibration() );
    const Eigen::Matrix3d first_rotation = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d second_rotation =
        rotationZ( 20.0 * std::numbers::pi_v<double> / 180.0 );
    const FrameTracks last_keyframe =
        seedGate( gate, first_rotation, second_rotation );

    const FrameTracks current = rotateTracks(
        last_keyframe, 300'000'000, second_rotation.transpose() * first_rotation );
    const Eigen::Matrix3d third_rotation =
        rotationZ( 35.0 * std::numbers::pi_v<double> / 180.0 );
    const KeyframeDecision non_keyframe = gate.decide( current );
    ASSERT_FALSE( non_keyframe.selected );
    resolve( gate, non_keyframe, UpdateStatus::kOk, third_rotation );

    const FrameTracks next = rotateTracks(
        last_keyframe, 400'000'000,
        third_rotation.transpose() * first_rotation );
    const KeyframeDecision decision = gate.decide( next );
    EXPECT_GT( decision.evidence.raw_parallax_px, 30.0 );
    EXPECT_NEAR( decision.evidence.compensated_parallax_px, 0.0, 1e-10 );
    EXPECT_FALSE( decision.selected );
    resolve( gate, decision, UpdateStatus::kOk );
  }

  TEST( KeyframeEpochGateTest, RejectedAndFailedFramesDoNotUpdateCausalPose )
  {
    KeyframeEpochGate     gate( makeCalibration() );
    const Eigen::Matrix3d first_rotation = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d second_rotation =
        rotationZ( 30.0 * std::numbers::pi_v<double> / 180.0 );
    const FrameTracks last_keyframe =
        seedGate( gate, first_rotation, second_rotation );
    const FrameTracks current = rotateTracks(
        last_keyframe, 300'000'000, second_rotation.transpose() * first_rotation );
    const Eigen::Matrix3d ignored_rotation =
        rotationZ( 80.0 * std::numbers::pi_v<double> / 180.0 );

    const KeyframeDecision rejected = gate.decide( current );
    ASSERT_FALSE( rejected.selected );
    resolve( gate, rejected, UpdateStatus::kRejected, ignored_rotation );

    const KeyframeDecision failed = gate.decide(
        rotateTracks( last_keyframe, 400'000'000,
                      second_rotation.transpose() * first_rotation ) );
    ASSERT_FALSE( failed.selected );
    resolve( gate, failed, UpdateStatus::kFailed, ignored_rotation );

    const KeyframeDecision after = gate.decide(
        rotateTracks( last_keyframe, 500'000'000,
                      second_rotation.transpose() * first_rotation ) );
    EXPECT_NEAR( after.evidence.compensated_parallax_px, 0.0, 1e-10 );
    EXPECT_FALSE( after.selected );
    resolve( gate, after, UpdateStatus::kOk );
  }

  TEST( KeyframeEpochGateTest, AlternativeIdentityMatchesProductionEvidence )
  {
    KeyframeEpochGate gate( makeCalibration() );
    const FrameTracks last_keyframe = seedGate( gate );
    const FrameTracks current =
        moveTracks( last_keyframe, 300'000'000, { 12.0, -4.0 } );

    const KeyframeDecision decision    = gate.decide( current );
    const auto             alternative = gate.evaluateRotation(
        decision.ticket, current, Eigen::Matrix3d::Identity() );
    EXPECT_EQ( alternative.common_count, decision.evidence.common_count );
    EXPECT_EQ( alternative.parallax_count,
               decision.evidence.parallax_count );
    EXPECT_DOUBLE_EQ( alternative.raw_parallax_px,
                      decision.evidence.raw_parallax_px );
    EXPECT_DOUBLE_EQ( alternative.compensated_parallax_px,
                      decision.evidence.compensated_parallax_px );

    const KeyframeEvent event =
        resolve( gate, decision, UpdateStatus::kOk );
    EXPECT_FALSE( event.epoch_committed );
    EXPECT_EQ( event.epoch, 2U );
  }

  TEST( KeyframeEpochGateTest, AlternativeEvidenceRequiresPendingFrame )
  {
    KeyframeEpochGate gate( makeCalibration() );
    const FrameTracks tracks =
        makeTracks( 100'000'000, 10U, { 100.0, 100.0 } );
    const KeyframeDecision decision = gate.decide( tracks );

    EXPECT_THROW(
        static_cast<void>( gate.evaluateRotation(
            decision.ticket + 1U, tracks, Eigen::Matrix3d::Identity() ) ),
        std::logic_error );
    FrameTracks wrong_frame = tracks;
    wrong_frame.timestamp   = Timestamp{ 100'000'001 };
    EXPECT_THROW(
        static_cast<void>( gate.evaluateRotation(
            decision.ticket, wrong_frame, Eigen::Matrix3d::Identity() ) ),
        std::logic_error );

    resolve( gate, decision, UpdateStatus::kRejected );
    EXPECT_THROW(
        static_cast<void>( gate.evaluateRotation(
            decision.ticket, tracks, Eigen::Matrix3d::Identity() ) ),
        std::logic_error );
  }

  TEST( KeyframeEpochGateTest, RejectsInvalidTransactionOrder )
  {
    KeyframeEpochGate gate( makeCalibration() );
    const FrameTracks tracks =
        makeTracks( 100'000'000, 10U, { 100.0, 100.0 } );
    const KeyframeDecision decision = gate.decide( tracks );

    EXPECT_THROW( static_cast<void>( gate.decide( tracks ) ),
                  std::logic_error );
    KeyframeFeedback feedback;
    feedback.status = UpdateStatus::kRejected;
    EXPECT_THROW( static_cast<void>(
                      gate.resolve( decision.ticket + 1U, feedback ) ),
                  std::logic_error );

    EXPECT_NO_THROW( static_cast<void>(
        gate.resolve( decision.ticket, feedback ) ) );
    EXPECT_THROW( static_cast<void>(
                      gate.resolve( decision.ticket, feedback ) ),
                  std::logic_error );
  }

  TEST( KeyframeEpochGateTest, AcceptsValidatedNonIdentityExtrinsicCalibration )
  {
    Eigen::Matrix4d T_B_C = Eigen::Matrix4d::Identity();
    T_B_C.block<3, 3>( 0, 0 ) =
        rotationZ( 15.0 * std::numbers::pi_v<double> / 180.0 );
    T_B_C.block<3, 1>( 0, 3 ) = Eigen::Vector3d{ 0.1, -0.02, 0.03 };
    KeyframeEpochGate gate( makeCalibration( T_B_C ) );
    const FrameTracks last_keyframe = seedGate( gate );

    const KeyframeDecision decision = gate.decide(
        moveTracks( last_keyframe, 300'000'000, { 0.0, 0.0 } ) );
    EXPECT_FALSE( decision.selected );
    EXPECT_EQ( decision.evidence.common_count, 10U );
    resolve( gate, decision, UpdateStatus::kOk );
  }

}  // namespace
