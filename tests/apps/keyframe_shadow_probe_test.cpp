#include "apps/keyframe_shadow_probe.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "apps/keyframe_epoch_gate.hpp"
#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/estimator/types.hpp"
#include "phad/frontend/stereo_tracks.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::apps::KeyframeDecision;
  using phad::apps::KeyframeEpochGate;
  using phad::apps::KeyframeEvent;
  using phad::apps::KeyframeFeedback;
  using phad::apps::KeyframeShadowProbe;
  using phad::camera::RectifiedStereoCalibration;
  using phad::common::LandmarkId;
  using phad::common::Timestamp;
  using phad::estimator::UpdateStatus;
  using phad::frontend::FrameTracks;
  using phad::frontend::StereoStatus;
  using phad::frontend::TrackObservation;
  using phad::sensor::ImuMeasurement;
  using phad::sensor::RigidTransform;

  struct CsvTable
  {
    std::unordered_map<std::string, std::size_t> columns;
    std::vector<std::vector<std::string>>        rows;
  };

  Eigen::Matrix3d rotation( const Eigen::Vector3d& axis,
                            const double           angle_rad )
  {
    return Eigen::AngleAxisd( angle_rad, axis.normalized() )
        .toRotationMatrix();
  }

  RectifiedStereoCalibration makeCalibration(
      const Eigen::Matrix3d& R_B_C = Eigen::Matrix3d::Identity() )
  {
    Eigen::Matrix4d T_B_C     = Eigen::Matrix4d::Identity();
    T_B_C.block<3, 3>( 0, 0 ) = R_B_C;
    auto transform            = RigidTransform::create( T_B_C );
    EXPECT_TRUE( transform );
    auto calibration = RectifiedStereoCalibration::create(
        400.0, 400.0, 320.0, 240.0, 0.1, 640, 480,
        std::move( transform ).value() );
    EXPECT_TRUE( calibration );
    return std::move( calibration ).value();
  }

  FrameTracks makeTracks( const std::int64_t     timestamp_ns,
                          const Eigen::Vector2d& pixel )
  {
    FrameTracks tracks;
    tracks.timestamp = Timestamp{ timestamp_ns };
    tracks.stats     = {};
    for ( std::uint64_t id = 0; id < 10U; ++id )
    {
      tracks.observations.push_back( TrackObservation{
          LandmarkId{ id }, pixel, 1.0, StereoStatus::kValid, 1U } );
    }
    return tracks;
  }

  FrameTracks rotateTracks( const FrameTracks&     source,
                            const std::int64_t     timestamp_ns,
                            const Eigen::Matrix3d& R_source_to_current )
  {
    FrameTracks tracks = source;
    tracks.timestamp   = Timestamp{ timestamp_ns };
    for ( auto& obs : tracks.observations )
    {
      const Eigen::Vector3d ray(
          ( obs.left_pixel.x() - 320.0 ) / 400.0,
          ( obs.left_pixel.y() - 240.0 ) / 400.0, 1.0 );
      const Eigen::Vector3d rotated = R_source_to_current * ray;
      obs.left_pixel                = { rotated.x() / rotated.z() * 400.0 + 320.0,
                                        rotated.y() / rotated.z() * 400.0 + 240.0 };
    }
    return tracks;
  }

  std::vector<ImuMeasurement> makeImuSegment(
      const std::int64_t from_ns, const std::int64_t to_ns,
      const Eigen::Vector3d& gyro )
  {
    ImuMeasurement first;
    first.timestamp     = Timestamp{ from_ns };
    first.gyro_radps    = { gyro.x(), gyro.y(), gyro.z() };
    ImuMeasurement last = first;
    last.timestamp      = Timestamp{ to_ns };
    return { first, last };
  }

  KeyframeEvent resolveGate(
      KeyframeEpochGate& gate, const KeyframeDecision& decision,
      const UpdateStatus             status,
      std::optional<Eigen::Matrix3d> R_W_B = std::nullopt )
  {
    KeyframeFeedback feedback;
    feedback.status = status;
    feedback.R_W_B  = std::move( R_W_B );
    return gate.resolve( decision.ticket, feedback );
  }

  std::vector<std::string> split( const std::string& line )
  {
    std::vector<std::string> cells;
    std::stringstream        stream( line );
    std::string              cell;
    while ( std::getline( stream, cell, ',' ) )
    {
      cells.push_back( cell );
    }
    return cells;
  }

  CsvTable readCsv( const std::filesystem::path& path )
  {
    std::ifstream input( path );
    EXPECT_TRUE( input );
    std::string header_line;
    std::getline( input, header_line );
    const std::vector<std::string> header = split( header_line );

    CsvTable table;
    for ( std::size_t i = 0; i < header.size(); ++i )
    {
      table.columns.emplace( header[ i ], i );
    }
    std::string line;
    while ( std::getline( input, line ) )
    {
      table.rows.push_back( split( line ) );
    }
    return table;
  }

  const std::string& cell( const CsvTable& table, const std::size_t row,
                           const std::string& name )
  {
    return table.rows.at( row ).at( table.columns.at( name ) );
  }

  TEST( KeyframeShadowProbeTest,
        WritesValidRotationWithNonIdentityExtrinsic )
  {
    const auto path = std::filesystem::temp_directory_path() /
                      "phad_keyframe_shadow_extrinsic.csv";
    std::filesystem::remove( path );
    const Eigen::Matrix3d R_B_C = rotation(
        Eigen::Vector3d::UnitX(),
        20.0 * std::numbers::pi_v<double> / 180.0 );
    const RectifiedStereoCalibration calibration = makeCalibration( R_B_C );
    KeyframeEpochGate                gate( calibration );
    {
      KeyframeShadowProbe   probe( path, calibration, true );
      const Eigen::Vector3d gyro{ 0.0, 0.0, 1.0 };

      const FrameTracks first_tracks =
          makeTracks( 100'000'000, { 420.0, 240.0 } );
      const KeyframeDecision first     = gate.decide( first_tracks );
      const auto             first_imu = makeImuSegment( 0, 100'000'000, gyro );
      probe.observe( 0U, first_imu, false, first_tracks, first, gate );
      const KeyframeEvent first_event = resolveGate(
          gate, first, UpdateStatus::kOk, Eigen::Matrix3d::Identity() );
      probe.resolve( first_event, Eigen::Vector3d::Zero() );

      const Eigen::Matrix3d R_Bi_Bj =
          rotation( Eigen::Vector3d::UnitZ(), 0.1 );
      const Eigen::Matrix3d R_Ccur_Clkf =
          R_B_C.transpose() * R_Bi_Bj.transpose() * R_B_C;
      const FrameTracks second_tracks =
          rotateTracks( first_tracks, 200'000'000, R_Ccur_Clkf );
      const KeyframeDecision second = gate.decide( second_tracks );
      const auto             second_imu =
          makeImuSegment( 100'000'000, 200'000'000, gyro );
      probe.observe( 1U, second_imu, false, second_tracks, second, gate );
      const KeyframeEvent second_event = resolveGate(
          gate, second, UpdateStatus::kOk, Eigen::Matrix3d::Identity() );
      probe.resolve( second_event, Eigen::Vector3d::Zero() );
    }

    const CsvTable table = readCsv( path );
    ASSERT_EQ( table.rows.size(), 2U );
    EXPECT_EQ( cell( table, 0U, "imu_reason" ), "no_accepted_kf" );
    EXPECT_EQ( cell( table, 0U, "epoch_committed" ), "1" );
    EXPECT_EQ( cell( table, 1U, "imu_valid" ), "1" );
    EXPECT_EQ( cell( table, 1U, "imu_reason" ), "none" );
    EXPECT_EQ( cell( table, 1U, "imu_sample_n" ), "2" );
    EXPECT_EQ( cell( table, 1U, "frames_since_kf" ), "1" );
    EXPECT_NEAR( std::stod( cell( table, 1U, "imu_dt_s" ) ), 0.1,
                 1e-12 );
    EXPECT_NEAR( std::stod( cell( table, 1U, "imu_angle_rad" ) ), 0.1,
                 1e-12 );
    EXPECT_NEAR( std::stod( cell( table, 1U, "imu_comp_px" ) ), 0.0,
                 1e-10 );
    std::filesystem::remove( path );
  }

  TEST( KeyframeShadowProbeTest, RetainsRejectedSegmentAndUsesCausalBias )
  {
    const auto path = std::filesystem::temp_directory_path() /
                      "phad_keyframe_shadow_bias.csv";
    std::filesystem::remove( path );
    const RectifiedStereoCalibration calibration = makeCalibration();
    KeyframeEpochGate                gate( calibration );
    {
      KeyframeShadowProbe probe( path, calibration, true );
      FrameTracks         tracks = makeTracks( 100'000'000, { 320.0, 240.0 } );
      auto                imu    = makeImuSegment(
          0, 100'000'000, Eigen::Vector3d::Zero() );
      KeyframeDecision decision = gate.decide( tracks );
      probe.observe( 0U, imu, false, tracks, decision, gate );
      KeyframeEvent event = resolveGate( gate, decision, UpdateStatus::kOk );
      probe.resolve( event, Eigen::Vector3d::Zero() );

      const Eigen::Vector3d causal_bias{ 0.0, 0.0, 0.2 };
      tracks.timestamp = Timestamp{ 200'000'000 };
      imu              = makeImuSegment(
          100'000'000, 200'000'000, causal_bias );
      decision = gate.decide( tracks );
      probe.observe( 1U, imu, false, tracks, decision, gate );
      event = resolveGate( gate, decision, UpdateStatus::kOk );
      probe.resolve( event, causal_bias );

      tracks.timestamp = Timestamp{ 300'000'000 };
      imu              = makeImuSegment( 200'000'000, 300'000'000, causal_bias );
      decision         = gate.decide( tracks );
      ASSERT_FALSE( decision.selected );
      probe.observe( 2U, imu, false, tracks, decision, gate );
      event = resolveGate( gate, decision, UpdateStatus::kRejected );
      probe.resolve( event, { 0.0, 0.0, 0.9 } );

      tracks.timestamp = Timestamp{ 400'000'000 };
      imu              = makeImuSegment( 300'000'000, 400'000'000, causal_bias );
      decision         = gate.decide( tracks );
      ASSERT_FALSE( decision.selected );
      probe.observe( 3U, imu, false, tracks, decision, gate );
      event = resolveGate( gate, decision, UpdateStatus::kOk );
      probe.resolve( event, { 0.0, 0.0, 0.3 } );
    }

    const CsvTable table = readCsv( path );
    ASSERT_EQ( table.rows.size(), 4U );
    EXPECT_EQ( cell( table, 2U, "status" ), "rejected" );
    EXPECT_EQ( cell( table, 2U, "imu_sample_n" ), "2" );
    EXPECT_NEAR( std::stod( cell( table, 2U, "bias_gyr_z" ) ), 0.2,
                 1e-12 );
    EXPECT_NEAR( std::stod( cell( table, 2U, "imu_angle_rad" ) ), 0.0,
                 1e-12 );
    EXPECT_EQ( cell( table, 3U, "imu_sample_n" ), "3" );
    EXPECT_NEAR( std::stod( cell( table, 3U, "imu_dt_s" ) ), 0.2,
                 1e-12 );
    EXPECT_NEAR( std::stod( cell( table, 3U, "bias_gyr_z" ) ), 0.2,
                 1e-12 );
    EXPECT_NEAR( std::stod( cell( table, 3U, "imu_angle_rad" ) ), 0.0,
                 1e-12 );
    std::filesystem::remove( path );
  }

  TEST( KeyframeShadowProbeTest, RejectsIoAndTransactionMisuse )
  {
    const auto blocker = std::filesystem::temp_directory_path() /
                         "phad_shadow_parent_file";
    std::filesystem::remove_all( blocker );
    {
      std::ofstream output( blocker );
      ASSERT_TRUE( output );
    }
    const auto missing = blocker / "probe.csv";
    EXPECT_THROW(
        KeyframeShadowProbe( missing, makeCalibration(), true ),
        std::runtime_error );
    std::filesystem::remove( blocker );

    const auto path = std::filesystem::temp_directory_path() /
                      "phad_keyframe_shadow_transaction.csv";
    std::filesystem::remove( path );
    const RectifiedStereoCalibration calibration = makeCalibration();
    KeyframeEpochGate                gate( calibration );
    {
      KeyframeShadowProbe probe( path, calibration, false );
      const FrameTracks   tracks =
          makeTracks( 100'000'000, { 320.0, 240.0 } );
      const KeyframeDecision decision = gate.decide( tracks );
      const auto             imu      = makeImuSegment(
          0, 100'000'000, Eigen::Vector3d::Zero() );
      probe.observe( 0U, imu, false, tracks, decision, gate );
      EXPECT_THROW(
          probe.observe( 1U, imu, false, tracks, decision, gate ),
          std::logic_error );

      KeyframeEvent wrong_event;
      wrong_event.ticket   = decision.ticket + 1U;
      wrong_event.selected = decision.selected;
      EXPECT_THROW( probe.resolve( wrong_event, Eigen::Vector3d::Zero() ),
                    std::logic_error );

      const KeyframeEvent event =
          resolveGate( gate, decision, UpdateStatus::kRejected );
      EXPECT_NO_THROW( probe.resolve( event, Eigen::Vector3d::Zero() ) );
      EXPECT_THROW( probe.resolve( event, Eigen::Vector3d::Zero() ),
                    std::logic_error );
    }
    std::filesystem::remove( path );
  }

}  // namespace
