#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/internal/gyro_bias_initial_value.hpp"
#include "phad/estimator/internal/gyro_interval_reducer.hpp"
#include "phad/estimator/internal/gyro_rotation_factor.hpp"
#include "phad/estimator/internal/stereo_vo_update_transaction.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/sensor/rigid_transform.hpp"

// PHAD_M4_ONLINE_BIAS_AUTHORITY_CONTROL_BEGIN
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
  using phad::estimator::VioUpdateResult;
  using phad::sensor::RigidTransform;

  struct VisualLandmark
  {
    LandmarkId      m_id;
    Eigen::Vector3d m_point_W;
  };

  const std::vector<VisualLandmark> kVisualLandmarks{
      { 1U, { 0.40, 0.10, 5.0 } },
      { 2U, { -0.30, 0.20, 4.5 } },
      { 3U, { 0.10, -0.25, 6.0 } },
      { 4U, { 0.60, -0.10, 5.5 } },
      { 5U, { -0.50, -0.20, 4.8 } },
      { 6U, { 0.00, 0.30, 5.2 } },
      { 7U, { 0.25, 0.15, 4.2 } },
      { 8U, { -0.20, -0.15, 5.8 } },
      { 9U, { 0.35, -0.05, 5.3 } },
      { 10U, { -0.15, 0.25, 4.6 } },
      { 1000U, { 1.40, -0.30, 5.4 } },
      { 1001U, { 0.90, 0.35, 4.9 } },
      { 1002U, { 1.10, -0.15, 6.2 } },
      { 1003U, { 1.60, 0.05, 5.1 } },
      { 1004U, { 0.65, -0.40, 4.7 } },
      { 1005U, { 1.00, 0.20, 5.6 } },
      { 1006U, { 1.25, -0.20, 4.4 } },
      { 1007U, { 0.80, 0.30, 5.9 } },
      { 1008U, { 1.35, -0.05, 5.0 } },
      { 1009U, { 0.85, 0.15, 4.8 } },
  };

  [[nodiscard]] RectifiedStereoCalibration makeVisualCalibration()
  {
    auto rigid = RigidTransform::create( Eigen::Isometry3d::Identity().matrix() )
                     .value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480,
               std::move( rigid ) )
        .value();
  }

  [[nodiscard]] EstimatorOptions makeVisualOptions()
  {
    EstimatorOptions options;
    options.window_size                     = 14;
    options.min_landmark_observations       = 2;
    options.min_seed_observations           = 10;
    options.min_track_observations_for_seed = 1;
    options.min_shared_landmarks            = 10;
    options.stereo_sigma_px                 = 0.003;
    options.huber_k_px                      = 0.0;
    options.prior_rotation_sigma_rad        = 1e-4;
    options.prior_translation_sigma_m       = 1e-4;
    options.use_constant_velocity_init      = true;
    options.enable_reanchor                 = false;
    options.enable_pnp_init                 = false;
    options.pnp_reproj_px                   = 2.0;
    options.pnp_confidence                  = 0.99;
    options.min_pnp_inliers                 = 10;
    options.enable_outlier_cull             = false;
    options.enable_outlier_reopt            = false;
    options.max_outlier_reopts              = 0;
    options.outlier_avg_reproj_px           = 4.0;
    options.block_culled_rebirth            = true;
    options.hanging_landmark_gate_m         = 1.0;
    options.far_return_refresh_px           = 6.0;
    options.enable_accumulated_seed         = false;
    options.enable_probe_b                  = false;
    return options;
  }

  [[nodiscard]] StereoObservation projectVisualLandmark(
      const RectifiedStereoCalibration& calibration,
      const VisualLandmark&             landmark )
  {
    const double z = landmark.m_point_W.z();
    EXPECT_GT( z, 0.0 );
    const double u_l = calibration.fxPixels() * landmark.m_point_W.x() / z +
                       calibration.cxPixels();
    const double v = calibration.fyPixels() * landmark.m_point_W.y() / z +
                     calibration.cyPixels();
    const double disparity =
        calibration.fxPixels() * calibration.baselineM() / z;
    return StereoObservation{ landmark.m_id, Eigen::Vector2d( u_l, v ),
                              disparity };
  }

  [[nodiscard]] KeyframeMeasurement makeVisualMeasurement(
      const RectifiedStereoCalibration& calibration, std::size_t frame_index )
  {
    KeyframeMeasurement measurement;
    measurement.timestamp = phad::common::Timestamp{
        static_cast<std::int64_t>( frame_index + 1U ) * 100'000'000 };
    measurement.observations.reserve( kVisualLandmarks.size() );
    for ( const VisualLandmark& landmark : kVisualLandmarks )
    {
      measurement.observations.push_back(
          projectVisualLandmark( calibration, landmark ) );
    }
    return measurement;
  }

  void appendU8( std::string& bytes, std::uint8_t value )
  {
    bytes.push_back( static_cast<char>( value ) );
  }

  void appendU32( std::string& bytes, std::uint32_t value )
  {
    for ( int shift = 0; shift < 32; shift += 8 )
    {
      appendU8( bytes,
                static_cast<std::uint8_t>( value >> static_cast<unsigned>( shift ) ) );
    }
  }

  void appendU64( std::string& bytes, std::uint64_t value )
  {
    for ( int shift = 0; shift < 64; shift += 8 )
    {
      appendU8( bytes,
                static_cast<std::uint8_t>( value >> static_cast<unsigned>( shift ) ) );
    }
  }

  void appendI64( std::string& bytes, std::int64_t value )
  {
    appendU64( bytes, std::bit_cast<std::uint64_t>( value ) );
  }

  void appendDouble( std::string& bytes, double value )
  {
    appendU64( bytes, std::bit_cast<std::uint64_t>( value ) );
  }

  void appendBool( std::string& bytes, bool value )
  {
    appendU8( bytes, value ? 1U : 0U );
  }

  void appendText( std::string& bytes, const std::string& value )
  {
    appendU64( bytes, static_cast<std::uint64_t>( value.size() ) );
    bytes.append( value );
  }

  void appendDiagnostics( std::string&             bytes,
                          const UpdateDiagnostics& diagnostics )
  {
    appendU32( bytes, diagnostics.num_observations );
    appendU32( bytes, diagnostics.num_landmarks );
    appendU32( bytes, diagnostics.num_shared );
    appendU32( bytes, diagnostics.num_disparity );
    appendU32( bytes, diagnostics.num_cheirality );
    appendU32( bytes, diagnostics.lm_iterations );
    appendU32( bytes, diagnostics.window_size );
    appendU32( bytes, diagnostics.segment_id );
    appendU64( bytes, diagnostics.prior_key );
    appendDouble( bytes, diagnostics.reproj_rms_before_px );
    appendDouble( bytes, diagnostics.reproj_rms_after_px );
    appendDouble( bytes, diagnostics.max_window_pose_shift_m );
    appendBool( bytes, diagnostics.low_connectivity );
    appendBool( bytes, diagnostics.pnp_success );
    appendU32( bytes, diagnostics.pnp_inliers );
    appendU32( bytes, diagnostics.outliers_culled );
    appendU32( bytes, diagnostics.outliers_culled_unique );
    appendDouble( bytes, diagnostics.reproj_rms_after_cull_px );
    appendBool( bytes, diagnostics.outlier_reopt );
    appendBool( bytes, diagnostics.outlier_reopt_failed );
    appendU32( bytes, diagnostics.outlier_reopt_rounds );
    appendU64( bytes,
               static_cast<std::uint64_t>( diagnostics.culled_landmark_ids.size() ) );
    for ( const LandmarkId id : diagnostics.culled_landmark_ids )
    {
      appendU64( bytes, id );
    }
    appendU32( bytes, diagnostics.probe_rejected_block_n );
    appendU32( bytes, diagnostics.probe_new_lm_n );
    appendU64( bytes,
               static_cast<std::uint64_t>( diagnostics.probe_shift_top.size() ) );
    for ( const auto& [ key, shift ] : diagnostics.probe_shift_top )
    {
      appendU64( bytes, key );
      appendDouble( bytes, shift );
    }
    appendDouble( bytes, diagnostics.probe_res_mean_px );
    appendDouble( bytes, diagnostics.probe_res_max_px );
    appendU64( bytes, diagnostics.probe_res_max_id );
    appendBool( bytes, diagnostics.probe_detail_valid );
  }

  [[nodiscard]] std::string encodeVisualResults(
      const std::vector<VioUpdateResult>& results )
  {
    std::string bytes;
    appendU64( bytes, static_cast<std::uint64_t>( results.size() ) );
    for ( std::size_t frame_index = 0; frame_index < results.size();
          ++frame_index )
    {
      const VioUpdateResult& result = results[ frame_index ];
      appendU64( bytes, static_cast<std::uint64_t>( frame_index ) );
      appendU8( bytes, static_cast<std::uint8_t>( result.status ) );
      appendText( bytes, result.message );
      appendBool( bytes, result.estimate.has_value() );
      if ( result.estimate.has_value() )
      {
        appendI64( bytes, result.estimate->timestamp.nanoseconds() );
        const Eigen::Matrix4d matrix = result.estimate->T_W_B.matrix();
        for ( Eigen::Index row = 0; row < matrix.rows(); ++row )
        {
          for ( Eigen::Index column = 0; column < matrix.cols(); ++column )
          {
            appendDouble( bytes, matrix( row, column ) );
          }
        }
      }
      appendDiagnostics( bytes, result.diagnostics );
    }
    return bytes;
  }

  [[nodiscard]] std::string toHex( const std::string& bytes )
  {
    constexpr char kHex[] = "0123456789abcdef";
    std::string    hex;
    hex.reserve( bytes.size() * 2U );
    for ( const char raw_byte : bytes )
    {
      const auto byte = static_cast<unsigned char>( raw_byte );
      hex.push_back( kHex[ static_cast<std::size_t>( byte >> 4U ) ] );
      hex.push_back( kHex[ static_cast<std::size_t>( byte & 0x0fU ) ] );
    }
    return hex;
  }

}  // namespace

TEST( StereoVoOnlineGyroBias, AuthorityBaselineVisualControl )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  const EstimatorOptions           options     = makeVisualOptions();
  StereoVoEstimator                estimator( calibration, options );

  std::vector<VioUpdateResult> results;
  results.reserve( 14U );
  for ( std::size_t frame_index = 0; frame_index < 14U; ++frame_index )
  {
    const bool keyframe = frame_index < 2U;
    results.push_back( estimator.update(
        makeVisualMeasurement( calibration, frame_index ), keyframe ) );
    const VioUpdateResult& result = results.back();
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    ASSERT_TRUE( result.estimate.has_value() );
    EXPECT_TRUE( result.message.empty() );
    EXPECT_EQ( result.estimate->timestamp.nanoseconds(),
               static_cast<std::int64_t>( frame_index + 1U ) * 100'000'000 );
    EXPECT_EQ( result.diagnostics.window_size,
               static_cast<std::uint32_t>( frame_index + 1U ) );
    EXPECT_EQ( result.diagnostics.segment_id, 0U );
    EXPECT_EQ( result.diagnostics.prior_key, 0U );
  }

  const std::string canonical = encodeVisualResults( results );
  std::cout << "PHAD_M4_ONLINE_BIAS_AUTHORITY_CANONICAL_HEX="
            << toHex( canonical ) << '\n';
}
// PHAD_M4_ONLINE_BIAS_AUTHORITY_CONTROL_END

// PHAD_M4_ONLINE_BIAS_NO_EVICTION_BEGIN
#include "phad/sensor/imu_measurement.hpp"

namespace
{

  using phad::estimator::GyroBiasOptions;
  using phad::estimator::GyroBreakReason;
  using phad::estimator::GyroDiagnostics;
  using phad::estimator::GyroInterval;
  using phad::sensor::ImuMeasurement;

  const Eigen::Vector3d kBiasBase{ 0.012, -0.018, 0.025 };
  const Eigen::Vector3d kBiasStep{ 0.0001, -0.00015, 0.0002 };

  [[nodiscard]] Eigen::Vector3d biasTruth( std::size_t frame_index )
  {
    return kBiasBase + static_cast<double>( frame_index ) * kBiasStep;
  }

  [[nodiscard]] GyroInterval makeGyroInterval( std::size_t frame_index )
  {
    const std::size_t  interval_index = frame_index - 1U;
    const std::int64_t t_prev_ns =
        static_cast<std::int64_t>( frame_index ) * 100'000'000;
    const Eigen::Vector3d bias_left  = biasTruth( interval_index );
    const Eigen::Vector3d bias_mid   = bias_left - 0.5 * kBiasStep;
    const Eigen::Vector3d bias_right = biasTruth( frame_index );

    auto make_sample = []( std::int64_t           timestamp_ns,
                           const Eigen::Vector3d& gyro ) {
      ImuMeasurement sample;
      sample.timestamp  = phad::common::Timestamp{ timestamp_ns };
      sample.accel_mps2 = { 0.0, 0.0, 0.0 };
      sample.gyro_radps = { gyro.x(), gyro.y(), gyro.z() };
      return sample;
    };

    GyroInterval interval;
    interval.m_t_prev  = phad::common::Timestamp{ t_prev_ns };
    interval.m_samples = {
        make_sample( t_prev_ns, bias_left ),
        make_sample( t_prev_ns + 50'000'000, bias_mid ),
        make_sample( t_prev_ns + 100'000'000, bias_right ),
    };
    interval.m_imu_gap = false;
    return interval;
  }

  [[nodiscard]] GyroBiasOptions makeGyroOptions()
  {
    GyroBiasOptions options;
    options.m_gyr_nd            = 1e-4;
    options.m_gyr_rw            = 1e-3;
    options.m_prior_mean_radps  = Eigen::Vector3d::Zero();
    options.m_prior_sigma_radps = 0.1;
    return options;
  }

  void expectBiasNearTruth( const Eigen::Vector3d& bias,
                            std::size_t            frame_index )
  {
    EXPECT_TRUE( bias.allFinite() );
    EXPECT_LE( ( bias - biasTruth( frame_index ) ).cwiseAbs().maxCoeff(),
               5e-4 );
  }

}  // namespace

TEST( StereoVoOnlineGyroBias, NoEvictionFourteenPoseRecovery )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  Eigen::Vector3d frame_one_bias = Eigen::Vector3d::Zero();
  bool            have_frame_one = false;
  Eigen::Vector3d final_bias     = Eigen::Vector3d::Zero();

  for ( std::size_t frame_index = 0; frame_index < 14U; ++frame_index )
  {
    KeyframeMeasurement measurement =
        makeVisualMeasurement( calibration, frame_index );
    if ( frame_index > 0U )
    {
      measurement.m_gyro_interval = makeGyroInterval( frame_index );
    }
    const bool            keyframe = frame_index < 2U;
    const VioUpdateResult result   = estimator.update( measurement, keyframe );

    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    ASSERT_TRUE( result.estimate.has_value() );
    ASSERT_TRUE( result.diagnostics.m_gyro.has_value() );
    const GyroDiagnostics& gyro = *result.diagnostics.m_gyro;
    EXPECT_EQ( result.diagnostics.window_size,
               static_cast<std::uint32_t>( frame_index + 1U ) );
    EXPECT_EQ( gyro.m_window_biases.size(), frame_index + 1U );
    EXPECT_EQ( gyro.m_rotation_factors,
               static_cast<std::uint32_t>( frame_index ) );
    EXPECT_EQ( gyro.m_rw_factors,
               static_cast<std::uint32_t>( frame_index ) );
    EXPECT_EQ( gyro.m_root_priors, 1U );
    EXPECT_EQ( gyro.m_relinearization_rounds,
               frame_index == 1U ? 1U : 0U );
    EXPECT_EQ( gyro.m_break_reason, GyroBreakReason::kNone );

    for ( std::size_t window_index = 0;
          window_index < gyro.m_window_biases.size(); ++window_index )
    {
      const auto& window_bias = gyro.m_window_biases[ window_index ];
      EXPECT_EQ( window_bias.m_frame_index, window_index );
      EXPECT_EQ( window_bias.m_timestamp.nanoseconds(),
                 static_cast<std::int64_t>( window_index + 1U ) *
                     100'000'000 );
      if ( frame_index == 0U )
      {
        EXPECT_FALSE( window_bias.m_bias_radps.has_value() );
      }
      else
      {
        ASSERT_TRUE( window_bias.m_bias_radps.has_value() );
        expectBiasNearTruth( *window_bias.m_bias_radps, window_index );
      }
    }

    if ( frame_index == 0U )
    {
      EXPECT_FALSE( gyro.m_bias_radps.has_value() );
      continue;
    }

    ASSERT_TRUE( gyro.m_bias_radps.has_value() );
    ASSERT_TRUE( gyro.m_window_biases.back().m_bias_radps.has_value() );
    EXPECT_TRUE(
        ( gyro.m_bias_radps->array() ==
          gyro.m_window_biases.back().m_bias_radps->array() )
            .all() );
    if ( frame_index == 1U )
    {
      frame_one_bias = *gyro.m_bias_radps;
      have_frame_one = true;
    }
    if ( frame_index == 13U )
    {
      final_bias = *gyro.m_bias_radps;
    }
  }

  ASSERT_TRUE( have_frame_one );
  const Eigen::Vector3d truth_drift  = biasTruth( 12U ) - biasTruth( 0U );
  const Eigen::Vector3d actual_drift = final_bias - frame_one_bias;
  for ( Eigen::Index axis = 0; axis < actual_drift.size(); ++axis )
  {
    EXPECT_GT( actual_drift[ axis ] * truth_drift[ axis ], 0.0 );
    const double ratio = actual_drift[ axis ] / truth_drift[ axis ];
    EXPECT_GE( ratio, 0.85 );
    EXPECT_LE( ratio, 1.05 );
  }
  const double l2_ratio = actual_drift.norm() / truth_drift.norm();
  EXPECT_GE( l2_ratio, 0.85 );
  EXPECT_LE( l2_ratio, 1.05 );
}
// PHAD_M4_ONLINE_BIAS_NO_EVICTION_END

namespace
{

  using phad::estimator::internal::deriveGyroNoiseScales;
  using phad::estimator::internal::GyroBiasInitialErrorCode;
  using phad::estimator::internal::GyroBiasInitialKind;
  using phad::estimator::internal::GyroFrameState;
  using phad::estimator::internal::GyroIntervalError;
  using phad::estimator::internal::GyroIntervalErrorCode;
  using phad::estimator::internal::GyroIntervalResult;
  using phad::estimator::internal::GyroNoiseScales;
  using phad::estimator::internal::GyroNoiseScalesResult;
  using phad::estimator::internal::GyroRotationFactor;
  using phad::estimator::internal::preintegrateGyroInterval;
  using phad::estimator::internal::selectGyroBiasInitialValue;
  using phad::estimator::internal::StereoVoUpdateState;
  using phad::estimator::internal::StereoVoUpdateTransaction;
  using phad::estimator::internal::ValidatedGyroInterval;
  using phad::estimator::internal::validateGyroInterval;
  using phad::estimator::internal::WindowFrame;

  const Eigen::Vector3d kNonIdentityRotationVector{ 0.2, -0.1, 0.05 };
  const Eigen::Vector3d kNonIdentityTranslation{ 0.1, -0.02, 0.03 };

  [[nodiscard]] RectifiedStereoCalibration
  makeNonIdentityVisualCalibration()
  {
    Eigen::Isometry3d T_B_left = Eigen::Isometry3d::Identity();
    T_B_left.linear() =
        gtsam::Rot3::Expmap( kNonIdentityRotationVector ).matrix();
    T_B_left.translation() = kNonIdentityTranslation;
    auto rigid             = RigidTransform::create( T_B_left.matrix() ).value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480,
               std::move( rigid ) )
        .value();
  }

  [[nodiscard]] KeyframeMeasurement makeBodyStationaryMeasurement(
      const RectifiedStereoCalibration& calibration,
      std::size_t                       frame_index )
  {
    Eigen::Isometry3d T_B_left = Eigen::Isometry3d::Identity();
    T_B_left.linear()          = calibration.T_B_left_rectified().rotation();
    T_B_left.translation() =
        calibration.T_B_left_rectified().translation();

    KeyframeMeasurement measurement;
    measurement.timestamp = phad::common::Timestamp{
        static_cast<std::int64_t>( frame_index + 1U ) * 100'000'000 };
    measurement.observations.reserve( kVisualLandmarks.size() );
    for ( const VisualLandmark& landmark : kVisualLandmarks )
    {
      const Eigen::Vector3d point_left =
          T_B_left.inverse() * landmark.m_point_W;
      EXPECT_GT( point_left.z(), 0.0 );
      const double u_l = calibration.fxPixels() * point_left.x() /
                             point_left.z() +
                         calibration.cxPixels();
      const double v = calibration.fyPixels() * point_left.y() /
                           point_left.z() +
                       calibration.cyPixels();
      const double disparity = calibration.fxPixels() *
                               calibration.baselineM() /
                               point_left.z();
      measurement.observations.push_back(
          StereoObservation{ landmark.m_id, Eigen::Vector2d{ u_l, v },
                             disparity } );
    }
    return measurement;
  }

  [[nodiscard]] ImuMeasurement reducerSample(
      std::int64_t timestamp_ns, const Eigen::Vector3d& gyro )
  {
    return ImuMeasurement{
        .timestamp  = phad::common::Timestamp{ timestamp_ns },
        .accel_mps2 = { 0.0, 0.0, 0.0 },
        .gyro_radps = { gyro.x(), gyro.y(), gyro.z() },
    };
  }

  [[nodiscard]] GyroInterval lifecycleGyroInterval(
      std::int64_t t_prev_ns, std::int64_t t_curr_ns,
      const Eigen::Vector3d& gyro )
  {
    const std::int64_t t_mid_ns =
        t_prev_ns + ( t_curr_ns - t_prev_ns ) / 2;
    GyroInterval interval;
    interval.m_t_prev  = phad::common::Timestamp{ t_prev_ns };
    interval.m_samples = { reducerSample( t_prev_ns, gyro ),
                           reducerSample( t_mid_ns, gyro ),
                           reducerSample( t_curr_ns, gyro ) };
    interval.m_imu_gap = false;
    return interval;
  }

  [[nodiscard]] const ValidatedGyroInterval& requireValidated(
      const GyroIntervalResult& result )
  {
    EXPECT_TRUE( std::holds_alternative<ValidatedGyroInterval>( result ) );
    return std::get<ValidatedGyroInterval>( result );
  }

  GyroIntervalError requireIntervalError(
      const GyroIntervalResult& result, GyroIntervalErrorCode expected )
  {
    EXPECT_TRUE( std::holds_alternative<GyroIntervalError>( result ) );
    const auto& error = std::get<GyroIntervalError>( result );
    EXPECT_EQ( error.m_code, expected );
    return error;
  }

  [[nodiscard]] Eigen::Isometry3d transactionPose( double offset )
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear()          = Eigen::AngleAxisd(
                        0.01 * offset,
                        Eigen::Vector3d{ 1.0, 2.0, 3.0 }.normalized() )
                        .toRotationMatrix();
    pose.translation() = Eigen::Vector3d{ offset, -2.0 * offset,
                                          3.0 * offset };
    return pose;
  }

  [[nodiscard]] ValidatedGyroInterval transactionInterval(
      std::int64_t t_prev_ns, double offset )
  {
    const std::int64_t    t_mid_ns  = t_prev_ns + 25'000'000;
    const std::int64_t    t_curr_ns = t_prev_ns + 100'000'000;
    const Eigen::Vector3d first{ offset, offset + 1.0, offset + 2.0 };
    const Eigen::Vector3d middle{ offset + 3.0, offset + 4.0,
                                  offset + 5.0 };
    const Eigen::Vector3d last{ offset + 6.0, offset + 7.0, offset + 8.0 };
    const std::array      samples{
        reducerSample( t_prev_ns, first ), reducerSample( t_mid_ns, middle ),
        reducerSample( t_curr_ns, last ) };
    const GyroIntervalResult result = validateGyroInterval( samples );
    EXPECT_TRUE( std::holds_alternative<ValidatedGyroInterval>( result ) );
    return std::get<ValidatedGyroInterval>( result );
  }

  [[nodiscard]] StereoObservation transactionObservation(
      LandmarkId id, double offset )
  {
    return StereoObservation{ id, Eigen::Vector2d{ offset, -offset },
                              offset + 0.5 };
  }

  [[nodiscard]] std::unique_ptr<StereoVoUpdateState> transactionState(
      std::uint64_t salt )
  {
    auto state = std::make_unique<StereoVoUpdateState>();

    const std::uint64_t root_index = 10U * salt + 1U;
    WindowFrame         first;
    first.m_frame_index = root_index;
    first.m_timestamp   = phad::common::Timestamp{
        static_cast<std::int64_t>( root_index ) * 100'000'000 };
    first.m_T_W_B        = transactionPose( static_cast<double>( salt ) + 0.25 );
    first.m_observations = {
        transactionObservation( 100U + salt, 0.25 + static_cast<double>( salt ) ),
        transactionObservation( 200U + salt, 0.5 + static_cast<double>( salt ) ) };
    first.m_is_keyframe = ( salt % 2U ) == 0U;

    WindowFrame middle;
    middle.m_frame_index = root_index + 1U;
    middle.m_timestamp   = phad::common::Timestamp{
        first.m_timestamp.nanoseconds() + 100'000'000 };
    middle.m_T_W_B        = transactionPose( static_cast<double>( salt ) + 1.25 );
    middle.m_observations = {
        transactionObservation( 300U + salt, 1.25 + static_cast<double>( salt ) ) };
    middle.m_is_keyframe = !first.m_is_keyframe;

    WindowFrame tail;
    tail.m_frame_index = root_index + 2U;
    tail.m_timestamp   = phad::common::Timestamp{
        middle.m_timestamp.nanoseconds() + 100'000'000 };
    tail.m_T_W_B        = transactionPose( static_cast<double>( salt ) + 2.25 );
    tail.m_observations = {
        transactionObservation( 400U + salt, 2.25 + static_cast<double>( salt ) ) };
    tail.m_is_keyframe = first.m_is_keyframe;

    const auto make_root = [ salt ]() {
      GyroFrameState gyro;
      gyro.m_bias_radps =
          Eigen::Vector3d{ 0.01, -0.02, 0.03 } *
          static_cast<double>( salt );
      gyro.m_segment_id   = static_cast<std::uint32_t>( salt + 2U );
      gyro.m_component_id = salt + 3U;
      return gyro;
    };
    const auto make_link = [ salt ]( const WindowFrame& predecessor ) {
      GyroFrameState gyro;
      gyro.m_bias_radps =
          Eigen::Vector3d{ -0.04, 0.05, -0.06 } *
          static_cast<double>( salt );
      gyro.m_segment_id              = static_cast<std::uint32_t>( salt + 4U );
      gyro.m_component_id            = salt + 3U;
      gyro.m_predecessor_frame_index = predecessor.m_frame_index;
      gyro.m_interval                = transactionInterval(
          predecessor.m_timestamp.nanoseconds(),
          static_cast<double>( salt ) );
      return gyro;
    };
    if ( ( salt % 2U ) == 0U )
    {
      middle.m_gyro = make_root();
      tail.m_gyro   = make_link( middle );
    }
    else
    {
      first.m_gyro  = make_root();
      middle.m_gyro = make_link( first );
    }

    state->m_window.push_back( first );
    state->m_window.push_back( middle );
    state->m_window.push_back( tail );
    state->m_landmarks_w.emplace(
        500U + salt,
        Eigen::Vector3d{ static_cast<double>( salt ),
                         static_cast<double>( salt ) + 1.0,
                         static_cast<double>( salt ) + 2.0 } );
    state->m_track_times.emplace(
        600U + salt,
        std::vector<phad::common::Timestamp>{
            phad::common::Timestamp{ static_cast<std::int64_t>( salt ) },
            phad::common::Timestamp{ static_cast<std::int64_t>( salt + 1U ) } } );
    state->m_T_W_B_last_stereo.emplace(
        700U + salt, transactionPose( static_cast<double>( salt ) + 3.25 ) );
    if ( ( salt % 2U ) == 0U )
    {
      state->m_T_W_B_last_accepted =
          transactionPose( static_cast<double>( salt ) + 4.25 );
    }
    else
    {
      state->m_T_W_B_prev_accepted =
          transactionPose( static_cast<double>( salt ) + 5.25 );
    }
    state->m_next_frame_index = root_index + 3U;
    state->m_initialized      = ( salt % 2U ) == 0U;
    state->m_segment_id       = static_cast<std::uint32_t>( salt + 6U );
    state->m_culled_ids.insert( 800U + salt );
    state->m_pending_seed_obs.emplace(
        900U + salt,
        transactionObservation( 900U + salt,
                                static_cast<double>( salt ) + 6.25 ) );
    if ( ( salt % 2U ) == 0U )
    {
      state->m_eligible_visual_rejected_timestamp =
          phad::common::Timestamp{ static_cast<std::int64_t>( salt + 7U ) };
    }
    state->m_next_gyro_component_id = salt + 8U;
    return state;
  }

  void expectVectorExact( const Eigen::Vector3d& actual,
                          const Eigen::Vector3d& expected )
  {
    EXPECT_TRUE( ( actual.array() == expected.array() ).all() );
  }

  void expectPoseExact( const Eigen::Isometry3d& actual,
                        const Eigen::Isometry3d& expected )
  {
    EXPECT_TRUE( ( actual.matrix().array() == expected.matrix().array() ).all() );
  }

  void expectObservationExact( const StereoObservation& actual,
                               const StereoObservation& expected )
  {
    EXPECT_EQ( actual.id, expected.id );
    EXPECT_TRUE( ( actual.left_pixel.array() == expected.left_pixel.array() ).all() );
    EXPECT_EQ( actual.disparity_px, expected.disparity_px );
  }

  void expectIntervalExact( const ValidatedGyroInterval& actual,
                            const ValidatedGyroInterval& expected )
  {
    ASSERT_EQ( actual.m_samples.size(), expected.m_samples.size() );
    for ( std::size_t index = 0; index < actual.m_samples.size(); ++index )
    {
      EXPECT_EQ( actual.m_samples[ index ].timestamp,
                 expected.m_samples[ index ].timestamp );
      EXPECT_EQ( actual.m_samples[ index ].accel_mps2,
                 expected.m_samples[ index ].accel_mps2 );
      EXPECT_EQ( actual.m_samples[ index ].gyro_radps,
                 expected.m_samples[ index ].gyro_radps );
    }
    ASSERT_EQ( actual.m_steps.size(), expected.m_steps.size() );
    for ( std::size_t index = 0; index < actual.m_steps.size(); ++index )
    {
      expectVectorExact( actual.m_steps[ index ].m_omega_mean_radps,
                         expected.m_steps[ index ].m_omega_mean_radps );
      EXPECT_EQ( actual.m_steps[ index ].m_dt_ns,
                 expected.m_steps[ index ].m_dt_ns );
      EXPECT_EQ( actual.m_steps[ index ].m_dt_s,
                 expected.m_steps[ index ].m_dt_s );
    }
    EXPECT_EQ( actual.m_t_prev, expected.m_t_prev );
    EXPECT_EQ( actual.m_t_curr, expected.m_t_curr );
    EXPECT_EQ( actual.m_duration_ns, expected.m_duration_ns );
  }

  void expectWindowFrameExact( const WindowFrame& actual,
                               const WindowFrame& expected )
  {
    EXPECT_EQ( actual.m_frame_index, expected.m_frame_index );
    EXPECT_EQ( actual.m_timestamp, expected.m_timestamp );
    expectPoseExact( actual.m_T_W_B, expected.m_T_W_B );
    ASSERT_EQ( actual.m_observations.size(), expected.m_observations.size() );
    for ( std::size_t index = 0; index < actual.m_observations.size(); ++index )
    {
      expectObservationExact( actual.m_observations[ index ],
                              expected.m_observations[ index ] );
    }
    EXPECT_EQ( actual.m_is_keyframe, expected.m_is_keyframe );
    ASSERT_EQ( actual.m_gyro.has_value(), expected.m_gyro.has_value() );
    if ( actual.m_gyro.has_value() )
    {
      expectVectorExact( actual.m_gyro->m_bias_radps,
                         expected.m_gyro->m_bias_radps );
      EXPECT_EQ( actual.m_gyro->m_segment_id,
                 expected.m_gyro->m_segment_id );
      EXPECT_EQ( actual.m_gyro->m_component_id,
                 expected.m_gyro->m_component_id );
      EXPECT_EQ( actual.m_gyro->m_predecessor_frame_index,
                 expected.m_gyro->m_predecessor_frame_index );
      ASSERT_EQ( actual.m_gyro->m_interval.has_value(),
                 expected.m_gyro->m_interval.has_value() );
      if ( actual.m_gyro->m_interval.has_value() )
      {
        expectIntervalExact( *actual.m_gyro->m_interval,
                             *expected.m_gyro->m_interval );
      }
    }
  }

  void expectStateExact( const StereoVoUpdateState& actual,
                         const StereoVoUpdateState& expected )
  {
    ASSERT_EQ( actual.m_window.size(), expected.m_window.size() );
    for ( std::size_t index = 0; index < actual.m_window.size(); ++index )
    {
      expectWindowFrameExact( actual.m_window[ index ],
                              expected.m_window[ index ] );
    }

    ASSERT_EQ( actual.m_landmarks_w.size(), expected.m_landmarks_w.size() );
    for ( const auto& [ id, point ] : expected.m_landmarks_w )
    {
      const auto it = actual.m_landmarks_w.find( id );
      ASSERT_NE( it, actual.m_landmarks_w.end() );
      expectVectorExact( it->second, point );
    }
    EXPECT_EQ( actual.m_track_times, expected.m_track_times );

    ASSERT_EQ( actual.m_T_W_B_last_stereo.size(),
               expected.m_T_W_B_last_stereo.size() );
    for ( const auto& [ id, pose ] : expected.m_T_W_B_last_stereo )
    {
      const auto it = actual.m_T_W_B_last_stereo.find( id );
      ASSERT_NE( it, actual.m_T_W_B_last_stereo.end() );
      expectPoseExact( it->second, pose );
    }

    ASSERT_EQ( actual.m_T_W_B_last_accepted.has_value(),
               expected.m_T_W_B_last_accepted.has_value() );
    if ( actual.m_T_W_B_last_accepted.has_value() )
    {
      expectPoseExact( *actual.m_T_W_B_last_accepted,
                       *expected.m_T_W_B_last_accepted );
    }
    ASSERT_EQ( actual.m_T_W_B_prev_accepted.has_value(),
               expected.m_T_W_B_prev_accepted.has_value() );
    if ( actual.m_T_W_B_prev_accepted.has_value() )
    {
      expectPoseExact( *actual.m_T_W_B_prev_accepted,
                       *expected.m_T_W_B_prev_accepted );
    }

    EXPECT_EQ( actual.m_next_frame_index, expected.m_next_frame_index );
    EXPECT_EQ( actual.m_initialized, expected.m_initialized );
    EXPECT_EQ( actual.m_segment_id, expected.m_segment_id );
    EXPECT_EQ( actual.m_culled_ids, expected.m_culled_ids );

    ASSERT_EQ( actual.m_pending_seed_obs.size(),
               expected.m_pending_seed_obs.size() );
    for ( const auto& [ id, observation ] : expected.m_pending_seed_obs )
    {
      const auto it = actual.m_pending_seed_obs.find( id );
      ASSERT_NE( it, actual.m_pending_seed_obs.end() );
      expectObservationExact( it->second, observation );
    }
    EXPECT_EQ( actual.m_eligible_visual_rejected_timestamp,
               expected.m_eligible_visual_rejected_timestamp );
    EXPECT_EQ( actual.m_next_gyro_component_id,
               expected.m_next_gyro_component_id );
  }

  void expectGyroDiagnosticsExact( const GyroDiagnostics& actual,
                                   const GyroDiagnostics& expected )
  {
    ASSERT_EQ( actual.m_bias_radps.has_value(),
               expected.m_bias_radps.has_value() );
    if ( actual.m_bias_radps.has_value() )
    {
      expectVectorExact( *actual.m_bias_radps, *expected.m_bias_radps );
    }
    ASSERT_EQ( actual.m_window_biases.size(),
               expected.m_window_biases.size() );
    for ( std::size_t index = 0; index < actual.m_window_biases.size();
          ++index )
    {
      const auto& actual_bias   = actual.m_window_biases[ index ];
      const auto& expected_bias = expected.m_window_biases[ index ];
      EXPECT_EQ( actual_bias.m_frame_index, expected_bias.m_frame_index );
      EXPECT_EQ( actual_bias.m_timestamp, expected_bias.m_timestamp );
      ASSERT_EQ( actual_bias.m_bias_radps.has_value(),
                 expected_bias.m_bias_radps.has_value() );
      if ( actual_bias.m_bias_radps.has_value() )
      {
        expectVectorExact( *actual_bias.m_bias_radps,
                           *expected_bias.m_bias_radps );
      }
    }
    EXPECT_EQ( actual.m_rotation_factors, expected.m_rotation_factors );
    EXPECT_EQ( actual.m_rw_factors, expected.m_rw_factors );
    EXPECT_EQ( actual.m_root_priors, expected.m_root_priors );
    EXPECT_EQ( actual.m_relinearization_rounds,
               expected.m_relinearization_rounds );
    EXPECT_EQ( actual.m_break_reason, expected.m_break_reason );
  }

  void appendOptionalVector( std::string&                          bytes,
                             const std::optional<Eigen::Vector3d>& value )
  {
    appendBool( bytes, value.has_value() );
    if ( value.has_value() )
    {
      appendDouble( bytes, value->x() );
      appendDouble( bytes, value->y() );
      appendDouble( bytes, value->z() );
    }
  }

  [[nodiscard]] std::string encodeOnlineBiasResults(
      const std::vector<VioUpdateResult>& results )
  {
    std::string bytes = encodeVisualResults( results );
    for ( const VioUpdateResult& result : results )
    {
      appendBool( bytes, result.diagnostics.m_gyro.has_value() );
      if ( !result.diagnostics.m_gyro.has_value() )
      {
        continue;
      }
      const GyroDiagnostics& gyro = *result.diagnostics.m_gyro;
      appendOptionalVector( bytes, gyro.m_bias_radps );
      appendU64( bytes,
                 static_cast<std::uint64_t>( gyro.m_window_biases.size() ) );
      for ( const auto& window_bias : gyro.m_window_biases )
      {
        appendU64( bytes, window_bias.m_frame_index );
        appendI64( bytes, window_bias.m_timestamp.nanoseconds() );
        appendOptionalVector( bytes, window_bias.m_bias_radps );
      }
      appendU32( bytes, gyro.m_rotation_factors );
      appendU32( bytes, gyro.m_rw_factors );
      appendU32( bytes, gyro.m_root_priors );
      appendU32( bytes, gyro.m_relinearization_rounds );
      appendU8( bytes, static_cast<std::uint8_t>( gyro.m_break_reason ) );
    }
    return bytes;
  }

}  // namespace

TEST( StereoVoUpdateTransaction, ScopeExitRestoresEveryField )
{
  auto                       owner           = transactionState( 2U );
  auto                       expected_after  = transactionState( 3U );
  const StereoVoUpdateState  expected_before = *owner;
  const StereoVoUpdateState* owner_before    = owner.get();

  {
    StereoVoUpdateTransaction transaction( owner );
    EXPECT_EQ( owner.get(), owner_before );
    *owner = *expected_after;
  }

  EXPECT_NE( owner.get(), owner_before );
  expectStateExact( *owner, expected_before );
}

TEST( StereoVoUpdateTransaction, ExceptionUnwindRestoresEveryField )
{
  auto                       owner           = transactionState( 4U );
  auto                       expected_after  = transactionState( 5U );
  const StereoVoUpdateState  expected_before = *owner;
  const StereoVoUpdateState* owner_before    = owner.get();

  EXPECT_THROW(
      {
        StereoVoUpdateTransaction transaction( owner );
        EXPECT_EQ( owner.get(), owner_before );
        *owner = *expected_after;
        throw std::runtime_error( "transaction unwind" );
      },
      std::runtime_error );

  EXPECT_NE( owner.get(), owner_before );
  expectStateExact( *owner, expected_before );
}

TEST( StereoVoUpdateTransaction, ExplicitRollbackRestoresEveryField )
{
  auto                       owner           = transactionState( 6U );
  auto                       expected_after  = transactionState( 7U );
  const StereoVoUpdateState  expected_before = *owner;
  const StereoVoUpdateState* owner_before    = owner.get();

  StereoVoUpdateTransaction transaction( owner );
  EXPECT_EQ( owner.get(), owner_before );
  *owner = *expected_after;
  transaction.rollback();
  transaction.rollback();

  EXPECT_NE( owner.get(), owner_before );
  expectStateExact( *owner, expected_before );
}

TEST( StereoVoUpdateTransaction, CommitPublishesEveryField )
{
  auto                       owner          = transactionState( 8U );
  auto                       expected_after = transactionState( 9U );
  const StereoVoUpdateState* owner_before   = owner.get();

  {
    StereoVoUpdateTransaction transaction( owner );
    EXPECT_EQ( owner.get(), owner_before );
    *owner = *expected_after;
    transaction.commit();
    transaction.commit();
    EXPECT_EQ( owner.get(), owner_before );
  }

  EXPECT_EQ( owner.get(), owner_before );
  expectStateExact( *owner, *expected_after );
}

TEST( StereoVoOnlineGyroBias, RejectedEndpointStartsIndependentComponent )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  const VioUpdateResult root =
      estimator.update( makeVisualMeasurement( calibration, 0U ), true );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;
  ASSERT_TRUE( root.diagnostics.m_gyro.has_value() );
  EXPECT_EQ( root.diagnostics.m_gyro->m_root_priors, 1U );

  KeyframeMeasurement rejected = makeVisualMeasurement( calibration, 1U );
  rejected.m_gyro_interval     = makeGyroInterval( 1U );
  for ( StereoObservation& observation : rejected.observations )
  {
    observation.id += 10'000U;
  }
  const VioUpdateResult rejected_result = estimator.update( rejected, true );
  EXPECT_EQ( rejected_result.status, UpdateStatus::kRejected );
  EXPECT_EQ( rejected_result.message, "zero shared landmarks with window" );

  KeyframeMeasurement after_rejected =
      makeVisualMeasurement( calibration, 2U );
  after_rejected.m_gyro_interval = makeGyroInterval( 2U );
  const VioUpdateResult broken   = estimator.update( after_rejected, false );
  ASSERT_EQ( broken.status, UpdateStatus::kOk ) << broken.message;
  ASSERT_TRUE( broken.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& broken_gyro = *broken.diagnostics.m_gyro;
  EXPECT_EQ( broken_gyro.m_break_reason,
             GyroBreakReason::kRejectedEndpoint );
  EXPECT_EQ( broken_gyro.m_rotation_factors, 0U );
  EXPECT_EQ( broken_gyro.m_rw_factors, 0U );
  EXPECT_EQ( broken_gyro.m_root_priors, 2U );
  ASSERT_EQ( broken_gyro.m_window_biases.size(), 2U );
  EXPECT_EQ( broken_gyro.m_window_biases[ 0 ].m_frame_index, 0U );
  EXPECT_EQ( broken_gyro.m_window_biases[ 0 ].m_timestamp.nanoseconds(),
             100'000'000 );
  EXPECT_FALSE( broken_gyro.m_window_biases[ 0 ].m_bias_radps.has_value() );
  EXPECT_EQ( broken_gyro.m_window_biases[ 1 ].m_frame_index, 1U );
  EXPECT_EQ( broken_gyro.m_window_biases[ 1 ].m_timestamp.nanoseconds(),
             300'000'000 );
  EXPECT_FALSE( broken_gyro.m_window_biases[ 1 ].m_bias_radps.has_value() );
  EXPECT_FALSE( broken_gyro.m_bias_radps.has_value() );

  KeyframeMeasurement exact    = makeVisualMeasurement( calibration, 3U );
  exact.m_gyro_interval        = makeGyroInterval( 3U );
  const VioUpdateResult linked = estimator.update( exact, false );
  ASSERT_EQ( linked.status, UpdateStatus::kOk ) << linked.message;
  ASSERT_TRUE( linked.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& linked_gyro = *linked.diagnostics.m_gyro;
  EXPECT_EQ( linked_gyro.m_break_reason, GyroBreakReason::kNone );
  EXPECT_EQ( linked_gyro.m_rotation_factors, 1U );
  EXPECT_EQ( linked_gyro.m_rw_factors, 1U );
  EXPECT_EQ( linked_gyro.m_root_priors, 2U );
  ASSERT_EQ( linked_gyro.m_window_biases.size(), 3U );
  EXPECT_FALSE( linked_gyro.m_window_biases[ 0 ].m_bias_radps.has_value() );
  EXPECT_TRUE( linked_gyro.m_window_biases[ 1 ].m_bias_radps.has_value() );
  EXPECT_TRUE( linked_gyro.m_window_biases[ 2 ].m_bias_radps.has_value() );
  ASSERT_TRUE( linked_gyro.m_bias_radps.has_value() );
  EXPECT_TRUE(
      ( linked_gyro.m_bias_radps->array() ==
        linked_gyro.m_window_biases.back().m_bias_radps->array() )
          .all() );
}

TEST( StereoVoOnlineGyroBias,
      LowSharedRejectedEndpointStartsIndependentComponent )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.enable_reanchor                      = true;
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  const VioUpdateResult root =
      estimator.update( makeVisualMeasurement( calibration, 0U ), true );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  KeyframeMeasurement rejected = makeVisualMeasurement( calibration, 1U );
  rejected.observations.resize( 5U );
  rejected.m_gyro_interval              = makeGyroInterval( 1U );
  const VioUpdateResult rejected_result = estimator.update( rejected, false );
  EXPECT_EQ( rejected_result.status, UpdateStatus::kRejected );
  EXPECT_EQ( rejected_result.message,
             "insufficient shared landmarks (non-keyframe)" );

  KeyframeMeasurement after_rejected =
      makeVisualMeasurement( calibration, 2U );
  after_rejected.m_gyro_interval = makeGyroInterval( 2U );
  const VioUpdateResult broken   = estimator.update( after_rejected, false );
  ASSERT_EQ( broken.status, UpdateStatus::kOk ) << broken.message;
  ASSERT_TRUE( broken.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& gyro = *broken.diagnostics.m_gyro;
  EXPECT_EQ( gyro.m_break_reason, GyroBreakReason::kRejectedEndpoint );
  EXPECT_EQ( gyro.m_rotation_factors, 0U );
  EXPECT_EQ( gyro.m_rw_factors, 0U );
  EXPECT_EQ( gyro.m_root_priors, 2U );
}

TEST( StereoVoOnlineGyroBias, FirstStateRejectsNonGapInterval )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  KeyframeMeasurement first = makeVisualMeasurement( calibration, 0U );
  first.m_gyro_interval     = lifecycleGyroInterval(
      0, first.timestamp.nanoseconds(), kBiasBase );
  const VioUpdateResult result = estimator.update( first, true );

  EXPECT_EQ( result.status, UpdateStatus::kRejected );
  EXPECT_EQ( result.message,
             "gyro interval endpoints do not match pose timestamps" );
  EXPECT_FALSE( result.estimate.has_value() );
  ASSERT_TRUE( result.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& gyro = *result.diagnostics.m_gyro;
  EXPECT_FALSE( gyro.m_bias_radps.has_value() );
  EXPECT_TRUE( gyro.m_window_biases.empty() );
  EXPECT_EQ( gyro.m_rotation_factors, 0U );
  EXPECT_EQ( gyro.m_rw_factors, 0U );
  EXPECT_EQ( gyro.m_root_priors, 0U );
  EXPECT_EQ( gyro.m_relinearization_rounds, 0U );
  EXPECT_EQ( gyro.m_break_reason, GyroBreakReason::kNone );
}

TEST( StereoVoOnlineGyroBias, InteriorEvictionBreaksLinkWithoutCrossing )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.window_size                          = 3;
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  VioUpdateResult result;
  for ( std::size_t frame_index = 0; frame_index < 4U; ++frame_index )
  {
    KeyframeMeasurement measurement =
        makeVisualMeasurement( calibration, frame_index );
    if ( frame_index > 0U )
    {
      measurement.m_gyro_interval = makeGyroInterval( frame_index );
    }
    result = estimator.update( measurement, frame_index < 2U );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  ASSERT_TRUE( result.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& gyro = *result.diagnostics.m_gyro;
  EXPECT_EQ( result.diagnostics.window_size, 3U );
  EXPECT_EQ( gyro.m_break_reason, GyroBreakReason::kEvictedEndpoint );
  EXPECT_EQ( gyro.m_rotation_factors, 1U );
  EXPECT_EQ( gyro.m_rw_factors, 1U );
  EXPECT_EQ( gyro.m_root_priors, 2U );
  ASSERT_EQ( gyro.m_window_biases.size(), 3U );
  EXPECT_EQ( gyro.m_window_biases[ 0 ].m_frame_index, 0U );
  EXPECT_EQ( gyro.m_window_biases[ 1 ].m_frame_index, 1U );
  EXPECT_EQ( gyro.m_window_biases[ 2 ].m_frame_index, 3U );
  EXPECT_TRUE( gyro.m_window_biases[ 0 ].m_bias_radps.has_value() );
  EXPECT_TRUE( gyro.m_window_biases[ 1 ].m_bias_radps.has_value() );
  EXPECT_FALSE( gyro.m_window_biases[ 2 ].m_bias_radps.has_value() );
  EXPECT_FALSE( gyro.m_bias_radps.has_value() );
}

TEST( StereoVoOnlineGyroBias,
      UnknownPredecessorRollsBackBeforeDiagnostics )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator subject( calibration, options );
  StereoVoEstimator control( calibration, options );

  const KeyframeMeasurement root         = makeVisualMeasurement( calibration, 0U );
  const VioUpdateResult     subject_root = subject.update( root, true );
  const VioUpdateResult     control_root = control.update( root, true );
  ASSERT_EQ( subject_root.status, UpdateStatus::kOk ) << subject_root.message;
  ASSERT_EQ( control_root.status, UpdateStatus::kOk ) << control_root.message;

  KeyframeMeasurement unknown = makeVisualMeasurement( calibration, 1U );
  unknown.m_gyro_interval     = lifecycleGyroInterval(
      50'000'000, unknown.timestamp.nanoseconds(), kBiasBase );
  const VioUpdateResult rejected = subject.update( unknown, false );
  EXPECT_EQ( rejected.status, UpdateStatus::kRejected );
  EXPECT_EQ( rejected.message,
             "gyro interval endpoints do not match pose timestamps" );
  EXPECT_FALSE( rejected.estimate.has_value() );
  ASSERT_TRUE( rejected.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& rejected_gyro = *rejected.diagnostics.m_gyro;
  EXPECT_FALSE( rejected_gyro.m_bias_radps.has_value() );
  EXPECT_EQ( rejected_gyro.m_relinearization_rounds, 0U );
  EXPECT_EQ( rejected_gyro.m_break_reason, GyroBreakReason::kNone );
  EXPECT_EQ( rejected_gyro.m_rotation_factors, 0U );
  EXPECT_EQ( rejected_gyro.m_rw_factors, 0U );
  EXPECT_EQ( rejected_gyro.m_root_priors, 1U );
  ASSERT_EQ( rejected_gyro.m_window_biases.size(), 1U );
  EXPECT_EQ( rejected_gyro.m_window_biases.front().m_frame_index, 0U );
  EXPECT_EQ( rejected_gyro.m_window_biases.front().m_timestamp,
             root.timestamp );
  EXPECT_FALSE(
      rejected_gyro.m_window_biases.front().m_bias_radps.has_value() );

  KeyframeMeasurement exact = makeVisualMeasurement( calibration, 2U );
  exact.m_gyro_interval     = lifecycleGyroInterval(
      root.timestamp.nanoseconds(), exact.timestamp.nanoseconds(),
      kBiasBase );
  const VioUpdateResult subject_after = subject.update( exact, false );
  const VioUpdateResult control_after = control.update( exact, false );
  ASSERT_EQ( subject_after.status, UpdateStatus::kOk )
      << subject_after.message;
  ASSERT_EQ( control_after.status, UpdateStatus::kOk )
      << control_after.message;
  EXPECT_EQ( encodeVisualResults( { subject_after } ),
             encodeVisualResults( { control_after } ) );
  ASSERT_TRUE( subject_after.diagnostics.m_gyro.has_value() );
  ASSERT_TRUE( control_after.diagnostics.m_gyro.has_value() );
  expectGyroDiagnosticsExact( *subject_after.diagnostics.m_gyro,
                              *control_after.diagnostics.m_gyro );
}

TEST( StereoVoOnlineGyroBias,
      MalformedIntervalRejectsWithoutStateMutation )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator subject( calibration, options );
  StereoVoEstimator control( calibration, options );

  const KeyframeMeasurement root = makeVisualMeasurement( calibration, 0U );
  ASSERT_EQ( subject.update( root, true ).status, UpdateStatus::kOk );
  ASSERT_EQ( control.update( root, true ).status, UpdateStatus::kOk );

  KeyframeMeasurement malformed = makeVisualMeasurement( calibration, 1U );
  GyroInterval        interval;
  interval.m_t_prev  = root.timestamp;
  interval.m_samples = {
      reducerSample( root.timestamp.nanoseconds(), kBiasBase ) };
  malformed.m_gyro_interval      = std::move( interval );
  const VioUpdateResult rejected = subject.update( malformed, false );
  EXPECT_EQ( rejected.status, UpdateStatus::kRejected );
  EXPECT_EQ( rejected.message,
             "gyro interval requires at least two samples" );
  EXPECT_FALSE( rejected.estimate.has_value() );
  ASSERT_TRUE( rejected.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& rejected_gyro = *rejected.diagnostics.m_gyro;
  EXPECT_FALSE( rejected_gyro.m_bias_radps.has_value() );
  EXPECT_EQ( rejected_gyro.m_relinearization_rounds, 0U );
  EXPECT_EQ( rejected_gyro.m_break_reason, GyroBreakReason::kNone );
  EXPECT_EQ( rejected_gyro.m_rotation_factors, 0U );
  EXPECT_EQ( rejected_gyro.m_rw_factors, 0U );
  EXPECT_EQ( rejected_gyro.m_root_priors, 1U );
  ASSERT_EQ( rejected_gyro.m_window_biases.size(), 1U );
  EXPECT_EQ( rejected_gyro.m_window_biases.front().m_frame_index, 0U );
  EXPECT_EQ( rejected_gyro.m_window_biases.front().m_timestamp,
             root.timestamp );
  EXPECT_FALSE(
      rejected_gyro.m_window_biases.front().m_bias_radps.has_value() );

  KeyframeMeasurement exact = makeVisualMeasurement( calibration, 2U );
  exact.m_gyro_interval     = lifecycleGyroInterval(
      root.timestamp.nanoseconds(), exact.timestamp.nanoseconds(),
      kBiasBase );
  const VioUpdateResult subject_after = subject.update( exact, false );
  const VioUpdateResult control_after = control.update( exact, false );
  ASSERT_EQ( subject_after.status, UpdateStatus::kOk )
      << subject_after.message;
  ASSERT_EQ( control_after.status, UpdateStatus::kOk )
      << control_after.message;
  EXPECT_EQ( encodeVisualResults( { subject_after } ),
             encodeVisualResults( { control_after } ) );
  ASSERT_TRUE( subject_after.diagnostics.m_gyro.has_value() );
  ASSERT_TRUE( control_after.diagnostics.m_gyro.has_value() );
  expectGyroDiagnosticsExact( *subject_after.diagnostics.m_gyro,
                              *control_after.diagnostics.m_gyro );
}

TEST( StereoVoOnlineGyroBias,
      HardResultsHideScalarAndExposeCommittedTopology )
{
  enum class FailureKind
  {
    kBasic,
    kMalformedGyro,
    kVisualSupport,
  };

  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  for ( const FailureKind kind : { FailureKind::kBasic,
                                   FailureKind::kMalformedGyro,
                                   FailureKind::kVisualSupport } )
  {
    EstimatorOptions options = makeVisualOptions();
    options.m_gyro_bias      = makeGyroOptions();
    StereoVoEstimator estimator( calibration, options );

    const KeyframeMeasurement root =
        makeVisualMeasurement( calibration, 0U );
    const VioUpdateResult root_result = estimator.update( root, true );
    ASSERT_EQ( root_result.status, UpdateStatus::kOk )
        << root_result.message;

    KeyframeMeasurement linked =
        makeVisualMeasurement( calibration, 1U );
    linked.m_gyro_interval              = makeGyroInterval( 1U );
    const VioUpdateResult linked_result = estimator.update( linked, true );
    ASSERT_EQ( linked_result.status, UpdateStatus::kOk )
        << linked_result.message;
    ASSERT_TRUE( linked_result.diagnostics.m_gyro.has_value() );
    ASSERT_TRUE(
        linked_result.diagnostics.m_gyro->m_bias_radps.has_value() );

    KeyframeMeasurement rejected =
        makeVisualMeasurement( calibration, 2U );
    rejected.m_gyro_interval = makeGyroInterval( 2U );
    if ( kind == FailureKind::kBasic )
    {
      rejected.observations.clear();
    }
    else if ( kind == FailureKind::kMalformedGyro )
    {
      rejected.m_gyro_interval->m_samples.resize( 1U );
    }
    else
    {
      for ( StereoObservation& observation : rejected.observations )
      {
        observation.id += 10'000U;
      }
    }

    const VioUpdateResult rejected_result =
        estimator.update( rejected, false );
    EXPECT_EQ( rejected_result.status, UpdateStatus::kRejected );
    EXPECT_FALSE( rejected_result.estimate.has_value() );
    ASSERT_TRUE( rejected_result.diagnostics.m_gyro.has_value() );

    GyroDiagnostics expected = *linked_result.diagnostics.m_gyro;
    expected.m_bias_radps.reset();
    expected.m_relinearization_rounds = 0U;
    expected.m_break_reason           = GyroBreakReason::kNone;
    expectGyroDiagnosticsExact( *rejected_result.diagnostics.m_gyro,
                                expected );
  }
}

TEST( StereoVoOnlineGyroBias, MissingIntervalStartsIndependentComponent )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  ASSERT_EQ( estimator.update( makeVisualMeasurement( calibration, 0U ), true )
                 .status,
             UpdateStatus::kOk );
  const VioUpdateResult missing =
      estimator.update( makeVisualMeasurement( calibration, 1U ), false );
  ASSERT_EQ( missing.status, UpdateStatus::kOk ) << missing.message;
  ASSERT_TRUE( missing.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& missing_gyro = *missing.diagnostics.m_gyro;
  EXPECT_EQ( missing_gyro.m_break_reason,
             GyroBreakReason::kMissingInterval );
  EXPECT_EQ( missing_gyro.m_rotation_factors, 0U );
  EXPECT_EQ( missing_gyro.m_rw_factors, 0U );
  EXPECT_EQ( missing_gyro.m_root_priors, 2U );
  ASSERT_EQ( missing_gyro.m_window_biases.size(), 2U );
  EXPECT_FALSE( missing_gyro.m_window_biases[ 0 ].m_bias_radps.has_value() );
  EXPECT_FALSE( missing_gyro.m_window_biases[ 1 ].m_bias_radps.has_value() );

  KeyframeMeasurement exact    = makeVisualMeasurement( calibration, 2U );
  exact.m_gyro_interval        = makeGyroInterval( 2U );
  const VioUpdateResult linked = estimator.update( exact, false );
  ASSERT_EQ( linked.status, UpdateStatus::kOk ) << linked.message;
  ASSERT_TRUE( linked.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& linked_gyro = *linked.diagnostics.m_gyro;
  EXPECT_EQ( linked_gyro.m_break_reason, GyroBreakReason::kNone );
  EXPECT_EQ( linked_gyro.m_rotation_factors, 1U );
  EXPECT_EQ( linked_gyro.m_rw_factors, 1U );
  EXPECT_EQ( linked_gyro.m_root_priors, 2U );
  ASSERT_EQ( linked_gyro.m_window_biases.size(), 3U );
  EXPECT_FALSE( linked_gyro.m_window_biases[ 0 ].m_bias_radps.has_value() );
  EXPECT_TRUE( linked_gyro.m_window_biases[ 1 ].m_bias_radps.has_value() );
  EXPECT_TRUE( linked_gyro.m_window_biases[ 2 ].m_bias_radps.has_value() );
}

TEST( StereoVoOnlineGyroBias, DeclaredGapIgnoresPayloadAndStartsComponent )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  GyroInterval gap;
  gap.m_t_prev = phad::common::Timestamp{
      std::numeric_limits<std::int64_t>::max() };
  gap.m_samples = {
      reducerSample( std::numeric_limits<std::int64_t>::max(),
                     Eigen::Vector3d{
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(), 0.0 } ) };
  gap.m_imu_gap = true;

  KeyframeMeasurement first  = makeVisualMeasurement( calibration, 0U );
  first.m_gyro_interval      = gap;
  const VioUpdateResult root = estimator.update( first, true );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;
  ASSERT_TRUE( root.diagnostics.m_gyro.has_value() );
  EXPECT_EQ( root.diagnostics.m_gyro->m_break_reason,
             GyroBreakReason::kNone );
  EXPECT_EQ( root.diagnostics.m_gyro->m_root_priors, 1U );

  KeyframeMeasurement second   = makeVisualMeasurement( calibration, 1U );
  second.m_gyro_interval       = gap;
  const VioUpdateResult broken = estimator.update( second, false );
  ASSERT_EQ( broken.status, UpdateStatus::kOk ) << broken.message;
  ASSERT_TRUE( broken.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& broken_gyro = *broken.diagnostics.m_gyro;
  EXPECT_EQ( broken_gyro.m_break_reason,
             GyroBreakReason::kDeclaredGap );
  EXPECT_EQ( broken_gyro.m_rotation_factors, 0U );
  EXPECT_EQ( broken_gyro.m_rw_factors, 0U );
  EXPECT_EQ( broken_gyro.m_root_priors, 2U );
  ASSERT_EQ( broken_gyro.m_window_biases.size(), 2U );
  EXPECT_FALSE( broken_gyro.m_window_biases[ 0 ].m_bias_radps.has_value() );
  EXPECT_FALSE( broken_gyro.m_window_biases[ 1 ].m_bias_radps.has_value() );

  KeyframeMeasurement exact    = makeVisualMeasurement( calibration, 2U );
  exact.m_gyro_interval        = makeGyroInterval( 2U );
  const VioUpdateResult linked = estimator.update( exact, false );
  ASSERT_EQ( linked.status, UpdateStatus::kOk ) << linked.message;
  ASSERT_TRUE( linked.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& linked_gyro = *linked.diagnostics.m_gyro;
  EXPECT_EQ( linked_gyro.m_break_reason, GyroBreakReason::kNone );
  EXPECT_EQ( linked_gyro.m_rotation_factors, 1U );
  EXPECT_EQ( linked_gyro.m_rw_factors, 1U );
  EXPECT_EQ( linked_gyro.m_root_priors, 2U );
}

TEST( StereoVoOnlineGyroBias, ReanchorClearsOldForestAndStartsNewSegment )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.enable_reanchor                      = true;
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  const VioUpdateResult root =
      estimator.update( makeVisualMeasurement( calibration, 0U ), true );
  ASSERT_EQ( root.status, UpdateStatus::kOk ) << root.message;

  KeyframeMeasurement reanchor = makeVisualMeasurement( calibration, 1U );
  reanchor.m_gyro_interval     = makeGyroInterval( 1U );
  for ( StereoObservation& observation : reanchor.observations )
  {
    observation.id += 10'000U;
  }
  const VioUpdateResult restarted = estimator.update( reanchor, true );
  ASSERT_EQ( restarted.status, UpdateStatus::kOk ) << restarted.message;
  EXPECT_EQ( restarted.diagnostics.segment_id, 1U );
  ASSERT_TRUE( restarted.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& restarted_gyro = *restarted.diagnostics.m_gyro;
  EXPECT_EQ( restarted_gyro.m_break_reason,
             GyroBreakReason::kSegmentChange );
  EXPECT_EQ( restarted_gyro.m_rotation_factors, 0U );
  EXPECT_EQ( restarted_gyro.m_rw_factors, 0U );
  EXPECT_EQ( restarted_gyro.m_root_priors, 1U );
  ASSERT_EQ( restarted_gyro.m_window_biases.size(), 1U );
  EXPECT_EQ( restarted_gyro.m_window_biases.front().m_frame_index, 1U );
  EXPECT_EQ( restarted_gyro.m_window_biases.front().m_timestamp,
             reanchor.timestamp );
  EXPECT_FALSE(
      restarted_gyro.m_window_biases.front().m_bias_radps.has_value() );

  KeyframeMeasurement exact = makeVisualMeasurement( calibration, 2U );
  exact.m_gyro_interval     = makeGyroInterval( 2U );
  for ( StereoObservation& observation : exact.observations )
  {
    observation.id += 10'000U;
  }
  const VioUpdateResult linked = estimator.update( exact, false );
  ASSERT_EQ( linked.status, UpdateStatus::kOk ) << linked.message;
  EXPECT_EQ( linked.diagnostics.segment_id, 1U );
  ASSERT_TRUE( linked.diagnostics.m_gyro.has_value() );
  const GyroDiagnostics& linked_gyro = *linked.diagnostics.m_gyro;
  EXPECT_EQ( linked_gyro.m_break_reason, GyroBreakReason::kNone );
  EXPECT_EQ( linked_gyro.m_rotation_factors, 1U );
  EXPECT_EQ( linked_gyro.m_rw_factors, 1U );
  EXPECT_EQ( linked_gyro.m_root_priors, 1U );
  ASSERT_EQ( linked_gyro.m_window_biases.size(), 2U );
  EXPECT_EQ( linked_gyro.m_window_biases[ 0 ].m_frame_index, 1U );
  EXPECT_EQ( linked_gyro.m_window_biases[ 1 ].m_frame_index, 2U );
  EXPECT_TRUE( linked_gyro.m_window_biases[ 0 ].m_bias_radps.has_value() );
  EXPECT_TRUE( linked_gyro.m_window_biases[ 1 ].m_bias_radps.has_value() );
}

TEST( StereoVoOnlineGyroBias, OffIgnoresEveryGyroPayloadBitExactly )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  const EstimatorOptions           options     = makeVisualOptions();

  const auto run = [ & ]( int payload_kind ) {
    StereoVoEstimator            estimator( calibration, options );
    std::vector<VioUpdateResult> results;
    results.reserve( 14U );
    for ( std::size_t frame_index = 0; frame_index < 14U; ++frame_index )
    {
      KeyframeMeasurement measurement =
          makeVisualMeasurement( calibration, frame_index );
      const std::int64_t t_curr_ns = measurement.timestamp.nanoseconds();
      const std::int64_t t_prev_ns = t_curr_ns - 100'000'000;
      if ( payload_kind == 1 )
      {
        measurement.m_gyro_interval =
            lifecycleGyroInterval( t_prev_ns, t_curr_ns, kBiasBase );
      }
      else if ( payload_kind == 2 )
      {
        GyroInterval interval;
        interval.m_t_prev           = phad::common::Timestamp{ t_prev_ns };
        interval.m_samples          = { reducerSample( t_prev_ns, kBiasBase ) };
        measurement.m_gyro_interval = std::move( interval );
      }
      else if ( payload_kind == 3 )
      {
        measurement.m_gyro_interval = lifecycleGyroInterval(
            t_prev_ns, t_curr_ns,
            Eigen::Vector3d{
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::infinity(), 0.0 } );
      }
      else if ( payload_kind == 4 )
      {
        GyroInterval interval;
        interval.m_t_prev = phad::common::Timestamp{
            std::numeric_limits<std::int64_t>::min() };
        interval.m_samples = {
            reducerSample( std::numeric_limits<std::int64_t>::min(),
                           kBiasBase ),
            reducerSample( std::numeric_limits<std::int64_t>::max(),
                           kBiasBase ) };
        measurement.m_gyro_interval = std::move( interval );
      }

      results.push_back( estimator.update( measurement, frame_index < 2U ) );
      EXPECT_EQ( results.back().status, UpdateStatus::kOk )
          << results.back().message;
      EXPECT_FALSE( results.back().diagnostics.m_gyro.has_value() );
    }
    return encodeVisualResults( results );
  };

  const std::string authority_baseline = run( 0 );
  for ( int payload_kind = 1; payload_kind <= 4; ++payload_kind )
  {
    EXPECT_EQ( run( payload_kind ), authority_baseline );
  }
}

TEST( StereoVoOnlineGyroBias, SixteenFreshInstancesAreBitDeterministic )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.m_gyro_bias                          = makeGyroOptions();

  const auto run = [ & ]() {
    StereoVoEstimator            estimator( calibration, options );
    std::vector<VioUpdateResult> results;
    results.reserve( 14U );
    for ( std::size_t frame_index = 0; frame_index < 14U; ++frame_index )
    {
      KeyframeMeasurement measurement =
          makeVisualMeasurement( calibration, frame_index );
      if ( frame_index > 0U )
      {
        measurement.m_gyro_interval = makeGyroInterval( frame_index );
      }
      results.push_back(
          estimator.update( measurement, frame_index < 2U ) );
      EXPECT_EQ( results.back().status, UpdateStatus::kOk )
          << results.back().message;
    }
    return encodeOnlineBiasResults( results );
  };

  const std::string expected = run();
  for ( int instance = 1; instance < 16; ++instance )
  {
    EXPECT_EQ( run(), expected ) << "fresh instance " << instance;
  }
}

TEST( StereoVoOnlineGyroBias, ConstantBiasHasZeroDriftAfterThreeIntervals )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 options     = makeVisualOptions();
  options.m_gyro_bias                          = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity() );
  Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity() );

  for ( std::size_t frame_index = 0; frame_index < 14U; ++frame_index )
  {
    KeyframeMeasurement measurement =
        makeVisualMeasurement( calibration, frame_index );
    if ( frame_index > 0U )
    {
      measurement.m_gyro_interval = lifecycleGyroInterval(
          static_cast<std::int64_t>( frame_index ) * 100'000'000,
          static_cast<std::int64_t>( frame_index + 1U ) * 100'000'000,
          kBiasBase );
    }

    const VioUpdateResult result =
        estimator.update( measurement, frame_index < 2U );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    ASSERT_TRUE( result.diagnostics.m_gyro.has_value() );
    const GyroDiagnostics& gyro = *result.diagnostics.m_gyro;
    EXPECT_EQ( gyro.m_break_reason, GyroBreakReason::kNone );
    EXPECT_EQ( gyro.m_rotation_factors,
               static_cast<std::uint32_t>( frame_index ) );
    EXPECT_EQ( gyro.m_rw_factors,
               static_cast<std::uint32_t>( frame_index ) );
    EXPECT_EQ( gyro.m_root_priors, 1U );
    ASSERT_EQ( gyro.m_window_biases.size(), frame_index + 1U );

    if ( frame_index == 0U )
    {
      EXPECT_FALSE( gyro.m_bias_radps.has_value() );
      EXPECT_FALSE(
          gyro.m_window_biases.front().m_bias_radps.has_value() );
      continue;
    }

    ASSERT_TRUE( gyro.m_bias_radps.has_value() );
    EXPECT_TRUE( gyro.m_bias_radps->allFinite() );
    if ( frame_index >= 3U )
    {
      EXPECT_LE( ( *gyro.m_bias_radps - kBiasBase )
                     .cwiseAbs()
                     .maxCoeff(),
                 1e-6 );
      minimum = minimum.cwiseMin( *gyro.m_bias_radps );
      maximum = maximum.cwiseMax( *gyro.m_bias_radps );
    }
  }

  EXPECT_LE( ( maximum - minimum ).cwiseAbs().maxCoeff(), 1e-6 );
}

TEST( StereoVoOnlineGyroBias, ExactZeroLeavesBiasAndPoseRotationAtZero )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 on_options  = makeVisualOptions();
  on_options.m_gyro_bias                       = makeGyroOptions();
  StereoVoEstimator on_estimator( calibration, on_options );
  StereoVoEstimator off_estimator( calibration, makeVisualOptions() );

  for ( std::size_t frame_index = 0; frame_index < 14U; ++frame_index )
  {
    KeyframeMeasurement on_measurement =
        makeVisualMeasurement( calibration, frame_index );
    if ( frame_index > 0U )
    {
      on_measurement.m_gyro_interval = lifecycleGyroInterval(
          static_cast<std::int64_t>( frame_index ) * 100'000'000,
          static_cast<std::int64_t>( frame_index + 1U ) * 100'000'000,
          Eigen::Vector3d::Zero() );
    }
    const KeyframeMeasurement off_measurement =
        makeVisualMeasurement( calibration, frame_index );

    const bool            keyframe = frame_index < 2U;
    const VioUpdateResult on_result =
        on_estimator.update( on_measurement, keyframe );
    const VioUpdateResult off_result =
        off_estimator.update( off_measurement, keyframe );
    ASSERT_EQ( on_result.status, UpdateStatus::kOk ) << on_result.message;
    ASSERT_EQ( off_result.status, UpdateStatus::kOk ) << off_result.message;
    ASSERT_TRUE( on_result.estimate.has_value() );
    ASSERT_TRUE( off_result.estimate.has_value() );
    ASSERT_TRUE( on_result.diagnostics.m_gyro.has_value() );

    const Eigen::Matrix3d relative =
        on_result.estimate->T_W_B.linear().transpose() *
        off_result.estimate->T_W_B.linear();
    EXPECT_LE( Eigen::AngleAxisd( relative ).angle(), 1e-12 );

    const GyroDiagnostics& gyro = *on_result.diagnostics.m_gyro;
    EXPECT_EQ( gyro.m_break_reason, GyroBreakReason::kNone );
    if ( frame_index == 0U )
    {
      EXPECT_FALSE( gyro.m_bias_radps.has_value() );
    }
    else
    {
      ASSERT_TRUE( gyro.m_bias_radps.has_value() );
      EXPECT_LE( gyro.m_bias_radps->norm(), 1e-12 );
    }
  }
}

TEST( StereoVoOnlineGyroBias, NonIdentityExtrinsicKeepsBiasInBodyFrame )
{
  const RectifiedStereoCalibration calibration =
      makeNonIdentityVisualCalibration();
  EstimatorOptions options = makeVisualOptions();
  options.m_gyro_bias      = makeGyroOptions();
  StereoVoEstimator estimator( calibration, options );

  Eigen::Vector3d frame_one_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d final_bias     = Eigen::Vector3d::Zero();
  for ( std::size_t frame_index = 0; frame_index < 14U; ++frame_index )
  {
    KeyframeMeasurement measurement =
        makeBodyStationaryMeasurement( calibration, frame_index );
    if ( frame_index > 0U )
    {
      measurement.m_gyro_interval = makeGyroInterval( frame_index );
    }

    const VioUpdateResult result =
        estimator.update( measurement, frame_index < 2U );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
    ASSERT_TRUE( result.diagnostics.m_gyro.has_value() );
    const GyroDiagnostics& gyro = *result.diagnostics.m_gyro;
    EXPECT_EQ( result.diagnostics.window_size,
               static_cast<std::uint32_t>( frame_index + 1U ) );
    EXPECT_EQ( gyro.m_window_biases.size(), frame_index + 1U );
    EXPECT_EQ( gyro.m_rotation_factors,
               static_cast<std::uint32_t>( frame_index ) );
    EXPECT_EQ( gyro.m_rw_factors,
               static_cast<std::uint32_t>( frame_index ) );
    EXPECT_EQ( gyro.m_root_priors, 1U );
    EXPECT_EQ( gyro.m_relinearization_rounds,
               frame_index == 1U ? 1U : 0U );
    EXPECT_EQ( gyro.m_break_reason, GyroBreakReason::kNone );

    for ( std::size_t window_index = 0;
          window_index < gyro.m_window_biases.size(); ++window_index )
    {
      const auto& window_bias = gyro.m_window_biases[ window_index ];
      EXPECT_EQ( window_bias.m_frame_index, window_index );
      EXPECT_EQ( window_bias.m_timestamp.nanoseconds(),
                 static_cast<std::int64_t>( window_index + 1U ) *
                     100'000'000 );
      if ( frame_index == 0U )
      {
        EXPECT_FALSE( window_bias.m_bias_radps.has_value() );
      }
      else
      {
        ASSERT_TRUE( window_bias.m_bias_radps.has_value() );
        expectBiasNearTruth( *window_bias.m_bias_radps, window_index );
      }
    }

    if ( frame_index == 0U )
    {
      EXPECT_FALSE( gyro.m_bias_radps.has_value() );
      continue;
    }

    ASSERT_TRUE( gyro.m_bias_radps.has_value() );
    EXPECT_TRUE( gyro.m_bias_radps->allFinite() );
    expectBiasNearTruth( *gyro.m_bias_radps, frame_index );
    if ( frame_index == 1U )
    {
      frame_one_bias = *gyro.m_bias_radps;
    }
    if ( frame_index == 13U )
    {
      final_bias = *gyro.m_bias_radps;
    }
  }

  const Eigen::Vector3d truth_drift  = biasTruth( 12U ) - biasTruth( 0U );
  const Eigen::Vector3d actual_drift = final_bias - frame_one_bias;
  for ( Eigen::Index axis = 0; axis < actual_drift.size(); ++axis )
  {
    EXPECT_GT( actual_drift[ axis ] * truth_drift[ axis ], 0.0 );
    const double ratio = actual_drift[ axis ] / truth_drift[ axis ];
    EXPECT_GE( ratio, 0.85 );
    EXPECT_LE( ratio, 1.05 );
  }
  const double l2_ratio = actual_drift.norm() / truth_drift.norm();
  EXPECT_GE( l2_ratio, 0.85 );
  EXPECT_LE( l2_ratio, 1.05 );

  const Eigen::Matrix3d R_B_left =
      gtsam::Rot3::Expmap( kNonIdentityRotationVector ).matrix();
  const Eigen::Vector3d final_truth = biasTruth( 12U );
  const Eigen::Vector3d kIndependentCorrectFinal{
      0.013126669336378, -0.019698172265706, 0.027261275987490 };
  const double correct_error =
      ( kIndependentCorrectFinal - final_truth ).cwiseAbs().maxCoeff();
  const double body_to_camera_error =
      ( R_B_left * kIndependentCorrectFinal - final_truth )
          .cwiseAbs()
          .maxCoeff();
  const double inverse_frame_error =
      ( R_B_left.transpose() * kIndependentCorrectFinal - final_truth )
          .cwiseAbs()
          .maxCoeff();
  EXPECT_LE( correct_error, 5e-4 );
  EXPECT_GT( body_to_camera_error, 5e-4 );
  EXPECT_GT( inverse_frame_error, 5e-4 );
}

TEST( StereoVoOnlineGyroBias, SuccessfulReoptWritesBackGyroBias )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 on_options  = makeVisualOptions();
  on_options.enable_outlier_cull               = true;
  on_options.enable_outlier_reopt              = true;
  on_options.max_outlier_reopts                = 1;
  on_options.outlier_avg_reproj_px             = 3.0;
  on_options.block_culled_rebirth              = false;
  on_options.m_gyro_bias                       = makeGyroOptions();
  EstimatorOptions off_options                 = on_options;
  off_options.enable_outlier_reopt             = false;
  StereoVoEstimator on_estimator( calibration, on_options );
  StereoVoEstimator off_estimator( calibration, off_options );

  const auto poison = []( KeyframeMeasurement& measurement,
                          std::size_t          frame_index ) {
    for ( StereoObservation& observation : measurement.observations )
    {
      if ( observation.id > 4U )
      {
        continue;
      }
      const double frame_sign = frame_index % 2U == 0U ? 1.0 : -1.0;
      const double id_sign    = observation.id % 2U == 0U ? -1.0 : 1.0;
      observation.left_pixel.x() += 80.0 * frame_sign * id_sign;
    }
  };

  bool                           saw_reopt      = false;
  bool                           saw_warm_start = false;
  std::optional<Eigen::Vector3d> reopt_bias;
  std::uint32_t                  reopt_rotation_factors = 0U;
  for ( std::size_t frame_index = 0; frame_index < 14U; ++frame_index )
  {
    KeyframeMeasurement measurement =
        makeVisualMeasurement( calibration, frame_index );
    if ( frame_index > 0U )
    {
      measurement.m_gyro_interval = makeGyroInterval( frame_index );
      if ( !saw_reopt )
      {
        poison( measurement, frame_index );
      }
    }
    const bool            keyframe = saw_reopt || frame_index < 2U;
    const VioUpdateResult on_result =
        on_estimator.update( measurement, keyframe );
    const VioUpdateResult off_result =
        off_estimator.update( measurement, keyframe );
    ASSERT_EQ( on_result.status, UpdateStatus::kOk ) << on_result.message;
    ASSERT_EQ( off_result.status, UpdateStatus::kOk ) << off_result.message;

    if ( saw_reopt )
    {
      ASSERT_TRUE( reopt_bias.has_value() );
      ASSERT_TRUE( on_result.diagnostics.m_gyro.has_value() );
      const GyroDiagnostics& gyro = *on_result.diagnostics.m_gyro;
      ASSERT_TRUE( gyro.m_bias_radps.has_value() );
      EXPECT_EQ( gyro.m_relinearization_rounds, 0U );
      EXPECT_EQ( gyro.m_rotation_factors, reopt_rotation_factors + 1U );
      EXPECT_EQ( gyro.m_rw_factors, gyro.m_rotation_factors );
      EXPECT_EQ( gyro.m_root_priors, 1U );
      ASSERT_GE( gyro.m_window_biases.size(), 2U );
      const auto& prior_frame_bias =
          gyro.m_window_biases[ gyro.m_window_biases.size() - 2U ];
      ASSERT_TRUE( prior_frame_bias.m_bias_radps.has_value() );
      EXPECT_LE( ( *prior_frame_bias.m_bias_radps - *reopt_bias )
                     .cwiseAbs()
                     .maxCoeff(),
                 1e-3 );
      saw_warm_start = true;
      break;
    }
    if ( !on_result.diagnostics.outlier_reopt )
    {
      continue;
    }
    EXPECT_FALSE( on_result.diagnostics.outlier_reopt_failed );
    EXPECT_EQ( on_result.diagnostics.outlier_reopt_rounds, 1U );
    EXPECT_FALSE( off_result.diagnostics.outlier_reopt );
    ASSERT_TRUE( on_result.diagnostics.m_gyro.has_value() );
    ASSERT_TRUE( off_result.diagnostics.m_gyro.has_value() );
    const GyroDiagnostics& on_gyro  = *on_result.diagnostics.m_gyro;
    const GyroDiagnostics& off_gyro = *off_result.diagnostics.m_gyro;
    ASSERT_TRUE( on_gyro.m_bias_radps.has_value() );
    ASSERT_TRUE( off_gyro.m_bias_radps.has_value() );
    EXPECT_TRUE( on_gyro.m_bias_radps->allFinite() );
    EXPECT_EQ( on_gyro.m_rotation_factors, off_gyro.m_rotation_factors );
    EXPECT_EQ( on_gyro.m_rw_factors, off_gyro.m_rw_factors );
    EXPECT_EQ( on_gyro.m_root_priors, off_gyro.m_root_priors );

    bool any_bias_bit_changed = false;
    for ( Eigen::Index axis = 0; axis < on_gyro.m_bias_radps->size(); ++axis )
    {
      any_bias_bit_changed =
          any_bias_bit_changed ||
          std::bit_cast<std::uint64_t>( ( *on_gyro.m_bias_radps )[ axis ] ) !=
              std::bit_cast<std::uint64_t>(
                  ( *off_gyro.m_bias_radps )[ axis ] );
    }
    EXPECT_TRUE( any_bias_bit_changed );
    reopt_bias             = *on_gyro.m_bias_radps;
    reopt_rotation_factors = on_gyro.m_rotation_factors;
    saw_reopt              = true;
  }
  EXPECT_TRUE( saw_reopt );
  EXPECT_TRUE( saw_warm_start );
}

TEST( StereoVoOnlineGyroBias, FailedReoptRestoresLm1GyroBias )
{
  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  EstimatorOptions                 on_options;
  on_options.min_track_observations_for_seed = 1;
  on_options.window_size                     = 6;
  on_options.min_seed_observations           = 4;
  on_options.min_shared_landmarks            = 2;
  on_options.min_landmark_observations       = 2;
  on_options.enable_outlier_cull             = true;
  on_options.enable_outlier_reopt            = true;
  on_options.max_outlier_reopts              = 1;
  on_options.outlier_avg_reproj_px           = 0.5;
  on_options.block_culled_rebirth            = false;
  on_options.m_gyro_bias                     = makeGyroOptions();
  on_options.m_gyro_bias->m_prior_mean_radps = kBiasBase;
  EstimatorOptions off_options               = on_options;
  off_options.enable_outlier_reopt           = false;
  StereoVoEstimator on_estimator( calibration, on_options );
  StereoVoEstimator off_estimator( calibration, off_options );

  const std::array<Eigen::Vector3d, 4> sparse_landmarks{
      Eigen::Vector3d{ 0.15, 0.12, 4.5 },
      Eigen::Vector3d{ -0.15, 0.12, 4.5 },
      Eigen::Vector3d{ 0.15, -0.12, 4.9 },
      Eigen::Vector3d{ -0.15, -0.12, 4.9 },
  };

  const auto make_measurement = [ & ]( std::size_t frame_index,
                                       bool        inject_outliers ) {
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    T_W_B.translation().x() = 0.05 * static_cast<double>( frame_index );
    KeyframeMeasurement measurement;
    measurement.timestamp = phad::common::Timestamp{
        static_cast<std::int64_t>( frame_index + 1U ) * 50'000'000 };
    for ( std::size_t index = 0U; index < sparse_landmarks.size(); ++index )
    {
      const Eigen::Vector3d point_left =
          T_W_B.inverse() * sparse_landmarks[ index ];
      const double u_l = calibration.fxPixels() * point_left.x() /
                             point_left.z() +
                         calibration.cxPixels();
      const double v = calibration.fyPixels() * point_left.y() /
                           point_left.z() +
                       calibration.cyPixels();
      const double disparity = calibration.fxPixels() *
                               calibration.baselineM() /
                               point_left.z();
      measurement.observations.push_back(
          StereoObservation{ static_cast<LandmarkId>( index + 1U ),
                             Eigen::Vector2d{ u_l, v },
                             disparity } );
    }
    if ( inject_outliers && frame_index >= 4U )
    {
      for ( StereoObservation& observation : measurement.observations )
      {
        const double frame_sign = frame_index % 2U == 0U ? 1.0 : -1.0;
        const double id_sign    = observation.id % 2U == 0U ? -1.0 : 1.0;
        observation.left_pixel.x() += 200.0 * frame_sign * id_sign;
      }
    }
    if ( frame_index == 1U )
    {
      const std::int64_t t_prev_ns =
          static_cast<std::int64_t>( frame_index ) * 50'000'000;
      const std::int64_t t_curr_ns =
          static_cast<std::int64_t>( frame_index + 1U ) * 50'000'000;
      measurement.m_gyro_interval = lifecycleGyroInterval(
          t_prev_ns, t_curr_ns, kBiasBase + 20.0 * kBiasStep );
    }
    return measurement;
  };

  bool          saw_failure = false;
  std::uint32_t max_culled  = 0U;
  for ( std::size_t frame_index = 0U; frame_index < 20U; ++frame_index )
  {
    const KeyframeMeasurement measurement =
        make_measurement( frame_index, true );
    const VioUpdateResult on_result = on_estimator.update( measurement, true );
    const VioUpdateResult off_result =
        off_estimator.update( measurement, true );
    ASSERT_EQ( on_result.status, UpdateStatus::kOk ) << on_result.message;
    ASSERT_EQ( off_result.status, UpdateStatus::kOk ) << off_result.message;
    max_culled =
        std::max( max_culled, on_result.diagnostics.outliers_culled );

    if ( on_result.diagnostics.outliers_culled < 4U )
    {
      continue;
    }
    EXPECT_FALSE( on_result.diagnostics.outlier_reopt );
    EXPECT_EQ( on_result.diagnostics.outlier_reopt_rounds, 0U );
    EXPECT_TRUE( on_result.diagnostics.outlier_reopt_failed );
    EXPECT_FALSE( off_result.diagnostics.outlier_reopt );
    EXPECT_FALSE( off_result.diagnostics.outlier_reopt_failed );
    ASSERT_TRUE( on_result.estimate.has_value() );
    ASSERT_TRUE( off_result.estimate.has_value() );
    EXPECT_TRUE( on_result.estimate->T_W_B.matrix().isApprox(
        off_result.estimate->T_W_B.matrix(), 1e-12 ) );
    ASSERT_TRUE( on_result.diagnostics.m_gyro.has_value() );
    ASSERT_TRUE( off_result.diagnostics.m_gyro.has_value() );
    const GyroDiagnostics& on_gyro = *on_result.diagnostics.m_gyro;
    EXPECT_GT( on_gyro.m_rotation_factors, 0U );
    EXPECT_EQ( on_gyro.m_rw_factors, on_gyro.m_rotation_factors );
    EXPECT_GT( on_gyro.m_root_priors, 0U );
    EXPECT_GT( on_gyro.m_relinearization_rounds, 0U );
    EXPECT_EQ( on_gyro.m_relinearization_rounds,
               off_result.diagnostics.m_gyro->m_relinearization_rounds );
    bool any_linked_bias_changed = false;
    for ( const auto& window_bias : on_gyro.m_window_biases )
    {
      if ( window_bias.m_bias_radps.has_value() )
      {
        any_linked_bias_changed =
            any_linked_bias_changed ||
            ( *window_bias.m_bias_radps - kBiasBase )
                    .cwiseAbs()
                    .maxCoeff() >
                0.0;
      }
    }
    EXPECT_TRUE( any_linked_bias_changed );
    GyroDiagnostics expected_gyro = *off_result.diagnostics.m_gyro;
    expected_gyro.m_relinearization_rounds =
        on_gyro.m_relinearization_rounds;
    expectGyroDiagnosticsExact( on_gyro, expected_gyro );

    KeyframeMeasurement continuation =
        make_measurement( frame_index + 1U, false );
    continuation.m_gyro_interval = lifecycleGyroInterval(
        static_cast<std::int64_t>( frame_index + 1U ) * 50'000'000,
        static_cast<std::int64_t>( frame_index + 2U ) * 50'000'000,
        kBiasBase + 21.0 * kBiasStep );
    const VioUpdateResult on_continuation =
        on_estimator.update( continuation, true );
    const VioUpdateResult off_continuation =
        off_estimator.update( continuation, true );
    ASSERT_EQ( on_continuation.status, UpdateStatus::kOk )
        << on_continuation.message;
    ASSERT_EQ( off_continuation.status, UpdateStatus::kOk )
        << off_continuation.message;
    EXPECT_EQ( encodeVisualResults( { on_continuation } ),
               encodeVisualResults( { off_continuation } ) );
    ASSERT_TRUE( on_continuation.diagnostics.m_gyro.has_value() );
    ASSERT_TRUE( off_continuation.diagnostics.m_gyro.has_value() );
    expectGyroDiagnosticsExact( *on_continuation.diagnostics.m_gyro,
                                *off_continuation.diagnostics.m_gyro );
    saw_failure = true;
    break;
  }
  EXPECT_TRUE( saw_failure ) << "max_culled=" << max_culled;
}

TEST( StereoVoOnlineGyroBias,
      MalformedIntervalMatrixPreservesPrecedenceAndState )
{
  struct MalformedCase
  {
    std::string                                 m_name;
    std::string                                 m_message;
    std::function<void( GyroBiasOptions& )>     m_options;
    std::function<void( KeyframeMeasurement& )> m_measurement;
  };

  constexpr std::int64_t kRootNs           = 100'000'000;
  constexpr std::int64_t kCurrentNs        = 200'000'000;
  const auto             no_options_change = []( GyroBiasOptions& ) {};
  const auto             interval          = []( std::int64_t t_prev_ns,
                            std::int64_t t_curr_ns ) {
    return lifecycleGyroInterval( t_prev_ns, t_curr_ns, kBiasBase );
  };
  const auto set_interval = []( KeyframeMeasurement& measurement,
                                GyroInterval         gyro ) {
    measurement.m_gyro_interval = std::move( gyro );
  };

  const std::vector<MalformedCase> cases{
      { "too-few", "gyro interval requires at least two samples",
        no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro;
          gyro.m_t_prev  = phad::common::Timestamp{ kRootNs };
          gyro.m_samples = { reducerSample( kRootNs, kBiasBase ) };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "endpoint", "gyro interval endpoints do not match pose timestamps",
        no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro = interval( kRootNs, kCurrentNs );
          gyro.m_t_prev     = phad::common::Timestamp{ kRootNs - 1 };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "duplicate", "gyro sample timestamps must be strictly increasing",
        no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro;
          gyro.m_t_prev  = phad::common::Timestamp{ kRootNs };
          gyro.m_samples = { reducerSample( kRootNs, kBiasBase ),
                             reducerSample( kRootNs, kBiasBase ),
                             reducerSample( kCurrentNs, kBiasBase ) };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "reverse", "gyro sample timestamps must be strictly increasing",
        no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro;
          gyro.m_t_prev  = phad::common::Timestamp{ kRootNs };
          gyro.m_samples = {
              reducerSample( kRootNs, kBiasBase ),
              reducerSample( kRootNs - 1, kBiasBase ),
              reducerSample( kCurrentNs, kBiasBase ) };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "nonfinite", "gyro sample is non-finite", no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro = interval( kRootNs, kCurrentNs );
          gyro.m_samples[ 1 ].gyro_radps[ 0 ] =
              std::numeric_limits<double>::quiet_NaN();
          set_interval( measurement, std::move( gyro ) );
        } },
      { "overflow", "gyro interval duration overflow", no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro;
          gyro.m_t_prev = phad::common::Timestamp{
              std::numeric_limits<std::int64_t>::min() };
          gyro.m_samples = {
              reducerSample( std::numeric_limits<std::int64_t>::min(),
                             kBiasBase ),
              reducerSample( kCurrentNs, kBiasBase ) };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "derived-noise", "gyro interval produces invalid noise scale",
        []( GyroBiasOptions& options ) { options.m_gyr_nd = 1e150; },
        [ & ]( KeyframeMeasurement& measurement ) {
          measurement.timestamp = phad::common::Timestamp{
              std::numeric_limits<std::int64_t>::max() };
          GyroInterval gyro;
          gyro.m_t_prev  = phad::common::Timestamp{ kRootNs };
          gyro.m_samples = {
              reducerSample( kRootNs, kBiasBase ),
              reducerSample( std::numeric_limits<std::int64_t>::max(),
                             kBiasBase ) };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "too-few-over-endpoint-and-nonfinite",
        "gyro interval requires at least two samples", no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro;
          gyro.m_t_prev  = phad::common::Timestamp{ kRootNs - 1 };
          gyro.m_samples = { reducerSample(
              kRootNs,
              Eigen::Vector3d{
                  std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0 } ) };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "endpoint-over-order-and-nonfinite",
        "gyro interval endpoints do not match pose timestamps",
        no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro;
          gyro.m_t_prev  = phad::common::Timestamp{ kRootNs - 1 };
          gyro.m_samples = {
              reducerSample(
                  kRootNs,
                  Eigen::Vector3d{
                      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0 } ),
              reducerSample( kRootNs, kBiasBase ),
              reducerSample( kCurrentNs, kBiasBase ) };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "order-over-nonfinite", "gyro sample timestamps must be strictly increasing",
        no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro;
          gyro.m_t_prev  = phad::common::Timestamp{ kRootNs };
          gyro.m_samples = {
              reducerSample(
                  kRootNs,
                  Eigen::Vector3d{
                      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0 } ),
              reducerSample( kRootNs, kBiasBase ),
              reducerSample( kCurrentNs, kBiasBase ) };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "nonfinite-over-overflow", "gyro sample is non-finite",
        no_options_change,
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro;
          gyro.m_t_prev = phad::common::Timestamp{
              std::numeric_limits<std::int64_t>::min() };
          gyro.m_samples = {
              reducerSample(
                  std::numeric_limits<std::int64_t>::min(),
                  Eigen::Vector3d{
                      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0 } ),
              reducerSample( kCurrentNs, kBiasBase ) };
          set_interval( measurement, std::move( gyro ) );
        } },
      { "overflow-over-derived-noise", "gyro interval duration overflow",
        []( GyroBiasOptions& options ) { options.m_gyr_nd = 1e150; },
        [ & ]( KeyframeMeasurement& measurement ) {
          GyroInterval gyro;
          gyro.m_t_prev = phad::common::Timestamp{
              std::numeric_limits<std::int64_t>::min() };
          gyro.m_samples = {
              reducerSample( std::numeric_limits<std::int64_t>::min(),
                             kBiasBase ),
              reducerSample( kCurrentNs, kBiasBase ) };
          set_interval( measurement, std::move( gyro ) );
        } },
  };

  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  for ( const MalformedCase& test_case : cases )
  {
    SCOPED_TRACE( test_case.m_name );
    EstimatorOptions options      = makeVisualOptions();
    GyroBiasOptions  gyro_options = makeGyroOptions();
    test_case.m_options( gyro_options );
    options.m_gyro_bias = gyro_options;
    StereoVoEstimator subject( calibration, options );
    StereoVoEstimator control( calibration, options );

    const KeyframeMeasurement root =
        makeVisualMeasurement( calibration, 0U );
    ASSERT_EQ( subject.update( root, true ).status, UpdateStatus::kOk );
    ASSERT_EQ( control.update( root, true ).status, UpdateStatus::kOk );

    KeyframeMeasurement malformed =
        makeVisualMeasurement( calibration, 1U );
    test_case.m_measurement( malformed );
    const VioUpdateResult rejected = subject.update( malformed, false );
    EXPECT_EQ( rejected.status, UpdateStatus::kRejected );
    EXPECT_EQ( rejected.message, test_case.m_message );
    EXPECT_FALSE( rejected.estimate.has_value() );
    ASSERT_TRUE( rejected.diagnostics.m_gyro.has_value() );
    const GyroDiagnostics& rejected_gyro = *rejected.diagnostics.m_gyro;
    EXPECT_FALSE( rejected_gyro.m_bias_radps.has_value() );
    EXPECT_EQ( rejected_gyro.m_relinearization_rounds, 0U );
    EXPECT_EQ( rejected_gyro.m_break_reason, GyroBreakReason::kNone );
    EXPECT_EQ( rejected_gyro.m_rotation_factors, 0U );
    EXPECT_EQ( rejected_gyro.m_rw_factors, 0U );
    EXPECT_EQ( rejected_gyro.m_root_priors, 1U );
    ASSERT_EQ( rejected_gyro.m_window_biases.size(), 1U );
    EXPECT_EQ( rejected_gyro.m_window_biases.front().m_frame_index, 0U );

    KeyframeMeasurement exact =
        makeVisualMeasurement( calibration, 2U );
    exact.m_gyro_interval = lifecycleGyroInterval(
        root.timestamp.nanoseconds(), exact.timestamp.nanoseconds(),
        kBiasBase );
    const VioUpdateResult subject_after = subject.update( exact, false );
    const VioUpdateResult control_after = control.update( exact, false );
    ASSERT_EQ( subject_after.status, UpdateStatus::kOk )
        << subject_after.message;
    ASSERT_EQ( control_after.status, UpdateStatus::kOk )
        << control_after.message;
    EXPECT_EQ( encodeVisualResults( { subject_after } ),
               encodeVisualResults( { control_after } ) );
    ASSERT_TRUE( subject_after.diagnostics.m_gyro.has_value() );
    ASSERT_TRUE( control_after.diagnostics.m_gyro.has_value() );
    expectGyroDiagnosticsExact( *subject_after.diagnostics.m_gyro,
                                *control_after.diagnostics.m_gyro );
  }
}

TEST( StereoVoOnlineGyroBias, ConstructorRejectsInvalidGyroOptions )
{
  using Mutator = std::function<void( GyroBiasOptions& )>;
  const std::vector<Mutator> mutators{
      []( GyroBiasOptions& options ) { options.m_gyr_nd = 0.0; },
      []( GyroBiasOptions& options ) {
        options.m_gyr_nd = std::numeric_limits<double>::infinity();
      },
      []( GyroBiasOptions& options ) { options.m_gyr_rw = 0.0; },
      []( GyroBiasOptions& options ) {
        options.m_gyr_rw = std::numeric_limits<double>::quiet_NaN();
      },
      []( GyroBiasOptions& options ) {
        options.m_prior_mean_radps.y() =
            std::numeric_limits<double>::infinity();
      },
      []( GyroBiasOptions& options ) { options.m_prior_sigma_radps = 0.0; },
      []( GyroBiasOptions& options ) {
        options.m_prior_sigma_radps =
            std::numeric_limits<double>::denorm_min();
      },
      []( GyroBiasOptions& options ) {
        options.m_prior_sigma_radps = std::numeric_limits<double>::max();
      },
  };

  const RectifiedStereoCalibration calibration = makeVisualCalibration();
  for ( const Mutator& mutate : mutators )
  {
    EstimatorOptions options = makeVisualOptions();
    GyroBiasOptions  gyro    = makeGyroOptions();
    mutate( gyro );
    options.m_gyro_bias = gyro;
    EXPECT_THROW( StereoVoEstimator( calibration, options ),
                  std::invalid_argument );
  }
}

TEST( OnlineGyroBiasFixedPose, MatchesFrozenBiasOracle )
{
  const std::array<Eigen::Vector3d, 14> expected{
      Eigen::Vector3d{ 0.012061728029528, -0.018092592044293,
                       0.025123449878755 },
      Eigen::Vector3d{ 0.012123576676337, -0.018185365014506,
                       0.025247150992009 },
      Eigen::Vector3d{ 0.012209001999483, -0.018313502999225,
                       0.025418003097272 },
      Eigen::Vector3d{ 0.012303429322112, -0.018455143983168,
                       0.025606858299808 },
      Eigen::Vector3d{ 0.012401285966853, -0.018601928950280,
                       0.025802571802151 },
      Eigen::Vector3d{ 0.012500428578447, -0.018750642867671,
                       0.026000857106645 },
      Eigen::Vector3d{ 0.012599999768489, -0.018899999652733,
                       0.026199999517784 },
      Eigen::Vector3d{ 0.012699570727019, -0.019049356090529,
                       0.026399141446707 },
      Eigen::Vector3d{ 0.012798712412569, -0.019198068618854,
                       0.026597424822338 },
      Eigen::Vector3d{ 0.012896566510688, -0.019344849766032,
                       0.026793133020306 },
      Eigen::Vector3d{ 0.012990987119495, -0.019486480679243,
                       0.026981974238579 },
      Eigen::Vector3d{ 0.013076394847798, -0.019614592271697,
                       0.027152789695432 },
      Eigen::Vector3d{ 0.013138197423899, -0.019707296135849,
                       0.027276394847716 },
      Eigen::Vector3d{ 0.013138197423899, -0.019707296135849,
                       0.027276394847716 },
  };

  std::array<Eigen::Vector3d, 14> bias_hats;
  bias_hats.fill( Eigen::Vector3d::Zero() );
  gtsam::Values optimized;
  std::size_t   extra_rounds = 0U;
  for ( std::size_t round = 0U; round < 3U; ++round )
  {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               values;
    for ( std::size_t index = 0U; index < bias_hats.size(); ++index )
    {
      const gtsam::Key pose_key = gtsam::Symbol( 'x', index );
      const gtsam::Key bias_key = gtsam::Symbol( 'g', index );
      graph.emplace_shared<gtsam::NonlinearEquality<gtsam::Pose3>>(
          pose_key, gtsam::Pose3{} );
      values.insert( pose_key, gtsam::Pose3{} );
      values.insert( bias_key, bias_hats[ index ] );
    }

    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        gtsam::Symbol( 'g', 0U ), Eigen::Vector3d::Zero(),
        gtsam::noiseModel::Isotropic::Sigma( 3, 0.1 ) );
    for ( std::size_t interval_index = 0U; interval_index < 13U;
          ++interval_index )
    {
      const GyroInterval       source = makeGyroInterval( interval_index + 1U );
      const GyroIntervalResult reduced =
          validateGyroInterval( source.m_samples );
      const ValidatedGyroInterval& interval = requireValidated( reduced );
      const auto                   pim      = preintegrateGyroInterval(
          interval, bias_hats[ interval_index ], 1e-4 );
      graph.emplace_shared<GyroRotationFactor>(
          gtsam::Symbol( 'x', interval_index ),
          gtsam::Symbol( 'x', interval_index + 1U ),
          gtsam::Symbol( 'g', interval_index ), pim );
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
          gtsam::Symbol( 'g', interval_index ),
          gtsam::Symbol( 'g', interval_index + 1U ),
          Eigen::Vector3d::Zero(),
          gtsam::noiseModel::Isotropic::Sigma( 3, 1e-3 * std::sqrt( 0.1 ) ) );
    }

    optimized =
        gtsam::LevenbergMarquardtOptimizer( graph, values ).optimize();
    bool rebuild = false;
    for ( std::size_t index = 0U; index < bias_hats.size(); ++index )
    {
      const Eigen::Vector3d candidate = optimized.at<gtsam::Vector3>(
          gtsam::Symbol( 'g', index ) );
      if ( index < 13U &&
           ( candidate - bias_hats[ index ] ).cwiseAbs().maxCoeff() >
               1e-3 )
      {
        rebuild = true;
      }
      bias_hats[ index ] = candidate;
    }
    if ( !rebuild )
    {
      break;
    }
    ++extra_rounds;
  }

  EXPECT_EQ( extra_rounds, 1U );
  for ( std::size_t index = 0U; index < expected.size(); ++index )
  {
    const Eigen::Vector3d actual = optimized.at<gtsam::Vector3>(
        gtsam::Symbol( 'g', index ) );
    EXPECT_LE( ( actual - expected[ index ] ).cwiseAbs().maxCoeff(), 1e-8 )
        << "G" << index;
  }
  const Eigen::Vector3d terminal = optimized.at<gtsam::Vector3>(
      gtsam::Symbol( 'g', 13U ) );
  const Eigen::Vector3d observed = optimized.at<gtsam::Vector3>(
      gtsam::Symbol( 'g', 12U ) );
  EXPECT_LE( ( terminal - observed ).cwiseAbs().maxCoeff(), 1e-12 );
}

TEST( GyroBiasInitialValue, SelectsExactLinkAndComponentRootSources )
{
  const Eigen::Vector3d provisional{ 0.03125, -0.0625, 0.125 };
  const Eigen::Vector3d committed{ -0.25, 0.5, -1.0 };
  const Eigen::Vector3d prior{ 1.5, -2.0, 0.75 };

  const auto selected_provisional = selectGyroBiasInitialValue(
      GyroBiasInitialKind::kExactLink, provisional, committed, prior );
  ASSERT_TRUE(
      std::holds_alternative<Eigen::Vector3d>( selected_provisional ) );
  EXPECT_TRUE( ( std::get<Eigen::Vector3d>( selected_provisional ).array() ==
                 provisional.array() )
                   .all() );

  const auto selected_committed = selectGyroBiasInitialValue(
      GyroBiasInitialKind::kExactLink, std::nullopt, committed, prior );
  ASSERT_TRUE(
      std::holds_alternative<Eigen::Vector3d>( selected_committed ) );
  EXPECT_TRUE( ( std::get<Eigen::Vector3d>( selected_committed ).array() ==
                 committed.array() )
                   .all() );

  const auto selected_prior = selectGyroBiasInitialValue(
      GyroBiasInitialKind::kComponentRoot, provisional, committed, prior );
  ASSERT_TRUE( std::holds_alternative<Eigen::Vector3d>( selected_prior ) );
  EXPECT_TRUE( ( std::get<Eigen::Vector3d>( selected_prior ).array() ==
                 prior.array() )
                   .all() );
}

TEST( GyroBiasInitialValue, RejectsMissingOrNonFiniteSelectedSource )
{
  const Eigen::Vector3d prior{ 1.5, -2.0, 0.75 };
  const auto            missing = selectGyroBiasInitialValue(
      GyroBiasInitialKind::kExactLink, std::nullopt, std::nullopt, prior );
  ASSERT_TRUE( std::holds_alternative<GyroBiasInitialErrorCode>( missing ) );
  EXPECT_EQ( std::get<GyroBiasInitialErrorCode>( missing ),
             GyroBiasInitialErrorCode::kMissingPredecessor );

  Eigen::Vector3d bad_provisional{ 0.03125, -0.0625, 0.125 };
  bad_provisional.x()           = std::numeric_limits<double>::quiet_NaN();
  const auto nonfinite_selected = selectGyroBiasInitialValue(
      GyroBiasInitialKind::kExactLink, bad_provisional,
      Eigen::Vector3d{ -0.25, 0.5, -1.0 }, prior );
  ASSERT_TRUE( std::holds_alternative<GyroBiasInitialErrorCode>(
      nonfinite_selected ) );
  EXPECT_EQ( std::get<GyroBiasInitialErrorCode>( nonfinite_selected ),
             GyroBiasInitialErrorCode::kNonFiniteSelectedValue );

  Eigen::Vector3d bad_prior = prior;
  bad_prior.z()             = std::numeric_limits<double>::infinity();
  const auto nonfinite_root = selectGyroBiasInitialValue(
      GyroBiasInitialKind::kComponentRoot,
      Eigen::Vector3d{ 0.03125, -0.0625, 0.125 },
      Eigen::Vector3d{ -0.25, 0.5, -1.0 }, bad_prior );
  ASSERT_TRUE(
      std::holds_alternative<GyroBiasInitialErrorCode>( nonfinite_root ) );
  EXPECT_EQ( std::get<GyroBiasInitialErrorCode>( nonfinite_root ),
             GyroBiasInitialErrorCode::kNonFiniteSelectedValue );
}

TEST( GyroIntervalReducer, FreezesEndpointTrapezoidAndExactDuration )
{
  std::vector<ImuMeasurement> samples{
      reducerSample( 100, Eigen::Vector3d{ 1.0, 2.0, 3.0 } ),
      reducerSample( 50'000'100, Eigen::Vector3d{ 3.0, 4.0, 5.0 } ),
      reducerSample( 100'000'100, Eigen::Vector3d{ 5.0, 6.0, 7.0 } ),
  };
  const auto result = validateGyroInterval(
      samples, phad::common::Timestamp{ 100 },
      phad::common::Timestamp{ 100'000'100 } );
  const ValidatedGyroInterval& interval = requireValidated( result );
  ASSERT_EQ( interval.m_samples.size(), 3U );
  ASSERT_EQ( interval.m_steps.size(), 2U );
  EXPECT_EQ( interval.m_t_prev.nanoseconds(), 100 );
  EXPECT_EQ( interval.m_t_curr.nanoseconds(), 100'000'100 );
  EXPECT_EQ( interval.m_duration_ns, 100'000'000 );
  EXPECT_EQ( interval.m_steps[ 0 ].m_dt_ns, 50'000'000 );
  EXPECT_EQ( interval.m_steps[ 1 ].m_dt_ns, 50'000'000 );
  EXPECT_EQ( interval.m_steps[ 0 ].m_dt_s, 0.05 );
  EXPECT_EQ( interval.m_steps[ 1 ].m_dt_s, 0.05 );
  EXPECT_TRUE( ( interval.m_steps[ 0 ].m_omega_mean_radps.array() ==
                 Eigen::Vector3d{ 2.0, 3.0, 4.0 }.array() )
                   .all() );
  EXPECT_TRUE( ( interval.m_steps[ 1 ].m_omega_mean_radps.array() ==
                 Eigen::Vector3d{ 4.0, 5.0, 6.0 }.array() )
                   .all() );

  samples[ 0 ].gyro_radps[ 0 ] = 99.0;
  EXPECT_EQ( interval.m_samples[ 0 ].gyro_radps[ 0 ], 1.0 );
}

TEST( GyroIntervalReducer, EnforcesFrozenErrorPrecedence )
{
  const auto       nan = std::numeric_limits<double>::quiet_NaN();
  const std::array one_sample{
      reducerSample( 10, Eigen::Vector3d{ nan, 0.0, 0.0 } ) };
  requireIntervalError(
      validateGyroInterval( one_sample, phad::common::Timestamp{ 0 },
                            phad::common::Timestamp{ 20 } ),
      GyroIntervalErrorCode::kTooFewSamples );

  const std::array wrong_endpoints{
      reducerSample( 10, Eigen::Vector3d{ nan, 0.0, 0.0 } ),
      reducerSample( 10, Eigen::Vector3d::Zero() ) };
  requireIntervalError(
      validateGyroInterval( wrong_endpoints, phad::common::Timestamp{ 0 },
                            phad::common::Timestamp{ 20 } ),
      GyroIntervalErrorCode::kEndpointMismatch );

  const std::array duplicate_and_nan{
      reducerSample( 10, Eigen::Vector3d{ nan, 0.0, 0.0 } ),
      reducerSample( 10, Eigen::Vector3d::Zero() ) };
  requireIntervalError(
      validateGyroInterval( duplicate_and_nan,
                            phad::common::Timestamp{ 10 },
                            phad::common::Timestamp{ 10 } ),
      GyroIntervalErrorCode::kNonIncreasingTimestamp );

  const std::array finite_timestamps{
      reducerSample( 10, Eigen::Vector3d{ nan, 0.0, 0.0 } ),
      reducerSample( 20, Eigen::Vector3d::Zero() ) };
  const GyroIntervalError nonfinite = requireIntervalError(
      validateGyroInterval( finite_timestamps,
                            phad::common::Timestamp{ 10 },
                            phad::common::Timestamp{ 20 } ),
      GyroIntervalErrorCode::kNonFiniteGyro );
  EXPECT_EQ( nonfinite.m_sample_index, 0U );
  EXPECT_EQ( nonfinite.m_timestamp_ns, 10 );
}

TEST( GyroIntervalReducer, RejectsSubtractionAndDurationOverflow )
{
  const std::array subtraction_overflow{
      reducerSample( std::numeric_limits<std::int64_t>::min(),
                     Eigen::Vector3d::Zero() ),
      reducerSample( std::numeric_limits<std::int64_t>::max(),
                     Eigen::Vector3d::Zero() ) };
  const GyroIntervalError subtraction_error = requireIntervalError(
      validateGyroInterval( subtraction_overflow ),
      GyroIntervalErrorCode::kTimestampOverflow );
  EXPECT_EQ( subtraction_error.m_sample_index, 1U );
  EXPECT_EQ( subtraction_error.m_interval_index, 0U );
  EXPECT_EQ( subtraction_error.m_timestamp_ns,
             std::numeric_limits<std::int64_t>::max() );

  const std::array duration_overflow{
      reducerSample( std::numeric_limits<std::int64_t>::min(),
                     Eigen::Vector3d::Zero() ),
      reducerSample( -1, Eigen::Vector3d::Zero() ),
      reducerSample( 0, Eigen::Vector3d::Zero() ) };
  const GyroIntervalError duration_error = requireIntervalError(
      validateGyroInterval( duration_overflow ),
      GyroIntervalErrorCode::kTimestampOverflow );
  EXPECT_FALSE( duration_error.m_sample_index.has_value() );
  EXPECT_EQ( duration_error.m_interval_index, 1U );
  EXPECT_FALSE( duration_error.m_timestamp_ns.has_value() );
}

TEST( GyroIntervalReducer, DerivesFinitePositiveNoiseScales )
{
  const GyroNoiseScalesResult result =
      deriveGyroNoiseScales( 250'000'000, 4e-4, 1e-3, 0.1 );
  ASSERT_TRUE( std::holds_alternative<GyroNoiseScales>( result ) );
  const GyroNoiseScales& scales = std::get<GyroNoiseScales>( result );
  EXPECT_NEAR( scales.m_rotation_variance, 4e-8, 1e-18 );
  EXPECT_NEAR( scales.m_rw_variance, 2.5e-7, 1e-18 );
  EXPECT_NEAR( scales.m_rw_sigma, 5e-4, 1e-18 );
  EXPECT_DOUBLE_EQ( scales.m_prior_variance, 0.1 * 0.1 );

  for ( const auto& bad : {
            deriveGyroNoiseScales( 0, 4e-4, 1e-3, 0.1 ),
            deriveGyroNoiseScales( 250'000'000, 0.0, 1e-3, 0.1 ),
            deriveGyroNoiseScales(
                250'000'000, std::numeric_limits<double>::max(), 1e-3,
                0.1 ),
            deriveGyroNoiseScales(
                250'000'000, 4e-4,
                std::numeric_limits<double>::denorm_min(), 0.1 ),
        } )
  {
    ASSERT_TRUE( std::holds_alternative<GyroIntervalError>( bad ) );
    EXPECT_EQ( std::get<GyroIntervalError>( bad ).m_code,
               GyroIntervalErrorCode::kInvalidNoiseScale );
  }
}

namespace
{

  using phad::estimator::internal::GyroRotationFactor;
  using phad::estimator::internal::preintegrateGyroInterval;

  [[nodiscard]] ValidatedGyroInterval constantGyroInterval(
      std::int64_t duration_ns, const Eigen::Vector3d& measured )
  {
    const std::array         samples{ reducerSample( 0, measured ),
                              reducerSample( duration_ns, measured ) };
    const GyroIntervalResult result = validateGyroInterval( samples );
    EXPECT_TRUE( std::holds_alternative<ValidatedGyroInterval>( result ) );
    return std::get<ValidatedGyroInterval>( result );
  }

  [[nodiscard]] double rotationDistance( const gtsam::Rot3& first,
                                         const gtsam::Rot3& second )
  {
    return gtsam::Rot3::Logmap( first.between( second ) ).norm();
  }

  [[nodiscard]] std::vector<ImuMeasurement> noncommutingGyroSamples(
      const Eigen::Vector3d& bias_hat )
  {
    std::vector<ImuMeasurement> samples;
    samples.reserve( 41U );
    for ( std::int64_t index = 0; index <= 40; ++index )
    {
      const std::int64_t    timestamp_ns = index * 10'000'000;
      const double          time_s       = static_cast<double>( index ) * 0.01;
      const Eigen::Vector3d true_omega{
          1.1 * std::cos( 0.7 * time_s ), 0.7,
          1.1 * std::sin( 0.7 * time_s ) };
      samples.push_back(
          reducerSample( timestamp_ns, true_omega + bias_hat ) );
    }
    return samples;
  }

}  // namespace

TEST( GyroRotationFactor, MatchesOfficialResidualAndPoseJacobians )
{
  constexpr double            kNumericalStep = 1e-7;
  const Eigen::Vector3d       omega{ 0.3, -0.22, 0.17 };
  const Eigen::Vector3d       bias{ 0.012, -0.018, 0.025 };
  const ValidatedGyroInterval interval =
      constantGyroInterval( 1'000'000'000, omega + bias );
  const auto               pim   = preintegrateGyroInterval( interval, bias, 4e-4 );
  const gtsam::Key         key_i = gtsam::Symbol( 'x', 0U );
  const gtsam::Key         key_j = gtsam::Symbol( 'x', 1U );
  const gtsam::Key         key_g = gtsam::Symbol( 'g', 0U );
  const GyroRotationFactor factor( key_i, key_j, key_g, pim );

  const gtsam::Rot3 rotation_i =
      gtsam::Rot3::Expmap( Eigen::Vector3d{ 0.2, -0.1, 0.05 } );
  const gtsam::Rot3 rotation_j =
      rotation_i.compose( gtsam::Rot3::Expmap( omega ) );
  const gtsam::Pose3 pose_i( rotation_i,
                             gtsam::Point3( 1.0, -2.0, 3.0 ) );
  const gtsam::Pose3 pose_j( rotation_j,
                             gtsam::Point3( -4.0, 5.0, -6.0 ) );

  gtsam::Matrix       h_i;
  gtsam::Matrix       h_j;
  gtsam::Matrix       h_g;
  const gtsam::Vector residual =
      factor.evaluateError( pose_i, pose_j, bias, h_i, h_j, h_g );
  EXPECT_LE( residual.norm(), 1e-12 );
  ASSERT_EQ( h_i.rows(), 3 );
  ASSERT_EQ( h_i.cols(), 6 );
  ASSERT_EQ( h_j.rows(), 3 );
  ASSERT_EQ( h_j.cols(), 6 );
  EXPECT_TRUE( h_i.rightCols<3>().isZero( 0.0 ) );
  EXPECT_TRUE( h_j.rightCols<3>().isZero( 0.0 ) );

  const gtsam::AHRSFactor official( key_i, key_j, key_g, pim );
  gtsam::Matrix           official_h_i;
  gtsam::Matrix           official_h_j;
  gtsam::Matrix           official_h_g;
  const gtsam::Vector     official_residual = official.evaluateError(
      rotation_i, rotation_j, bias, official_h_i, official_h_j,
      official_h_g );
  EXPECT_TRUE( residual.isApprox( official_residual, 0.0 ) );
  EXPECT_TRUE( h_i.leftCols<3>().isApprox( official_h_i, 0.0 ) );
  EXPECT_TRUE( h_j.leftCols<3>().isApprox( official_h_j, 0.0 ) );
  EXPECT_TRUE( h_g.isApprox( official_h_g, 0.0 ) );

  const std::function<gtsam::Vector(
      const gtsam::Pose3&, const gtsam::Pose3&, const gtsam::Vector3& )>
      evaluate = [ &factor ]( const gtsam::Pose3&   first,
                              const gtsam::Pose3&   second,
                              const gtsam::Vector3& candidate ) {
        return factor.evaluateError( first, second, candidate );
      };
  const gtsam::Matrix numerical_h_i =
      gtsam::numericalDerivative31<gtsam::Vector, gtsam::Pose3,
                                   gtsam::Pose3, gtsam::Vector3>(
          evaluate, pose_i, pose_j, bias, kNumericalStep );
  const gtsam::Matrix numerical_h_j =
      gtsam::numericalDerivative32<gtsam::Vector, gtsam::Pose3,
                                   gtsam::Pose3, gtsam::Vector3>(
          evaluate, pose_i, pose_j, bias, kNumericalStep );
  const gtsam::Matrix numerical_h_g =
      gtsam::numericalDerivative33<gtsam::Vector, gtsam::Pose3,
                                   gtsam::Pose3, gtsam::Vector3>(
          evaluate, pose_i, pose_j, bias, kNumericalStep );
  EXPECT_LE( ( h_i - numerical_h_i ).cwiseAbs().maxCoeff(), 1e-6 );
  EXPECT_LE( ( h_j - numerical_h_j ).cwiseAbs().maxCoeff(), 1e-6 );
  EXPECT_LE( ( h_g - numerical_h_g ).cwiseAbs().maxCoeff(), 1e-6 );

  const gtsam::Pose3 translated_i( rotation_i,
                                   gtsam::Point3( 20.0, 30.0, 40.0 ) );
  const gtsam::Pose3 translated_j( rotation_j,
                                   gtsam::Point3( -50.0, -60.0, -70.0 ) );
  EXPECT_LE( ( factor.evaluateError( translated_i, translated_j, bias ) -
               residual )
                 .norm(),
             1e-12 );
}

TEST( GyroRotationFactor, RejectsFrozenSo3Mutants )
{
  const Eigen::Vector3d initial{ 0.2, -0.1, 0.05 };
  const Eigen::Vector3d omega{ 0.3, -0.22, 0.17 };
  const Eigen::Vector3d bias{ 0.012, -0.018, 0.025 };
  const Eigen::Vector3d measured   = omega + bias;
  const gtsam::Rot3     rotation_i = gtsam::Rot3::Expmap( initial );
  const gtsam::Rot3     delta      = gtsam::Rot3::Expmap( omega );
  const gtsam::Rot3     rotation_j = rotation_i.compose( delta );
  const gtsam::Rot3     actual     = rotation_i.between( rotation_j );

  const auto mutant_norm = [ &actual ]( const gtsam::Rot3& predicted ) {
    return rotationDistance( predicted, actual );
  };
  const double wrong_bias_sign =
      mutant_norm( gtsam::Rot3::Expmap( measured + bias ) );
  const double omit_bias  = mutant_norm( gtsam::Rot3::Expmap( measured ) );
  const double rad_as_deg = mutant_norm(
      gtsam::Rot3::Expmap( omega * ( std::numbers::pi / 180.0 ) ) );
  const double deg_as_rad = mutant_norm(
      gtsam::Rot3::Expmap( omega * ( 180.0 / std::numbers::pi ) ) );
  const double      visual_inverse = rotationDistance( delta, actual.inverse() );
  const gtsam::Rot3 left_absolute  = delta.compose( rotation_i );
  const double      left_compose =
      rotationDistance( delta, rotation_i.between( left_absolute ) );
  const gtsam::Rot3 rotation_z_90 =
      gtsam::Rot3::Rz( std::numbers::pi / 2.0 );
  const Eigen::Vector3d wrong_frame_omega =
      rotation_z_90.matrix().transpose() * measured - bias;
  const double wrong_frame =
      mutant_norm( gtsam::Rot3::Expmap( wrong_frame_omega ) );

  EXPECT_NEAR( wrong_bias_sign, 0.06601234538, 1e-10 );
  EXPECT_NEAR( omit_bias, 0.03300615423, 1e-10 );
  EXPECT_NEAR( rad_as_deg, 0.40188442470, 1e-10 );
  EXPECT_NEAR( deg_as_rad, 2.10645984131, 1e-10 );
  EXPECT_NEAR( visual_inverse, 0.81804645345, 1e-10 );
  EXPECT_NEAR( left_compose, 0.02412947757, 1e-10 );
  EXPECT_NEAR( wrong_frame, 0.55273358029, 1e-10 );
  for ( const double mutant : { wrong_bias_sign, omit_bias, rad_as_deg,
                                deg_as_rad, visual_inverse, left_compose,
                                wrong_frame } )
  {
    EXPECT_GE( mutant, 0.02 );
  }
}

TEST( GyroRotationFactor, FreezesNoiseWhiteningAndCost )
{
  const Eigen::Vector3d       zero = Eigen::Vector3d::Zero();
  const ValidatedGyroInterval interval =
      constantGyroInterval( 250'000'000, zero );
  const auto               pim = preintegrateGyroInterval( interval, zero, 4e-4 );
  const GyroRotationFactor factor( gtsam::Symbol( 'x', 0U ),
                                   gtsam::Symbol( 'x', 1U ),
                                   gtsam::Symbol( 'g', 0U ), pim );
  EXPECT_LE( ( pim.preintMeasCov() -
               4e-8 * Eigen::Matrix3d::Identity() )
                 .cwiseAbs()
                 .maxCoeff(),
             1e-18 );

  const Eigen::Vector3d rotation_residual{ 1e-4, -2e-4, 3e-4 };
  const Eigen::Vector3d rw_residual{ 2e-4, -1e-4, 3e-4 };
  const gtsam::Vector   rotation_whitened =
      factor.noiseModel()->whiten( rotation_residual );
  const double rw_sigma = 1e-3 * std::sqrt( 0.25 );
  const auto   rw_model =
      gtsam::noiseModel::Diagonal::Sigmas(
          Eigen::Vector3d::Constant( rw_sigma ) );
  const gtsam::Vector rw_whitened = rw_model->whiten( rw_residual );

  EXPECT_TRUE( rotation_whitened.isApprox(
      Eigen::Vector3d{ 0.5, -1.0, 1.5 }, 1e-12 ) );
  EXPECT_TRUE( rw_whitened.isApprox(
      Eigen::Vector3d{ 0.4, -0.2, 0.6 }, 1e-12 ) );
  EXPECT_NEAR( rotation_whitened.squaredNorm(), 3.5, 1e-12 );
  EXPECT_NEAR( rw_whitened.squaredNorm(), 0.56, 1e-12 );
  EXPECT_NEAR( 0.5 * rotation_whitened.squaredNorm(), 1.75, 1e-12 );
  EXPECT_NEAR( 0.5 * rw_whitened.squaredNorm(), 0.28, 1e-12 );
  EXPECT_NEAR( 0.5 * ( rotation_whitened.squaredNorm() +
                       rw_whitened.squaredNorm() ),
               2.03, 1e-12 );
}

TEST( GyroRotationFactor, ExactZeroGraphCostIsNumericalZero )
{
  const Eigen::Vector3d       zero = Eigen::Vector3d::Zero();
  const ValidatedGyroInterval interval =
      constantGyroInterval( 100'000'000, zero );
  const auto                                 pim        = preintegrateGyroInterval( interval, zero, 1e-4 );
  const gtsam::Key                           pose_i_key = gtsam::Symbol( 'x', 0U );
  const gtsam::Key                           pose_j_key = gtsam::Symbol( 'x', 1U );
  const gtsam::Key                           bias_i_key = gtsam::Symbol( 'g', 0U );
  const gtsam::Key                           bias_j_key = gtsam::Symbol( 'g', 1U );
  const GyroRotationFactor                   rotation_factor( pose_i_key, pose_j_key,
                                                              bias_i_key, pim );
  const gtsam::BetweenFactor<gtsam::Vector3> rw_factor(
      bias_i_key, bias_j_key, zero,
      gtsam::noiseModel::Isotropic::Sigma( 3, 1e-3 * std::sqrt( 0.1 ) ) );
  const gtsam::PriorFactor<gtsam::Vector3> root_factor(
      bias_i_key, zero,
      gtsam::noiseModel::Isotropic::Sigma( 3, 0.1 ) );

  gtsam::Values values;
  values.insert( pose_i_key, gtsam::Pose3{} );
  values.insert( pose_j_key, gtsam::Pose3{} );
  values.insert( bias_i_key, zero );
  values.insert( bias_j_key, zero );
  const double total_cost = rotation_factor.error( values ) +
                            rw_factor.error( values ) +
                            root_factor.error( values );
  EXPECT_LE( total_cost, 1e-18 );
}

TEST( GyroRotationFactor, CachedCorrectionMatchesFreshReintegrationDomain )
{
  constexpr double                  kDomain         = 1e-3;
  constexpr double                  kLatticeStep    = 1e-4;
  constexpr double                  kDerivativeStep = 1e-7;
  const Eigen::Vector3d             bias_hat{ 0.12, -0.08, 0.05 };
  const std::vector<ImuMeasurement> samples =
      noncommutingGyroSamples( bias_hat );
  const GyroIntervalResult     reduced  = validateGyroInterval( samples );
  const ValidatedGyroInterval& interval = requireValidated( reduced );
  const auto                   cached =
      preintegrateGyroInterval( interval, bias_hat, 1e-4 );

  double max_rotation_error = 0.0;
  for ( int x = -10; x <= 10; ++x )
  {
    for ( int y = -10; y <= 10; ++y )
    {
      for ( int z = -10; z <= 10; ++z )
      {
        const Eigen::Vector3d delta{
            static_cast<double>( x ) * kLatticeStep,
            static_cast<double>( y ) * kLatticeStep,
            static_cast<double>( z ) * kLatticeStep };
        EXPECT_LE( delta.cwiseAbs().maxCoeff(), kDomain );
        const gtsam::Rot3 first_order =
            cached.biascorrectedDeltaRij( delta );
        const auto fresh = preintegrateGyroInterval(
            interval, bias_hat + delta, 1e-4 );
        max_rotation_error =
            std::max( max_rotation_error,
                      rotationDistance( first_order, fresh.deltaRij() ) );
      }
    }
  }
  EXPECT_LE( max_rotation_error, 3e-8 );
  EXPECT_NEAR( max_rotation_error, 2.0431967621531336e-08, 1e-12 );

  double max_jacobian_error = 0.0;
  for ( const double x : { -kDomain, 0.0, kDomain } )
  {
    for ( const double y : { -kDomain, 0.0, kDomain } )
    {
      for ( const double z : { -kDomain, 0.0, kDomain } )
      {
        const Eigen::Vector3d candidate = bias_hat +
                                          Eigen::Vector3d{ x, y, z };
        const auto candidate_pim =
            preintegrateGyroInterval( interval, candidate, 1e-4 );
        gtsam::Matrix3 analytic;
        static_cast<void>( candidate_pim.biascorrectedDeltaRij(
            Eigen::Vector3d::Zero(), analytic ) );
        const std::function<gtsam::Rot3( const gtsam::Vector3& )>
            full_reintegration = [ &interval ](
                                     const gtsam::Vector3& bias ) {
              return preintegrateGyroInterval( interval, bias, 1e-4 )
                  .deltaRij();
            };
        const gtsam::Matrix3 numerical =
            gtsam::numericalDerivative11<gtsam::Rot3, gtsam::Vector3>(
                full_reintegration, candidate, kDerivativeStep );
        max_jacobian_error =
            std::max( max_jacobian_error,
                      ( analytic - numerical ).cwiseAbs().maxCoeff() );
      }
    }
  }
  EXPECT_LE( max_jacobian_error, 1e-7 );
}
