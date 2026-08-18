#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
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
