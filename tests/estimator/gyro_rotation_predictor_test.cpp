#include "phad/estimator/gyro_rotation_predictor.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <variant>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"

namespace
{

  using phad::common::Timestamp;
  using phad::estimator::GyroRotationError;
  using phad::estimator::GyroRotationErrorCode;
  using phad::estimator::GyroRotationPrediction;
  using phad::estimator::GyroRotationResult;
  using phad::estimator::integrateGyroRotation;
  using phad::sensor::ImuMeasurement;

  constexpr std::array<std::int64_t, 8> kIrregularTimestamps{
      0, 7'000'000, 31'000'000, 60'000'000,
      113'000'000, 191'000'000, 260'000'000, 400'000'000 };
  const Eigen::Vector3d kBias{ 0.12, -0.08, 0.05 };

  ImuMeasurement sample( std::int64_t           timestamp_ns,
                         const Eigen::Vector3d& gyro_radps,
                         std::array<double, 3>  accel_mps2 = {} )
  {
    return ImuMeasurement{
        .timestamp  = Timestamp{ timestamp_ns },
        .accel_mps2 = accel_mps2,
        .gyro_radps = { gyro_radps.x(), gyro_radps.y(), gyro_radps.z() },
    };
  }

  Eigen::Matrix3d skew( const Eigen::Vector3d& vector )
  {
    Eigen::Matrix3d result;
    result << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return result;
  }

  Eigen::Matrix3d rodrigues( const Eigen::Vector3d& vector )
  {
    const double          angle = vector.norm();
    const Eigen::Matrix3d K     = skew( vector );
    if ( angle == 0.0 )
    {
      return Eigen::Matrix3d::Identity();
    }
    return Eigen::Matrix3d::Identity() + ( std::sin( angle ) / angle ) * K +
           ( ( 1.0 - std::cos( angle ) ) / ( angle * angle ) ) * K * K;
  }

  double rotationError( const Eigen::Matrix3d& reference,
                        const Eigen::Matrix3d& actual )
  {
    const Eigen::Matrix3d difference = reference.transpose() * actual;
    const double          sine =
        0.5 *
        Eigen::Vector3d{ difference( 2, 1 ) - difference( 1, 2 ),
                         difference( 0, 2 ) - difference( 2, 0 ),
                         difference( 1, 0 ) - difference( 0, 1 ) }
            .norm();
    const double cosine = 0.5 * ( difference.trace() - 1.0 );
    return std::atan2( sine, cosine );
  }

  void expectRotation( const Eigen::Matrix3d& rotation )
  {
    EXPECT_TRUE( rotation.allFinite() );
    EXPECT_LE( ( rotation.transpose() * rotation - Eigen::Matrix3d::Identity() )
                   .norm(),
               1e-12 );
    EXPECT_LE( std::abs( rotation.determinant() - 1.0 ), 1e-12 );
  }

  GyroRotationPrediction requirePrediction(
      const GyroRotationResult& result )
  {
    EXPECT_TRUE( std::holds_alternative<GyroRotationPrediction>( result ) );
    return std::get<GyroRotationPrediction>( result );
  }

  GyroRotationError requireError( const GyroRotationResult& result,
                                  GyroRotationErrorCode     expected_code )
  {
    EXPECT_TRUE( std::holds_alternative<GyroRotationError>( result ) );
    const auto& error = std::get<GyroRotationError>( result );
    EXPECT_EQ( error.code, expected_code );
    return error;
  }

  std::vector<ImuMeasurement> irregularSamples(
      const auto& measured_gyro )
  {
    std::vector<ImuMeasurement> samples;
    samples.reserve( kIrregularTimestamps.size() );
    for ( const std::int64_t timestamp_ns : kIrregularTimestamps )
    {
      const double time_s = static_cast<double>( timestamp_ns ) * 1e-9;
      samples.push_back( sample( timestamp_ns, measured_gyro( time_s ) ) );
    }
    return samples;
  }

  Eigen::Matrix3d exactNoncommutingRotation()
  {
    Eigen::Matrix3d result;
    result << 0.96105543831077094792, 0.0, 0.27635564856411373332,
        0.11771077711736598480, 0.90475166321996341717,
        -0.40935143929285955846, -0.25003323267861359515,
        0.42593946506599960277, 0.86951650625826096424;
    return result;
  }

  Eigen::Matrix3d exactConstantRotation()
  {
    Eigen::Matrix3d result;
    result << 0x1.f5d6d554f2303p-1, -0x1.e5ecfd3e7bb9fp-4,
        -0x1.4541aed6189d5p-3, 0x1.86107ff3762f5p-4,
        0x1.f8093833ffa60p-1, -0x1.2e5f8ced3715cp-3,
        0x1.6411b289a389ep-3, 0x1.0965eee290604p-3,
        0x1.f3d15013cca11p-1;
    return result;
  }

  Eigen::Matrix3d exactLinearRotation()
  {
    Eigen::Matrix3d result;
    result << 0.98511582940562973936, -0.15923660095246974381,
        -0.06473412988186461127, 0.14732926447697353530,
        0.97618532704900758298, -0.15923660095246974381,
        0.08854880283285702829, 0.14732926447697353530,
        0.98511582940562973936;
    return result;
  }

  Eigen::Matrix3d exactSharedEndpointRotation()
  {
    Eigen::Matrix3d result;
    result << 0.99949032240964221999, -0.02875400367993384841,
        -0.01386732424960914419, 0.02834626160764762440,
        0.99918451585542755198, -0.02875400367993384841,
        0.01468280839418159221, 0.02834626160764762440,
        0.99949032240964221999;
    return result;
  }

  std::vector<ImuMeasurement> noncommutingSamples( std::int64_t step_ns )
  {
    std::vector<ImuMeasurement> samples;
    for ( std::int64_t timestamp_ns = 0; timestamp_ns <= 400'000'000;
          timestamp_ns += step_ns )
    {
      const double          time_s = static_cast<double>( timestamp_ns ) * 1e-9;
      const Eigen::Vector3d true_gyro{
          1.1 * std::cos( 0.7 * time_s ), 0.7,
          1.1 * std::sin( 0.7 * time_s ) };
      samples.push_back( sample( timestamp_ns, true_gyro + kBias ) );
    }
    return samples;
  }

  Eigen::Matrix3d integrateMutant( std::span<const ImuMeasurement> samples,
                                   const Eigen::Vector3d&          bias,
                                   bool reverse_compose, bool left_zoh,
                                   double scale, bool double_bias,
                                   bool sign_flip )
  {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    for ( std::size_t index = 0; index + 1 < samples.size(); ++index )
    {
      const auto toVector = []( const ImuMeasurement& measurement ) {
        return Eigen::Vector3d{ measurement.gyro_radps[ 0 ],
                                measurement.gyro_radps[ 1 ],
                                measurement.gyro_radps[ 2 ] };
      };
      Eigen::Vector3d omega = left_zoh
                                  ? toVector( samples[ index ] )
                                  : 0.5 * toVector( samples[ index ] ) +
                                        0.5 * toVector( samples[ index + 1 ] );
      omega -= bias;
      if ( double_bias )
      {
        omega -= bias;
      }
      omega *= scale * ( sign_flip ? -1.0 : 1.0 );
      const std::int64_t dt_ns =
          samples[ index + 1 ].timestamp.nanoseconds() -
          samples[ index ].timestamp.nanoseconds();
      const Eigen::Matrix3d increment =
          rodrigues( omega * ( static_cast<double>( dt_ns ) * 1e-9 ) );
      rotation = reverse_compose ? increment * rotation : rotation * increment;
    }
    return rotation;
  }

  TEST( GyroRotationPredictor, CommutingCasesMatchIndependentOracle )
  {
    const Eigen::Vector3d rate{ 0.35, -0.42, 0.27 };
    const Eigen::Vector3d axis       = Eigen::Vector3d{ 2.0, -1.0, 2.0 } / 3.0;
    const auto            stationary = irregularSamples(
        []( double ) { return kBias; } );
    const auto constant = irregularSamples(
        [ & ]( double ) { return rate + kBias; } );
    const auto linear = irregularSamples( [ & ]( double time_s ) {
      return axis * ( 0.3 + 1.4 * time_s ) + kBias;
    } );

    const auto& stationary_prediction =
        requirePrediction( integrateGyroRotation( stationary, kBias ) );
    const auto& constant_prediction =
        requirePrediction( integrateGyroRotation( constant, kBias ) );
    const auto& linear_prediction =
        requirePrediction( integrateGyroRotation( linear, kBias ) );

    EXPECT_EQ( stationary_prediction.duration_ns, 400'000'000 );
    EXPECT_EQ( constant_prediction.duration_ns, 400'000'000 );
    EXPECT_EQ( linear_prediction.duration_ns, 400'000'000 );
    EXPECT_LE( rotationError( Eigen::Matrix3d::Identity(),
                              stationary_prediction.delta_R_i_j ),
               2e-12 );
    EXPECT_LE( rotationError( exactConstantRotation(),
                              constant_prediction.delta_R_i_j ),
               2e-12 );
    EXPECT_LE( rotationError( exactLinearRotation(),
                              linear_prediction.delta_R_i_j ),
               2e-12 );
    expectRotation( stationary_prediction.delta_R_i_j );
    expectRotation( constant_prediction.delta_R_i_j );
    expectRotation( linear_prediction.delta_R_i_j );
  }

  TEST( GyroRotationPredictor, NoncommutingTrajectoryIsSecondOrder )
  {
    constexpr std::array<std::int64_t, 3> kSteps{ 40'000'000, 20'000'000,
                                                  10'000'000 };
    constexpr std::array<double, 3>       kUpper{ 9.6e-5, 2.4e-5, 6.0e-6 };
    std::array<double, 3>                 errors{};
    for ( std::size_t index = 0; index < kSteps.size(); ++index )
    {
      const auto  samples = noncommutingSamples( kSteps[ index ] );
      const auto& prediction =
          requirePrediction( integrateGyroRotation( samples, kBias ) );
      EXPECT_EQ( prediction.duration_ns, 400'000'000 );
      expectRotation( prediction.delta_R_i_j );
      errors[ index ] = rotationError( exactNoncommutingRotation(),
                                       prediction.delta_R_i_j );
      EXPECT_LE( errors[ index ], kUpper[ index ] );
      const double mutant_error = rotationError(
          exactNoncommutingRotation(),
          integrateMutant( samples, kBias, true, false, 1.0, false, false ) );
      EXPECT_GE( mutant_error, 8e-3 );
    }
    EXPECT_GE( errors[ 0 ] / errors[ 1 ], 3.8 );
    EXPECT_LE( errors[ 0 ] / errors[ 1 ], 4.2 );
    EXPECT_GE( errors[ 1 ] / errors[ 2 ], 3.8 );
    EXPECT_LE( errors[ 1 ] / errors[ 2 ], 4.2 );
  }

  TEST( GyroRotationPredictor, RejectsFiveAdditionalMutants )
  {
    const Eigen::Vector3d rate{ 0.35, -0.42, 0.27 };
    const Eigen::Vector3d axis       = Eigen::Vector3d{ 2.0, -1.0, 2.0 } / 3.0;
    const auto            stationary = irregularSamples(
        []( double ) { return kBias; } );
    const auto constant = irregularSamples(
        [ & ]( double ) { return rate + kBias; } );
    const auto            linear             = irregularSamples( [ & ]( double time_s ) {
      return axis * ( 0.3 + 1.4 * time_s ) + kBias;
    } );
    const Eigen::Matrix3d constant_reference = exactConstantRotation();

    EXPECT_LE( rotationError(
                   exactLinearRotation(),
                   requirePrediction( integrateGyroRotation( linear, kBias ) )
                       .delta_R_i_j ),
               2e-12 );
    EXPECT_GE( rotationError( exactLinearRotation(),
                              integrateMutant( linear, kBias, false, true, 1.0,
                                               false, false ) ),
               2e-2 );
    EXPECT_LE( rotationError(
                   Eigen::Matrix3d::Identity(),
                   requirePrediction( integrateGyroRotation( stationary, kBias ) )
                       .delta_R_i_j ),
               2e-12 );
    EXPECT_GE( rotationError( Eigen::Matrix3d::Identity(),
                              integrateMutant( stationary, kBias, false, false,
                                               1.0, true, false ) ),
               5e-2 );
    EXPECT_LE( rotationError(
                   constant_reference,
                   requirePrediction( integrateGyroRotation( constant, kBias ) )
                       .delta_R_i_j ),
               2e-12 );
    EXPECT_GE( rotationError(
                   constant_reference,
                   integrateMutant( constant, kBias, false, false,
                                    std::numbers::pi / 180.0, false, false ) ),
               0.20 );
    EXPECT_GE( rotationError(
                   constant_reference,
                   integrateMutant( constant, kBias, false, false,
                                    180.0 / std::numbers::pi, false, false ) ),
               1.0 );
    EXPECT_GE( rotationError( constant_reference,
                              integrateMutant( constant, kBias, false, false,
                                               1.0, false, true ) ),
               0.40 );
  }

  TEST( GyroRotationPredictor, ReportsTypedInputAndTimestampErrors )
  {
    const auto finite      = sample( 7, Eigen::Vector3d::Zero() );
    const auto empty_error = requireError(
        integrateGyroRotation( {}, Eigen::Vector3d::Zero() ),
        GyroRotationErrorCode::kInsufficientSamples );
    EXPECT_FALSE( empty_error.sample_index.has_value() );
    EXPECT_FALSE( empty_error.interval_index.has_value() );
    EXPECT_FALSE( empty_error.timestamp_ns.has_value() );
    const std::array one_sample{ finite };
    const auto       one_sample_error = requireError(
        integrateGyroRotation( one_sample, Eigen::Vector3d::Zero() ),
        GyroRotationErrorCode::kInsufficientSamples );
    EXPECT_FALSE( one_sample_error.sample_index.has_value() );
    EXPECT_FALSE( one_sample_error.interval_index.has_value() );
    EXPECT_FALSE( one_sample_error.timestamp_ns.has_value() );

    for ( const double nonfinite : { std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::infinity(),
                                     -std::numeric_limits<double>::infinity() } )
    {
      auto             first_bad  = sample( -9, Eigen::Vector3d{ nonfinite, 0.0, 0.0 } );
      auto             second_bad = sample( 7, Eigen::Vector3d{ 0.0, nonfinite, 0.0 } );
      const std::array samples{ first_bad, second_bad };
      const auto&      error = requireError(
          integrateGyroRotation( samples, Eigen::Vector3d::Zero() ),
          GyroRotationErrorCode::kNonFiniteGyro );
      ASSERT_TRUE( error.sample_index.has_value() );
      EXPECT_EQ( *error.sample_index, 0U );
      ASSERT_TRUE( error.timestamp_ns.has_value() );
      EXPECT_EQ( *error.timestamp_ns, -9 );

      const std::array valid_samples{
          sample( 0, Eigen::Vector3d::Zero() ),
          sample( 1, Eigen::Vector3d::Zero() ) };
      const auto& bias_error = requireError(
          integrateGyroRotation( valid_samples,
                                 Eigen::Vector3d{ 0.0, nonfinite, 0.0 } ),
          GyroRotationErrorCode::kNonFiniteBias );
      EXPECT_FALSE( bias_error.sample_index.has_value() );
      EXPECT_FALSE( bias_error.interval_index.has_value() );
      EXPECT_FALSE( bias_error.timestamp_ns.has_value() );
    }

    for ( Eigen::Index axis = 0; axis < 3; ++axis )
    {
      for ( const double nonfinite : {
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity() } )
      {
        Eigen::Vector3d bad_gyro = Eigen::Vector3d::Zero();
        bad_gyro( axis )         = nonfinite;
        const std::array gyro_samples{
            sample( 0, Eigen::Vector3d::Zero() ), sample( 1, bad_gyro ) };
        const auto gyro_error = requireError(
            integrateGyroRotation( gyro_samples, Eigen::Vector3d::Zero() ),
            GyroRotationErrorCode::kNonFiniteGyro );
        EXPECT_EQ( gyro_error.sample_index, 1U );
        EXPECT_EQ( gyro_error.timestamp_ns, 1 );

        Eigen::Vector3d bad_bias = Eigen::Vector3d::Zero();
        bad_bias( axis )         = nonfinite;
        const std::array valid_samples{
            sample( 0, Eigen::Vector3d::Zero() ),
            sample( 1, Eigen::Vector3d::Zero() ) };
        const auto bias_error = requireError(
            integrateGyroRotation( valid_samples, bad_bias ),
            GyroRotationErrorCode::kNonFiniteBias );
        EXPECT_FALSE( bias_error.sample_index.has_value() );
        EXPECT_FALSE( bias_error.interval_index.has_value() );
        EXPECT_FALSE( bias_error.timestamp_ns.has_value() );
      }
    }

    const std::array duplicate{ sample( 10, Eigen::Vector3d::Zero() ),
                                sample( 10, Eigen::Vector3d::Zero() ) };
    const auto&      duplicate_error = requireError(
        integrateGyroRotation( duplicate, Eigen::Vector3d::Zero() ),
        GyroRotationErrorCode::kDuplicateTimestamp );
    EXPECT_EQ( duplicate_error.sample_index, 1U );
    EXPECT_EQ( duplicate_error.interval_index, 0U );
    EXPECT_EQ( duplicate_error.timestamp_ns, 10 );

    const std::array decreasing{ sample( 10, Eigen::Vector3d::Zero() ),
                                 sample( 9, Eigen::Vector3d::Zero() ) };
    const auto&      decreasing_error = requireError(
        integrateGyroRotation( decreasing, Eigen::Vector3d::Zero() ),
        GyroRotationErrorCode::kOutOfOrderTimestamp );
    EXPECT_EQ( decreasing_error.sample_index, 1U );
    EXPECT_EQ( decreasing_error.interval_index, 0U );
    EXPECT_EQ( decreasing_error.timestamp_ns, 9 );
  }

  TEST( GyroRotationPredictor, HandlesIntegerBoundariesAndOverflow )
  {
    const auto expectDuration = []( std::int64_t first, std::int64_t last,
                                    std::int64_t expected ) {
      const std::array samples{ sample( first, Eigen::Vector3d::Zero() ),
                                sample( last, Eigen::Vector3d::Zero() ) };
      const auto&      prediction = requirePrediction(
          integrateGyroRotation( samples, Eigen::Vector3d::Zero() ) );
      EXPECT_EQ( prediction.duration_ns, expected );
      expectRotation( prediction.delta_R_i_j );
    };
    expectDuration( -10, -3, 7 );
    expectDuration( 0, 1, 1 );
    expectDuration( std::numeric_limits<std::int64_t>::min(),
                    std::numeric_limits<std::int64_t>::min() + 1, 1 );
    expectDuration( std::numeric_limits<std::int64_t>::max() - 1,
                    std::numeric_limits<std::int64_t>::max(), 1 );

    const std::array subtraction_overflow{
        sample( std::numeric_limits<std::int64_t>::min(),
                Eigen::Vector3d::Zero() ),
        sample( std::numeric_limits<std::int64_t>::max(),
                Eigen::Vector3d::Zero() ) };
    const auto subtraction_error = requireError(
        integrateGyroRotation( subtraction_overflow, Eigen::Vector3d::Zero() ),
        GyroRotationErrorCode::kTimestampDeltaOverflow );
    EXPECT_EQ( subtraction_error.sample_index, 1U );
    EXPECT_EQ( subtraction_error.interval_index, 0U );
    EXPECT_EQ( subtraction_error.timestamp_ns,
               std::numeric_limits<std::int64_t>::max() );

    const std::array duration_overflow{
        sample( std::numeric_limits<std::int64_t>::min(),
                Eigen::Vector3d::Zero() ),
        sample( -1, Eigen::Vector3d::Zero() ),
        sample( 0,
                Eigen::Vector3d::Zero() ) };
    const auto duration_error = requireError(
        integrateGyroRotation( duration_overflow, Eigen::Vector3d::Zero() ),
        GyroRotationErrorCode::kDurationOverflow );
    EXPECT_FALSE( duration_error.sample_index.has_value() );
    EXPECT_EQ( duration_error.interval_index, 1U );
    EXPECT_FALSE( duration_error.timestamp_ns.has_value() );
  }

  TEST( GyroRotationPredictor, ReportsNaturalNonFiniteComputation )
  {
    const double     maximum = std::numeric_limits<double>::max();
    const std::array samples{
        sample( 0, Eigen::Vector3d{ maximum, 0.0, 0.0 } ),
        sample( 1, Eigen::Vector3d{ maximum, 0.0, 0.0 } ) };
    const auto& error = requireError(
        integrateGyroRotation( samples, Eigen::Vector3d{ -maximum, 0.0, 0.0 } ),
        GyroRotationErrorCode::kNonFiniteComputation );
    EXPECT_EQ( error.interval_index, 0U );
    EXPECT_FALSE( error.sample_index.has_value() );
    EXPECT_FALSE( error.timestamp_ns.has_value() );
  }

  TEST( GyroRotationPredictor, ReportsNaturalNonFinitePrediction )
  {
    const double          maximum = std::numeric_limits<double>::max();
    const Eigen::Vector3d gyro{ maximum, 0.0, 0.0 };
    const Eigen::Vector3d mean      = 0.5 * gyro + 0.5 * gyro;
    const Eigen::Vector3d corrected = mean - Eigen::Vector3d::Zero();
    const Eigen::Vector3d vector    = corrected * 1e-9;
    for ( Eigen::Index axis = 0; axis < 3; ++axis )
    {
      EXPECT_TRUE( std::isfinite( mean( axis ) ) );
      EXPECT_TRUE( std::isfinite( corrected( axis ) ) );
      EXPECT_TRUE( std::isfinite( vector( axis ) ) );
    }

    const std::array samples{ sample( 0, gyro ), sample( 1, gyro ) };
    const auto       error = requireError(
        integrateGyroRotation( samples, Eigen::Vector3d::Zero() ),
        GyroRotationErrorCode::kNonFinitePrediction );
    EXPECT_FALSE( error.sample_index.has_value() );
    EXPECT_FALSE( error.interval_index.has_value() );
    EXPECT_FALSE( error.timestamp_ns.has_value() );
  }

  TEST( GyroRotationPredictor, ReportsNaturalInvalidRotation )
  {
    const Eigen::Vector3d gyro{ -2.4664720996731243e54,
                                1.9669167514335542e54,
                                -2.5705088372089910e54 };
    const Eigen::Vector3d mean      = 0.5 * gyro + 0.5 * gyro;
    const Eigen::Vector3d corrected = mean - Eigen::Vector3d::Zero();
    const Eigen::Vector3d vector    = corrected * 1e-9;
    for ( Eigen::Index axis = 0; axis < 3; ++axis )
    {
      EXPECT_TRUE( std::isfinite( mean( axis ) ) );
      EXPECT_TRUE( std::isfinite( corrected( axis ) ) );
      EXPECT_TRUE( std::isfinite( vector( axis ) ) );
    }

    std::vector<ImuMeasurement> samples;
    samples.reserve( 471 );
    for ( std::int64_t timestamp_ns = 0; timestamp_ns <= 470;
          ++timestamp_ns )
    {
      samples.push_back( sample( timestamp_ns, gyro ) );
    }
    const auto error = requireError(
        integrateGyroRotation( samples, Eigen::Vector3d::Zero() ),
        GyroRotationErrorCode::kInvalidRotation );
    EXPECT_FALSE( error.sample_index.has_value() );
    EXPECT_FALSE( error.interval_index.has_value() );
    EXPECT_FALSE( error.timestamp_ns.has_value() );
  }

  TEST( GyroRotationPredictor, IgnoresEveryAccelValueElementExactly )
  {
    const Eigen::Vector3d rate{ 0.35, -0.42, 0.27 };
    const auto            baseline_samples = irregularSamples(
        [ & ]( double ) { return rate + kBias; } );
    const auto baseline =
        requirePrediction( integrateGyroRotation( baseline_samples, kBias ) );
    const std::array accel_values{
        std::array{ 12.0, -4.0, 0.5 },
        std::array{ std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0 },
        std::array{ std::numeric_limits<double>::infinity(), 0.0, 0.0 },
        std::array{ -std::numeric_limits<double>::infinity(), 0.0, 0.0 } };
    for ( const auto& accel : accel_values )
    {
      auto changed = baseline_samples;
      for ( auto& measurement : changed )
      {
        measurement.accel_mps2 = accel;
      }
      const auto& prediction =
          requirePrediction( integrateGyroRotation( changed, kBias ) );
      EXPECT_EQ( prediction.duration_ns, baseline.duration_ns );
      for ( Eigen::Index row = 0; row < 3; ++row )
      {
        for ( Eigen::Index column = 0; column < 3; ++column )
        {
          EXPECT_EQ( prediction.delta_R_i_j( row, column ),
                     baseline.delta_R_i_j( row, column ) );
        }
      }
    }
  }

  TEST( GyroRotationPredictor, ReusesSharedEndpointWithoutGapOrOverlap )
  {
    // Frozen endpoint-aligned samples from the qualified upstream sync chain.
    const ImuMeasurement t0 =
        sample( 0, Eigen::Vector3d{ 0.32, -0.18, 0.25 } );
    const ImuMeasurement t1 = sample(
        60'000'000, Eigen::Vector3d{ 0.376, -0.208, 0.306 } );
    const ImuMeasurement t2 = sample(
        113'000'000,
        Eigen::Vector3d{ 0.4254666666666667, -0.23273333333333334,
                         0.3554666666666667 } );
    const std::array first{ t0, t1 };
    const std::array second{ t1, t2 };
    const std::array whole{ t0, t1, t2 };
    EXPECT_EQ( first.size(), 2U );
    EXPECT_EQ( second.size(), 2U );
    EXPECT_EQ( whole.size(), 3U );
    EXPECT_EQ( first[ 1 ].timestamp.nanoseconds(), 60'000'000 );
    EXPECT_EQ( second[ 0 ].timestamp.nanoseconds(), 60'000'000 );
    EXPECT_EQ( whole[ 0 ].timestamp.nanoseconds(), 0 );
    EXPECT_EQ( whole[ 1 ].timestamp.nanoseconds(), 60'000'000 );
    EXPECT_EQ( whole[ 2 ].timestamp.nanoseconds(), 113'000'000 );
    EXPECT_EQ( t0.gyro_radps, ( std::array{ 0.32, -0.18, 0.25 } ) );
    EXPECT_EQ( t1.gyro_radps, ( std::array{ 0.376, -0.208, 0.306 } ) );
    EXPECT_EQ(
        t2.gyro_radps,
        ( std::array{ 0.4254666666666667, -0.23273333333333334,
                      0.3554666666666667 } ) );
    EXPECT_EQ( first[ 1 ].gyro_radps, second[ 0 ].gyro_radps );

    const auto first_prediction =
        requirePrediction( integrateGyroRotation( first, kBias ) );
    const auto second_prediction =
        requirePrediction( integrateGyroRotation( second, kBias ) );
    const auto whole_prediction =
        requirePrediction( integrateGyroRotation( whole, kBias ) );
    EXPECT_EQ( first_prediction.duration_ns, 60'000'000 );
    EXPECT_EQ( second_prediction.duration_ns, 53'000'000 );
    EXPECT_EQ( whole_prediction.duration_ns, 113'000'000 );
    EXPECT_EQ( first_prediction.duration_ns + second_prediction.duration_ns,
               whole_prediction.duration_ns );
    const Eigen::Matrix3d segmented = first_prediction.delta_R_i_j *
                                      second_prediction.delta_R_i_j;
    const Eigen::Matrix3d analytic = exactSharedEndpointRotation();
    EXPECT_LE( rotationError( segmented, whole_prediction.delta_R_i_j ),
               2e-12 );
    EXPECT_LE( rotationError( analytic, segmented ), 2e-12 );
    EXPECT_LE( rotationError( analytic, whole_prediction.delta_R_i_j ),
               2e-12 );
  }

}  // namespace
