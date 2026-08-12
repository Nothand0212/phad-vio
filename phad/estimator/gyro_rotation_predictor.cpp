#include "phad/estimator/gyro_rotation_predictor.hpp"

#include <gtsam/navigation/PreintegratedRotation.h>

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace phad::estimator
{
  namespace
  {

    constexpr double kNsToSeconds       = 1e-9;
    constexpr double kRotationTolerance = 1e-12;

    static_assert( std::numeric_limits<double>::is_iec559,
                   "gyro prediction requires IEEE-754 finite checks" );

    GyroRotationError makeError(
        GyroRotationErrorCode code, std::string detail,
        std::optional<std::size_t>  sample_index   = std::nullopt,
        std::optional<std::size_t>  interval_index = std::nullopt,
        std::optional<std::int64_t> timestamp_ns   = std::nullopt )
    {
      return GyroRotationError{ code, sample_index, interval_index,
                                timestamp_ns, std::move( detail ) };
    }

    bool isFinite( const Eigen::Vector3d& vector )
    {
      return vector.allFinite();
    }

    Eigen::Vector3d gyro( const sensor::ImuMeasurement& sample )
    {
      return { sample.gyro_radps[ 0 ], sample.gyro_radps[ 1 ],
               sample.gyro_radps[ 2 ] };
    }

    bool subtractOverflows( std::int64_t next, std::int64_t current )
    {
      return current < 0 &&
             next > std::numeric_limits<std::int64_t>::max() + current;
    }

  }  // namespace

  GyroRotationResult integrateGyroRotation(
      std::span<const sensor::ImuMeasurement> samples,
      const Eigen::Vector3d&                  known_bias_radps )
  {
    if ( samples.size() < 2 )
    {
      return makeError( GyroRotationErrorCode::kInsufficientSamples,
                        "at least two gyro samples are required" );
    }
    if ( !isFinite( known_bias_radps ) )
    {
      return makeError( GyroRotationErrorCode::kNonFiniteBias,
                        "known gyro bias must be finite" );
    }
    for ( std::size_t index = 0; index < samples.size(); ++index )
    {
      if ( !isFinite( gyro( samples[ index ] ) ) )
      {
        return makeError(
            GyroRotationErrorCode::kNonFiniteGyro,
            "gyro sample must be finite", index, std::nullopt,
            samples[ index ].timestamp.nanoseconds() );
      }
    }

    auto                         params = std::make_shared<gtsam::PreintegratedRotation::Params>();
    gtsam::PreintegratedRotation pim( params );
    std::int64_t                 duration_ns = 0;

    for ( std::size_t index = 0; index + 1 < samples.size(); ++index )
    {
      const std::int64_t current_ns =
          samples[ index ].timestamp.nanoseconds();
      const std::int64_t next_ns =
          samples[ index + 1 ].timestamp.nanoseconds();
      if ( next_ns == current_ns )
      {
        return makeError( GyroRotationErrorCode::kDuplicateTimestamp,
                          "adjacent timestamps must be distinct", index + 1,
                          index, next_ns );
      }
      if ( next_ns < current_ns )
      {
        return makeError( GyroRotationErrorCode::kOutOfOrderTimestamp,
                          "timestamps must be strictly increasing", index + 1,
                          index, next_ns );
      }
      if ( subtractOverflows( next_ns, current_ns ) )
      {
        return makeError( GyroRotationErrorCode::kTimestampDeltaOverflow,
                          "timestamp delta exceeds int64", index + 1, index,
                          next_ns );
      }

      const std::int64_t delta_ns = next_ns - current_ns;
      if ( delta_ns >
           std::numeric_limits<std::int64_t>::max() - duration_ns )
      {
        return makeError( GyroRotationErrorCode::kDurationOverflow,
                          "accumulated duration exceeds int64", std::nullopt,
                          index, std::nullopt );
      }
      duration_ns += delta_ns;
      const double dt_s = static_cast<double>( delta_ns ) * kNsToSeconds;

      const Eigen::Vector3d mean = 0.5 * gyro( samples[ index ] ) +
                                   0.5 * gyro( samples[ index + 1 ] );
      const Eigen::Vector3d corrected = mean - known_bias_radps;
      const Eigen::Vector3d vector    = corrected * dt_s;
      if ( !isFinite( mean ) || !isFinite( corrected ) ||
           !isFinite( vector ) )
      {
        return makeError( GyroRotationErrorCode::kNonFiniteComputation,
                          "gyro interval computation is non-finite",
                          std::nullopt, index, std::nullopt );
      }

      try
      {
        pim.integrateGyroMeasurement( mean, known_bias_radps, dt_s );
      }
      catch ( const std::exception& error )
      {
        return makeError( GyroRotationErrorCode::kNonFinitePrediction,
                          std::string{ "GTSAM gyro integration failed: " } +
                              error.what(),
                          std::nullopt, index, std::nullopt );
      }
      catch ( ... )
      {
        return makeError( GyroRotationErrorCode::kNonFinitePrediction,
                          "GTSAM gyro integration failed", std::nullopt,
                          index, std::nullopt );
      }
    }

    const Eigen::Matrix3d rotation              = pim.deltaRij().matrix();
    const double          integrated_duration_s = pim.deltaTij();
    const double          expected_duration_s =
        static_cast<double>( duration_ns ) * kNsToSeconds;
    const double duration_tolerance =
        std::numeric_limits<double>::epsilon() *
        static_cast<double>( samples.size() ) *
        std::max( 1.0, expected_duration_s );
    if ( !rotation.allFinite() || !std::isfinite( integrated_duration_s ) )
    {
      return makeError( GyroRotationErrorCode::kNonFinitePrediction,
                        "GTSAM integration output is non-finite" );
    }
    if ( std::abs( integrated_duration_s - expected_duration_s ) >
         duration_tolerance )
    {
      return makeError( GyroRotationErrorCode::kNonFinitePrediction,
                        "GTSAM duration disagrees with exact integer duration" );
    }
    const double orthogonality_error =
        ( rotation.transpose() * rotation - Eigen::Matrix3d::Identity() )
            .norm();
    const double determinant_error =
        std::abs( rotation.determinant() - 1.0 );
    if ( orthogonality_error > kRotationTolerance ||
         determinant_error > kRotationTolerance )
    {
      return makeError( GyroRotationErrorCode::kInvalidRotation,
                        "integrated rotation violates SO(3) invariants" );
    }

    return GyroRotationPrediction{ rotation, duration_ns };
  }

}  // namespace phad::estimator
