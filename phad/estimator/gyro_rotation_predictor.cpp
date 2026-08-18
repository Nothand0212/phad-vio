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
#include <variant>

#include "phad/estimator/internal/gyro_interval_reducer.hpp"

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

    GyroRotationError mapIntervalError(
        const internal::GyroIntervalError&      error,
        std::span<const sensor::ImuMeasurement> samples )
    {
      using internal::GyroIntervalErrorCode;
      switch ( error.m_code )
      {
        case GyroIntervalErrorCode::kTooFewSamples:
          return makeError( GyroRotationErrorCode::kInsufficientSamples,
                            "at least two gyro samples are required" );
        case GyroIntervalErrorCode::kNonIncreasingTimestamp:
        {
          const bool duplicate =
              error.m_sample_index.has_value() &&
              *error.m_sample_index > 0U &&
              samples[ *error.m_sample_index ].timestamp ==
                  samples[ *error.m_sample_index - 1U ].timestamp;
          return makeError(
              duplicate ? GyroRotationErrorCode::kDuplicateTimestamp
                        : GyroRotationErrorCode::kOutOfOrderTimestamp,
              duplicate ? "adjacent timestamps must be distinct"
                        : "timestamps must be strictly increasing",
              error.m_sample_index, error.m_interval_index,
              error.m_timestamp_ns );
        }
        case GyroIntervalErrorCode::kNonFiniteGyro:
          return makeError( GyroRotationErrorCode::kNonFiniteGyro,
                            "gyro sample must be finite",
                            error.m_sample_index, error.m_interval_index,
                            error.m_timestamp_ns );
        case GyroIntervalErrorCode::kTimestampOverflow:
        {
          const bool delta_overflow = error.m_sample_index.has_value();
          return makeError(
              delta_overflow
                  ? GyroRotationErrorCode::kTimestampDeltaOverflow
                  : GyroRotationErrorCode::kDurationOverflow,
              delta_overflow ? "timestamp delta exceeds int64"
                             : "accumulated duration exceeds int64",
              error.m_sample_index, error.m_interval_index,
              error.m_timestamp_ns );
        }
        case GyroIntervalErrorCode::kEndpointMismatch:
        case GyroIntervalErrorCode::kInvalidNoiseScale:
          return makeError(
              GyroRotationErrorCode::kNonFinitePrediction,
              "internal gyro interval validation contract was violated" );
      }
      return makeError( GyroRotationErrorCode::kNonFinitePrediction,
                        "unknown gyro interval validation error" );
    }

  }  // namespace

  GyroRotationResult integrateGyroRotation(
      std::span<const sensor::ImuMeasurement> samples,
      const Eigen::Vector3d&                  known_bias_radps )
  {
    if ( samples.size() < 2U )
    {
      return makeError( GyroRotationErrorCode::kInsufficientSamples,
                        "at least two gyro samples are required" );
    }
    if ( !isFinite( known_bias_radps ) )
    {
      return makeError( GyroRotationErrorCode::kNonFiniteBias,
                        "known gyro bias must be finite" );
    }

    const internal::GyroIntervalResult reduced =
        internal::validateGyroInterval( samples );
    if ( const auto* error =
             std::get_if<internal::GyroIntervalError>( &reduced ) )
    {
      return mapIntervalError( *error, samples );
    }
    const auto& interval =
        std::get<internal::ValidatedGyroInterval>( reduced );

    auto                         params = std::make_shared<gtsam::PreintegratedRotation::Params>();
    gtsam::PreintegratedRotation pim( params );

    for ( std::size_t index = 0U; index < interval.m_steps.size(); ++index )
    {
      const internal::GyroIntervalStep& step = interval.m_steps[ index ];
      const Eigen::Vector3d             corrected =
          step.m_omega_mean_radps - known_bias_radps;
      const Eigen::Vector3d vector = corrected * step.m_dt_s;
      if ( !isFinite( corrected ) || !isFinite( vector ) )
      {
        return makeError( GyroRotationErrorCode::kNonFiniteComputation,
                          "gyro interval computation is non-finite",
                          std::nullopt, index, std::nullopt );
      }

      try
      {
        pim.integrateGyroMeasurement( step.m_omega_mean_radps,
                                      known_bias_radps, step.m_dt_s );
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
        static_cast<double>( interval.m_duration_ns ) * kNsToSeconds;
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

    return GyroRotationPrediction{ rotation, interval.m_duration_ns };
  }

}  // namespace phad::estimator
