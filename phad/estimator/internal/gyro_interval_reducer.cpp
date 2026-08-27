#include "phad/estimator/internal/gyro_interval_reducer.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace phad::estimator::internal
{
  namespace
  {

    constexpr double kNsToSeconds = 1e-9;

    [[nodiscard]] GyroIntervalError makeError(
        GyroIntervalErrorCode code, std::string detail,
        std::optional<std::size_t>  sample_index   = std::nullopt,
        std::optional<std::size_t>  interval_index = std::nullopt,
        std::optional<std::int64_t> timestamp_ns   = std::nullopt )
    {
      return GyroIntervalError{ code, sample_index, interval_index,
                                timestamp_ns, std::move( detail ) };
    }

    [[nodiscard]] Eigen::Vector3d gyro(
        const sensor::ImuMeasurement& sample )
    {
      return { sample.gyro_radps[ 0 ], sample.gyro_radps[ 1 ],
               sample.gyro_radps[ 2 ] };
    }

    [[nodiscard]] bool subtractionOverflows( std::int64_t next,
                                             std::int64_t current )
    {
      return current < 0 &&
             next > std::numeric_limits<std::int64_t>::max() + current;
    }

    [[nodiscard]] bool validPositive( double value )
    {
      return std::isfinite( value ) && value > 0.0;
    }

  }  // namespace

  GyroIntervalResult validateGyroInterval(
      std::span<const sensor::ImuMeasurement> samples,
      std::optional<common::Timestamp>        expected_t_prev,
      std::optional<common::Timestamp>        expected_t_curr )
  {
    if ( samples.size() < 2U )
    {
      return makeError( GyroIntervalErrorCode::kTooFewSamples,
                        "gyro interval requires at least two samples" );
    }

    if ( ( expected_t_prev.has_value() &&
           samples.front().timestamp != *expected_t_prev ) ||
         ( expected_t_curr.has_value() &&
           samples.back().timestamp != *expected_t_curr ) )
    {
      return makeError(
          GyroIntervalErrorCode::kEndpointMismatch,
          "gyro interval endpoints do not match pose timestamps" );
    }

    for ( std::size_t index = 0U; index + 1U < samples.size(); ++index )
    {
      if ( samples[ index + 1U ].timestamp <= samples[ index ].timestamp )
      {
        return makeError(
            GyroIntervalErrorCode::kNonIncreasingTimestamp,
            "gyro sample timestamps must be strictly increasing", index + 1U,
            index, samples[ index + 1U ].timestamp.nanoseconds() );
      }
    }

    for ( std::size_t index = 0U; index < samples.size(); ++index )
    {
      if ( !gyro( samples[ index ] ).allFinite() )
      {
        return makeError( GyroIntervalErrorCode::kNonFiniteGyro,
                          "gyro sample is non-finite", index, std::nullopt,
                          samples[ index ].timestamp.nanoseconds() );
      }
    }

    ValidatedGyroInterval interval;
    interval.m_samples.assign( samples.begin(), samples.end() );
    interval.m_steps.reserve( samples.size() - 1U );
    interval.m_t_prev = samples.front().timestamp;
    interval.m_t_curr = samples.back().timestamp;

    for ( std::size_t index = 0U; index + 1U < samples.size(); ++index )
    {
      const std::int64_t current_ns =
          samples[ index ].timestamp.nanoseconds();
      const std::int64_t next_ns =
          samples[ index + 1U ].timestamp.nanoseconds();
      if ( subtractionOverflows( next_ns, current_ns ) )
      {
        return makeError( GyroIntervalErrorCode::kTimestampOverflow,
                          "gyro interval duration overflow", index + 1U,
                          index, next_ns );
      }

      const std::int64_t dt_ns = next_ns - current_ns;
      if ( dt_ns > std::numeric_limits<std::int64_t>::max() -
                       interval.m_duration_ns )
      {
        return makeError( GyroIntervalErrorCode::kTimestampOverflow,
                          "gyro interval duration overflow", std::nullopt,
                          index, std::nullopt );
      }
      interval.m_duration_ns += dt_ns;

      const double          dt_s = static_cast<double>( dt_ns ) * kNsToSeconds;
      const Eigen::Vector3d omega_mean =
          0.5 * gyro( samples[ index ] ) +
          0.5 * gyro( samples[ index + 1U ] );
      if ( !std::isfinite( dt_s ) || !omega_mean.allFinite() )
      {
        return makeError( GyroIntervalErrorCode::kTimestampOverflow,
                          "gyro interval duration overflow", std::nullopt,
                          index, std::nullopt );
      }
      interval.m_steps.push_back(
          GyroIntervalStep{ omega_mean, dt_ns, dt_s } );
    }

    return interval;
  }

  GyroNoiseScalesResult deriveGyroNoiseScales(
      std::int64_t duration_ns, double gyr_nd, double gyr_rw,
      double prior_sigma )
  {
    const double dt_s              = static_cast<double>( duration_ns ) * kNsToSeconds;
    const double rotation_variance = gyr_nd * gyr_nd * dt_s;
    const double rw_variance       = gyr_rw * gyr_rw * dt_s;
    const double rw_sigma          = gyr_rw * std::sqrt( dt_s );
    const double prior_variance    = prior_sigma * prior_sigma;
    if ( duration_ns <= 0 || !validPositive( gyr_nd ) ||
         !validPositive( gyr_rw ) || !validPositive( prior_sigma ) ||
         !validPositive( dt_s ) || !validPositive( rotation_variance ) ||
         !validPositive( rw_variance ) || !validPositive( rw_sigma ) ||
         !validPositive( prior_variance ) )
    {
      return makeError(
          GyroIntervalErrorCode::kInvalidNoiseScale,
          "gyro interval produces invalid noise scale" );
    }

    return GyroNoiseScales{ rotation_variance, rw_variance, rw_sigma,
                            prior_variance };
  }

}  // namespace phad::estimator::internal
