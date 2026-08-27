#include "phad/estimator/internal/imu_interval.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace phad::estimator::internal
{
  namespace
  {

    constexpr double kNsToSeconds = 1e-9;

    [[nodiscard]] ImuIntervalError makeError(
        ImuIntervalErrorCode code, std::string detail,
        std::optional<std::size_t> sample_index = std::nullopt )
    {
      return ImuIntervalError{ code, sample_index, std::move( detail ) };
    }

    [[nodiscard]] bool subtractionOverflows( std::int64_t next,
                                             std::int64_t current )
    {
      return current < 0 &&
             next > std::numeric_limits<std::int64_t>::max() + current;
    }

    [[nodiscard]] bool isFinite( const sensor::ImuMeasurement& sample )
    {
      for ( std::size_t axis = 0; axis < 3U; ++axis )
      {
        if ( !std::isfinite( sample.accel_mps2[ axis ] ) ||
             !std::isfinite( sample.gyro_radps[ axis ] ) )
        {
          return false;
        }
      }
      return true;
    }

    [[nodiscard]] bool sameBits( const double lhs, const double rhs )
    {
      static_assert( sizeof( double ) == sizeof( std::uint64_t ) );
      return std::bit_cast<std::uint64_t>( lhs ) ==
             std::bit_cast<std::uint64_t>( rhs );
    }

    [[nodiscard]] bool sameMeasurementBits(
        const sensor::ImuMeasurement& lhs,
        const sensor::ImuMeasurement& rhs )
    {
      for ( std::size_t axis = 0U; axis < 3U; ++axis )
      {
        if ( !sameBits( lhs.accel_mps2[ axis ], rhs.accel_mps2[ axis ] ) ||
             !sameBits( lhs.gyro_radps[ axis ], rhs.gyro_radps[ axis ] ) )
        {
          return false;
        }
      }
      return true;
    }

    [[nodiscard]] Eigen::Vector3d acc(
        const sensor::ImuMeasurement& sample )
    {
      return { sample.accel_mps2[ 0 ], sample.accel_mps2[ 1 ],
               sample.accel_mps2[ 2 ] };
    }

    [[nodiscard]] Eigen::Vector3d gyr(
        const sensor::ImuMeasurement& sample )
    {
      return { sample.gyro_radps[ 0 ], sample.gyro_radps[ 1 ],
               sample.gyro_radps[ 2 ] };
    }

    [[nodiscard]] std::variant<sensor::ImuMeasurement, ImuIntervalError>
    endpointMeasurement( const std::vector<sensor::ImuMeasurement>& samples,
                         common::Timestamp                          endpoint )
    {
      const auto right = std::lower_bound(
          samples.begin(), samples.end(), endpoint,
          []( const sensor::ImuMeasurement& sample,
              common::Timestamp             target ) {
            return sample.timestamp < target;
          } );
      if ( right != samples.end() && right->timestamp == endpoint )
      {
        return *right;
      }
      if ( right == samples.begin() || right == samples.end() )
      {
        return makeError( ImuIntervalErrorCode::kMissingBracket,
                          "raw IMU interval does not bracket an endpoint" );
      }

      const auto         left      = std::prev( right );
      const std::int64_t left_ns   = left->timestamp.nanoseconds();
      const std::int64_t right_ns  = right->timestamp.nanoseconds();
      const std::int64_t target_ns = endpoint.nanoseconds();
      if ( subtractionOverflows( right_ns, left_ns ) )
      {
        return makeError( ImuIntervalErrorCode::kTimestampOverflow,
                          "raw IMU bracket duration overflows int64" );
      }
      const std::int64_t width_ns = right_ns - left_ns;
      const double       alpha =
          static_cast<double>( target_ns - left_ns ) /
          static_cast<double>( width_ns );
      if ( !( alpha > 0.0 && alpha < 1.0 ) || !std::isfinite( alpha ) )
      {
        return makeError( ImuIntervalErrorCode::kNonFiniteInterpolation,
                          "raw IMU endpoint interpolation is invalid" );
      }

      sensor::ImuMeasurement interpolated;
      interpolated.timestamp = endpoint;
      for ( std::size_t axis = 0; axis < 3U; ++axis )
      {
        interpolated.accel_mps2[ axis ] =
            ( 1.0 - alpha ) * left->accel_mps2[ axis ] +
            alpha * right->accel_mps2[ axis ];
        interpolated.gyro_radps[ axis ] =
            ( 1.0 - alpha ) * left->gyro_radps[ axis ] +
            alpha * right->gyro_radps[ axis ];
      }
      if ( !isFinite( interpolated ) )
      {
        return makeError( ImuIntervalErrorCode::kNonFiniteInterpolation,
                          "raw IMU endpoint interpolation is non-finite" );
      }
      return interpolated;
    }

  }  // namespace

  ImuIntervalResult normalizeRawImuInterval(
      const sensor::RawImuInterval&    raw,
      std::optional<common::Timestamp> expected_t_begin,
      common::Timestamp                expected_t_end )
  {
    if ( raw.m_t_begin >= raw.m_t_end )
    {
      return makeError( ImuIntervalErrorCode::kEndpointOrder,
                        "raw IMU interval requires t_begin < t_end" );
    }
    if ( raw.m_t_end != expected_t_end ||
         ( expected_t_begin.has_value() &&
           raw.m_t_begin != *expected_t_begin ) )
    {
      return makeError( ImuIntervalErrorCode::kEndpointMismatch,
                        "raw IMU interval endpoints do not match state timestamps" );
    }
    if ( raw.m_samples.size() < 2U )
    {
      return makeError( ImuIntervalErrorCode::kTooFewSamples,
                        "raw IMU interval requires at least two samples" );
    }

    for ( std::size_t index = 0U; index < raw.m_samples.size(); ++index )
    {
      if ( !isFinite( raw.m_samples[ index ] ) )
      {
        return makeError( ImuIntervalErrorCode::kNonFiniteMeasurement,
                          "raw IMU sample is non-finite", index );
      }
      if ( index > 0U &&
           raw.m_samples[ index ].timestamp <=
               raw.m_samples[ index - 1U ].timestamp )
      {
        return makeError( ImuIntervalErrorCode::kNonIncreasingTimestamp,
                          "raw IMU sample timestamps must be strictly increasing",
                          index );
      }
    }

    auto begin_result = endpointMeasurement( raw.m_samples, raw.m_t_begin );
    if ( std::holds_alternative<ImuIntervalError>( begin_result ) )
    {
      return std::get<ImuIntervalError>( std::move( begin_result ) );
    }
    auto end_result = endpointMeasurement( raw.m_samples, raw.m_t_end );
    if ( std::holds_alternative<ImuIntervalError>( end_result ) )
    {
      return std::get<ImuIntervalError>( std::move( end_result ) );
    }

    NormalizedImuInterval normalized;
    normalized.m_raw = raw;
    normalized.m_nodes.reserve( raw.m_samples.size() + 2U );
    normalized.m_nodes.push_back(
        std::get<sensor::ImuMeasurement>( std::move( begin_result ) ) );
    for ( const sensor::ImuMeasurement& sample : raw.m_samples )
    {
      if ( sample.timestamp > raw.m_t_begin &&
           sample.timestamp < raw.m_t_end )
      {
        normalized.m_nodes.push_back( sample );
      }
    }
    normalized.m_nodes.push_back(
        std::get<sensor::ImuMeasurement>( std::move( end_result ) ) );
    normalized.m_steps.reserve( normalized.m_nodes.size() - 1U );

    for ( std::size_t index = 0U; index + 1U < normalized.m_nodes.size();
          ++index )
    {
      const sensor::ImuMeasurement& current = normalized.m_nodes[ index ];
      const sensor::ImuMeasurement& next    = normalized.m_nodes[ index + 1U ];
      if ( next.timestamp <= current.timestamp ||
           subtractionOverflows( next.timestamp.nanoseconds(),
                                 current.timestamp.nanoseconds() ) )
      {
        return makeError( ImuIntervalErrorCode::kInvalidStep,
                          "normalized IMU step must have positive representable dt",
                          index );
      }
      const std::int64_t dt_ns =
          next.timestamp.nanoseconds() - current.timestamp.nanoseconds();
      if ( dt_ns > std::numeric_limits<std::int64_t>::max() -
                       normalized.m_duration_ns )
      {
        return makeError( ImuIntervalErrorCode::kTimestampOverflow,
                          "normalized IMU duration overflows int64", index );
      }
      normalized.m_duration_ns += dt_ns;
      const double          dt_s     = static_cast<double>( dt_ns ) * kNsToSeconds;
      const Eigen::Vector3d acc_mean = 0.5 * ( acc( current ) + acc( next ) );
      const Eigen::Vector3d gyr_mean = 0.5 * ( gyr( current ) + gyr( next ) );
      if ( !std::isfinite( dt_s ) || !( dt_s > 0.0 ) ||
           !acc_mean.allFinite() || !gyr_mean.allFinite() )
      {
        return makeError( ImuIntervalErrorCode::kInvalidStep,
                          "normalized IMU step is non-finite", index );
      }
      normalized.m_steps.push_back(
          ImuIntervalStep{ acc_mean, gyr_mean, dt_ns, dt_s } );
    }

    if ( subtractionOverflows( raw.m_t_end.nanoseconds(),
                               raw.m_t_begin.nanoseconds() ) ||
         normalized.m_duration_ns !=
             raw.m_t_end.nanoseconds() - raw.m_t_begin.nanoseconds() )
    {
      return makeError( ImuIntervalErrorCode::kClosureMismatch,
                        "normalized IMU interval does not close exactly" );
    }
    return normalized;
  }

  ImuIntervalResult spliceNormalizedImuIntervals(
      const NormalizedImuInterval& before,
      const NormalizedImuInterval& after,
      const common::Timestamp      expected_t_begin,
      const common::Timestamp      shared_endpoint,
      const common::Timestamp      expected_t_end )
  {
    if ( before.m_raw.m_t_begin != expected_t_begin ||
         before.m_raw.m_t_end != shared_endpoint ||
         after.m_raw.m_t_begin != shared_endpoint ||
         after.m_raw.m_t_end != expected_t_end || before.m_nodes.empty() ||
         after.m_nodes.empty() ||
         before.m_nodes.front().timestamp != expected_t_begin ||
         before.m_nodes.back().timestamp != shared_endpoint ||
         after.m_nodes.front().timestamp != shared_endpoint ||
         after.m_nodes.back().timestamp != expected_t_end )
    {
      return makeError(
          ImuIntervalErrorCode::kSharedEndpointMismatch,
          "stored IMU provenance does not match adjacent state timestamps" );
    }

    if ( !sameMeasurementBits( before.m_nodes.back(),
                               after.m_nodes.front() ) )
    {
      return makeError(
          ImuIntervalErrorCode::kSharedEndpointMismatch,
          "shared IMU endpoint has bit-inconsistent accel/gyro values" );
    }

    sensor::RawImuInterval joined;
    joined.m_t_begin = expected_t_begin;
    joined.m_t_end   = expected_t_end;
    joined.m_samples.reserve( before.m_nodes.size() + after.m_nodes.size() -
                              1U );
    joined.m_samples.insert( joined.m_samples.end(), before.m_nodes.begin(),
                             before.m_nodes.end() );
    joined.m_samples.insert( joined.m_samples.end(),
                             std::next( after.m_nodes.begin() ),
                             after.m_nodes.end() );
    return normalizeRawImuInterval( joined, expected_t_begin,
                                    expected_t_end );
  }

}  // namespace phad::estimator::internal
