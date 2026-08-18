#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"

namespace phad::estimator::internal
{

  enum class GyroIntervalErrorCode
  {
    kTooFewSamples,
    kEndpointMismatch,
    kNonIncreasingTimestamp,
    kNonFiniteGyro,
    kTimestampOverflow,
    kInvalidNoiseScale
  };

  struct GyroIntervalError
  {
    GyroIntervalErrorCode       m_code;
    std::optional<std::size_t>  m_sample_index;
    std::optional<std::size_t>  m_interval_index;
    std::optional<std::int64_t> m_timestamp_ns;
    std::string                 m_detail;
  };

  struct GyroIntervalStep
  {
    Eigen::Vector3d m_omega_mean_radps;
    std::int64_t    m_dt_ns = 0;
    double          m_dt_s  = 0.0;
  };

  struct ValidatedGyroInterval
  {
    std::vector<sensor::ImuMeasurement> m_samples;
    std::vector<GyroIntervalStep>       m_steps;
    common::Timestamp                   m_t_prev;
    common::Timestamp                   m_t_curr;
    std::int64_t                        m_duration_ns = 0;
  };

  using GyroIntervalResult =
      std::variant<ValidatedGyroInterval, GyroIntervalError>;

  [[nodiscard]] GyroIntervalResult validateGyroInterval(
      std::span<const sensor::ImuMeasurement> samples,
      std::optional<common::Timestamp>        expected_t_prev = std::nullopt,
      std::optional<common::Timestamp>        expected_t_curr = std::nullopt );

  struct GyroNoiseScales
  {
    double m_rotation_variance = 0.0;
    double m_rw_variance       = 0.0;
    double m_rw_sigma          = 0.0;
    double m_prior_variance    = 0.0;
  };

  using GyroNoiseScalesResult =
      std::variant<GyroNoiseScales, GyroIntervalError>;

  [[nodiscard]] GyroNoiseScalesResult deriveGyroNoiseScales(
      std::int64_t duration_ns, double gyr_nd, double gyr_rw,
      double prior_sigma );

}  // namespace phad::estimator::internal
