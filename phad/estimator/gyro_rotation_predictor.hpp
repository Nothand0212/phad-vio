#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <variant>

#include "phad/sensor/imu_measurement.hpp"

namespace phad::estimator
{

  struct GyroRotationPrediction
  {
    Eigen::Matrix3d delta_R_i_j;
    std::int64_t    duration_ns;
  };

  enum class GyroRotationErrorCode
  {
    kInsufficientSamples,
    kNonFiniteGyro,
    kNonFiniteBias,
    kDuplicateTimestamp,
    kOutOfOrderTimestamp,
    kTimestampDeltaOverflow,
    kDurationOverflow,
    kNonFiniteComputation,
    kNonFinitePrediction,
    kInvalidRotation,
  };

  struct GyroRotationError
  {
    GyroRotationErrorCode       code;
    std::optional<std::size_t>  sample_index;
    std::optional<std::size_t>  interval_index;
    std::optional<std::int64_t> timestamp_ns;
    std::string                 detail;
  };

  using GyroRotationResult =
      std::variant<GyroRotationPrediction, GyroRotationError>;

  [[nodiscard]] GyroRotationResult integrateGyroRotation(
      std::span<const sensor::ImuMeasurement> samples,
      const Eigen::Vector3d&                  known_bias_radps );

}  // namespace phad::estimator
