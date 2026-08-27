#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <span>
#include <string_view>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"

namespace phad::estimator
{

  enum class ImuRotationError : std::uint8_t
  {
    kNone                  = 0,
    kInsufficientSamples   = 1,
    kNonMonotonicTimestamp = 2,
    kNonFiniteGyro         = 3,
    kNonFiniteBias         = 4,
    kNonFiniteResult       = 5
  };

  struct ImuRotationResult
  {
    bool              valid = false;
    ImuRotationError  error = ImuRotationError::kInsufficientSamples;
    common::Timestamp t_i{ 0 };
    common::Timestamp t_j{ 0 };
    double            dt_s      = 0.0;
    double            angle_rad = 0.0;
    Eigen::Matrix3d   R_Bi_Bj   = Eigen::Matrix3d::Identity();
  };

  [[nodiscard]] ImuRotationResult integrateImuRotation(
      std::span<const sensor::ImuMeasurement> samples,
      const Eigen::Vector3d&                  bias_gyr );

  [[nodiscard]] std::string_view imuRotationErrorName(
      ImuRotationError error ) noexcept;

}  // namespace phad::estimator
