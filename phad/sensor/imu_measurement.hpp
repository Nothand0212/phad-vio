#pragma once

#include <array>

#include "phad/common/timestamp.hpp"

/**
 * @file imu_measurement.hpp
 * @brief ImuMeasurement 结构体用于表示 IMU 测量。
 *
 * 该文件定义了 phad::sensor::ImuMeasurement 结构体，
 * 封装了 IMU 测量的时间戳、加速度计测量和陀螺仪测量，提供对 IMU 测量的快速访问和引用。
 */

namespace phad::sensor
{

  /**
   * @brief ImuMeasurement 结构体用于表示 IMU 测量。
   *
   * 该结构体封装了 IMU 测量的时间戳、加速度计测量和陀螺仪测量，提供对 IMU 测量的快速访问和引用。
   */
  struct ImuMeasurement
  {
    common::Timestamp     timestamp;
    std::array<double, 3> accel_mps2{};
    std::array<double, 3> gyro_radps{};
  };

}  // namespace phad::sensor
