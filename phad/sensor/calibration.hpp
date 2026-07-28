#pragma once

#include <array>
#include <cstdint>


/**
 * @file calibration.hpp
 * @brief CameraCalibration 和 ImuCalibration 结构体用于表示相机和 IMU 的内外参数。
 *
 * 该文件定义了 phad::sensor::CameraCalibration 和 phad::sensor::ImuCalibration 结构体，
 * 封装了相机和 IMU 的内外参数，提供对相机和 IMU 内外参数的快速访问和引用。
 */

namespace phad::sensor
{

  /**
   * @brief CameraModel 枚举类型用于表示相机模型。
   *
   * 每个枚举值对应一个特定的相机模型，用于标识相机的几何形状。目前只支持针孔模型。
   */
  enum class CameraModel : std::uint8_t
  {
    kPinhole = 0
  };

  /**
   * @brief DistortionModel 枚举类型用于表示畸变模型。
   *
   * 每个枚举值对应一个特定的畸变模型，用于标识畸变的类型。目前只支持径向切向畸变模型。
   */
  enum class DistortionModel : std::uint8_t
  {
    kRadialTangential = 0
  };

  /**
   * @brief RigidTransform 结构体用于表示刚体变换。
   *
   * 该结构体封装了刚体变换的矩阵，提供对刚体变换的快速访问和引用。该矩阵为行主矩阵，用于表示 4x4 的齐次变换矩阵。
   */
  struct RigidTransform
  {
    // Row-major homogeneous transform with semantics encoded by the field name.
    std::array<double, 16> matrix{};
  };

  /**
   * @brief ImageResolution 结构体用于表示图像分辨率。
   *
   * 该结构体封装了图像的宽度、高度，提供对图像分辨率的快速访问和引用。
   */
  struct ImageResolution
  {
    int width  = 0;
    int height = 0;
  };

  /**
   * @brief CameraIntrinsics 结构体用于表示相机内参。
   *
   * 该结构体封装了相机的内参，提供对相机内参的快速访问和引用。
   */
  struct CameraIntrinsics
  {
    double fx_pixels = 0.0;
    double fy_pixels = 0.0;
    double cx_pixels = 0.0;
    double cy_pixels = 0.0;
  };

  /**
   * @brief CameraCalibration 结构体用于表示相机外参。
   *
   * 该结构体封装了相机的外参，提供对相机外参的快速访问和引用。
   */
  struct CameraCalibration
  {
    CameraModel           camera_model = CameraModel::kPinhole;
    CameraIntrinsics      intrinsics;
    DistortionModel       distortion_model = DistortionModel::kRadialTangential;
    std::array<double, 4> distortion_coefficients{};
    ImageResolution       resolution;
    RigidTransform        T_B_camera;
    double                rate_hz = 0.0;
  };

  /**
   * @brief ImuCalibration 结构体用于表示 IMU 外参。
   *
   * 该结构体封装了 IMU 的外参，提供对 IMU 外参的快速访问和引用。
   */
  struct ImuCalibration
  {
    RigidTransform T_B_imu;        // IMU在body系下的齐次变换
    double         rate_hz = 0.0;  // 采样频率 (Hz)
    double         acc_nd  = 0.0;  // accelerometer_noise_density_mps2_per_sqrt_hz 加速度计噪声密度 (m/s^2/√Hz)
    double         gyr_nd  = 0.0;  // gyroscope_noise_density_radps_per_sqrt_hz 陀螺仪噪声密度 (rad/s/√Hz)
    double         acc_rw  = 0.0;  // accelerometer_random_walk_mps3_per_sqrt_hz 加速度计随机游走 (m/s^3/√Hz)
    double         gyr_rw  = 0.0;  // gyroscope_random_walk_radps2_per_sqrt_hz 陀螺仪随机游走 (rad/s^2/√Hz)
  };

}  // namespace phad::sensor
