#pragma once

#include "phad/sensor/calibration_error.hpp"
#include "phad/sensor/camera_parameters.hpp"
#include "phad/sensor/imu_parameters.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace phad::sensor
{

  class StereoImuCalibration
  {
  public:
    [[nodiscard]] static CalibrationResult<StereoImuCalibration> create(
        CameraParameters left_camera, CameraParameters right_camera,
        ImuParameters imu, RigidTransform T_B_left_camera,
        RigidTransform T_B_right_camera );

    [[nodiscard]] const CameraParameters& leftCamera() const noexcept;
    [[nodiscard]] const CameraParameters& rightCamera() const noexcept;
    [[nodiscard]] const ImuParameters&    imu() const noexcept;
    [[nodiscard]] const RigidTransform&   T_B_left_camera() const noexcept;
    [[nodiscard]] const RigidTransform&   T_B_right_camera() const noexcept;

  private:
    StereoImuCalibration( CameraParameters left_camera,
                          CameraParameters right_camera, ImuParameters imu,
                          RigidTransform T_B_left_camera,
                          RigidTransform T_B_right_camera );

    CameraParameters m_left_camera;
    CameraParameters m_right_camera;
    ImuParameters    m_imu;
    RigidTransform   m_T_B_left_camera;
    RigidTransform   m_T_B_right_camera;
  };

}  // namespace phad::sensor
