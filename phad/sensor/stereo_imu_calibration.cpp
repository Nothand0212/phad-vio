#include "phad/sensor/stereo_imu_calibration.hpp"

#include <utility>

namespace phad::sensor
{

  CalibrationResult<StereoImuCalibration> StereoImuCalibration::create(
      CameraParameters left_camera, CameraParameters right_camera,
      ImuParameters imu, RigidTransform T_B_left_camera,
      RigidTransform T_B_right_camera )
  {
    const double baseline_m =
        ( T_B_left_camera.translation() -
          T_B_right_camera.translation() )
            .norm();
    if ( baseline_m <= 0.0 )
    {
      return CalibrationError{
          CalibrationErrorCode::kZeroStereoBaseline,
          "stereo_imu_calibration.camera_centers",
          "left and right camera centers must not coincide" };
    }
    return StereoImuCalibration(
        std::move( left_camera ), std::move( right_camera ),
        std::move( imu ), std::move( T_B_left_camera ),
        std::move( T_B_right_camera ) );
  }

  StereoImuCalibration::StereoImuCalibration(
      CameraParameters left_camera, CameraParameters right_camera,
      ImuParameters imu, RigidTransform T_B_left_camera,
      RigidTransform T_B_right_camera )
      : m_left_camera( std::move( left_camera ) ),
        m_right_camera( std::move( right_camera ) ),
        m_imu( std::move( imu ) ),
        m_T_B_left_camera( std::move( T_B_left_camera ) ),
        m_T_B_right_camera( std::move( T_B_right_camera ) )
  {
  }

  const CameraParameters& StereoImuCalibration::leftCamera() const noexcept
  {
    return m_left_camera;
  }

  const CameraParameters& StereoImuCalibration::rightCamera() const noexcept
  {
    return m_right_camera;
  }

  const ImuParameters& StereoImuCalibration::imu() const noexcept
  {
    return m_imu;
  }

  const RigidTransform&
  StereoImuCalibration::T_B_left_camera() const noexcept
  {
    return m_T_B_left_camera;
  }

  const RigidTransform&
  StereoImuCalibration::T_B_right_camera() const noexcept
  {
    return m_T_B_right_camera;
  }

}  // namespace phad::sensor
