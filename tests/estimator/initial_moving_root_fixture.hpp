#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/estimator/types.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_imu_packet.hpp"

namespace phad::test_support::initial_moving
{

  inline constexpr std::int64_t kRootTimestampNs = 50'000'000;
  inline constexpr std::int64_t kFramePeriodNs   = 50'000'000;
  inline constexpr std::int64_t kImuPeriodNs     = 100'000;
  inline const Eigen::Vector3d kGravityW( 0.0, 0.0, -9.81 );
  inline const Eigen::Vector3d kGyroBias( 0.01, -0.02, 0.03 );

  [[nodiscard]] inline double relativeSeconds(
      const std::int64_t timestamp_ns )
  {
    return static_cast<double>( timestamp_ns - kRootTimestampNs ) * 1e-9;
  }

  [[nodiscard]] inline Eigen::Vector3d eulerZyx( const double t )
  {
    return Eigen::Vector3d(
        0.8 * t + 0.6 * t * t,
        -0.55 * t + 0.4 * t * t,
        0.7 * t - 0.3 * t * t );
  }

  [[nodiscard]] inline Eigen::Vector3d eulerZyxRate( const double t )
  {
    return Eigen::Vector3d(
        0.8 + 1.2 * t,
        -0.55 + 0.8 * t,
        0.7 - 0.6 * t );
  }

  [[nodiscard]] inline Eigen::Matrix3d rotationWb( const double t )
  {
    const Eigen::Vector3d euler = eulerZyx( t );
    return ( Eigen::AngleAxisd( euler.x(), Eigen::Vector3d::UnitZ() ) *
             Eigen::AngleAxisd( euler.y(), Eigen::Vector3d::UnitY() ) *
             Eigen::AngleAxisd( euler.z(), Eigen::Vector3d::UnitX() ) )
        .toRotationMatrix();
  }

  [[nodiscard]] inline Eigen::Vector3d angularVelocityB( const double t )
  {
    const Eigen::Vector3d euler = eulerZyx( t );
    const Eigen::Vector3d rate  = eulerZyxRate( t );
    const double alpha_dot = rate.x();
    const double beta_dot  = rate.y();
    const double gamma_dot = rate.z();
    const double beta      = euler.y();
    const double gamma     = euler.z();
    return Eigen::Vector3d(
        gamma_dot - alpha_dot * std::sin( beta ),
        beta_dot * std::cos( gamma ) +
            alpha_dot * std::sin( gamma ) * std::cos( beta ),
        -beta_dot * std::sin( gamma ) +
            alpha_dot * std::cos( gamma ) * std::cos( beta ) );
  }

  [[nodiscard]] inline Eigen::Vector3d positionW( const double t )
  {
    return Eigen::Vector3d(
        0.18 * t + 0.35 * t * t + 0.16 * t * t * t,
        -0.12 * t + 0.28 * t * t - 0.08 * t * t * t,
        0.06 * t + 0.15 * t * t + 0.05 * t * t * t );
  }

  [[nodiscard]] inline Eigen::Vector3d velocityW( const double t )
  {
    return Eigen::Vector3d(
        0.18 + 0.70 * t + 0.48 * t * t,
        -0.12 + 0.56 * t - 0.24 * t * t,
        0.06 + 0.30 * t + 0.15 * t * t );
  }

  [[nodiscard]] inline Eigen::Vector3d accelerationW( const double t )
  {
    return Eigen::Vector3d(
        0.70 + 0.96 * t,
        0.56 - 0.48 * t,
        0.30 + 0.30 * t );
  }

  [[nodiscard]] inline Eigen::Isometry3d poseWb(
      const std::int64_t timestamp_ns )
  {
    const double t = relativeSeconds( timestamp_ns );
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    T_W_B.linear()          = rotationWb( t );
    T_W_B.translation()     = positionW( t );
    return T_W_B;
  }

  [[nodiscard]] inline Eigen::Vector3d velocityW(
      const std::int64_t timestamp_ns )
  {
    return velocityW( relativeSeconds( timestamp_ns ) );
  }

  [[nodiscard]] inline sensor::ImuMeasurement imu(
      const std::int64_t timestamp_ns )
  {
    const double t = relativeSeconds( timestamp_ns );
    const Eigen::Matrix3d R_W_B = rotationWb( t );
    const Eigen::Vector3d specific_force =
        R_W_B.transpose() * ( accelerationW( t ) - kGravityW );
    const Eigen::Vector3d gyro = angularVelocityB( t ) + kGyroBias;
    return sensor::ImuMeasurement{
      .timestamp = common::Timestamp{ timestamp_ns },
      .accel_mps2 = std::array<double, 3>{
        specific_force.x(), specific_force.y(), specific_force.z() },
      .gyro_radps =
          std::array<double, 3>{ gyro.x(), gyro.y(), gyro.z() }
    };
  }

  [[nodiscard]] inline sensor::RawImuInterval rawInterval(
      const std::int64_t begin_ns, const std::int64_t end_ns )
  {
    sensor::RawImuInterval interval{
      .m_t_begin = common::Timestamp{ begin_ns },
      .m_t_end   = common::Timestamp{ end_ns },
      .m_samples = {}
    };
    for ( std::int64_t timestamp_ns = begin_ns;
          timestamp_ns <= end_ns; timestamp_ns += kImuPeriodNs )
    {
      interval.m_samples.push_back( imu( timestamp_ns ) );
    }
    return interval;
  }

  [[nodiscard]] inline std::vector<Eigen::Vector3d> landmarksW()
  {
    std::vector<Eigen::Vector3d> landmarks;
    landmarks.reserve( 20U );
    for ( int row = 0; row < 4; ++row )
    {
      for ( int column = 0; column < 5; ++column )
      {
        landmarks.emplace_back(
            -1.0 + 0.5 * static_cast<double>( column ),
            -0.6 + 0.4 * static_cast<double>( row ),
            5.0 + 0.35 * static_cast<double>( ( row + column ) % 4 ) );
      }
    }
    return landmarks;
  }

  [[nodiscard]] inline std::vector<estimator::StereoObservation>
  observations( const std::int64_t timestamp_ns )
  {
    constexpr double kFx       = 400.0;
    constexpr double kFy       = 400.0;
    constexpr double kCx       = 320.0;
    constexpr double kCy       = 240.0;
    constexpr double kBaseline = 0.12;
    const Eigen::Isometry3d T_B_W = poseWb( timestamp_ns ).inverse();
    const std::vector<Eigen::Vector3d> landmarks = landmarksW();
    std::vector<estimator::StereoObservation> result;
    result.reserve( landmarks.size() );
    for ( std::size_t index = 0U; index < landmarks.size(); ++index )
    {
      const Eigen::Vector3d point_B = T_B_W * landmarks[ index ];
      const double u = kFx * point_B.x() / point_B.z() + kCx;
      const double v = kFy * point_B.y() / point_B.z() + kCy;
      const double disparity = kFx * kBaseline / point_B.z();
      result.push_back( estimator::StereoObservation{
        .id           = static_cast<common::LandmarkId>( index + 1U ),
        .left_pixel   = Eigen::Vector2d( u, v ),
        .disparity_px = disparity
      } );
    }
    return result;
  }

  namespace non_z_gravity
  {

    inline constexpr double kTiltRad = 0.35;
    // R_N_V is a pure pitch rotation, so gravity alignment introduces no yaw
    // gauge choice.
    inline const Eigen::Matrix3d kRNavigationVisual =
        Eigen::AngleAxisd( kTiltRad, Eigen::Vector3d::UnitY() )
            .toRotationMatrix();
    inline const Eigen::Vector3d kGravityV =
        kRNavigationVisual.transpose() * kGravityW;

    [[nodiscard]] inline Eigen::Isometry3d poseNb(
        const std::int64_t timestamp_ns )
    {
      const Eigen::Isometry3d T_V_B = poseWb( timestamp_ns );
      Eigen::Isometry3d       T_N_B = Eigen::Isometry3d::Identity();
      T_N_B.linear()                = kRNavigationVisual * T_V_B.rotation();
      T_N_B.translation() =
          kRNavigationVisual * T_V_B.translation();
      return T_N_B;
    }

    [[nodiscard]] inline Eigen::Vector3d velocityN(
        const std::int64_t timestamp_ns )
    {
      return kRNavigationVisual * velocityW( timestamp_ns );
    }

    [[nodiscard]] inline sensor::ImuMeasurement imu(
        const std::int64_t timestamp_ns )
    {
      const double          t     = relativeSeconds( timestamp_ns );
      const Eigen::Matrix3d R_V_B = rotationWb( t );
      const Eigen::Vector3d specific_force =
          R_V_B.transpose() * ( accelerationW( t ) - kGravityV );
      const Eigen::Vector3d gyro = angularVelocityB( t ) + kGyroBias;
      return sensor::ImuMeasurement{
          .timestamp  = common::Timestamp{ timestamp_ns },
          .accel_mps2 = std::array<double, 3>{
              specific_force.x(), specific_force.y(), specific_force.z() },
          .gyro_radps = std::array<double, 3>{ gyro.x(), gyro.y(), gyro.z() } };
    }

    [[nodiscard]] inline sensor::RawImuInterval rawInterval(
        const std::int64_t begin_ns, const std::int64_t end_ns )
    {
      sensor::RawImuInterval interval{
          .m_t_begin = common::Timestamp{ begin_ns },
          .m_t_end   = common::Timestamp{ end_ns },
          .m_samples = {} };
      for ( std::int64_t timestamp_ns = begin_ns;
            timestamp_ns <= end_ns; timestamp_ns += kImuPeriodNs )
      {
        interval.m_samples.push_back( imu( timestamp_ns ) );
      }
      return interval;
    }

  }  // namespace non_z_gravity

}  // namespace phad::test_support::initial_moving
