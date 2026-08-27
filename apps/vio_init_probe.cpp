#include "apps/vio_init_probe.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace phad::apps
{
  namespace
  {

    constexpr std::string_view kHeader =
        "imu_t_i_ns,imu_t_j_ns,imu_sample_n,imu_dt_s,"
        "gyro_mean_x,gyro_mean_y,gyro_mean_z,"
        "gyro_std_x,gyro_std_y,gyro_std_z,"
        "acc_mean_x,acc_mean_y,acc_mean_z,"
        "acc_std_x,acc_std_y,acc_std_z,"
        "gyro_std_limit,acc_std_limit,"
        "init_q_x,init_q_y,init_q_z,init_q_w,"
        "init_v_x,init_v_y,init_v_z,"
        "init_bg_x,init_bg_y,init_bg_z,"
        "init_ba_x,init_ba_y,init_ba_z,"
        "acc_mean_norm,gravity_model_magnitude";

    void requireFiniteVector( const Eigen::Vector3d& value,
                              const std::string_view name )
    {
      if ( !value.allFinite() )
      {
        throw std::invalid_argument( "VIO init probe has non-finite " +
                                     std::string( name ) );
      }
    }

    void requireNonnegativeVector( const Eigen::Vector3d& value,
                                   const std::string_view name )
    {
      requireFiniteVector( value, name );
      if ( ( value.array() < 0.0 ).any() )
      {
        throw std::invalid_argument( "VIO init probe has negative " +
                                     std::string( name ) );
      }
    }

    void requirePositive( const double value, const std::string_view name )
    {
      if ( !std::isfinite( value ) || !( value > 0.0 ) )
      {
        throw std::invalid_argument( "VIO init probe has invalid " +
                                     std::string( name ) );
      }
    }

    [[nodiscard]] Eigen::Quaterniond validateSnapshot(
        const estimator::ImuInitDiagnostics& init )
    {
      if ( init.imu_sample_count < 2U )
      {
        throw std::invalid_argument(
            "VIO init probe has fewer than two IMU samples" );
      }
      if ( init.imu_t_i_ns >= init.imu_t_j_ns )
      {
        throw std::invalid_argument(
            "VIO init probe IMU interval is not increasing" );
      }
      requirePositive( init.imu_dt_s, "IMU duration" );
      const double expected_dt =
          static_cast<double>( init.imu_t_j_ns - init.imu_t_i_ns ) * 1e-9;
      const double tolerance =
          std::max( 1e-12, std::abs( expected_dt ) * 1e-12 );
      if ( std::abs( init.imu_dt_s - expected_dt ) > tolerance )
      {
        throw std::invalid_argument(
            "VIO init probe IMU duration does not match endpoints" );
      }

      requireFiniteVector( init.gyro_mean, "gyro mean" );
      requireNonnegativeVector( init.gyro_std, "gyro std" );
      requireFiniteVector( init.acc_mean, "accelerometer mean" );
      requireNonnegativeVector( init.acc_std, "accelerometer std" );
      requireFiniteVector( init.velocity_W, "initial velocity" );
      requireFiniteVector( init.bias_gyro, "initial gyro bias" );
      requireFiniteVector( init.bias_acc, "initial accelerometer bias" );
      requirePositive( init.acc_mean_norm, "accelerometer mean norm" );
      requirePositive( init.gravity_model_magnitude,
                       "gravity model magnitude" );
      requirePositive( init.gyro_std_limit, "gyro std limit" );
      requirePositive( init.acc_std_limit, "accelerometer std limit" );

      if ( !init.T_W_B0.matrix().allFinite() )
      {
        throw std::invalid_argument( "VIO init probe has invalid pose" );
      }
      const Eigen::Matrix3d& rotation = init.T_W_B0.linear();
      if ( !( rotation.transpose() * rotation )
                .isApprox( Eigen::Matrix3d::Identity(), 1e-9 ) ||
           std::abs( rotation.determinant() - 1.0 ) > 1e-9 )
      {
        throw std::invalid_argument( "VIO init probe has invalid pose" );
      }
      const Eigen::Quaterniond quaternion( rotation );
      if ( !quaternion.coeffs().allFinite() ||
           std::abs( quaternion.norm() - 1.0 ) > 1e-9 )
      {
        throw std::invalid_argument(
            "VIO init probe has invalid quaternion" );
      }
      return quaternion;
    }

  }  // namespace

  struct VioInitProbe::Impl
  {
    explicit Impl( const std::filesystem::path& path )
        : out( path, std::ios::out | std::ios::trunc )
    {
      if ( !out )
      {
        throw std::runtime_error( "failed to open VIO init probe csv: " +
                                  path.string() );
      }
      out << kHeader << '\n';
      out.flush();
      if ( !out )
      {
        throw std::runtime_error(
            "failed to write VIO init probe csv header" );
      }
    }

    std::ofstream out;
    bool          wrote_event = false;
  };

  VioInitProbe::VioInitProbe( const std::filesystem::path& path )
      : m_impl( std::make_unique<Impl>( path ) )
  {
  }

  VioInitProbe::~VioInitProbe()
  {
    if ( m_impl->out.is_open() )
    {
      m_impl->out.flush();
      m_impl->out.close();
    }
  }

  void VioInitProbe::write( const estimator::VioUpdateResult& update )
  {
    if ( !update.diagnostics.imu_init.has_value() )
    {
      return;
    }
    if ( m_impl->wrote_event )
    {
      throw std::logic_error( "VIO init probe received a duplicate event" );
    }

    const estimator::ImuInitDiagnostics& init =
        *update.diagnostics.imu_init;
    const Eigen::Quaterniond quaternion = validateSnapshot( init );
    std::ostringstream       row;
    row << std::setprecision( std::numeric_limits<double>::max_digits10 )
        << init.imu_t_i_ns << ',' << init.imu_t_j_ns << ','
        << init.imu_sample_count << ',' << init.imu_dt_s << ','
        << init.gyro_mean.x() << ',' << init.gyro_mean.y() << ','
        << init.gyro_mean.z() << ',' << init.gyro_std.x() << ','
        << init.gyro_std.y() << ',' << init.gyro_std.z() << ','
        << init.acc_mean.x() << ',' << init.acc_mean.y() << ','
        << init.acc_mean.z() << ',' << init.acc_std.x() << ','
        << init.acc_std.y() << ',' << init.acc_std.z() << ','
        << init.gyro_std_limit << ',' << init.acc_std_limit << ','
        << quaternion.x() << ',' << quaternion.y() << ',' << quaternion.z()
        << ',' << quaternion.w() << ',' << init.velocity_W.x() << ','
        << init.velocity_W.y() << ',' << init.velocity_W.z() << ','
        << init.bias_gyro.x() << ',' << init.bias_gyro.y() << ','
        << init.bias_gyro.z() << ',' << init.bias_acc.x() << ','
        << init.bias_acc.y() << ',' << init.bias_acc.z() << ','
        << init.acc_mean_norm << ',' << init.gravity_model_magnitude << '\n';

    m_impl->out << row.str();
    m_impl->out.flush();
    if ( !m_impl->out )
    {
      throw std::runtime_error( "failed to write VIO init probe csv row" );
    }
    m_impl->wrote_event = true;
  }

}  // namespace phad::apps
