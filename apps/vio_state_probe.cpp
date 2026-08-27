#include "apps/vio_state_probe.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace phad::apps
{
  namespace
  {

    constexpr std::string_view kHeader =
        "timestamp_ns,state_key,prior_key,window_size,is_keyframe,"
        "fusion_mode,gyro_factor_n,gyro_alignment_residual_rms_rad,"
        "imu_sample_n,imu_t_i_ns,imu_t_j_ns,imu_dt_s,prediction_valid,"
        "pred_p_x,pred_p_y,pred_p_z,pred_q_x,pred_q_y,pred_q_z,pred_q_w,"
        "pred_bg_x,pred_bg_y,pred_bg_z,"
        "graph_init_p_x,graph_init_p_y,graph_init_p_z,graph_init_q_x,"
        "graph_init_q_y,graph_init_q_z,graph_init_q_w,"
        "post_p_x,post_p_y,post_p_z,post_q_x,post_q_y,post_q_z,post_q_w,"
        "post_bg_x,post_bg_y,post_bg_z,"
        "gyro_obj_valid,gyro_obj_factor_n,gyro_obj_interior_n,"
        "gyro_obj_stereo_n,gyro_obj_total_init,gyro_obj_total_post,"
        "gyro_obj_gyro_init,gyro_obj_gyro_post,gyro_obj_boundary_init,"
        "gyro_obj_boundary_post,gyro_obj_boundary_r_init_rad,"
        "gyro_obj_boundary_r_post_rad,gyro_obj_boundary_w_init,"
        "gyro_obj_boundary_w_post,gyro_obj_interior_init,"
        "gyro_obj_interior_post,gyro_obj_new_init,gyro_obj_new_post,"
        "gyro_obj_new_r_init_rad,gyro_obj_new_r_post_rad,"
        "gyro_obj_new_w_init,gyro_obj_new_w_post,gyro_obj_stereo_init,"
        "gyro_obj_stereo_post,gyro_obj_pose_prior_r_post_rad,"
        "gyro_obj_pose_prior_t_post_m,gyro_obj_pose_prior_post,"
        "gyro_obj_bias_prior_r_post_rps,gyro_obj_bias_prior_post";

    [[nodiscard]] bool isValidPose( const Eigen::Isometry3d& pose )
    {
      if ( !pose.matrix().allFinite() )
      {
        return false;
      }
      const Eigen::Matrix3d& rotation = pose.linear();
      return ( rotation.transpose() * rotation )
                 .isApprox( Eigen::Matrix3d::Identity(), 1e-9 ) &&
             std::abs( rotation.determinant() - 1.0 ) <= 1e-9;
    }

    [[nodiscard]] std::string_view fusionModeName(
        const estimator::FusionMode mode )
    {
      switch ( mode )
      {
        case estimator::FusionMode::kVisionOnly:
          return "vision_only";
        case estimator::FusionMode::kGyroVisual:
          return "gyro_visual";
      }
      throw std::invalid_argument( "vio state probe has invalid fusion mode" );
    }

    void requireFiniteVector( const Eigen::Vector3d& value,
                              const std::string_view name )
    {
      if ( !value.allFinite() )
      {
        throw std::invalid_argument( "vio state probe has non-finite " +
                                     std::string( name ) );
      }
    }

    void validateGyroGraphCost(
        const estimator::GyroGraphCostDiagnostics& cost,
        const std::uint32_t                        expected_gyro_factor_count )
    {
      const auto requireNonNegative = []( const double value ) {
        return std::isfinite( value ) && value >= 0.0;
      };
      const double scalars[] = {
          cost.total_initial_cost,
          cost.total_posterior_cost,
          cost.gyro_initial_cost,
          cost.gyro_posterior_cost,
          cost.boundary_initial_cost,
          cost.boundary_posterior_cost,
          cost.boundary_initial_residual_norm_rad,
          cost.boundary_posterior_residual_norm_rad,
          cost.boundary_initial_whitened_norm,
          cost.boundary_posterior_whitened_norm,
          cost.interior_initial_cost,
          cost.interior_posterior_cost,
          cost.newest_initial_cost,
          cost.newest_posterior_cost,
          cost.newest_initial_residual_norm_rad,
          cost.newest_posterior_residual_norm_rad,
          cost.newest_initial_whitened_norm,
          cost.newest_posterior_whitened_norm,
          cost.stereo_initial_cost,
          cost.stereo_posterior_cost,
          cost.pose_prior_posterior_rotation_norm,
          cost.pose_prior_posterior_translation_norm,
          cost.pose_prior_posterior_cost,
          cost.bias_prior_posterior_norm,
          cost.bias_prior_posterior_cost };
      const bool scalars_valid =
          std::all_of( std::begin( scalars ), std::end( scalars ),
                       requireNonNegative );
      if ( cost.gyro_factor_count != expected_gyro_factor_count ||
           cost.gyro_factor_count == 0U ||
           cost.interior_factor_count + 1U != cost.gyro_factor_count ||
           cost.stereo_factor_count == 0U || !scalars_valid )
      {
        throw std::invalid_argument(
            "vio state probe has invalid gyro graph cost diagnostics" );
      }
    }

    void validateInterval( const estimator::GyroStateDiagnostics& state )
    {
      if ( !std::isfinite( state.imu_dt_s ) || state.imu_dt_s < 0.0 )
      {
        throw std::invalid_argument(
            "vio state probe has invalid IMU duration" );
      }
      if ( state.prediction_valid && state.imu_sample_count < 2U )
      {
        throw std::invalid_argument(
            "vio state probe valid prediction has fewer than two IMU samples" );
      }
      if ( state.imu_sample_count < 2U )
      {
        return;
      }
      if ( state.imu_t_i_ns >= state.imu_t_j_ns )
      {
        throw std::invalid_argument(
            "vio state probe IMU interval is not increasing" );
      }
      const double expected_dt =
          static_cast<double>( state.imu_t_j_ns - state.imu_t_i_ns ) * 1e-9;
      const double tolerance =
          std::max( 1e-12, std::abs( expected_dt ) * 1e-12 );
      if ( std::abs( state.imu_dt_s - expected_dt ) > tolerance )
      {
        throw std::invalid_argument(
            "vio state probe IMU duration does not match endpoints" );
      }
    }

    [[nodiscard]] Eigen::Quaterniond validatedQuaternion(
        const Eigen::Isometry3d& pose, const std::string_view name )
    {
      if ( !isValidPose( pose ) )
      {
        throw std::invalid_argument( "vio state probe has invalid " +
                                     std::string( name ) + " pose" );
      }
      const Eigen::Quaterniond quaternion( pose.linear() );
      if ( !quaternion.coeffs().allFinite() ||
           std::abs( quaternion.norm() - 1.0 ) > 1e-9 )
      {
        throw std::invalid_argument( "vio state probe has invalid " +
                                     std::string( name ) + " quaternion" );
      }
      return quaternion;
    }

  }  // namespace

  struct VioStateProbe::Impl
  {
    explicit Impl( const std::filesystem::path& path )
        : out( path, std::ios::out | std::ios::trunc )
    {
      if ( !out )
      {
        throw std::runtime_error( "failed to open VIO state probe csv: " +
                                  path.string() );
      }
      out << kHeader << '\n';
      out.flush();
      if ( !out )
      {
        throw std::runtime_error(
            "failed to write VIO state probe csv header" );
      }
    }

    std::ofstream                out;
    std::optional<std::int64_t>  last_timestamp_ns;
    std::optional<std::uint64_t> last_state_key;
  };

  VioStateProbe::VioStateProbe( const std::filesystem::path& path )
      : m_impl( std::make_unique<Impl>( path ) )
  {
  }

  VioStateProbe::~VioStateProbe()
  {
    if ( m_impl->out.is_open() )
    {
      m_impl->out.flush();
      m_impl->out.close();
    }
  }

  void VioStateProbe::write(
      const bool is_keyframe, const estimator::VioUpdateResult& update )
  {
    if ( update.status != estimator::UpdateStatus::kOk )
    {
      return;
    }
    if ( !update.estimate.has_value() )
    {
      throw std::invalid_argument(
          "accepted VIO state probe update has no estimate" );
    }
    if ( !update.diagnostics.gyro_state.has_value() )
    {
      throw std::invalid_argument(
          "accepted VIO state probe update has no gyro snapshot" );
    }

    const std::int64_t timestamp_ns =
        update.estimate->timestamp.nanoseconds();
    const estimator::GyroStateDiagnostics& state =
        *update.diagnostics.gyro_state;
    if ( m_impl->last_timestamp_ns.has_value() &&
         timestamp_ns <= *m_impl->last_timestamp_ns )
    {
      throw std::invalid_argument(
          "VIO state probe timestamp is not strictly increasing" );
    }
    if ( m_impl->last_state_key.has_value() &&
         state.state_key <= *m_impl->last_state_key )
    {
      throw std::invalid_argument(
          "VIO state probe state key is not strictly increasing" );
    }

    validateInterval( state );
    const estimator::FusionMode mode = update.diagnostics.fusion_mode;
    if ( mode == estimator::FusionMode::kVisionOnly &&
         update.diagnostics.gyro_factor_count != 0U )
    {
      throw std::invalid_argument(
          "vio state probe vision-only graph has IMU factors" );
    }
    if ( mode == estimator::FusionMode::kGyroVisual )
    {
      if ( !update.diagnostics.gyro_graph_cost.has_value() )
      {
        throw std::invalid_argument(
            "vio state probe gyro graph has no objective diagnostics" );
      }
      validateGyroGraphCost( *update.diagnostics.gyro_graph_cost,
                             update.diagnostics.gyro_factor_count );
    }
    else if ( update.diagnostics.gyro_graph_cost.has_value() )
    {
      throw std::invalid_argument(
          "vio state probe non-gyro graph has gyro objective diagnostics" );
    }
    if ( !std::isfinite(
             update.diagnostics.gyro_alignment_residual_rms_rad ) ||
         update.diagnostics.gyro_alignment_residual_rms_rad < 0.0 )
    {
      throw std::invalid_argument(
          "vio state probe has invalid gyro alignment residual RMS" );
    }
    const Eigen::Quaterniond predicted_q = validatedQuaternion(
        state.predicted_T_W_B, "predicted" );
    const Eigen::Quaterniond graph_initial_q = validatedQuaternion(
        state.graph_initial_T_W_B, "graph initial" );
    const Eigen::Quaterniond posterior_q = validatedQuaternion(
        update.estimate->T_W_B, "posterior" );
    requireFiniteVector( state.prediction_bias_gyro,
                         "prediction gyro bias" );
    requireFiniteVector( update.diagnostics.bias_gyro,
                         "posterior gyro bias" );

    const Eigen::Vector3d predicted_p =
        state.predicted_T_W_B.translation();
    const Eigen::Vector3d graph_initial_p =
        state.graph_initial_T_W_B.translation();
    const Eigen::Vector3d posterior_p =
        update.estimate->T_W_B.translation();
    const bool gyro_cost_valid =
        update.diagnostics.gyro_graph_cost.has_value();
    const estimator::GyroGraphCostDiagnostics gyro_cost =
        update.diagnostics.gyro_graph_cost.value_or(
            estimator::GyroGraphCostDiagnostics{} );
    std::ostringstream row;
    row << std::setprecision( std::numeric_limits<double>::max_digits10 )
        << timestamp_ns << ',' << state.state_key << ','
        << update.diagnostics.prior_key << ','
        << update.diagnostics.window_size << ',' << ( is_keyframe ? 1 : 0 )
        << ',' << fusionModeName( update.diagnostics.fusion_mode ) << ','
        << update.diagnostics.gyro_factor_count << ','
        << update.diagnostics.gyro_alignment_residual_rms_rad << ','
        << state.imu_sample_count << ',' << state.imu_t_i_ns << ','
        << state.imu_t_j_ns << ',' << state.imu_dt_s << ','
        << ( state.prediction_valid ? 1 : 0 ) << ',' << predicted_p.x() << ','
        << predicted_p.y() << ',' << predicted_p.z() << ','
        << predicted_q.x() << ',' << predicted_q.y() << ','
        << predicted_q.z() << ',' << predicted_q.w() << ','
        << state.prediction_bias_gyro.x() << ','
        << state.prediction_bias_gyro.y() << ','
        << state.prediction_bias_gyro.z() << ',' << graph_initial_p.x() << ','
        << graph_initial_p.y() << ',' << graph_initial_p.z() << ','
        << graph_initial_q.x() << ',' << graph_initial_q.y() << ','
        << graph_initial_q.z() << ',' << graph_initial_q.w() << ','
        << posterior_p.x() << ','
        << posterior_p.y() << ',' << posterior_p.z() << ','
        << posterior_q.x() << ',' << posterior_q.y() << ','
        << posterior_q.z() << ',' << posterior_q.w() << ','
        << update.diagnostics.bias_gyro.x() << ','
        << update.diagnostics.bias_gyro.y() << ','
        << update.diagnostics.bias_gyro.z() << ','
        << ( gyro_cost_valid ? 1 : 0 ) << ','
        << gyro_cost.gyro_factor_count << ','
        << gyro_cost.interior_factor_count << ','
        << gyro_cost.stereo_factor_count << ','
        << gyro_cost.total_initial_cost << ','
        << gyro_cost.total_posterior_cost << ','
        << gyro_cost.gyro_initial_cost << ','
        << gyro_cost.gyro_posterior_cost << ','
        << gyro_cost.boundary_initial_cost << ','
        << gyro_cost.boundary_posterior_cost << ','
        << gyro_cost.boundary_initial_residual_norm_rad << ','
        << gyro_cost.boundary_posterior_residual_norm_rad << ','
        << gyro_cost.boundary_initial_whitened_norm << ','
        << gyro_cost.boundary_posterior_whitened_norm << ','
        << gyro_cost.interior_initial_cost << ','
        << gyro_cost.interior_posterior_cost << ','
        << gyro_cost.newest_initial_cost << ','
        << gyro_cost.newest_posterior_cost << ','
        << gyro_cost.newest_initial_residual_norm_rad << ','
        << gyro_cost.newest_posterior_residual_norm_rad << ','
        << gyro_cost.newest_initial_whitened_norm << ','
        << gyro_cost.newest_posterior_whitened_norm << ','
        << gyro_cost.stereo_initial_cost << ','
        << gyro_cost.stereo_posterior_cost << ','
        << gyro_cost.pose_prior_posterior_rotation_norm << ','
        << gyro_cost.pose_prior_posterior_translation_norm << ','
        << gyro_cost.pose_prior_posterior_cost << ','
        << gyro_cost.bias_prior_posterior_norm << ','
        << gyro_cost.bias_prior_posterior_cost << '\n';

    m_impl->out << row.str();
    m_impl->out.flush();
    if ( !m_impl->out )
    {
      throw std::runtime_error( "failed to write VIO state probe csv row" );
    }
    m_impl->last_timestamp_ns = timestamp_ns;
    m_impl->last_state_key    = state.state_key;
  }

}  // namespace phad::apps
