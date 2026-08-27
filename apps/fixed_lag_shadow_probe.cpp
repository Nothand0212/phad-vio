#include "apps/fixed_lag_shadow_probe.hpp"

#include <Eigen/Geometry>
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
        "timestamp_ns,state_key,fusion_mode,active,update_ok,reset,"
        "reset_reason,current_epoch,cutoff_epoch,batch_window_n,"
        "smoother_pose_n,smoother_landmark_n,bias_present,bias_fixed,"
        "bias_delta_norm,user_factor_n,missing_owned_slot_n,"
        "timestamp_without_value_n,marginalized_pose_n,"
        "retired_landmark_n,new_landmark_generation_n,shadow_p_x,"
        "shadow_p_y,shadow_p_z,shadow_q_x,shadow_q_y,shadow_q_z,"
        "shadow_q_w,newest_rot_delta_rad,newest_trans_delta_m";

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
      throw std::invalid_argument(
          "fixed-lag shadow probe has invalid fusion mode" );
    }

    [[nodiscard]] std::string_view resetReasonName(
        const estimator::FixedLagShadowReset reason )
    {
      switch ( reason )
      {
        case estimator::FixedLagShadowReset::kNone:
          return "none";
        case estimator::FixedLagShadowReset::kBootstrap:
          return "bootstrap";
        case estimator::FixedLagShadowReset::kSegment:
          return "segment";
      }
      throw std::invalid_argument(
          "fixed-lag shadow probe has invalid reset reason" );
    }

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

    [[nodiscard]] bool isNonNegativeFinite( const double value )
    {
      return std::isfinite( value ) && value >= 0.0;
    }

    void validateInactive(
        const estimator::FixedLagShadowDiagnostics& shadow )
    {
      if ( shadow.reset ||
           shadow.reset_reason != estimator::FixedLagShadowReset::kNone ||
           shadow.current_epoch != 0U || shadow.cutoff_epoch != 0U ||
           shadow.batch_window_size != 0U ||
           shadow.smoother_pose_count != 0U ||
           shadow.smoother_landmark_count != 0U || shadow.bias_present ||
           shadow.bias_fixed || shadow.bias_delta_norm != 0.0 ||
           shadow.user_factor_count != 0U ||
           shadow.missing_owned_slot_count != 0U ||
           shadow.timestamp_without_value_count != 0U ||
           shadow.marginalized_pose_count != 0U ||
           shadow.retired_landmark_count != 0U ||
           shadow.new_landmark_generation_count != 0U ||
           !shadow.newest_T_W_B.matrix().isApprox(
               Eigen::Isometry3d::Identity().matrix(), 1e-12 ) ||
           shadow.newest_rotation_delta_rad != 0.0 ||
           shadow.newest_translation_delta_m != 0.0 )
      {
        throw std::invalid_argument(
            "fixed-lag shadow probe inactive row has shadow state" );
      }
    }

    void validateActive(
        const estimator::VioUpdateResult&           update,
        const estimator::GyroStateDiagnostics&      state,
        const estimator::FixedLagShadowDiagnostics& shadow )
    {
      if ( !shadow.update_ok || shadow.current_epoch != state.state_key ||
           shadow.cutoff_epoch > shadow.current_epoch ||
           shadow.batch_window_size == 0U ||
           shadow.batch_window_size != update.diagnostics.window_size ||
           shadow.smoother_pose_count != shadow.batch_window_size ||
           !shadow.bias_present || shadow.user_factor_count == 0U ||
           shadow.missing_owned_slot_count != 0U ||
           shadow.timestamp_without_value_count != 0U )
      {
        throw std::invalid_argument(
            "fixed-lag shadow probe active lifecycle invariant failed" );
      }
      const bool reset_reason_matches =
          shadow.reset
              ? shadow.reset_reason != estimator::FixedLagShadowReset::kNone
              : shadow.reset_reason == estimator::FixedLagShadowReset::kNone;
      if ( !reset_reason_matches )
      {
        throw std::invalid_argument(
            "fixed-lag shadow probe reset reason is inconsistent" );
      }
      if ( !isValidPose( shadow.newest_T_W_B ) ||
           !isNonNegativeFinite( shadow.bias_delta_norm ) ||
           !isNonNegativeFinite( shadow.newest_rotation_delta_rad ) ||
           !isNonNegativeFinite( shadow.newest_translation_delta_m ) )
      {
        throw std::invalid_argument(
            "fixed-lag shadow probe has invalid numeric diagnostics" );
      }
    }

  }  // namespace

  struct FixedLagShadowProbe::Impl
  {
    explicit Impl( const std::filesystem::path& path )
        : out( path, std::ios::out | std::ios::trunc )
    {
      if ( !out )
      {
        throw std::runtime_error(
            "failed to open fixed-lag shadow probe csv: " + path.string() );
      }
      out << kHeader << '\n';
      out.flush();
      if ( !out )
      {
        throw std::runtime_error(
            "failed to write fixed-lag shadow probe csv header" );
      }
    }

    std::ofstream                out;
    std::optional<std::int64_t>  last_timestamp_ns;
    std::optional<std::uint64_t> last_state_key;
  };

  FixedLagShadowProbe::FixedLagShadowProbe(
      const std::filesystem::path& path )
      : m_impl( std::make_unique<Impl>( path ) )
  {
  }

  FixedLagShadowProbe::~FixedLagShadowProbe()
  {
    if ( m_impl->out.is_open() )
    {
      m_impl->out.flush();
      m_impl->out.close();
    }
  }

  void FixedLagShadowProbe::write(
      const estimator::VioUpdateResult& update )
  {
    if ( update.status != estimator::UpdateStatus::kOk )
    {
      return;
    }
    if ( !update.estimate.has_value() ||
         !update.diagnostics.gyro_state.has_value() ||
         !update.diagnostics.fixed_lag_shadow.has_value() )
    {
      throw std::invalid_argument(
          "accepted fixed-lag shadow probe update is incomplete" );
    }

    const std::int64_t timestamp_ns =
        update.estimate->timestamp.nanoseconds();
    const estimator::GyroStateDiagnostics& state =
        *update.diagnostics.gyro_state;
    const estimator::FixedLagShadowDiagnostics& shadow =
        *update.diagnostics.fixed_lag_shadow;
    if ( m_impl->last_timestamp_ns.has_value() &&
         timestamp_ns <= *m_impl->last_timestamp_ns )
    {
      throw std::invalid_argument(
          "fixed-lag shadow probe timestamp is not strictly increasing" );
    }
    if ( m_impl->last_state_key.has_value() &&
         state.state_key <= *m_impl->last_state_key )
    {
      throw std::invalid_argument(
          "fixed-lag shadow probe state key is not strictly increasing" );
    }

    if ( !shadow.update_ok )
    {
      throw std::invalid_argument(
          "fixed-lag shadow probe update is not successful" );
    }
    if ( shadow.active )
    {
      if ( update.diagnostics.fusion_mode !=
           estimator::FusionMode::kGyroVisual )
      {
        throw std::invalid_argument(
            "fixed-lag shadow probe active row is not gyro-visual" );
      }
      validateActive( update, state, shadow );
    }
    else
    {
      if ( update.diagnostics.fusion_mode !=
           estimator::FusionMode::kVisionOnly )
      {
        throw std::invalid_argument(
            "fixed-lag shadow probe inactive row is not vision-only" );
      }
      validateInactive( shadow );
    }

    const Eigen::Quaterniond quaternion( shadow.newest_T_W_B.linear() );
    if ( !quaternion.coeffs().allFinite() ||
         std::abs( quaternion.norm() - 1.0 ) > 1e-9 )
    {
      throw std::invalid_argument(
          "fixed-lag shadow probe has invalid quaternion" );
    }
    const Eigen::Vector3d position = shadow.newest_T_W_B.translation();
    std::ostringstream    row;
    row << std::setprecision( std::numeric_limits<double>::max_digits10 )
        << timestamp_ns << ',' << state.state_key << ','
        << fusionModeName( update.diagnostics.fusion_mode ) << ','
        << ( shadow.active ? 1 : 0 ) << ','
        << ( shadow.update_ok ? 1 : 0 ) << ','
        << ( shadow.reset ? 1 : 0 ) << ','
        << resetReasonName( shadow.reset_reason ) << ','
        << shadow.current_epoch << ',' << shadow.cutoff_epoch << ','
        << shadow.batch_window_size << ',' << shadow.smoother_pose_count
        << ',' << shadow.smoother_landmark_count << ','
        << ( shadow.bias_present ? 1 : 0 ) << ','
        << ( shadow.bias_fixed ? 1 : 0 ) << ',' << shadow.bias_delta_norm
        << ',' << shadow.user_factor_count << ','
        << shadow.missing_owned_slot_count << ','
        << shadow.timestamp_without_value_count << ','
        << shadow.marginalized_pose_count << ','
        << shadow.retired_landmark_count << ','
        << shadow.new_landmark_generation_count << ',' << position.x() << ','
        << position.y() << ',' << position.z() << ',' << quaternion.x()
        << ',' << quaternion.y() << ',' << quaternion.z() << ','
        << quaternion.w() << ',' << shadow.newest_rotation_delta_rad << ','
        << shadow.newest_translation_delta_m << '\n';
    m_impl->out << row.str();
    m_impl->out.flush();
    if ( !m_impl->out )
    {
      throw std::runtime_error(
          "failed to write fixed-lag shadow probe csv row" );
    }
    m_impl->last_timestamp_ns = timestamp_ns;
    m_impl->last_state_key    = state.state_key;
  }

}  // namespace phad::apps
