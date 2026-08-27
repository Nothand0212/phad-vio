#include "apps/keyframe_shadow_probe.hpp"

#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "phad/estimator/imu_rotation.hpp"

namespace phad::apps
{
  namespace
  {

    constexpr std::string_view kHeader =
        "frame,ts_ns,epoch_before,frames_since_kf,prod_selected,prod_rule,"
        "prod_empty,prod_bootstrap,prod_low_tracks,prod_timeout,"
        "prod_low_survival,prod_parallax,obs,common,survival,raw_px,"
        "pose_comp_px,pose_parallax_n,since_kf_ns,imu_valid,imu_reason,"
        "imu_sample_n,imu_gap,imu_t_i_ns,imu_t_j_ns,imu_dt_s,"
        "imu_angle_rad,bias_gyr_x,bias_gyr_y,bias_gyr_z,imu_comp_px,"
        "imu_parallax_n,imu_gt_10,imu_gt_15,imu_gt_30,status,"
        "epoch_committed,epoch_after";

    [[nodiscard]] std::string_view ruleName(
        const KeyframeRule rule ) noexcept
    {
      switch ( rule )
      {
        case KeyframeRule::kNone:
          return "none";
        case KeyframeRule::kBootstrap:
          return "bootstrap";
        case KeyframeRule::kLowTracks:
          return "low_tracks";
        case KeyframeRule::kTimeout:
          return "timeout";
        case KeyframeRule::kLowSurvival:
          return "low_survival";
        case KeyframeRule::kParallax:
          return "parallax";
      }
      return "unknown";
    }

    [[nodiscard]] std::string_view statusName(
        const estimator::UpdateStatus status ) noexcept
    {
      switch ( status )
      {
        case estimator::UpdateStatus::kOk:
          return "ok";
        case estimator::UpdateStatus::kRejected:
          return "rejected";
        case estimator::UpdateStatus::kFailed:
          return "failed";
      }
      return "unknown";
    }

    struct ShadowRow
    {
      std::uint64_t           frame           = 0;
      std::int64_t            ts_ns           = 0;
      std::uint64_t           epoch_before    = 0;
      std::int64_t            frames_since_kf = -1;
      bool                    prod_selected   = false;
      KeyframeRule            prod_rule       = KeyframeRule::kNone;
      KeyframeTriggers        prod_triggers;
      std::size_t             obs             = 0;
      std::size_t             common          = 0;
      double                  survival        = 0.0;
      double                  raw_px          = 0.0;
      double                  pose_comp_px    = 0.0;
      std::size_t             pose_parallax_n = 0;
      std::int64_t            since_kf_ns     = 0;
      bool                    imu_valid       = false;
      std::string             imu_reason      = "unavailable";
      std::size_t             imu_sample_n    = 0;
      bool                    imu_gap         = false;
      std::int64_t            imu_t_i_ns      = 0;
      std::int64_t            imu_t_j_ns      = 0;
      double                  imu_dt_s        = 0.0;
      double                  imu_angle_rad   = 0.0;
      Eigen::Vector3d         bias_gyr        = Eigen::Vector3d::Zero();
      double                  imu_comp_px     = 0.0;
      std::size_t             imu_parallax_n  = 0;
      bool                    imu_gt_10       = false;
      bool                    imu_gt_15       = false;
      bool                    imu_gt_30       = false;
      estimator::UpdateStatus status          = estimator::UpdateStatus::kFailed;
      bool                    epoch_committed = false;
      std::uint64_t           epoch_after     = 0;
      std::uint64_t           ticket          = 0;
    };

    void writeRow( std::ofstream& out, const ShadowRow& row )
    {
      const auto boolValue = []( const bool value ) { return value ? 1 : 0; };
      out << row.frame << ',' << row.ts_ns << ',' << row.epoch_before << ','
          << row.frames_since_kf << ',' << boolValue( row.prod_selected )
          << ',' << ruleName( row.prod_rule ) << ','
          << boolValue( row.prod_triggers.empty ) << ','
          << boolValue( row.prod_triggers.bootstrap ) << ','
          << boolValue( row.prod_triggers.low_tracks ) << ','
          << boolValue( row.prod_triggers.timeout ) << ','
          << boolValue( row.prod_triggers.low_survival ) << ','
          << boolValue( row.prod_triggers.parallax ) << ',' << row.obs << ','
          << row.common << ',' << row.survival << ',' << row.raw_px << ','
          << row.pose_comp_px << ',' << row.pose_parallax_n << ','
          << row.since_kf_ns << ',' << boolValue( row.imu_valid ) << ','
          << row.imu_reason << ',' << row.imu_sample_n << ','
          << boolValue( row.imu_gap ) << ',' << row.imu_t_i_ns << ','
          << row.imu_t_j_ns << ',' << row.imu_dt_s << ','
          << row.imu_angle_rad << ',' << row.bias_gyr.x() << ','
          << row.bias_gyr.y() << ',' << row.bias_gyr.z() << ','
          << row.imu_comp_px << ',' << row.imu_parallax_n << ','
          << boolValue( row.imu_gt_10 ) << ','
          << boolValue( row.imu_gt_15 ) << ','
          << boolValue( row.imu_gt_30 ) << ',' << statusName( row.status )
          << ',' << boolValue( row.epoch_committed ) << ',' << row.epoch_after
          << '\n';
      if ( !out )
      {
        throw std::runtime_error(
            "failed to write keyframe shadow probe csv row" );
      }
    }

  }  // namespace

  struct KeyframeShadowProbe::Impl
  {
    Impl( const std::filesystem::path&       path,
          camera::RectifiedStereoCalibration calibration,
          const bool                         imu_enabled_value )
        : out( path, std::ios::out | std::ios::trunc ),
          R_B_C( calibration.T_B_left_rectified().rotation() ),
          imu_enabled( imu_enabled_value )
    {
      if ( !out )
      {
        throw std::runtime_error(
            "failed to open keyframe shadow probe csv: " + path.string() );
      }
      out << kHeader << '\n';
      out << std::setprecision( std::numeric_limits<double>::max_digits10 );
      if ( !out )
      {
        throw std::runtime_error(
            "failed to write keyframe shadow probe csv header" );
      }
    }

    std::ofstream                       out;
    Eigen::Matrix3d                     R_B_C;
    bool                                imu_enabled = false;
    std::vector<sensor::ImuMeasurement> imu_samples;
    bool                                imu_gap  = false;
    Eigen::Vector3d                     bias_gyr = Eigen::Vector3d::Zero();
    std::optional<std::uint64_t>        last_kf_frame;
    std::optional<ShadowRow>            pending;
  };

  KeyframeShadowProbe::KeyframeShadowProbe(
      const std::filesystem::path&       path,
      camera::RectifiedStereoCalibration calibration, const bool imu_enabled )
      : m_impl( std::make_unique<Impl>(
            path, std::move( calibration ), imu_enabled ) )
  {
  }

  KeyframeShadowProbe::~KeyframeShadowProbe()
  {
    if ( m_impl->out.is_open() )
    {
      m_impl->out.flush();
      m_impl->out.close();
    }
  }

  void KeyframeShadowProbe::observe(
      const std::uint64_t                           frame,
      const std::span<const sensor::ImuMeasurement> imu_samples,
      const bool imu_gap, const frontend::FrameTracks& tracks,
      const KeyframeDecision& decision, const KeyframeEpochGate& gate )
  {
    if ( m_impl->pending.has_value() )
    {
      throw std::logic_error(
          "keyframe shadow frame must be resolved before observing another" );
    }
    if ( decision.evidence.timestamp != tracks.timestamp )
    {
      throw std::logic_error(
          "keyframe shadow decision does not match observed frame" );
    }

    auto first = imu_samples.begin();
    if ( !m_impl->imu_samples.empty() && first != imu_samples.end() &&
         m_impl->imu_samples.back().timestamp == first->timestamp )
    {
      ++first;
    }
    m_impl->imu_samples.insert(
        m_impl->imu_samples.end(), first, imu_samples.end() );
    m_impl->imu_gap = m_impl->imu_gap || imu_gap;

    ShadowRow row;
    row.frame           = frame;
    row.ts_ns           = tracks.timestamp.nanoseconds();
    row.epoch_before    = decision.evidence.epoch;
    row.frames_since_kf = m_impl->last_kf_frame.has_value()
                              ? static_cast<std::int64_t>(
                                    frame - *m_impl->last_kf_frame )
                              : -1;
    row.prod_selected   = decision.selected;
    row.prod_rule       = decision.rule;
    row.prod_triggers   = decision.evidence.triggers;
    row.obs             = decision.evidence.observation_count;
    row.common          = decision.evidence.common_count;
    row.survival        = decision.evidence.survival_ratio;
    row.raw_px          = decision.evidence.raw_parallax_px;
    row.pose_comp_px    = decision.evidence.compensated_parallax_px;
    row.pose_parallax_n = decision.evidence.parallax_count;
    row.since_kf_ns     = decision.evidence.since_keyframe_ns;
    row.imu_sample_n    = m_impl->imu_samples.size();
    row.imu_gap         = m_impl->imu_gap;
    row.bias_gyr        = m_impl->bias_gyr;
    row.ticket          = decision.ticket;

    if ( !m_impl->imu_enabled )
    {
      row.imu_reason = "imu_disabled";
    }
    else if ( !m_impl->last_kf_frame.has_value() )
    {
      row.imu_reason = "no_accepted_kf";
    }
    else if ( m_impl->imu_gap )
    {
      row.imu_reason = "imu_gap";
    }
    else
    {
      const estimator::ImuRotationResult rotation =
          estimator::integrateImuRotation(
              m_impl->imu_samples, m_impl->bias_gyr );
      row.imu_valid     = rotation.valid;
      row.imu_reason    = estimator::imuRotationErrorName( rotation.error );
      row.imu_t_i_ns    = rotation.t_i.nanoseconds();
      row.imu_t_j_ns    = rotation.t_j.nanoseconds();
      row.imu_dt_s      = rotation.dt_s;
      row.imu_angle_rad = rotation.angle_rad;
      if ( rotation.valid )
      {
        const Eigen::Matrix3d R_Ccur_Clkf =
            m_impl->R_B_C.transpose() * rotation.R_Bi_Bj.transpose() *
            m_impl->R_B_C;
        const KeyframeRotationEvidence evidence = gate.evaluateRotation(
            decision.ticket, tracks, R_Ccur_Clkf );
        row.imu_comp_px    = evidence.compensated_parallax_px;
        row.imu_parallax_n = evidence.parallax_count;
        row.imu_gt_10      = row.imu_comp_px > 10.0;
        row.imu_gt_15      = row.imu_comp_px > 15.0;
        row.imu_gt_30      = row.imu_comp_px > 30.0;
      }
    }

    m_impl->pending = std::move( row );
  }

  void KeyframeShadowProbe::resolve( const KeyframeEvent&   event,
                                     const Eigen::Vector3d& bias_gyr )
  {
    if ( !m_impl->pending.has_value() )
    {
      throw std::logic_error( "no keyframe shadow frame is pending" );
    }
    if ( m_impl->pending->ticket != event.ticket ||
         m_impl->pending->prod_selected != event.selected )
    {
      throw std::logic_error( "keyframe shadow event mismatch" );
    }

    m_impl->pending->status          = event.status;
    m_impl->pending->epoch_committed = event.epoch_committed;
    m_impl->pending->epoch_after     = event.epoch;
    writeRow( m_impl->out, *m_impl->pending );

    if ( event.epoch_committed )
    {
      if ( m_impl->imu_samples.empty() )
      {
        m_impl->imu_samples.clear();
      }
      else
      {
        const sensor::ImuMeasurement endpoint =
            m_impl->imu_samples.back();
        m_impl->imu_samples.assign( 1U, endpoint );
      }
      m_impl->imu_gap       = false;
      m_impl->last_kf_frame = m_impl->pending->frame;
    }
    if ( event.status == estimator::UpdateStatus::kOk )
    {
      m_impl->bias_gyr = bias_gyr;
    }
    m_impl->pending.reset();
  }

}  // namespace phad::apps
