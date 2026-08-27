#include "apps/keyframe_epoch_gate.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "phad/common/landmark_id.hpp"

namespace phad::apps
{
  namespace
  {

    constexpr double      kParallaxPx   = 30.0;
    constexpr std::size_t kMinPnpTracks = 10U;
    constexpr double      kMinSurvival  = 0.6;

    struct EpochState
    {
      std::unordered_map<common::LandmarkId, Eigen::Vector2d>
                        last_kf_pixels;
      common::Timestamp last_kf_timestamp{ 0 };
      std::uint64_t     epoch = 0;
      Eigen::Matrix3d   last_accepted_rotation =
          Eigen::Matrix3d::Identity();
      Eigen::Matrix3d last_kf_rotation = Eigen::Matrix3d::Identity();
    };

    struct PendingDecision
    {
      std::uint64_t                                               ticket   = 0;
      bool                                                        selected = false;
      common::Timestamp                                           timestamp{ 0 };
      std::size_t                                                 observation_count = 0;
      std::vector<std::pair<common::LandmarkId, Eigen::Vector2d>> pixels;
    };

    class EvidenceBuilder
    {
    public:
      explicit EvidenceBuilder(
          camera::RectifiedStereoCalibration calibration,
          std::int64_t                       timeout_ns )
          : m_calibration( std::move( calibration ) ),
            m_timeout_ns( timeout_ns )
      {
      }

      [[nodiscard]] KeyframeEvidence build(
          const frontend::FrameTracks& tracks,
          const EpochState&            state ) const
      {
        KeyframeEvidence evidence;
        evidence.timestamp         = tracks.timestamp;
        evidence.epoch             = state.epoch;
        evidence.observation_count = tracks.observations.size();
        evidence.since_keyframe_ns =
            tracks.timestamp.nanoseconds() -
            state.last_kf_timestamp.nanoseconds();

        const Eigen::Matrix3d R_kf_to_cur =
            state.last_accepted_rotation.transpose() *
            state.last_kf_rotation;
        const KeyframeRotationEvidence rotation = measure(
            tracks, state, R_kf_to_cur );
        evidence.common_count    = rotation.common_count;
        evidence.parallax_count  = rotation.parallax_count;
        evidence.raw_parallax_px = rotation.raw_parallax_px;
        evidence.compensated_parallax_px =
            rotation.compensated_parallax_px;

        evidence.survival_ratio =
            static_cast<double>( evidence.common_count ) /
            static_cast<double>(
                std::max( state.last_kf_pixels.size(), std::size_t{ 1 } ) );
        evidence.triggers.empty     = tracks.observations.empty();
        evidence.triggers.bootstrap = state.epoch < 2U;
        evidence.triggers.low_tracks =
            tracks.observations.size() < kMinPnpTracks;
        evidence.triggers.timeout =
            evidence.since_keyframe_ns > m_timeout_ns;
        evidence.triggers.low_survival =
            evidence.survival_ratio < kMinSurvival;
        evidence.triggers.parallax =
            evidence.parallax_count > 0U &&
            evidence.compensated_parallax_px > kParallaxPx;
        return evidence;
      }

      [[nodiscard]] KeyframeRotationEvidence measure(
          const frontend::FrameTracks& tracks, const EpochState& state,
          const Eigen::Matrix3d& R_kf_to_cur ) const
      {
        KeyframeRotationEvidence evidence;
        const double             fx = m_calibration.fxPixels();
        const double             fy = m_calibration.fyPixels();
        const double             cx = m_calibration.cxPixels();
        const double             cy = m_calibration.cyPixels();

        double raw_parallax_sum         = 0.0;
        double compensated_parallax_sum = 0.0;
        for ( const auto& obs : tracks.observations )
        {
          const auto it = state.last_kf_pixels.find( obs.id );
          if ( it == state.last_kf_pixels.end() )
          {
            continue;
          }

          ++evidence.common_count;
          const Eigen::Vector2d& p_last_kf = it->second;
          const Eigen::Vector2d& p_cur     = obs.left_pixel;
          const double           raw_dx    = p_cur.x() - p_last_kf.x();
          const double           raw_dy    = p_cur.y() - p_last_kf.y();
          raw_parallax_sum += std::sqrt( raw_dx * raw_dx + raw_dy * raw_dy );

          const Eigen::Vector3d ray_kf(
              ( p_last_kf.x() - cx ) / fx,
              ( p_last_kf.y() - cy ) / fy, 1.0 );
          const Eigen::Vector3d ray_cur = R_kf_to_cur * ray_kf;
          if ( ray_cur.z() <= 1e-6 )
          {
            continue;
          }
          const Eigen::Vector2d p_comp(
              ray_cur.x() / ray_cur.z() * fx + cx,
              ray_cur.y() / ray_cur.z() * fy + cy );
          const double dx = p_cur.x() - p_comp.x();
          const double dy = p_cur.y() - p_comp.y();
          compensated_parallax_sum += std::sqrt( dx * dx + dy * dy );
          ++evidence.parallax_count;
        }

        if ( evidence.common_count > 0U )
        {
          evidence.raw_parallax_px =
              raw_parallax_sum /
              static_cast<double>( evidence.common_count );
        }
        if ( evidence.parallax_count > 0U )
        {
          evidence.compensated_parallax_px =
              compensated_parallax_sum /
              static_cast<double>( evidence.parallax_count );
        }

        return evidence;
      }

    private:
      camera::RectifiedStereoCalibration m_calibration;
      std::int64_t                       m_timeout_ns = 0;
    };

    class EpochScheduler
    {
    public:
      [[nodiscard]] static KeyframeRule select(
          const KeyframeEvidence& evidence ) noexcept
      {
        if ( evidence.triggers.empty )
        {
          return KeyframeRule::kNone;
        }
        if ( evidence.triggers.bootstrap )
        {
          return KeyframeRule::kBootstrap;
        }
        if ( evidence.triggers.low_tracks )
        {
          return KeyframeRule::kLowTracks;
        }
        if ( evidence.triggers.timeout )
        {
          return KeyframeRule::kTimeout;
        }
        if ( evidence.triggers.low_survival )
        {
          return KeyframeRule::kLowSurvival;
        }
        if ( evidence.triggers.parallax )
        {
          return KeyframeRule::kParallax;
        }
        return KeyframeRule::kNone;
      }
    };

  }  // namespace

  struct KeyframeEpochGate::Impl
  {
    explicit Impl( camera::RectifiedStereoCalibration calibration,
                   std::int64_t                       timeout_ns )
        : evidence_builder( std::move( calibration ), timeout_ns )
    {
    }

    EpochState                     state;
    EvidenceBuilder                evidence_builder;
    std::optional<PendingDecision> pending;
    std::uint64_t                  next_ticket = 1;
  };

  KeyframeEpochGate::KeyframeEpochGate(
      camera::RectifiedStereoCalibration calibration,
      std::int64_t                       timeout_ns )
      : m_impl( std::make_unique<Impl>( std::move( calibration ),
                                        timeout_ns ) )
  {
  }

  KeyframeEpochGate::~KeyframeEpochGate() = default;

  KeyframeEpochGate::KeyframeEpochGate( KeyframeEpochGate&& ) noexcept =
      default;

  KeyframeEpochGate& KeyframeEpochGate::operator=(
      KeyframeEpochGate&& ) noexcept = default;

  KeyframeDecision KeyframeEpochGate::decide(
      const frontend::FrameTracks& tracks )
  {
    if ( m_impl->pending.has_value() )
    {
      throw std::logic_error(
          "keyframe decision must be resolved before deciding another frame" );
    }

    KeyframeDecision decision;
    decision.ticket   = m_impl->next_ticket++;
    decision.evidence = m_impl->evidence_builder.build( tracks, m_impl->state );
    decision.rule     = EpochScheduler::select( decision.evidence );
    decision.selected = decision.rule != KeyframeRule::kNone;

    PendingDecision pending;
    pending.ticket            = decision.ticket;
    pending.selected          = decision.selected;
    pending.timestamp         = tracks.timestamp;
    pending.observation_count = tracks.observations.size();
    if ( decision.selected )
    {
      pending.pixels.reserve( tracks.observations.size() );
      for ( const auto& obs : tracks.observations )
      {
        pending.pixels.emplace_back( obs.id, obs.left_pixel );
      }
    }
    m_impl->pending = std::move( pending );
    return decision;
  }

  KeyframeRotationEvidence KeyframeEpochGate::evaluateRotation(
      const std::uint64_t ticket, const frontend::FrameTracks& tracks,
      const Eigen::Matrix3d& R_kf_to_cur ) const
  {
    if ( !m_impl->pending.has_value() )
    {
      throw std::logic_error( "no keyframe decision is pending" );
    }
    if ( m_impl->pending->ticket != ticket )
    {
      throw std::logic_error( "keyframe decision ticket mismatch" );
    }
    if ( m_impl->pending->timestamp != tracks.timestamp ||
         m_impl->pending->observation_count != tracks.observations.size() )
    {
      throw std::logic_error(
          "alternative rotation evidence does not match pending frame" );
    }
    return m_impl->evidence_builder.measure(
        tracks, m_impl->state, R_kf_to_cur );
  }

  KeyframeEvent KeyframeEpochGate::resolve(
      const std::uint64_t ticket, const KeyframeFeedback& feedback )
  {
    if ( !m_impl->pending.has_value() )
    {
      throw std::logic_error( "no keyframe decision is pending" );
    }
    if ( m_impl->pending->ticket != ticket )
    {
      throw std::logic_error( "keyframe decision ticket mismatch" );
    }

    const bool selected        = m_impl->pending->selected;
    bool       epoch_committed = false;
    if ( selected && feedback.status == estimator::UpdateStatus::kOk )
    {
      std::unordered_map<common::LandmarkId, Eigen::Vector2d> pixels;
      for ( const auto& [ id, pixel ] : m_impl->pending->pixels )
      {
        pixels[ id ] = pixel;
      }
      m_impl->state.last_kf_pixels    = std::move( pixels );
      m_impl->state.last_kf_timestamp = m_impl->pending->timestamp;
      m_impl->state.last_kf_rotation =
          m_impl->state.last_accepted_rotation;
      ++m_impl->state.epoch;
      epoch_committed = true;
    }

    if ( feedback.status == estimator::UpdateStatus::kOk &&
         feedback.R_W_B.has_value() )
    {
      m_impl->state.last_accepted_rotation = *feedback.R_W_B;
    }

    KeyframeEvent event;
    event.ticket          = ticket;
    event.selected        = selected;
    event.status          = feedback.status;
    event.epoch_committed = epoch_committed;
    event.epoch           = m_impl->state.epoch;
    m_impl->pending.reset();
    return event;
  }

}  // namespace phad::apps
