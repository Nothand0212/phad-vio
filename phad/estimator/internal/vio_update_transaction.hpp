#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/estimator/internal/imu_interval.hpp"
#include "phad/estimator/types.hpp"

namespace phad::estimator::internal
{

  struct WindowFrame
  {
    std::uint64_t                        m_frame_index = 0;
    common::Timestamp                    m_timestamp{ 0 };
    Eigen::Isometry3d                    m_T_W_B = Eigen::Isometry3d::Identity();
    std::vector<StereoObservation>       m_observations;
    bool                                 m_is_keyframe = true;
    Eigen::Vector3d                      m_v_W_B       = Eigen::Vector3d::Zero();
    ImuBias                              m_bias;
    std::optional<std::uint64_t>         m_predecessor_frame_index;
    std::optional<NormalizedImuInterval> m_imu;
  };

  struct VioUpdateState
  {
    std::deque<WindowFrame>                         m_window;
    std::unordered_map<LandmarkId, Eigen::Vector3d> m_landmarks_w;
    std::unordered_map<LandmarkId, std::vector<common::Timestamp>>
        m_track_times;
    std::unordered_map<LandmarkId, Eigen::Isometry3d>
                                                      m_T_W_B_last_stereo;
    std::uint64_t                                     m_next_frame_index = 0;
    bool                                              m_initialized      = false;
    std::uint32_t                                     m_segment_id       = 0;
    std::unordered_set<LandmarkId>                    m_culled_ids;
    std::unordered_map<LandmarkId, StereoObservation> m_pending_seed_obs;
    std::vector<sensor::ImuMeasurement>               m_bootstrap_nodes;
    std::optional<common::Timestamp>                  m_continuity_anchor;
    std::optional<common::Timestamp>                  m_last_visual_support_timestamp;
    std::int64_t                                      m_unsupported_span_ns      = 0;
    std::int64_t                                      m_visual_coast_duration_ns = 0;
    std::uint64_t                                     m_non_keyframe_evictions   = 0;
    std::uint64_t                                     m_imu_reintegrations       = 0;
    VioDiagnostics                                    m_vio_diagnostics;
  };

  class VioUpdateTransaction final
  {
  public:
    explicit VioUpdateTransaction(
        std::unique_ptr<VioUpdateState>& owner )
        : m_owner( owner ), m_before( clone( owner ) )
    {
    }

    ~VioUpdateTransaction() noexcept
    {
      rollback();
    }

    VioUpdateTransaction( const VioUpdateTransaction& )            = delete;
    VioUpdateTransaction& operator=( const VioUpdateTransaction& ) = delete;
    VioUpdateTransaction( VioUpdateTransaction&& )                 = delete;
    VioUpdateTransaction& operator=( VioUpdateTransaction&& )      = delete;

    void rollback() noexcept
    {
      if ( !m_before )
      {
        return;
      }
      m_owner.swap( m_before );
      m_before.reset();
    }

    void commit() noexcept
    {
      m_before.reset();
    }

  private:
    [[nodiscard]] static std::unique_ptr<VioUpdateState> clone(
        const std::unique_ptr<VioUpdateState>& owner )
    {
      if ( !owner )
      {
        throw std::invalid_argument(
            "VioUpdateTransaction owner must not be null" );
      }
      return std::make_unique<VioUpdateState>( *owner );
    }

    std::unique_ptr<VioUpdateState>& m_owner;
    std::unique_ptr<VioUpdateState>  m_before;
  };

}  // namespace phad::estimator::internal
