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
#include "phad/estimator/internal/gyro_interval_reducer.hpp"
#include "phad/estimator/types.hpp"

namespace phad::estimator::internal
{

  struct GyroFrameState
  {
    Eigen::Vector3d                      m_bias_radps   = Eigen::Vector3d::Zero();
    std::uint32_t                        m_segment_id   = 0;
    std::uint64_t                        m_component_id = 0;
    std::optional<std::uint64_t>         m_predecessor_frame_index;
    std::optional<ValidatedGyroInterval> m_interval;
  };

  struct WindowFrame
  {
    std::uint64_t                  m_frame_index = 0;
    common::Timestamp              m_timestamp{ 0 };
    Eigen::Isometry3d              m_T_W_B = Eigen::Isometry3d::Identity();
    std::vector<StereoObservation> m_observations;
    bool                           m_is_keyframe = true;
    std::optional<GyroFrameState>  m_gyro;
  };

  struct StereoVoUpdateState
  {
    std::deque<WindowFrame>                         m_window;
    std::unordered_map<LandmarkId, Eigen::Vector3d> m_landmarks_w;
    std::unordered_map<LandmarkId, std::vector<common::Timestamp>>
        m_track_times;
    std::unordered_map<LandmarkId, Eigen::Isometry3d>
                                                      m_T_W_B_last_stereo;
    std::optional<Eigen::Isometry3d>                  m_T_W_B_last_accepted;
    std::optional<Eigen::Isometry3d>                  m_T_W_B_prev_accepted;
    std::uint64_t                                     m_next_frame_index = 0;
    bool                                              m_initialized      = false;
    std::uint32_t                                     m_segment_id       = 0;
    std::unordered_set<LandmarkId>                    m_culled_ids;
    std::unordered_map<LandmarkId, StereoObservation> m_pending_seed_obs;
    std::optional<common::Timestamp>                  m_eligible_visual_rejected_timestamp;
    std::uint64_t                                     m_next_gyro_component_id = 0;
  };

  class StereoVoUpdateTransaction final
  {
  public:
    explicit StereoVoUpdateTransaction(
        std::unique_ptr<StereoVoUpdateState>& owner )
        : m_owner( owner ), m_before( clone( owner ) )
    {
    }

    ~StereoVoUpdateTransaction() noexcept
    {
      rollback();
    }

    StereoVoUpdateTransaction( const StereoVoUpdateTransaction& ) = delete;
    StereoVoUpdateTransaction& operator=(
        const StereoVoUpdateTransaction& )                              = delete;
    StereoVoUpdateTransaction( StereoVoUpdateTransaction&& )            = delete;
    StereoVoUpdateTransaction& operator=( StereoVoUpdateTransaction&& ) = delete;

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
    [[nodiscard]] static std::unique_ptr<StereoVoUpdateState> clone(
        const std::unique_ptr<StereoVoUpdateState>& owner )
    {
      if ( !owner )
      {
        throw std::invalid_argument(
            "StereoVoUpdateTransaction owner must not be null" );
      }
      return std::make_unique<StereoVoUpdateState>( *owner );
    }

    std::unique_ptr<StereoVoUpdateState>& m_owner;
    std::unique_ptr<StereoVoUpdateState>  m_before;
  };

}  // namespace phad::estimator::internal
