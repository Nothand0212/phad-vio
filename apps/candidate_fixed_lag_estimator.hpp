#pragma once

#include <cstdint>
#include <memory>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/types.hpp"

namespace phad::apps
{

  /// M4.4 P2b: candidate-owned fixed-lag 图估计器 —— 唯一 concrete
  /// fixed-lag implementation（GTSAM 类型留在 PIMPL 内，不越过 apps 库）。
  ///
  /// Slice ② 骨架期：KF-only pose 链（bootstrap Prior + 相邻 KF Between
  /// 占位），landmark 观测与 gyro bias 由 slice ③/④ 加入；非 KF 帧返回
  /// 上一 estimate（slice ⑤ 换成 PnP）。异常时内部已恢复 snapshot，
  /// 原样 rethrow（由 CandidatePipeline 映射为 kEstimatorInvariant
  /// terminal）。本类不是 batch `StereoVoEstimator` 的替代品，只服务
  /// candidate 路径。
  struct FixedLagGraphStats
  {
    std::uint32_t active_pose_count       = 0;
    std::uint32_t active_factor_count     = 0;
    std::uint32_t marginalized_pose_count = 0;
    std::uint32_t retired_landmark_count  = 0;
  };

  struct FixedLagUpdateResult
  {
    estimator::VioUpdateResult vio;
    FixedLagGraphStats         graph;
  };

  class CandidateFixedLagEstimator
  {
  public:
    CandidateFixedLagEstimator(
        camera::RectifiedStereoCalibration calibration,
        estimator::EstimatorOptions        options );
    ~CandidateFixedLagEstimator();

    CandidateFixedLagEstimator( const CandidateFixedLagEstimator& ) = delete;
    CandidateFixedLagEstimator& operator=(
        const CandidateFixedLagEstimator& ) = delete;
    CandidateFixedLagEstimator( CandidateFixedLagEstimator&& ) noexcept;
    CandidateFixedLagEstimator& operator=( CandidateFixedLagEstimator&& ) noexcept;

    /// 每 accepted 帧调用一次（含非 KF 帧）。内部 frame 计数即 graph
    /// timestamp（double 帧序号，cutoff 严格 `<` 由 smoother lag 保证）。
    [[nodiscard]] FixedLagUpdateResult update(
        const estimator::KeyframeMeasurement& measurement,
        bool                                  keyframe );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::apps
