#pragma once

#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_frame.hpp"

namespace phad::sensor
{

  /**
   * @brief StereoImuPacket 已配对双目 + 该帧与上一帧之间的 IMU 段。
   *
   * 由 `phad::sync::StereoPairSynchronizer` 在配对时构造(M4.1):
   * - `samples` 是区间 `[t_prev, frame.timestamp]` 的原始 IMU 样本,含两端
   *   线性插值样本;段内 ΣΔt ≡ 图像间隔(非 `imu_gap` 时);
   * - 左端样本归本段,相邻段共享右端样本(右端 = 下段的左端);
   * - `imu_gap` 表示段不完整(帧间 IMU 大间断或两端无法插值),消费方应跳过
   *   该段的预积分因子(视觉照常)。
   */
  struct StereoImuPacket
  {
    StereoFrame                  frame;
    std::vector<ImuMeasurement>  samples;
    common::Timestamp            t_prev;
    bool                         imu_gap = false;
  };

}  // namespace phad::sensor
