#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "phad/sensor/camera_id.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/stereo_frame.hpp"
#include "phad/sensor/stereo_imu_packet.hpp"

/**
 * @file stereo_pair_synchronizer.hpp
 * @brief B 族双目配对同步器（StereoOnly + IMU 流，M4.1 扩展）。
 *
 * 左右两队列比较 front；|tL-tR|<=tol_ns 则配对（输出 left stamp），否则丢弃
 * 偏早一侧。pushImage / pushImu 只表达校验结果；丢弃/溢出进计数器；配对经
 * tryPop / tryPopPacket。M4.1: pushImu 维护独立 IMU 队列,配对时切段
 * [t_prev, t_cur] 并两端线性插值,构造 StereoImuPacket;大间断标 imu_gap。
 */

namespace phad::sync
{

  enum class PushStatus : std::uint8_t
  {
    kOk           = 0,
    kOutOfOrder   = 1,
    kDuplicate    = 2,
    kInvalidStamp = 3,
    kInvalidValue = 4  // 非有限 IMU 测量(M4.1)
  };

  struct StereoPairSynchronizerOptions
  {
    std::int64_t tol_ns         = 0;
    std::size_t  max_queue      = 4096;
    // M4.1: IMU 队列上限(200 Hz 下 4096 ≈ 20 s 缓冲)
    std::size_t  max_imu_queue  = 4096;
    // M4.1: 帧间 IMU 大间断判定阈值(默认 0.5 s;段宽超过即 imu_gap)
    std::int64_t imu_gap_ns     = 500'000'000;
  };

  struct StereoPairDiagnostics
  {
    std::uint64_t pushed_left            = 0;
    std::uint64_t pushed_right           = 0;
    std::uint64_t emitted_stereo         = 0;
    std::uint64_t dropped_left           = 0;
    std::uint64_t dropped_right          = 0;
    std::uint64_t dropped_left_overflow  = 0;
    std::uint64_t dropped_right_overflow = 0;
    std::size_t   max_left_queue         = 0;
    std::size_t   max_right_queue        = 0;
    // M4.1: IMU 流
    std::uint64_t pushed_imu             = 0;
    std::uint64_t dropped_imu            = 0;  // 过期(早于段左端) + flush 剩余
    std::uint64_t dropped_imu_overflow   = 0;
    std::uint64_t imu_gap_count          = 0;
    std::size_t   max_imu_queue          = 0;
  };

  class StereoPairSynchronizer
  {
  public:
    explicit StereoPairSynchronizer(
        StereoPairSynchronizerOptions options = {} );

    [[nodiscard]] PushStatus                         pushImage( sensor::ImageFrameEvent event );
    /// M4.1: IMU 流单调性校验 + 有界入队;sticky 与图像路径独立。
    [[nodiscard]] PushStatus                         pushImu( sensor::ImuMeasurement measurement );
    [[nodiscard]] std::optional<sensor::StereoFrame> tryPop();
    /// M4.1: 与 tryPop 同源,取完整 StereoImuPacket(含帧间 IMU 段)。
    [[nodiscard]] std::optional<sensor::StereoImuPacket> tryPopPacket();
    void                                             flush();

    [[nodiscard]] const StereoPairDiagnostics& diagnostics() const noexcept;
    [[nodiscard]] std::optional<PushStatus>    stickyError() const noexcept;
    /// M4.1: IMU 路径的 sticky(与图像路径独立)。
    [[nodiscard]] std::optional<PushStatus>    imuStickyError() const noexcept;

  private:
    void drain();
    void updateMaxQueue( sensor::CameraId camera );
    void dropExpiredImu( common::Timestamp t_prev );
    [[nodiscard]] sensor::StereoImuPacket makePacket( common::Timestamp t_cur,
                                                      std::optional<common::Timestamp> t_prev,
                                                      sensor::StereoFrame              frame );

    StereoPairSynchronizerOptions       m_options;
    std::deque<sensor::ImageFrameEvent> m_left;
    std::deque<sensor::ImageFrameEvent> m_right;
    std::deque<sensor::StereoImuPacket> m_ready;
    StereoPairDiagnostics               m_diag;
    std::optional<PushStatus>           m_sticky;
    std::optional<common::Timestamp>    m_last_left;
    std::optional<common::Timestamp>    m_last_right;
    // M4.1: IMU 流
    std::deque<sensor::ImuMeasurement>  m_imu;
    std::optional<PushStatus>           m_imu_sticky;
    std::optional<common::Timestamp>    m_last_imu;
    std::optional<common::Timestamp>    m_last_emitted_left;  // 上一配对 left stamp
    std::optional<sensor::ImuMeasurement> m_last_boundary;    // 上一段右端样本(下段左端)
  };

}  // namespace phad::sync
