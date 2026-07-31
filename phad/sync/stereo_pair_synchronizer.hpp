#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

#include "phad/sensor/camera_id.hpp"
#include "phad/sensor/stereo_frame.hpp"

/**
 * @file stereo_pair_synchronizer.hpp
 * @brief B 族双目配对同步器（StereoOnly）。
 *
 * 左右两队列比较 front；|tL-tR|<=tol_ns 则配对（输出 left stamp），否则丢弃
 * 偏早一侧。pushImage 只表达校验结果；丢弃/溢出进计数器；配对经 tryPop。
 */

namespace phad::sync
{

  enum class PushStatus : std::uint8_t
  {
    kOk           = 0,
    kOutOfOrder   = 1,
    kDuplicate    = 2,
    kInvalidStamp = 3
  };

  struct StereoPairSynchronizerOptions
  {
    std::int64_t tol_ns    = 0;
    std::size_t  max_queue = 4096;
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
  };

  class StereoPairSynchronizer
  {
  public:
    explicit StereoPairSynchronizer(
        StereoPairSynchronizerOptions options = {} );

    [[nodiscard]] PushStatus                         pushImage( sensor::ImageFrameEvent event );
    [[nodiscard]] std::optional<sensor::StereoFrame> tryPop();
    void                                             flush();

    [[nodiscard]] const StereoPairDiagnostics& diagnostics() const noexcept;
    [[nodiscard]] std::optional<PushStatus>    stickyError() const noexcept;

  private:
    void drain();
    void updateMaxQueue( sensor::CameraId camera );

    StereoPairSynchronizerOptions       m_options;
    std::deque<sensor::ImageFrameEvent> m_left;
    std::deque<sensor::ImageFrameEvent> m_right;
    std::deque<sensor::StereoFrame>     m_ready;
    StereoPairDiagnostics               m_diag;
    std::optional<PushStatus>           m_sticky;
    std::optional<common::Timestamp>    m_last_left;
    std::optional<common::Timestamp>    m_last_right;
  };

}  // namespace phad::sync
