#pragma once

#include <cstdint>

#include "phad/common/timestamp.hpp"
#include "phad/sensor/stereo_frame.hpp"

/**
 * @file camera_id.hpp
 * @brief 单路相机标识与未配对图像事件。
 *
 * StereoPairSynchronizer / dataset per-camera API 共用。两目以上暂不预留。
 */

namespace phad::sensor
{

  enum class CameraId : std::uint8_t
  {
    kLeft  = 0,
    kRight = 1
  };

  struct ImageFrameEvent
  {
    CameraId          camera;
    common::Timestamp timestamp;
    Image             image;
  };

}  // namespace phad::sensor
