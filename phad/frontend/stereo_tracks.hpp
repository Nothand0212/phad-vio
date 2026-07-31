#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "phad/common/timestamp.hpp"

namespace phad::frontend
{

  using LandmarkId = std::uint64_t;

  enum class StereoStatus : std::uint8_t
  {
    kValid = 0,
    kNoRightMatch,
    kInvalidDisparity,
    kDepthOutOfRange
  };

  struct TrackObservation
  {
    LandmarkId      id;
    Eigen::Vector2d left_pixel;
    double          disparity_px;  // meaningful only when status == kValid
    StereoStatus    status;
    std::uint32_t   length;  // observations so far, including this frame
  };

  struct FrameStats
  {
    std::uint32_t tracked;  // temporal LK survivors
    std::uint32_t detected;  // new detections this frame
    std::uint32_t forward_backward_rejected;
    std::uint32_t epipolar_rejected;
    std::uint32_t disparity_rejected;
    std::uint32_t depth_rejected;
    double        epipolar_median_px;  // right-match successes only
    double        epipolar_p95_px;
    std::uint32_t track_length_median;
    std::uint32_t track_length_max;
  };

  struct FrameTracks
  {
    common::Timestamp             timestamp;
    std::vector<TrackObservation> observations;
    FrameStats                    stats;
  };

}  // namespace phad::frontend
