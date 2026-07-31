#pragma once

#include <memory>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/frontend/stereo_tracks.hpp"
#include "phad/sensor/stereo_frame.hpp"

namespace phad::frontend
{

  struct StereoTrackerOptions
  {
    int    max_tracks          = 200;
    double quality_level       = 0.01;
    double min_distance_px     = 20.0;
    int    mask_radius_px      = 20;
    int    lk_window_px        = 21;
    int    lk_pyramid_levels   = 3;
    double forward_backward_px = 0.5;
    double max_epipolar_px     = 1.5;
    double min_disparity_px    = 0.5;
    double min_depth_m         = 0.3;
    double max_depth_m         = 40.0;
  };

  /**
   * @brief Left-temporal LK tracker with GFTT refill on rectified stereo.
   *
   * Slice M2.2-② fills left tracks only; right-match status stays
   * `kNoRightMatch` until the stereo-match slice.
   */
  class StereoTracker
  {
  public:
    StereoTracker( camera::RectifiedStereoCalibration calibration,
                   StereoTrackerOptions               options = {} );
    ~StereoTracker();

    StereoTracker( const StereoTracker& )            = delete;
    StereoTracker& operator=( const StereoTracker& ) = delete;
    StereoTracker( StereoTracker&& ) noexcept;
    StereoTracker& operator=( StereoTracker&& ) noexcept;

    [[nodiscard]] FrameTracks process(
        const sensor::StereoFrame& rectified );

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
  };

}  // namespace phad::frontend
