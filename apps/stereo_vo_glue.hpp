#pragma once

#include <utility>

#include "phad/estimator/types.hpp"
#include "phad/frontend/stereo_tracks.hpp"

namespace phad::apps
{

  /// Filter tracks into an estimator measurement. `kValid` tracks carry their
  /// stereo disparity; Slice ⑦ additionally admits `kNoRightMatch` tracks that
  /// survived at least two frontend frames — these carry `disparity_px = 0`
  /// (stereo failed, no depth). Zero-disparity observations never seed or
  /// constrain the graph, but keep the track alive in the window until
  /// stereo returns. Does not live in `phad_*` libraries (keeps estimator
  /// free of a frontend dependency).
  [[nodiscard]] inline estimator::VioMeasurement toVioMeasurement(
      const frontend::FrameTracks& tracks, sensor::ImuPayload imu )
  {
    estimator::VioMeasurement measurement{
        .m_timestamp    = tracks.timestamp,
        .m_observations = {},
        .m_imu          = std::move( imu ) };
    measurement.m_observations.reserve( tracks.observations.size() );
    for ( const frontend::TrackObservation& observation : tracks.observations )
    {
      if ( observation.status == frontend::StereoStatus::kValid )
      {
        measurement.m_observations.push_back( estimator::StereoObservation{
            .id           = observation.id,
            .left_pixel   = observation.left_pixel,
            .disparity_px = observation.disparity_px,
        } );
      }
      else if ( observation.status == frontend::StereoStatus::kNoRightMatch &&
                observation.length >= 2 )
      {
        // Slice ⑦: stereo failed but the track is stable — zero-disparity
        // observation (kept in the window; seeds only via a later stereo
        // observation).
        measurement.m_observations.push_back( estimator::StereoObservation{
            .id           = observation.id,
            .left_pixel   = observation.left_pixel,
            .disparity_px = 0.0,
        } );
      }
    }
    return measurement;
  }

  [[nodiscard]] inline const char* updateStatusName(
      estimator::UpdateStatus status )
  {
    switch ( status )
    {
      case estimator::UpdateStatus::kOk:
        return "ok";
      case estimator::UpdateStatus::kInitializing:
        return "initializing";
      case estimator::UpdateStatus::kDiscontinuity:
        return "discontinuity";
      case estimator::UpdateStatus::kVisualOutage:
        return "visual_outage";
      case estimator::UpdateStatus::kInvalidInput:
        return "invalid_input";
      case estimator::UpdateStatus::kRejected:
        return "rejected";
      case estimator::UpdateStatus::kFailed:
        return "failed";
    }
    return "unknown";
  }

}  // namespace phad::apps
