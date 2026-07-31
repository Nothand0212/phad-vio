#pragma once

#include "phad/estimator/types.hpp"
#include "phad/frontend/stereo_tracks.hpp"

namespace phad::apps
{

  /// Filter `kValid` tracks into an estimator measurement. Does not live in
  /// `phad_*` libraries (keeps estimator free of a frontend dependency).
  [[nodiscard]] inline estimator::KeyframeMeasurement toKeyframeMeasurement(
      const frontend::FrameTracks& tracks )
  {
    estimator::KeyframeMeasurement measurement;
    measurement.timestamp = tracks.timestamp;
    measurement.observations.reserve( tracks.observations.size() );
    for ( const frontend::TrackObservation& observation : tracks.observations )
    {
      if ( observation.status != frontend::StereoStatus::kValid )
      {
        continue;
      }
      measurement.observations.push_back( estimator::StereoObservation{
          .id           = observation.id,
          .left_pixel   = observation.left_pixel,
          .disparity_px = observation.disparity_px,
      } );
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
      case estimator::UpdateStatus::kRejected:
        return "rejected";
      case estimator::UpdateStatus::kFailed:
        return "failed";
    }
    return "unknown";
  }

}  // namespace phad::apps
