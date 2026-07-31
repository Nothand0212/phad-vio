#include "phad/frontend/stereo_tracker.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>

#include "tests/frontend/synthetic_stereo.hpp"

namespace
{

  using phad::frontend::FrameTracks;
  using phad::frontend::LandmarkId;
  using phad::frontend::StereoStatus;
  using phad::frontend::StereoTracker;
  using phad::frontend::StereoTrackerOptions;
  using phad::testing::kStereoEpochNs;
  using phad::testing::kStereoStepNs;
  using phad::testing::makePointGrid;
  using phad::testing::makeRectifiedCalibration;
  using phad::testing::renderStereo;

  StereoTrackerOptions testOptions( int max_tracks )
  {
    StereoTrackerOptions options;
    options.max_tracks          = max_tracks;
    options.quality_level       = 0.01;
    options.min_distance_px     = 15.0;
    options.mask_radius_px      = 12;
    options.lk_window_px        = 21;
    options.lk_pyramid_levels   = 2;
    options.forward_backward_px = 1.0;
    return options;
  }

  [[nodiscard]] std::set<LandmarkId> idsOf( const FrameTracks& tracks )
  {
    std::set<LandmarkId> ids;
    for ( const auto& observation : tracks.observations )
    {
      ids.insert( observation.id );
    }
    return ids;
  }

  TEST( StereoTrackerTest, SurvivesPureTranslationWithStableIds )
  {
    const auto calibration = makeRectifiedCalibration();
    const std::vector<Eigen::Vector3d> points =
        makePointGrid( calibration, 3, 4 );
    const int max_tracks = static_cast<int>( points.size() );

    StereoTracker tracker( calibration, testOptions( max_tracks ) );
    constexpr int kFrames = 5;
    std::set<LandmarkId> first_ids;

    for ( int frame = 0; frame < kFrames; ++frame )
    {
      std::vector<Eigen::Vector3d> moved;
      moved.reserve( points.size() );
      const double shift_m = 0.01 * static_cast<double>( frame );
      for ( const Eigen::Vector3d& point : points )
      {
        moved.emplace_back( point.x() - shift_m, point.y(), point.z() );
      }

      const auto stereo = renderStereo(
          calibration, moved,
          phad::common::Timestamp{
              kStereoEpochNs +
              static_cast<std::int64_t>( frame ) * kStereoStepNs },
          2.5 );
      const FrameTracks tracks = tracker.process( stereo );

      ASSERT_FALSE( tracks.observations.empty() );
      for ( const auto& observation : tracks.observations )
      {
        EXPECT_EQ( observation.status, StereoStatus::kNoRightMatch );
        EXPECT_DOUBLE_EQ( observation.disparity_px, 0.0 );
      }

      if ( frame == 0 )
      {
        first_ids = idsOf( tracks );
        ASSERT_GE( first_ids.size(), points.size() / 2U );
      }
      else
      {
        const std::set<LandmarkId> current = idsOf( tracks );
        std::vector<LandmarkId>    overlap;
        std::set_intersection( first_ids.begin(), first_ids.end(),
                               current.begin(), current.end(),
                               std::back_inserter( overlap ) );
        ASSERT_FALSE( overlap.empty() );
        for ( const auto& observation : tracks.observations )
        {
          if ( first_ids.count( observation.id ) != 0U )
          {
            EXPECT_EQ( observation.length,
                       static_cast<std::uint32_t>( frame + 1 ) );
          }
        }
      }
    }
  }

  TEST( StereoTrackerTest, DoesNotReuseIdsAfterTracksLeaveFov )
  {
    const auto calibration = makeRectifiedCalibration();
    std::vector<Eigen::Vector3d> points = makePointGrid( calibration, 2, 3 );
    StereoTracker tracker( calibration, testOptions( 12 ) );

    const auto first = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const FrameTracks first_tracks = tracker.process( first );
    ASSERT_FALSE( first_tracks.observations.empty() );
    LandmarkId max_id = 0;
    for ( const auto& observation : first_tracks.observations )
    {
      max_id = std::max( max_id, observation.id );
    }

    // Sweep points far to the left so they leave the image.
    for ( Eigen::Vector3d& point : points )
    {
      point.x() -= 2.5;
    }
    const auto empty_frame = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );
    (void)tracker.process( empty_frame );

    // Bring a fresh grid back into view.
    const std::vector<Eigen::Vector3d> refill =
        makePointGrid( calibration, 2, 3 );
    const auto refill_frame = renderStereo(
        calibration, refill,
        phad::common::Timestamp{ kStereoEpochNs + 2 * kStereoStepNs }, 2.5 );
    const FrameTracks refilled = tracker.process( refill_frame );
    ASSERT_FALSE( refilled.observations.empty() );
    for ( const auto& observation : refilled.observations )
    {
      EXPECT_GT( observation.id, max_id );
    }
  }

  TEST( StereoTrackerTest, RefillsTowardMaxTracksAfterMassExodus )
  {
    const auto calibration = makeRectifiedCalibration();
    const std::vector<Eigen::Vector3d> points =
        makePointGrid( calibration, 4, 5 );
    constexpr int kMaxTracks = 20;
    StereoTracker tracker( calibration, testOptions( kMaxTracks ) );

    const auto first = renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs }, 2.5 );
    const FrameTracks seeded = tracker.process( first );
    ASSERT_GE( static_cast<int>( seeded.observations.size() ), kMaxTracks / 2 );

    std::vector<Eigen::Vector3d> gone = points;
    for ( Eigen::Vector3d& point : gone )
    {
      point.x() -= 3.0;
    }
    (void)tracker.process( renderStereo(
        calibration, gone,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 ) );

    const FrameTracks refilled = tracker.process( renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + 2 * kStereoStepNs }, 2.5 ) );
    EXPECT_GE( static_cast<int>( refilled.observations.size() ),
               ( kMaxTracks * 3 ) / 4 );
    EXPECT_LE( static_cast<int>( refilled.observations.size() ), kMaxTracks );
    EXPECT_GT( refilled.stats.detected, 0U );
  }

}  // namespace
