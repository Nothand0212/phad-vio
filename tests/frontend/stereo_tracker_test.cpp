#include "phad/frontend/stereo_tracker.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
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
  using phad::testing::StereoRenderOptions;
  using phad::testing::kStereoEpochNs;
  using phad::testing::kStereoStepNs;
  using phad::testing::makePointGrid;
  using phad::testing::makeRectifiedCalibration;
  using phad::testing::projectLeft;
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
    options.max_epipolar_px     = 1.5;
    options.min_disparity_px    = 0.5;
    options.min_depth_m         = 0.3;
    options.max_depth_m         = 40.0;
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

  [[nodiscard]] std::optional<phad::frontend::TrackObservation> nearestTrack(
      const FrameTracks& tracks, const Eigen::Vector2d& pixel,
      double max_dist_px = 4.0 )
  {
    std::optional<phad::frontend::TrackObservation> best;
    double best_dist = max_dist_px;
    for ( const auto& observation : tracks.observations )
    {
      const double dist = ( observation.left_pixel - pixel ).norm();
      if ( dist < best_dist )
      {
        best_dist = dist;
        best      = observation;
      }
    }
    return best;
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
      std::size_t valid_count = 0;
      for ( const auto& observation : tracks.observations )
      {
        if ( observation.status == StereoStatus::kValid )
        {
          ++valid_count;
          EXPECT_GT( observation.disparity_px, 0.0 );
        }
      }
      EXPECT_GE( valid_count, points.size() / 2U );

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

    for ( Eigen::Vector3d& point : points )
    {
      point.x() -= 2.5;
    }
    const auto empty_frame = renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, 2.5 );
    (void)tracker.process( empty_frame );

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

  TEST( StereoTrackerTest, ValidDisparityMatchesGeometry )
  {
    const auto calibration = makeRectifiedCalibration();
    constexpr double kDepth = 3.0;
    const std::vector<Eigen::Vector3d> points =
        makePointGrid( calibration, 3, 4, kDepth );
    StereoTracker tracker( calibration,
                           testOptions( static_cast<int>( points.size() ) ) );

    const FrameTracks tracks = tracker.process( renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs },
        2.5 ) );

    const double expected_disp =
        calibration.fxPixels() * calibration.baselineM() / kDepth;
    std::size_t valid_count = 0;
    for ( const auto& observation : tracks.observations )
    {
      if ( observation.status != StereoStatus::kValid )
      {
        continue;
      }
      ++valid_count;
      EXPECT_NEAR( observation.disparity_px, expected_disp, 0.75 );
    }
    ASSERT_GE( valid_count, points.size() / 2U );
    EXPECT_LT( tracks.stats.epipolar_median_px, 1.0 );
  }

  TEST( StereoTrackerTest, MissingRightBlobBecomesNoRightMatch )
  {
    const auto calibration = makeRectifiedCalibration();
    std::vector<Eigen::Vector3d> points = {
        { -0.3, -0.2, 2.5 },
        { 0.3, -0.2, 2.5 },
        { -0.3, 0.2, 2.5 },
        { 0.3, 0.2, 2.5 },
    };
    StereoTrackerOptions options = testOptions( 8 );
    options.min_distance_px      = 8.0;
    options.mask_radius_px       = 8;
    StereoTracker tracker( calibration, options );

    // Seed tracks with a complete stereo pair.
    (void)tracker.process( renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs },
        2.5 ) );

    StereoRenderOptions render;
    render.sigma_px = 2.5;
    render.paint_right.assign( points.size(), true );
    render.paint_right[ 0 ] = false;

    const FrameTracks tracks = tracker.process( renderStereo(
        calibration, points,
        phad::common::Timestamp{ kStereoEpochNs + kStereoStepNs }, render ) );

    const Eigen::Vector2d target = projectLeft( calibration, points[ 0 ] );
    const auto            hit    = nearestTrack( tracks, target, 6.0 );
    ASSERT_TRUE( hit.has_value() );
    EXPECT_EQ( hit->status, StereoStatus::kNoRightMatch );
    EXPECT_DOUBLE_EQ( hit->disparity_px, 0.0 );
    // Track is retained even without a right match.
    EXPECT_GE( hit->length, 1U );
  }

  TEST( StereoTrackerTest, FarPointBecomesDepthOutOfRange )
  {
    const auto calibration = makeRectifiedCalibration();
    StereoTrackerOptions options = testOptions( 4 );
    options.max_depth_m          = 5.0;
    options.min_distance_px      = 8.0;
    options.mask_radius_px       = 8;
    StereoTracker tracker( calibration, options );

    const std::vector<Eigen::Vector3d> points{ { 0.0, 0.0, 12.0 } };
    const FrameTracks tracks = tracker.process( renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs },
        3.0 ) );

    ASSERT_FALSE( tracks.observations.empty() );
    bool saw_depth = false;
    for ( const auto& observation : tracks.observations )
    {
      if ( observation.status == StereoStatus::kDepthOutOfRange )
      {
        saw_depth = true;
        EXPECT_DOUBLE_EQ( observation.disparity_px, 0.0 );
      }
    }
    EXPECT_TRUE( saw_depth );
    EXPECT_GE( tracks.stats.depth_rejected, 1U );
  }

  TEST( StereoTrackerTest, RejectsNonPositiveStereoBidir )
  {
    auto options = testOptions( 4 );
    options.stereo_bidir_px = 0.0;
    EXPECT_THROW(
        ( StereoTracker{ makeRectifiedCalibration(), options } ),
        std::invalid_argument );
  }

  TEST( StereoTrackerTest, RejectsNegativeRowTol )
  {
    auto options = testOptions( 4 );
    options.stereo_row_tol_px = -1;
    EXPECT_THROW(
        ( StereoTracker{ makeRectifiedCalibration(), options } ),
        std::invalid_argument );
  }

  TEST( StereoTrackerTest, RejectsNonPositiveSadHalfWin )
  {
    auto options = testOptions( 4 );
    options.stereo_sad_half_win_px = 0;
    EXPECT_THROW(
        ( StereoTracker{ makeRectifiedCalibration(), options } ),
        std::invalid_argument );
  }

  TEST( StereoTrackerTest, VerticalRightOffsetBecomesInvalidDisparity )
  {
    const auto calibration = makeRectifiedCalibration();
    const std::vector<Eigen::Vector3d> points =
        makePointGrid( calibration, 2, 3, 3.0 );
    StereoTrackerOptions options = testOptions( 12 );
    options.max_epipolar_px      = 1.0;
    StereoTracker tracker( calibration, options );

    StereoRenderOptions render;
    render.sigma_px          = 2.5;
    render.right_v_offset_px = 4.0;

    const FrameTracks tracks = tracker.process( renderStereo(
        calibration, points, phad::common::Timestamp{ kStereoEpochNs },
        render ) );

    ASSERT_FALSE( tracks.observations.empty() );
    std::size_t invalid = 0;
    for ( const auto& observation : tracks.observations )
    {
      if ( observation.status == StereoStatus::kInvalidDisparity )
      {
        ++invalid;
        EXPECT_DOUBLE_EQ( observation.disparity_px, 0.0 );
      }
    }
    EXPECT_GE( invalid, 1U );
    EXPECT_GE( tracks.stats.epipolar_rejected, 1U );
  }

}  // namespace
