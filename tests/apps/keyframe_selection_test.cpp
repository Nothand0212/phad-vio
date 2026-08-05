#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_map>

#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"
#include "phad/frontend/stereo_tracks.hpp"

namespace
{
using phad::common::LandmarkId;
using phad::common::Timestamp;
using phad::frontend::FrameTracks;
using phad::frontend::TrackObservation;

// Duplicated from apps/offline_vo_session.cpp (Slice ⑤)
// — kept here to keep test expectations independent of implementation
// changes inside the session.  This is a regression contract, not a
// full unit-test of every code path.
struct KeyframeSelectorState
{
  std::unordered_map<LandmarkId, Eigen::Vector2d> last_kf_pixels;
  Timestamp     last_kf_timestamp{ 0 };
  std::uint32_t total_keyframes = 0;
};

[[nodiscard]] bool isKeyframeTest(
    const FrameTracks&    tracks,
    const Timestamp       current_ts,
    KeyframeSelectorState& state )
{
  if ( state.total_keyframes < 2 ) return true;

  const std::int64_t dt_ns =
      current_ts.nanoseconds() - state.last_kf_timestamp.nanoseconds();
  if ( dt_ns > 500'000'000 ) return true;

  std::size_t common_count = 0;
  for ( const auto& obs : tracks.observations )
  {
    if ( state.last_kf_pixels.count( obs.id ) != 0U ) ++common_count;
  }
  const double survive_ratio =
      static_cast<double>( common_count ) /
      std::max( state.last_kf_pixels.size(), std::size_t{ 1 } );
  if ( survive_ratio < 0.6 ) return true;

  double      parallax_sum   = 0.0;
  std::size_t parallax_count = 0;
  for ( const auto& obs : tracks.observations )
  {
    auto it = state.last_kf_pixels.find( obs.id );
    if ( it == state.last_kf_pixels.end() ) continue;
    const double dx = obs.left_pixel.x() - it->second.x();
    const double dy = obs.left_pixel.y() - it->second.y();
    parallax_sum += std::sqrt( dx * dx + dy * dy );
    ++parallax_count;
  }
  if ( parallax_count > 0 &&
       ( parallax_sum / static_cast<double>( parallax_count ) ) > 30.0 )
    return true;

  return false;
}

void updateKeyframeSnapshotTest(
    const FrameTracks&    tracks,
    const Timestamp       ts,
    KeyframeSelectorState& state )
{
  state.last_kf_pixels.clear();
  for ( const auto& obs : tracks.observations )
    state.last_kf_pixels[ obs.id ] = obs.left_pixel;
  state.last_kf_timestamp = ts;
  ++state.total_keyframes;
}

// Helper: build a FrameTracks with N observations at a given timestamp.
FrameTracks makeTracks( std::int64_t                 ts_ns,
                        int                          n,
                        const Eigen::Vector2d&       left_px,
                        std::uint64_t                start_id = 0 )
{
  FrameTracks t;
  t.timestamp = Timestamp{ ts_ns };
  for ( int i = 0; i < n; ++i )
  {
    TrackObservation obs;
    obs.id         = LandmarkId{ start_id + static_cast<std::uint64_t>( i ) };
    obs.left_pixel = left_px;
    obs.disparity_px = 1.0;
    t.observations.push_back( obs );
  }
  return t;
}

}  // namespace

TEST( KeyframeSelectionTest, FirstTwoFramesAlwaysKeyframes )
{
  KeyframeSelectorState state;
  // total_keyframes = 0
  auto tracks = makeTracks( 100'000'000, 10, { 0.0, 0.0 } );
  EXPECT_TRUE( isKeyframeTest( tracks, Timestamp{ 100'000'000 }, state ) );
  updateKeyframeSnapshotTest( tracks, Timestamp{ 100'000'000 }, state );
  EXPECT_EQ( state.total_keyframes, 1U );

  // total_keyframes = 1 (still < 2)
  auto t2 = makeTracks( 200'000'000, 10, { 1.0, 0.0 }, 10 );
  EXPECT_TRUE( isKeyframeTest( t2, Timestamp{ 200'000'000 }, state ) );
  updateKeyframeSnapshotTest( t2, Timestamp{ 200'000'000 }, state );
  EXPECT_EQ( state.total_keyframes, 2U );
}

TEST( KeyframeSelectionTest, ZeroParallaxNotKeyframe )
{
  KeyframeSelectorState state;
  // Seed 2 keyframes with 10 tracks each.
  auto kf0 = makeTracks( 100'000'000, 10, { 100.0, 100.0 }, 0 );
  EXPECT_TRUE( isKeyframeTest( kf0, Timestamp{ 100'000'000 }, state ) );
  updateKeyframeSnapshotTest( kf0, Timestamp{ 100'000'000 }, state );

  auto kf1 = makeTracks( 200'000'000, 10, { 150.0, 100.0 }, 0 );
  EXPECT_TRUE( isKeyframeTest( kf1, Timestamp{ 200'000'000 }, state ) );
  updateKeyframeSnapshotTest( kf1, Timestamp{ 200'000'000 }, state );
  // Now total_keyframes = 2.

  // Third frame: same positions as last KF, within time window.
  // parallax = 0, survive = 1.0, dt < 0.5s → not keyframe.
  auto t = makeTracks( 300'000'000, 10, { 150.0, 100.0 }, 0 );
  EXPECT_FALSE( isKeyframeTest( t, Timestamp{ 300'000'000 }, state ) );
}

TEST( KeyframeSelectionTest, HighParallaxIsKeyframe )
{
  KeyframeSelectorState state;
  auto kf0 = makeTracks( 100'000'000, 10, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf0, Timestamp{ 100'000'000 }, state );
  updateKeyframeSnapshotTest( kf0, Timestamp{ 100'000'000 }, state );

  auto kf1 = makeTracks( 200'000'000, 10, { 150.0, 100.0 }, 0 );
  isKeyframeTest( kf1, Timestamp{ 200'000'000 }, state );
  updateKeyframeSnapshotTest( kf1, Timestamp{ 200'000'000 }, state );
  // total_keyframes = 2.

  // Same IDs but moved 50 px right → avg parallax 50 > 30.
  auto t = makeTracks( 300'000'000, 10, { 200.0, 100.0 }, 0 );
  EXPECT_TRUE( isKeyframeTest( t, Timestamp{ 300'000'000 }, state ) );
}

TEST( KeyframeSelectionTest, LowSurvivalIsKeyframe )
{
  KeyframeSelectorState state;
  // 2 keyframes with 20 tracks each.
  auto kf = makeTracks( 100'000'000, 20, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf, Timestamp{ 100'000'000 }, state );
  updateKeyframeSnapshotTest( kf, Timestamp{ 100'000'000 }, state );

  auto kf2 = makeTracks( 200'000'000, 20, { 105.0, 100.0 }, 0 );
  isKeyframeTest( kf2, Timestamp{ 200'000'000 }, state );
  updateKeyframeSnapshotTest( kf2, Timestamp{ 200'000'000 }, state );

  // Only 5 of 20 IDs survive: survive_ratio = 5/20 = 0.25 < 0.6.
  // Parallax small so rule 4 won't trigger.
  auto t = makeTracks( 300'000'000, 5, { 105.0, 100.0 }, 0 );
  EXPECT_TRUE( isKeyframeTest( t, Timestamp{ 300'000'000 }, state ) );
}

TEST( KeyframeSelectionTest, TimeFallbackIsKeyframe )
{
  KeyframeSelectorState state;
  auto kf0 = makeTracks( 100'000'000, 10, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf0, Timestamp{ 100'000'000 }, state );
  updateKeyframeSnapshotTest( kf0, Timestamp{ 100'000'000 }, state );

  auto kf1 = makeTracks( 200'000'000, 10, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf1, Timestamp{ 200'000'000 }, state );
  updateKeyframeSnapshotTest( kf1, Timestamp{ 200'000'000 }, state );

  // Same IDs, small parallax, survive=1.0 — but 1 second later (> 0.5s).
  auto t = makeTracks( 1'200'000'000LL, 10, { 101.0, 100.0 }, 0 );
  EXPECT_TRUE( isKeyframeTest( t, Timestamp{ 1'200'000'000LL }, state ) );
}

TEST( KeyframeSelectionTest, EmptyTracksBoundary )
{
  KeyframeSelectorState state;
  // Seed 2 keyframes.
  auto kf = makeTracks( 100'000'000, 5, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf, Timestamp{ 100'000'000 }, state );
  updateKeyframeSnapshotTest( kf, Timestamp{ 100'000'000 }, state );

  auto kf2 = makeTracks( 200'000'000, 5, { 105.0, 100.0 }, 0 );
  isKeyframeTest( kf2, Timestamp{ 200'000'000 }, state );
  updateKeyframeSnapshotTest( kf2, Timestamp{ 200'000'000 }, state );

  // Empty tracks, within time window → survive=0/5=0 < 0.6 → keyframe.
  FrameTracks empty;
  empty.timestamp = Timestamp{ 300'000'000 };
  EXPECT_TRUE( isKeyframeTest( empty, Timestamp{ 300'000'000 }, state ) );
}

TEST( KeyframeSelectionTest, AllTracksNew )
{
  KeyframeSelectorState state;
  auto kf0 = makeTracks( 100'000'000, 10, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf0, Timestamp{ 100'000'000 }, state );
  updateKeyframeSnapshotTest( kf0, Timestamp{ 100'000'000 }, state );

  auto kf1 = makeTracks( 200'000'000, 10, { 105.0, 100.0 }, 0 );
  isKeyframeTest( kf1, Timestamp{ 200'000'000 }, state );
  updateKeyframeSnapshotTest( kf1, Timestamp{ 200'000'000 }, state );

  // All new IDs (starting from 100, none match 0-9).
  // common_count=0, survive=0/10=0 < 0.6 → keyframe.
  auto t = makeTracks( 300'000'000, 10, { 105.0, 100.0 }, 100 );
  EXPECT_TRUE( isKeyframeTest( t, Timestamp{ 300'000'000 }, state ) );
}
