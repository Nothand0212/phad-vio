#include <gtest/gtest.h>

#include <cmath>
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
  // Rotation compensation (Slice ⑤b).
  Eigen::Matrix3d last_accepted_rotation = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d last_kf_rotation       = Eigen::Matrix3d::Identity();
};

[[nodiscard]] bool isKeyframeTest(
    const FrameTracks&    tracks,
    const Timestamp       current_ts,
    KeyframeSelectorState& state )
{
  // Rule 0: empty observations never become keyframes (Slice ⑤b).
  if ( tracks.observations.empty() ) return false;

  if ( state.total_keyframes < 2 ) return true;

  // Rule 1b: too few tracks to run PnP -> force keyframe.
  if ( tracks.observations.size() < 10U ) return true;

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

  // Rule 4: rotation-compensated parallax (Slice ⑤b).
  // Normalized-coordinate projection: pixel -> normalized ray, rotate,
  // back to pixel.
  const Eigen::Matrix3d R_kf_to_cur =
      state.last_accepted_rotation.transpose() * state.last_kf_rotation;
  const double fx = 400.0;  // makeCalibration()
  const double fy = 400.0;
  const double cx = 320.0;
  const double cy = 240.0;
  double      parallax_sum   = 0.0;
  std::size_t parallax_count = 0;
  for ( const auto& obs : tracks.observations )
  {
    auto it = state.last_kf_pixels.find( obs.id );
    if ( it == state.last_kf_pixels.end() ) continue;
    const Eigen::Vector3d ray_kf(
        ( it->second.x() - cx ) / fx, ( it->second.y() - cy ) / fy, 1.0 );
    const Eigen::Vector3d ray_cur = R_kf_to_cur * ray_kf;
    if ( ray_cur.z() <= 1e-6 )
    {
      continue;
    }
    const Eigen::Vector2d p_comp(
        ray_cur.x() / ray_cur.z() * fx + cx,
        ray_cur.y() / ray_cur.z() * fy + cy );
    const double dx = obs.left_pixel.x() - p_comp.x();
    const double dy = obs.left_pixel.y() - p_comp.y();
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
  state.last_kf_rotation  = state.last_accepted_rotation;
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

  // Empty tracks are NOT keyframes (Slice ⑤b Rule 0; previously they
  // cascaded as spurious keyframes: survive=0/1=0 < 0.6).
  FrameTracks empty;
  empty.timestamp = Timestamp{ 300'000'000 };
  EXPECT_FALSE( isKeyframeTest( empty, Timestamp{ 300'000'000 }, state ) );
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

TEST( KeyframeSelectionTest, LowTrackForcesKeyframe )
{
  KeyframeSelectorState state;
  auto kf0 = makeTracks( 100'000'000, 10, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf0, Timestamp{ 100'000'000 }, state );
  updateKeyframeSnapshotTest( kf0, Timestamp{ 100'000'000 }, state );

  auto kf1 = makeTracks( 200'000'000, 10, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf1, Timestamp{ 200'000'000 }, state );
  updateKeyframeSnapshotTest( kf1, Timestamp{ 200'000'000 }, state );

  // Only 5 tracks (>=1 but < 10 = min_pnp_inliers): forced keyframe even
  // though parallax=0, survival=1.0, dt small.
  auto t = makeTracks( 300'000'000, 5, { 100.0, 100.0 }, 0 );
  EXPECT_TRUE( isKeyframeTest( t, Timestamp{ 300'000'000 }, state ) );
}

TEST( KeyframeSelectionTest, EmptyTracksNotSpuriousKeyframe )
{
  KeyframeSelectorState state;
  auto kf0 = makeTracks( 100'000'000, 10, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf0, Timestamp{ 100'000'000 }, state );
  updateKeyframeSnapshotTest( kf0, Timestamp{ 100'000'000 }, state );

  auto kf1 = makeTracks( 200'000'000, 10, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf1, Timestamp{ 200'000'000 }, state );
  updateKeyframeSnapshotTest( kf1, Timestamp{ 200'000'000 }, state );

  // Empty observations: NOT a keyframe (Slice ⑤b Rule 0). Previously this
  // was a spurious keyframe (survive=0/1=0 < 0.6) that cascaded.
  FrameTracks empty;
  empty.timestamp = Timestamp{ 300'000'000 };
  EXPECT_FALSE( isKeyframeTest( empty, Timestamp{ 300'000'000 }, state ) );
}

TEST( KeyframeSelectionTest, RotationCompensatedParallax )
{
  // Pure rotation between last-KF and current frame: raw parallax would
  // exceed 30 px, but rotation compensation brings it near zero -> NOT a
  // keyframe (VINS compensatedParallax2 idea).
  KeyframeSelectorState state;
  auto kf0 = makeTracks( 100'000'000, 10, { 100.0, 100.0 }, 0 );
  isKeyframeTest( kf0, Timestamp{ 100'000'000 }, state );
  updateKeyframeSnapshotTest( kf0, Timestamp{ 100'000'000 }, state );

  auto kf1 = makeTracks( 200'000'000, 10, { 105.0, 100.0 }, 0 );
  isKeyframeTest( kf1, Timestamp{ 200'000'000 }, state );
  updateKeyframeSnapshotTest( kf1, Timestamp{ 200'000'000 }, state );
  // total_keyframes = 2.

  // Simulate a rotation: last-accepted rotation is 30° about Z, and the
  // current observations are exactly the rotated last-KF pixels.
  const double angle = 30.0 * M_PI / 180.0;
  const double c     = std::cos( angle );
  const double s     = std::sin( angle );
  Eigen::Matrix3d R_cur;
  R_cur << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  state.last_accepted_rotation = R_cur;
  state.last_kf_rotation       = Eigen::Matrix3d::Identity();

  // Rotate the last-KF rays into the current camera frame:
  // ray_cur = R_cur^T * R_kf * ray_kf (R_kf = Identity here), then
  // project back to pixels with the same normalized-coordinate pipeline.
  std::vector<Eigen::Vector2d> rotated;
  for ( const auto& [ id, px ] : state.last_kf_pixels )
  {
    const Eigen::Vector3d ray_kf(
        ( px.x() - 320.0 ) / 400.0, ( px.y() - 240.0 ) / 400.0, 1.0 );
    const Eigen::Vector3d ray_cur = R_cur.transpose() * ray_kf;
    rotated.emplace_back( ray_cur.x() / ray_cur.z() * 400.0 + 320.0,
                          ray_cur.y() / ray_cur.z() * 400.0 + 240.0 );
  }

  FrameTracks t;
  t.timestamp = Timestamp{ 300'000'000 };
  int i       = 0;
  for ( const auto& [ id, px ] : state.last_kf_pixels )
  {
    TrackObservation obs;
    obs.id           = id;
    obs.left_pixel   = rotated[ i++ ];
    obs.disparity_px = 1.0;
    t.observations.push_back( obs );
  }
  // 10 tracks (>= min_pnp_inliers), all survive, dt small. Raw parallax
  // (before compensation) would be large; compensated should be ~0.
  EXPECT_FALSE( isKeyframeTest( t, Timestamp{ 300'000'000 }, state ) );
}
