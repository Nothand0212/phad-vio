#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "apps/stereo_vo_glue.hpp"
#include "phad/camera/stereo_rectifier.hpp"
#include "phad/common/trajectory.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/frontend/stereo_tracker.hpp"
#include "phad/io/dataset/dataset_replay_source.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/io/dataset/euroc/euroc_groundtruth.hpp"
#include "phad/io/sensor_source.hpp"
#include "phad/sensor/stereo_frame.hpp"
#include "phad/viz/image_window.hpp"
#include "phad/viz/trajectory_panel.hpp"

namespace
{

  constexpr char kWindowName[] = "phad-vio EuRoC stereo";

  constexpr int    kPointRadius = 2;
  const cv::Scalar kValidColor{ 0, 255, 0 };      // BGR
  const cv::Scalar kInvalidColor{ 0, 165, 255 };  // BGR orange
  const cv::Scalar kMatchLineColor{ 255, 200, 0 };
  const cv::Scalar kTextColor{ 0, 255, 0 };
  const cv::Scalar kEstimatePathColor{ 0, 220, 255 };  // BGR cyan
  const cv::Scalar kEstimateMarkerColor{ 0, 255, 255 };

  [[nodiscard]] cv::Mat copyGrayImage( const phad::sensor::Image& image )
  {
    if ( image.width() <= 0 || image.height() <= 0 ||
         image.channels() != 1 ||
         image.pixelType() != phad::sensor::PixelType::kUint8 )
    {
      throw std::runtime_error(
          "expected a non-empty, single-channel uint8 image" );
    }

    const auto pixels = image.pixels<std::uint8_t>();
    if ( !pixels.has_value() )
    {
      throw std::runtime_error( "uint8 image does not expose uint8 pixels" );
    }

    cv::Mat result( image.height(), image.width(), CV_8UC1 );
    if ( pixels->size() != result.total() )
    {
      throw std::runtime_error( "image metadata does not match pixel count" );
    }
    std::copy( pixels->begin(), pixels->end(), result.ptr<std::uint8_t>() );
    return result;
  }

  [[nodiscard]] cv::Mat renderTracks(
      const phad::sensor::StereoFrame&   frame,
      const phad::frontend::FrameTracks& tracks )
  {
    if ( frame.left.width() != frame.right.width() ||
         frame.left.height() != frame.right.height() )
    {
      throw std::runtime_error( "left and right images have incompatible sizes" );
    }

    const cv::Mat left  = copyGrayImage( frame.left );
    const cv::Mat right = copyGrayImage( frame.right );

    cv::Mat stitched;
    cv::hconcat( left, right, stitched );
    cv::Mat canvas;
    cv::cvtColor( stitched, canvas, cv::COLOR_GRAY2BGR );
    const int right_offset = left.cols;

    for ( const auto& observation : tracks.observations )
    {
      const cv::Point left_pt{ cvRound( observation.left_pixel.x() ),
                               cvRound( observation.left_pixel.y() ) };
      const bool      valid =
          observation.status == phad::frontend::StereoStatus::kValid;
      const cv::Scalar& color = valid ? kValidColor : kInvalidColor;
      cv::circle( canvas, left_pt, kPointRadius, color, -1, cv::LINE_AA );

      if ( valid )
      {
        const cv::Point right_pt{
            cvRound( observation.left_pixel.x() - observation.disparity_px ) +
                right_offset,
            cvRound( observation.left_pixel.y() ) };
        cv::circle( canvas, right_pt, kPointRadius, kValidColor, -1,
                    cv::LINE_AA );
        cv::line( canvas, left_pt, right_pt, kMatchLineColor, 1, cv::LINE_AA );
      }
    }

    const auto&       stats = tracks.stats;
    const std::string line1 =
        "tracks=" + std::to_string( tracks.observations.size() ) +
        " tracked=" + std::to_string( stats.tracked ) +
        " detected=" + std::to_string( stats.detected );
    const std::string line2 =
        "fb=" + std::to_string( stats.forward_backward_rejected ) +
        " epi=" + std::to_string( stats.epipolar_rejected ) +
        " disp=" + std::to_string( stats.disparity_rejected ) +
        " depth=" + std::to_string( stats.depth_rejected );
    std::ostringstream line3_stream;
    line3_stream << std::fixed << std::setprecision( 2 )
                 << "epi_med=" << stats.epipolar_median_px
                 << " epi_p95=" << stats.epipolar_p95_px
                 << " len_med=" << stats.track_length_median;
    const std::string line3 = line3_stream.str();

    cv::putText( canvas, line1, cv::Point( 10, 22 ),
                 cv::FONT_HERSHEY_SIMPLEX, 0.55, kTextColor, 1, cv::LINE_AA );
    cv::putText( canvas, line2, cv::Point( 10, 44 ),
                 cv::FONT_HERSHEY_SIMPLEX, 0.55, kTextColor, 1, cv::LINE_AA );
    cv::putText( canvas, line3, cv::Point( 10, 66 ),
                 cv::FONT_HERSHEY_SIMPLEX, 0.55, kTextColor, 1, cv::LINE_AA );
    return canvas;
  }

  class PlaybackClock
  {
  public:
    [[nodiscard]] bool waitUntil( phad::common::Timestamp timestamp,
                                  phad::viz::ImageWindow& window )
    {
      const auto now = std::chrono::steady_clock::now();
      if ( !m_first_timestamp.has_value() )
      {
        m_first_timestamp = timestamp;
        m_first_wall_time = now;
        return true;
      }

      const std::int64_t elapsed_nanoseconds =
          timestamp.nanoseconds() - m_first_timestamp->nanoseconds();
      if ( elapsed_nanoseconds < 0 )
      {
        throw std::runtime_error(
            "stereo timestamps are not monotonically ordered" );
      }

      const auto deadline =
          m_first_wall_time + std::chrono::nanoseconds{ elapsed_nanoseconds };
      while ( std::chrono::steady_clock::now() < deadline )
      {
        const auto remaining = deadline - std::chrono::steady_clock::now();
        const auto remaining_ms =
            std::chrono::ceil<std::chrono::milliseconds>( remaining ).count();
        const auto bounded_ms =
            std::clamp<std::int64_t>( remaining_ms, 1,
                                      std::numeric_limits<int>::max() );
        if ( !window.pump( static_cast<int>( bounded_ms ) ) )
        {
          return false;
        }
      }
      return window.isOpen();
    }

  private:
    std::optional<phad::common::Timestamp> m_first_timestamp;
    std::chrono::steady_clock::time_point  m_first_wall_time;
  };

  [[nodiscard]] std::optional<phad::common::Trajectory> loadGroundtruth(
      const std::filesystem::path& sequence_root )
  {
    auto trajectory =
        phad::io::dataset::euroc::openGroundtruth( sequence_root );
    if ( !trajectory )
    {
      std::cerr << "trajectory panel disabled: "
                << trajectory.error().describe() << '\n';
      return std::nullopt;
    }
    return std::move( trajectory ).value();
  }

  [[nodiscard]] int run( const std::filesystem::path& sequence_root )
  {
    auto opened = phad::io::dataset::euroc::open( sequence_root );
    if ( !opened )
    {
      std::cerr << opened.error().describe() << '\n';
      return 1;
    }

    auto rectifier =
        phad::camera::StereoRectifier::create( opened.value().calibration() );
    if ( !rectifier )
    {
      std::cerr << "stereo rectifier: " << rectifier.error().detail << '\n';
      return 1;
    }
    const auto&                        rectified_cal = rectifier.value().calibration();
    phad::frontend::StereoTracker      tracker( rectified_cal );
    phad::estimator::StereoVoEstimator estimator( rectified_cal );

    phad::io::dataset::DatasetReplaySource replay_source{ opened.value() };
    phad::io::SensorSource&                source = replay_source;

    const std::optional<phad::common::Trajectory> groundtruth =
        loadGroundtruth( sequence_root );
    std::optional<phad::viz::TrajectoryPanel> panel;
    std::vector<cv::Point>                    estimate_path_px;

    phad::viz::ImageWindow window{ kWindowName };
    PlaybackClock          playback_clock;

    while ( true )
    {
      phad::io::SensorReadResult result = source.next();
      if ( std::holds_alternative<phad::io::EndOfStream>( result ) )
      {
        return 0;
      }
      if ( const auto* error =
               std::get_if<phad::io::SensorSourceError>( &result ) )
      {
        std::cerr << "sensor source error: " << error->cause << '\n';
        return 1;
      }

      auto& event = std::get<phad::io::SensorEvent>( result );
      auto* frame = std::get_if<phad::sensor::StereoFrame>( &event );
      if ( frame == nullptr )
      {
        continue;
      }

      auto rectified = rectifier.value().rectify( *frame );
      if ( !rectified )
      {
        std::cerr << "rectify failed: " << rectified.error().detail << '\n';
        return 1;
      }
      const phad::frontend::FrameTracks tracks =
          tracker.process( rectified.value() );
      const phad::estimator::VioUpdateResult vo =
          estimator.update( phad::apps::toKeyframeMeasurement( tracks ) );

      cv::Mat canvas = renderTracks( rectified.value(), tracks );
      if ( groundtruth.has_value() )
      {
        if ( !panel.has_value() )
        {
          panel.emplace( *groundtruth,
                         phad::viz::TrajectoryPanelOptions{
                             .width_px  = canvas.rows,
                             .height_px = canvas.rows } );
        }
        cv::Mat panel_image = panel->render( frame->timestamp );
        if ( vo.status == phad::estimator::UpdateStatus::kOk &&
             vo.estimate.has_value() )
        {
          estimate_path_px.push_back(
              panel->project( vo.estimate->T_W_B.translation() ) );
        }
        for ( std::size_t index = 1; index < estimate_path_px.size(); ++index )
        {
          cv::line( panel_image, estimate_path_px[ index - 1 ],
                    estimate_path_px[ index ], kEstimatePathColor, 1,
                    cv::LINE_AA );
        }
        if ( !estimate_path_px.empty() )
        {
          cv::circle( panel_image, estimate_path_px.back(), 4,
                      kEstimateMarkerColor, -1, cv::LINE_AA );
        }
        cv::Mat composed;
        cv::hconcat( canvas, panel_image, composed );
        canvas = composed;
      }

      if ( !playback_clock.waitUntil( frame->timestamp, window ) )
      {
        return 0;
      }

      window.show( canvas );
      if ( !window.pump( 1 ) )
      {
        return 0;
      }
    }
  }

}  // namespace

int main( int argc, char** argv )
{
  if ( argc != 2 )
  {
    std::cerr << "usage: phad_euroc_runner <sequence-root>\n";
    return 2;
  }

  try
  {
    return run( std::filesystem::path{ argv[ 1 ] } );
  }
  catch ( const cv::Exception& exception )
  {
    std::cerr << "OpenCV error: " << exception.what() << '\n';
  }
  catch ( const std::exception& exception )
  {
    std::cerr << "runner error: " << exception.what() << '\n';
  }
  return 1;
}
