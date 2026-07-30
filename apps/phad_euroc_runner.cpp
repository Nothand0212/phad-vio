#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "phad/common/trajectory.hpp"
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

  constexpr int    kMaxCorners   = 500;
  constexpr double kQualityLevel = 0.01;
  constexpr double kMinDistance  = 20.0;
  constexpr int    kCornerRadius = 3;
  const cv::Scalar kCornerColor{ 0, 255, 0 };  // BGR
  const cv::Scalar kTextColor{ 0, 255, 0 };    // BGR

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

  [[nodiscard]] std::vector<cv::Point2f> detectCorners( const cv::Mat& gray )
  {
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack( gray, corners, kMaxCorners, kQualityLevel,
                             kMinDistance );
    return corners;
  }

  void drawCorners( cv::Mat& canvas, const std::vector<cv::Point2f>& corners,
                    int x_offset )
  {
    for ( const cv::Point2f& corner : corners )
    {
      const cv::Point center{ cvRound( corner.x ) + x_offset,
                              cvRound( corner.y ) };
      cv::circle( canvas, center, kCornerRadius, kCornerColor, 1,
                  cv::LINE_AA );
    }
  }

  [[nodiscard]] cv::Mat renderStereo(
      const phad::sensor::StereoFrame& frame )
  {
    if ( frame.left.width() != frame.right.width() ||
         frame.left.height() != frame.right.height() ||
         frame.left.channels() != frame.right.channels() ||
         frame.left.pixelType() != frame.right.pixelType() )
    {
      throw std::runtime_error(
          "left and right images have incompatible shapes or pixel types" );
    }

    const cv::Mat left  = copyGrayImage( frame.left );
    const cv::Mat right = copyGrayImage( frame.right );

    cv::Mat stitched;
    cv::hconcat( left, right, stitched );
    cv::Mat canvas;
    cv::cvtColor( stitched, canvas, cv::COLOR_GRAY2BGR );

    const std::vector<cv::Point2f> left_corners  = detectCorners( left );
    const std::vector<cv::Point2f> right_corners = detectCorners( right );
    drawCorners( canvas, left_corners, 0 );
    drawCorners( canvas, right_corners, left.cols );

    const std::string count_text =
        "L:" + std::to_string( left_corners.size() ) +
        " R:" + std::to_string( right_corners.size() );
    cv::putText( canvas, count_text, cv::Point( 10, 24 ),
                 cv::FONT_HERSHEY_SIMPLEX, 0.7, kTextColor, 1, cv::LINE_AA );
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

  /**
   * @brief 加载序列真值，用于俯视面板。
   *
   * 没有 state_groundtruth_estimate0 的序列仍然可以回放图像，因此加载失败
   * 只关闭面板并说明原因，不中断回放。
   */
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

    phad::io::dataset::DatasetReplaySource replay_source{ opened.value() };
    phad::io::SensorSource&                source = replay_source;

    const std::optional<phad::common::Trajectory> groundtruth =
        loadGroundtruth( sequence_root );
    std::optional<phad::viz::TrajectoryPanel> panel;

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

      cv::Mat canvas = renderStereo( *frame );
      if ( groundtruth.has_value() )
      {
        if ( !panel.has_value() )
        {
          // 面板与图像等高的正方形，接在双目画面右侧。
          panel.emplace( *groundtruth,
                         phad::viz::TrajectoryPanelOptions{
                             .width_px  = canvas.rows,
                             .height_px = canvas.rows } );
        }
        cv::Mat composed;
        cv::hconcat( canvas, panel->render( frame->timestamp ), composed );
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
