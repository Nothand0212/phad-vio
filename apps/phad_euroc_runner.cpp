#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

#include "phad/io/dataset/dataset_replay_source.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/io/sensor_source.hpp"
#include "phad/sensor/stereo_frame.hpp"

namespace
{

  constexpr char kWindowName[] = "phad-vio EuRoC stereo";
  constexpr int  kEscapeKey    = 27;
  constexpr int  kQKey         = 'q';

  [[nodiscard]] bool isQuitKey( int key ) noexcept
  {
    return key == kEscapeKey || key == kQKey;
  }

  [[nodiscard]] bool windowIsOpen()
  {
    return cv::getWindowProperty( kWindowName, cv::WND_PROP_VISIBLE ) >= 1.0;
  }

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

  [[nodiscard]] cv::Mat stitchStereo(
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
    cv::Mat       stitched;
    cv::hconcat( left, right, stitched );
    return stitched;
  }

  class PlaybackClock
  {
  public:
    [[nodiscard]] bool waitUntil( phad::common::Timestamp timestamp )
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
        if ( !windowIsOpen() )
        {
          return false;
        }

        const auto remaining = deadline - std::chrono::steady_clock::now();
        const auto remaining_ms =
            std::chrono::ceil<std::chrono::milliseconds>( remaining ).count();
        const auto bounded_ms =
            std::clamp<std::int64_t>( remaining_ms, 1,
                                      std::numeric_limits<int>::max() );
        if ( isQuitKey( cv::waitKey( static_cast<int>( bounded_ms ) ) ) )
        {
          return false;
        }
      }
      return windowIsOpen();
    }

  private:
    std::optional<phad::common::Timestamp> m_first_timestamp;
    std::chrono::steady_clock::time_point  m_first_wall_time;
  };

  [[nodiscard]] int run( const std::filesystem::path& sequence_root )
  {
    auto opened = phad::io::dataset::euroc::open( sequence_root );
    if ( !opened )
    {
      std::cerr << opened.error().describe() << '\n';
      return 1;
    }

    phad::io::dataset::DatasetReplaySource replay_source{ std::move( opened ).value() };
    phad::io::SensorSource&                source = replay_source;

    cv::namedWindow( kWindowName, cv::WINDOW_AUTOSIZE );
    PlaybackClock playback_clock;

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

      const cv::Mat stitched = stitchStereo( *frame );
      if ( !playback_clock.waitUntil( frame->timestamp ) )
      {
        return 0;
      }

      cv::imshow( kWindowName, stitched );
      if ( isQuitKey( cv::waitKey( 1 ) ) || !windowIsOpen() )
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
    const int result = run( std::filesystem::path{ argv[ 1 ] } );
    cv::destroyAllWindows();
    return result;
  }
  catch ( const cv::Exception& exception )
  {
    std::cerr << "OpenCV error: " << exception.what() << '\n';
  }
  catch ( const std::exception& exception )
  {
    std::cerr << "runner error: " << exception.what() << '\n';
  }
  cv::destroyAllWindows();
  return 1;
}
