#include "phad/frontend/stereo_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

namespace phad::frontend
{
  namespace
  {

    struct LiveTrack
    {
      LandmarkId    id;
      std::uint32_t length;
      cv::Point2f   pixel;
    };

    [[nodiscard]] bool isGrayUint8( const sensor::Image& image )
    {
      return image.width() > 0 && image.height() > 0 &&
             image.channels() == 1 &&
             image.pixelType() == sensor::PixelType::kUint8;
    }

    [[nodiscard]] cv::Mat toGrayMat( const sensor::Image& image )
    {
      if ( !isGrayUint8( image ) )
      {
        throw std::runtime_error(
            "StereoTracker expects single-channel uint8 left images" );
      }
      const auto pixels = image.pixels<std::uint8_t>();
      if ( !pixels.has_value() )
      {
        throw std::runtime_error( "uint8 image does not expose uint8 pixels" );
      }
      cv::Mat mat( image.height(), image.width(), CV_8UC1 );
      if ( pixels->size() != mat.total() )
      {
        throw std::runtime_error( "image metadata does not match pixel count" );
      }
      std::copy( pixels->begin(), pixels->end(), mat.ptr<std::uint8_t>() );
      return mat;
    }

    [[nodiscard]] bool inBounds( const cv::Point2f& pixel, int width,
                                 int height )
    {
      return pixel.x >= 0.0F && pixel.y >= 0.0F &&
             pixel.x <= static_cast<float>( width - 1 ) &&
             pixel.y <= static_cast<float>( height - 1 );
    }

    [[nodiscard]] std::uint32_t medianLength(
        std::vector<std::uint32_t> lengths )
    {
      if ( lengths.empty() )
      {
        return 0U;
      }
      const auto mid = lengths.begin() +
                       static_cast<std::ptrdiff_t>( lengths.size() / 2 );
      std::nth_element( lengths.begin(), mid, lengths.end() );
      return *mid;
    }

  }  // namespace

  struct StereoTracker::Impl
  {
    camera::RectifiedStereoCalibration calibration;
    StereoTrackerOptions               options;
    cv::Mat                            prev_left;
    std::vector<LiveTrack>             tracks;
    LandmarkId                         next_id = 1;
    bool                               has_prev = false;

    explicit Impl( camera::RectifiedStereoCalibration calib,
                   StereoTrackerOptions               opts )
        : calibration( std::move( calib ) ), options( opts )
    {
      if ( options.max_tracks <= 0 )
      {
        throw std::invalid_argument( "max_tracks must be positive" );
      }
      if ( options.lk_window_px <= 0 ||
           ( options.lk_window_px % 2 ) == 0 )
      {
        throw std::invalid_argument( "lk_window_px must be a positive odd int" );
      }
      if ( options.lk_pyramid_levels < 0 )
      {
        throw std::invalid_argument( "lk_pyramid_levels must be non-negative" );
      }
      if ( options.mask_radius_px <= 0 )
      {
        throw std::invalid_argument( "mask_radius_px must be positive" );
      }
      if ( !( options.forward_backward_px > 0.0 ) )
      {
        throw std::invalid_argument( "forward_backward_px must be positive" );
      }
    }

    void detectNewTracks( const cv::Mat& left, std::uint32_t& detected )
    {
      const int deficit =
          options.max_tracks - static_cast<int>( tracks.size() );
      if ( deficit <= 0 )
      {
        return;
      }

      cv::Mat mask = cv::Mat::ones( left.size(), CV_8UC1 ) * 255;
      std::vector<LiveTrack> ordered = tracks;
      std::sort( ordered.begin(), ordered.end(),
                 []( const LiveTrack& lhs, const LiveTrack& rhs )
                 {
                   return lhs.length > rhs.length;
                 } );
      for ( const LiveTrack& track : ordered )
      {
        cv::circle( mask, cv::Point( cvRound( track.pixel.x ),
                                     cvRound( track.pixel.y ) ),
                    options.mask_radius_px, cv::Scalar( 0 ), cv::FILLED );
      }

      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack( left, corners, deficit, options.quality_level,
                               options.min_distance_px, mask );
      detected = static_cast<std::uint32_t>( corners.size() );
      tracks.reserve( tracks.size() + corners.size() );
      for ( const cv::Point2f& corner : corners )
      {
        tracks.push_back( LiveTrack{ next_id++, 1U, corner } );
      }
    }

    [[nodiscard]] FrameTracks makeOutput( common::Timestamp timestamp ) const
    {
      FrameTracks output;
      output.timestamp = timestamp;
      output.observations.reserve( tracks.size() );

      std::vector<std::uint32_t> lengths;
      lengths.reserve( tracks.size() );
      std::uint32_t length_max = 0U;
      for ( const LiveTrack& track : tracks )
      {
        output.observations.push_back( TrackObservation{
            track.id,
            Eigen::Vector2d{ track.pixel.x, track.pixel.y },
            0.0,
            StereoStatus::kNoRightMatch,
            track.length } );
        lengths.push_back( track.length );
        length_max = std::max( length_max, track.length );
      }

      output.stats.tracked                   = 0U;
      output.stats.detected                  = 0U;
      output.stats.forward_backward_rejected = 0U;
      output.stats.epipolar_rejected         = 0U;
      output.stats.disparity_rejected        = 0U;
      output.stats.depth_rejected            = 0U;
      output.stats.epipolar_median_px        = 0.0;
      output.stats.epipolar_p95_px           = 0.0;
      output.stats.track_length_median = medianLength( std::move( lengths ) );
      output.stats.track_length_max    = length_max;
      return output;
    }
  };

  StereoTracker::StereoTracker( camera::RectifiedStereoCalibration calibration,
                                StereoTrackerOptions               options )
      : m_impl( std::make_unique<Impl>( std::move( calibration ), options ) )
  {
  }

  StereoTracker::~StereoTracker() = default;

  StereoTracker::StereoTracker( StereoTracker&& ) noexcept = default;

  StereoTracker& StereoTracker::operator=( StereoTracker&& ) noexcept =
      default;

  FrameTracks StereoTracker::process( const sensor::StereoFrame& rectified )
  {
    if ( rectified.left.width() != m_impl->calibration.imageWidth() ||
         rectified.left.height() != m_impl->calibration.imageHeight() )
    {
      throw std::invalid_argument(
          "rectified left image size does not match tracker calibration" );
    }

    const cv::Mat left = toGrayMat( rectified.left );
    FrameStats    stats{};
    stats.epipolar_median_px = 0.0;
    stats.epipolar_p95_px    = 0.0;

    if ( !m_impl->has_prev || m_impl->tracks.empty() )
    {
      m_impl->tracks.clear();
      m_impl->detectNewTracks( left, stats.detected );
      stats.tracked = 0U;
    }
    else
    {
      std::vector<cv::Point2f> prev_pts;
      prev_pts.reserve( m_impl->tracks.size() );
      for ( const LiveTrack& track : m_impl->tracks )
      {
        prev_pts.push_back( track.pixel );
      }

      std::vector<cv::Point2f> curr_pts;
      std::vector<std::uint8_t> forward_status;
      std::vector<float>        forward_error;
      const cv::Size win( m_impl->options.lk_window_px,
                          m_impl->options.lk_window_px );
      cv::calcOpticalFlowPyrLK(
          m_impl->prev_left, left, prev_pts, curr_pts, forward_status,
          forward_error, win, m_impl->options.lk_pyramid_levels );

      std::vector<cv::Point2f> back_pts;
      std::vector<std::uint8_t> backward_status;
      std::vector<float>        backward_error;
      cv::calcOpticalFlowPyrLK(
          left, m_impl->prev_left, curr_pts, back_pts, backward_status,
          backward_error, win, m_impl->options.lk_pyramid_levels );

      std::vector<LiveTrack> survivors;
      survivors.reserve( m_impl->tracks.size() );
      const double fb_limit_sq =
          m_impl->options.forward_backward_px *
          m_impl->options.forward_backward_px;
      for ( std::size_t index = 0; index < m_impl->tracks.size(); ++index )
      {
        if ( forward_status[ index ] == 0 || backward_status[ index ] == 0 ||
             !inBounds( curr_pts[ index ], left.cols, left.rows ) )
        {
          ++stats.forward_backward_rejected;
          continue;
        }
        const float dx = back_pts[ index ].x - prev_pts[ index ].x;
        const float dy = back_pts[ index ].y - prev_pts[ index ].y;
        if ( ( static_cast<double>( dx ) * static_cast<double>( dx ) +
               static_cast<double>( dy ) * static_cast<double>( dy ) ) >
             fb_limit_sq )
        {
          ++stats.forward_backward_rejected;
          continue;
        }
        LiveTrack track = m_impl->tracks[ index ];
        track.pixel     = curr_pts[ index ];
        ++track.length;
        survivors.push_back( track );
      }
      m_impl->tracks  = std::move( survivors );
      stats.tracked   = static_cast<std::uint32_t>( m_impl->tracks.size() );
      m_impl->detectNewTracks( left, stats.detected );
    }

    m_impl->prev_left = left.clone();
    m_impl->has_prev  = true;

    FrameTracks output = m_impl->makeOutput( rectified.timestamp );
    output.stats.tracked                   = stats.tracked;
    output.stats.detected                  = stats.detected;
    output.stats.forward_backward_rejected =
        stats.forward_backward_rejected;
    return output;
  }

}  // namespace phad::frontend
