#include <algorithm>
#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

#include "phad/frontend/stereo_tracker.hpp"

namespace phad::frontend
{
  namespace
  {

    /**
     * @brief 描述一个跟踪。
     *
     * 用于描述一个跟踪，包括：
     * - id：跟踪的特征点 ID
     * - length：跟踪的长度
     * - pixel：跟踪的左目像素坐标
     * - last_disp_px：跟踪的上一帧视差
     * - disparity_px：跟踪的视差
     * - status：跟踪的立体匹配状态
     */
    struct LiveTrack
    {
      LandmarkId    id           = 0;
      std::uint32_t length       = 0;
      cv::Point2f   pixel        = {};
      float         last_disp_px = 0.0F;
      double        disparity_px = 0.0;
      StereoStatus  status       = StereoStatus::kNoRightMatch;
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
            "StereoTracker expects single-channel uint8 images" );
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

    [[nodiscard]] double percentileSorted( std::vector<double> values,
                                           double              fraction )
    {
      if ( values.empty() )
      {
        return 0.0;
      }
      std::sort( values.begin(), values.end() );
      const double position =
          fraction * static_cast<double>( values.size() - 1U );
      const auto lower = static_cast<std::size_t>( std::floor( position ) );
      const auto upper = static_cast<std::size_t>( std::ceil( position ) );
      if ( lower == upper )
      {
        return values[ lower ];
      }
      const double weight = position - static_cast<double>( lower );
      return values[ lower ] * ( 1.0 - weight ) + values[ upper ] * weight;
    }

  }  // namespace

  struct StereoTracker::Impl
  {
    camera::RectifiedStereoCalibration calibration;
    StereoTrackerOptions               options;
    cv::Mat                            prev_left;
    std::vector<LiveTrack>             tracks;
    LandmarkId                         next_id  = 1;
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
      if ( !( options.max_epipolar_px > 0.0 ) ||
           !( options.min_disparity_px > 0.0 ) )
      {
        throw std::invalid_argument(
            "max_epipolar_px and min_disparity_px must be positive" );
      }
      if ( !( options.min_depth_m > 0.0 ) ||
           !( options.max_depth_m > options.min_depth_m ) )
      {
        throw std::invalid_argument(
            "depth range must satisfy 0 < min_depth_m < max_depth_m" );
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

      cv::Mat                mask    = cv::Mat::ones( left.size(), CV_8UC1 ) * 255;
      std::vector<LiveTrack> ordered = tracks;
      std::sort( ordered.begin(), ordered.end(),
                 []( const LiveTrack& lhs, const LiveTrack& rhs ) {
                   return lhs.length > rhs.length;
                 } );
      for ( const LiveTrack& track : ordered )
      {
        cv::circle( mask, cv::Point( cvRound( track.pixel.x ), cvRound( track.pixel.y ) ),
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

    void matchRight( const cv::Mat& left, const cv::Mat& right,
                     FrameStats& stats )
    {
      if ( tracks.empty() )
      {
        stats.epipolar_median_px = 0.0;
        stats.epipolar_p95_px    = 0.0;
        return;
      }

      std::vector<cv::Point2f> left_pts;
      std::vector<cv::Point2f> right_seed;
      left_pts.reserve( tracks.size() );
      right_seed.reserve( tracks.size() );
      for ( const LiveTrack& track : tracks )
      {
        left_pts.push_back( track.pixel );
        right_seed.emplace_back( track.pixel.x - track.last_disp_px,
                                 track.pixel.y );
      }

      const cv::Size            win( options.lk_window_px, options.lk_window_px );
      std::vector<cv::Point2f>  right_pts = right_seed;
      std::vector<std::uint8_t> forward_status;
      std::vector<float>        forward_error;
      cv::calcOpticalFlowPyrLK( left, right, left_pts, right_pts,
                                forward_status, forward_error, win,
                                options.lk_pyramid_levels );

      std::vector<cv::Point2f>  back_pts;
      std::vector<std::uint8_t> backward_status;
      std::vector<float>        backward_error;
      cv::calcOpticalFlowPyrLK( right, left, right_pts, back_pts,
                                backward_status, backward_error, win,
                                options.lk_pyramid_levels );

      const double fb_limit_sq =
          options.forward_backward_px * options.forward_backward_px;
      std::vector<double> epipolar_errors;
      epipolar_errors.reserve( tracks.size() );

      for ( std::size_t index = 0; index < tracks.size(); ++index )
      {
        LiveTrack& track   = tracks[ index ];
        track.disparity_px = 0.0;
        track.status       = StereoStatus::kNoRightMatch;

        if ( forward_status[ index ] == 0 || backward_status[ index ] == 0 ||
             !inBounds( right_pts[ index ], right.cols, right.rows ) )
        {
          continue;
        }
        const float bdx = back_pts[ index ].x - left_pts[ index ].x;
        const float bdy = back_pts[ index ].y - left_pts[ index ].y;
        if ( ( static_cast<double>( bdx ) * static_cast<double>( bdx ) +
               static_cast<double>( bdy ) * static_cast<double>( bdy ) ) >
             fb_limit_sq )
        {
          continue;
        }

        const double abs_dy = std::abs( static_cast<double>(
            left_pts[ index ].y - right_pts[ index ].y ) );
        epipolar_errors.push_back( abs_dy );

        const double disparity = static_cast<double>( left_pts[ index ].x -
                                                      right_pts[ index ].x );
        track.last_disp_px =
            static_cast<float>( disparity );  // seed next frame even if rejected

        if ( abs_dy > options.max_epipolar_px )
        {
          track.status = StereoStatus::kInvalidDisparity;
          ++stats.epipolar_rejected;
          continue;
        }
        if ( disparity <= options.min_disparity_px )
        {
          track.status = StereoStatus::kInvalidDisparity;
          ++stats.disparity_rejected;
          continue;
        }

        const double depth_m =
            calibration.fxPixels() * calibration.baselineM() / disparity;
        if ( depth_m < options.min_depth_m || depth_m > options.max_depth_m )
        {
          track.status = StereoStatus::kDepthOutOfRange;
          ++stats.depth_rejected;
          continue;
        }

        track.status       = StereoStatus::kValid;
        track.disparity_px = disparity;
      }

      stats.epipolar_median_px = percentileSorted( epipolar_errors, 0.5 );
      stats.epipolar_p95_px    = percentileSorted( epipolar_errors, 0.95 );
    }

    [[nodiscard]] FrameTracks makeOutput( common::Timestamp timestamp,
                                          const FrameStats& stats ) const
    {
      FrameTracks output;
      output.timestamp = timestamp;
      output.stats     = stats;
      output.observations.reserve( tracks.size() );

      std::vector<std::uint32_t> lengths;
      lengths.reserve( tracks.size() );
      std::uint32_t length_max = 0U;
      for ( const LiveTrack& track : tracks )
      {
        output.observations.push_back( TrackObservation{
            track.id,
            Eigen::Vector2d{ track.pixel.x, track.pixel.y },
            track.disparity_px,
            track.status,
            track.length } );
        lengths.push_back( track.length );
        length_max = std::max( length_max, track.length );
      }

      output.stats.track_length_median =
          medianLength( std::move( lengths ) );
      output.stats.track_length_max = length_max;
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
         rectified.left.height() != m_impl->calibration.imageHeight() ||
         rectified.right.width() != m_impl->calibration.imageWidth() ||
         rectified.right.height() != m_impl->calibration.imageHeight() )
    {
      throw std::invalid_argument(
          "rectified stereo frame size does not match tracker calibration" );
    }

    const cv::Mat left  = toGrayMat( rectified.left );
    const cv::Mat right = toGrayMat( rectified.right );
    FrameStats    stats{};

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

      std::vector<cv::Point2f>  curr_pts;
      std::vector<std::uint8_t> forward_status;
      std::vector<float>        forward_error;
      const cv::Size            win( m_impl->options.lk_window_px,
                                     m_impl->options.lk_window_px );
      cv::calcOpticalFlowPyrLK(
          m_impl->prev_left, left, prev_pts, curr_pts, forward_status,
          forward_error, win, m_impl->options.lk_pyramid_levels );

      std::vector<cv::Point2f>  back_pts;
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
      m_impl->tracks = std::move( survivors );
      stats.tracked  = static_cast<std::uint32_t>( m_impl->tracks.size() );
      m_impl->detectNewTracks( left, stats.detected );
    }

    m_impl->matchRight( left, right, stats );
    m_impl->prev_left = left.clone();
    m_impl->has_prev  = true;
    return m_impl->makeOutput( rectified.timestamp, stats );
  }

}  // namespace phad::frontend
