#include "phad/frontend/stereo_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

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
     * - disparity_px：跟踪的视差
     * - status：跟踪的立体匹配状态
     */
    struct LiveTrack
    {
      LandmarkId    id           = 0;
      std::uint32_t length       = 0;
      cv::Point2f   pixel        = {};
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

    [[nodiscard]] bool patchInBounds( const cv::Mat& image, int u, int v,
                                      int half )
    {
      return u - half >= 0 && u + half < image.cols && v - half >= 0 &&
             v + half < image.rows;
    }

    [[nodiscard]] int sadPatch( const cv::Mat& left, const cv::Mat& right,
                                int u_l, int v_l, int u_r, int v_r, int half )
    {
      int sad = 0;
      for ( int dy = -half; dy <= half; ++dy )
      {
        const auto* lrow = left.ptr<std::uint8_t>( v_l + dy );
        const auto* rrow = right.ptr<std::uint8_t>( v_r + dy );
        for ( int dx = -half; dx <= half; ++dx )
        {
          sad += std::abs( static_cast<int>( lrow[ u_l + dx ] ) -
                           static_cast<int>( rrow[ u_r + dx ] ) );
        }
      }
      return sad;
    }

    struct SadPeak
    {
      int  u   = 0;
      int  sad = 0;
      bool ok  = false;
    };

    [[nodiscard]] SadPeak searchRow( const cv::Mat& left, const cv::Mat& right,
                                     int u_l, int v_l, int v_r, int u_lo,
                                     int u_hi, int half )
    {
      SadPeak peak;
      if ( u_lo > u_hi || !patchInBounds( left, u_l, v_l, half ) )
      {
        return peak;
      }

      int  best_u   = u_lo;
      int  best_sad = std::numeric_limits<int>::max();
      bool found    = false;
      for ( int u_r = u_lo; u_r <= u_hi; ++u_r )
      {
        if ( !patchInBounds( right, u_r, v_r, half ) )
        {
          continue;
        }
        const int sad =
            sadPatch( left, right, u_l, v_l, u_r, v_r, half );
        if ( !found || sad < best_sad )
        {
          found    = true;
          best_sad = sad;
          best_u   = u_r;
        }
      }
      if ( !found || best_u == u_lo || best_u == u_hi )
      {
        return peak;
      }
      peak.u   = best_u;
      peak.sad = best_sad;
      peak.ok  = true;
      return peak;
    }

    [[nodiscard]] bool refineSubpixel( const cv::Mat& left, const cv::Mat& right,
                                       int u_l, int v_l, int u_r, int v_r,
                                       int half, double& u_r_sub )
    {
      if ( !patchInBounds( left, u_l, v_l, half ) ||
           !patchInBounds( right, u_r - 1, v_r, half ) ||
           !patchInBounds( right, u_r + 1, v_r, half ) )
      {
        return false;
      }
      const int    s0 = sadPatch( left, right, u_l, v_l, u_r - 1, v_r, half );
      const int    s1 = sadPatch( left, right, u_l, v_l, u_r, v_r, half );
      const int    s2 = sadPatch( left, right, u_l, v_l, u_r + 1, v_r, half );
      const double denom =
          static_cast<double>( s0 - 2 * s1 + s2 );
      if ( std::abs( denom ) < 1e-12 )
      {
        return false;
      }
      const double delta =
          0.5 * static_cast<double>( s0 - s2 ) / denom;
      if ( std::abs( delta ) > 1.0 )
      {
        return false;
      }
      u_r_sub = static_cast<double>( u_r ) + delta;
      return true;
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
      if ( options.stereo_sad_half_win_px < 1 )
      {
        throw std::invalid_argument( "stereo_sad_half_win_px must be >= 1" );
      }
      if ( options.stereo_row_tol_px < 0 )
      {
        throw std::invalid_argument( "stereo_row_tol_px must be >= 0" );
      }
      if ( !( options.stereo_bidir_px > 0.0 ) )
      {
        throw std::invalid_argument( "stereo_bidir_px must be positive" );
      }
      if ( options.stereo_uniq_ratio < 0.0 )
      {
        throw std::invalid_argument( "stereo_uniq_ratio must be >= 0" );
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

      const double fx_b =
          calibration.fxPixels() * calibration.baselineM();
      const int           half = options.stereo_sad_half_win_px;
      std::vector<double> epipolar_errors;
      epipolar_errors.reserve( tracks.size() );

      for ( LiveTrack& track : tracks )
      {
        track.disparity_px = 0.0;
        track.status       = StereoStatus::kNoRightMatch;

        const double d_min = std::max( options.min_disparity_px,
                                       fx_b / options.max_depth_m );
        const double d_max = fx_b / options.min_depth_m;
        if ( d_max < d_min )
        {
          continue;
        }

        const int u_l = cvRound( track.pixel.x );
        const int v_l = cvRound( track.pixel.y );
        const int u_lo =
            u_l - static_cast<int>( std::floor( d_max ) );
        const int u_hi =
            u_l - static_cast<int>( std::ceil( d_min ) );

        // Global SAD minimum over the epipolar band (single peak; no cascade).
        // Uniqueness compares rivals outside ±1 px of the peak.
        int  best_u     = 0;
        int  best_v     = v_l;
        int  best_sad   = std::numeric_limits<int>::max();
        bool found_peak = false;
        if ( patchInBounds( left, u_l, v_l, half ) && u_lo <= u_hi )
        {
          for ( int v_r = v_l - options.stereo_row_tol_px;
                v_r <= v_l + options.stereo_row_tol_px; ++v_r )
          {
            for ( int u_r = u_lo; u_r <= u_hi; ++u_r )
            {
              if ( !patchInBounds( right, u_r, v_r, half ) )
              {
                continue;
              }
              const int sad =
                  sadPatch( left, right, u_l, v_l, u_r, v_r, half );
              if ( !found_peak || sad < best_sad )
              {
                best_sad   = sad;
                best_u     = u_r;
                best_v     = v_r;
                found_peak = true;
              }
            }
          }
        }
        if ( !found_peak )
        {
          continue;
        }
        if ( options.stereo_uniq_ratio > 0.0 )
        {
          int second_sad = std::numeric_limits<int>::max();
          for ( int v_r = v_l - options.stereo_row_tol_px;
                v_r <= v_l + options.stereo_row_tol_px; ++v_r )
          {
            for ( int u_r = u_lo; u_r <= u_hi; ++u_r )
            {
              if ( std::abs( u_r - best_u ) <= 1 && v_r == best_v )
              {
                continue;
              }
              if ( !patchInBounds( right, u_r, v_r, half ) )
              {
                continue;
              }
              const int sad =
                  sadPatch( left, right, u_l, v_l, u_r, v_r, half );
              second_sad = std::min( second_sad, sad );
            }
          }
          if ( second_sad == std::numeric_limits<int>::max() )
          {
            continue;
          }
          if ( best_sad == 0 )
          {
            if ( second_sad <= 0 )
            {
              continue;
            }
          }
          else
          {
            const double margin =
                static_cast<double>( second_sad - best_sad ) /
                static_cast<double>( best_sad );
            if ( margin < options.stereo_uniq_ratio )
            {
              continue;
            }
          }
        }

        double u_r_sub = 0.0;
        if ( !refineSubpixel( left, right, u_l, v_l, best_u, best_v, half,
                              u_r_sub ) )
        {
          continue;
        }

        // Reverse 1D SAD on the same row; do not touch
        // forward_backward_rejected (temporal FB only).
        if ( options.stereo_check_bidir )
        {
          const int back_lo =
              best_u + static_cast<int>( std::ceil( d_min ) );
          const int back_hi =
              best_u + static_cast<int>( std::floor( d_max ) );
          const SadPeak back =
              searchRow( right, left, best_u, best_v, best_v, back_lo,
                         back_hi, half );
          if ( !back.ok ||
               std::abs( static_cast<double>( back.u - u_l ) ) >
                   options.stereo_bidir_px )
          {
            continue;
          }
        }

        const double abs_dy =
            std::abs( static_cast<double>( v_l - best_v ) );
        epipolar_errors.push_back( abs_dy );

        const double disparity =
            static_cast<double>( track.pixel.x ) - u_r_sub;

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

        const double depth_m = fx_b / disparity;
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

  void StereoTracker::dropTracks( std::span<const LandmarkId> ids )
  {
    if ( ids.empty() )
    {
      return;
    }
    const std::unordered_set<LandmarkId> drop_set( ids.begin(), ids.end() );
    m_impl->tracks.erase(
        std::remove_if( m_impl->tracks.begin(), m_impl->tracks.end(),
                        [ &drop_set ]( const LiveTrack& track ) {
                          return drop_set.count( track.id ) != 0U;
                        } ),
        m_impl->tracks.end() );
  }

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
