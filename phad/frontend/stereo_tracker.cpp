#include "phad/frontend/stereo_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/calib3d.hpp>
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
      bool          evictable    = false;
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
                                int u_l, int v_l, int u_r, int v_r, int half,
                                bool zero_mean = false )
    {
      if ( !zero_mean )
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
      // 零均值 SAD: 先算 patch 均值差 c=(ΣL-ΣR)/N, 再 Σ|(L-R)-c| —— 等价于
      // 左右 patch 各自去均值后的 SAD, 对单调整亮度差(offset)不变。与帧级
      // ③(cv::add 饱和偏移)的关键区别: 不改写像素, 无饱和裁剪毁对比度。
      // 代价 ~2×(两遍扫描); 仅当 enable_zero_mean_sad 打开时走此路径。
      const int   n     = ( 2 * half + 1 ) * ( 2 * half + 1 );
      long long   sum_l = 0;
      long long   sum_r = 0;
      for ( int dy = -half; dy <= half; ++dy )
      {
        const auto* lrow = left.ptr<std::uint8_t>( v_l + dy );
        const auto* rrow = right.ptr<std::uint8_t>( v_r + dy );
        for ( int dx = -half; dx <= half; ++dx )
        {
          sum_l += lrow[ u_l + dx ];
          sum_r += rrow[ u_r + dx ];
        }
      }
      const double c   = static_cast<double>( sum_l - sum_r ) / n;
      double       sad = 0.0;
      for ( int dy = -half; dy <= half; ++dy )
      {
        const auto* lrow = left.ptr<std::uint8_t>( v_l + dy );
        const auto* rrow = right.ptr<std::uint8_t>( v_r + dy );
        for ( int dx = -half; dx <= half; ++dx )
        {
          sad += std::abs(
              static_cast<double>( lrow[ u_l + dx ] ) - rrow[ u_r + dx ] - c );
        }
      }
      return static_cast<int>( std::llround( sad ) );
    }

    // Census 变换 5×5 窗: 中心像素 vs 周围 24 邻域, 产出 24bit 描述子。
    // 对单调整亮度差(增益/曝光)不变 —— 非归一化 SAD 在 cam1 过暗时全败
    // (V2_03 诊断链 A/B1), Census 是该场景的行业标准替代。
    // 调用方需保证 (u,v) ± 2 在图像内。
    [[nodiscard]] std::uint32_t census5x5( const cv::Mat& image, int u, int v )
    {
      const std::uint8_t center = image.at<std::uint8_t>( v, u );
      std::uint32_t      desc   = 0;
      std::uint32_t      bit    = 0;
      for ( int dy = -2; dy <= 2; ++dy )
      {
        const auto* row = image.ptr<std::uint8_t>( v + dy );
        for ( int dx = -2; dx <= 2; ++dx )
        {
          if ( dx == 0 && dy == 0 )
          {
            continue;
          }
          if ( row[ u + dx ] > center )
          {
            desc |= ( 1u << bit );
          }
          ++bit;
        }
      }
      return desc;
    }

    [[nodiscard]] int hammingDist( std::uint32_t a, std::uint32_t b )
    {
      return __builtin_popcount( a ^ b );
    }

    struct SadPeak
    {
      int  u   = 0;
      int  sad = 0;
      bool ok  = false;
    };

    [[nodiscard]] SadPeak searchRow( const cv::Mat& left, const cv::Mat& right,
                                     int u_l, int v_l, int v_r, int u_lo,
                                     int u_hi, int half, bool use_census,
                                     bool zero_mean )
    {
      SadPeak peak;
      if ( u_lo > u_hi || !patchInBounds( left, u_l, v_l, half ) )
      {
        return peak;
      }

      const std::uint32_t desc_l =
          use_census ? census5x5( left, u_l, v_l ) : 0;
      // 平坦邻域(全零描述子)在 Census 下与任何平坦区域 Hamming 0, 无判别
      // 力 → 反向搜索同样退回 SAD(与 matchRight 的 per-track 规则一致)。
      const bool census_effective = use_census && desc_l != 0;
      const auto cost = [&]( int u_r, int v_r ) {
        return census_effective
                   ? hammingDist( desc_l, census5x5( right, u_r, v_r ) )
                   : sadPatch( left, right, u_l, v_l, u_r, v_r, half,
                               zero_mean );
      };

      int  best_u       = u_lo;
      int  best_sad     = std::numeric_limits<int>::max();
      int  best_tie_sad = std::numeric_limits<int>::max();
      bool found        = false;
      for ( int u_r = u_lo; u_r <= u_hi; ++u_r )
      {
        if ( !patchInBounds( right, u_r, v_r, half ) )
        {
          continue;
        }
        const int sad = cost( u_r, v_r );
        // 与 matchRight 相同的 tiebreak: hamming 并列时取 SAD 最小者。
        const int tie_sad =
            census_effective && ( !found || sad <= best_sad )
                ? sadPatch( left, right, u_l, v_l, u_r, v_r, half, zero_mean )
                : 0;
        if ( !found || sad < best_sad ||
             ( census_effective && sad == best_sad &&
               tie_sad < best_tie_sad ) )
        {
          found        = true;
          best_sad     = sad;
          best_tie_sad = tie_sad;
          best_u       = u_r;
        }
      }
      // Only reject a peak clipped at the upper (far-depth) bound: the true
      // match may lie beyond the search window. A peak at u_hi (parallax ==
      // min_disparity) is a legitimate minimal-disparity match and must be
      // kept — rejecting it discarded all far points exactly at the gate
      // (Slice ⑥ audit).
      if ( !found || best_u == u_lo )
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
                                       int half, bool zero_mean,
                                       double& u_r_sub )
    {
      if ( !patchInBounds( left, u_l, v_l, half ) ||
           !patchInBounds( right, u_r - 1, v_r, half ) ||
           !patchInBounds( right, u_r + 1, v_r, half ) )
      {
        return false;
      }
      const int s0 =
          sadPatch( left, right, u_l, v_l, u_r - 1, v_r, half, zero_mean );
      const int s1 =
          sadPatch( left, right, u_l, v_l, u_r, v_r, half, zero_mean );
      const int s2 =
          sadPatch( left, right, u_l, v_l, u_r + 1, v_r, half, zero_mean );
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
    // Slice ⑥: median inter-frame flow of the last accepted pair, used as
    // the LK initial guess on the next frame (constant-velocity model).
    std::optional<cv::Point2f> median_flow;

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

    void detectNewTracks( const cv::Mat& left, std::uint32_t& detected,
                          std::uint32_t& evicted )
    {
      std::uint32_t non_evictable = 0;
      for ( const LiveTrack& track : tracks )
      {
        if ( !track.evictable )
        {
          ++non_evictable;
        }
      }
      const int deficit =
          options.max_tracks - static_cast<int>( non_evictable );
      if ( deficit <= 0 )
      {
        return;
      }

      const int room =
          options.max_tracks - static_cast<int>( tracks.size() );
      const int need_evict = deficit - room;
      if ( need_evict > 0 )
      {
        std::vector<LandmarkId> evictable_ids;
        evictable_ids.reserve( tracks.size() );
        for ( const LiveTrack& track : tracks )
        {
          if ( track.evictable )
          {
            evictable_ids.push_back( track.id );
          }
        }
        std::sort( evictable_ids.begin(), evictable_ids.end() );
        const auto drop_n = static_cast<std::size_t>( std::min(
            need_evict, static_cast<int>( evictable_ids.size() ) ) );
        const std::unordered_set<LandmarkId> drop_set(
            evictable_ids.begin(),
            evictable_ids.begin() + static_cast<std::ptrdiff_t>( drop_n ) );
        tracks.erase( std::remove_if( tracks.begin(), tracks.end(),
                                      [ &drop_set ]( const LiveTrack& track ) {
                                        return drop_set.count( track.id ) !=
                                               0U;
                                      } ),
                      tracks.end() );
        evicted += static_cast<std::uint32_t>( drop_n );
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

        // Global minimum over the epipolar band (single peak; no cascade).
        // Uniqueness 比较 ±1 px 之外的对手峰。
        const bool left_in_bounds = patchInBounds( left, u_l, v_l, half );
        const std::uint32_t desc_l =
            options.enable_census && left_in_bounds
                ? census5x5( left, u_l, v_l )
                : 0;
        // 全零描述子 = 5×5 平坦邻域(中心 ≥ 全部邻居)。此时 Census 与
        // 任何平坦区域(含纯黑背景)Hamming 距离都为 0, 无判别力 → 不可用。
        // (合成高斯 blob 顶部平坦即触发;真实暗场景有纹理, 描述子非零,
        // 曝光鲁棒性不受影响)
        const bool census_available =
            options.enable_census && left_in_bounds && desc_l != 0;

        int  best_u     = 0;
        int  best_v     = v_l;
        double u_r_sub  = 0.0;
        bool  used_census = false;
        // 一次带验收的匹配尝试。
        // census=false → 非归一化 SAD(主路径, 原行为: 选择+相对裕度唯一性+
        // SAD 抛物线子像素)。census=true → Census 选择 + SAD tiebreak +
        // Hamming 唯一性 + 同样的 SAD 抛物线子像素(offset 在 ±1px 内
        // 局部常数, 抛物线拟合不受影响)。
        const auto try_match = [&]( bool census, int& out_u, int& out_v,
                                    double& out_sub ) -> bool {
          int  best_cost    = std::numeric_limits<int>::max();
          int  best_tie_sad = std::numeric_limits<int>::max();
          bool found_peak   = false;
          if ( left_in_bounds && u_lo <= u_hi )
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
                const int cost_val =
                    census ? hammingDist( desc_l, census5x5( right, u_r, v_r ) )
                           : sadPatch( left, right, u_l, v_l, u_r, v_r, half,
                                       options.enable_zero_mean_sad );
                // Census 量化 → 相邻列 Hamming 常并列。用 SAD 作 tiebreak:
                // 在 hamming 相同的位置里取 SAD 最小者, 使 best_u 落在真实
                // 匹配处(子像素抛物线的代价函数才有正确极小)。
                const int tie_sad =
                    census && ( !found_peak || cost_val <= best_cost )
                        ? sadPatch( left, right, u_l, v_l, u_r, v_r, half,
                                    options.enable_zero_mean_sad )
                        : 0;
                if ( !found_peak || cost_val < best_cost ||
                     ( census && cost_val == best_cost &&
                       tie_sad < best_tie_sad ) )
                {
                  best_cost    = cost_val;
                  best_tie_sad = tie_sad;
                  out_u        = u_r;
                  out_v        = v_r;
                  found_peak   = true;
                }
              }
            }
          }
          if ( !found_peak )
          {
            return false;
          }
          if ( options.stereo_uniq_ratio > 0.0 )
          {
            if ( census )
            {
              // Hamming 唯一性: 绝对阈值 1 位。实测 Hamming 在真匹配
              // ±2-3px 处是平顶(margin 中位 0-1 位), 阈值 2+ 会把多数
              // 真匹配拒掉; SAD 相对裕度不能用于 Census 验收 —— 暗帧的
              // 亮度偏移不均匀, SAD 极小位置漂移(实测 SAD@hamming-best
              // 的 margin 为负)。只拒绝"远处有第二个同样好匹配"的歧义。
              int second = std::numeric_limits<int>::max();
              for ( int v_r = v_l - options.stereo_row_tol_px;
                    v_r <= v_l + options.stereo_row_tol_px; ++v_r )
              {
                for ( int u_r = u_lo; u_r <= u_hi; ++u_r )
                {
                  if ( std::abs( u_r - out_u ) <= 1 && v_r == out_v )
                  {
                    continue;
                  }
                  if ( !patchInBounds( right, u_r, v_r, half ) )
                  {
                    continue;
                  }
                  const int h = hammingDist( desc_l,
                                             census5x5( right, u_r, v_r ) );
                  second = std::min( second, h );
                }
              }
              if ( second == std::numeric_limits<int>::max() ||
                   second - best_cost < 1 )
              {
                return false;
              }
            }
            else
            {
              // 原 SAD 相对裕度 (s2-s1)/s1 ≥ ratio。
              int second_sad = std::numeric_limits<int>::max();
              for ( int v_r = v_l - options.stereo_row_tol_px;
                    v_r <= v_l + options.stereo_row_tol_px; ++v_r )
              {
                for ( int u_r = u_lo; u_r <= u_hi; ++u_r )
                {
                  if ( std::abs( u_r - out_u ) <= 1 && v_r == out_v )
                  {
                    continue;
                  }
                  if ( !patchInBounds( right, u_r, v_r, half ) )
                  {
                    continue;
                  }
                  const int sad = sadPatch(
                      left, right, u_l, v_l, u_r, v_r, half,
                      options.enable_zero_mean_sad );
                  second_sad = std::min( second_sad, sad );
                }
              }
              if ( second_sad == std::numeric_limits<int>::max() )
              {
                return false;
              }
              if ( best_cost == 0 )
              {
                if ( second_sad <= 0 )
                {
                  return false;
                }
              }
              else
              {
                const double margin =
                    static_cast<double>( second_sad - best_cost ) /
                    static_cast<double>( best_cost );
                if ( margin < options.stereo_uniq_ratio )
                {
                  return false;
                }
              }
            }
          }
          return refineSubpixel( left, right, u_l, v_l, out_u, out_v, half,
                                 options.enable_zero_mean_sad, out_sub );
        };

        // 主路径 SAD;被拒(无峰/唯一性/子像素)时,暗曝光帧(如 V2_03
        // cam1 欠曝光)走 Census 兜底 —— SAD 极小漂移是数据侧 offset,
        // Census 对单调亮度差不变。
        if ( !try_match( false, best_u, best_v, u_r_sub ) )
        {
          if ( !census_available ||
               !try_match( true, best_u, best_v, u_r_sub ) )
          {
            continue;
          }
          used_census = true;
        }


          // Reverse 1D search on the same row; do not touch
          // forward_backward_rejected (temporal FB only).
          // 反搜必须与主路径同一 mode: Census 命中(暗帧)时反搜的右图
          // 暗 patch 描述子同样对曝光不变;若暗 patch 平坦(desc=0),
          // searchRow 内部自动退回 SAD。
          if ( options.stereo_check_bidir )
          {
            const int back_lo =
                best_u + static_cast<int>( std::ceil( d_min ) );
            const int back_hi =
                best_u + static_cast<int>( std::floor( d_max ) );
            const SadPeak back =
                searchRow( right, left, best_u, best_v, best_v, back_lo,
                           back_hi, half, used_census,
                           options.enable_zero_mean_sad );
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

  void StereoTracker::markEvictable( std::span<const LandmarkId> ids )
  {
    if ( ids.empty() )
    {
      return;
    }
    const std::unordered_set<LandmarkId> mark_set( ids.begin(), ids.end() );
    for ( LiveTrack& track : m_impl->tracks )
    {
      if ( mark_set.count( track.id ) != 0U )
      {
        track.evictable = true;
      }
    }
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

    const cv::Mat left_raw  = toGrayMat( rectified.left );
    const cv::Mat right_raw = toGrayMat( rectified.right );
    // Slice ⑥: CLAHE equalization (ORB-SLAM3 practice). V1_03/V2_03 have
    // exposure inconsistencies that break LK brightness constancy and the
    // raw-SAD stereo match; one pre-pass fixes GFTT/LK/SAD at once.
    // Slice ⑥: CLAHE equalization (ORB-SLAM3 practice). V1_03/V2_03 have
    // exposure inconsistencies that break LK brightness constancy and the
    // raw-SAD stereo match. Milder params (clip 2.0, tile 16x16) to avoid
    // hurting MH sequences with normal lighting.
    cv::Mat left, right;
    if ( m_impl->options.enable_clahe )
    {
      // Slice ⑥: CLAHE equalization (ORB-SLAM3 practice). V1_03/V2_03 have
      // exposure inconsistencies that break LK brightness constancy and the
      // raw-SAD stereo match. Milder params (clip 2.0, tile 16x16) to avoid
      // hurting MH sequences with normal lighting.
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE( 2.0, cv::Size( 16, 16 ) );
      clahe->apply( left_raw, left );
      clahe->apply( right_raw, right );
    }
    else
    {
      left  = left_raw;
      right = right_raw;
    }
    // pre-M4 round 2: frame-level exposure normalization. CLAHE equalizes
    // each camera independently and does not remove a systematic cam0/cam1
    // brightness offset; on V2_03 dark frames the raw-SAD cost carries a
    // 225-px × |offset| penalty that drowns the true minimum. Align the
    // right image's mean to the left before matching (SVO-style affine
    // offset, frame-level form). Saturation is safe: cv::add clamps [0,255].
    if ( m_impl->options.enable_exposure_normalize )
    {
      const double mean_l = cv::mean( left )[0];
      const double mean_r = cv::mean( right )[0];
      const double offset = mean_l - mean_r;
      // Threshold 20 px: only correct systematic large exposure mismatch.
      // Measured on all 11 EuRoC sequences (2026-08-07): healthy sequences
      // stay |offset| < 20 (MH_04 max 17.9, V1_01 max 20.7), while V2_03
      // dark frames reach +38~+89. A small correction (MH_01 +2.2) hurt
      // MH_01 ATE +41% — the saturating cv::add clips bright pixels and
      // destroys contrast, so leave small offsets to the raw SAD.
      if ( std::abs( offset ) > 20.0 )
      {
        cv::add( right, cv::Scalar( offset ), right );
      }
    }
    FrameStats stats{};

    if ( !m_impl->has_prev || m_impl->tracks.empty() )
    {
      m_impl->tracks.clear();
      // All tracks lost: the old median-flow guess is stale and must not
      // seed the newly detected tracks (Slice ⑥ audit).
      m_impl->median_flow.reset();
      m_impl->detectNewTracks( left, stats.detected, stats.evicted );
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
      // Slice ⑥: seed LK with the constant-velocity median-flow guess so
      // fast yaw (V-series) does not push displacement out of the LK
      // convergence region.
      const bool use_initial_flow =
          m_impl->options.enable_median_flow &&
          m_impl->median_flow.has_value() && !prev_pts.empty();
      curr_pts.resize( prev_pts.size() );
      if ( use_initial_flow )
      {
        for ( std::size_t i = 0; i < prev_pts.size(); ++i )
        {
          curr_pts[ i ] = prev_pts[ i ] + *m_impl->median_flow;
        }
      }
      cv::calcOpticalFlowPyrLK(
          m_impl->prev_left, left, prev_pts, curr_pts, forward_status,
          forward_error, win, m_impl->options.lk_pyramid_levels,
          cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                            30, 0.01 ),
          use_initial_flow ? cv::OPTFLOW_USE_INITIAL_FLOW : 0,
          1e-4 );

      std::vector<cv::Point2f>  back_pts;
      std::vector<std::uint8_t> backward_status;
      std::vector<float>        backward_error;
      cv::calcOpticalFlowPyrLK(
          left, m_impl->prev_left, curr_pts, back_pts, backward_status,
          backward_error, win, m_impl->options.lk_pyramid_levels );

      std::vector<LiveTrack> survivors;
      survivors.reserve( m_impl->tracks.size() );
      std::vector<std::size_t> survivor_indices;  // original track indices
      survivor_indices.reserve( m_impl->tracks.size() );
      std::vector<cv::Point2f> flows;  // Slice ⑥: survivor displacements
      flows.reserve( m_impl->tracks.size() );
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
        survivor_indices.push_back( index );
        flows.push_back( cv::Point2f(
            curr_pts[ index ].x - prev_pts[ index ].x,
            curr_pts[ index ].y - prev_pts[ index ].y ) );
      }
      // Slice ⑥: F-matrix RANSAC geometric check after FB (VINS/Kimera
      // practice). FB rejects "inconsistent" tracks but not "consistently
      // wrong" ones under motion blur; geometry culls them before they
      // pollute PnP/BA.
      if ( m_impl->options.enable_fransac &&
           survivor_indices.size() >= 8U )
      {
        std::vector<cv::Point2f> prev_f, curr_f;
        prev_f.reserve( survivor_indices.size() );
        curr_f.reserve( survivor_indices.size() );
        for ( const std::size_t orig : survivor_indices )
        {
          prev_f.push_back( prev_pts[ orig ] );
          curr_f.push_back( curr_pts[ orig ] );
        }
        std::vector<uchar> geo_status;
        cv::findFundamentalMat( prev_f, curr_f, cv::FM_RANSAC, 3.0, 0.99,
                                geo_status );
        if ( !geo_status.empty() &&
             geo_status.size() == survivor_indices.size() )
        {
          std::vector<LiveTrack>       geo_survivors;
          std::vector<std::size_t>     geo_indices;
          std::vector<cv::Point2f>     geo_flows;
          geo_survivors.reserve( survivors.size() );
          geo_indices.reserve( survivor_indices.size() );
          geo_flows.reserve( flows.size() );
          for ( std::size_t i = 0; i < survivor_indices.size(); ++i )
          {
            if ( geo_status[ i ] == 0 )
            {
              ++stats.forward_backward_rejected;
              continue;
            }
            geo_survivors.push_back( survivors[ i ] );
            geo_indices.push_back( survivor_indices[ i ] );
            geo_flows.push_back( flows[ i ] );
          }
          survivors         = std::move( geo_survivors );
          survivor_indices  = std::move( geo_indices );
          flows             = std::move( geo_flows );
        }
      }

      // Slice ⑥: update the median-flow guess from survivor displacements.
      if ( flows.size() >= 5U )
      {
        std::vector<float> fx, fy;
        fx.reserve( flows.size() );
        fy.reserve( flows.size() );
        for ( const cv::Point2f& f : flows )
        {
          fx.push_back( f.x );
          fy.push_back( f.y );
        }
        std::sort( fx.begin(), fx.end() );
        std::sort( fy.begin(), fy.end() );
        m_impl->median_flow = cv::Point2f(
            fx[ fx.size() / 2U ], fy[ fy.size() / 2U ] );
      }
      else
      {
        m_impl->median_flow.reset();
      }
      m_impl->tracks = std::move( survivors );
      stats.tracked  = static_cast<std::uint32_t>( m_impl->tracks.size() );
      m_impl->detectNewTracks( left, stats.detected, stats.evicted );
    }

    m_impl->matchRight( left, right, stats );
    m_impl->prev_left = left.clone();
    m_impl->has_prev  = true;
    return m_impl->makeOutput( rectified.timestamp, stats );
  }

}  // namespace phad::frontend
