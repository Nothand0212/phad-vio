#include "phad/viz/trajectory_panel.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>

namespace phad::viz
{

  namespace
  {

    // 只用于避免静止序列的除零；跨度小于它时全部位姿落在画布中心。
    constexpr double kMinSpanM = 1e-9;

    constexpr int    kPathThickness = 1;
    constexpr int    kMarkerRadius  = 4;
    const cv::Scalar kBackgroundColor{ 24, 24, 24 };  // BGR
    const cv::Scalar kPathColor{ 180, 180, 180 };     // BGR
    const cv::Scalar kMarkerColor{ 0, 0, 255 };       // BGR

    struct Bounds
    {
      Eigen::Vector2d min = Eigen::Vector2d::Zero();
      Eigen::Vector2d max = Eigen::Vector2d::Zero();
    };

    [[nodiscard]] Bounds horizontalBounds( const common::Trajectory& trajectory )
    {
      Bounds bounds;
      bounds.min.setConstant( std::numeric_limits<double>::max() );
      bounds.max.setConstant( std::numeric_limits<double>::lowest() );
      for ( const common::TimedPose& pose : trajectory.poses() )
      {
        const Eigen::Vector2d position = pose.T_W_B.translation().head<2>();
        bounds.min                     = bounds.min.cwiseMin( position );
        bounds.max                     = bounds.max.cwiseMax( position );
      }
      return bounds;
    }

  }  // namespace

  TrajectoryPanel::TrajectoryPanel( const common::Trajectory&     trajectory,
                                    const TrajectoryPanelOptions& options )
  {
    const int usable_width  = options.width_px - 2 * options.margin_px;
    const int usable_height = options.height_px - 2 * options.margin_px;
    if ( options.margin_px < 0 || usable_width <= 0 || usable_height <= 0 )
    {
      throw std::invalid_argument(
          "trajectory panel of " + std::to_string( options.width_px ) + "x" +
          std::to_string( options.height_px ) + " with margin " +
          std::to_string( options.margin_px ) + " has no usable area" );
    }

    const Bounds          bounds = horizontalBounds( trajectory );
    const Eigen::Vector2d span   = bounds.max - bounds.min;
    m_origin_W                   = bounds.min;
    m_scale_px_per_m             = std::min(
        static_cast<double>( usable_width ) / std::max( span.x(), kMinSpanM ),
        static_cast<double>( usable_height ) /
            std::max( span.y(), kMinSpanM ) );

    // 轨迹在未被比例占满的方向上居中。
    const Eigen::Vector2d extent_px = span * m_scale_px_per_m;
    m_origin_px.x                   = static_cast<double>( options.margin_px ) +
                    0.5 * ( static_cast<double>( usable_width ) -
                            extent_px.x() );
    m_origin_px.y = static_cast<double>( options.height_px - options.margin_px ) -
                    0.5 * ( static_cast<double>( usable_height ) -
                            extent_px.y() );

    m_timestamps.reserve( trajectory.size() );
    m_points.reserve( trajectory.size() );
    for ( const common::TimedPose& pose : trajectory.poses() )
    {
      m_timestamps.push_back( pose.timestamp );
      m_points.push_back( project( pose.T_W_B.translation() ) );
    }

    m_background = cv::Mat( options.height_px, options.width_px, CV_8UC3,
                            kBackgroundColor );
    if ( m_points.size() >= 2U )
    {
      cv::polylines( m_background, m_points, false, kPathColor, kPathThickness,
                     cv::LINE_AA );
    }
  }

  cv::Point TrajectoryPanel::project(
      const Eigen::Vector3d& position_W ) const noexcept
  {
    const Eigen::Vector2d offset_px =
        ( position_W.head<2>() - m_origin_W ) * m_scale_px_per_m;
    return cv::Point{ cvRound( m_origin_px.x + offset_px.x() ),
                      cvRound( m_origin_px.y - offset_px.y() ) };
  }

  cv::Mat TrajectoryPanel::render( common::Timestamp timestamp ) const
  {
    cv::Mat canvas = m_background.clone();
    cv::circle( canvas, m_points[ indexAt( timestamp ) ], kMarkerRadius,
                kMarkerColor, cv::FILLED, cv::LINE_AA );
    return canvas;
  }

  std::size_t TrajectoryPanel::indexAt(
      common::Timestamp timestamp ) const noexcept
  {
    // 不晚于 timestamp 的最后一个位姿；早于首个位姿时停在首个。
    const auto after = std::upper_bound( m_timestamps.begin(),
                                         m_timestamps.end(), timestamp );
    if ( after == m_timestamps.begin() )
    {
      return 0U;
    }
    return static_cast<std::size_t>(
        std::distance( m_timestamps.begin(), std::prev( after ) ) );
  }

}  // namespace phad::viz
