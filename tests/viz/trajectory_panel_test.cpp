#include "phad/viz/trajectory_panel.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

#include "phad/common/timestamp.hpp"
#include "phad/common/trajectory.hpp"
#include "tests/common/synthetic_trajectory.hpp"

/**
 * @file trajectory_panel_test.cpp
 * @brief 俯视 x-y 面板的投影与标记位置。
 *
 * 测试不打开窗口，只检查渲染到内存画布的结果，因此在无显示环境下也能跑。
 */

namespace
{

  using phad::common::TimedPose;
  using phad::common::Timestamp;
  using phad::common::Trajectory;
  using phad::testing::kEurocEpochNs;
  using phad::testing::kStepNs;
  using phad::testing::makePose;
  using phad::viz::TrajectoryPanel;
  using phad::viz::TrajectoryPanelOptions;

  // 200 x 200 的画布留白 20，可用区域 160 x 160；配合边长 10 m 的正方形
  // 轨迹，比例恰为 16 px/m。
  constexpr TrajectoryPanelOptions kOptions{ 200, 200, 20 };

  [[nodiscard]] Trajectory makeTrajectory(
      const std::vector<Eigen::Vector3d>& positions )
  {
    std::vector<TimedPose> poses;
    poses.reserve( positions.size() );
    for ( std::size_t index = 0; index < positions.size(); ++index )
    {
      poses.push_back( makePose(
          kEurocEpochNs + static_cast<std::int64_t>( index ) * kStepNs,
          positions[ index ] ) );
    }
    return Trajectory::create( std::move( poses ) ).value();
  }

  [[nodiscard]] Trajectory makeSquare()
  {
    return makeTrajectory( { { 0.0, 0.0, 0.0 },
                             { 10.0, 0.0, 0.0 },
                             { 10.0, 10.0, 0.0 },
                             { 0.0, 10.0, 0.0 } } );
  }

  [[nodiscard]] Timestamp timestampAt( std::int64_t index )
  {
    return Timestamp{ kEurocEpochNs + index * kStepNs };
  }

  TEST( TrajectoryPanelTest, MapsWorldSpanOntoTheUsableArea )
  {
    const TrajectoryPanel panel{ makeSquare(), kOptions };

    // 世界系 y 向上，画布行号向下，因此 y 最小的角落在画布底部。
    const cv::Point bottom_left = panel.project( { 0.0, 0.0, 0.0 } );
    EXPECT_EQ( bottom_left.x, 20 );
    EXPECT_EQ( bottom_left.y, 180 );

    const cv::Point top_right = panel.project( { 10.0, 10.0, 0.0 } );
    EXPECT_EQ( top_right.x, 180 );
    EXPECT_EQ( top_right.y, 20 );
  }

  TEST( TrajectoryPanelTest, KeepsTheSameScaleOnBothAxes )
  {
    // x 跨度 10 m、y 跨度 1 m：长边定比例（16 px/m），短边在画布内居中，
    // 否则俯视图会把轨迹拉伸变形。
    const TrajectoryPanel panel{
        makeTrajectory( { { 0.0, 0.0, 0.0 }, { 10.0, 1.0, 0.0 } } ), kOptions };

    const cv::Point origin = panel.project( { 0.0, 0.0, 0.0 } );
    EXPECT_EQ( panel.project( { 1.0, 0.0, 0.0 } ).x - origin.x, 16 );
    EXPECT_EQ( origin.y - panel.project( { 0.0, 1.0, 0.0 } ).y, 16 );
    EXPECT_EQ( origin.x, 20 );
    EXPECT_EQ( origin.y, 108 );
  }

  TEST( TrajectoryPanelTest, IgnoresTheVerticalAxis )
  {
    const TrajectoryPanel panel{ makeSquare(), kOptions };
    EXPECT_EQ( panel.project( { 3.0, 4.0, 0.0 } ),
               panel.project( { 3.0, 4.0, 100.0 } ) );
  }

  TEST( TrajectoryPanelTest, RendersTheRequestedCanvas )
  {
    const TrajectoryPanel panel{ makeSquare(), kOptions };
    const cv::Mat         canvas = panel.render( timestampAt( 0 ) );
    EXPECT_EQ( canvas.cols, kOptions.width_px );
    EXPECT_EQ( canvas.rows, kOptions.height_px );
    EXPECT_EQ( canvas.type(), CV_8UC3 );
  }

  TEST( TrajectoryPanelTest, MarksThePoseAtTheRequestedTimestamp )
  {
    const TrajectoryPanel panel{ makeSquare(), kOptions };
    const cv::Point       first  = panel.project( { 0.0, 0.0, 0.0 } );
    const cv::Point       second = panel.project( { 10.0, 0.0, 0.0 } );

    const cv::Mat at_first  = panel.render( timestampAt( 0 ) );
    const cv::Mat at_second = panel.render( timestampAt( 1 ) );

    // 标记所在像素随时间戳移动：两次渲染在两个角落上的颜色互换。
    EXPECT_NE( at_first.at<cv::Vec3b>( first ),
               at_second.at<cv::Vec3b>( first ) );
    EXPECT_NE( at_first.at<cv::Vec3b>( second ),
               at_second.at<cv::Vec3b>( second ) );
    EXPECT_EQ( at_first.at<cv::Vec3b>( first ),
               at_second.at<cv::Vec3b>( second ) );
  }

  TEST( TrajectoryPanelTest, HoldsTheLastPoseNotAfterTheTimestamp )
  {
    const TrajectoryPanel panel{ makeSquare(), kOptions };
    const cv::Mat         at_second = panel.render( timestampAt( 1 ) );

    // 时间戳落在两个位姿之间时停在前一个，早于首个位姿时停在首个，
    // 晚于末个位姿时停在末个。
    const Timestamp between{ kEurocEpochNs + kStepNs + kStepNs / 2 };
    EXPECT_TRUE( std::equal( at_second.begin<cv::Vec3b>(),
                             at_second.end<cv::Vec3b>(),
                             panel.render( between ).begin<cv::Vec3b>() ) );

    const cv::Mat at_first = panel.render( timestampAt( 0 ) );
    EXPECT_TRUE( std::equal( at_first.begin<cv::Vec3b>(),
                             at_first.end<cv::Vec3b>(),
                             panel.render( Timestamp{ 0 } )
                                 .begin<cv::Vec3b>() ) );

    const cv::Mat at_last = panel.render( timestampAt( 3 ) );
    EXPECT_TRUE( std::equal( at_last.begin<cv::Vec3b>(),
                             at_last.end<cv::Vec3b>(),
                             panel.render( timestampAt( 1000 ) )
                                 .begin<cv::Vec3b>() ) );
  }

  TEST( TrajectoryPanelTest, CentersATrajectoryWithoutHorizontalExtent )
  {
    // 静止序列的跨度为零，比例无从确定；此时全部位姿落在画布中心，
    // 而不是产生除零或飞出画布的像素。
    const TrajectoryPanel panel{
        makeTrajectory( { { 2.0, -3.0, 0.0 }, { 2.0, -3.0, 1.0 } } ),
        kOptions };
    const cv::Point center = panel.project( { 2.0, -3.0, 0.0 } );
    EXPECT_EQ( center.x, 100 );
    EXPECT_EQ( center.y, 100 );
    EXPECT_NO_THROW( (void)panel.render( timestampAt( 0 ) ) );
  }

  TEST( TrajectoryPanelTest, RejectsOptionsWithoutUsableArea )
  {
    EXPECT_THROW( TrajectoryPanel( makeSquare(), { 0, 200, 20 } ),
                  std::invalid_argument );
    EXPECT_THROW( TrajectoryPanel( makeSquare(), { 200, 200, 100 } ),
                  std::invalid_argument );
    EXPECT_THROW( TrajectoryPanel( makeSquare(), { 200, 200, -1 } ),
                  std::invalid_argument );
  }

}  // namespace
