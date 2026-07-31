#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <variant>
#include <vector>

#include "apps/stereo_pair_stream.hpp"
#include "phad/camera/stereo_rectifier.hpp"
#include "phad/io/dataset/dataset_replay_source.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"

/**
 * @file euroc_mh01_rectify_test.cpp
 * @brief MH_01_easy 上门控的立体校正行对齐 smoke。
 *
 * 需要本地数据集：PHAD_ENABLE_MH01_TESTS=ON 且设置 PHAD_EUROC_MH01_PATH。
 */

namespace
{

  namespace fs = std::filesystem;

  using phad::camera::StereoRectifier;
  using phad::sensor::Image;
  using phad::sensor::StereoFrame;

  [[nodiscard]] fs::path sequenceRoot()
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr )
    {
      return {};
    }
    return fs::path{ configured_path };
  }

  [[nodiscard]] cv::Mat toGrayMat( const Image& image )
  {
    const auto pixels = image.pixels<std::uint8_t>();
    EXPECT_TRUE( pixels.has_value() );
    cv::Mat mat( image.height(), image.width(), CV_8UC1 );
    std::copy( pixels->begin(), pixels->end(), mat.ptr<std::uint8_t>() );
    return mat;
  }

  [[nodiscard]] double medianAbsolute( std::vector<double> values )
  {
    EXPECT_FALSE( values.empty() );
    const auto mid = values.begin() +
                     static_cast<std::ptrdiff_t>( values.size() / 2 );
    std::nth_element( values.begin(), mid, values.end() );
    return *mid;
  }

  class Mh01RectifyTest : public ::testing::Test
  {
  protected:
    void SetUp() override
    {
      m_root = sequenceRoot();
      if ( m_root.empty() )
      {
        GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
      }
    }

    fs::path m_root;
  };

  TEST_F( Mh01RectifyTest, RectifiedPairsHaveAlignedRows )
  {
    auto opened = phad::io::dataset::euroc::open( m_root );
    ASSERT_TRUE( opened ) << opened.error().describe();

    auto rectifier =
        StereoRectifier::create( opened.value().calibration() );
    ASSERT_TRUE( rectifier ) << rectifier.error().detail;
    EXPECT_GT( rectifier.value().calibration().baselineM(), 0.05 );
    EXPECT_LT( rectifier.value().calibration().baselineM(), 0.20 );

    phad::io::dataset::DatasetReplaySource source{ opened.value() };
    phad::apps::StereoPairStream           stream{ source };
    constexpr int                          kFramesToCheck = 5;
    constexpr int                          kMaxCorners    = 400;
    std::vector<double>                    frame_medians;

    for ( int frame_index = 0; frame_index < kFramesToCheck; ++frame_index )
    {
      const auto loaded = stream.next();
      ASSERT_TRUE( std::holds_alternative<StereoFrame>( loaded ) )
          << "failed to load stereo frame " << frame_index;
      const auto& raw = std::get<StereoFrame>( loaded );

      auto rectified = rectifier.value().rectify( raw );
      ASSERT_TRUE( rectified ) << rectified.error().detail;

      const cv::Mat left  = toGrayMat( rectified.value().left );
      const cv::Mat right = toGrayMat( rectified.value().right );

      std::vector<cv::Point2f> left_corners;
      cv::goodFeaturesToTrack( left, left_corners, kMaxCorners, 0.01, 12.0 );
      ASSERT_GE( left_corners.size(), 50U );

      std::vector<cv::Point2f>  right_corners;
      std::vector<std::uint8_t> status;
      std::vector<float>        error;
      cv::calcOpticalFlowPyrLK( left, right, left_corners, right_corners,
                                status, error, cv::Size( 21, 21 ), 3 );

      std::vector<double> abs_dy;
      abs_dy.reserve( left_corners.size() );
      for ( std::size_t i = 0; i < left_corners.size(); ++i )
      {
        if ( status[ i ] == 0 )
        {
          continue;
        }
        const double disparity =
            static_cast<double>( left_corners[ i ].x - right_corners[ i ].x );
        if ( disparity <= 0.5 )
        {
          continue;
        }
        abs_dy.push_back( std::abs( static_cast<double>(
            left_corners[ i ].y - right_corners[ i ].y ) ) );
      }
      ASSERT_GE( abs_dy.size(), 30U ) << "frame " << frame_index;
      frame_medians.push_back( medianAbsolute( std::move( abs_dy ) ) );
    }

    const double overall_median = medianAbsolute( frame_medians );
    EXPECT_LT( overall_median, 1.0 )
        << "median |y_L - y_R| after rectify should be sub-pixel on MH_01";
  }

}  // namespace
