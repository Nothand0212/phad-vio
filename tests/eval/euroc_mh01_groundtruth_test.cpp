#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <random>
#include <string>
#include <utility>

#include "phad/eval/ate.hpp"
#include "phad/eval/tum_io.hpp"
#include "phad/io/dataset/euroc/euroc_groundtruth.hpp"

/**
 * @file euroc_mh01_groundtruth_test.cpp
 * @brief 在真实 MH_01_easy 序列上检查真值加载与自比 ATE。
 *
 * 该测试需要本地数据集，通过 PHAD_ENABLE_MH01_TESTS 启用，并由
 * PHAD_EUROC_MH01_PATH 指定序列根目录。
 */

namespace
{

  namespace fs = std::filesystem;

  using phad::common::Trajectory;

  [[nodiscard]] fs::path sequenceRoot()
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr )
    {
      return {};
    }
    return fs::path{ configured_path };
  }

  class Mh01GroundtruthTest : public ::testing::Test
  {
  protected:
    void SetUp() override
    {
      m_root = sequenceRoot();
      if ( m_root.empty() )
      {
        GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
      }
      std::random_device random;
      m_export_path =
          fs::temp_directory_path() /
          ( "phad_mh01_gt_" + std::to_string( random() ) + ".tum" );
    }

    void TearDown() override
    {
      std::error_code error;
      fs::remove( m_export_path, error );
    }

    fs::path m_root;
    fs::path m_export_path;
  };

  TEST_F( Mh01GroundtruthTest, LoadsMonotonicGroundtruth )
  {
    auto trajectory = phad::io::dataset::euroc::openGroundtruth( m_root );
    ASSERT_TRUE( trajectory ) << trajectory.error().describe();
    EXPECT_GT( trajectory.value().size(), 30'000U );
    EXPECT_LT( trajectory.value().firstTimestamp().nanoseconds(),
               trajectory.value().lastTimestamp().nanoseconds() );
  }

  TEST_F( Mh01GroundtruthTest, ExportedGroundtruthComparesToItselfWithZeroAte )
  {
    auto trajectory = phad::io::dataset::euroc::openGroundtruth( m_root );
    ASSERT_TRUE( trajectory ) << trajectory.error().describe();
    const Trajectory groundtruth = std::move( trajectory ).value();

    ASSERT_FALSE(
        phad::eval::writeTum( m_export_path, groundtruth ).has_value() );
    auto restored = phad::eval::readTum( m_export_path );
    ASSERT_TRUE( restored ) << restored.error().describe();
    ASSERT_EQ( restored.value().size(), groundtruth.size() );

    const auto report = phad::eval::computeAte( restored.value(), groundtruth );
    ASSERT_TRUE( report ) << report.error().describe();
    EXPECT_EQ( report.value().association.pairs.size(), groundtruth.size() );
    EXPECT_EQ( report.value().association.droppedTotal(), 0U );
    EXPECT_NEAR( report.value().trans_m.rmse, 0.0, 1e-9 );
    EXPECT_NEAR( report.value().rot_deg.rmse, 0.0, 1e-6 );
  }

}  // namespace
