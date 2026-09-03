#include "apps/internal/vo_bench_dataset.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{

  namespace fs = std::filesystem;

  using phad::apps::internal::openVoBenchDataset;
  using phad::apps::internal::openVoBenchGroundtruth;
  using phad::apps::internal::voBenchDatasetConfigToken;
  using phad::apps::internal::VoBenchDatasetFormat;
  using phad::io::dataset::DatasetErrorCode;

  class VoBenchDatasetFixture
  {
  public:
    VoBenchDatasetFixture()
    {
      const auto nonce =
          std::chrono::steady_clock::now().time_since_epoch().count();
      m_root = fs::temp_directory_path() /
               ( "phad-vo-bench-dataset-test-" + std::to_string( nonce ) );

      for ( const auto* sensor : { "cam0", "cam1", "imu0" } )
      {
        const auto sensor_root = m_root / "mav0" / sensor;
        fs::create_directories( sensor_root );
        std::ofstream( sensor_root / "data.csv" );
      }
      fs::create_directories( m_root / "mav0" / "cam0" / "data" );
      fs::create_directories( m_root / "mav0" / "cam1" / "data" );

      const auto euroc_gt =
          m_root / "mav0" / "state_groundtruth_estimate0";
      fs::create_directories( euroc_gt );
      std::ofstream( euroc_gt / "sensor.yaml" )
          << "sensor_type: visual-inertial\n"
             "T_BS:\n"
             "  rows: 4\n"
             "  cols: 4\n"
             "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n";
      std::ofstream( euroc_gt / "data.csv" ) << "not a EuRoC header\n";

      fs::create_directories( m_root / "dso" );
      std::ofstream( m_root / "dso" / "gt_imu.csv" )
          << "not a TUM-VI header\n";
    }

    ~VoBenchDatasetFixture()
    {
      std::error_code error;
      fs::remove_all( m_root, error );
    }

    VoBenchDatasetFixture( const VoBenchDatasetFixture& )            = delete;
    VoBenchDatasetFixture& operator=( const VoBenchDatasetFixture& ) = delete;

    [[nodiscard]] const fs::path& root() const noexcept { return m_root; }

  private:
    fs::path m_root;
  };

  TEST( VoBenchDatasetTest, ConfigTokensAreStable )
  {
    EXPECT_EQ( voBenchDatasetConfigToken( VoBenchDatasetFormat::kEuroc ),
               "euroc" );
    EXPECT_EQ( voBenchDatasetConfigToken( VoBenchDatasetFormat::kTumVi ),
               "tum_vi" );
  }

  TEST( VoBenchDatasetTest, InputDispatchUsesSelectedAdapter )
  {
    const VoBenchDatasetFixture fixture;

    const auto euroc =
        openVoBenchDataset( VoBenchDatasetFormat::kEuroc, fixture.root() );
    ASSERT_FALSE( euroc );
    EXPECT_EQ( euroc.error().code,
               DatasetErrorCode::kRequiredFileMissing );
    EXPECT_EQ( euroc.error().sensor_id, "cam0" );
    EXPECT_EQ( euroc.error().source_path,
               fixture.root() / "mav0" / "cam0" / "sensor.yaml" );

    const auto tum_vi =
        openVoBenchDataset( VoBenchDatasetFormat::kTumVi, fixture.root() );
    ASSERT_FALSE( tum_vi );
    EXPECT_EQ( tum_vi.error().code,
               DatasetErrorCode::kRequiredFileMissing );
    EXPECT_EQ( tum_vi.error().source_path,
               fixture.root() / "dso" / "camchain.yaml" );
  }

  TEST( VoBenchDatasetTest, GroundtruthDispatchUsesSelectedNativeFormat )
  {
    const VoBenchDatasetFixture fixture;

    const auto euroc = openVoBenchGroundtruth(
        VoBenchDatasetFormat::kEuroc, fixture.root() );
    ASSERT_FALSE( euroc );
    EXPECT_EQ( euroc.error().code, DatasetErrorCode::kInvalidCsvHeader );
    EXPECT_EQ( euroc.error().sensor_id, "state_groundtruth_estimate0" );
    EXPECT_EQ( euroc.error().field, "header" );
    EXPECT_EQ( euroc.error().source_path,
               fixture.root() / "mav0" / "state_groundtruth_estimate0" /
                   "data.csv" );

    const auto tum_vi = openVoBenchGroundtruth(
        VoBenchDatasetFormat::kTumVi, fixture.root() );
    ASSERT_FALSE( tum_vi );
    EXPECT_EQ( tum_vi.error().code, DatasetErrorCode::kInvalidCsvHeader );
    EXPECT_EQ( tum_vi.error().sensor_id, "gt_imu" );
    EXPECT_EQ( tum_vi.error().field, "header" );
    EXPECT_EQ( tum_vi.error().source_path,
               fixture.root() / "dso" / "gt_imu.csv" );
  }

}  // namespace
