#include "phad/io/dataset/tum_vi/tum_vi_dataset.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <string>

namespace
{

  namespace fs = std::filesystem;

  class TumViFixture
  {
  public:
    static constexpr std::int64_t kFirstTimestamp =
        1'520'531'829'251'142'058;

    TumViFixture()
    {
      const auto nonce =
          std::chrono::steady_clock::now().time_since_epoch().count();
      m_root = fs::temp_directory_path() /
               ( "phad-tum-vi-test-" + std::to_string( nonce ) );
      for ( const auto* sensor_id : { "cam0", "cam1" } )
      {
        fs::create_directories( m_root / "mav0" / sensor_id / "data" );
      }
      fs::create_directories( m_root / "mav0" / "imu0" );
      fs::create_directories( m_root / "dso" );
      writeCamchain( "equidistant", false );
      writeImuConfig();
      writeDefaultData();
    }

    ~TumViFixture() { fs::remove_all( m_root ); }

    TumViFixture( const TumViFixture& )            = delete;
    TumViFixture& operator=( const TumViFixture& ) = delete;

    [[nodiscard]] const fs::path& root() const noexcept { return m_root; }

    void writeCamchain( const std::string& distortion_model,
                        bool               invalid_bottom_row )
    {
      std::ofstream stream( m_root / "dso" / "camchain.yaml",
                            std::ios::trunc );
      const double  bottom_right = invalid_bottom_row ? 2.0 : 1.0;
      for ( const auto& [ sensor_id, translation_x ] :
            std::array<std::pair<std::string, double>, 2>{
                std::pair{ "cam0", 0.1 }, std::pair{ "cam1", -0.1 } } )
      {
        stream << sensor_id << ":\n"
               << "  T_cam_imu:\n"
               << "  - [1.0, 0.0, 0.0, " << translation_x << "]\n"
               << "  - [0.0, 1.0, 0.0, -0.2]\n"
               << "  - [0.0, 0.0, 1.0, 0.3]\n"
               << "  - [0.0, 0.0, 0.0, " << bottom_right << "]\n"
               << "  camera_model: pinhole\n"
               << "  distortion_coeffs: [0.01, 0.02, 0.03, 0.04]\n"
               << "  distortion_model: " << distortion_model << '\n'
               << "  intrinsics: [190.0, 191.0, 255.0, 256.0]\n"
               << "  resolution: [4, 3]\n";
      }
    }

    void writeCameraCsv( const std::string& sensor_id,
                         const std::string& contents )
    {
      std::ofstream stream( m_root / "mav0" / sensor_id / "data.csv",
                            std::ios::trunc );
      stream << contents;
    }

    void writeImage( const std::string& sensor_id,
                     const std::string& filename, std::uint16_t value,
                     int type = CV_16UC1 )
    {
      cv::Mat image( 3, 4, type, cv::Scalar( value ) );
      ASSERT_TRUE( cv::imwrite(
          ( m_root / "mav0" / sensor_id / "data" / filename ).string(),
          image ) );
    }

  private:
    void writeImuConfig()
    {
      std::ofstream stream( m_root / "dso" / "imu_config.yaml" );
      stream << "update_rate: 200.0\n"
             << "accelerometer_noise_density: 0.0028\n"
             << "accelerometer_random_walk: 0.00086\n"
             << "gyroscope_noise_density: 0.00016\n"
             << "gyroscope_random_walk: 0.000022\n";
    }

    void writeDefaultData()
    {
      std::string left_csv  = "#timestamp [ns],filename\n";
      std::string right_csv = "#timestamp [ns],filename\n";
      for ( std::size_t index = 0; index < 3; ++index )
      {
        const auto timestamp =
            kFirstTimestamp + static_cast<std::int64_t>( index ) * 50'000'000;
        const std::string filename = std::to_string( timestamp ) + ".png";
        left_csv += std::to_string( timestamp ) + ',' + filename + '\n';
        right_csv += std::to_string( timestamp ) + ',' + filename + '\n';
        writeImage( "cam0", filename,
                    static_cast<std::uint16_t>( 1000 + index ) );
        writeImage( "cam1", filename,
                    static_cast<std::uint16_t>( 2000 + index ) );
      }
      writeCameraCsv( "cam0", left_csv );
      writeCameraCsv( "cam1", right_csv );

      std::ofstream imu( m_root / "mav0" / "imu0" / "data.csv" );
      imu << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
             "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],"
             "a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
      for ( std::size_t index = 0; index < 4; ++index )
      {
        imu << kFirstTimestamp -
                   5'000'000 +
                   static_cast<std::int64_t>( index ) * 5'000'000
            << ",0.1,0.2,0.3,1.0,2.0,9.8\n";
      }
    }

    fs::path m_root;
  };

  TEST( TumViDatasetTest, OpensNormalizedCalibrationImuAndUint16Stereo )
  {
    TumViFixture fixture;
    auto         opened = phad::io::dataset::tum_vi::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();

    const auto& dataset = opened.value();
    ASSERT_EQ( dataset.stereoIndex().size(), 3U );
    ASSERT_EQ( dataset.imuMeasurements().size(), 4U );
    const auto& calibration = dataset.calibration();
    EXPECT_EQ( calibration.left.distortion_model,
               phad::sensor::DistortionModel::kEquidistant );
    EXPECT_DOUBLE_EQ( calibration.left.rate_hz, 20.0 );
    EXPECT_DOUBLE_EQ( calibration.imu.rate_hz, 200.0 );
    EXPECT_DOUBLE_EQ( calibration.imu.acc_nd, 0.0028 );
    EXPECT_DOUBLE_EQ( calibration.imu.gyr_rw, 0.000022 );
    EXPECT_DOUBLE_EQ( calibration.left.T_B_camera.matrix[ 3 ], -0.1 );
    EXPECT_DOUBLE_EQ( calibration.left.T_B_camera.matrix[ 7 ], 0.2 );
    EXPECT_DOUBLE_EQ( calibration.left.T_B_camera.matrix[ 11 ], -0.3 );
    EXPECT_DOUBLE_EQ( calibration.imu.T_B_imu.matrix[ 0 ], 1.0 );
    EXPECT_DOUBLE_EQ( calibration.imu.T_B_imu.matrix[ 15 ], 1.0 );

    auto loaded = dataset.loadStereo( 1 );
    ASSERT_TRUE( loaded.hasValue() ) << loaded.error().describe();
    EXPECT_EQ( loaded.value().left.pixelType(),
               phad::sensor::PixelType::kUint16 );
    const auto left_pixels =
        loaded.value().left.pixels<std::uint16_t>();
    const auto right_pixels =
        loaded.value().right.pixels<std::uint16_t>();
    ASSERT_TRUE( left_pixels.has_value() );
    ASSERT_TRUE( right_pixels.has_value() );
    EXPECT_EQ( left_pixels->size(), 12U );
    EXPECT_EQ( left_pixels->front(), 1001U );
    EXPECT_EQ( right_pixels->front(), 2001U );
    EXPECT_FALSE(
        loaded.value().left.pixels<std::uint8_t>().has_value() );
  }

  TEST( TumViDatasetTest, RejectsInvalidKalibrTransformAndDistortion )
  {
    {
      TumViFixture fixture;
      fixture.writeCamchain( "equidistant", true );
      auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
      ASSERT_FALSE( opened.hasValue() );
      EXPECT_EQ( opened.error().code,
                 phad::io::dataset::DatasetErrorCode::kInvalidCalibration );
      EXPECT_EQ( opened.error().field, "T_cam_imu.bottom_row" );
    }
    {
      TumViFixture fixture;
      fixture.writeCamchain( "radtan", false );
      auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
      ASSERT_FALSE( opened.hasValue() );
      EXPECT_EQ(
          opened.error().code,
          phad::io::dataset::DatasetErrorCode::kUnsupportedDistortionModel );
    }
  }

  TEST( TumViDatasetTest, RejectsStereoMismatchAndWrongPixelDepth )
  {
    {
      TumViFixture fixture;
      fixture.writeCameraCsv(
          "cam1",
          "#timestamp [ns],filename\n"
          "1520531829251142059,1520531829251142058.png\n" );
      auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
      ASSERT_FALSE( opened.hasValue() );
      EXPECT_EQ(
          opened.error().code,
          phad::io::dataset::DatasetErrorCode::kStereoTimestampMismatch );
    }
    {
      TumViFixture fixture;
      fixture.writeImage( "cam0", "1520531829301142058.png", 12,
                          CV_8UC1 );
      auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
      ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
      auto loaded = opened.value().loadStereo( 1 );
      ASSERT_FALSE( loaded.hasValue() );
      EXPECT_EQ(
          loaded.error().code,
          phad::io::dataset::DatasetErrorCode::kImageFormatMismatch );
      EXPECT_EQ( loaded.error().field, "pixel_type" );
    }
  }

}  // namespace
