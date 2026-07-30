#include "phad/io/dataset/tum_vi/tum_vi_dataset.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <string>
#include <variant>

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
                     int type = CV_16UC1, int width = 4, int height = 3 )
    {
      cv::Mat image( height, width, type, cv::Scalar( value ) );
      ASSERT_TRUE( cv::imwrite(
          ( m_root / "mav0" / sensor_id / "data" / filename ).string(),
          image ) );
    }

    void writeCorruptImage( const std::string& sensor_id,
                            const std::string& filename )
    {
      std::ofstream stream(
          m_root / "mav0" / sensor_id / "data" / filename,
          std::ios::binary | std::ios::trunc );
      stream << "not a png";
    }

    void replaceInFile( const fs::path& relative, const std::string& from,
                        const std::string& to )
    {
      const fs::path     path = m_root / relative;
      std::ifstream      input( path );
      std::ostringstream contents;
      contents << input.rdbuf();
      std::string text     = contents.str();
      const auto  position = text.find( from );
      ASSERT_NE( position, std::string::npos );
      text.replace( position, from.size(), to );
      std::ofstream( path, std::ios::trunc ) << text;
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

  void expectStickyTerminalError(
      phad::io::dataset::StereoImuDatasetReader&   reader,
      const phad::io::dataset::DatasetReaderError& expected )
  {
    const auto expect_error = [ &expected ]( const auto& result ) {
      ASSERT_TRUE(
          std::holds_alternative<phad::io::dataset::DatasetReaderError>(
              result ) );
      EXPECT_EQ(
          std::get<phad::io::dataset::DatasetReaderError>( result ),
          expected );
    };
    expect_error( reader.takeImu() );
    expect_error( reader.peekStereoTimestamp() );
    expect_error( reader.takeStereo() );
    expect_error( reader.takeStereo() );
  }

  void expectLazyStickyStereoError(
      phad::io::dataset::StereoImuDatasetReader& reader,
      phad::io::dataset::DatasetReaderErrorCode  expected_code,
      const std::string&                         expected_sensor_id,
      std::int64_t expected_timestamp, std::size_t expected_record_number,
      const fs::path& fixture_root )
  {
    const auto peeked = reader.peekStereoTimestamp();
    ASSERT_TRUE(
        std::holds_alternative<phad::common::Timestamp>( peeked ) );
    EXPECT_EQ( std::get<phad::common::Timestamp>( peeked ).nanoseconds(),
               expected_timestamp );

    const auto failed = reader.takeStereo();
    ASSERT_TRUE(
        std::holds_alternative<phad::io::dataset::DatasetReaderError>(
            failed ) );
    const auto error =
        std::get<phad::io::dataset::DatasetReaderError>( failed );
    EXPECT_EQ( error.code, expected_code );
    EXPECT_EQ( error.sensor_id, expected_sensor_id );
    EXPECT_EQ( error.timestamp.nanoseconds(), expected_timestamp );
    EXPECT_EQ( error.record_number, expected_record_number );
    EXPECT_FALSE( error.cause.empty() );
    EXPECT_EQ( error.cause.find( fixture_root.string() ),
               std::string::npos );
    expectStickyTerminalError( reader, error );
  }

  void expectCoreCalibrationCause(
      const phad::io::dataset::DatasetError& error,
      phad::sensor::CalibrationErrorCode     code,
      const std::string&                     canonical_field,
      const std::string&                     detail )
  {
    EXPECT_NE(
        error.cause.find( "core calibration error " +
                          std::to_string( static_cast<int>( code ) ) ),
        std::string::npos );
    EXPECT_NE( error.cause.find( "field=" + canonical_field ),
               std::string::npos );
    ASSERT_FALSE( detail.empty() );
    EXPECT_NE( error.cause.find( ": " + detail ), std::string::npos );
  }

  TEST( TumViDatasetTest, OpensNormalizedCalibrationImuAndUint16Stereo )
  {
    TumViFixture fixture;
    auto         opened = phad::io::dataset::tum_vi::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();

    const auto& dataset     = opened.value();
    const auto  summary     = dataset.summary();
    const auto  calibration = dataset.calibration();
    ASSERT_EQ( summary.imu.count, 4U );
    ASSERT_TRUE( summary.imu.first_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.last_timestamp.has_value() );
    EXPECT_EQ( summary.imu.first_timestamp->nanoseconds(),
               TumViFixture::kFirstTimestamp - 5'000'000 );
    EXPECT_EQ( summary.imu.last_timestamp->nanoseconds(),
               TumViFixture::kFirstTimestamp + 10'000'000 );
    ASSERT_EQ( summary.stereo.count, 3U );
    ASSERT_TRUE( summary.stereo.first_timestamp.has_value() );
    ASSERT_TRUE( summary.stereo.last_timestamp.has_value() );
    EXPECT_EQ( summary.stereo.first_timestamp->nanoseconds(),
               TumViFixture::kFirstTimestamp );
    EXPECT_EQ( summary.stereo.last_timestamp->nanoseconds(),
               TumViFixture::kFirstTimestamp + 100'000'000 );
    EXPECT_TRUE(
        std::holds_alternative<phad::sensor::PinholeEquidistantParameters>(
            calibration.leftCamera().modelParameters() ) );
    EXPECT_DOUBLE_EQ( calibration.leftCamera().rateHz(), 20.0 );
    EXPECT_DOUBLE_EQ( calibration.imu().rateHz(), 200.0 );
    EXPECT_DOUBLE_EQ(
        calibration.imu().accelerometerNoiseDensityMps2PerSqrtHz(), 0.0028 );
    EXPECT_DOUBLE_EQ(
        calibration.imu().gyroscopeBiasRandomWalkRadps2PerSqrtHz(),
        0.000022 );
    const auto translation =
        calibration.T_B_left_camera().translation();
    EXPECT_DOUBLE_EQ( translation.x(), -0.1 );
    EXPECT_DOUBLE_EQ( translation.y(), 0.2 );
    EXPECT_DOUBLE_EQ( translation.z(), -0.3 );

    auto reader = dataset.reader();
    for ( std::size_t index = 0; index < summary.imu.count; ++index )
    {
      const auto taken = reader.takeImu();
      ASSERT_TRUE( std::holds_alternative<phad::sensor::ImuMeasurement>(
          taken ) );
      const auto& measurement =
          std::get<phad::sensor::ImuMeasurement>( taken );
      EXPECT_EQ(
          measurement.timestamp.nanoseconds(),
          TumViFixture::kFirstTimestamp - 5'000'000 +
              static_cast<std::int64_t>( index ) * 5'000'000 );
      EXPECT_EQ( measurement.gyro_radps,
                 ( std::array<double, 3>{ 0.1, 0.2, 0.3 } ) );
      EXPECT_EQ( measurement.accel_mps2,
                 ( std::array<double, 3>{ 1.0, 2.0, 9.8 } ) );
    }
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeImu() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeImu() ) );

    for ( std::size_t index = 0; index < summary.stereo.count; ++index )
    {
      const auto expected_timestamp =
          TumViFixture::kFirstTimestamp +
          static_cast<std::int64_t>( index ) * 50'000'000;
      const auto peeked = reader.peekStereoTimestamp();
      ASSERT_TRUE(
          std::holds_alternative<phad::common::Timestamp>( peeked ) );
      EXPECT_EQ( std::get<phad::common::Timestamp>( peeked ).nanoseconds(),
                 expected_timestamp );

      const auto taken = reader.takeStereo();
      ASSERT_TRUE(
          std::holds_alternative<phad::sensor::StereoFrame>( taken ) );
      const auto& frame = std::get<phad::sensor::StereoFrame>( taken );
      EXPECT_EQ( frame.timestamp.nanoseconds(), expected_timestamp );
      EXPECT_EQ( frame.left.pixelType(), phad::sensor::PixelType::kUint16 );
      EXPECT_EQ( frame.right.pixelType(), phad::sensor::PixelType::kUint16 );
      const auto left_pixels  = frame.left.pixels<std::uint16_t>();
      const auto right_pixels = frame.right.pixels<std::uint16_t>();
      ASSERT_TRUE( left_pixels.has_value() );
      ASSERT_TRUE( right_pixels.has_value() );
      EXPECT_EQ( left_pixels->size(), 12U );
      EXPECT_EQ( left_pixels->front(), 1000U + index );
      EXPECT_EQ( right_pixels->front(), 2000U + index );
      EXPECT_FALSE( frame.left.pixels<std::uint8_t>().has_value() );
      EXPECT_FALSE( frame.right.pixels<std::uint8_t>().has_value() );
    }
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.peekStereoTimestamp() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.peekStereoTimestamp() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeStereo() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeStereo() ) );
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
      expectCoreCalibrationCause(
          opened.error(),
          phad::sensor::CalibrationErrorCode::kInvalidHomogeneousRow,
          "rigid_transform.bottom_row",
          "bottom row must be [0, 0, 0, 1]" );
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

  TEST( TumViDatasetTest, MapsEveryCoreSourceFieldWithProvenance )
  {
    struct MappingCase
    {
      const char*                        relative_path;
      const char*                        sensor_id;
      const char*                        original;
      const char*                        replacement;
      const char*                        raw_field;
      phad::sensor::CalibrationErrorCode code;
      const char*                        canonical_field;
      const char*                        detail;
    };
    const std::array<MappingCase, 18> cases{ {
        { "dso/camchain.yaml", "cam0", "- [1.0, 0.0, 0.0, 0.1]",
          "- [1.0, 0.0, 0.0, .nan]", "T_cam_imu",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "rigid_transform.matrix", "all matrix elements must be finite" },
        { "dso/camchain.yaml", "cam0", "- [1.0, 0.0, 0.0, 0.1]",
          "- [2.0, 0.0, 0.0, 0.1]", "T_cam_imu.rotation",
          phad::sensor::CalibrationErrorCode::kInvalidRotation,
          "rigid_transform.rotation",
          "rotation must be orthogonal with determinant +1" },
        { "dso/camchain.yaml", "cam0", "- [0.0, 0.0, 0.0, 1]",
          "- [0.0, 0.0, 0.0, 2]", "T_cam_imu.bottom_row",
          phad::sensor::CalibrationErrorCode::kInvalidHomogeneousRow,
          "rigid_transform.bottom_row",
          "bottom row must be [0, 0, 0, 1]" },
        { "dso/camchain.yaml", "cam0",
          "intrinsics: [190.0, 191.0, 255.0, 256.0]",
          "intrinsics: [0.0, 191.0, 255.0, 256.0]", "intrinsics",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.equidistant.fx_pixels",
          "fx_pixels must be strictly positive" },
        { "dso/camchain.yaml", "cam0",
          "intrinsics: [190.0, 191.0, 255.0, 256.0]",
          "intrinsics: [190.0, 0.0, 255.0, 256.0]", "intrinsics",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.equidistant.fy_pixels",
          "fy_pixels must be strictly positive" },
        { "dso/camchain.yaml", "cam0",
          "intrinsics: [190.0, 191.0, 255.0, 256.0]",
          "intrinsics: [190.0, 191.0, .nan, 256.0]", "intrinsics",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.equidistant.cx_pixels",
          "camera model parameter must be finite" },
        { "dso/camchain.yaml", "cam0",
          "intrinsics: [190.0, 191.0, 255.0, 256.0]",
          "intrinsics: [190.0, 191.0, 255.0, .nan]", "intrinsics",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.equidistant.cy_pixels",
          "camera model parameter must be finite" },
        { "dso/camchain.yaml", "cam0",
          "distortion_coeffs: [0.01, 0.02, 0.03, 0.04]",
          "distortion_coeffs: [.nan, 0.02, 0.03, 0.04]",
          "distortion_coeffs",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.equidistant.k1",
          "camera model parameter must be finite" },
        { "dso/camchain.yaml", "cam0",
          "distortion_coeffs: [0.01, 0.02, 0.03, 0.04]",
          "distortion_coeffs: [0.01, .nan, 0.03, 0.04]",
          "distortion_coeffs",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.equidistant.k2",
          "camera model parameter must be finite" },
        { "dso/camchain.yaml", "cam0",
          "distortion_coeffs: [0.01, 0.02, 0.03, 0.04]",
          "distortion_coeffs: [0.01, 0.02, .nan, 0.04]",
          "distortion_coeffs",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.equidistant.k3",
          "camera model parameter must be finite" },
        { "dso/camchain.yaml", "cam0",
          "distortion_coeffs: [0.01, 0.02, 0.03, 0.04]",
          "distortion_coeffs: [0.01, 0.02, 0.03, .nan]",
          "distortion_coeffs",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.equidistant.k4",
          "camera model parameter must be finite" },
        { "dso/camchain.yaml", "cam0", "resolution: [4, 3]",
          "resolution: [0, 3]", "resolution",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "camera.image_width", "image width must be positive" },
        { "dso/camchain.yaml", "cam0", "resolution: [4, 3]",
          "resolution: [4, 0]", "resolution",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "camera.image_height", "image height must be positive" },
        { "dso/imu_config.yaml", "imu0", "update_rate: 200.0",
          "update_rate: 0.0", "update_rate",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.rate_hz", "IMU parameter must be strictly positive" },
        { "dso/imu_config.yaml", "imu0",
          "accelerometer_noise_density: 0.0028",
          "accelerometer_noise_density: 0", "accelerometer_noise_density",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.accelerometer_noise_density_mps2_per_sqrt_hz",
          "IMU parameter must be strictly positive" },
        { "dso/imu_config.yaml", "imu0",
          "gyroscope_noise_density: 0.00016",
          "gyroscope_noise_density: 0", "gyroscope_noise_density",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.gyroscope_noise_density_radps_per_sqrt_hz",
          "IMU parameter must be strictly positive" },
        { "dso/imu_config.yaml", "imu0",
          "accelerometer_random_walk: 0.00086",
          "accelerometer_random_walk: 0", "accelerometer_random_walk",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.accelerometer_bias_random_walk_mps3_per_sqrt_hz",
          "IMU parameter must be strictly positive" },
        { "dso/imu_config.yaml", "imu0",
          "gyroscope_random_walk: 0.000022",
          "gyroscope_random_walk: 0", "gyroscope_random_walk",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.gyroscope_bias_random_walk_radps2_per_sqrt_hz",
          "IMU parameter must be strictly positive" },
    } };

    for ( const auto& test_case : cases )
    {
      SCOPED_TRACE( test_case.canonical_field );
      TumViFixture fixture;
      fixture.replaceInFile( test_case.relative_path, test_case.original,
                             test_case.replacement );
      auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
      ASSERT_FALSE( opened );
      EXPECT_EQ( opened.error().code,
                 phad::io::dataset::DatasetErrorCode::kInvalidCalibration );
      EXPECT_EQ( opened.error().sensor_id, test_case.sensor_id );
      EXPECT_EQ( opened.error().source_path,
                 fixture.root() / test_case.relative_path );
      EXPECT_EQ( opened.error().field, test_case.raw_field );
      expectCoreCalibrationCause(
          opened.error(), test_case.code, test_case.canonical_field,
          test_case.detail );
    }
  }

  TEST( TumViDatasetTest, MapsZeroBaselineWithJointProvenance )
  {
    TumViFixture fixture;
    fixture.replaceInFile( "dso/camchain.yaml",
                           "- [1.0, 0.0, 0.0, -0.1]",
                           "- [1.0, 0.0, 0.0, 0.1]" );
    auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
    ASSERT_FALSE( opened );
    EXPECT_EQ( opened.error().sensor_id, "cam0/cam1" );
    EXPECT_EQ( opened.error().source_path,
               fixture.root() / "dso/camchain.yaml" );
    EXPECT_EQ( opened.error().field,
               "cam0.T_cam_imu/cam1.T_cam_imu" );
    expectCoreCalibrationCause(
        opened.error(),
        phad::sensor::CalibrationErrorCode::kZeroStereoBaseline,
        "stereo_imu_calibration.camera_centers",
        "left and right camera centers must not coincide" );
  }

  TEST( TumViDatasetTest, PreservesUnsupportedModelCodesBeforeFactories )
  {
    {
      TumViFixture fixture;
      fixture.replaceInFile( "dso/camchain.yaml", "camera_model: pinhole",
                             "camera_model: omnidirectional" );
      auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
      ASSERT_FALSE( opened );
      EXPECT_EQ(
          opened.error().code,
          phad::io::dataset::DatasetErrorCode::kUnsupportedCameraModel );
    }
    {
      TumViFixture fixture;
      fixture.writeCamchain( "radtan", false );
      auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
      ASSERT_FALSE( opened );
      EXPECT_EQ(
          opened.error().code,
          phad::io::dataset::DatasetErrorCode::kUnsupportedDistortionModel );
    }
  }

  TEST( TumViDatasetTest, RejectsStereoMismatch )
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

  TEST( TumViDatasetTest, WrongPixelDepthFailsLazilyAndIsSticky )
  {
    TumViFixture fixture;
    fixture.writeImage( "cam0", "1520531829301142058.png", 12,
                        CV_8UC1 );
    auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    EXPECT_EQ( opened.value().summary().stereo.count, 3U );

    auto reader = opened.value().reader();
    EXPECT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
        reader.takeStereo() ) );
    expectLazyStickyStereoError(
        reader,
        phad::io::dataset::DatasetReaderErrorCode::kImageFormatMismatch,
        "left_camera", TumViFixture::kFirstTimestamp + 50'000'000, 2U,
        fixture.root() );
  }

  TEST( TumViDatasetTest, CorruptImageFailsLazilyAndIsSticky )
  {
    TumViFixture fixture;
    fixture.writeCorruptImage( "cam1", "1520531829301142058.png" );
    auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    EXPECT_EQ( opened.value().summary().stereo.count, 3U );

    auto reader = opened.value().reader();
    EXPECT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
        reader.takeStereo() ) );
    expectLazyStickyStereoError(
        reader, phad::io::dataset::DatasetReaderErrorCode::kImageDecodeFailed,
        "right_camera", TumViFixture::kFirstTimestamp + 50'000'000, 2U,
        fixture.root() );
  }

  TEST( TumViDatasetTest, WrongImageDimensionsFailLazilyAndAreSticky )
  {
    TumViFixture fixture;
    fixture.writeImage( "cam0", "1520531829251142058.png", 12,
                        CV_16UC1, 5, 3 );
    auto opened = phad::io::dataset::tum_vi::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    EXPECT_EQ( opened.value().summary().stereo.count, 3U );

    auto reader = opened.value().reader();
    expectLazyStickyStereoError(
        reader,
        phad::io::dataset::DatasetReaderErrorCode::kImageFormatMismatch,
        "left_camera", TumViFixture::kFirstTimestamp, 1U, fixture.root() );
  }

}  // namespace
