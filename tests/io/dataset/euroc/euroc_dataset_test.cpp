#include "phad/io/dataset/euroc/euroc_dataset.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>

#ifdef __linux__
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace
{

  namespace fs = std::filesystem;

  class EurocFixture
  {
  public:
    EurocFixture()
    {
      m_root = makeUniqueRoot();
      for ( const auto* sensor : { "cam0", "cam1", "imu0" } )
      {
        fs::create_directories( m_root / "mav0" / sensor / "data" );
      }
      writeDefaultCalibrations();
      writeDefaultCsvFiles();
      writeImage( "cam0", "left-a.png", 11 );
      writeImage( "cam0", "left-b.png", 12 );
      writeImage( "cam0", "left-c.png", 13 );
      writeImage( "cam1", "right-a.png", 21 );
      writeImage( "cam1", "right-b.png", 22 );
      writeImage( "cam1", "right-c.png", 23 );
    }

    ~EurocFixture() { fs::remove_all( m_root ); }

    EurocFixture( const EurocFixture& )            = delete;
    EurocFixture& operator=( const EurocFixture& ) = delete;

    [[nodiscard]] const fs::path& root() const { return m_root; }
    [[nodiscard]] fs::path        sensorPath( const std::string& sensor,
                                              const std::string& relative ) const
    {
      return m_root / "mav0" / sensor / relative;
    }

    void writeSensorFile( const std::string& sensor, const std::string& contents )
    {
      std::ofstream( m_root / "mav0" / sensor / "sensor.yaml" ) << contents;
    }

    void writeCsv( const std::string& sensor, const std::string& contents )
    {
      std::ofstream( m_root / "mav0" / sensor / "data.csv" ) << contents;
    }

    void writeImage( const std::string& sensor, const std::string& filename,
                     std::uint8_t value, int width = 4, int height = 3,
                     int type = CV_8UC1 )
    {
      cv::Mat image( height, width, type, cv::Scalar( value, value, value ) );
      ASSERT_TRUE(
          cv::imwrite( ( m_root / "mav0" / sensor / "data" / filename ).string(), image ) );
    }

    static constexpr std::int64_t kFirstTimestamp =
        1'403'636'579'763'555'584;

    static std::string identityTransformYaml()
    {
      return "T_BS:\n"
             "  rows: 4\n"
             "  cols: 4\n"
             "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n";
    }

    static std::string validCameraYaml(
        const std::string& camera_model     = "pinhole",
        const std::string& distortion_model = "radial-tangential",
        int width = 4, int height = 3 )
    {
      return "sensor_type: camera\n" + identityTransformYaml() +
             "rate_hz: 20\n"
             "resolution: [" +
             std::to_string( width ) + ", " +
             std::to_string( height ) +
             "]\n"
             "camera_model: " +
             camera_model +
             "\n"
             "intrinsics: [100, 101, 2, 1.5]\n"
             "distortion_model: " +
             distortion_model +
             "\n"
             "distortion_coefficients: [0.1, -0.2, 0.001, -0.002]\n";
    }

    static std::string validImuYaml()
    {
      return "sensor_type: imu\n" + identityTransformYaml() +
             "rate_hz: 200\n"
             "gyroscope_noise_density: 0.0001\n"
             "gyroscope_random_walk: 0.00001\n"
             "accelerometer_noise_density: 0.002\n"
             "accelerometer_random_walk: 0.003\n";
    }

    static std::string validRightCameraYaml(
        const std::string& camera_model     = "pinhole",
        const std::string& distortion_model = "radial-tangential",
        int width = 4, int height = 3 )
    {
      std::string yaml = validCameraYaml(
          camera_model, distortion_model, width, height );
      const std::string identity = identityTransformYaml();
      const std::string translated =
          "T_BS:\n"
          "  rows: 4\n"
          "  cols: 4\n"
          "  data: [1, 0, 0, 0.1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n";
      yaml.replace( yaml.find( identity ), identity.size(), translated );
      return yaml;
    }

  private:
    static fs::path makeUniqueRoot()
    {
      std::random_device random;
      for ( int attempt = 0; attempt < 100; ++attempt )
      {
        const fs::path candidate =
            fs::temp_directory_path() /
            ( "phad_euroc_test_" + std::to_string( random() ) + "_" +
              std::to_string( random() ) );
        std::error_code error;
        if ( fs::create_directory( candidate, error ) )
        {
          return candidate;
        }
      }
      throw std::runtime_error(
          "failed to create unique EuRoC test directory" );
    }

    void writeDefaultCalibrations()
    {
      writeSensorFile( "cam0", validCameraYaml() );
      writeSensorFile( "cam1", validRightCameraYaml() );
      writeSensorFile( "imu0", validImuYaml() );
    }

    void writeDefaultCsvFiles()
    {
      const auto t0 = std::to_string( kFirstTimestamp );
      const auto t1 = std::to_string( kFirstTimestamp + 50'000'000 );
      const auto t2 = std::to_string( kFirstTimestamp + 100'000'000 );
      writeCsv( "cam0", "#timestamp [ns],filename\n" + t0 + ",left-a.png\n" +
                            t1 + ",left-b.png\n" + t2 + ",left-c.png\n" );
      writeCsv( "cam1", "#timestamp [ns],filename\n" + t0 + ",right-a.png\n" +
                            t1 + ",right-b.png\n" + t2 + ",right-c.png\n" );
      writeCsv(
          "imu0",
          "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
          "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
          "a_RS_S_z [m s^-2]\n" +
              std::to_string( kFirstTimestamp - 5'000'000 ) +
              ",0.1,0.2,0.3,1.1,1.2,9.8\n" + t0 +
              ",0.4,0.5,0.6,1.4,1.5,9.7\n" +
              std::to_string( kFirstTimestamp + 5'000'000 ) +
              ",0.7,0.8,0.9,1.7,1.8,9.6\n" +
              std::to_string( kFirstTimestamp + 10'000'000 ) +
              ",1.0,1.1,1.2,2.0,2.1,9.5\n" );
    }

    fs::path m_root;
  };

  std::string readFile( const fs::path& path )
  {
    std::ifstream      input( path );
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
  }

  template <typename T>
  void expectReaderError(
      const phad::io::dataset::DatasetReaderResult<T>& result,
      const phad::io::dataset::DatasetReaderError&     expected )
  {
    ASSERT_TRUE(
        std::holds_alternative<phad::io::dataset::DatasetReaderError>(
            result ) );
    EXPECT_EQ( std::get<phad::io::dataset::DatasetReaderError>( result ),
               expected );
  }

  void expectStickyTerminalError(
      phad::io::dataset::StereoImuDatasetReader&   reader,
      const phad::io::dataset::DatasetReaderError& expected )
  {
    expectReaderError( reader.takeImu(), expected );
    expectReaderError( reader.peekStereoTimestamp(), expected );
    expectReaderError( reader.takeStereo(), expected );
  }

  TEST( EurocInspectTest, PrintsCalibrationAndSummaryWithoutDecodingImages )
  {
    EurocFixture fixture;
    {
      std::ofstream corrupt(
          fixture.sensorPath( "cam0", "data/left-a.png" ),
          std::ios::binary | std::ios::trunc );
      corrupt << "not a png";
    }
    const fs::path    stdout_path = fixture.root() / "inspect.stdout";
    const fs::path    stderr_path = fixture.root() / "inspect.stderr";
    const std::string command =
        std::string{ "\"" PHAD_EUROC_INSPECT_PATH "\" \"" } +
        fixture.root().string() + "\" > \"" + stdout_path.string() +
        "\" 2> \"" + stderr_path.string() + "\"";

    ASSERT_EQ( std::system( command.c_str() ), 0 ) << readFile( stderr_path );
    const std::string expected_output =
        "left_camera_model: pinhole\n"
        "left_camera_distortion_model: radial-tangential\n"
        "left_camera_resolution: 4x3\n"
        "left_camera_rate_hz: 20\n"
        "left_camera_intrinsics: [100, 101, 2, 1.5]\n"
        "left_camera_distortion_coefficients: [0.1, -0.2, 0.001, -0.002]\n"
        "left_camera_T_B_camera: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, "
        "0, 0, 0, 1]\n"
        "right_camera_model: pinhole\n"
        "right_camera_distortion_model: radial-tangential\n"
        "right_camera_resolution: 4x3\n"
        "right_camera_rate_hz: 20\n"
        "right_camera_intrinsics: [100, 101, 2, 1.5]\n"
        "right_camera_distortion_coefficients: [0.1, -0.2, 0.001, "
        "-0.002]\n"
        "right_camera_T_B_camera: [1, 0, 0, 0.1, 0, 1, 0, 0, 0, 0, 1, "
        "0, 0, 0, 0, 1]\n"
        "imu_T_B_imu: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n"
        "imu_rate_hz: 200\n"
        "imu_acc_nd: 0.002\n"
        "imu_gyr_nd: 0.0001\n"
        "imu_acc_rw: 0.003\n"
        "imu_gyr_rw: 1e-05\n"
        "stereo_frames: 3\n"
        "imu_measurements: 4\n"
        "stereo_first_ns: 1403636579763555584\n"
        "stereo_last_ns: 1403636579863555584\n"
        "imu_first_ns: 1403636579758555584\n"
        "imu_last_ns: 1403636579773555584\n";
    EXPECT_EQ( readFile( stdout_path ), expected_output );
    EXPECT_TRUE( readFile( stderr_path ).empty() );
  }

  TEST( EurocInspectTest, ReportsExactOpenErrorAndReturnsOne )
  {
    EurocFixture      fixture;
    const fs::path    missing_path = fixture.root() / "missing";
    const fs::path    stdout_path  = fixture.root() / "inspect.stdout";
    const fs::path    stderr_path  = fixture.root() / "inspect.stderr";
    const std::string command =
        std::string{ "\"" PHAD_EUROC_INSPECT_PATH "\" \"" } +
        missing_path.string() + "\" > \"" + stdout_path.string() +
        "\" 2> \"" + stderr_path.string() + "\"";

    const int status = std::system( command.c_str() );
#ifdef __linux__
    ASSERT_TRUE( WIFEXITED( status ) );
    EXPECT_EQ( WEXITSTATUS( status ), 1 );
#else
    EXPECT_EQ( status, 1 );
#endif
    EXPECT_TRUE( readFile( stdout_path ).empty() );
    EXPECT_EQ( readFile( stderr_path ),
               "dataset error 0, path=" + missing_path.string() +
                   ": No such file or directory\n" );
  }

  TEST( EurocAdapterTest,
        AdapterOpensCalibrationSummaryAndSequentialMeasurements )
  {
    EurocFixture fixture;
    auto         result = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( result.hasValue() ) << result.error().describe();
    const auto calibration = result.value().calibration();
    const auto summary     = result.value().summary();
    EXPECT_EQ( calibration.leftCamera().imageWidth(), 4U );
    EXPECT_EQ( calibration.leftCamera().imageHeight(), 3U );
    const auto& left_model =
        std::get<phad::sensor::PinholeRadialTangentialParameters>(
            calibration.leftCamera().modelParameters() );
    EXPECT_DOUBLE_EQ( left_model.fxPixels(), 100.0 );
    EXPECT_DOUBLE_EQ( calibration.T_B_left_camera().rotation()( 0, 0 ), 1.0 );
    EXPECT_DOUBLE_EQ( calibration.imu().rateHz(), 200.0 );
    EXPECT_DOUBLE_EQ(
        calibration.imu().gyroscopeNoiseDensityRadpsPerSqrtHz(), 0.0001 );
    EXPECT_DOUBLE_EQ(
        calibration.imu().accelerometerBiasRandomWalkMps3PerSqrtHz(), 0.003 );
    EXPECT_EQ( summary.imu.count, 4U );
    EXPECT_EQ( summary.stereo.count, 3U );

    struct ExpectedImu
    {
      std::int64_t timestamp;
      double       gyro_z;
      double       accel_z;
    };
    const std::array<ExpectedImu, 4> expected_imu{
        ExpectedImu{ EurocFixture::kFirstTimestamp - 5'000'000, 0.3, 9.8 },
        ExpectedImu{ EurocFixture::kFirstTimestamp, 0.6, 9.7 },
        ExpectedImu{ EurocFixture::kFirstTimestamp + 5'000'000, 0.9, 9.6 },
        ExpectedImu{ EurocFixture::kFirstTimestamp + 10'000'000, 1.2, 9.5 } };
    auto                                   reader = result.value().reader();
    std::optional<phad::common::Timestamp> previous_imu_timestamp;
    for ( const auto& expected : expected_imu )
    {
      const auto imu_result = reader.takeImu();
      ASSERT_TRUE( std::holds_alternative<phad::sensor::ImuMeasurement>(
          imu_result ) );
      const auto& measurement =
          std::get<phad::sensor::ImuMeasurement>( imu_result );
      EXPECT_EQ( measurement.timestamp.nanoseconds(), expected.timestamp );
      EXPECT_DOUBLE_EQ( measurement.gyro_radps[ 2 ], expected.gyro_z );
      EXPECT_DOUBLE_EQ( measurement.accel_mps2[ 2 ], expected.accel_z );
      if ( previous_imu_timestamp.has_value() )
      {
        EXPECT_LT( *previous_imu_timestamp, measurement.timestamp );
      }
      previous_imu_timestamp = measurement.timestamp;
    }
    const std::array<std::array<std::int64_t, 3>, 3> expected_stereo{
        std::array<std::int64_t, 3>{
            EurocFixture::kFirstTimestamp, 11, 21 },
        std::array<std::int64_t, 3>{
            EurocFixture::kFirstTimestamp + 50'000'000, 12, 22 },
        std::array<std::int64_t, 3>{
            EurocFixture::kFirstTimestamp + 100'000'000, 13, 23 } };
    std::optional<phad::common::Timestamp> previous_stereo_timestamp;
    for ( const auto& expected : expected_stereo )
    {
      const auto stereo_result = reader.takeStereo();
      ASSERT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
          stereo_result ) );
      const auto& frame =
          std::get<phad::sensor::StereoFrame>( stereo_result );
      EXPECT_EQ( frame.timestamp.nanoseconds(), expected[ 0 ] );
      if ( previous_stereo_timestamp.has_value() )
      {
        EXPECT_LT( *previous_stereo_timestamp, frame.timestamp );
      }
      previous_stereo_timestamp = frame.timestamp;
      EXPECT_EQ( frame.left.pixelType(), phad::sensor::PixelType::kUint8 );
      EXPECT_EQ( frame.right.pixelType(), phad::sensor::PixelType::kUint8 );
      const auto left_pixels  = frame.left.pixels<std::uint8_t>();
      const auto right_pixels = frame.right.pixels<std::uint8_t>();
      ASSERT_TRUE( left_pixels.has_value() );
      ASSERT_TRUE( right_pixels.has_value() );
      EXPECT_EQ( left_pixels->front(), expected[ 1 ] );
      EXPECT_EQ( right_pixels->front(), expected[ 2 ] );
    }
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeImu() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeImu() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.peekStereoTimestamp() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeStereo() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeStereo() ) );
  }

  TEST( EurocAdapterTest, CopiedHandleReturnsCalibrationAndSummaryByValue )
  {
    EurocFixture fixture;
    auto         opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();

    const auto dataset = opened.value();
    const auto copy    = dataset;
    static_assert( std::is_same_v<decltype( dataset.calibration() ),
                                  phad::sensor::StereoImuCalibration> );

    const auto calibration = copy.calibration();
    const auto summary     = copy.summary();
    EXPECT_EQ( calibration.leftCamera().imageWidth(), 4U );
    EXPECT_EQ( summary.imu.count, 4U );
    ASSERT_TRUE( summary.imu.first_timestamp.has_value() );
    ASSERT_TRUE( summary.imu.last_timestamp.has_value() );
    EXPECT_EQ( summary.imu.first_timestamp->nanoseconds(),
               EurocFixture::kFirstTimestamp - 5'000'000 );
    EXPECT_EQ( summary.imu.last_timestamp->nanoseconds(),
               EurocFixture::kFirstTimestamp + 10'000'000 );
    EXPECT_EQ( summary.stereo.count, 3U );
    ASSERT_TRUE( summary.stereo.first_timestamp.has_value() );
    ASSERT_TRUE( summary.stereo.last_timestamp.has_value() );
    EXPECT_EQ( summary.stereo.first_timestamp->nanoseconds(),
               EurocFixture::kFirstTimestamp );
    EXPECT_EQ( summary.stereo.last_timestamp->nanoseconds(),
               EurocFixture::kFirstTimestamp + 100'000'000 );
  }

  TEST( EurocAdapterTest, MoveOnlyReaderOutlivesDatasetHandle )
  {
    EurocFixture fixture;
    auto         reader = [ &fixture ] {
      auto opened = phad::io::dataset::euroc::open( fixture.root() );
      EXPECT_TRUE( opened.hasValue() ) << opened.error().describe();
      return opened.value().reader();
    }();

    static_assert(
        !std::is_copy_constructible_v<decltype( reader )> );
    static_assert( std::is_move_constructible_v<decltype( reader )> );
    const auto first = reader.takeImu();
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::ImuMeasurement>( first ) );
    EXPECT_EQ( std::get<phad::sensor::ImuMeasurement>( first )
                   .timestamp.nanoseconds(),
               EurocFixture::kFirstTimestamp - 5'000'000 );
  }

  TEST( EurocAdapterTest, RepeatedStereoPeekDoesNotDecodeOrAdvance )
  {
    EurocFixture fixture;
    auto         opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    {
      std::ofstream corrupt(
          fixture.sensorPath( "cam0", "data/left-a.png" ),
          std::ios::binary | std::ios::trunc );
      corrupt << "not a png";
    }
    auto reader = opened.value().reader();

    const auto first  = reader.peekStereoTimestamp();
    const auto second = reader.peekStereoTimestamp();
    ASSERT_TRUE( std::holds_alternative<phad::common::Timestamp>( first ) );
    ASSERT_TRUE( std::holds_alternative<phad::common::Timestamp>( second ) );
    EXPECT_EQ( std::get<phad::common::Timestamp>( first ).nanoseconds(),
               EurocFixture::kFirstTimestamp );
    EXPECT_EQ( std::get<phad::common::Timestamp>( second ),
               std::get<phad::common::Timestamp>( first ) );
  }

  TEST( EurocAdapterTest, TakeStereoDecodesOwnedFrameThenAdvances )
  {
    EurocFixture fixture;
    auto         opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    auto reader = opened.value().reader();

    auto taken = reader.takeStereo();
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::StereoFrame>( taken ) );
    const auto& frame = std::get<phad::sensor::StereoFrame>( taken );
    EXPECT_EQ( frame.timestamp.nanoseconds(),
               EurocFixture::kFirstTimestamp );
    EXPECT_EQ( frame.left.pixelType(), phad::sensor::PixelType::kUint8 );
    const auto left_pixels = frame.left.pixels<std::uint8_t>();
    ASSERT_TRUE( left_pixels.has_value() );
    EXPECT_EQ( left_pixels->front(), 11 );

    const auto next = reader.peekStereoTimestamp();
    ASSERT_TRUE( std::holds_alternative<phad::common::Timestamp>( next ) );
    EXPECT_EQ( std::get<phad::common::Timestamp>( next ).nanoseconds(),
               EurocFixture::kFirstTimestamp + 50'000'000 );
  }

  TEST( EurocAdapterTest, ReadersHaveIndependentCursorsAndDecodedImages )
  {
    EurocFixture fixture;
    auto         opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    auto first_reader  = opened.value().reader();
    auto second_reader = opened.value().reader();

    const auto first_imu  = first_reader.takeImu();
    const auto second_imu = second_reader.takeImu();
    ASSERT_TRUE( std::holds_alternative<phad::sensor::ImuMeasurement>(
        first_imu ) );
    ASSERT_TRUE( std::holds_alternative<phad::sensor::ImuMeasurement>(
        second_imu ) );
    EXPECT_EQ( std::get<phad::sensor::ImuMeasurement>( first_imu ).timestamp,
               std::get<phad::sensor::ImuMeasurement>( second_imu ).timestamp );
    static_cast<void>( first_reader.takeImu() );
    const auto second_reader_next = second_reader.takeImu();
    EXPECT_EQ(
        std::get<phad::sensor::ImuMeasurement>( second_reader_next )
            .timestamp.nanoseconds(),
        EurocFixture::kFirstTimestamp );

    auto first_stereo  = first_reader.takeStereo();
    auto second_stereo = second_reader.takeStereo();
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::StereoFrame>( first_stereo ) );
    ASSERT_TRUE(
        std::holds_alternative<phad::sensor::StereoFrame>( second_stereo ) );
    const auto first_pixels =
        std::get<phad::sensor::StereoFrame>( first_stereo )
            .left.pixels<std::uint8_t>();
    const auto second_pixels =
        std::get<phad::sensor::StereoFrame>( second_stereo )
            .left.pixels<std::uint8_t>();
    ASSERT_TRUE( first_pixels.has_value() );
    ASSERT_TRUE( second_pixels.has_value() );
    EXPECT_EQ( first_pixels->front(), second_pixels->front() );
    EXPECT_NE( first_pixels->data(), second_pixels->data() );
  }

  TEST( EurocAdapterTest, ImuAndStereoStreamsEndIndependentlyAndStably )
  {
    EurocFixture fixture;
    auto         opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    auto reader = opened.value().reader();

    for ( std::size_t index = 0; index < 3; ++index )
    {
      ASSERT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
          reader.takeStereo() ) );
    }
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.peekStereoTimestamp() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeStereo() ) );
    EXPECT_TRUE( std::holds_alternative<phad::sensor::ImuMeasurement>(
        reader.takeImu() ) );

    for ( std::size_t index = 1; index < 4; ++index )
    {
      ASSERT_TRUE( std::holds_alternative<phad::sensor::ImuMeasurement>(
          reader.takeImu() ) );
    }
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeImu() ) );
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeImu() ) );
  }

  TEST( EurocAdapterTest, SummaryRepresentsEmptyAndSingleRecordStreams )
  {
    const std::string imu_header =
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
        "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
        "a_RS_S_z [m s^-2]\n";
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", "#timestamp [ns],filename\n" );
      fixture.writeCsv( "cam1", "#timestamp [ns],filename\n" );
      fixture.writeCsv( "imu0", imu_header );
      auto opened = phad::io::dataset::euroc::open( fixture.root() );
      ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
      const auto summary = opened.value().summary();
      EXPECT_EQ( summary.imu.count, 0U );
      EXPECT_FALSE( summary.imu.first_timestamp.has_value() );
      EXPECT_FALSE( summary.imu.last_timestamp.has_value() );
      EXPECT_EQ( summary.stereo.count, 0U );
      EXPECT_FALSE( summary.stereo.first_timestamp.has_value() );
      EXPECT_FALSE( summary.stereo.last_timestamp.has_value() );
    }
    {
      EurocFixture fixture;
      const auto   timestamp =
          std::to_string( EurocFixture::kFirstTimestamp );
      fixture.writeCsv( "cam0", "#timestamp [ns],filename\n" + timestamp +
                                    ",left-a.png\n" );
      fixture.writeCsv( "cam1", "#timestamp [ns],filename\n" + timestamp +
                                    ",right-a.png\n" );
      fixture.writeCsv( "imu0", imu_header + timestamp +
                                    ",0.1,0.2,0.3,1.1,1.2,9.8\n" );
      auto opened = phad::io::dataset::euroc::open( fixture.root() );
      ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
      const auto summary = opened.value().summary();
      EXPECT_EQ( summary.imu.count, 1U );
      EXPECT_EQ( summary.imu.first_timestamp, summary.imu.last_timestamp );
      EXPECT_EQ( summary.stereo.count, 1U );
      EXPECT_EQ( summary.stereo.first_timestamp,
                 summary.stereo.last_timestamp );
    }
  }

  TEST( EurocAdapterTest, StereoFailureIsStickyAndLocalToReader )
  {
    EurocFixture fixture;
    auto         opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    auto failed_reader = opened.value().reader();
    auto other_reader  = opened.value().reader();
    {
      std::ofstream corrupt(
          fixture.sensorPath( "cam1", "data/right-a.png" ),
          std::ios::binary | std::ios::trunc );
      corrupt << "not a png";
    }

    const auto failed = failed_reader.takeStereo();
    ASSERT_TRUE(
        std::holds_alternative<phad::io::dataset::DatasetReaderError>(
            failed ) );
    const auto error =
        std::get<phad::io::dataset::DatasetReaderError>( failed );
    EXPECT_EQ(
        error.code,
        phad::io::dataset::DatasetReaderErrorCode::kImageDecodeFailed );
    EXPECT_EQ( error.sensor_id, "right_camera" );
    EXPECT_EQ( error.timestamp.nanoseconds(),
               EurocFixture::kFirstTimestamp );
    EXPECT_EQ( error.record_number, 1U );
    EXPECT_FALSE( error.cause.empty() );
    EXPECT_EQ( error.cause.find( fixture.root().string() ),
               std::string::npos );

    fixture.writeImage( "cam1", "right-a.png", 21 );
    expectStickyTerminalError( failed_reader, error );
    EXPECT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
        other_reader.takeStereo() ) );
  }

  TEST( EurocAdapterTest, ReaderReportsImageFormatMismatchWithoutPath )
  {
    EurocFixture fixture;
    auto         opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    fixture.writeImage( "cam0", "left-a.png", 11, 5, 3 );

    auto       reader = opened.value().reader();
    const auto failed = reader.takeStereo();
    ASSERT_TRUE(
        std::holds_alternative<phad::io::dataset::DatasetReaderError>(
            failed ) );
    const auto& error =
        std::get<phad::io::dataset::DatasetReaderError>( failed );
    EXPECT_EQ(
        error.code,
        phad::io::dataset::DatasetReaderErrorCode::kImageFormatMismatch );
    EXPECT_EQ( error.sensor_id, "left_camera" );
    EXPECT_EQ( error.timestamp.nanoseconds(),
               EurocFixture::kFirstTimestamp );
    EXPECT_EQ( error.record_number, 1U );
    EXPECT_EQ( error.cause.find( fixture.root().string() ),
               std::string::npos );
  }

  TEST( EurocAdapterTest, PreservesEurocTbsAsProjectBodyFromCameraTransform )
  {
    EurocFixture      fixture;
    std::string       yaml     = EurocFixture::validCameraYaml();
    const std::string identity = EurocFixture::identityTransformYaml();
    const std::string translated =
        "T_BS:\n"
        "  rows: 4\n"
        "  cols: 4\n"
        "  data: [1, 0, 0, 1.25, 0, 1, 0, -2.5, 0, 0, 1, 3.75, 0, 0, 0, 1]\n";
    yaml.replace( yaml.find( identity ), identity.size(), translated );
    fixture.writeSensorFile( "cam0", yaml );

    auto opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    const auto T_B_camera =
        opened.value().calibration().T_B_left_camera().translation();
    EXPECT_DOUBLE_EQ( T_B_camera.x(), 1.25 );
    EXPECT_DOUBLE_EQ( T_B_camera.y(), -2.5 );
    EXPECT_DOUBLE_EQ( T_B_camera.z(), 3.75 );
  }

  TEST( EurocAdapterTest, IndependentDatasetsAndReadersAreDeterministic )
  {
    EurocFixture fixture;
    auto         first  = phad::io::dataset::euroc::open( fixture.root() );
    auto         second = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( first.hasValue() ) << first.error().describe();
    ASSERT_TRUE( second.hasValue() ) << second.error().describe();
    EXPECT_EQ( first.value().summary().imu.count,
               second.value().summary().imu.count );
    EXPECT_EQ( first.value().summary().stereo.count,
               second.value().summary().stereo.count );

    auto first_reader  = first.value().reader();
    auto second_reader = second.value().reader();
    for ( std::size_t consumed = 0;
          consumed < first.value().summary().imu.count; ++consumed )
    {
      const auto first_imu  = first_reader.takeImu();
      const auto second_imu = second_reader.takeImu();
      ASSERT_TRUE( std::holds_alternative<phad::sensor::ImuMeasurement>(
          first_imu ) );
      ASSERT_TRUE( std::holds_alternative<phad::sensor::ImuMeasurement>(
          second_imu ) );
      EXPECT_EQ( std::get<phad::sensor::ImuMeasurement>( first_imu ).timestamp,
                 std::get<phad::sensor::ImuMeasurement>( second_imu ).timestamp );
      EXPECT_EQ( std::get<phad::sensor::ImuMeasurement>( first_imu ).accel_mps2,
                 std::get<phad::sensor::ImuMeasurement>( second_imu ).accel_mps2 );
      EXPECT_EQ( std::get<phad::sensor::ImuMeasurement>( first_imu ).gyro_radps,
                 std::get<phad::sensor::ImuMeasurement>( second_imu ).gyro_radps );
    }
    for ( std::size_t consumed = 0;
          consumed < first.value().summary().stereo.count; ++consumed )
    {
      const auto first_stereo  = first_reader.takeStereo();
      const auto second_stereo = second_reader.takeStereo();
      ASSERT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
          first_stereo ) );
      ASSERT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
          second_stereo ) );
      const auto& first_frame =
          std::get<phad::sensor::StereoFrame>( first_stereo );
      const auto& second_frame =
          std::get<phad::sensor::StereoFrame>( second_stereo );
      EXPECT_EQ( first_frame.timestamp, second_frame.timestamp );
      ASSERT_EQ( first_frame.left.pixelType(),
                 phad::sensor::PixelType::kUint8 );
      ASSERT_EQ( first_frame.right.pixelType(),
                 phad::sensor::PixelType::kUint8 );
      ASSERT_EQ( second_frame.left.pixelType(),
                 phad::sensor::PixelType::kUint8 );
      ASSERT_EQ( second_frame.right.pixelType(),
                 phad::sensor::PixelType::kUint8 );
      const auto first_left_pixels =
          first_frame.left.pixels<std::uint8_t>();
      const auto first_right_pixels =
          first_frame.right.pixels<std::uint8_t>();
      const auto second_left_pixels =
          second_frame.left.pixels<std::uint8_t>();
      const auto second_right_pixels =
          second_frame.right.pixels<std::uint8_t>();
      ASSERT_TRUE( first_left_pixels.has_value() );
      ASSERT_TRUE( first_right_pixels.has_value() );
      ASSERT_TRUE( second_left_pixels.has_value() );
      ASSERT_TRUE( second_right_pixels.has_value() );
      EXPECT_TRUE(
          std::ranges::equal( *first_left_pixels, *second_left_pixels ) );
      EXPECT_TRUE(
          std::ranges::equal( *first_right_pixels, *second_right_pixels ) );
    }
  }

  TEST( EurocAdapterTest,
        DefersCorruptPngFailureUntilSequentialStereoConsumption )
  {
    EurocFixture fixture;
    {
      std::ofstream corrupt( fixture.sensorPath( "cam1", "data/right-b.png" ),
                             std::ios::binary | std::ios::trunc );
      corrupt << "not a png";
    }

    auto opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    EXPECT_EQ( opened.value().summary().stereo.count, 3U );
    auto reader = opened.value().reader();
    EXPECT_TRUE( std::holds_alternative<phad::common::Timestamp>(
        reader.peekStereoTimestamp() ) );
    EXPECT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
        reader.takeStereo() ) );

    const auto failed = reader.takeStereo();
    ASSERT_TRUE(
        std::holds_alternative<phad::io::dataset::DatasetReaderError>(
            failed ) );
    const auto error =
        std::get<phad::io::dataset::DatasetReaderError>( failed );
    EXPECT_EQ(
        error.code,
        phad::io::dataset::DatasetReaderErrorCode::kImageDecodeFailed );
    EXPECT_EQ( error.sensor_id, "right_camera" );
    EXPECT_EQ( error.record_number, 2U );
    EXPECT_EQ( error.timestamp.nanoseconds(),
               EurocFixture::kFirstTimestamp + 50'000'000 );
    expectStickyTerminalError( reader, error );
  }

  void expectOpenError( const fs::path&                     root,
                        phad::io::dataset::DatasetErrorCode expected_code,
                        const std::string&                  expected_sensor = {},
                        const std::string&                  expected_field  = {} )
  {
    auto result = phad::io::dataset::euroc::open( root );
    ASSERT_FALSE( result.hasValue() );
    EXPECT_EQ( result.error().code, expected_code ) << result.error().describe();
    if ( !expected_sensor.empty() )
    {
      EXPECT_EQ( result.error().sensor_id, expected_sensor );
    }
    if ( !expected_field.empty() )
    {
      EXPECT_EQ( result.error().field, expected_field );
    }
    EXPECT_FALSE( result.error().source_path.empty() );
    EXPECT_FALSE( result.error().cause.empty() );
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

  TEST( EurocAdapterTest, RejectsMissingRootAndRequiredFiles )
  {
    EurocFixture fixture;
    expectOpenError( fixture.root() / "absent",
                     phad::io::dataset::DatasetErrorCode::kRootNotFound );

    fs::remove( fixture.sensorPath( "cam1", "sensor.yaml" ) );
    expectOpenError( fixture.root(),
                     phad::io::dataset::DatasetErrorCode::kRequiredFileMissing,
                     "cam1" );
  }

  TEST( EurocAdapterTest, RejectsCsvHeaderColumnAndFieldErrors )
  {
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", "timestamp,filename\n" );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kInvalidCsvHeader,
                       "cam0", "header" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "cam0",
          "#timestamp [ns],filename\n1403636579763555584,left-a.png,extra\n" );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kInvalidColumnCount,
                       "cam0" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "imu0",
          "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
          "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
          "a_RS_S_z [m s^-2]\n1403636579763555584,nope,0,0,0,0,1\n" );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kInvalidField, "imu0",
                       "w_RS_S_x" );
    }
  }

  TEST( EurocAdapterTest, RejectsInvalidOverflowDuplicateAndReverseTimestamps )
  {
    const auto camera_csv = []( const std::string& rows ) {
      return "#timestamp [ns],filename\n" + rows;
    };
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", camera_csv( "1.5,left-a.png\n" ) );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kInvalidTimestamp,
                       "cam0", "timestamp" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0",
                        camera_csv( "9223372036854775808,left-a.png\n" ) );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kTimestampOverflow,
                       "cam0", "timestamp" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "cam0", camera_csv( "1403636579763555584,left-a.png\n"
                              "1403636579763555584,left-b.png\n" ) );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kDuplicateTimestamp,
                       "cam0", "timestamp" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "cam0", camera_csv( "1403636579813555584,left-b.png\n"
                              "1403636579763555584,left-a.png\n" ) );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kOutOfOrderTimestamp,
                       "cam0", "timestamp" );
    }
  }

  TEST( EurocAdapterTest, RejectsNonFiniteImuMeasurementsWithContext )
  {
    for ( const std::string value : { "nan", "inf", "-inf" } )
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "imu0",
          "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
          "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
          "a_RS_S_z [m s^-2]\n1403636579763555584," +
              value + ",0,0,0,0,1\n" );
      auto result = phad::io::dataset::euroc::open( fixture.root() );
      ASSERT_FALSE( result.hasValue() );
      EXPECT_EQ( result.error().code,
                 phad::io::dataset::DatasetErrorCode::kNonFiniteMeasurement );
      EXPECT_EQ( result.error().sensor_id, "imu0" );
      EXPECT_EQ( result.error().line_or_record_index, 2U );
      EXPECT_EQ( result.error().timestamp->nanoseconds(),
                 EurocFixture::kFirstTimestamp );
      EXPECT_EQ( result.error().field, "w_RS_S_x" );
    }
  }

  TEST( EurocAdapterTest, RejectsDuplicateAndReverseImuTimestamps )
  {
    const std::string header =
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
        "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
        "a_RS_S_z [m s^-2]\n";
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "imu0", header +
                      "1403636579763555584,0,0,0,0,0,1\n"
                      "1403636579763555584,0,0,0,0,0,1\n" );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kDuplicateTimestamp,
                       "imu0", "timestamp" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "imu0", header +
                      "1403636579813555584,0,0,0,0,0,1\n"
                      "1403636579763555584,0,0,0,0,0,1\n" );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kOutOfOrderTimestamp,
                       "imu0", "timestamp" );
    }
  }

  TEST( EurocAdapterTest, RejectsMissingOrMismatchedStereoRecords )
  {
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "cam1",
          "#timestamp [ns],filename\n"
          "1403636579763555584,right-a.png\n"
          "1403636579813555584,right-b.png\n" );
      expectOpenError(
          fixture.root(),
          phad::io::dataset::DatasetErrorCode::kStereoTimestampMismatch,
          "cam0/cam1", "timestamp" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "cam1",
          "#timestamp [ns],filename\n"
          "1403636579763555584,right-a.png\n"
          "1403636579813555585,right-b.png\n"
          "1403636579863555584,right-c.png\n" );
      expectOpenError(
          fixture.root(),
          phad::io::dataset::DatasetErrorCode::kStereoTimestampMismatch,
          "cam0/cam1", "timestamp" );
    }
  }

  TEST( EurocAdapterTest, RejectsUnsafeAndMissingManifestImagePaths )
  {
    const auto csv = []( const std::string& filename ) {
      return "#timestamp [ns],filename\n1403636579763555584," + filename + "\n";
    };
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", csv( "/tmp/image.png" ) );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kUnsafeImagePath, "cam0",
                       "filename" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", csv( "../left-a.png" ) );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kUnsafeImagePath, "cam0",
                       "filename" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", csv( "missing.png" ) );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kImageFileMissing, "cam0",
                       "filename" );
    }
  }

  TEST( EurocAdapterTest, RejectsInvalidTransformAndUnsupportedCameraModels )
  {
    {
      EurocFixture fixture;
      fixture.writeSensorFile(
          "cam0",
          "sensor_type: camera\n"
          "T_BS: {rows: 4, cols: 4, data: [1, 0, 0, 0, 0, 2, 0, 0, 0, "
          "0, 1, 0, 0, 0, 0, 1]}\n"
          "rate_hz: 20\nresolution: [4, 3]\ncamera_model: pinhole\n"
          "intrinsics: [100, 101, 2, 1.5]\n"
          "distortion_model: radial-tangential\n"
          "distortion_coefficients: [0, 0, 0, 0]\n" );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kInvalidCalibration,
                       "cam0", "T_BS.rotation" );
      const auto opened =
          phad::io::dataset::euroc::open( fixture.root() );
      expectCoreCalibrationCause(
          opened.error(), phad::sensor::CalibrationErrorCode::kInvalidRotation,
          "rigid_transform.rotation",
          "rotation must be orthogonal with determinant +1" );
    }
    {
      EurocFixture fixture;
      fixture.writeSensorFile(
          "cam0",
          "sensor_type: camera\n"
          "T_BS: {rows: 4, cols: 4, data: [1, 0, 0, 0, 0, 1, 0, 0, 0, "
          "0, 1, 0, 0.000002, 0, 0, 1]}\n"
          "rate_hz: 20\nresolution: [4, 3]\ncamera_model: pinhole\n"
          "intrinsics: [100, 101, 2, 1.5]\n"
          "distortion_model: radial-tangential\n"
          "distortion_coefficients: [0, 0, 0, 0]\n" );
      auto opened = phad::io::dataset::euroc::open( fixture.root() );
      ASSERT_FALSE( opened );
      EXPECT_EQ( opened.error().field, "T_BS.bottom_row" );
      expectCoreCalibrationCause(
          opened.error(),
          phad::sensor::CalibrationErrorCode::kInvalidHomogeneousRow,
          "rigid_transform.bottom_row",
          "bottom row must be [0, 0, 0, 1]" );
    }
    {
      EurocFixture fixture;
      fixture.writeSensorFile( "cam0",
                               EurocFixture::validCameraYaml( "omnidirectional" ) );
      expectOpenError(
          fixture.root(),
          phad::io::dataset::DatasetErrorCode::kUnsupportedCameraModel, "cam0",
          "camera_model" );
    }
    {
      EurocFixture fixture;
      fixture.writeSensorFile(
          "cam0", EurocFixture::validCameraYaml( "pinhole", "equidistant" ) );
      expectOpenError(
          fixture.root(),
          phad::io::dataset::DatasetErrorCode::kUnsupportedDistortionModel, "cam0",
          "distortion_model" );
    }
  }

  TEST( EurocAdapterTest, MapsEveryCoreSourceFieldWithProvenance )
  {
    struct MappingCase
    {
      const char*                        sensor_id;
      const char*                        original;
      const char*                        replacement;
      const char*                        raw_field;
      phad::sensor::CalibrationErrorCode code;
      const char*                        canonical_field;
      const char*                        detail;
    };
    constexpr const char* kTransform =
        "data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]";
    const std::array<MappingCase, 19> cases{ {
        { "cam0", kTransform,
          "data: [.nan, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]",
          "T_BS", phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "rigid_transform.matrix", "all matrix elements must be finite" },
        { "cam0", kTransform,
          "data: [2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]",
          "T_BS.rotation",
          phad::sensor::CalibrationErrorCode::kInvalidRotation,
          "rigid_transform.rotation",
          "rotation must be orthogonal with determinant +1" },
        { "cam0", kTransform,
          "data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2]",
          "T_BS.bottom_row",
          phad::sensor::CalibrationErrorCode::kInvalidHomogeneousRow,
          "rigid_transform.bottom_row",
          "bottom row must be [0, 0, 0, 1]" },
        { "cam0", "intrinsics: [100, 101, 2, 1.5]",
          "intrinsics: [0, 101, 2, 1.5]", "intrinsics",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.radial_tangential.fx_pixels",
          "fx_pixels must be strictly positive" },
        { "cam0", "intrinsics: [100, 101, 2, 1.5]",
          "intrinsics: [100, 0, 2, 1.5]", "intrinsics",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "camera.model_parameters.radial_tangential.fy_pixels",
          "fy_pixels must be strictly positive" },
        { "cam0", "intrinsics: [100, 101, 2, 1.5]",
          "intrinsics: [100, 101, .nan, 1.5]", "intrinsics",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.radial_tangential.cx_pixels",
          "camera model parameter must be finite" },
        { "cam0", "intrinsics: [100, 101, 2, 1.5]",
          "intrinsics: [100, 101, 2, .nan]", "intrinsics",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.radial_tangential.cy_pixels",
          "camera model parameter must be finite" },
        { "cam0", "distortion_coefficients: [0.1, -0.2, 0.001, -0.002]",
          "distortion_coefficients: [.nan, -0.2, 0.001, -0.002]",
          "distortion_coefficients",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.radial_tangential.k1",
          "camera model parameter must be finite" },
        { "cam0", "distortion_coefficients: [0.1, -0.2, 0.001, -0.002]",
          "distortion_coefficients: [0.1, .nan, 0.001, -0.002]",
          "distortion_coefficients",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.radial_tangential.k2",
          "camera model parameter must be finite" },
        { "cam0", "distortion_coefficients: [0.1, -0.2, 0.001, -0.002]",
          "distortion_coefficients: [0.1, -0.2, .nan, -0.002]",
          "distortion_coefficients",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.radial_tangential.p1",
          "camera model parameter must be finite" },
        { "cam0", "distortion_coefficients: [0.1, -0.2, 0.001, -0.002]",
          "distortion_coefficients: [0.1, -0.2, 0.001, .nan]",
          "distortion_coefficients",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.model_parameters.radial_tangential.p2",
          "camera model parameter must be finite" },
        { "cam0", "resolution: [4, 3]", "resolution: [0, 3]",
          "resolution", phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "camera.image_width", "image width must be positive" },
        { "cam0", "resolution: [4, 3]", "resolution: [4, 0]",
          "resolution", phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "camera.image_height", "image height must be positive" },
        { "cam0", "rate_hz: 20", "rate_hz: .nan", "rate_hz",
          phad::sensor::CalibrationErrorCode::kNonFiniteValue,
          "camera.rate_hz", "declared sample rate must be finite" },
        { "imu0", "rate_hz: 200", "rate_hz: 0", "rate_hz",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.rate_hz", "IMU parameter must be strictly positive" },
        { "imu0", "accelerometer_noise_density: 0.002",
          "accelerometer_noise_density: 0", "accelerometer_noise_density",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.accelerometer_noise_density_mps2_per_sqrt_hz",
          "IMU parameter must be strictly positive" },
        { "imu0", "gyroscope_noise_density: 0.0001",
          "gyroscope_noise_density: 0", "gyroscope_noise_density",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.gyroscope_noise_density_radps_per_sqrt_hz",
          "IMU parameter must be strictly positive" },
        { "imu0", "accelerometer_random_walk: 0.003",
          "accelerometer_random_walk: 0", "accelerometer_random_walk",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.accelerometer_bias_random_walk_mps3_per_sqrt_hz",
          "IMU parameter must be strictly positive" },
        { "imu0", "gyroscope_random_walk: 0.00001",
          "gyroscope_random_walk: 0", "gyroscope_random_walk",
          phad::sensor::CalibrationErrorCode::kNonPositiveValue,
          "imu.gyroscope_bias_random_walk_radps2_per_sqrt_hz",
          "IMU parameter must be strictly positive" },
    } };

    for ( const auto& test_case : cases )
    {
      SCOPED_TRACE( test_case.canonical_field );
      EurocFixture fixture;
      std::string  yaml     = std::string{ test_case.sensor_id } == "imu0"
                                  ? EurocFixture::validImuYaml()
                                  : EurocFixture::validCameraYaml();
      const auto   position = yaml.find( test_case.original );
      ASSERT_NE( position, std::string::npos );
      yaml.replace( position, std::string{ test_case.original }.size(),
                    test_case.replacement );
      fixture.writeSensorFile( test_case.sensor_id, yaml );

      auto opened = phad::io::dataset::euroc::open( fixture.root() );
      ASSERT_FALSE( opened );
      EXPECT_EQ( opened.error().code,
                 phad::io::dataset::DatasetErrorCode::kInvalidCalibration );
      EXPECT_EQ( opened.error().sensor_id, test_case.sensor_id );
      EXPECT_EQ( opened.error().source_path,
                 fixture.sensorPath( test_case.sensor_id, "sensor.yaml" ) );
      EXPECT_EQ( opened.error().field, test_case.raw_field );
      expectCoreCalibrationCause(
          opened.error(), test_case.code, test_case.canonical_field,
          test_case.detail );
    }
  }

  TEST( EurocAdapterTest, MapsZeroBaselineWithJointProvenance )
  {
    {
      EurocFixture fixture;
      fixture.writeSensorFile( "cam1", EurocFixture::validCameraYaml() );

      auto opened = phad::io::dataset::euroc::open( fixture.root() );
      ASSERT_FALSE( opened );
      EXPECT_EQ( opened.error().code,
                 phad::io::dataset::DatasetErrorCode::kInvalidCalibration );
      EXPECT_EQ( opened.error().sensor_id, "cam0/cam1" );
      EXPECT_EQ( opened.error().source_path, fixture.root() / "mav0" );
      EXPECT_EQ( opened.error().field, "cam0.T_BS/cam1.T_BS" );
      expectCoreCalibrationCause(
          opened.error(),
          phad::sensor::CalibrationErrorCode::kZeroStereoBaseline,
          "stereo_imu_calibration.camera_centers",
          "left and right camera centers must not coincide" );
    }
  }

  TEST( EurocAdapterTest, PreservesRawRateFieldForYamlConversionFailures )
  {
    struct TestCase
    {
      const char* sensor_id;
      const char* valid_line;
      const char* invalid_line;
    };
    const std::array<TestCase, 4> cases{ {
        { "cam0", "rate_hz: 20\n", "rate_hz: not-a-number\n" },
        { "cam0", "rate_hz: 20\n", "" },
        { "imu0", "rate_hz: 200\n", "rate_hz: not-a-number\n" },
        { "imu0", "rate_hz: 200\n", "" },
    } };
    for ( const auto& test_case : cases )
    {
      SCOPED_TRACE( std::string{ test_case.sensor_id } +
                    ( test_case.invalid_line[ 0 ] == '\0' ? " missing"
                                                          : " conversion" ) );
      EurocFixture fixture;
      std::string  yaml     = std::string{ test_case.sensor_id } == "imu0"
                                  ? EurocFixture::validImuYaml()
                                  : EurocFixture::validCameraYaml();
      const auto   position = yaml.find( test_case.valid_line );
      ASSERT_NE( position, std::string::npos );
      yaml.replace( position, std::string{ test_case.valid_line }.size(),
                    test_case.invalid_line );
      fixture.writeSensorFile( test_case.sensor_id, yaml );

      auto opened = phad::io::dataset::euroc::open( fixture.root() );
      ASSERT_FALSE( opened );
      EXPECT_EQ( opened.error().code,
                 phad::io::dataset::DatasetErrorCode::kInvalidCalibration );
      EXPECT_EQ( opened.error().sensor_id, test_case.sensor_id );
      EXPECT_EQ( opened.error().source_path,
                 fixture.sensorPath( test_case.sensor_id, "sensor.yaml" ) );
      EXPECT_EQ( opened.error().field, "rate_hz" );
      EXPECT_FALSE( opened.error().cause.empty() );
      EXPECT_EQ( opened.error().cause.find( "core calibration error" ),
                 std::string::npos );
    }
  }

  TEST( EurocAdapterTest, RejectsNonIdentityImuExtrinsicsInM1 )
  {
    EurocFixture      fixture;
    std::string       yaml     = EurocFixture::validImuYaml();
    const std::string identity = EurocFixture::identityTransformYaml();
    const std::string translated =
        "T_BS:\n"
        "  rows: 4\n"
        "  cols: 4\n"
        "  data: [1, 0, 0, 0.01, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n";
    yaml.replace( yaml.find( identity ), identity.size(), translated );
    fixture.writeSensorFile( "imu0", yaml );

    expectOpenError(
        fixture.root(),
        phad::io::dataset::DatasetErrorCode::kUnsupportedImuExtrinsics, "imu0",
        "T_BS" );
  }

  TEST( EurocAdapterTest, RejectsMissingAndInvalidImuNoiseFields )
  {
    {
      EurocFixture      fixture;
      std::string       yaml = EurocFixture::validImuYaml();
      const std::string line = "gyroscope_noise_density: 0.0001\n";
      yaml.erase( yaml.find( line ), line.size() );
      fixture.writeSensorFile( "imu0", yaml );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kInvalidCalibration,
                       "imu0", "gyroscope_noise_density" );
    }
    for ( const std::string value : { "0", "-1", ".nan", ".inf" } )
    {
      EurocFixture      fixture;
      std::string       yaml     = EurocFixture::validImuYaml();
      const std::string old_line = "accelerometer_random_walk: 0.003";
      yaml.replace( yaml.find( old_line ), old_line.size(),
                    "accelerometer_random_walk: " + value );
      fixture.writeSensorFile( "imu0", yaml );
      expectOpenError( fixture.root(),
                       phad::io::dataset::DatasetErrorCode::kInvalidCalibration,
                       "imu0", "accelerometer_random_walk" );
    }
  }

  void expectSequentialFormatMismatch( EurocFixture& fixture )
  {
    auto opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    auto reader = opened.value().reader();
    ASSERT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
        reader.takeStereo() ) );

    const auto failed = reader.takeStereo();
    ASSERT_TRUE(
        std::holds_alternative<phad::io::dataset::DatasetReaderError>(
            failed ) );
    const auto error =
        std::get<phad::io::dataset::DatasetReaderError>( failed );
    EXPECT_EQ(
        error.code,
        phad::io::dataset::DatasetReaderErrorCode::kImageFormatMismatch );
    EXPECT_EQ( error.sensor_id, "left_camera" );
    EXPECT_EQ( error.record_number, 2U );
    EXPECT_EQ( error.timestamp.nanoseconds(),
               EurocFixture::kFirstTimestamp + 50'000'000 );
    EXPECT_FALSE( error.cause.empty() );
    expectStickyTerminalError( reader, error );
  }

  TEST( EurocAdapterTest,
        ReportsWrongImageDimensionsAndPixelTypesAsStickyReaderErrors )
  {
    {
      EurocFixture fixture;
      fixture.writeImage( "cam0", "left-b.png", 12, 5, 3 );
      expectSequentialFormatMismatch( fixture );
    }
    for ( const int type : { CV_8UC3, CV_16UC1 } )
    {
      EurocFixture fixture;
      fixture.writeImage( "cam0", "left-b.png", 12, 4, 3, type );
      expectSequentialFormatMismatch( fixture );
    }
  }

#ifdef __linux__
  std::size_t CurrentResidentBytes()
  {
    std::ifstream statm( "/proc/self/statm" );
    std::size_t   virtual_pages  = 0;
    std::size_t   resident_pages = 0;
    statm >> virtual_pages >> resident_pages;
    return resident_pages * static_cast<std::size_t>( sysconf( _SC_PAGESIZE ) );
  }

  TEST( EurocAdapterTest, ResidentImageMemoryDoesNotGrowWithTraversedFrameCount )
  {
#if defined( __SANITIZE_ADDRESS__ )
    GTEST_SKIP() << "ASan quarantine makes process RSS unsuitable for cache detection";
#endif
    constexpr int         kWidth      = 512;
    constexpr int         kHeight     = 512;
    constexpr std::size_t kFrameCount = 48;
    EurocFixture          fixture;
    fixture.writeSensorFile(
        "cam0", EurocFixture::validCameraYaml( "pinhole", "radial-tangential",
                                               kWidth, kHeight ) );
    fixture.writeSensorFile(
        "cam1",
        EurocFixture::validRightCameraYaml(
            "pinhole", "radial-tangential", kWidth, kHeight ) );

    std::ostringstream left_csv;
    std::ostringstream right_csv;
    left_csv << "#timestamp [ns],filename\n";
    right_csv << "#timestamp [ns],filename\n";
    for ( std::size_t index = 0; index < kFrameCount; ++index )
    {
      const auto timestamp =
          EurocFixture::kFirstTimestamp + static_cast<std::int64_t>( index );
      const std::string left_name = "memory-left-" + std::to_string( index ) + ".png";
      const std::string right_name =
          "memory-right-" + std::to_string( index ) + ".png";
      left_csv << timestamp << ',' << left_name << '\n';
      right_csv << timestamp << ',' << right_name << '\n';
      fixture.writeImage( "cam0", left_name,
                          static_cast<std::uint8_t>( index ), kWidth, kHeight );
      fixture.writeImage( "cam1", right_name,
                          static_cast<std::uint8_t>( index + 1 ), kWidth, kHeight );
    }
    fixture.writeCsv( "cam0", left_csv.str() );
    fixture.writeCsv( "cam1", right_csv.str() );

    auto opened = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    auto reader = opened.value().reader();
    {
      const auto warmup = reader.takeStereo();
      ASSERT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
          warmup ) );
    }
    const std::size_t resident_after_warmup = CurrentResidentBytes();
    for ( std::size_t consumed = 1; consumed < kFrameCount; ++consumed )
    {
      const auto loaded = reader.takeStereo();
      ASSERT_TRUE( std::holds_alternative<phad::sensor::StereoFrame>(
          loaded ) );
      const auto pixels =
          std::get<phad::sensor::StereoFrame>( loaded )
              .left.pixels<std::uint8_t>();
      ASSERT_TRUE( pixels.has_value() );
      EXPECT_EQ( pixels->front(), static_cast<std::uint8_t>( consumed ) );
    }
    EXPECT_TRUE( std::holds_alternative<phad::io::dataset::DatasetReaderEnd>(
        reader.takeStereo() ) );
    const std::size_t resident_after_traversal = CurrentResidentBytes();

    // A retained cache would hold roughly 24 MiB for this fixture. Allow 8 MiB
    // for allocator/OpenCV working-set variation while still detecting growth
    // proportional to all decoded frames.
    EXPECT_LE( resident_after_traversal, resident_after_warmup + 8U * 1024U * 1024U );
  }
#endif

}  // namespace
