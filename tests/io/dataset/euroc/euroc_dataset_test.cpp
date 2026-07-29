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
      writeSensorFile( "cam1", validCameraYaml() );
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
    const std::string output = readFile( stdout_path );
    EXPECT_NE( output.find( "left_camera_resolution: 4x3" ),
               std::string::npos );
    EXPECT_NE( output.find( "right_camera_resolution: 4x3" ),
               std::string::npos );
    EXPECT_NE( output.find( "imu_rate_hz: 200" ), std::string::npos );
    EXPECT_NE( output.find( "stereo_frames: 3" ), std::string::npos );
    EXPECT_NE( output.find( "imu_measurements: 4" ), std::string::npos );
    EXPECT_EQ( output.find( "sample[" ), std::string::npos );
    EXPECT_TRUE( readFile( stderr_path ).empty() );
  }

  TEST( EurocInspectTest, ReportsOpenErrorAndReturnsNonzero )
  {
    EurocFixture      fixture;
    const fs::path    missing_path = fixture.root() / "missing";
    const fs::path    stdout_path  = fixture.root() / "inspect.stdout";
    const fs::path    stderr_path  = fixture.root() / "inspect.stderr";
    const std::string command =
        std::string{ "\"" PHAD_EUROC_INSPECT_PATH "\" \"" } +
        missing_path.string() + "\" > \"" + stdout_path.string() +
        "\" 2> \"" + stderr_path.string() + "\"";

    EXPECT_NE( std::system( command.c_str() ), 0 );
    EXPECT_TRUE( readFile( stdout_path ).empty() );
    const std::string error = readFile( stderr_path );
    EXPECT_NE( error.find( "dataset error" ), std::string::npos );
    EXPECT_NE( error.find( missing_path.string() ), std::string::npos );
  }

  TEST( EurocAdapterTest,
        AdapterOpensCalibrationSummaryAndSequentialMeasurements )
  {
    EurocFixture fixture;
    auto         result = phad::io::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( result.hasValue() ) << result.error().describe();
    const auto calibration = result.value().calibration();
    const auto summary     = result.value().summary();
    EXPECT_EQ( calibration.left.resolution.width, 4 );
    EXPECT_EQ( calibration.left.resolution.height, 3 );
    EXPECT_DOUBLE_EQ( calibration.left.intrinsics.fx_pixels, 100.0 );
    EXPECT_DOUBLE_EQ( calibration.left.T_B_camera.matrix[ 0 ], 1.0 );
    EXPECT_DOUBLE_EQ( calibration.imu.rate_hz, 200.0 );
    EXPECT_DOUBLE_EQ( calibration.imu.gyr_nd, 0.0001 );
    EXPECT_DOUBLE_EQ( calibration.imu.acc_rw, 0.003 );
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
    EXPECT_EQ( calibration.left.resolution.width, 4 );
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
    const auto& T_B_camera = opened.value().calibration().left.T_B_camera.matrix;
    EXPECT_DOUBLE_EQ( T_B_camera[ 3 ], 1.25 );
    EXPECT_DOUBLE_EQ( T_B_camera[ 7 ], -2.5 );
    EXPECT_DOUBLE_EQ( T_B_camera[ 11 ], 3.75 );
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
        "cam1", EurocFixture::validCameraYaml( "pinhole", "radial-tangential",
                                               kWidth, kHeight ) );

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
