#include "phad/io/dataset/dataset_replay_source.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <random>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "phad/io/dataset/euroc/euroc_dataset.hpp"

namespace
{

  namespace fs = std::filesystem;
  using phad::io::EndOfStream;
  using phad::io::SensorEvent;
  using phad::io::SensorReadResult;
  using phad::io::SensorSource;
  using phad::io::SensorSourceError;
  using phad::io::dataset::DatasetReplaySource;

  class ReplayDatasetFixture
  {
  public:
    static constexpr std::int64_t kFirstTimestamp =
        1'403'636'579'763'555'584;

    ReplayDatasetFixture()
    {
      m_root = makeUniqueRoot();
      for ( const auto* sensor : { "cam0", "cam1", "imu0" } )
      {
        fs::create_directories( m_root / "mav0" / sensor / "data" );
      }
      writeCalibrations();
      writeDefaultCsvFiles();
      writeImage( "cam0", "left-a.png", 11 );
      writeImage( "cam0", "left-b.png", 12 );
      writeImage( "cam1", "right-a.png", 21 );
      writeImage( "cam1", "right-b.png", 22 );
    }

    ~ReplayDatasetFixture() { fs::remove_all( m_root ); }

    ReplayDatasetFixture( const ReplayDatasetFixture& )            = delete;
    ReplayDatasetFixture& operator=( const ReplayDatasetFixture& ) = delete;

    [[nodiscard]] const fs::path& root() const noexcept { return m_root; }

    void corruptSecondRightImage()
    {
      std::ofstream stream(
          m_root / "mav0" / "cam1" / "data" / "right-b.png",
          std::ios::binary | std::ios::trunc );
      stream << "not a png";
    }

    void writeEmptyStreams()
    {
      writeCsv( "cam0", "#timestamp [ns],filename\n" );
      writeCsv( "cam1", "#timestamp [ns],filename\n" );
      writeCsv(
          "imu0",
          "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
          "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
          "a_RS_S_z [m s^-2]\n" );
    }

  private:
    static fs::path makeUniqueRoot()
    {
      std::random_device random;
      for ( int attempt = 0; attempt < 100; ++attempt )
      {
        const fs::path candidate =
            fs::temp_directory_path() /
            ( "phad_replay_source_test_" + std::to_string( random() ) + "_" +
              std::to_string( random() ) );
        std::error_code error;
        if ( fs::create_directory( candidate, error ) )
        {
          return candidate;
        }
      }
      throw std::runtime_error(
          "failed to create unique replay source test directory" );
    }

    static std::string identityTransformYaml()
    {
      return "T_BS:\n"
             "  rows: 4\n"
             "  cols: 4\n"
             "  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n";
    }

    static std::string cameraYaml()
    {
      return "sensor_type: camera\n" + identityTransformYaml() +
             "rate_hz: 20\n"
             "resolution: [4, 3]\n"
             "camera_model: pinhole\n"
             "intrinsics: [100, 101, 2, 1.5]\n"
             "distortion_model: radial-tangential\n"
             "distortion_coefficients: [0.1, -0.2, 0.001, -0.002]\n";
    }

    static std::string imuYaml()
    {
      return "sensor_type: imu\n" + identityTransformYaml() +
             "rate_hz: 200\n"
             "gyroscope_noise_density: 0.0001\n"
             "gyroscope_random_walk: 0.00001\n"
             "accelerometer_noise_density: 0.002\n"
             "accelerometer_random_walk: 0.003\n";
    }

    void writeCalibrations()
    {
      writeSensorFile( "cam0", cameraYaml() );
      writeSensorFile( "cam1", cameraYaml() );
      writeSensorFile( "imu0", imuYaml() );
    }

    void writeDefaultCsvFiles()
    {
      const auto t0 = std::to_string( kFirstTimestamp );
      const auto t1 = std::to_string( kFirstTimestamp + 10'000'000 );
      writeCsv( "cam0", "#timestamp [ns],filename\n" + t0 +
                            ",left-a.png\n" + t1 + ",left-b.png\n" );
      writeCsv( "cam1", "#timestamp [ns],filename\n" + t0 +
                            ",right-a.png\n" + t1 + ",right-b.png\n" );
      writeCsv(
          "imu0",
          "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
          "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
          "a_RS_S_z [m s^-2]\n" +
              std::to_string( kFirstTimestamp - 5'000'000 ) +
              ",0.1,0.2,0.3,1.1,1.2,9.8\n" + t0 +
              ",0.4,0.5,0.6,1.4,1.5,9.7\n" +
              std::to_string( kFirstTimestamp + 5'000'000 ) +
              ",0.7,0.8,0.9,1.7,1.8,9.6\n" );
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
                     std::uint8_t value )
    {
      const cv::Mat image( 3, 4, CV_8UC1, cv::Scalar( value ) );
      ASSERT_TRUE( cv::imwrite(
          ( m_root / "mav0" / sensor / "data" / filename ).string(),
          image ) );
    }

    fs::path m_root;
  };

  DatasetReplaySource openReplaySource( const fs::path& root )
  {
    auto opened = phad::io::dataset::euroc::open( root );
    if ( !opened )
    {
      throw std::runtime_error( opened.error().describe() );
    }
    const auto dataset = std::move( opened ).value();
    return DatasetReplaySource{ dataset };
  }

  const SensorEvent& expectEvent( const SensorReadResult& result )
  {
    EXPECT_TRUE( std::holds_alternative<SensorEvent>( result ) );
    return std::get<SensorEvent>( result );
  }

  TEST( DatasetReplaySourceTest, ExposesCalibrationThroughSensorSourceSeam )
  {
    ReplayDatasetFixture fixture;
    auto                 source = openReplaySource( fixture.root() );
    SensorSource&        seam   = source;

    EXPECT_EQ( seam.calibration().left.resolution.width, 4 );
    EXPECT_EQ( seam.calibration().left.resolution.height, 3 );
    EXPECT_DOUBLE_EQ( seam.calibration().imu.rate_hz, 200.0 );
  }

  TEST( DatasetReplaySourceTest,
        EmitsAllEventsInOrderWithImuFirstAtEqualTimestamp )
  {
    ReplayDatasetFixture fixture;
    auto                 source = openReplaySource( fixture.root() );
    SensorSource&        seam   = source;

    std::vector<std::pair<std::int64_t, bool>> observed;
    while ( true )
    {
      const SensorReadResult result = seam.next();
      if ( std::holds_alternative<EndOfStream>( result ) )
      {
        break;
      }
      const SensorEvent& event = expectEvent( result );
      if ( const auto* imu =
               std::get_if<phad::sensor::ImuMeasurement>( &event ) )
      {
        observed.emplace_back( imu->timestamp.nanoseconds(), true );
      }
      else
      {
        const auto& stereo = std::get<phad::sensor::StereoFrame>( event );
        observed.emplace_back( stereo.timestamp.nanoseconds(), false );
      }
    }

    EXPECT_EQ(
        observed,
        ( std::vector<std::pair<std::int64_t, bool>>{
            { ReplayDatasetFixture::kFirstTimestamp - 5'000'000, true },
            { ReplayDatasetFixture::kFirstTimestamp, true },
            { ReplayDatasetFixture::kFirstTimestamp, false },
            { ReplayDatasetFixture::kFirstTimestamp + 5'000'000, true },
            { ReplayDatasetFixture::kFirstTimestamp + 10'000'000, false } } ) );
    EXPECT_TRUE( std::holds_alternative<EndOfStream>( seam.next() ) );
    EXPECT_TRUE( std::holds_alternative<EndOfStream>( seam.next() ) );
  }

  TEST( DatasetReplaySourceTest, LazilyDecodesAndRetainsTerminalReadError )
  {
    ReplayDatasetFixture fixture;
    auto                 source = openReplaySource( fixture.root() );
    fixture.corruptSecondRightImage();
    SensorSource& seam = source;

    for ( int index = 0; index < 4; ++index )
    {
      const SensorReadResult result = seam.next();
      ASSERT_TRUE( std::holds_alternative<SensorEvent>( result ) );
    }

    const SensorReadResult first_failure = seam.next();
    ASSERT_TRUE(
        std::holds_alternative<SensorSourceError>( first_failure ) );
    const auto& error = std::get<SensorSourceError>( first_failure );
    EXPECT_EQ( error.code, phad::io::SensorSourceErrorCode::kReadFailed );
    EXPECT_EQ( error.source_id, "right_camera" );
    ASSERT_TRUE( error.timestamp.has_value() );
    EXPECT_EQ( error.timestamp->nanoseconds(),
               ReplayDatasetFixture::kFirstTimestamp + 10'000'000 );
    EXPECT_NE( error.cause.find( "record=2" ), std::string::npos );
    EXPECT_NE( error.cause.find( "OpenCV could not decode the image" ),
               std::string::npos );
    EXPECT_EQ( error.cause.find( fixture.root().string() ), std::string::npos );

    const SensorReadResult repeated_failure = seam.next();
    ASSERT_TRUE(
        std::holds_alternative<SensorSourceError>( repeated_failure ) );
    const auto& repeated_error =
        std::get<SensorSourceError>( repeated_failure );
    EXPECT_EQ( repeated_error.source_id, error.source_id );
    EXPECT_EQ( repeated_error.timestamp, error.timestamp );
    EXPECT_EQ( repeated_error.cause, error.cause );
  }

  TEST( DatasetReplaySourceTest,
        OwnsReaderAndStereoPixelsAfterDatasetHandleDestruction )
  {
    static_assert( !std::is_copy_constructible_v<DatasetReplaySource> );
    static_assert( std::is_move_constructible_v<DatasetReplaySource> );

    ReplayDatasetFixture fixture;
    auto                 source = openReplaySource( fixture.root() );
    SensorSource&        seam   = source;

    static_cast<void>( seam.next() );
    static_cast<void>( seam.next() );
    const SensorReadResult stereo_result = seam.next();
    const SensorEvent&     event         = expectEvent( stereo_result );
    const auto&            stereo        = std::get<phad::sensor::StereoFrame>( event );
    const auto             left_pixels   = stereo.left.pixels<std::uint8_t>();
    const auto             right_pixels  = stereo.right.pixels<std::uint8_t>();
    ASSERT_TRUE( left_pixels.has_value() );
    ASSERT_TRUE( right_pixels.has_value() );
    EXPECT_EQ( left_pixels->front(), 11 );
    EXPECT_EQ( right_pixels->front(), 21 );
  }

  TEST( DatasetReplaySourceTest, EmptyDatasetReturnsStableEndOfStream )
  {
    ReplayDatasetFixture fixture;
    fixture.writeEmptyStreams();
    auto          source = openReplaySource( fixture.root() );
    SensorSource& seam   = source;

    EXPECT_TRUE( std::holds_alternative<EndOfStream>( seam.next() ) );
    EXPECT_TRUE( std::holds_alternative<EndOfStream>( seam.next() ) );
  }

}  // namespace
