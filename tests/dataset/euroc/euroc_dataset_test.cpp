#include "phad/dataset/euroc/euroc_dataset.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <random>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef __linux__
#include <unistd.h>
#endif

namespace
{

  namespace fs = std::filesystem;
  using phad::dataset::EurocDataset;

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

  TEST( EurocDatasetTest, AdapterReturnsNormalizedStereoImuDataset )
  {
    EurocFixture fixture;
    auto         result = phad::dataset::euroc::open( fixture.root() );
    ASSERT_TRUE( result.hasValue() ) << result.error().describe();
    EXPECT_EQ( result.value().stereoIndex().size(), 3U );
    EXPECT_EQ( result.value().imuMeasurements().size(), 4U );
  }

  TEST( EurocDatasetTest, OpensCalibrationImuAndExactStereoIndex )
  {
    EurocFixture fixture;

    auto result = EurocDataset::open( fixture.root() );
    ASSERT_TRUE( result.hasValue() ) << result.error().describe();
    const auto& dataset = result.value();

    ASSERT_EQ( dataset.imuMeasurements().size(), 4U );
    EXPECT_EQ( dataset.imuMeasurements()[ 1 ].timestamp.nanoseconds(),
               EurocFixture::kFirstTimestamp );
    EXPECT_DOUBLE_EQ( dataset.imuMeasurements()[ 1 ].gyro_radps[ 2 ], 0.6 );
    EXPECT_DOUBLE_EQ( dataset.imuMeasurements()[ 1 ].accel_mps2[ 2 ], 9.7 );
    EXPECT_LT( dataset.imuMeasurements()[ 0 ].timestamp,
               dataset.imuMeasurements()[ 1 ].timestamp );

    ASSERT_EQ( dataset.stereoIndex().size(), 3U );
    EXPECT_EQ( dataset.stereoIndex()[ 0 ].timestamp.nanoseconds(),
               EurocFixture::kFirstTimestamp );
    EXPECT_EQ( dataset.stereoIndex()[ 0 ].left_path.filename(), "left-a.png" );
    EXPECT_EQ( dataset.stereoIndex()[ 0 ].right_path.filename(), "right-a.png" );
    EXPECT_LT( dataset.stereoIndex()[ 1 ].timestamp,
               dataset.stereoIndex()[ 2 ].timestamp );

    const auto& calibration = dataset.calibration();
    EXPECT_EQ( calibration.left.resolution.width, 4 );
    EXPECT_EQ( calibration.left.resolution.height, 3 );
    EXPECT_DOUBLE_EQ( calibration.left.intrinsics.fx_pixels, 100.0 );
    EXPECT_DOUBLE_EQ( calibration.left.T_B_camera.matrix[ 0 ], 1.0 );
    EXPECT_DOUBLE_EQ( calibration.imu.rate_hz, 200.0 );
    EXPECT_DOUBLE_EQ(
        calibration.imu.gyr_nd, 0.0001 );
    EXPECT_DOUBLE_EQ(
        calibration.imu.acc_rw, 0.003 );
  }

  TEST( EurocDatasetTest, PreservesEurocTbsAsProjectBodyFromCameraTransform )
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

    auto opened = EurocDataset::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    const auto& T_B_camera = opened.value().calibration().left.T_B_camera.matrix;
    EXPECT_DOUBLE_EQ( T_B_camera[ 3 ], 1.25 );
    EXPECT_DOUBLE_EQ( T_B_camera[ 7 ], -2.5 );
    EXPECT_DOUBLE_EQ( T_B_camera[ 11 ], 3.75 );
  }

  TEST( EurocDatasetTest, RepeatedFullManifestTraversalIsDeterministic )
  {
    EurocFixture fixture;
    auto         first  = EurocDataset::open( fixture.root() );
    auto         second = EurocDataset::open( fixture.root() );
    ASSERT_TRUE( first.hasValue() ) << first.error().describe();
    ASSERT_TRUE( second.hasValue() ) << second.error().describe();

    ASSERT_EQ( first.value().stereoIndex().size(),
               second.value().stereoIndex().size() );
    for ( std::size_t index = 0; index < first.value().stereoIndex().size();
          ++index )
    {
      const auto& lhs = first.value().stereoIndex()[ index ];
      const auto& rhs = second.value().stereoIndex()[ index ];
      EXPECT_EQ( lhs.timestamp, rhs.timestamp );
      EXPECT_EQ( lhs.left_path, rhs.left_path );
      EXPECT_EQ( lhs.right_path, rhs.right_path );
    }

    ASSERT_EQ( first.value().imuMeasurements().size(),
               second.value().imuMeasurements().size() );
    for ( std::size_t index = 0; index < first.value().imuMeasurements().size();
          ++index )
    {
      const auto& lhs = first.value().imuMeasurements()[ index ];
      const auto& rhs = second.value().imuMeasurements()[ index ];
      EXPECT_EQ( lhs.timestamp, rhs.timestamp );
      EXPECT_EQ( lhs.accel_mps2, rhs.accel_mps2 );
      EXPECT_EQ( lhs.gyro_radps, rhs.gyro_radps );
    }
  }

  TEST( EurocDatasetTest, LazilyDecodesFirstMiddleAndLastOwnedStereoFrames )
  {
    EurocFixture fixture;
    auto         opened = EurocDataset::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    const auto& dataset = opened.value();

    for ( const auto [ index, left_value, right_value ] :
          std::array<std::array<std::size_t, 3>, 3>{
              std::array<std::size_t, 3>{ 0, 11, 21 },
              std::array<std::size_t, 3>{ 1, 12, 22 },
              std::array<std::size_t, 3>{ 2, 13, 23 } } )
    {
      auto loaded = dataset.loadStereo( index );
      ASSERT_TRUE( loaded.hasValue() ) << loaded.error().describe();
      const auto& frame = loaded.value();
      EXPECT_EQ( frame.timestamp, dataset.stereoIndex()[ index ].timestamp );
      EXPECT_EQ( frame.left.width(), 4 );
      EXPECT_EQ( frame.left.height(), 3 );
      EXPECT_EQ( frame.left.channels(), 1 );
      EXPECT_EQ( frame.left.pixelType(), phad::sensor::PixelType::kUint8 );
      const auto left_pixels  = frame.left.pixels<std::uint8_t>();
      const auto right_pixels = frame.right.pixels<std::uint8_t>();
      ASSERT_TRUE( left_pixels.has_value() );
      ASSERT_TRUE( right_pixels.has_value() );
      EXPECT_EQ( left_pixels->size(), 12U );
      EXPECT_EQ( left_pixels->front(), left_value );
      EXPECT_EQ( right_pixels->front(), right_value );
      EXPECT_FALSE( frame.left.pixels<std::uint16_t>().has_value() );
    }

    auto first_load  = dataset.loadStereo( 1 );
    auto second_load = dataset.loadStereo( 1 );
    ASSERT_TRUE( first_load.hasValue() );
    ASSERT_TRUE( second_load.hasValue() );
    const auto first_pixels =
        first_load.value().left.pixels<std::uint8_t>();
    const auto second_pixels =
        second_load.value().left.pixels<std::uint8_t>();
    ASSERT_TRUE( first_pixels.has_value() );
    ASSERT_TRUE( second_pixels.has_value() );
    EXPECT_TRUE( std::ranges::equal( *first_pixels, *second_pixels ) );
    EXPECT_NE( first_pixels->data(), second_pixels->data() );
  }

  TEST( EurocDatasetTest, DefersCorruptPngFailureUntilStereoIsRequested )
  {
    EurocFixture fixture;
    {
      std::ofstream corrupt( fixture.sensorPath( "cam1", "data/right-b.png" ),
                             std::ios::binary | std::ios::trunc );
      corrupt << "not a png";
    }

    auto opened = EurocDataset::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    auto loaded = opened.value().loadStereo( 1 );

    ASSERT_FALSE( loaded.hasValue() );
    EXPECT_EQ( loaded.error().code,
               phad::dataset::DatasetErrorCode::kImageDecodeFailed );
    EXPECT_EQ( loaded.error().sensor_id, "cam1" );
    EXPECT_EQ( loaded.error().line_or_record_index, 1U );
    EXPECT_EQ( loaded.error().timestamp->nanoseconds(),
               EurocFixture::kFirstTimestamp + 50'000'000 );
  }

  void expectOpenError( const fs::path&                 root,
                        phad::dataset::DatasetErrorCode expected_code,
                        const std::string&              expected_sensor = {},
                        const std::string&              expected_field  = {} )
  {
    auto result = EurocDataset::open( root );
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

  TEST( EurocDatasetTest, RejectsMissingRootAndRequiredFiles )
  {
    EurocFixture fixture;
    expectOpenError( fixture.root() / "absent",
                     phad::dataset::DatasetErrorCode::kRootNotFound );

    fs::remove( fixture.sensorPath( "cam1", "sensor.yaml" ) );
    expectOpenError( fixture.root(),
                     phad::dataset::DatasetErrorCode::kRequiredFileMissing,
                     "cam1" );
  }

  TEST( EurocDatasetTest, RejectsCsvHeaderColumnAndFieldErrors )
  {
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", "timestamp,filename\n" );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kInvalidCsvHeader,
                       "cam0", "header" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "cam0",
          "#timestamp [ns],filename\n1403636579763555584,left-a.png,extra\n" );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kInvalidColumnCount,
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
                       phad::dataset::DatasetErrorCode::kInvalidField, "imu0",
                       "w_RS_S_x" );
    }
  }

  TEST( EurocDatasetTest, RejectsInvalidOverflowDuplicateAndReverseTimestamps )
  {
    const auto camera_csv = []( const std::string& rows ) {
      return "#timestamp [ns],filename\n" + rows;
    };
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", camera_csv( "1.5,left-a.png\n" ) );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kInvalidTimestamp,
                       "cam0", "timestamp" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0",
                        camera_csv( "9223372036854775808,left-a.png\n" ) );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kTimestampOverflow,
                       "cam0", "timestamp" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "cam0", camera_csv( "1403636579763555584,left-a.png\n"
                              "1403636579763555584,left-b.png\n" ) );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kDuplicateTimestamp,
                       "cam0", "timestamp" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "cam0", camera_csv( "1403636579813555584,left-b.png\n"
                              "1403636579763555584,left-a.png\n" ) );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kOutOfOrderTimestamp,
                       "cam0", "timestamp" );
    }
  }

  TEST( EurocDatasetTest, RejectsNonFiniteImuMeasurementsWithContext )
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
      auto result = EurocDataset::open( fixture.root() );
      ASSERT_FALSE( result.hasValue() );
      EXPECT_EQ( result.error().code,
                 phad::dataset::DatasetErrorCode::kNonFiniteMeasurement );
      EXPECT_EQ( result.error().sensor_id, "imu0" );
      EXPECT_EQ( result.error().line_or_record_index, 2U );
      EXPECT_EQ( result.error().timestamp->nanoseconds(),
                 EurocFixture::kFirstTimestamp );
      EXPECT_EQ( result.error().field, "w_RS_S_x" );
    }
  }

  TEST( EurocDatasetTest, RejectsDuplicateAndReverseImuTimestamps )
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
                       phad::dataset::DatasetErrorCode::kDuplicateTimestamp,
                       "imu0", "timestamp" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv(
          "imu0", header +
                      "1403636579813555584,0,0,0,0,0,1\n"
                      "1403636579763555584,0,0,0,0,0,1\n" );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kOutOfOrderTimestamp,
                       "imu0", "timestamp" );
    }
  }

  TEST( EurocDatasetTest, RejectsMissingOrMismatchedStereoRecords )
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
          phad::dataset::DatasetErrorCode::kStereoTimestampMismatch,
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
          phad::dataset::DatasetErrorCode::kStereoTimestampMismatch,
          "cam0/cam1", "timestamp" );
    }
  }

  TEST( EurocDatasetTest, RejectsUnsafeAndMissingManifestImagePaths )
  {
    const auto csv = []( const std::string& filename ) {
      return "#timestamp [ns],filename\n1403636579763555584," + filename + "\n";
    };
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", csv( "/tmp/image.png" ) );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kUnsafeImagePath, "cam0",
                       "filename" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", csv( "../left-a.png" ) );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kUnsafeImagePath, "cam0",
                       "filename" );
    }
    {
      EurocFixture fixture;
      fixture.writeCsv( "cam0", csv( "missing.png" ) );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kImageFileMissing, "cam0",
                       "filename" );
    }
  }

  TEST( EurocDatasetTest, RejectsInvalidTransformAndUnsupportedCameraModels )
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
                       phad::dataset::DatasetErrorCode::kInvalidCalibration,
                       "cam0", "T_BS.rotation" );
    }
    {
      EurocFixture fixture;
      fixture.writeSensorFile( "cam0",
                               EurocFixture::validCameraYaml( "omnidirectional" ) );
      expectOpenError(
          fixture.root(),
          phad::dataset::DatasetErrorCode::kUnsupportedCameraModel, "cam0",
          "camera_model" );
    }
    {
      EurocFixture fixture;
      fixture.writeSensorFile(
          "cam0", EurocFixture::validCameraYaml( "pinhole", "equidistant" ) );
      expectOpenError(
          fixture.root(),
          phad::dataset::DatasetErrorCode::kUnsupportedDistortionModel, "cam0",
          "distortion_model" );
    }
  }

  TEST( EurocDatasetTest, RejectsNonIdentityImuExtrinsicsInM1 )
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
        phad::dataset::DatasetErrorCode::kUnsupportedImuExtrinsics, "imu0",
        "T_BS" );
  }

  TEST( EurocDatasetTest, RejectsMissingAndInvalidImuNoiseFields )
  {
    {
      EurocFixture      fixture;
      std::string       yaml = EurocFixture::validImuYaml();
      const std::string line = "gyroscope_noise_density: 0.0001\n";
      yaml.erase( yaml.find( line ), line.size() );
      fixture.writeSensorFile( "imu0", yaml );
      expectOpenError( fixture.root(),
                       phad::dataset::DatasetErrorCode::kInvalidCalibration,
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
                       phad::dataset::DatasetErrorCode::kInvalidCalibration,
                       "imu0", "accelerometer_random_walk" );
    }
  }

  TEST( EurocDatasetTest, RejectsWrongImageDimensionsAndPixelTypesOnDecode )
  {
    {
      EurocFixture fixture;
      fixture.writeImage( "cam0", "left-b.png", 12, 5, 3 );
      auto opened = EurocDataset::open( fixture.root() );
      ASSERT_TRUE( opened.hasValue() );
      auto loaded = opened.value().loadStereo( 1 );
      ASSERT_FALSE( loaded.hasValue() );
      EXPECT_EQ( loaded.error().code,
                 phad::dataset::DatasetErrorCode::kImageFormatMismatch );
      EXPECT_EQ( loaded.error().field, "resolution" );
    }
    for ( const int type : { CV_8UC3, CV_16UC1 } )
    {
      EurocFixture fixture;
      fixture.writeImage( "cam0", "left-b.png", 12, 4, 3, type );
      auto opened = EurocDataset::open( fixture.root() );
      ASSERT_TRUE( opened.hasValue() );
      auto loaded = opened.value().loadStereo( 1 );
      ASSERT_FALSE( loaded.hasValue() );
      EXPECT_EQ( loaded.error().code,
                 phad::dataset::DatasetErrorCode::kImageFormatMismatch );
      EXPECT_EQ( loaded.error().field, "pixel_type" );
    }
  }

  TEST( EurocDatasetTest, RejectsOutOfRangeStereoIndex )
  {
    EurocFixture fixture;
    auto         opened = EurocDataset::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() );
    auto loaded = opened.value().loadStereo( 3 );
    ASSERT_FALSE( loaded.hasValue() );
    EXPECT_EQ( loaded.error().code,
               phad::dataset::DatasetErrorCode::kIndexOutOfRange );
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

  TEST( EurocDatasetTest, ResidentImageMemoryDoesNotGrowWithTraversedFrameCount )
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

    auto opened = EurocDataset::open( fixture.root() );
    ASSERT_TRUE( opened.hasValue() ) << opened.error().describe();
    {
      auto warmup = opened.value().loadStereo( 0 );
      ASSERT_TRUE( warmup.hasValue() );
    }
    const std::size_t resident_after_warmup = CurrentResidentBytes();
    for ( std::size_t index = 0; index < kFrameCount; ++index )
    {
      auto loaded = opened.value().loadStereo( index );
      ASSERT_TRUE( loaded.hasValue() ) << loaded.error().describe();
      const auto pixels = loaded.value().left.pixels<std::uint8_t>();
      ASSERT_TRUE( pixels.has_value() );
      EXPECT_EQ( pixels->front(), static_cast<std::uint8_t>( index ) );
    }
    const std::size_t resident_after_traversal = CurrentResidentBytes();

    // A retained cache would hold roughly 24 MiB for this fixture. Allow 8 MiB
    // for allocator/OpenCV working-set variation while still detecting growth
    // proportional to all decoded frames.
    EXPECT_LE( resident_after_traversal, resident_after_warmup + 8U * 1024U * 1024U );
  }
#endif

}  // namespace
