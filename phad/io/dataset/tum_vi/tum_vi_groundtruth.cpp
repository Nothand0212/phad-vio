#include "phad/io/dataset/tum_vi/tum_vi_groundtruth.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "phad/io/dataset/internal/dataset_adapter_utils.hpp"

namespace phad::io::dataset::tum_vi
{
  namespace
  {

    namespace fs      = std::filesystem;
    namespace adapter = dataset::internal;

    constexpr std::string_view kSensorId = "gt_imu";
    constexpr std::string_view kHeader =
        "# timestamp[ns],tx,ty,tz,qw,qx,qy,qz";
    constexpr std::size_t                     kColumnCount             = 8U;
    constexpr double                          kUnitQuaternionTolerance = 1e-6;
    constexpr std::array<std::string_view, 7> kPoseFields{
        "tx", "ty", "tz", "qw", "qx", "qy", "qz" };

    DatasetError mapTrajectoryError( const common::TrajectoryError& error,
                                     const fs::path&                path )
    {
      const std::size_t line = error.index + 2U;
      switch ( error.code )
      {
        case common::TrajectoryErrorCode::kEmpty:
          return adapter::makeError( DatasetErrorCode::kEmptyStream,
                                     std::string{ kSensorId }, path, {},
                                     error.detail );
        case common::TrajectoryErrorCode::kNonFinitePose:
          return adapter::makeError(
              DatasetErrorCode::kNonFiniteMeasurement,
              std::string{ kSensorId }, path, "pose", error.detail, line );
        case common::TrajectoryErrorCode::kInvalidRotation:
          return adapter::makeError( DatasetErrorCode::kInvalidField,
                                     std::string{ kSensorId }, path,
                                     "q_W_B", error.detail, line );
        case common::TrajectoryErrorCode::kDuplicateTimestamp:
          return adapter::makeError(
              DatasetErrorCode::kDuplicateTimestamp,
              std::string{ kSensorId }, path, "timestamp", error.detail,
              line );
        case common::TrajectoryErrorCode::kOutOfOrderTimestamp:
          break;
      }
      return adapter::makeError( DatasetErrorCode::kOutOfOrderTimestamp,
                                 std::string{ kSensorId }, path, "timestamp",
                                 error.detail, line );
    }

    DatasetResult<std::vector<common::TimedPose>> parseGroundtruthCsv(
        const fs::path& csv_path )
    {
      std::ifstream stream( csv_path );
      if ( !stream )
      {
        return adapter::makeError( DatasetErrorCode::kIoError,
                                   std::string{ kSensorId }, csv_path, {},
                                   "failed to open groundtruth CSV" );
      }

      std::string line_text;
      if ( !std::getline( stream, line_text ) )
      {
        return adapter::makeError( DatasetErrorCode::kInvalidCsvHeader,
                                   std::string{ kSensorId }, csv_path,
                                   "header", "CSV is empty", 1U );
      }
      adapter::removeCarriageReturn( line_text );
      if ( line_text != kHeader )
      {
        return adapter::makeError(
            DatasetErrorCode::kInvalidCsvHeader, std::string{ kSensorId },
            csv_path, "header", "unexpected TUM-VI groundtruth CSV header",
            1U );
      }

      std::vector<common::TimedPose> poses;
      std::size_t                    line_number = 1U;
      while ( std::getline( stream, line_text ) )
      {
        ++line_number;
        adapter::removeCarriageReturn( line_text );
        const auto fields = adapter::splitCsv( line_text );
        if ( fields.size() != kColumnCount )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidColumnCount,
              std::string{ kSensorId }, csv_path, {},
              "groundtruth row must contain 8 columns", line_number );
        }

        auto timestamp = adapter::parseTimestamp(
            fields[ 0 ], std::string{ kSensorId }, csv_path, line_number );
        if ( !timestamp )
        {
          return timestamp.error();
        }
        if ( !poses.empty() )
        {
          if ( auto error = adapter::checkIncreasing(
                   poses.back().timestamp, timestamp.value(),
                   std::string{ kSensorId }, csv_path, line_number ) )
          {
            return *std::move( error );
          }
        }

        std::array<double, 7> values{};
        for ( std::size_t index = 0U; index < values.size(); ++index )
        {
          auto value = adapter::parseFiniteDouble(
              fields[ index + 1U ], std::string{ kSensorId }, csv_path,
              line_number, std::string{ kPoseFields[ index ] } );
          if ( !value )
          {
            DatasetError error = value.error();
            error.timestamp    = timestamp.value();
            return error;
          }
          values[ index ] = value.value();
        }

        const Eigen::Quaterniond rotation{ values[ 3 ], values[ 4 ],
                                           values[ 5 ], values[ 6 ] };
        if ( std::abs( rotation.norm() - 1.0 ) >
             kUnitQuaternionTolerance )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidField, std::string{ kSensorId },
              csv_path, "q_W_B", "quaternion must have unit norm",
              line_number, timestamp.value() );
        }

        common::TimedPose pose;
        pose.timestamp      = timestamp.value();
        pose.T_W_B.linear() = rotation.normalized().toRotationMatrix();
        pose.T_W_B.translation() =
            Eigen::Vector3d{ values[ 0 ], values[ 1 ], values[ 2 ] };
        poses.push_back( pose );
      }
      if ( stream.bad() )
      {
        return adapter::makeError( DatasetErrorCode::kIoError,
                                   std::string{ kSensorId }, csv_path, {},
                                   "failed while reading groundtruth CSV" );
      }
      return poses;
    }

  }  // namespace

  DatasetResult<common::Trajectory> openGroundtruth(
      const fs::path& sequence_root )
  {
    std::error_code ec;
    if ( !fs::is_directory( sequence_root, ec ) || ec )
    {
      return adapter::makeError(
          DatasetErrorCode::kRootNotFound, {}, sequence_root, {},
          ec ? ec.message() : "sequence root is not a directory" );
    }

    const fs::path dso_root = sequence_root / "dso";
    const fs::path csv_path = dso_root / "gt_imu.csv";
    for ( const auto& [ path, directory ] :
          std::array<std::pair<fs::path, bool>, 2>{
              std::pair{ dso_root, true }, std::pair{ csv_path, false } } )
    {
      if ( auto error = adapter::validateRequiredPath( path, directory ) )
      {
        error->sensor_id = kSensorId;
        return *std::move( error );
      }
    }

    auto poses = parseGroundtruthCsv( csv_path );
    if ( !poses )
    {
      return poses.error();
    }

    auto trajectory = common::Trajectory::create( std::move( poses ).value() );
    if ( !trajectory )
    {
      return mapTrajectoryError( trajectory.error(), csv_path );
    }
    return std::move( trajectory ).value();
  }

}  // namespace phad::io::dataset::tum_vi
