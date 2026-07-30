#include "phad/io/dataset/euroc/euroc_groundtruth.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "phad/io/dataset/euroc/internal/euroc_yaml.hpp"
#include "phad/io/dataset/internal/dataset_adapter_utils.hpp"

/**
 * @file euroc_groundtruth.cpp
 * @brief EuRoC 真值轨迹加载实现。
 */

namespace phad::io::dataset::euroc
{
  namespace
  {

    namespace fs      = std::filesystem;
    namespace adapter = dataset::internal;

    const std::string kSensorId = "state_groundtruth_estimate0";

    constexpr std::string_view kGroundtruthHeader =
        "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], "
        "q_RS_x [], q_RS_y [], q_RS_z [], v_RS_R_x [m s^-1], "
        "v_RS_R_y [m s^-1], v_RS_R_z [m s^-1], b_w_RS_S_x [rad s^-1], "
        "b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1], b_a_RS_S_x [m s^-2], "
        "b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]";

    constexpr std::size_t kColumnCount = 17U;
    // EuRoC 真值的四元数未严格归一化：MH_01 最大偏差约 7e-5，V2_02 约
    // 1.3e-4。该检查的目的是识别损坏的记录（全零、列错位），不是校核
    // 数据集的数值精度，因此阈值取 1e-3，读入后统一归一化。
    constexpr double kUnitQuaternionTolerance = 1e-3;

    /// 位姿列的名称，顺序与 CSV 中第 1..7 列一致。
    constexpr std::array<std::string_view, 7> kPoseFields{
        "p_RS_R_x", "p_RS_R_y", "p_RS_R_z", "q_RS_w",
        "q_RS_x", "q_RS_y", "q_RS_z" };

    DatasetError mapTrajectoryError( const common::TrajectoryError& error,
                                     const fs::path&                path )
    {
      const std::size_t line = error.index + 2U;
      switch ( error.code )
      {
        case common::TrajectoryErrorCode::kEmpty:
          return adapter::makeError( DatasetErrorCode::kEmptyStream, kSensorId,
                                     path, {}, error.detail );
        case common::TrajectoryErrorCode::kNonFinitePose:
          return adapter::makeError( DatasetErrorCode::kNonFiniteMeasurement,
                                     kSensorId, path, "pose", error.detail,
                                     line );
        case common::TrajectoryErrorCode::kInvalidRotation:
          return adapter::makeError( DatasetErrorCode::kInvalidField, kSensorId,
                                     path, "q_RS", error.detail, line );
        case common::TrajectoryErrorCode::kDuplicateTimestamp:
          return adapter::makeError( DatasetErrorCode::kDuplicateTimestamp,
                                     kSensorId, path, "timestamp",
                                     error.detail, line );
        case common::TrajectoryErrorCode::kOutOfOrderTimestamp:
          break;
      }
      return adapter::makeError( DatasetErrorCode::kOutOfOrderTimestamp,
                                 kSensorId, path, "timestamp", error.detail,
                                 line );
    }

    DatasetResult<sensor::RigidTransform> parseGroundtruthExtrinsics(
        const fs::path& path )
    {
      auto yaml = adapter::loadYaml( path, kSensorId );
      if ( !yaml )
      {
        return yaml.error();
      }
      auto transform =
          internal::parseSensorTransform( yaml.value(), kSensorId, path );
      if ( !transform )
      {
        return transform.error();
      }
      if ( !adapter::isIdentity( transform.value() ) )
      {
        return adapter::makeError(
            DatasetErrorCode::kUnsupportedImuExtrinsics, kSensorId, path,
            "T_BS",
            "EuRoC adapter requires identity groundtruth extrinsics so the "
            "trajectory already expresses T_W_B" );
      }
      return std::move( transform ).value();
    }

    DatasetResult<std::vector<common::TimedPose>> parseGroundtruthCsv(
        const fs::path& csv_path )
    {
      std::ifstream stream( csv_path );
      if ( !stream )
      {
        return adapter::makeError( DatasetErrorCode::kIoError, kSensorId,
                                   csv_path, {},
                                   "failed to open groundtruth CSV" );
      }

      std::string line_text;
      if ( !std::getline( stream, line_text ) )
      {
        return adapter::makeError( DatasetErrorCode::kInvalidCsvHeader,
                                   kSensorId, csv_path, "header",
                                   "CSV is empty", 1 );
      }
      adapter::removeCarriageReturn( line_text );
      if ( line_text != kGroundtruthHeader )
      {
        return adapter::makeError( DatasetErrorCode::kInvalidCsvHeader,
                                   kSensorId, csv_path, "header",
                                   "unexpected groundtruth CSV header", 1 );
      }

      std::vector<common::TimedPose> poses;
      std::size_t                    line_number = 1;
      while ( std::getline( stream, line_text ) )
      {
        ++line_number;
        adapter::removeCarriageReturn( line_text );
        const auto fields = adapter::splitCsv( line_text );
        if ( fields.size() != kColumnCount )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidColumnCount, kSensorId, csv_path, {},
              "groundtruth row must contain 17 columns", line_number );
        }
        auto timestamp = adapter::parseTimestamp( fields[ 0 ], kSensorId,
                                                  csv_path, line_number );
        if ( !timestamp )
        {
          return timestamp.error();
        }
        if ( !poses.empty() )
        {
          if ( auto error = adapter::checkIncreasing(
                   poses.back().timestamp, timestamp.value(), kSensorId,
                   csv_path, line_number ) )
          {
            return *std::move( error );
          }
        }

        std::array<double, 7> values{};
        for ( std::size_t index = 0; index < values.size(); ++index )
        {
          auto value = adapter::parseFiniteDouble(
              fields[ index + 1U ], kSensorId, csv_path, line_number,
              std::string{ kPoseFields[ index ] } );
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
        if ( std::abs( rotation.norm() - 1.0 ) > kUnitQuaternionTolerance )
        {
          return adapter::makeError(
              DatasetErrorCode::kInvalidField, kSensorId, csv_path, "q_RS",
              "quaternion must have unit norm", line_number,
              timestamp.value() );
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
        return adapter::makeError( DatasetErrorCode::kIoError, kSensorId,
                                   csv_path, {},
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

    const fs::path groundtruth_root =
        sequence_root / "mav0" / kSensorId;
    const fs::path yaml_path = groundtruth_root / "sensor.yaml";
    const fs::path csv_path  = groundtruth_root / "data.csv";
    for ( const auto& [ path, directory ] :
          std::array<std::pair<fs::path, bool>, 3>{
              std::pair{ groundtruth_root, true },
              std::pair{ yaml_path, false }, std::pair{ csv_path, false } } )
    {
      if ( auto error = adapter::validateRequiredPath( path, directory ) )
      {
        error->sensor_id = kSensorId;
        return *std::move( error );
      }
    }

    if ( auto extrinsics = parseGroundtruthExtrinsics( yaml_path );
         !extrinsics )
    {
      return extrinsics.error();
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

}  // namespace phad::io::dataset::euroc
