#include "phad/io/dataset/euroc/internal/euroc_yaml.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <string_view>
#include <utility>

#include "phad/io/dataset/internal/dataset_adapter_utils.hpp"

/**
 * @file euroc_yaml.cpp
 * @brief EuRoC T_BS 解析实现。
 */

namespace phad::io::dataset::euroc::internal
{
  namespace
  {

    namespace fs      = std::filesystem;
    namespace adapter = dataset::internal;

    std::string transformSourceField( const sensor::CalibrationError& error,
                                      std::string_view                base )
    {
      if ( error.field_path == "rigid_transform.rotation" )
      {
        return std::string{ base } + ".rotation";
      }
      if ( error.field_path == "rigid_transform.bottom_row" )
      {
        return std::string{ base } + ".bottom_row";
      }
      return std::string{ base };
    }

  }  // namespace

  DatasetResult<sensor::RigidTransform> parseSensorTransform(
      const YAML::Node& root, const std::string& sensor_id,
      const fs::path& path )
  {
    try
    {
      const YAML::Node transform = root[ "T_BS" ];
      if ( !transform || transform[ "rows" ].as<int>() != 4 ||
           transform[ "cols" ].as<int>() != 4 )
      {
        return adapter::makeError( DatasetErrorCode::kInvalidCalibration,
                                   sensor_id, path, "T_BS",
                                   "T_BS must be a 4x4 matrix" );
      }
      const YAML::Node data = transform[ "data" ];
      if ( !data.IsSequence() || data.size() != 16U )
      {
        return adapter::makeError( DatasetErrorCode::kInvalidCalibration,
                                   sensor_id, path, "T_BS.data",
                                   "T_BS data must contain 16 values" );
      }

      Eigen::Matrix4d matrix;
      for ( std::size_t index = 0; index < 16U; ++index )
      {
        const Eigen::Index row    = static_cast<Eigen::Index>( index / 4U );
        const Eigen::Index column = static_cast<Eigen::Index>( index % 4U );
        matrix( row, column )     = data[ index ].as<double>();
      }
      auto result = sensor::RigidTransform::create( std::move( matrix ) );
      if ( !result )
      {
        return adapter::mapCalibrationError(
            result.error(), sensor_id, path,
            transformSourceField( result.error(), "T_BS" ) );
      }
      return std::move( result ).value();
    }
    catch ( const YAML::Exception& exception )
    {
      return adapter::makeError( DatasetErrorCode::kInvalidCalibration,
                                 sensor_id, path, "T_BS", exception.what() );
    }
  }

}  // namespace phad::io::dataset::euroc::internal
