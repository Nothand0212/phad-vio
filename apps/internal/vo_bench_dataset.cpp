#include "apps/internal/vo_bench_dataset.hpp"

#include "phad/io/dataset/euroc/euroc_dataset.hpp"
#include "phad/io/dataset/euroc/euroc_groundtruth.hpp"
#include "phad/io/dataset/tum_vi/tum_vi_dataset.hpp"
#include "phad/io/dataset/tum_vi/tum_vi_groundtruth.hpp"

namespace phad::apps::internal
{

  io::dataset::DatasetResult<io::dataset::StereoImuDataset>
  openVoBenchDataset( const VoBenchDatasetFormat   format,
                      const std::filesystem::path& sequence_root )
  {
    if ( format == VoBenchDatasetFormat::kEuroc )
    {
      return io::dataset::euroc::open( sequence_root );
    }
    return io::dataset::tum_vi::open( sequence_root );
  }

  io::dataset::DatasetResult<common::Trajectory>
  openVoBenchGroundtruth( const VoBenchDatasetFormat   format,
                          const std::filesystem::path& sequence_root )
  {
    if ( format == VoBenchDatasetFormat::kEuroc )
    {
      return io::dataset::euroc::openGroundtruth( sequence_root );
    }
    return io::dataset::tum_vi::openGroundtruth( sequence_root );
  }

  std::string_view voBenchDatasetConfigToken(
      const VoBenchDatasetFormat format ) noexcept
  {
    return format == VoBenchDatasetFormat::kEuroc ? "euroc" : "tum_vi";
  }

}  // namespace phad::apps::internal
