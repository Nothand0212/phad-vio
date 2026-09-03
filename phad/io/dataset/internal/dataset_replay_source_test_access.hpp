#pragma once

#ifndef PHAD_REPLAY_DIAGNOSTIC_TESTING
#error "dataset_replay_source_test_access.hpp is available only to replay diagnostic tests"
#endif

#include <string>

#include "phad/io/dataset/dataset_replay_source.hpp"

namespace phad::io::dataset::internal
{

  class DatasetReplaySourceTestAccess
  {
  public:
    [[nodiscard]] static std::string canonicalStateJson(
        const DatasetReplaySource& source );
  };

  [[nodiscard]] inline std::string canonicalStateJson(
      const DatasetReplaySource& source )
  {
    return DatasetReplaySourceTestAccess::canonicalStateJson( source );
  }

}  // namespace phad::io::dataset::internal
