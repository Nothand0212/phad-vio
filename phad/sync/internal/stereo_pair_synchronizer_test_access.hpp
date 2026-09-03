#pragma once

#ifndef PHAD_REPLAY_DIAGNOSTIC_TESTING
#error "stereo_pair_synchronizer_test_access.hpp is available only to replay diagnostic tests"
#endif

#include <string>

#include "phad/sync/stereo_pair_synchronizer.hpp"

namespace phad::sync::internal
{

  class StereoPairSynchronizerTestAccess
  {
  public:
    [[nodiscard]] static std::string canonicalStateJson(
        const StereoPairSynchronizer& synchronizer );
  };

  [[nodiscard]] inline std::string canonicalStateJson(
      const StereoPairSynchronizer& synchronizer )
  {
    return StereoPairSynchronizerTestAccess::canonicalStateJson( synchronizer );
  }

}  // namespace phad::sync::internal
