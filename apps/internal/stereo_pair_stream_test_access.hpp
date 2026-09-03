#pragma once

#ifndef PHAD_REPLAY_DIAGNOSTIC_TESTING
#error "stereo_pair_stream_test_access.hpp is available only to replay diagnostic tests"
#endif

#include <string>

#include "apps/stereo_pair_stream.hpp"

namespace phad::apps::internal
{

  class StereoPairStreamTestAccess
  {
  public:
    [[nodiscard]] static std::string canonicalStateJson(
        const StereoPairStream& stream );
  };

  [[nodiscard]] inline std::string canonicalStateJson(
      const StereoPairStream& stream )
  {
    return StereoPairStreamTestAccess::canonicalStateJson( stream );
  }

}  // namespace phad::apps::internal
