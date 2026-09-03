#pragma once

#ifndef PHAD_FRONTEND_TESTING
#error "stereo_tracker_test_access.hpp is available only to frontend tests"
#endif

#include <string>

namespace phad::frontend
{
  class StereoTracker;

  namespace internal
  {

    class StereoTrackerTestAccess
    {
    public:
      [[nodiscard]] static std::string canonicalStateJson(
          const StereoTracker& tracker );
    };

  }  // namespace internal
}  // namespace phad::frontend
