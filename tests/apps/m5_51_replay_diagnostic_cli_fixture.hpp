#pragma once

#if !defined( PHAD_M5_51_REPLAY_DIAGNOSTIC_CLI_FIXTURE_TESTING )
#error "M5 #51 CLI fixture is available only in its dedicated test executable"
#endif

#include <filesystem>

#include "apps/internal/m5_51_prefix_diagnostic.hpp"
#include "apps/offline_vo_session.hpp"

namespace phad::apps::testing
{

  [[nodiscard]] OfflineVoSessionResult runM551ReplayDiagnosticCliFixture(
      const std::filesystem::path&            baseline_state_receipt,
      internal::M5RpePrefixArm                arm,
      internal::M5RpePrefixDiagnosticReceipt& receipt );

}  // namespace phad::apps::testing
