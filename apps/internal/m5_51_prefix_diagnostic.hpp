#pragma once

#if !defined( PHAD_REPLAY_DIAGNOSTIC_TESTING )
#error "M5 #51 prefix diagnostics are available only in diagnostic test builds"
#endif

#include <cstdint>
#include <string>

#include "apps/offline_vo_session.hpp"

namespace phad::apps::internal
{

  enum class M5RpePrefixArm : std::uint8_t
  {
    kBaseline,
    kSelectorWarmup
  };

  struct M5RpeCanonicalState
  {
    std::string m_canonical_json;
    std::string m_sha256;
  };

  struct M5RpePrefixDiagnosticReceipt
  {
    bool                m_hook_executed    = false;
    std::uint64_t       m_hook_frame_index = 6U;
    M5RpeCanonicalState m_pre_hook_state_v1;
    M5RpeCanonicalState m_post_hook_state_v1;
    M5RpeCanonicalState m_post_state_v1;
    std::string         m_hook_diff_json = "[]";
    std::string         m_failure_reason;
  };

  [[nodiscard]] OfflineVoSessionResult runM5RpePrefixDiagnostic(
      io::dataset::StereoImuDataset  dataset,
      const OfflineVoSessionOptions& options,
      M5RpePrefixArm                 arm,
      M5RpePrefixDiagnosticReceipt&  receipt );

  [[nodiscard]] std::string m5RpePrefixDiagnosticReceiptJson(
      const M5RpePrefixDiagnosticReceipt& receipt );

}  // namespace phad::apps::internal
