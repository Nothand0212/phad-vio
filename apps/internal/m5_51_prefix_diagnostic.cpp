#include "apps/internal/m5_51_prefix_diagnostic.hpp"

#include <nlohmann/json.hpp>

namespace phad::apps::internal
{
  namespace
  {
    [[nodiscard]] nlohmann::ordered_json canonicalStateJson(
        const M5RpeCanonicalState& state )
    {
      nlohmann::ordered_json encoded;
      encoded[ "sha256" ]         = state.m_sha256;
      encoded[ "canonical_json" ] = nlohmann::ordered_json::parse(
          state.m_canonical_json.empty() ? "null" : state.m_canonical_json );
      return encoded;
    }
  }  // namespace

  std::string m5RpePrefixDiagnosticReceiptJson(
      const M5RpePrefixDiagnosticReceipt& receipt )
  {
    nlohmann::ordered_json encoded;
    encoded[ "schema" ]           = "m5_51_prefix_diagnostic_receipt_v1";
    encoded[ "hook_executed" ]    = receipt.m_hook_executed;
    encoded[ "hook_frame_index" ] = receipt.m_hook_frame_index;
    encoded[ "pre_hook_state_v1" ] =
        canonicalStateJson( receipt.m_pre_hook_state_v1 );
    encoded[ "post_hook_state_v1" ] =
        canonicalStateJson( receipt.m_post_hook_state_v1 );
    encoded[ "post_state_v1" ] =
        canonicalStateJson( receipt.m_post_state_v1 );
    encoded[ "hook_diff" ] = nlohmann::ordered_json::parse(
        receipt.m_hook_diff_json.empty() ? "[]" : receipt.m_hook_diff_json );
    encoded[ "failure_reason" ] = receipt.m_failure_reason;
    return encoded.dump();
  }

}  // namespace phad::apps::internal
