#include "tests/apps/m5_51_replay_diagnostic_cli_fixture.hpp"

#include <openssl/evp.h>

#include <array>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <string_view>

namespace phad::apps::testing
{
  namespace
  {
    constexpr std::string_view kFixtureMarker =
        "phad.m5_51.cli_fixture.v1";

    [[nodiscard]] std::string sha256( const std::string_view bytes )
    {
      std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
      std::size_t                                digest_size = digest.size();
      if ( EVP_Q_digest( nullptr, "SHA256", nullptr, bytes.data(),
                         bytes.size(), digest.data(), &digest_size ) != 1 ||
           digest_size != 32U )
      {
        throw std::runtime_error( "cannot compute fixture SHA-256" );
      }

      constexpr char kHex[] = "0123456789abcdef";
      std::string    encoded;
      encoded.reserve( digest_size * 2U );
      for ( std::size_t index = 0U; index < digest_size; ++index )
      {
        const unsigned char value = digest[ index ];
        encoded.push_back( kHex[ value >> 4U ] );
        encoded.push_back( kHex[ value & 0x0fU ] );
      }
      return encoded;
    }

    [[nodiscard]] internal::M5RpeCanonicalState canonicalState(
        const nlohmann::ordered_json& state )
    {
      const std::string canonical_json = state.dump();
      return internal::M5RpeCanonicalState{
          .m_canonical_json = canonical_json,
          .m_sha256         = sha256( canonical_json ) };
    }

    [[nodiscard]] internal::M5RpeCanonicalState readCanonicalState(
        const nlohmann::ordered_json& wrapped )
    {
      return internal::M5RpeCanonicalState{
          .m_canonical_json = wrapped.at( "canonical_json" ).dump(),
          .m_sha256         = wrapped.at( "sha256" ).get<std::string>() };
    }
  }  // namespace

  OfflineVoSessionResult runM551ReplayDiagnosticCliFixture(
      const std::filesystem::path&            baseline_state_receipt,
      const internal::M5RpePrefixArm          arm,
      internal::M5RpePrefixDiagnosticReceipt& receipt )
  {
    OfflineVoSessionResult result;
    result.counts.image_frames            = 43U;
    result.counts.ok                      = 43U;
    result.counts.segments                = 1U;
    result.counts.total_keyframes         = 2U;
    result.counts.total_track_only_frames = 41U;

    try
    {
      if ( arm != internal::M5RpePrefixArm::kSelectorWarmup )
      {
        throw std::runtime_error( "fixture requires selector-warmup arm" );
      }
      std::ifstream input( baseline_state_receipt, std::ios::binary );
      if ( !input )
      {
        throw std::runtime_error( "cannot read baseline state receipt" );
      }
      const std::string            bytes{ std::istreambuf_iterator<char>( input ),
                               std::istreambuf_iterator<char>() };
      const nlohmann::ordered_json baseline =
          nlohmann::ordered_json::parse( bytes );

      receipt.m_hook_executed    = true;
      receipt.m_hook_frame_index = 6U;
      receipt.m_pre_hook_state_v1 =
          readCanonicalState( baseline.at( "pre_hook_state_v1" ) );

      nlohmann::ordered_json post_hook =
          baseline.at( "pre_hook_state_v1" ).at( "canonical_json" );
      post_hook[ "keyframe_selector" ][ "total_keyframes" ] = 2U;
      receipt.m_post_hook_state_v1                          = canonicalState( post_hook );

      nlohmann::ordered_json post_state =
          baseline.at( "post_state_v1" ).at( "canonical_json" );
      post_state[ "keyframe_selector" ][ "total_keyframes" ] = 2U;
      post_state[ "session" ][ "result" ][ "diag" ][ 7 ]
                [ "is_keyframe" ] = false;
      receipt.m_post_state_v1     = canonicalState( post_state );
      receipt.m_hook_diff_json =
          R"([{"op":"replace","path":"/keyframe_selector/total_keyframes","value":2}])";
    }
    catch ( const std::exception& exception )
    {
      receipt.m_failure_reason = std::string{ kFixtureMarker } + ": " +
                                 exception.what();
    }
    return result;
  }

}  // namespace phad::apps::testing
