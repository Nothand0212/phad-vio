#include <gtest/gtest.h>

#include <cstdlib>
#include <nlohmann/json.hpp>
#include <utility>

#include "apps/internal/m5_51_prefix_diagnostic.hpp"
#include "apps/offline_vo_session.hpp"
#include "phad/io/dataset/euroc/euroc_dataset.hpp"

namespace
{

  TEST( M551ReplayDiagnosticSessionTest,
        Mh01SelectorWarmupChangesOnlyThePostRootCounter )
  {
    const char* configured_path = std::getenv( "PHAD_EUROC_MH01_PATH" );
    if ( configured_path == nullptr || configured_path[ 0 ] == '\0' )
    {
      GTEST_SKIP() << "PHAD_EUROC_MH01_PATH is not set";
    }

    phad::apps::OfflineVoSessionOptions options;
    options.max_frames                          = 8;
    options.collect_timing                      = false;
    options.estimator.m_enable_moving_bootstrap = true;

    phad::apps::internal::M5RpePrefixDiagnosticReceipt baseline_receipt;
    auto                                               baseline_dataset =
        phad::io::dataset::euroc::open( configured_path );
    ASSERT_TRUE( baseline_dataset ) << baseline_dataset.error().describe();
    const auto baseline = phad::apps::internal::runM5RpePrefixDiagnostic(
        std::move( baseline_dataset ).value(), options,
        phad::apps::internal::M5RpePrefixArm::kBaseline, baseline_receipt );

    phad::apps::internal::M5RpePrefixDiagnosticReceipt intervention_receipt;
    auto                                               intervention_dataset =
        phad::io::dataset::euroc::open( configured_path );
    ASSERT_TRUE( intervention_dataset )
        << intervention_dataset.error().describe();
    const auto intervention = phad::apps::internal::runM5RpePrefixDiagnostic(
        std::move( intervention_dataset ).value(), options,
        phad::apps::internal::M5RpePrefixArm::kSelectorWarmup,
        intervention_receipt );

    ASSERT_FALSE( baseline.error.has_value() ) << baseline.error->detail;
    ASSERT_FALSE( intervention.error.has_value() )
        << intervention.error->detail;
    ASSERT_TRUE( baseline_receipt.m_hook_executed );
    ASSERT_TRUE( intervention_receipt.m_hook_executed );
    EXPECT_EQ( baseline_receipt.m_pre_hook_state_v1.m_sha256,
               intervention_receipt.m_pre_hook_state_v1.m_sha256 );
    EXPECT_EQ( baseline_receipt.m_pre_hook_state_v1.m_canonical_json,
               intervention_receipt.m_pre_hook_state_v1.m_canonical_json );

    const auto baseline_hook_diff =
        nlohmann::json::parse( baseline_receipt.m_hook_diff_json );
    EXPECT_TRUE( baseline_hook_diff.empty() );
    const auto intervention_hook_diff =
        nlohmann::json::parse( intervention_receipt.m_hook_diff_json );
    ASSERT_EQ( intervention_hook_diff.size(), 1U );
    EXPECT_EQ( intervention_hook_diff.at( 0 ).at( "op" ), "replace" );
    EXPECT_EQ( intervention_hook_diff.at( 0 ).at( "path" ),
               "/keyframe_selector/total_keyframes" );
    EXPECT_EQ( intervention_hook_diff.at( 0 ).at( "value" ), 2U );

    const auto baseline_pre = nlohmann::json::parse(
        baseline_receipt.m_pre_hook_state_v1.m_canonical_json );
    const auto baseline_after = nlohmann::json::parse(
        baseline_receipt.m_post_hook_state_v1.m_canonical_json );
    const auto intervention_after = nlohmann::json::parse(
        intervention_receipt.m_post_hook_state_v1.m_canonical_json );
    EXPECT_EQ(
        baseline_pre.at( "keyframe_selector" ).at( "total_keyframes" ),
        1U );
    EXPECT_EQ( baseline_after, baseline_pre );
    EXPECT_EQ( intervention_after.at( "keyframe_selector" )
                   .at( "total_keyframes" ),
               2U );

    ASSERT_EQ( baseline.diag.size(), 8U );
    ASSERT_EQ( intervention.diag.size(), 8U );
    for ( std::size_t index = 0U; index <= 6U; ++index )
    {
      EXPECT_EQ( baseline.diag[ index ].status,
                 intervention.diag[ index ].status )
          << "diag_index=" << index;
      EXPECT_EQ( baseline.diag[ index ].is_keyframe,
                 intervention.diag[ index ].is_keyframe )
          << "diag_index=" << index;
      EXPECT_EQ( baseline.diag[ index ].initialization.m_path,
                 intervention.diag[ index ].initialization.m_path )
          << "diag_index=" << index;
      EXPECT_EQ( baseline.diag[ index ].initialization.m_phase,
                 intervention.diag[ index ].initialization.m_phase )
          << "diag_index=" << index;
      EXPECT_EQ( baseline.diag[ index ].initialization.m_outcome,
                 intervention.diag[ index ].initialization.m_outcome )
          << "diag_index=" << index;
      EXPECT_EQ( baseline.diag[ index ].initialization.m_transaction,
                 intervention.diag[ index ].initialization.m_transaction )
          << "diag_index=" << index;
    }
    EXPECT_TRUE( baseline.diag[ 7 ].is_keyframe );
    EXPECT_FALSE( intervention.diag[ 7 ].is_keyframe );
  }

}  // namespace
