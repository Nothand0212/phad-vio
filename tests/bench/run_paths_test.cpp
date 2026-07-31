#include "phad/bench/run_paths.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace
{

  using phad::bench::CodeIdentity;
  using phad::bench::composeRunDir;
  using phad::bench::decideOverwrite;
  using phad::bench::OverwriteDecision;

  TEST( RunPathsTest, CleanCommitSegment )
  {
    CodeIdentity code;
    code.git_commit_short = "abcdef1";
    code.git_dirty        = false;
    const auto path =
        composeRunDir( "/tmp/bench", "MH_01_easy", code, "default", "9f2ab41c" );
    EXPECT_EQ( path, std::filesystem::path(
                         "/tmp/bench/MH_01_easy/abcdef1/default_9f2ab41c" ) );
  }

  TEST( RunPathsTest, DirtyCommitSegment )
  {
    CodeIdentity code;
    code.git_commit_short = "abcdef1";
    code.git_dirty        = true;
    const auto path =
        composeRunDir( "/tmp/bench", "MH_01_easy", code, "default", "9f2ab41c" );
    EXPECT_EQ(
        path, std::filesystem::path(
                  "/tmp/bench/MH_01_easy/abcdef1_dirty/default_9f2ab41c" ) );
  }

  TEST( RunPathsTest, UnknownCommitSegment )
  {
    CodeIdentity code;
    const auto   path =
        composeRunDir( "/tmp/bench", "MH_01_easy", code, "default", "9f2ab41c" );
    EXPECT_EQ( path, std::filesystem::path(
                         "/tmp/bench/MH_01_easy/unknown/default_9f2ab41c" ) );
  }

  TEST( RunPathsTest, EmptySequenceThrows )
  {
    CodeIdentity code;
    EXPECT_THROW(
        (void)composeRunDir( "/tmp/bench", "", code, "default", "9f2ab41c" ),
        std::invalid_argument );
  }

  class DecideOverwriteTest : public ::testing::Test
  {
  protected:
    void SetUp() override
    {
      m_root = std::filesystem::temp_directory_path() /
               "phad_bench_overwrite_test";
      std::filesystem::remove_all( m_root );
      std::filesystem::create_directories( m_root );
    }

    void TearDown() override
    {
      std::filesystem::remove_all( m_root );
    }

    void writeSummary()
    {
      std::ofstream( m_root / "summary.json" ) << "{}\n";
    }

    std::filesystem::path m_root;
  };

  TEST_F( DecideOverwriteTest, MissingSummaryWrites )
  {
    EXPECT_EQ( decideOverwrite( m_root, false, false ),
               OverwriteDecision::kWrite );
  }

  TEST_F( DecideOverwriteTest, CleanExistingRefuses )
  {
    writeSummary();
    EXPECT_EQ( decideOverwrite( m_root, false, false ),
               OverwriteDecision::kRefuse );
  }

  TEST_F( DecideOverwriteTest, CleanExistingForceOverwrites )
  {
    writeSummary();
    EXPECT_EQ( decideOverwrite( m_root, false, true ),
               OverwriteDecision::kOverwriteWithWarning );
  }

  TEST_F( DecideOverwriteTest, DirtyExistingOverwrites )
  {
    writeSummary();
    EXPECT_EQ( decideOverwrite( m_root, true, false ),
               OverwriteDecision::kOverwriteWithWarning );
  }

  TEST_F( DecideOverwriteTest, DirtyExistingForceOverwrites )
  {
    writeSummary();
    EXPECT_EQ( decideOverwrite( m_root, true, true ),
               OverwriteDecision::kOverwriteWithWarning );
  }

}  // namespace
