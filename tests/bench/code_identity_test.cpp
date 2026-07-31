#include "phad/bench/code_identity.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <vector>

namespace
{

  using phad::bench::queryGitIdentity;

  TEST( CodeIdentityTest, MissingRepoReturnsUnknownWithWarning )
  {
    const std::filesystem::path missing =
        std::filesystem::temp_directory_path() /
        "phad_bench_missing_repo_does_not_exist";
    std::filesystem::remove_all( missing );

    std::vector<std::string> warnings;
    const auto               identity = queryGitIdentity( missing, warnings );
    EXPECT_FALSE( identity.git_commit.has_value() );
    EXPECT_EQ( identity.git_commit_short, "unknown" );
    EXPECT_FALSE( identity.git_dirty );
    ASSERT_FALSE( warnings.empty() );
    EXPECT_NE( warnings.front().find( "missing" ), std::string::npos );
  }

}  // namespace
