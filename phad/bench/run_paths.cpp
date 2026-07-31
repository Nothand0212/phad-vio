#include "phad/bench/run_paths.hpp"

#include <stdexcept>
#include <string>

/**
 * @file run_paths.cpp
 * @brief run 目录拼接与覆盖策略。
 */

namespace phad::bench
{

  std::filesystem::path composeRunDir( const std::filesystem::path& bench_root,
                                       std::string_view             sequence,
                                       const CodeIdentity&          code,
                                       std::string_view             config_label,
                                       std::string_view             config_hash8 )
  {
    if ( bench_root.empty() )
    {
      throw std::invalid_argument( "composeRunDir: bench_root must be non-empty" );
    }
    if ( sequence.empty() )
    {
      throw std::invalid_argument( "composeRunDir: sequence must be non-empty" );
    }
    if ( config_label.empty() )
    {
      throw std::invalid_argument(
          "composeRunDir: config_label must be non-empty" );
    }
    if ( config_hash8.empty() )
    {
      throw std::invalid_argument(
          "composeRunDir: config_hash8 must be non-empty" );
    }

    std::string commit_segment = code.git_commit_short.empty()
                                     ? std::string( "unknown" )
                                     : code.git_commit_short;
    if ( code.git_dirty )
    {
      commit_segment += "_dirty";
    }

    const std::string config_segment =
        std::string( config_label ) + '_' + std::string( config_hash8 );

    return bench_root / std::string( sequence ) / commit_segment /
           config_segment;
  }

  OverwriteDecision decideOverwrite( const std::filesystem::path& run_dir,
                                     bool git_dirty, bool force )
  {
    const std::filesystem::path sentinel = run_dir / "summary.json";
    if ( !std::filesystem::exists( sentinel ) )
    {
      return OverwriteDecision::kWrite;
    }
    if ( force || git_dirty )
    {
      return OverwriteDecision::kOverwriteWithWarning;
    }
    return OverwriteDecision::kRefuse;
  }

}  // namespace phad::bench
