#include "phad/bench/code_identity.hpp"

#include <sys/wait.h>

#include <array>
#include <cstdio>
#include <memory>
#include <string>

/**
 * @file code_identity.cpp
 * @brief 本库唯一调用外部进程（git）的 TU。
 */

namespace phad::bench
{
  namespace
  {

    struct PipeCloser
    {
      void operator()( FILE* pipe ) const noexcept
      {
        if ( pipe != nullptr )
        {
          (void)pclose( pipe );
        }
      }
    };

    using PipePtr = std::unique_ptr<FILE, PipeCloser>;

    [[nodiscard]] std::string shellQuote( const std::filesystem::path& path )
    {
      std::string quoted = "'";
      for ( const char ch : path.string() )
      {
        if ( ch == '\'' )
        {
          quoted += "'\\''";
        }
        else
        {
          quoted += ch;
        }
      }
      quoted += '\'';
      return quoted;
    }

    struct CommandResult
    {
      int         exit_code = -1;
      std::string stdout_text;
    };

    [[nodiscard]] CommandResult runCommand( const std::string& command )
    {
      CommandResult result;
      PipePtr       pipe( popen( command.c_str(), "r" ) );
      if ( !pipe )
      {
        return result;
      }

      std::array<char, 256> buffer{};
      while ( std::fgets( buffer.data(), static_cast<int>( buffer.size() ),
                          pipe.get() ) != nullptr )
      {
        result.stdout_text += buffer.data();
      }

      const int status = pclose( pipe.release() );
      if ( status == -1 )
      {
        result.exit_code = -1;
        return result;
      }
      if ( !WIFEXITED( status ) )
      {
        result.exit_code = -1;
        return result;
      }
      result.exit_code = WEXITSTATUS( status );
      return result;
    }

    [[nodiscard]] std::string trimTrailingNewline( std::string text )
    {
      while ( !text.empty() &&
              ( text.back() == '\n' || text.back() == '\r' ) )
      {
        text.pop_back();
      }
      return text;
    }

  }  // namespace

  CodeIdentity queryGitIdentity( const std::filesystem::path& repo,
                                 std::vector<std::string>&    warnings )
  {
    CodeIdentity identity;
    if ( repo.empty() || !std::filesystem::exists( repo ) )
    {
      warnings.emplace_back(
          "git identity: repository path missing; using unknown commit" );
      return identity;
    }

    const std::string repo_q = shellQuote( repo );

    const CommandResult head = runCommand( "git -C " + repo_q + " rev-parse HEAD" );
    if ( head.exit_code != 0 )
    {
      warnings.emplace_back(
          "git identity: rev-parse HEAD failed; using unknown commit" );
      return identity;
    }
    const std::string commit = trimTrailingNewline( head.stdout_text );
    if ( commit.empty() )
    {
      warnings.emplace_back(
          "git identity: empty HEAD; using unknown commit" );
      return identity;
    }
    identity.git_commit = commit;

    const CommandResult short_head =
        runCommand( "git -C " + repo_q + " rev-parse --short HEAD" );
    const std::string short_commit =
        trimTrailingNewline( short_head.stdout_text );
    if ( short_head.exit_code == 0 && !short_commit.empty() )
    {
      identity.git_commit_short = short_commit;
    }
    else if ( commit.size() >= 7U )
    {
      identity.git_commit_short = commit.substr( 0, 7 );
    }

    const CommandResult branch =
        runCommand( "git -C " + repo_q + " rev-parse --abbrev-ref HEAD" );
    const std::string branch_name = trimTrailingNewline( branch.stdout_text );
    if ( branch.exit_code == 0 && !branch_name.empty() )
    {
      identity.git_branch = branch_name;
    }

    // dirty 只看 tracked：diff --quiet HEAD；退出码 1 = dirty，0 = clean。
    const CommandResult dirty =
        runCommand( "git -C " + repo_q + " diff --quiet HEAD" );
    if ( dirty.exit_code == 1 )
    {
      identity.git_dirty = true;
    }
    else if ( dirty.exit_code != 0 )
    {
      warnings.emplace_back(
          "git identity: diff --quiet HEAD failed; treating tree as clean" );
      identity.git_dirty = false;
    }

    return identity;
  }

}  // namespace phad::bench
