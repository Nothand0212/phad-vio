#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

/**
 * @file code_identity.hpp
 * @brief 运行时代码身份（git commit / dirty / branch）。
 *
 * dirty 只看 tracked（`git diff --quiet HEAD`）；untracked 不计。
 * 查询失败不抛异常，返回 unknown 并追加 warning。
 */

namespace phad::bench
{

  struct CodeIdentity
  {
    std::optional<std::string> git_commit;  // 40 hex；失败时为空
    std::string                git_commit_short = "unknown";
    bool                       git_dirty        = false;
    std::optional<std::string> git_branch;
  };

  /// 运行时调用 git；失败时返回默认值并附 warning，不抛异常。
  [[nodiscard]] CodeIdentity queryGitIdentity(
      const std::filesystem::path& repo, std::vector<std::string>& warnings );

}  // namespace phad::bench
