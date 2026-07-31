#pragma once

#include <cstdint>
#include <filesystem>
#include <string_view>

#include "phad/bench/code_identity.hpp"

/**
 * @file run_paths.hpp
 * @brief bench run 目录模板与覆盖策略判定。
 *
 * 模板：
 * `<bench_root>/<sequence>/<commit_short>[_dirty]/<config_label>_<hash8>/`
 */

namespace phad::bench
{

  enum class OverwriteDecision : std::uint8_t
  {
    kWrite                = 0,
    kOverwriteWithWarning = 1,
    kRefuse               = 2,
  };

  [[nodiscard]] std::filesystem::path composeRunDir(
      const std::filesystem::path& bench_root, std::string_view sequence,
      const CodeIdentity& code, std::string_view config_label,
      std::string_view config_hash8 );

  /// 以 `run_dir/summary.json` 是否存在为哨兵；不写日志、不抛异常。
  [[nodiscard]] OverwriteDecision decideOverwrite(
      const std::filesystem::path& run_dir, bool git_dirty, bool force );

}  // namespace phad::bench
