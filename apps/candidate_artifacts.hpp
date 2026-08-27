#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include "apps/candidate_pipeline.hpp"
#include "apps/offline_vo_session.hpp"

/**
 * @file candidate_artifacts.hpp
 * @brief M4.4 P2a: candidate-owned twin 的独立 artifacts 写入。
 *
 * 只写 candidate 自己的 trajectory/KF trajectory 之外的 strict diagnostics
 * 与 ownership meta；TUM 由 bench（链接 phad_eval 侧）用 writeTum 写。
 * I/O 错误返回 SessionError，升级为 bench error，不混入 estimator
 * transaction（设计 §8.3）。
 */

namespace phad::apps
{

  /// candidate diagnostics CSV：每行 = 送入 candidate 的一个 frame（含
  /// terminal failure 行）。schema 固定，供
  /// scripts/candidate_pipeline_probe.py 校验。
  [[nodiscard]] std::optional<SessionError> writeCandidateDiagCsv(
      const std::filesystem::path&                   path,
      const std::vector<CandidateFrameDiagnostics>&  rows );

  /// candidate ownership/meta JSON：显式记录 ownership=full_pipeline 与
  /// terminal/failed/计数；wall_s 为 candidate 单独计时，不混 production
  /// stage timing（设计 §9.3 门 6）。
  [[nodiscard]] std::optional<SessionError> writeCandidateMeta(
      const std::filesystem::path& path, const CandidateRunResult& run,
      double candidate_wall_s );

}  // namespace phad::apps
