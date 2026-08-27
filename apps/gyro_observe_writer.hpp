#pragma once

#include <filesystem>
#include <optional>
#include <string_view>

#include "apps/offline_vo_session.hpp"

/**
 * @file gyro_observe_writer.hpp
 * @brief Q1 Observe CSV 的 app 层原子发布接口。
 */

namespace phad::apps
{

  [[nodiscard]] std::string_view gyroPacketStatusName(
      GyroPacketStatus status ) noexcept;

  [[nodiscard]] std::optional<SessionError> writeGyroObserveCsvs(
      const std::filesystem::path& packets_path,
      const std::filesystem::path& samples_path,
      const GyroObserveArtifacts&  artifacts );

}  // namespace phad::apps
