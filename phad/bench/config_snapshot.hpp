#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <variant>

/**
 * @file config_snapshot.hpp
 * @brief 有序 key→scalar 配置快照，供 config_hash 与 meta.json 使用。
 *
 * phad::bench 不认识 frontend / estimator / eval 类型；展平由 apps 侧完成。
 */

namespace phad::bench
{

  class ConfigSnapshot
  {
  public:
    using Value = std::variant<std::string, double, std::int64_t, bool>;

    void set( std::string key, std::string value );
    void set( std::string key, double value );
    void set( std::string key, std::int64_t value );
    void set( std::string key, bool value );

    /// key 字典序、浮点 "%.17g"、bool true/false、每行 "key=value"。
    [[nodiscard]] std::string canonicalText() const;

    /// canonicalText() 的 FNV-1a 64，十六进制前 8 位。
    [[nodiscard]] std::string hash8() const;

    /// JSON object 文本；public API 不暴露 nlohmann 类型。
    [[nodiscard]] std::string toJson() const;

    [[nodiscard]] bool empty() const noexcept
    {
      return m_values.empty();
    }

    [[nodiscard]] std::size_t size() const noexcept
    {
      return m_values.size();
    }

  private:
    std::map<std::string, Value> m_values;
  };

}  // namespace phad::bench
