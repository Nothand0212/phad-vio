#pragma once

#include <compare>
#include <cstdint>

/**
 * @file timestamp.hpp
 * @brief Timestamp 类型用于表示和操作以纳秒为单位的时间戳。
 *
 * 该文件定义了 phad::common::Timestamp 类，封装了 64 位整数纳秒时间戳的存储与基本比较操作。
 * 常用于传感器数据、数据集帧等需要高精度时间定位的场景，提供对纳秒级时间的统一表示。
 */

namespace phad::common
{

  class Timestamp
  {
  public:
    constexpr explicit Timestamp( std::int64_t nanoseconds = 0 ) noexcept
        : m_nanoseconds( nanoseconds ) {}

    [[nodiscard]] constexpr std::int64_t nanoseconds() const noexcept
    {
      return m_nanoseconds;
    }

    auto operator<=>( const Timestamp & ) const = default;

  private:
    std::int64_t m_nanoseconds;
  };

}  // namespace phad::common
