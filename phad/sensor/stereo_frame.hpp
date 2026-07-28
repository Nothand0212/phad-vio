#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "phad/common/timestamp.hpp"

namespace phad::sensor
{

  /**
   * @brief PixelType 枚举类型用于表示图像像素类型。
   *
   * 每个枚举值对应一个特定的像素类型，用于标识图像像素的存储方式。
   * 当前支持无符号 8 位和 16 位整数。
   */
  enum class PixelType : std::uint8_t
  {
    kUint8  = 0,
    kUint16 = 1
  };

  /**
   * @brief Image 类用于表示图像。
   *
   * 该类封装了图像的宽度、高度、通道数、像素类型和像素数据，
   * 提供对图像的快速访问和引用。
   */
  class Image
  {
  public:
    Image( int width, int height, int channels,
           std::vector<std::uint8_t> pixels )
        : m_width( width ),
          m_height( height ),
          m_channels( channels ),
          m_pixels( std::move( pixels ) ) {}

    Image( int width, int height, int channels,
           std::vector<std::uint16_t> pixels )
        : m_width( width ),
          m_height( height ),
          m_channels( channels ),
          m_pixels( std::move( pixels ) ) {}

    [[nodiscard]] int       width() const noexcept { return m_width; }
    [[nodiscard]] int       height() const noexcept { return m_height; }
    [[nodiscard]] int       channels() const noexcept { return m_channels; }
    [[nodiscard]] PixelType pixelType() const noexcept
    {
      return std::holds_alternative<std::vector<std::uint8_t>>( m_pixels )
                 ? PixelType::kUint8
                 : PixelType::kUint16;
    }

    template <typename Pixel>
    [[nodiscard]] std::optional<std::span<const Pixel>> pixels() const noexcept
    {
      static_assert( std::is_same_v<Pixel, std::uint8_t> ||
                         std::is_same_v<Pixel, std::uint16_t>,
                     "Image only supports uint8_t and uint16_t pixels" );
      const auto* pixels = std::get_if<std::vector<Pixel>>( &m_pixels );
      if ( pixels == nullptr )
      {
        return std::nullopt;
      }
      return std::span<const Pixel>{ *pixels };
    }

  private:
    int m_width;
    int m_height;
    int m_channels;
    std::variant<std::vector<std::uint8_t>, std::vector<std::uint16_t>>
        m_pixels;
  };

  /**
   * @brief StereoFrame 结构体用于表示立体帧。
   *
   * 该结构体封装了时间戳、左右图像，提供对立体帧的快速访问和引用。
   */
  struct StereoFrame
  {
    common::Timestamp timestamp;
    Image             left;
    Image             right;
  };

}  // namespace phad::sensor
