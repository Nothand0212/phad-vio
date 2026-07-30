#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

/**
 * @file eval_error.hpp
 * @brief 评估模块的错误类型。
 *
 * 评估失败与数据集加载失败是两类不同的问题：前者关于轨迹之间的关系
 * （时间是否重叠、匹配是否足够、点集是否退化），后者关于数据集文件的
 * 格式。因此 phad::eval 使用自己的错误类型，而不是复用 DatasetError。
 */

namespace phad::eval
{

  enum class EvalErrorCode : std::uint8_t
  {
    kIoError             = 0,
    kInvalidRecord       = 1,
    kInvalidTrajectory   = 2,
    kNoOverlap           = 3,
    kTooFewMatches       = 4,
    kMatchRateTooLow     = 5,
    kDegenerateAlignment = 6,
  };

  struct EvalError
  {
    EvalErrorCode              code = EvalErrorCode::kIoError;
    std::filesystem::path      source_path;
    std::optional<std::size_t> line;
    std::string                field;
    std::string                cause;

    [[nodiscard]] std::string describe() const
    {
      std::ostringstream stream;
      stream << "eval error " << static_cast<int>( code );
      if ( !source_path.empty() )
      {
        stream << ", path=" << source_path.string();
      }
      if ( line.has_value() )
      {
        stream << ", line=" << *line;
      }
      if ( !field.empty() )
      {
        stream << ", field=" << field;
      }
      if ( !cause.empty() )
      {
        stream << ": " << cause;
      }
      return stream.str();
    }
  };

  template <typename T>
  class EvalResult
  {
  public:
    EvalResult( T value ) : m_storage( std::move( value ) ) {}

    EvalResult( EvalError error ) : m_storage( std::move( error ) ) {}

    [[nodiscard]] bool hasValue() const noexcept
    {
      return std::holds_alternative<T>( m_storage );
    }

    explicit operator bool() const noexcept { return hasValue(); }

    [[nodiscard]] T& value() & { return std::get<T>( m_storage ); }

    [[nodiscard]] const T& value() const& { return std::get<T>( m_storage ); }

    [[nodiscard]] T&& value() &&
    {
      return std::get<T>( std::move( m_storage ) );
    }

    [[nodiscard]] const EvalError& error() const&
    {
      return std::get<EvalError>( m_storage );
    }

  private:
    std::variant<T, EvalError> m_storage;
  };

}  // namespace phad::eval
