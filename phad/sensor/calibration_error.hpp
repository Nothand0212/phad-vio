#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <variant>

namespace phad::sensor
{

  enum class CalibrationErrorCode : std::uint8_t
  {
    kNonFiniteValue = 0,
    kNonPositiveValue,
    kInvalidRotation,
    kInvalidHomogeneousRow,
    kZeroStereoBaseline
  };

  struct CalibrationError
  {
    CalibrationErrorCode code;
    std::string          field_path;
    std::string          detail;
  };

  template <typename T>
  class CalibrationResult
  {
  public:
    CalibrationResult( T value ) : m_storage( std::move( value ) ) {}

    CalibrationResult( CalibrationError error )
        : m_storage( std::move( error ) )
    {
    }

    [[nodiscard]] bool hasValue() const noexcept
    {
      return std::holds_alternative<T>( m_storage );
    }

    explicit operator bool() const noexcept { return hasValue(); }

    [[nodiscard]] T& value() & { return std::get<T>( m_storage ); }

    [[nodiscard]] const T& value() const&
    {
      return std::get<T>( m_storage );
    }

    [[nodiscard]] T&& value() &&
    {
      return std::get<T>( std::move( m_storage ) );
    }

    [[nodiscard]] const CalibrationError& error() const&
    {
      return std::get<CalibrationError>( m_storage );
    }

  private:
    std::variant<T, CalibrationError> m_storage;
  };

}  // namespace phad::sensor
