#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <variant>

#include "phad/sensor/camera_parameters.hpp"

namespace phad::camera
{

  enum class CameraModelErrorCode : std::uint8_t
  {
    kNonFiniteInput = 0,
    kOutsideModelDomain,
    kNumericalFailure
  };

  struct CameraModelError
  {
    CameraModelErrorCode code;
    std::string          detail;
  };

  template <typename T>
  class CameraModelResult
  {
  public:
    CameraModelResult( T value ) : m_storage( std::move( value ) ) {}

    CameraModelResult( CameraModelError error )
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

    [[nodiscard]] const CameraModelError& error() const&
    {
      return std::get<CameraModelError>( m_storage );
    }

  private:
    std::variant<T, CameraModelError> m_storage;
  };

  class CameraModel
  {
  public:
    CameraModel( const CameraModel& )            = delete;
    CameraModel& operator=( const CameraModel& ) = delete;
    CameraModel( CameraModel&& )                 = delete;
    CameraModel& operator=( CameraModel&& )      = delete;

    virtual ~CameraModel() = default;

    [[nodiscard]] virtual CameraModelResult<Eigen::Vector2d> project(
        const Eigen::Vector3d& point_camera ) const = 0;

    [[nodiscard]] virtual CameraModelResult<Eigen::Vector3d> backProject(
        const Eigen::Vector2d& pixel ) const = 0;

  protected:
    CameraModel() = default;
  };

  [[nodiscard]] std::unique_ptr<CameraModel> createCameraModel(
      sensor::CameraModelParameters model_parameters );

}  // namespace phad::camera
