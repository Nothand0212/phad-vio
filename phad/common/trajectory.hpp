#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <new>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "phad/common/timestamp.hpp"

/**
 * @file trajectory.hpp
 * @brief Trajectory 表示一条时间戳严格递增的 body 位姿序列。
 *
 * 数据集真值、评估输入和评估输出共用同一种轨迹载体，因此该类型放在
 * phad::common：phad::io 与 phad::eval 都能依赖它而不产生反向依赖。
 * 时间关联与相对位姿误差都以「时间戳严格递增」为前提，该不变量在
 * Trajectory::create 中集中校验一次。
 */

namespace phad::common
{

  /**
   * @brief 单个带时间戳的 body 位姿。
   *
   * T_W_B 把 body 坐标系中表达的量转换到 world，方向约定见
   * docs/conventions.md。
   */
  struct TimedPose
  {
    Timestamp         timestamp;
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
  };

  // Eigen 固定尺寸类型带有自身对齐要求，位姿序列使用默认分配器的
  // std::vector，因此该对齐必须不超过 operator new 的保证。
  static_assert( alignof( TimedPose ) <= __STDCPP_DEFAULT_NEW_ALIGNMENT__,
                 "TimedPose alignment must not exceed the guarantee of "
                 "global operator new" );

  enum class TrajectoryErrorCode : std::uint8_t
  {
    kEmpty               = 0,
    kNonFinitePose       = 1,
    kInvalidRotation     = 2,
    kDuplicateTimestamp  = 3,
    kOutOfOrderTimestamp = 4,
  };

  struct TrajectoryError
  {
    TrajectoryErrorCode code;
    std::size_t         index = 0;  // 出错位姿在输入序列中的下标
    std::string         detail;
  };

  template <typename T>
  class TrajectoryResult
  {
  public:
    TrajectoryResult( T value ) : m_storage( std::move( value ) ) {}

    TrajectoryResult( TrajectoryError error )
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

    [[nodiscard]] const TrajectoryError& error() const&
    {
      return std::get<TrajectoryError>( m_storage );
    }

  private:
    std::variant<T, TrajectoryError> m_storage;
  };

  /**
   * @brief 时间戳严格递增的 body 位姿序列。
   *
   * 只能通过 create() 构造：非空、时间戳严格递增、平移有限且旋转正交的
   * 序列才能成为 Trajectory，消费方无需重复校验这些前提。
   */
  class Trajectory
  {
  public:
    static constexpr double kRotationTolerance = 1e-6;

    [[nodiscard]] static TrajectoryResult<Trajectory> create(
        std::vector<TimedPose> poses );

    [[nodiscard]] const std::vector<TimedPose>& poses() const noexcept;
    [[nodiscard]] std::size_t                   size() const noexcept;
    [[nodiscard]] Timestamp                     firstTimestamp() const noexcept;
    [[nodiscard]] Timestamp                     lastTimestamp() const noexcept;

  private:
    explicit Trajectory( std::vector<TimedPose> poses );

    std::vector<TimedPose> m_poses;
  };

}  // namespace phad::common
