#include "phad/common/trajectory.hpp"

#include <cmath>
#include <optional>
#include <sstream>

/**
 * @file trajectory.cpp
 * @brief Trajectory 的构造校验实现。
 */

namespace phad::common
{
  namespace
  {

    std::optional<TrajectoryError> validatePose( const TimedPose& pose,
                                                 std::size_t      index )
    {
      if ( !pose.T_W_B.matrix().allFinite() )
      {
        return TrajectoryError{ TrajectoryErrorCode::kNonFinitePose, index,
                                "pose elements must be finite" };
      }

      const Eigen::Matrix3d rotation = pose.T_W_B.linear();
      const double          orthogonality_error =
          ( rotation.transpose() * rotation - Eigen::Matrix3d::Identity() )
              .norm();
      const double determinant_error =
          std::abs( rotation.determinant() - 1.0 );
      if ( orthogonality_error > Trajectory::kRotationTolerance ||
           determinant_error > Trajectory::kRotationTolerance )
      {
        return TrajectoryError{
            TrajectoryErrorCode::kInvalidRotation, index,
            "rotation must be orthogonal with determinant +1" };
      }
      return std::nullopt;
    }

    std::optional<TrajectoryError> validateOrder( Timestamp   previous,
                                                  Timestamp   current,
                                                  std::size_t index )
    {
      if ( current == previous )
      {
        std::ostringstream detail;
        detail << "duplicate timestamp " << current.nanoseconds() << " ns";
        return TrajectoryError{ TrajectoryErrorCode::kDuplicateTimestamp,
                                index, std::move( detail ).str() };
      }
      if ( current < previous )
      {
        std::ostringstream detail;
        detail << "timestamp " << current.nanoseconds()
               << " ns is earlier than the previous "
               << previous.nanoseconds() << " ns";
        return TrajectoryError{ TrajectoryErrorCode::kOutOfOrderTimestamp,
                                index, std::move( detail ).str() };
      }
      return std::nullopt;
    }

  }  // namespace

  TrajectoryResult<Trajectory> Trajectory::create(
      std::vector<TimedPose> poses )
  {
    if ( poses.empty() )
    {
      return TrajectoryError{ TrajectoryErrorCode::kEmpty, 0,
                              "trajectory must contain at least one pose" };
    }

    for ( std::size_t index = 0; index < poses.size(); ++index )
    {
      if ( auto error = validatePose( poses[ index ], index ) )
      {
        return *std::move( error );
      }
      if ( index > 0 )
      {
        if ( auto error = validateOrder( poses[ index - 1 ].timestamp,
                                         poses[ index ].timestamp, index ) )
        {
          return *std::move( error );
        }
      }
    }
    return Trajectory( std::move( poses ) );
  }

  const std::vector<TimedPose>& Trajectory::poses() const noexcept
  {
    return m_poses;
  }

  std::size_t Trajectory::size() const noexcept { return m_poses.size(); }

  Timestamp Trajectory::firstTimestamp() const noexcept
  {
    return m_poses.front().timestamp;
  }

  Timestamp Trajectory::lastTimestamp() const noexcept
  {
    return m_poses.back().timestamp;
  }

  Trajectory::Trajectory( std::vector<TimedPose> poses )
      : m_poses( std::move( poses ) )
  {
  }

}  // namespace phad::common
