#include "phad/eval/align.hpp"

#include <Eigen/SVD>
#include <cstddef>
#include <sstream>
#include <utility>

/**
 * @file align.cpp
 * @brief Umeyama/Kabsch 对齐实现。
 */

namespace phad::eval
{
  namespace
  {

    Eigen::Vector3d centroid( const std::vector<Eigen::Vector3d>& points )
    {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      for ( const Eigen::Vector3d& point : points )
      {
        sum += point;
      }
      return sum / static_cast<double>( points.size() );
    }

  }  // namespace

  EvalResult<Eigen::Isometry3d> alignSe3(
      const std::vector<Eigen::Vector3d>& source,
      const std::vector<Eigen::Vector3d>& target, double rank_tolerance )
  {
    if ( source.size() != target.size() )
    {
      std::ostringstream cause;
      cause << "point sets must have equal size, got " << source.size()
            << " and " << target.size();
      return EvalError{ EvalErrorCode::kTooFewMatches, {}, std::nullopt, {}, std::move( cause ).str() };
    }
    if ( source.size() < 3U )
    {
      std::ostringstream cause;
      cause << "alignment needs at least 3 point pairs, got "
            << source.size();
      return EvalError{ EvalErrorCode::kTooFewMatches, {}, std::nullopt, {}, std::move( cause ).str() };
    }

    const Eigen::Vector3d source_centroid = centroid( source );
    const Eigen::Vector3d target_centroid = centroid( target );

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for ( std::size_t index = 0; index < source.size(); ++index )
    {
      covariance += ( target[ index ] - target_centroid ) *
                    ( source[ index ] - source_centroid ).transpose();
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        covariance, Eigen::ComputeFullU | Eigen::ComputeFullV );
    const Eigen::Vector3d singular_values = svd.singularValues();
    // 秩至少为 2 才能唯一确定旋转：共面点集可解（反射由行列式修正），
    // 共线或重合点集不可解。
    if ( singular_values( 0 ) <= 0.0 ||
         singular_values( 1 ) / singular_values( 0 ) < rank_tolerance )
    {
      std::ostringstream cause;
      cause << "point set is degenerate (collinear or coincident); singular "
               "values are "
            << singular_values( 0 ) << ", " << singular_values( 1 ) << ", "
            << singular_values( 2 );
      return EvalError{ EvalErrorCode::kDegenerateAlignment,
                        {},
                        std::nullopt,
                        {},
                        std::move( cause ).str() };
    }

    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    if ( ( svd.matrixU() * svd.matrixV().transpose() ).determinant() < 0.0 )
    {
      correction( 2, 2 ) = -1.0;
    }
    const Eigen::Matrix3d rotation =
        svd.matrixU() * correction * svd.matrixV().transpose();

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear()          = rotation;
    transform.translation()     = target_centroid - rotation * source_centroid;
    return transform;
  }

}  // namespace phad::eval
