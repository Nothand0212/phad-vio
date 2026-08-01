#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdexcept>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::StereoVoEstimator;
  using phad::sensor::RigidTransform;

  RectifiedStereoCalibration makeCalibration()
  {
    auto rigid = RigidTransform::create( Eigen::Isometry3d::Identity().matrix() )
                     .value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480, std::move( rigid ) )
        .value();
  }

}  // namespace

TEST( StereoVoPnpTest, RejectsNonPositivePnpReproj )
{
  EstimatorOptions options;
  options.pnp_reproj_px = 0.0;
  EXPECT_THROW( ( StereoVoEstimator{ makeCalibration(), options } ),
                std::invalid_argument );
}

TEST( StereoVoPnpTest, RejectsInvalidPnpConfidence )
{
  EstimatorOptions options;
  options.pnp_confidence = 1.0;  // 要求 ∈ (0, 1)
  EXPECT_THROW( ( StereoVoEstimator{ makeCalibration(), options } ),
                std::invalid_argument );
}

TEST( StereoVoPnpTest, RejectsMinPnpInliersBelowFour )
{
  EstimatorOptions options;
  options.min_pnp_inliers = 3;
  EXPECT_THROW( ( StereoVoEstimator{ makeCalibration(), options } ),
                std::invalid_argument );
}
