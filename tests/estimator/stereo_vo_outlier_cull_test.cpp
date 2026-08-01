#include <gtest/gtest.h>

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

TEST( StereoVoOutlierCullTest, RejectsNonPositiveOutlierAvgReproj )
{
  EstimatorOptions options;
  options.outlier_avg_reproj_px = 0.0;
  EXPECT_THROW( ( StereoVoEstimator{ makeCalibration(), options } ),
                std::invalid_argument );
}
