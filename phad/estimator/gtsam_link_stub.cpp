// Slice 0 placeholder: force phad_estimator to compile and link against GTSAM.
// Replaced by StereoVoEstimator implementation in later slices.

#include <gtsam/geometry/Pose3.h>

namespace phad::estimator::detail
{

  gtsam::Pose3 identityPose()
  {
    return gtsam::Pose3();
  }

}  // namespace phad::estimator::detail
