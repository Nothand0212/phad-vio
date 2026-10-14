#pragma once

#ifndef PHAD_ESTIMATOR_TESTING
#error "pnp_gauge_capture.hpp is available only to estimator diagnostics"
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

#include "phad/common/landmark_id.hpp"
#include "phad/common/timestamp.hpp"

namespace phad::estimator::internal
{
  struct PnpCorrespondence
  {
    common::LandmarkId m_id{};
    Eigen::Vector3d    m_point_W;
    Eigen::Vector2d    m_pixel;
  };

  // Values own their storage; floating-point fields retain the native call bits.
  struct PnpCallCapture
  {
    std::vector<PnpCorrespondence>   m_correspondences;
    Eigen::Isometry3d                m_guess_T_W_B         = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d                m_T_B_left            = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d                  m_K                   = Eigen::Matrix3d::Identity();
    Eigen::Vector3d                  m_guess_rvec          = Eigen::Vector3d::Zero();
    Eigen::Vector3d                  m_guess_tvec          = Eigen::Vector3d::Zero();
    int                              m_rvec_type           = -1;
    int                              m_tvec_type           = -1;
    int                              m_K_type              = -1;
    int                              m_object_points_type  = -1;
    int                              m_image_points_type   = -1;
    bool                             m_use_extrinsic_guess = true;
    int                              m_iterations          = 100;
    float                            m_reprojection_error  = 0.0F;
    double                           m_confidence          = 0.0;
    int                              m_flags               = 0;
    bool                             m_no_distortion       = true;
    bool                             m_solved              = false;
    std::string                      m_exception;
    std::optional<Eigen::Vector3d>   m_returned_rvec;
    std::optional<Eigen::Vector3d>   m_returned_tvec;
    int                              m_returned_rvec_type = -1;
    int                              m_returned_tvec_type = -1;
    int                              m_inliers_type       = -1;
    std::vector<int>                 m_inlier_indices;
    std::optional<Eigen::Isometry3d> m_returned_T_W_B;
  };

  struct PnpGaugeCapture
  {
    common::Timestamp                m_root_timestamp{ 0 };
    common::Timestamp                m_pnp_timestamp{ 0 };
    std::optional<Eigen::Isometry3d> m_A;
    std::optional<PnpCallCapture>    m_pnp;
  };
}  // namespace phad::estimator::internal
