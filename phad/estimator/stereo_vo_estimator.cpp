#include "phad/estimator/stereo_vo_estimator.hpp"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/StereoFactor.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <map>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace phad::estimator
{
  namespace
  {

    // GTSAM 4.3 exports uppercase Symbol helpers (X/L); older docs used x/l.
    using gtsam::symbol_shorthand::L;
    using gtsam::symbol_shorthand::W;
    using gtsam::symbol_shorthand::X;

    [[nodiscard]] gtsam::SharedNoiseModel makeAhrsNoise(
        const gtsam::PreintegratedAhrsMeasurements& preintegration,
        const double                                alignment_residual_rms_rad )
    {
      if ( !std::isfinite( alignment_residual_rms_rad ) ||
           alignment_residual_rms_rad < 0.0 )
      {
        throw std::invalid_argument(
            "AHRS alignment residual RMS must be finite and non-negative" );
      }
      gtsam::Matrix3 covariance = preintegration.preintMeasCov();
      // The reported RMS is ||Log(R_visual^-1 R_gyro)||. Convert it to an
      // isotropic per-axis variance before combining independent covariance.
      covariance.diagonal().array() +=
          alignment_residual_rms_rad * alignment_residual_rms_rad / 3.0;
      covariance = 0.5 * ( covariance + covariance.transpose() );
      if ( !covariance.allFinite() )
      {
        throw std::invalid_argument( "AHRS covariance is non-finite" );
      }
      return gtsam::noiseModel::Gaussian::Covariance( covariance );
    }

    class PoseAhrsFactor final
        : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3,
                                          gtsam::Vector3>
    {
      using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3,
                                            gtsam::Vector3>;

    public:
      PoseAhrsFactor(
          gtsam::Key pose_i, gtsam::Key pose_j, gtsam::Key bias_gyr,
          const gtsam::PreintegratedAhrsMeasurements& preintegration,
          const double                                alignment_residual_rms_rad )
          : Base( makeAhrsNoise( preintegration, alignment_residual_rms_rad ),
                  pose_i, pose_j, bias_gyr ),
            m_preintegration( preintegration )
      {
      }

      using Base::evaluateError;

      [[nodiscard]] gtsam::Vector evaluateError(
          const gtsam::Pose3& pose_i, const gtsam::Pose3& pose_j,
          const gtsam::Vector3& bias_gyr, gtsam::OptionalMatrixType H1,
          gtsam::OptionalMatrixType H2,
          gtsam::OptionalMatrixType H3 ) const override
      {
        gtsam::Matrix36   D_rotation_i_pose_i;
        gtsam::Matrix36   D_rotation_j_pose_j;
        const gtsam::Rot3 rotation_i = pose_i.rotation(
            H1 ? &D_rotation_i_pose_i : nullptr );
        const gtsam::Rot3 rotation_j = pose_j.rotation(
            H2 ? &D_rotation_j_pose_j : nullptr );

        gtsam::Matrix           D_error_rotation_i;
        gtsam::Matrix           D_error_rotation_j;
        gtsam::Matrix           D_error_bias;
        const gtsam::AHRSFactor factor( 0, 1, 2, m_preintegration );
        const gtsam::Vector     error = factor.evaluateError(
            rotation_i, rotation_j, bias_gyr,
            H1 ? &D_error_rotation_i : nullptr,
            H2 ? &D_error_rotation_j : nullptr,
            H3 ? &D_error_bias : nullptr );
        if ( H1 )
        {
          *H1 = D_error_rotation_i * D_rotation_i_pose_i;
        }
        if ( H2 )
        {
          *H2 = D_error_rotation_j * D_rotation_j_pose_j;
        }
        if ( H3 )
        {
          *H3 = D_error_bias;
        }
        return error;
      }

    private:
      gtsam::PreintegratedAhrsMeasurements m_preintegration;
    };

    [[nodiscard]] gtsam::Pose3 toPose3( const Eigen::Isometry3d& T_a_b )
    {
      return gtsam::Pose3( T_a_b.matrix() );
    }

    [[nodiscard]] Eigen::Isometry3d toIsometry( const gtsam::Pose3& pose )
    {
      // GTSAM Rot3 matrices can drift slightly off SO(3); Trajectory::create
      // and constant-velocity init both require a proper rotation.
      Eigen::Quaterniond rotation( pose.rotation().matrix() );
      rotation.normalize();
      Eigen::Isometry3d T_a_b = Eigen::Isometry3d::Identity();
      T_a_b.linear()          = rotation.toRotationMatrix();
      T_a_b.translation()     = pose.translation();
      return T_a_b;
    }

    [[nodiscard]] Eigen::Isometry3d toIsometry(
        const sensor::RigidTransform& transform )
    {
      Eigen::Isometry3d T_a_b = Eigen::Isometry3d::Identity();
      T_a_b.linear()          = transform.rotation();
      T_a_b.translation()     = transform.translation();
      return T_a_b;
    }

    [[nodiscard]] gtsam::StereoPoint2 toStereoPoint(
        const StereoObservation& observation )
    {
      return gtsam::StereoPoint2(
          observation.left_pixel.x(),
          observation.left_pixel.x() - observation.disparity_px,
          observation.left_pixel.y() );
    }

    [[nodiscard]] bool isFinite( const Eigen::Isometry3d& T_a_b )
    {
      return T_a_b.matrix().allFinite();
    }

    [[nodiscard]] bool isFinite( const Eigen::Vector3d& point )
    {
      return point.allFinite();
    }

    [[nodiscard]] gtsam::SharedNoiseModel makeStereoNoise(
        const EstimatorOptions& options )
    {
      const auto gaussian =
          gtsam::noiseModel::Isotropic::Sigma( 3, options.stereo_sigma_px );
      if ( options.huber_k_px <= 0.0 )
      {
        return gaussian;
      }
      return gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create( options.huber_k_px ),
          gaussian );
    }

    [[nodiscard]] gtsam::SharedNoiseModel makePriorNoise(
        const EstimatorOptions& options )
    {
      gtsam::Vector6 sigmas;
      sigmas << options.prior_rotation_sigma_rad,
          options.prior_rotation_sigma_rad, options.prior_rotation_sigma_rad,
          options.prior_translation_sigma_m, options.prior_translation_sigma_m,
          options.prior_translation_sigma_m;
      return gtsam::noiseModel::Diagonal::Sigmas( sigmas );
    }

    // Gyro-active graph anchors the oldest X with one isotropic pose sigma.
    [[nodiscard]] gtsam::SharedNoiseModel makeImuPosePriorNoise(
        const EstimatorOptions& options )
    {
      gtsam::Vector6 sigmas;
      sigmas << options.imu_prior_pose_sigma, options.imu_prior_pose_sigma,
          options.imu_prior_pose_sigma, options.imu_prior_pose_sigma,
          options.imu_prior_pose_sigma, options.imu_prior_pose_sigma;
      return gtsam::noiseModel::Diagonal::Sigmas( sigmas );
    }

    struct WindowFrame
    {
      std::uint64_t                  frame_index = 0;
      common::Timestamp              timestamp{ 0 };
      Eigen::Isometry3d              T_W_B = Eigen::Isometry3d::Identity();
      std::vector<StereoObservation> observations;
      bool                           is_keyframe = true;  // Slice ⑤c
      // Shared gyro bias used as the AHRS preintegration linearisation point.
      Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
      // 帧间段 [t_prev, timestamp] (sync 切段语义, 见 StereoImuPacket)。
      // buildGraph 每次从原始样本即时重建预积分 (C3)。
      std::vector<sensor::ImuMeasurement> imu_samples;
      common::Timestamp                   t_prev{ 0 };
      // 段不完整/不可积分 (sync 标记或样本不足) → 跳过该 AHRS factor，
      // 视觉图仍照常优化。
      bool imu_gap = false;
    };

    struct GyroBiasEvidence
    {
      gtsam::Rot3                         R_W_B_i;
      gtsam::Rot3                         R_W_B_j;
      std::vector<sensor::ImuMeasurement> imu_samples;
      double                              duration_s = 0.0;
    };

    [[nodiscard]] double stereoReprojRms(
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values&               values )
    {
      double      sum_sq = 0.0;
      std::size_t count  = 0;
      for ( const auto& factor : graph )
      {
        if ( factor == nullptr )
        {
          continue;
        }
        const auto* stereo =
            dynamic_cast<const gtsam::GenericStereoFactor<gtsam::Pose3,
                                                          gtsam::Point3>*>(
                factor.get() );
        if ( stereo == nullptr )
        {
          continue;
        }
        const gtsam::Vector error = stereo->unwhitenedError( values );
        sum_sq += error.squaredNorm();
        ++count;
      }
      if ( count == 0 )
      {
        return 0.0;
      }
      return std::sqrt( sum_sq / static_cast<double>( count ) );
    }

    [[nodiscard]] bool validCost( const double value )
    {
      return std::isfinite( value ) && value >= 0.0;
    }

    // Read-only objective decomposition for the exact gyro graph round being
    // committed. Boundary is selected by the smallest destination pose key;
    // newest by the largest. The optional is absent on any schema/numeric
    // mismatch so the state writer can fail loudly without affecting state.
    [[nodiscard]] std::optional<GyroGraphCostDiagnostics>
    summarizeGyroGraphCosts( const gtsam::NonlinearFactorGraph& graph,
                             const gtsam::Values&               initial,
                             const gtsam::Values&               posterior )
    {
      using BiasPrior = gtsam::PriorFactor<gtsam::Vector3>;
      using PosePrior = gtsam::PriorFactor<gtsam::Pose3>;
      using StereoFactor =
          gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;

      struct AhrsCost
      {
        const PoseAhrsFactor* factor         = nullptr;
        gtsam::Key            pose_j         = 0;
        double                initial_cost   = 0.0;
        double                posterior_cost = 0.0;
      };

      try
      {
        GyroGraphCostDiagnostics diagnostics;
        std::vector<AhrsCost>    ahrs_costs;
        const PosePrior*         pose_prior       = nullptr;
        const BiasPrior*         bias_prior       = nullptr;
        std::size_t              pose_prior_count = 0U;
        std::size_t              bias_prior_count = 0U;

        for ( const auto& factor : graph )
        {
          if ( factor == nullptr )
          {
            continue;
          }
          const double initial_cost   = factor->error( initial );
          const double posterior_cost = factor->error( posterior );
          if ( !validCost( initial_cost ) || !validCost( posterior_cost ) )
          {
            return std::nullopt;
          }
          diagnostics.total_initial_cost += initial_cost;
          diagnostics.total_posterior_cost += posterior_cost;

          if ( const auto* ahrs =
                   dynamic_cast<const PoseAhrsFactor*>( factor.get() ) )
          {
            ahrs_costs.push_back( AhrsCost{ ahrs, ahrs->key2(), initial_cost,
                                            posterior_cost } );
            diagnostics.gyro_initial_cost += initial_cost;
            diagnostics.gyro_posterior_cost += posterior_cost;
            continue;
          }
          if ( dynamic_cast<const StereoFactor*>( factor.get() ) != nullptr )
          {
            ++diagnostics.stereo_factor_count;
            diagnostics.stereo_initial_cost += initial_cost;
            diagnostics.stereo_posterior_cost += posterior_cost;
            continue;
          }
          if ( const auto* prior =
                   dynamic_cast<const PosePrior*>( factor.get() ) )
          {
            pose_prior = prior;
            ++pose_prior_count;
            continue;
          }
          if ( const auto* prior =
                   dynamic_cast<const BiasPrior*>( factor.get() ) )
          {
            bias_prior = prior;
            ++bias_prior_count;
          }
        }

        if ( ahrs_costs.empty() || diagnostics.stereo_factor_count == 0U ||
             pose_prior_count != 1U || bias_prior_count != 1U )
        {
          return std::nullopt;
        }
        std::sort( ahrs_costs.begin(), ahrs_costs.end(),
                   []( const AhrsCost& left, const AhrsCost& right ) {
                     return left.pose_j < right.pose_j;
                   } );
        diagnostics.gyro_factor_count =
            static_cast<std::uint32_t>( ahrs_costs.size() );
        diagnostics.interior_factor_count =
            static_cast<std::uint32_t>( ahrs_costs.size() - 1U );

        const AhrsCost& boundary            = ahrs_costs.front();
        const AhrsCost& newest              = ahrs_costs.back();
        diagnostics.boundary_initial_cost   = boundary.initial_cost;
        diagnostics.boundary_posterior_cost = boundary.posterior_cost;
        diagnostics.newest_initial_cost     = newest.initial_cost;
        diagnostics.newest_posterior_cost   = newest.posterior_cost;
        for ( auto it = std::next( ahrs_costs.begin() );
              it != ahrs_costs.end(); ++it )
        {
          diagnostics.interior_initial_cost += it->initial_cost;
          diagnostics.interior_posterior_cost += it->posterior_cost;
        }

        const auto residualNorms = []( const PoseAhrsFactor& factor,
                                       const gtsam::Values&  values )
            -> std::optional<std::pair<double, double>> {
          const gtsam::Vector raw = factor.unwhitenedError( values );
          if ( raw.size() != 3 || !raw.allFinite() ||
               factor.noiseModel() == nullptr )
          {
            return std::nullopt;
          }
          const gtsam::Vector whitened =
              factor.noiseModel()->whiten( raw );
          if ( !whitened.allFinite() )
          {
            return std::nullopt;
          }
          return std::pair<double, double>{ raw.norm(), whitened.norm() };
        };
        const auto boundary_initial =
            residualNorms( *boundary.factor, initial );
        const auto boundary_posterior =
            residualNorms( *boundary.factor, posterior );
        const auto newest_initial = residualNorms( *newest.factor, initial );
        const auto newest_posterior =
            residualNorms( *newest.factor, posterior );
        if ( !boundary_initial.has_value() ||
             !boundary_posterior.has_value() ||
             !newest_initial.has_value() || !newest_posterior.has_value() )
        {
          return std::nullopt;
        }
        diagnostics.boundary_initial_residual_norm_rad =
            boundary_initial->first;
        diagnostics.boundary_initial_whitened_norm =
            boundary_initial->second;
        diagnostics.boundary_posterior_residual_norm_rad =
            boundary_posterior->first;
        diagnostics.boundary_posterior_whitened_norm =
            boundary_posterior->second;
        diagnostics.newest_initial_residual_norm_rad = newest_initial->first;
        diagnostics.newest_initial_whitened_norm     = newest_initial->second;
        diagnostics.newest_posterior_residual_norm_rad =
            newest_posterior->first;
        diagnostics.newest_posterior_whitened_norm =
            newest_posterior->second;

        const gtsam::Vector pose_error =
            pose_prior->unwhitenedError( posterior );
        const gtsam::Vector bias_error =
            bias_prior->unwhitenedError( posterior );
        if ( pose_error.size() != 6 || bias_error.size() != 3 ||
             !pose_error.allFinite() || !bias_error.allFinite() )
        {
          return std::nullopt;
        }
        diagnostics.pose_prior_posterior_rotation_norm =
            pose_error.head<3>().norm();
        diagnostics.pose_prior_posterior_translation_norm =
            pose_error.tail<3>().norm();
        diagnostics.pose_prior_posterior_cost =
            pose_prior->error( posterior );
        diagnostics.bias_prior_posterior_norm = bias_error.norm();
        diagnostics.bias_prior_posterior_cost =
            bias_prior->error( posterior );

        const double scalars[] = {
            diagnostics.total_initial_cost,
            diagnostics.total_posterior_cost,
            diagnostics.gyro_initial_cost,
            diagnostics.gyro_posterior_cost,
            diagnostics.boundary_initial_cost,
            diagnostics.boundary_posterior_cost,
            diagnostics.boundary_initial_residual_norm_rad,
            diagnostics.boundary_posterior_residual_norm_rad,
            diagnostics.boundary_initial_whitened_norm,
            diagnostics.boundary_posterior_whitened_norm,
            diagnostics.interior_initial_cost,
            diagnostics.interior_posterior_cost,
            diagnostics.newest_initial_cost,
            diagnostics.newest_posterior_cost,
            diagnostics.newest_initial_residual_norm_rad,
            diagnostics.newest_posterior_residual_norm_rad,
            diagnostics.newest_initial_whitened_norm,
            diagnostics.newest_posterior_whitened_norm,
            diagnostics.stereo_initial_cost,
            diagnostics.stereo_posterior_cost,
            diagnostics.pose_prior_posterior_rotation_norm,
            diagnostics.pose_prior_posterior_translation_norm,
            diagnostics.pose_prior_posterior_cost,
            diagnostics.bias_prior_posterior_norm,
            diagnostics.bias_prior_posterior_cost };
        const bool valid = std::all_of(
            std::begin( scalars ), std::end( scalars ), validCost );
        return valid ? std::optional<GyroGraphCostDiagnostics>( diagnostics )
                     : std::nullopt;
      }
      catch ( const std::exception& )
      {
        return std::nullopt;
      }
    }

    // Keep top-K pose shifts by |Δt|; at most K entries, unordered until sorted.
    void considerShiftTopK(
        std::vector<std::pair<std::uint64_t, double>>& top, std::uint64_t key,
        double dt_m, std::size_t k )
    {
      if ( top.size() < k )
      {
        top.emplace_back( key, dt_m );
        return;
      }
      auto min_it = std::min_element(
          top.begin(), top.end(),
          []( const std::pair<std::uint64_t, double>& a,
              const std::pair<std::uint64_t, double>& b ) {
            return a.second < b.second;
          } );
      if ( dt_m > min_it->second )
      {
        *min_it = { key, dt_m };
      }
    }

    // Per-landmark mean stereo residual (unwhitened L2), then mean/max/max_id.
    void fillProbeLandmarkResiduals( const gtsam::NonlinearFactorGraph& graph,
                                     const gtsam::Values&               values,
                                     UpdateDiagnostics&                 diagnostics )
    {
      std::unordered_map<LandmarkId, std::pair<double, std::size_t>> per_lm;
      for ( const auto& factor : graph )
      {
        if ( factor == nullptr )
        {
          continue;
        }
        const auto* stereo =
            dynamic_cast<const gtsam::GenericStereoFactor<gtsam::Pose3,
                                                          gtsam::Point3>*>(
                factor.get() );
        if ( stereo == nullptr )
        {
          continue;
        }
        const LandmarkId id =
            static_cast<LandmarkId>( gtsam::Symbol( stereo->key2() ).index() );
        auto& entry = per_lm[ id ];
        entry.first += stereo->unwhitenedError( values ).norm();
        ++entry.second;
      }

      diagnostics.probe_detail_valid = true;
      if ( per_lm.empty() )
      {
        return;
      }

      double     sum_means = 0.0;
      double     max_px    = -1.0;
      LandmarkId max_id{};
      for ( const auto& [ id, score ] : per_lm )
      {
        const double mean =
            score.first / static_cast<double>( score.second );
        sum_means += mean;
        if ( mean > max_px )
        {
          max_px = mean;
          max_id = id;
        }
      }
      diagnostics.probe_res_mean_px =
          sum_means / static_cast<double>( per_lm.size() );
      diagnostics.probe_res_max_px = max_px;
      diagnostics.probe_res_max_id = max_id;
    }

    // Same RMS as stereoReprojRms but skips factors whose landmark is no longer
    // in landmarks_W (cheirality / mean-reproj cull). Does not rebuild the graph.
    [[nodiscard]] double stereoReprojRmsSkippingMissingLandmarks(
        const gtsam::NonlinearFactorGraph&                     graph,
        const gtsam::Values&                                   values,
        const std::unordered_map<LandmarkId, Eigen::Vector3d>& landmarks_W )
    {
      double      sum_sq = 0.0;
      std::size_t count  = 0;
      for ( const auto& factor : graph )
      {
        if ( factor == nullptr )
        {
          continue;
        }
        const auto* stereo =
            dynamic_cast<const gtsam::GenericStereoFactor<gtsam::Pose3,
                                                          gtsam::Point3>*>(
                factor.get() );
        if ( stereo == nullptr )
        {
          continue;
        }
        const gtsam::Key lkey = stereo->key2();
        const LandmarkId id =
            static_cast<LandmarkId>( gtsam::Symbol( lkey ).index() );
        if ( landmarks_W.find( id ) == landmarks_W.end() )
        {
          continue;
        }
        const gtsam::Vector error = stereo->unwhitenedError( values );
        sum_sq += error.squaredNorm();
        ++count;
      }
      if ( count == 0 )
      {
        return 0.0;
      }
      return std::sqrt( sum_sq / static_cast<double>( count ) );
    }

    // GenericStereoFactor returns 2*fx on each residual axis when the point is
    // behind the camera (throwCheirality=false).
    [[nodiscard]] std::uint32_t countCheiralityFactors(
        const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
        double fx_pixels )
    {
      const double  sentinel = 2.0 * fx_pixels;
      std::uint32_t count    = 0;
      for ( const auto& factor : graph )
      {
        if ( factor == nullptr )
        {
          continue;
        }
        const auto* stereo =
            dynamic_cast<const gtsam::GenericStereoFactor<gtsam::Pose3,
                                                          gtsam::Point3>*>(
                factor.get() );
        if ( stereo == nullptr )
        {
          continue;
        }
        const gtsam::Vector error = stereo->unwhitenedError( values );
        if ( error.size() == 3 &&
             std::abs( error( 0 ) - sentinel ) < 1e-6 &&
             std::abs( error( 1 ) - sentinel ) < 1e-6 &&
             std::abs( error( 2 ) - sentinel ) < 1e-6 )
        {
          ++count;
        }
      }
      return count;
    }

    [[nodiscard]] std::uint32_t countBehindCameraLandmarks(
        const std::deque<WindowFrame>&                         window,
        const std::unordered_map<LandmarkId, Eigen::Vector3d>& landmarks_W,
        const gtsam::Pose3&                                    body_P_sensor,
        const gtsam::Values&                                   values )
    {
      std::uint32_t count = 0;
      for ( const auto& [ id, point_W ] : landmarks_W )
      {
        const gtsam::Key key = L( id );
        if ( !values.exists( key ) )
        {
          continue;
        }
        const gtsam::Point3 point = values.at<gtsam::Point3>( key );
        for ( const WindowFrame& frame : window )
        {
          bool observes = false;
          for ( const StereoObservation& observation : frame.observations )
          {
            // Slice ⑦: zero-disparity frames carry no constraint on the
            // landmark; count only stereo observations (mirrors
            // dropCheiralityLandmarks).
            if ( observation.id == id &&
                 observation.disparity_px > 0.0 )
            {
              observes = true;
              break;
            }
          }
          if ( !observes || !values.exists( X( frame.frame_index ) ) )
          {
            continue;
          }
          const gtsam::Pose3 T_W_left =
              values.at<gtsam::Pose3>( X( frame.frame_index ) ) *
              body_P_sensor;
          if ( T_W_left.transformTo( point ).z() <= 0.0 )
          {
            ++count;
            break;
          }
        }
      }
      return count;
    }

  }  // namespace

  struct StereoVoEstimator::Impl
  {
    enum class ShadowFactorKind : std::uint8_t
    {
      kPosePrior,
      kBiasPrior,
      kAhrs,
      kStereo
    };

    struct ShadowFactorId
    {
      ShadowFactorKind          kind = ShadowFactorKind::kPosePrior;
      std::array<gtsam::Key, 3> keys{};
      std::uint8_t              key_count = 0;

      [[nodiscard]] bool operator<( const ShadowFactorId& other ) const
      {
        return std::tie( kind, key_count, keys ) <
               std::tie( other.kind, other.key_count, other.keys );
      }
    };

    struct FixedLagShadowState
    {
      explicit FixedLagShadowState( const double lag )
          : smoother( lag, makeParams() )
      {
      }

      [[nodiscard]] static gtsam::ISAM2Params makeParams()
      {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold  = 0.0;
        params.relinearizeSkip       = 1;
        params.findUnusedFactorSlots = true;
        return params;
      }

      gtsam::IncrementalFixedLagSmoother         smoother;
      std::map<std::size_t, ShadowFactorId>      owned_slots;
      std::set<ShadowFactorId>                   committed_factors;
      std::unordered_map<LandmarkId, gtsam::Key> live_landmarks;
      std::uint64_t                              next_landmark_generation = 0;
      std::uint32_t                              segment_id               = 0;
    };

    camera::RectifiedStereoCalibration              calibration;
    EstimatorOptions                                options;
    gtsam::Cal3_S2Stereo::shared_ptr                K;
    gtsam::Pose3                                    body_P_sensor;
    gtsam::SharedNoiseModel                         stereo_noise;
    gtsam::SharedNoiseModel                         prior_noise;
    std::deque<WindowFrame>                         window;
    std::unordered_map<LandmarkId, Eigen::Vector3d> landmarks_W;
    std::unordered_map<LandmarkId, std::vector<common::Timestamp>>
        track_times;
    // Slice ⑦ E13: body pose at each landmark's most recent stereo
    // observation, kept beyond the window. The hang-distance gate measures
    // against this — findLastStereoFrameInWindow's pose evaporates once the
    // stereo frame leaves the window, silently turning every gate value into
    // "drop all far-return candidates" (E11's bug, never a real distance
    // test). Updated with the BA-refined pose; erased with the landmark.
    std::unordered_map<LandmarkId, Eigen::Isometry3d> last_stereo_pose_W;
    std::optional<Eigen::Isometry3d>                  last_accepted_T_W_B;
    std::optional<Eigen::Isometry3d>                  prev_accepted_T_W_B;
    std::uint64_t                                     next_frame_index = 0;
    bool                                              initialized      = false;
    std::uint32_t                                     segment_id       = 0;
    // Cross-segment reject set: mean-cull ∪ cheirality erasures (block rebirth).
    std::unordered_set<LandmarkId> culled_ids_;
    // pre-M4 round 2: accumulated seeding buffer. While overlap is broken
    // (or before first-segment init) and a frame's stereo yield is below
    // min_seed_observations, the frame's valid stereo observations accumulate
    // here (latest observation wins per track — pixels stay as fresh as the
    // track allows). Once the buffer holds min_seed_observations unique
    // tracks, a synthetic measurement built from it replaces the frame
    // measurement for seeding (SVO DepthFilter-style evidence accumulation).
    // Cleared when a single frame passes the gate, when overlap recovers
    // (num_shared > 0), or after a successful accumulated seeding.
    std::unordered_map<LandmarkId, StereoObservation> pending_seed_obs;
    // ---- M4.4 gyro-visual fusion ----
    // Gyro-only preintegration parameters, created only when IMU is enabled.
    std::shared_ptr<gtsam::PreintegratedRotationParams> ahrs_params;
    gtsam::SharedNoiseModel                             imu_pose_prior_noise;
    gtsam::SharedNoiseModel                             imu_gyr_bias_prior_noise;
    // D9: 被拒/失败帧的段累积于此 (共享边界样本去重);成功帧消费 (段入窗 +
    // pending 清空), 下帧从最后接受位姿继续预积分。
    std::vector<sensor::ImuMeasurement> pending_imu;
    common::Timestamp                   pending_from{ 0 };
    bool                                pending_gap = false;
    // D12: overlap 断开后 landmark 表只重建一次;重建窗口内的种子持续累积,
    // 直到 overlap 恢复 (num_shared > 0) 重置。
    bool imu_window_rebuilt = false;
    // ---- M4.4 静止 gyro audit ----
    // kNone: 尚未观察 IMU; kCollecting: 与视觉估计并行累计 low-variance
    // window; kReady: audit/gyro seed 已发布; kFailed: timeout 后 sticky。
    // Collecting 不得阻塞视觉帧，否则 IMU-on 会在 gyro factor 激活前改变
    // accepted-keyframe epoch，混淆 VIO/VO 对拍。
    enum class InitPhase : std::uint8_t
    {
      kNone,
      kCollecting,
      kReady,
      kFailed
    };
    InitPhase init_phase = InitPhase::kNone;
    // Audit 累积缓冲，与逐接受帧消费的 pending_imu 分离。
    std::vector<sensor::ImuMeasurement> init_imu;
    // 数据起点 (首个被拒帧时刻;gap 重置后更新) —— 超时判定基准。
    common::Timestamp init_start_ts{ 0 };
    // 检测通过时缓冲尾窗口的起点，仅用于 audit 时间支持。
    common::Timestamp init_t0{ 0 };
    // Static orientation/gravity/acc bias 只用于 audit；生产 pose 保持 VO gauge。
    Eigen::Isometry3d init_T_W_I0 =
        Eigen::Isometry3d::Identity();
    Eigen::Vector3d    init_gyro_bias0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d    init_acc_bias0  = Eigen::Vector3d::Zero();
    double             init_g          = 0.0;
    ImuInitDiagnostics init_diagnostics;
    std::string        init_failure;
    // 被拒帧 diag bias 回填用 (上一接受值;IMU-off 恒 0)。
    Eigen::Vector3d last_reported_gyro_bias = Eigen::Vector3d::Zero();
    // M4.4 staged gyro activation.  Before activation the estimator graph is
    // exactly visual and accepted visual rotations accumulate here.  Once the
    // configured support is reached, one shared bias is solved and frozen as
    // the prior target; gyro factors start on the following update.
    std::vector<GyroBiasEvidence>        gyro_bias_evidence;
    double                               gyro_align_support_s            = 0.0;
    Eigen::Vector3d                      gyro_bias_prior                 = Eigen::Vector3d::Zero();
    double                               gyro_alignment_residual_rms_rad = 0.0;
    bool                                 gyro_fusion_active              = false;
    std::unique_ptr<FixedLagShadowState> fixed_lag_shadow;

    explicit Impl( camera::RectifiedStereoCalibration calibration_in,
                   EstimatorOptions                   options_in )
        : calibration( std::move( calibration_in ) ), options( std::move( options_in ) ), K( std::make_shared<gtsam::Cal3_S2Stereo>( calibration.fxPixels(), calibration.fyPixels(), 0.0, calibration.cxPixels(), calibration.cyPixels(), calibration.baselineM() ) ), body_P_sensor( toPose3( toIsometry( calibration.T_B_left_rectified() ) ) ), stereo_noise( makeStereoNoise( options ) ), prior_noise( makePriorNoise( options ) )
    {
      if ( options.window_size < 1 )
      {
        throw std::invalid_argument(
            "EstimatorOptions.window_size must be >= 1" );
      }
      if ( options.min_landmark_observations < 1 )
      {
        throw std::invalid_argument(
            "EstimatorOptions.min_landmark_observations must be >= 1" );
      }
      if ( options.min_seed_observations < 1 )
      {
        throw std::invalid_argument(
            "EstimatorOptions.min_seed_observations must be >= 1" );
      }
      if ( options.stereo_sigma_px <= 0.0 )
      {
        throw std::invalid_argument(
            "EstimatorOptions.stereo_sigma_px must be > 0" );
      }
      if ( !( options.pnp_reproj_px > 0.0 ) )
      {
        throw std::invalid_argument(
            "EstimatorOptions.pnp_reproj_px must be > 0" );
      }
      if ( !( options.pnp_confidence > 0.0 && options.pnp_confidence < 1.0 ) )
      {
        throw std::invalid_argument(
            "EstimatorOptions.pnp_confidence must be in (0, 1)" );
      }
      if ( options.min_pnp_inliers < 4 )
      {
        throw std::invalid_argument(
            "EstimatorOptions.min_pnp_inliers must be >= 4" );
      }
      if ( !( options.outlier_avg_reproj_px > 0.0 ) )
      {
        throw std::invalid_argument(
            "EstimatorOptions.outlier_avg_reproj_px must be > 0" );
      }
      if ( options.max_outlier_reopts < 0 )
      {
        throw std::invalid_argument(
            "EstimatorOptions.max_outlier_reopts must be >= 0" );
      }
      if ( options.enable_fixed_lag_shadow && !options.enable_imu )
      {
        throw std::invalid_argument(
            "fixed-lag shadow requires EstimatorOptions.enable_imu" );
      }
      if ( options.enable_imu )
      {
        if ( !( options.imu_gravity > 0.0 ) )
        {
          throw std::invalid_argument(
              "EstimatorOptions.imu_gravity must be > 0" );
        }
        if ( !( options.imu_gyr_noise_nd > 0.0 ) )
        {
          throw std::invalid_argument(
              "EstimatorOptions.imu_gyr_noise_nd must be > 0" );
        }
        if ( !( options.imu_prior_pose_sigma > 0.0 ) ||
             !( options.imu_prior_bias_gyro_sigma > 0.0 ) )
        {
          throw std::invalid_argument(
              "EstimatorOptions gyro prior sigmas must be > 0" );
        }
        if ( !( options.imu_init_window_s > 0.0 ) ||
             !( options.imu_init_gyro_std > 0.0 ) ||
             !( options.imu_init_accel_std > 0.0 ) ||
             !( options.imu_init_timeout_s > 0.0 ) ||
             !( options.imu_gyro_align_window_s > 0.0 ) )
        {
          throw std::invalid_argument(
              "EstimatorOptions IMU init/alignment thresholds must be > 0" );
        }
        ahrs_params =
            std::make_shared<gtsam::PreintegratedRotationParams>();
        ahrs_params->setGyroscopeCovariance(
            Eigen::Matrix3d::Identity() * options.imu_gyr_noise_nd *
            options.imu_gyr_noise_nd );
        imu_pose_prior_noise     = makeImuPosePriorNoise( options );
        imu_gyr_bias_prior_noise = gtsam::noiseModel::Isotropic::Sigma(
            3, options.imu_prior_bias_gyro_sigma );
      }
    }

    void eraseLandmarkFromWindow( LandmarkId id )
    {
      for ( WindowFrame& frame : window )
      {
        auto& obs = frame.observations;
        obs.erase( std::remove_if( obs.begin(), obs.end(),
                                   [ id ]( const StereoObservation& o ) {
                                     return o.id == id;
                                   } ),
                   obs.end() );
      }
    }

    [[nodiscard]] Eigen::Vector3d backprojectWorld(
        const Eigen::Isometry3d& T_W_B,
        const StereoObservation& observation ) const
    {
      const gtsam::Pose3        T_W_left = toPose3( T_W_B ) * body_P_sensor;
      const gtsam::StereoCamera camera( T_W_left, K );
      const gtsam::Point3       point_W =
          camera.backproject( toStereoPoint( observation ) );
      return Eigen::Vector3d( point_W.x(), point_W.y(), point_W.z() );
    }

    // 返回 false 表示 backproject 失败（调用方回滚并 kRejected）
    bool seedSegment( const Eigen::Isometry3d&   anchor_T_W_B,
                      const KeyframeMeasurement& measurement,
                      std::uint32_t&             probe_rejected_block_n,
                      std::uint32_t&             probe_new_lm_n )
    {
      landmarks_W.clear();

      WindowFrame candidate;
      candidate.frame_index  = next_frame_index;
      candidate.timestamp    = measurement.timestamp;
      candidate.observations = measurement.observations;
      candidate.T_W_B        = anchor_T_W_B;
      candidate.is_keyframe  = true;  // seed/re-anchor frames are keyframes
      // A seed/re-anchor is the head of a new interval chain. Marking it as a
      // gap prevents an AHRS factor from crossing the segment boundary.
      candidate.imu_gap = true;
      candidate.t_prev  = measurement.t_prev;

      for ( const StereoObservation& observation : measurement.observations )
      {
        if ( options.block_culled_rebirth &&
             culled_ids_.find( observation.id ) != culled_ids_.end() )
        {
          ++probe_rejected_block_n;
          continue;
        }
        // Slice ⑦: seed/re-anchor has no window history — a zero-disparity
        // observation cannot be backprojected (infinite depth); it is
        // seeded on a later keyframe once the window is rebuilt.
        if ( observation.disparity_px <= 0.0 )
        {
          continue;
        }
        const Eigen::Vector3d point_W =
            backprojectWorld( candidate.T_W_B, observation );
        const gtsam::Pose3 T_W_left =
            toPose3( candidate.T_W_B ) * body_P_sensor;
        const gtsam::Point3 point_left = T_W_left.transformTo( gtsam::Point3(
            point_W.x(), point_W.y(), point_W.z() ) );
        if ( !isFinite( point_W ) || point_left.z() <= 0.0 )
        {
          return false;
        }
        landmarks_W[ observation.id ] = point_W;
        track_times[ observation.id ].push_back( measurement.timestamp );
        ++probe_new_lm_n;
      }

      if ( landmarks_W.empty() )
      {
        return false;
      }

      window.clear();
      window.push_back( std::move( candidate ) );
      ++next_frame_index;
      return true;
    }

    [[nodiscard]] Eigen::Isometry3d poseInitialValue() const
    {
      if ( !last_accepted_T_W_B.has_value() )
      {
        return Eigen::Isometry3d::Identity();
      }
      if ( !options.use_constant_velocity_init ||
           !prev_accepted_T_W_B.has_value() )
      {
        return *last_accepted_T_W_B;
      }
      const Eigen::Isometry3d& T_prev     = *last_accepted_T_W_B;
      const Eigen::Isometry3d& T_prevprev = *prev_accepted_T_W_B;
      const Eigen::Isometry3d  predicted =
          T_prev * ( T_prevprev.inverse() * T_prev );
      if ( !isFinite( predicted ) )
      {
        return T_prev;
      }
      return predicted;
    }

    // M4.1 interval semantics: sample i covers [t_i, t_{i+1}); the right
    // endpoint is not integrated, so Σdt equals t_cur - t_prev.
    [[nodiscard]] std::shared_ptr<gtsam::PreintegratedAhrsMeasurements>
    rebuildAhrsPreintegration(
        const std::vector<sensor::ImuMeasurement>& samples,
        const Eigen::Vector3d&                     bias_gyr ) const
    {
      auto preint = std::make_shared<gtsam::PreintegratedAhrsMeasurements>(
          ahrs_params, bias_gyr );
      for ( std::size_t i = 1; i < samples.size(); ++i )
      {
        const double dt =
            static_cast<double>( samples[ i ].timestamp.nanoseconds() -
                                 samples[ i - 1 ].timestamp.nanoseconds() ) *
            1e-9;
        if ( dt <= 0.0 )
        {
          continue;
        }
        const auto& gyro = samples[ i - 1 ].gyro_radps;
        preint->integrateMeasurement(
            Eigen::Vector3d( gyro[ 0 ], gyro[ 1 ], gyro[ 2 ] ), dt );
      }
      return preint;
    }

    [[nodiscard]] bool estimateGyroBias(
        Eigen::Vector3d& bias_out, double& residual_rms_out,
        std::string& error_out ) const
    {
      if ( gyro_bias_evidence.empty() )
      {
        error_out = "gyro bias alignment has no visual rotation evidence";
        return false;
      }

      Eigen::Vector3d bias = gyro_bias_prior;
      for ( int iteration = 0; iteration < 8; ++iteration )
      {
        Eigen::Matrix3d normal   = Eigen::Matrix3d::Zero();
        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
        for ( const GyroBiasEvidence& evidence : gyro_bias_evidence )
        {
          const auto preint = rebuildAhrsPreintegration(
              evidence.imu_samples, bias );
          const gtsam::AHRSFactor factor( 0, 1, 2, *preint );
          gtsam::Matrix           H_bias;
          const gtsam::Vector     residual = factor.evaluateError(
              evidence.R_W_B_i, evidence.R_W_B_j, bias, nullptr, nullptr,
              &H_bias );
          if ( residual.size() != 3 || H_bias.rows() != 3 ||
               H_bias.cols() != 3 || !residual.allFinite() ||
               !H_bias.allFinite() )
          {
            error_out = "gyro bias alignment produced non-finite Jacobian";
            return false;
          }
          normal.noalias() += H_bias.transpose() * H_bias;
          gradient.noalias() += H_bias.transpose() * residual;
        }

        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen( normal );
        if ( eigen.info() != Eigen::Success ||
             !eigen.eigenvalues().allFinite() ||
             eigen.eigenvalues().minCoeff() <= 1e-12 ||
             eigen.eigenvalues().maxCoeff() /
                     eigen.eigenvalues().minCoeff() >
                 1e8 )
        {
          error_out = "gyro bias alignment normal matrix is ill-conditioned";
          return false;
        }
        const Eigen::LDLT<Eigen::Matrix3d> solver( normal );
        if ( solver.info() != Eigen::Success )
        {
          error_out = "gyro bias alignment normal solve failed";
          return false;
        }
        const Eigen::Vector3d delta = solver.solve( -gradient );
        if ( solver.info() != Eigen::Success || !delta.allFinite() )
        {
          error_out = "gyro bias alignment produced a non-finite update";
          return false;
        }
        bias += delta;
        if ( !bias.allFinite() )
        {
          error_out = "gyro bias alignment produced a non-finite bias";
          return false;
        }
        if ( delta.norm() < 1e-10 )
        {
          break;
        }
      }
      bias_out               = bias;
      double residual_sum_sq = 0.0;
      for ( const GyroBiasEvidence& evidence : gyro_bias_evidence )
      {
        const auto preint = rebuildAhrsPreintegration(
            evidence.imu_samples, bias );
        const gtsam::AHRSFactor factor( 0, 1, 2, *preint );
        const gtsam::Vector     residual = factor.evaluateError(
            evidence.R_W_B_i, evidence.R_W_B_j, bias );
        if ( residual.size() != 3 || !residual.allFinite() )
        {
          error_out = "gyro bias alignment produced a non-finite residual";
          return false;
        }
        residual_sum_sq += residual.squaredNorm();
      }
      residual_rms_out = std::sqrt(
          residual_sum_sq /
          static_cast<double>( gyro_bias_evidence.size() ) );
      if ( !std::isfinite( residual_rms_out ) )
      {
        error_out = "gyro bias alignment residual RMS is non-finite";
        return false;
      }
      return true;
    }

    void retainContiguousGyroSuffix()
    {
      if ( window.empty() )
      {
        return;
      }
      std::size_t first = window.size() - 1;
      while ( first > 0 )
      {
        const WindowFrame& prev = window[ first - 1 ];
        const WindowFrame& cur  = window[ first ];
        if ( cur.imu_gap || cur.imu_samples.size() < 2 ||
             cur.t_prev != prev.timestamp )
        {
          break;
        }
        --first;
      }
      if ( first > 0 )
      {
        window.erase( window.begin(),
                      window.begin() + static_cast<std::ptrdiff_t>( first ) );
      }
      WindowFrame& anchor = window.front();
      anchor.imu_samples.clear();
      anchor.imu_gap = true;
      anchor.t_prev  = anchor.timestamp;
      pruneLandmarksNotInWindow();
    }

    // Gyro-first: rotation 初值来自 AHRS 预积分，translation 仍沿用视觉
    // 恒速 guess；accelerometer 不参与 prediction。
    [[nodiscard]] bool gyroPredict(
        const std::vector<sensor::ImuMeasurement>& samples,
        bool                                       gap,
        Eigen::Isometry3d&                         pose_out,
        Eigen::Vector3d&                           bias_out ) const
    {
      if ( !options.enable_imu || !gyro_fusion_active ||
           !last_accepted_T_W_B.has_value() ||
           window.empty() || gap || samples.size() < 2 )
      {
        return false;
      }
      const WindowFrame& last   = window.back();
      const auto         preint = rebuildAhrsPreintegration(
          samples, last.gyro_bias );
      Eigen::Isometry3d T = poseInitialValue();
      T.linear() =
          last_accepted_T_W_B->linear() * preint->deltaRij().matrix();
      if ( !isFinite( T ) )
      {
        return false;
      }
      pose_out = T;
      bias_out = last.gyro_bias;
      return true;
    }

    // M4.3: 静止检测 (设计稿 §5.1)。窗口 = init_imu 尾部跨度 ≥
    // imu_init_window_s 的连续样本;逐轴 std 全部低于阈值 → 静止。
    // 通过: 填充 init 状态 (t0 / R_W_I0 / bias0 / g), 缓冲裁剪为窗口
    // 部分 (首条预积分段 = [t0, t_cur])。失败: 不裁剪, 滑动窗口下帧
    // 重试。返回 false 也可能表示缓冲尚不足窗口长度。
    [[nodiscard]] bool runStaticInitDetection()
    {
      if ( init_imu.size() < 2 )
      {
        return false;
      }
      const std::int64_t window_ns = static_cast<std::int64_t>(
          std::llround( options.imu_init_window_s * 1e9 ) );
      std::size_t        start = init_imu.size() - 1;
      const std::int64_t t_end = init_imu.back().timestamp.nanoseconds();
      while ( start > 0 &&
              t_end - init_imu[ start ].timestamp.nanoseconds() <
                  window_ns )
      {
        --start;  // 当前起点跨窗不足 → 前移一位 (更早起点)
      }
      if ( t_end - init_imu[ start ].timestamp.nanoseconds() < window_ns )
      {
        return false;  // 缓冲不足一个窗口
      }

      Eigen::Vector3d   g_sum = Eigen::Vector3d::Zero();
      Eigen::Vector3d   a_sum = Eigen::Vector3d::Zero();
      const std::size_t n     = init_imu.size() - start;
      for ( std::size_t i = start; i < init_imu.size(); ++i )
      {
        const auto& m = init_imu[ i ];
        g_sum += Eigen::Vector3d( m.gyro_radps[ 0 ], m.gyro_radps[ 1 ],
                                  m.gyro_radps[ 2 ] );
        a_sum += Eigen::Vector3d( m.accel_mps2[ 0 ], m.accel_mps2[ 1 ],
                                  m.accel_mps2[ 2 ] );
      }
      const Eigen::Vector3d g_mean = g_sum / static_cast<double>( n );
      const Eigen::Vector3d a_mean = a_sum / static_cast<double>( n );
      Eigen::Vector3d       g_var  = Eigen::Vector3d::Zero();
      Eigen::Vector3d       a_var  = Eigen::Vector3d::Zero();
      for ( std::size_t i = start; i < init_imu.size(); ++i )
      {
        const auto&           m = init_imu[ i ];
        const Eigen::Vector3d g( m.gyro_radps[ 0 ], m.gyro_radps[ 1 ],
                                 m.gyro_radps[ 2 ] );
        const Eigen::Vector3d a( m.accel_mps2[ 0 ], m.accel_mps2[ 1 ],
                                 m.accel_mps2[ 2 ] );
        const Eigen::Vector3d dg = g - g_mean;
        const Eigen::Vector3d da = a - a_mean;
        g_var += dg.cwiseProduct( dg );
        a_var += da.cwiseProduct( da );
      }
      const Eigen::Vector3d g_std =
          ( g_var / static_cast<double>( n ) ).cwiseSqrt();
      const Eigen::Vector3d a_std =
          ( a_var / static_cast<double>( n ) ).cwiseSqrt();
      if ( g_std.maxCoeff() >= options.imu_init_gyro_std ||
           a_std.maxCoeff() >= options.imu_init_accel_std )
      {
        return false;  // 运动中: 滑动窗口下帧重试
      }

      // 通过: t0 = 窗口起点 (第一个状态帧, D10); R_W_I0 使 body 系"上"
      // (gravity_dir = 静止时比力方向, 即 -n_gravity 方向) 对齐世界 Z
      // (roll/pitch), yaw 自由 (C5, ATE 吸收)。静止时世界 Z 即与 -g_w
      // 对齐。注意目标必须是 +UnitZ: FromTwoVectors(gravity_dir, -UnitZ)
      // 会把传感器"上"映射到世界"下" (π 翻转, 非 gauge, 首个运动段
      // IMU 残差 -2g·Δt 与 X prior 冲突)。
      init_t0                           = init_imu[ start ].timestamp;
      const Eigen::Vector3d gravity_dir = a_mean.normalized();
      init_g                            = a_mean.norm();
      if ( !std::isfinite( init_g ) || init_g <= 0.0 )
      {
        init_g = options.imu_gravity;  // C10 兜底
      }
      init_T_W_I0 = Eigen::Isometry3d( Eigen::Quaterniond::FromTwoVectors(
          gravity_dir, Eigen::Vector3d::UnitZ() ) );
      // acc bias init 不能用 init_g·gravity_dir (= a_mean, 恒等式) →
      // acc_bias ≡ 0 (首图 gauge 无梯度, LM 停在 0, 积分不消除本征 acc
      // bias → 位置按 ½·b·t² 漂移, MH_01 ATE 0.3518)。改用常数真重力
      // (imu_gravity): acc_bias = a_mean − 9.81007·ẑ_body ≈ 真值
      // (−0.0333, +0.0018, +0.0091)。静止假设下 a_mean 与真重力只差
      // scale/tilt 混淆 (门① 归因, 见 benchmark m4.3 checkpoint)。
      const Eigen::Vector3d acc_bias =
          a_mean - options.imu_gravity * gravity_dir;
      init_gyro_bias0                   = g_mean;
      init_acc_bias0                    = acc_bias;
      gyro_bias_prior                   = g_mean;
      init_diagnostics                  = ImuInitDiagnostics{};
      init_diagnostics.imu_sample_count = static_cast<std::uint32_t>( n );
      init_diagnostics.imu_t_i_ns       = init_t0.nanoseconds();
      init_diagnostics.imu_t_j_ns       = t_end;
      init_diagnostics.imu_dt_s =
          static_cast<double>( t_end - init_t0.nanoseconds() ) * 1e-9;
      init_diagnostics.gyro_mean               = g_mean;
      init_diagnostics.gyro_std                = g_std;
      init_diagnostics.acc_mean                = a_mean;
      init_diagnostics.acc_std                 = a_std;
      init_diagnostics.T_W_B0                  = init_T_W_I0;
      init_diagnostics.velocity_W              = Eigen::Vector3d::Zero();
      init_diagnostics.bias_gyro               = init_gyro_bias0;
      init_diagnostics.bias_acc                = acc_bias;
      init_diagnostics.acc_mean_norm           = init_g;
      init_diagnostics.gravity_model_magnitude = options.imu_gravity;
      init_diagnostics.gyro_std_limit          = options.imu_init_gyro_std;
      init_diagnostics.acc_std_limit           = options.imu_init_accel_std;
      init_imu.erase(
          init_imu.begin(),
          init_imu.begin() + static_cast<std::ptrdiff_t>( start ) );
      return true;
    }

    [[nodiscard]] bool frameHasGyroFactor( std::size_t index_in_window ) const
    {
      if ( !options.enable_imu || !gyro_fusion_active ||
           index_in_window == 0 )
      {
        return false;
      }
      const WindowFrame& frame = window[ index_in_window ];
      return !frame.imu_gap && frame.imu_samples.size() >= 2;
    }

    [[nodiscard]] std::uint32_t gyroFactorCount() const
    {
      std::uint32_t count = 0;
      for ( std::size_t j = 1; j < window.size(); ++j )
      {
        if ( frameHasGyroFactor( j ) )
        {
          ++count;
        }
      }
      return count;
    }

    struct PnpInitResult
    {
      bool              success = false;
      Eigen::Isometry3d T_W_B   = Eigen::Isometry3d::Identity();
      std::vector<int>  inlier_indices;  // into shared correspondence list
    };

    [[nodiscard]] std::optional<double> stereoRmsAtPose(
        const Eigen::Isometry3d&                     T_W_B,
        const std::vector<const StereoObservation*>& shared_observations,
        const std::vector<int>&                      inlier_indices ) const
    {
      if ( !isFinite( T_W_B ) || inlier_indices.empty() )
      {
        return std::nullopt;
      }

      const gtsam::Pose3        T_W_left = toPose3( T_W_B ) * body_P_sensor;
      const gtsam::StereoCamera camera( T_W_left, K );
      double                    sum_sq = 0.0;
      for ( const int index : inlier_indices )
      {
        if ( index < 0 ||
             static_cast<std::size_t>( index ) >= shared_observations.size() )
        {
          return std::nullopt;
        }
        const StereoObservation* observation =
            shared_observations[ static_cast<std::size_t>( index ) ];
        if ( observation == nullptr )
        {
          return std::nullopt;
        }
        // Slice ⑦: a zero-disparity observation carries no stereo
        // information — toStereoPoint would fabricate a right pixel equal to
        // the left one and inflate this pose's RMS (it still supports the PnP
        // correspondence itself via its left pixel).
        if ( observation->disparity_px <= 0.0 )
        {
          continue;
        }
        const auto landmark_it = landmarks_W.find( observation->id );
        if ( landmark_it == landmarks_W.end() ||
             !isFinite( landmark_it->second ) )
        {
          return std::nullopt;
        }

        const gtsam::Point3 point_W( landmark_it->second.x(),
                                     landmark_it->second.y(),
                                     landmark_it->second.z() );
        const gtsam::Point3 point_left = T_W_left.transformTo( point_W );
        if ( !point_left.allFinite() || point_left.z() <= 0.0 )
        {
          return std::nullopt;
        }

        gtsam::StereoPoint2 projected;
        try
        {
          projected = camera.project( point_W );
        }
        catch ( const gtsam::StereoCheiralityException& )
        {
          return std::nullopt;
        }
        const gtsam::Vector residual =
            ( projected - toStereoPoint( *observation ) ).vector();
        if ( !residual.allFinite() )
        {
          return std::nullopt;
        }
        sum_sq += residual.squaredNorm();
        if ( !std::isfinite( sum_sq ) )
        {
          return std::nullopt;
        }
      }
      return std::sqrt( sum_sq /
                        static_cast<double>( inlier_indices.size() ) );
    }

    // Shared landmarks_W ∩ current left observations → solvePnPRansac.
    // A finite, sufficiently supported proposal is accepted only when its
    // stereo RMS on the PnP inlier set is no more than one observation sigma
    // worse than the existing guess.
    [[nodiscard]] PnpInitResult tryPnpInit(
        const KeyframeMeasurement& measurement,
        const Eigen::Isometry3d&   guess_T_W_B ) const
    {
      PnpInitResult result;

      std::vector<cv::Point3d>              pts3d;
      std::vector<cv::Point2d>              pts2d;
      std::vector<const StereoObservation*> shared_observations;
      pts3d.reserve( measurement.observations.size() );
      pts2d.reserve( measurement.observations.size() );
      shared_observations.reserve( measurement.observations.size() );
      for ( const StereoObservation& observation : measurement.observations )
      {
        const auto landmark_it = landmarks_W.find( observation.id );
        if ( landmark_it == landmarks_W.end() )
        {
          continue;
        }
        // Slice ⑦: zero-disparity observations cannot constrain PnP — they
        // have no stereo depth, and the stereo RMS acceptance gate cannot
        // see them.
        if ( observation.disparity_px <= 0.0 )
        {
          continue;
        }
        const Eigen::Vector3d& point_W = landmark_it->second;
        pts3d.emplace_back( point_W.x(), point_W.y(), point_W.z() );
        pts2d.emplace_back( observation.left_pixel.x(),
                            observation.left_pixel.y() );
        shared_observations.push_back( &observation );
      }
      if ( static_cast<int>( pts3d.size() ) < options.min_pnp_inliers )
      {
        return result;
      }

      const Eigen::Isometry3d T_B_left       = toIsometry( body_P_sensor );
      const Eigen::Isometry3d T_W_left_guess = guess_T_W_B * T_B_left;
      const Eigen::Isometry3d T_left_W_guess = T_W_left_guess.inverse();

      cv::Mat R_guess( 3, 3, CV_64F );
      for ( int row = 0; row < 3; ++row )
      {
        for ( int col = 0; col < 3; ++col )
        {
          R_guess.at<double>( row, col ) =
              T_left_W_guess.linear()( row, col );
        }
      }
      cv::Mat rvec;
      cv::Mat tvec;
      cv::Rodrigues( R_guess, rvec );
      tvec = ( cv::Mat_<double>( 3, 1 ) << T_left_W_guess.translation().x(),
               T_left_W_guess.translation().y(),
               T_left_W_guess.translation().z() );

      const cv::Mat K =
          ( cv::Mat_<double>( 3, 3 ) << calibration.fxPixels(), 0.0,
            calibration.cxPixels(), 0.0, calibration.fyPixels(),
            calibration.cyPixels(), 0.0, 0.0, 1.0 );

      cv::Mat inliers;
      bool    solved = false;
      try
      {
        solved = cv::solvePnPRansac(
            pts3d, pts2d, K, cv::noArray(), rvec, tvec,
            /*useExtrinsicGuess=*/true, /*iterationsCount=*/100,
            static_cast<float>( options.pnp_reproj_px ),
            options.pnp_confidence, inliers, cv::SOLVEPNP_ITERATIVE );
      }
      catch ( const cv::Exception& )
      {
        return result;
      }
      if ( !solved || inliers.empty() )
      {
        return result;
      }

      std::vector<int> inlier_indices;
      inlier_indices.reserve( static_cast<std::size_t>( inliers.rows ) );
      for ( int row = 0; row < inliers.rows; ++row )
      {
        inlier_indices.push_back( inliers.at<int>( row, 0 ) );
      }
      if ( static_cast<int>( inlier_indices.size() ) < options.min_pnp_inliers )
      {
        return result;
      }

      cv::Mat R_left_W;
      cv::Rodrigues( rvec, R_left_W );
      Eigen::Matrix3d rotation;
      Eigen::Vector3d translation;
      for ( int row = 0; row < 3; ++row )
      {
        translation( row ) = tvec.at<double>( row, 0 );
        for ( int col = 0; col < 3; ++col )
        {
          rotation( row, col ) = R_left_W.at<double>( row, col );
        }
      }
      Eigen::Quaterniond quat( rotation );
      quat.normalize();
      Eigen::Isometry3d T_left_W = Eigen::Isometry3d::Identity();
      T_left_W.linear()          = quat.toRotationMatrix();
      T_left_W.translation()     = translation;

      const Eigen::Isometry3d T_W_left = T_left_W.inverse();
      const Eigen::Isometry3d T_W_B    = T_W_left * T_B_left.inverse();
      if ( !isFinite( T_W_B ) )
      {
        return result;
      }

      const std::optional<double> proposal_rms = stereoRmsAtPose(
          T_W_B, shared_observations, inlier_indices );
      if ( !proposal_rms.has_value() )
      {
        return result;
      }
      const std::optional<double> guess_rms = stereoRmsAtPose(
          guess_T_W_B, shared_observations, inlier_indices );
      if ( guess_rms.has_value() &&
           *proposal_rms > *guess_rms + options.stereo_sigma_px )
      {
        return result;
      }

      result.success        = true;
      result.T_W_B          = T_W_B;
      result.inlier_indices = std::move( inlier_indices );
      return result;
    }

    // Slice ⑦: most recent frame in the window that observed `id` with
    // stereo (disparity > 0), if any.
    [[nodiscard]] std::optional<Eigen::Isometry3d> findLastStereoFrameInWindow(
        LandmarkId id ) const
    {
      std::optional<Eigen::Isometry3d> last_stereo_pose;
      for ( auto it = window.rbegin(); it != window.rend(); ++it )
      {
        for ( const StereoObservation& observation : it->observations )
        {
          if ( observation.id == id && observation.disparity_px > 0.0 )
          {
            last_stereo_pose = it->T_W_B;
            break;
          }
        }
        if ( last_stereo_pose.has_value() )
        {
          break;
        }
      }
      return last_stereo_pose;
    }

    // Slice ⑦: a landmark kept alive only by zero-disparity observations
    // ("hanging") carries a 3D point that is stale once the body has moved
    // far since its last stereo observation — E10 showed the hanging
    // mechanism is the sole carrier of the Slice ⑦ effect, good (V2_02) or
    // bad (MH_03/V2_01). Gate the stale half out.
    [[nodiscard]] bool hangingLandmarkStale(
        LandmarkId id, const Eigen::Isometry3d& current_T_W_B ) const
    {
      // 0 disables the gate entirely (e9a21b3 behavior — keep every
      // hanging landmark). MUST be checked before the no-stereo-in-window
      // early return: with the gate off, an absent last stereo frame is
      // not a reason to drop (E12d2 caught this: the early return fired
      // unconditionally, silently turning gate=0 runs into the E11
      // full-revert and making every E12 experiment dead code).
      if ( options.hanging_landmark_gate_m <= 0.0 )
      {
        return false;
      }
      // E13: measure against the persistent last-stereo pose (survives the
      // frame leaving the window). findLastStereoFrameInWindow would return
      // nullopt for every far-return candidate — an unconditional drop that
      // made E11's gate sweep a never-tested distance hypothesis.
      const auto it = last_stereo_pose_W.find( id );
      if ( it == last_stereo_pose_W.end() )
      {
        // No stereo observation on record (e.g. triangulated-only seed) —
        // infinitely stale.
        return true;
      }
      const double moved_m =
          ( it->second.translation() - current_T_W_B.translation() ).norm();
      // Gate configurable via options.hanging_landmark_gate_m (bench CLI
      // --hanging-gate-m).
      return moved_m > options.hanging_landmark_gate_m;
    }

    void pruneLandmarksNotInWindow()
    {
      std::unordered_set<LandmarkId> live;
      std::unordered_set<LandmarkId> zero_live;
      for ( const WindowFrame& frame : window )
      {
        for ( const StereoObservation& observation : frame.observations )
        {
          if ( observation.disparity_px > 0.0 )
          {
            live.insert( observation.id );
          }
          else
          {
            zero_live.insert( observation.id );
          }
        }
      }
      for ( auto it = landmarks_W.begin(); it != landmarks_W.end(); )
      {
        if ( live.find( it->first ) == live.end() )
        {
          if ( zero_live.find( it->first ) != zero_live.end() &&
               !hangingLandmarkStale( it->first, window.back().T_W_B ) )
          {
            // Fresh hanging landmark (Slice ⑦): keep — it constrains the
            // BA once stereo returns and stabilizes the track.
            ++it;
            continue;
          }
          // Keep track_times for diagnostics across the whole run.
          last_stereo_pose_W.erase( it->first );
          it = landmarks_W.erase( it );
        }
        else
        {
          ++it;
        }
      }
    }

    // Slice ⑦: count only stereo observations in a measurement — those are
    // the ones that can seed a segment / constrain the BA graph. Zero-disparity
    // observations must not inflate the init / re-anchor gates (a gate that
    // passes on ~180 no-depth observations but seeds ~18 weak landmarks leaves
    // an almost factor-free graph that drifts freely).
    [[nodiscard]] std::size_t countStereoObservations(
        const KeyframeMeasurement& measurement ) const
    {
      return static_cast<std::size_t>( std::count_if(
          measurement.observations.begin(), measurement.observations.end(),
          []( const StereoObservation& observation ) {
            return observation.disparity_px > 0.0;
          } ) );
    }

    // Slice ⑦: count only stereo observations (disparity_px > 0) — those are
    // the ones that become BA factors. A landmark whose window observations
    // are all zero-disparity must NOT enter the graph: it would carry no
    // factor, and counting a mixed landmark's zero-disparity observations
    // toward min_landmark_observations could admit a 1-factor point that
    // slides freely along its ray.
    [[nodiscard]] std::unordered_map<LandmarkId, int> countObservations()
        const
    {
      std::unordered_map<LandmarkId, int> counts;
      for ( const WindowFrame& frame : window )
      {
        for ( const StereoObservation& observation : frame.observations )
        {
          if ( observation.disparity_px > 0.0 )
          {
            ++counts[ observation.id ];
          }
        }
      }
      return counts;
    }

    void buildGraph( gtsam::NonlinearFactorGraph& graph,
                     gtsam::Values&               values,
                     std::uint64_t&               prior_key_out,
                     std::uint32_t&               num_landmarks_out ) const
    {
      graph.resize( 0 );
      values.clear();
      num_landmarks_out = 0;
      if ( window.empty() )
      {
        prior_key_out = 0;
        return;
      }

      const std::uint64_t oldest = window.front().frame_index;
      prior_key_out              = oldest;
      for ( const WindowFrame& frame : window )
      {
        values.insert( X( frame.frame_index ), toPose3( frame.T_W_B ) );
      }
      // Alignment warm-up is a pure visual graph.  The IMU pose prior is used
      // only after gyro factors actually activate.
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          X( oldest ), toPose3( window.front().T_W_B ),
          gyro_fusion_active ? imu_pose_prior_noise : prior_noise );

      // ---- M4.4 B: gyro-first pose factor + shared gyro bias ----
      if ( gyro_fusion_active )
      {
        const Eigen::Vector3d bias_gyr = window.front().gyro_bias;
        values.insert( W( 0 ), bias_gyr );
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
            W( 0 ), gyro_bias_prior, imu_gyr_bias_prior_noise );

        for ( std::size_t j = 1; j < window.size(); ++j )
        {
          const WindowFrame& prev = window[ j - 1 ];
          const WindowFrame& cur  = window[ j ];
          if ( !frameHasGyroFactor( j ) )
          {
            continue;
          }
          const auto preint = rebuildAhrsPreintegration(
              cur.imu_samples, prev.gyro_bias );
          graph.emplace_shared<PoseAhrsFactor>(
              X( prev.frame_index ), X( cur.frame_index ), W( 0 ), *preint,
              gyro_alignment_residual_rms_rad );
        }
      }

      const auto                     counts = countObservations();
      std::unordered_set<LandmarkId> inserted;
      for ( const WindowFrame& frame : window )
      {
        for ( const StereoObservation& observation : frame.observations )
        {
          const auto count_it = counts.find( observation.id );
          if ( count_it == counts.end() ||
               count_it->second < options.min_landmark_observations )
          {
            continue;
          }
          const auto landmark_it = landmarks_W.find( observation.id );
          if ( landmark_it == landmarks_W.end() )
          {
            continue;
          }
          if ( inserted.insert( observation.id ).second )
          {
            values.insert(
                L( observation.id ),
                gtsam::Point3( landmark_it->second.x(),
                               landmark_it->second.y(),
                               landmark_it->second.z() ) );
            ++num_landmarks_out;
          }
          // Slice ⑦: zero-disparity observations are not stereo measurements
          // — building a StereoFactor from them would project a degenerate
          // right pixel.
          if ( observation.disparity_px > 0.0 )
          {
            graph.emplace_shared<
                gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
                toStereoPoint( observation ), stereo_noise,
                X( frame.frame_index ), L( observation.id ), K,
                body_P_sensor );
          }
        }
      }
    }

    [[nodiscard]] static ShadowFactorId shadowUnaryFactor(
        const ShadowFactorKind kind, const gtsam::Key key )
    {
      return ShadowFactorId{ kind, { key, 0, 0 }, 1 };
    }

    [[nodiscard]] static ShadowFactorId shadowBinaryFactor(
        const ShadowFactorKind kind, const gtsam::Key key_i,
        const gtsam::Key key_j )
    {
      return ShadowFactorId{ kind, { key_i, key_j, 0 }, 2 };
    }

    [[nodiscard]] static ShadowFactorId shadowTernaryFactor(
        const ShadowFactorKind kind, const gtsam::Key key_i,
        const gtsam::Key key_j, const gtsam::Key key_k )
    {
      return ShadowFactorId{ kind, { key_i, key_j, key_k }, 3 };
    }

    [[nodiscard]] static bool shadowOwnsAllKeys(
        const FixedLagShadowState& state, const ShadowFactorId& factor )
    {
      for ( std::uint8_t i = 0; i < factor.key_count; ++i )
      {
        if ( !state.smoother.getISAM2().valueExists( factor.keys[ i ] ) )
        {
          return false;
        }
      }
      return true;
    }

    [[nodiscard]] static std::uint32_t reconcileShadowFactorSlots(
        FixedLagShadowState& state )
    {
      std::uint32_t missing = 0;
      for ( auto it = state.owned_slots.begin();
            it != state.owned_slots.end(); )
      {
        if ( state.smoother.getFactors().exists( it->first ) )
        {
          ++it;
          continue;
        }
        if ( shadowOwnsAllKeys( state, it->second ) )
        {
          ++missing;
        }
        state.committed_factors.erase( it->second );
        it = state.owned_slots.erase( it );
      }
      return missing;
    }

    [[nodiscard]] static std::uint32_t countShadowTimestampOrphans(
        const FixedLagShadowState& state )
    {
      std::uint32_t count = 0;
      for ( const auto& [ key, timestamp ] : state.smoother.timestamps() )
      {
        (void)timestamp;
        if ( !state.smoother.getISAM2().valueExists( key ) )
        {
          ++count;
        }
      }
      return count;
    }

    [[nodiscard]] static std::set<gtsam::Key> shadowPoseKeys(
        const FixedLagShadowState& state )
    {
      std::set<gtsam::Key> keys;
      for ( const auto& [ key, timestamp ] : state.smoother.timestamps() )
      {
        (void)timestamp;
        if ( gtsam::Symbol( key ).chr() == 'x' &&
             state.smoother.getISAM2().valueExists( key ) )
        {
          keys.insert( key );
        }
      }
      return keys;
    }

    [[nodiscard]] static std::uint32_t countShadowLandmarks(
        const FixedLagShadowState& state )
    {
      std::uint32_t count = 0;
      for ( const auto& [ key, timestamp ] : state.smoother.timestamps() )
      {
        (void)timestamp;
        if ( gtsam::Symbol( key ).chr() == 'q' &&
             state.smoother.getISAM2().valueExists( key ) )
        {
          ++count;
        }
      }
      return count;
    }

    [[nodiscard]] FixedLagShadowDiagnostics updateFixedLagShadow()
    {
      if ( window.empty() )
      {
        throw std::runtime_error(
            "fixed-lag shadow cannot update an empty window" );
      }

      FixedLagShadowDiagnostics diagnostics;
      diagnostics.active        = true;
      diagnostics.current_epoch = window.back().frame_index;
      diagnostics.cutoff_epoch  = window.front().frame_index;
      diagnostics.batch_window_size =
          static_cast<std::uint32_t>( window.size() );

      const bool reset = fixed_lag_shadow == nullptr ||
                         fixed_lag_shadow->segment_id != segment_id;
      diagnostics.reset = reset;
      diagnostics.reset_reason =
          !reset ? FixedLagShadowReset::kNone
          : fixed_lag_shadow == nullptr
              ? FixedLagShadowReset::kBootstrap
              : FixedLagShadowReset::kSegment;

      const double                         lag = static_cast<double>( diagnostics.current_epoch -
                                                                      diagnostics.cutoff_epoch );
      std::unique_ptr<FixedLagShadowState> candidate =
          reset ? std::make_unique<FixedLagShadowState>( lag )
                : std::make_unique<FixedLagShadowState>(
                      *fixed_lag_shadow );
      candidate->segment_id             = segment_id;
      candidate->smoother.smootherLag() = lag;

      const std::set<gtsam::Key> poses_before = shadowPoseKeys( *candidate );
      diagnostics.missing_owned_slot_count =
          reconcileShadowFactorSlots( *candidate );
      if ( diagnostics.missing_owned_slot_count != 0U )
      {
        throw std::runtime_error(
            "fixed-lag shadow lost an owned factor while all keys survived" );
      }

      for ( auto it = candidate->live_landmarks.begin();
            it != candidate->live_landmarks.end(); )
      {
        const bool domain_present =
            landmarks_W.find( it->first ) != landmarks_W.end();
        const bool graph_present =
            candidate->smoother.getISAM2().valueExists( it->second );
        if ( domain_present && graph_present )
        {
          ++it;
          continue;
        }
        ++diagnostics.retired_landmark_count;
        it = candidate->live_landmarks.erase( it );
      }

      gtsam::NonlinearFactorGraph              new_factors;
      gtsam::Values                            new_values;
      gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps;
      std::vector<ShadowFactorId>              new_factor_ids;

      for ( const WindowFrame& frame : window )
      {
        const gtsam::Key key = X( frame.frame_index );
        if ( candidate->smoother.getISAM2().valueExists( key ) )
        {
          continue;
        }
        if ( !reset && frame.frame_index != window.back().frame_index )
        {
          throw std::runtime_error(
              "fixed-lag shadow is missing a retained production pose" );
        }
        new_values.insert( key, toPose3( frame.T_W_B ) );
        new_timestamps.emplace( key,
                                static_cast<double>( frame.frame_index ) );
      }

      if ( !candidate->smoother.getISAM2().valueExists( W( 0 ) ) )
      {
        if ( !reset )
        {
          throw std::runtime_error(
              "fixed-lag shadow lost the shared gyro bias" );
        }
        new_values.insert( W( 0 ), window.back().gyro_bias );
      }
      new_timestamps[ W( 0 ) ] =
          static_cast<double>( diagnostics.current_epoch );

      if ( reset )
      {
        const ShadowFactorId pose_prior = shadowUnaryFactor(
            ShadowFactorKind::kPosePrior, X( window.front().frame_index ) );
        candidate->committed_factors.insert( pose_prior );
        new_factor_ids.push_back( pose_prior );
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            X( window.front().frame_index ),
            toPose3( window.front().T_W_B ), imu_pose_prior_noise );

        const ShadowFactorId bias_prior =
            shadowUnaryFactor( ShadowFactorKind::kBiasPrior, W( 0 ) );
        candidate->committed_factors.insert( bias_prior );
        new_factor_ids.push_back( bias_prior );
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
            W( 0 ), gyro_bias_prior, imu_gyr_bias_prior_noise );
      }

      for ( std::size_t j = 1; j < window.size(); ++j )
      {
        if ( !frameHasGyroFactor( j ) )
        {
          continue;
        }
        const WindowFrame&   prev   = window[ j - 1 ];
        const WindowFrame&   cur    = window[ j ];
        const ShadowFactorId factor = shadowTernaryFactor(
            ShadowFactorKind::kAhrs, X( prev.frame_index ),
            X( cur.frame_index ), W( 0 ) );
        if ( !candidate->committed_factors.insert( factor ).second )
        {
          continue;
        }
        const auto preintegration =
            rebuildAhrsPreintegration( cur.imu_samples, prev.gyro_bias );
        new_factor_ids.push_back( factor );
        new_factors.emplace_shared<PoseAhrsFactor>(
            X( prev.frame_index ), X( cur.frame_index ), W( 0 ),
            *preintegration, gyro_alignment_residual_rms_rad );
      }

      const auto counts = countObservations();
      for ( const auto& [ id, point_W ] : landmarks_W )
      {
        const auto count_it = counts.find( id );
        if ( count_it == counts.end() ||
             count_it->second < options.min_landmark_observations )
        {
          continue;
        }
        if ( candidate->live_landmarks.find( id ) !=
             candidate->live_landmarks.end() )
        {
          continue;
        }
        const gtsam::Key key = gtsam::Symbol(
            'q', candidate->next_landmark_generation++ );
        candidate->live_landmarks.emplace( id, key );
        new_values.insert(
            key, gtsam::Point3( point_W.x(), point_W.y(), point_W.z() ) );
        ++diagnostics.new_landmark_generation_count;
      }

      for ( const WindowFrame& frame : window )
      {
        for ( const StereoObservation& observation : frame.observations )
        {
          if ( observation.disparity_px <= 0.0 )
          {
            continue;
          }
          const auto count_it = counts.find( observation.id );
          const auto landmark_it =
              candidate->live_landmarks.find( observation.id );
          if ( count_it == counts.end() ||
               count_it->second < options.min_landmark_observations ||
               landmark_it == candidate->live_landmarks.end() ||
               landmarks_W.find( observation.id ) == landmarks_W.end() )
          {
            continue;
          }
          const ShadowFactorId factor = shadowBinaryFactor(
              ShadowFactorKind::kStereo, X( frame.frame_index ),
              landmark_it->second );
          if ( !candidate->committed_factors.insert( factor ).second )
          {
            continue;
          }
          new_factor_ids.push_back( factor );
          new_factors.emplace_shared<
              gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
              toStereoPoint( observation ), stereo_noise,
              X( frame.frame_index ), landmark_it->second, K,
              body_P_sensor );
          new_timestamps[ landmark_it->second ] =
              static_cast<double>( frame.frame_index );
        }
      }

      candidate->smoother.update( new_factors, new_values, new_timestamps );
      const gtsam::FactorIndices& new_slots =
          candidate->smoother.getISAM2Result().newFactorsIndices;
      if ( new_slots.size() != new_factor_ids.size() )
      {
        throw std::runtime_error(
            "fixed-lag shadow factor slot result has the wrong size" );
      }
      for ( std::size_t i = 0; i < new_slots.size(); ++i )
      {
        candidate->owned_slots[ new_slots[ i ] ] = new_factor_ids[ i ];
      }

      diagnostics.missing_owned_slot_count +=
          reconcileShadowFactorSlots( *candidate );
      diagnostics.timestamp_without_value_count =
          countShadowTimestampOrphans( *candidate );
      if ( diagnostics.missing_owned_slot_count != 0U ||
           diagnostics.timestamp_without_value_count != 0U )
      {
        throw std::runtime_error(
            "fixed-lag shadow lifecycle invariant failed after update" );
      }

      const std::set<gtsam::Key> poses_after = shadowPoseKeys( *candidate );
      std::set<gtsam::Key>       expected_poses;
      for ( const WindowFrame& frame : window )
      {
        expected_poses.insert( X( frame.frame_index ) );
      }
      if ( poses_after != expected_poses )
      {
        throw std::runtime_error(
            "fixed-lag shadow pose set differs from the production window" );
      }
      for ( const gtsam::Key key : poses_before )
      {
        if ( poses_after.find( key ) == poses_after.end() )
        {
          ++diagnostics.marginalized_pose_count;
        }
      }

      diagnostics.smoother_pose_count =
          static_cast<std::uint32_t>( poses_after.size() );
      diagnostics.smoother_landmark_count =
          countShadowLandmarks( *candidate );
      diagnostics.user_factor_count =
          static_cast<std::uint32_t>( candidate->owned_slots.size() );
      diagnostics.bias_present =
          candidate->smoother.getISAM2().valueExists( W( 0 ) );
      diagnostics.bias_fixed =
          candidate->smoother.getISAM2().getFixedVariables().count( W( 0 ) ) !=
          0U;
      if ( !diagnostics.bias_present )
      {
        throw std::runtime_error(
            "fixed-lag shadow has no shared gyro bias after update" );
      }

      const gtsam::Pose3 newest_pose =
          candidate->smoother.calculateEstimate<gtsam::Pose3>(
              X( diagnostics.current_epoch ) );
      const Eigen::Vector3d bias =
          candidate->smoother.calculateEstimate<gtsam::Vector3>( W( 0 ) );
      diagnostics.newest_T_W_B = toIsometry( newest_pose );
      diagnostics.bias_delta_norm =
          ( bias - window.back().gyro_bias ).norm();
      diagnostics.newest_translation_delta_m =
          ( diagnostics.newest_T_W_B.translation() -
            window.back().T_W_B.translation() )
              .norm();
      diagnostics.newest_rotation_delta_rad = Eigen::AngleAxisd(
                                                  window.back().T_W_B.linear().transpose() *
                                                  diagnostics.newest_T_W_B.linear() )
                                                  .angle();
      if ( !isFinite( diagnostics.newest_T_W_B ) ||
           !std::isfinite( diagnostics.bias_delta_norm ) ||
           !std::isfinite( diagnostics.newest_translation_delta_m ) ||
           !std::isfinite( diagnostics.newest_rotation_delta_rad ) )
      {
        throw std::runtime_error(
            "fixed-lag shadow produced non-finite diagnostics" );
      }

      fixed_lag_shadow = std::move( candidate );
      return diagnostics;
    }

    [[nodiscard]] std::uint32_t dropCheiralityLandmarks(
        const gtsam::Values&     values,
        std::vector<LandmarkId>& frame_culled )
    {
      std::uint32_t           dropped = 0;
      std::vector<LandmarkId> to_drop;
      for ( const auto& [ id, point_W ] : landmarks_W )
      {
        const gtsam::Key key = L( id );
        if ( !values.exists( key ) )
        {
          continue;
        }
        const gtsam::Point3 optimized = values.at<gtsam::Point3>( key );
        bool                behind    = false;
        for ( const WindowFrame& frame : window )
        {
          bool observes = false;
          for ( const StereoObservation& observation : frame.observations )
          {
            // Slice ⑦: only stereo observations constrain the landmark — a
            // zero-disparity frame's pose drift must not be able to cull a
            // good landmark.
            if ( observation.id == id &&
                 observation.disparity_px > 0.0 )
            {
              observes = true;
              break;
            }
          }
          if ( !observes )
          {
            continue;
          }
          const gtsam::Pose3 T_W_B =
              values.exists( X( frame.frame_index ) )
                  ? values.at<gtsam::Pose3>( X( frame.frame_index ) )
                  : toPose3( frame.T_W_B );
          const gtsam::Pose3  T_W_left   = T_W_B * body_P_sensor;
          const gtsam::Point3 point_left = T_W_left.transformTo( optimized );
          if ( point_left.z() <= 0.0 )
          {
            behind = true;
            break;
          }
        }
        if ( behind )
        {
          to_drop.push_back( id );
        }
      }
      for ( const LandmarkId id : to_drop )
      {
        landmarks_W.erase( id );
        eraseLandmarkFromWindow( id );
        culled_ids_.insert( id );
        frame_culled.push_back( id );
        ++dropped;
      }
      return dropped;
    }

    // Returns mean-cull count this pass. Cheirality ids are appended to
    // frame_culled but do not count toward the reopt trigger.
    [[nodiscard]] std::uint32_t runCheiralityAndMeanCull(
        const gtsam::Values&               optimized,
        const gtsam::NonlinearFactorGraph& graph_for_scoring,
        std::vector<LandmarkId>&           frame_culled,
        std::uint32_t&                     outliers_culled_total,
        std::uint32_t&                     outliers_culled_unique_total,
        std::uint32_t&                     num_cheirality_inout )
    {
      const std::uint32_t cheirality_after = countCheiralityFactors(
          graph_for_scoring, optimized, calibration.fxPixels() );
      const std::uint32_t dropped =
          dropCheiralityLandmarks( optimized, frame_culled );
      num_cheirality_inout = std::max(
          num_cheirality_inout, std::max( cheirality_after, dropped ) );

      std::uint32_t culled_round = 0;
      if ( !options.enable_outlier_cull )
      {
        return culled_round;
      }

      std::vector<LandmarkId> candidate_ids;
      candidate_ids.reserve( landmarks_W.size() );
      for ( const auto& [ id, _unused ] : landmarks_W )
      {
        candidate_ids.push_back( id );
      }
      for ( const LandmarkId id : candidate_ids )
      {
        double      sum_norm  = 0.0;
        std::size_t n_factors = 0;
        for ( const auto& factor : graph_for_scoring )
        {
          if ( factor == nullptr )
          {
            continue;
          }
          const auto* stereo =
              dynamic_cast<const gtsam::GenericStereoFactor<gtsam::Pose3,
                                                            gtsam::Point3>*>(
                  factor.get() );
          if ( stereo == nullptr )
          {
            continue;
          }
          if ( stereo->key2() != L( id ) )
          {
            continue;
          }
          sum_norm += stereo->unwhitenedError( optimized ).norm();
          ++n_factors;
        }
        if ( n_factors < 4 )
        {
          continue;
        }
        const double score = sum_norm / static_cast<double>( n_factors );
        if ( score > options.outlier_avg_reproj_px )
        {
          landmarks_W.erase( id );
          eraseLandmarkFromWindow( id );
          frame_culled.push_back( id );
          ++culled_round;
          ++outliers_culled_total;
          if ( culled_ids_.insert( id ).second )
          {
            ++outliers_culled_unique_total;
          }
        }
      }
      return culled_round;
    }
  };

  StereoVoEstimator::StereoVoEstimator(
      camera::RectifiedStereoCalibration calibration,
      EstimatorOptions                   options )
      : m_impl( std::make_unique<Impl>( std::move( calibration ),
                                        std::move( options ) ) )
  {
  }

  StereoVoEstimator::~StereoVoEstimator() = default;

  StereoVoEstimator::StereoVoEstimator( StereoVoEstimator&& ) noexcept =
      default;

  StereoVoEstimator& StereoVoEstimator::operator=(
      StereoVoEstimator&& ) noexcept = default;

  std::vector<common::Timestamp> StereoVoEstimator::observationTimestamps(
      LandmarkId id ) const
  {
    const auto it = m_impl->track_times.find( id );
    if ( it == m_impl->track_times.end() )
    {
      return {};
    }
    return it->second;
  }

  VioUpdateResult StereoVoEstimator::update(
      const KeyframeMeasurement& measurement,
      const bool                 keyframe )
  {
    VioUpdateResult      result;
    GyroStateDiagnostics gyro_state;
    result.diagnostics.culled_landmark_ids.clear();
    result.diagnostics.num_observations =
        static_cast<std::uint32_t>( measurement.observations.size() );
    result.diagnostics.segment_id  = m_impl->segment_id;
    result.diagnostics.fusion_mode = m_impl->gyro_fusion_active
                                         ? FusionMode::kGyroVisual
                                         : FusionMode::kVisionOnly;
    result.diagnostics.gyro_alignment_residual_rms_rad =
        m_impl->gyro_fusion_active
            ? m_impl->gyro_alignment_residual_rms_rad
            : 0.0;
    // Rejected and accepted frames default to the last accepted graph state.
    result.diagnostics.bias_gyro = m_impl->last_reported_gyro_bias;

    // Static gyro audit 与纯视觉估计并行。pending 时仍运行完整视觉路径，
    // 因而 gyro factor 激活前的 status/pose/keyframe feedback 与 VO 一致；
    // timeout 仍 sticky kFailed，不把未 ready 冒充 VIO。
    if ( m_impl->options.enable_imu )
    {
      if ( m_impl->init_phase == Impl::InitPhase::kFailed )
      {
        result.status                          = UpdateStatus::kFailed;
        result.message                         = m_impl->init_failure;
        result.diagnostics.init_failed         = true;
        result.diagnostics.init_failure_reason = m_impl->init_failure;
        return result;
      }
      if ( m_impl->init_phase != Impl::InitPhase::kReady )
      {
        if ( m_impl->init_phase == Impl::InitPhase::kNone )
        {
          m_impl->init_phase    = Impl::InitPhase::kCollecting;
          m_impl->init_start_ts = measurement.timestamp;
        }
        if ( measurement.imu_gap )
        {
          m_impl->init_imu.clear();
          m_impl->init_start_ts = measurement.timestamp;
        }
        else if ( !measurement.imu_samples.empty() )
        {
          auto first = measurement.imu_samples.begin();
          if ( !m_impl->init_imu.empty() &&
               m_impl->init_imu.back().timestamp == first->timestamp )
          {
            ++first;
          }
          m_impl->init_imu.insert( m_impl->init_imu.end(), first,
                                   measurement.imu_samples.end() );
        }
        const double elapsed_s =
            static_cast<double>( measurement.timestamp.nanoseconds() -
                                 m_impl->init_start_ts.nanoseconds() ) *
            1e-9;
        if ( elapsed_s > m_impl->options.imu_init_timeout_s )
        {
          m_impl->init_phase = Impl::InitPhase::kFailed;
          m_impl->init_failure =
              "stationary init failed: no static window within " +
              std::to_string( m_impl->options.imu_init_timeout_s ) +
              " s (gyro/accel variance above thresholds)";
          result.status                          = UpdateStatus::kFailed;
          result.message                         = m_impl->init_failure;
          result.diagnostics.init_failed         = true;
          result.diagnostics.init_failure_reason = m_impl->init_failure;
          return result;
        }
        if ( m_impl->runStaticInitDetection() )
        {
          result.diagnostics.imu_init = m_impl->init_diagnostics;
          m_impl->init_phase          = Impl::InitPhase::kReady;
          m_impl->init_imu.clear();
          for ( WindowFrame& frame : m_impl->window )
          {
            frame.gyro_bias = m_impl->init_gyro_bias0;
          }
          m_impl->last_reported_gyro_bias = m_impl->init_gyro_bias0;
        }
        else
        {
          result.diagnostics.init_pending = true;
        }
      }

      // Every frame interval is also accounted independently of the audit.
      // kOk consumes pending at the end; rejected/failed visual updates retain
      // it for the next accepted pose pair.
      if ( m_impl->pending_imu.empty() )
      {
        m_impl->pending_from = measurement.t_prev;
      }
      if ( !measurement.imu_samples.empty() )
      {
        auto first = measurement.imu_samples.begin();
        if ( !m_impl->pending_imu.empty() &&
             m_impl->pending_imu.back().timestamp == first->timestamp )
        {
          ++first;  // 共享边界样本
        }
        m_impl->pending_imu.insert( m_impl->pending_imu.end(), first,
                                    measurement.imu_samples.end() );
      }
      m_impl->pending_gap = m_impl->pending_gap || measurement.imu_gap;
    }

    if ( measurement.observations.empty() )
    {
      // Gyro-only factors constrain rotation but not translation. Accepting an
      // empty frame would leave the new pose translation unobservable and
      // merely preserve its initial guess as a false-success trajectory point.
      result.status  = UpdateStatus::kRejected;
      result.message = "empty observations";
      return result;
    }
    // Slice ⑦: disparity_px == 0 is legal — it marks "stereo failed, no
    // depth"; negative or non-finite is not.
    for ( const StereoObservation& observation : measurement.observations )
    {
      if ( observation.disparity_px < 0.0 ||
           !observation.left_pixel.allFinite() ||
           !std::isfinite( observation.disparity_px ) )
      {
        result.status  = UpdateStatus::kRejected;
        result.message = "non-finite pixel or negative disparity";
        return result;
      }
    }
    if ( m_impl->initialized && !m_impl->window.empty() &&
         measurement.timestamp <= m_impl->window.back().timestamp )
    {
      result.status  = UpdateStatus::kRejected;
      result.message = "timestamp not strictly increasing";
      return result;
    }

    std::uint32_t num_shared    = 0;
    std::uint32_t num_disparity = 0;
    for ( const StereoObservation& observation : measurement.observations )
    {
      // E3 experiment: zero-disparity observations cannot constrain PnP or
      // BA, so exclude them from the shared-overlap accounting that gates PnP
      // and low-connectivity.
      if ( observation.disparity_px > 0.0 )
      {
        ++num_disparity;  // diag: frontend stereo matching health, independent
                          // of landmark-table overlap (num_shared below)
        if ( m_impl->landmarks_W.find( observation.id ) !=
             m_impl->landmarks_W.end() )
        {
          ++num_shared;
        }
      }
    }
    result.diagnostics.num_shared    = num_shared;
    result.diagnostics.num_disparity = num_disparity;

    // pre-M4 round 2: overlap recovered (normal tracking) — the
    // accumulated-seeding buffer is stale; drop it.
    if ( num_shared > 0 )
    {
      m_impl->pending_seed_obs.clear();
      // M4.2 (D12): overlap 恢复 → 重建窗口的种子累积标记重置, 下一次断开
      // 才重新清 landmark 表。
      m_impl->imu_window_rebuilt = false;
    }

    const bool overlap_broken = m_impl->initialized && num_shared == 0;

    if ( overlap_broken && !m_impl->options.enable_reanchor )
    {
      result.status  = UpdateStatus::kRejected;
      result.message = "zero shared landmarks with window";
      result.diagnostics.window_size =
          static_cast<std::uint32_t>( m_impl->window.size() );
      if ( !m_impl->window.empty() )
      {
        result.diagnostics.prior_key = m_impl->window.front().frame_index;
      }
      return result;
    }
    // Non-keyframe cannot re-anchor; reject when overlap is broken.
    if ( overlap_broken && !keyframe )
    {
      result.status  = UpdateStatus::kRejected;
      result.message = "zero shared landmarks (non-keyframe)";
      result.diagnostics.window_size =
          static_cast<std::uint32_t>( m_impl->window.size() );
      if ( !m_impl->window.empty() )
      {
        result.diagnostics.prior_key = m_impl->window.front().frame_index;
      }
      return result;
    }
    // Non-keyframe cannot initialise the first segment; reject.
    if ( !m_impl->initialized && !keyframe )
    {
      result.status  = UpdateStatus::kRejected;
      result.message = "not initialised (non-keyframe)";
      return result;
    }
    // Non-keyframe with too few shared landmarks cannot run PnP; gyro-only
    // does not constrain translation, so a raw CV guess would pollute the
    // pose chain (Slice ⑤b gate).
    if ( !keyframe &&
         static_cast<int>( num_shared ) < m_impl->options.min_pnp_inliers )
    {
      result.status  = UpdateStatus::kRejected;
      result.message = "insufficient shared landmarks (non-keyframe)";
      result.diagnostics.window_size =
          static_cast<std::uint32_t>( m_impl->window.size() );
      if ( !m_impl->window.empty() )
      {
        result.diagnostics.prior_key = m_impl->window.front().frame_index;
      }
      return result;
    }
    // pre-M4 round 2 残存: 首段跨帧累积播种。enable_accumulated_seed 时,
    // 未初始化帧的视差观测跨帧累积进 pending_seed_obs (最新覆盖), 累积到
    // min_seed_observations 个唯一 track 后用合成 measurement 播种 —— 只
    // 用于 Gate F (首段), V2_03 启动段暗帧 1-7 obs/帧饿死的突破手段。
    // Gate E (re-anchor) 保持 slice-7 原拒绝: 实测放宽 re-anchor 门槛使
    // V2_03 re-anchor 9 → 32-68、段错位 +2.739 → +3.9~+5.6m、ATE 3.628 →
    // 5.2-6.7 —— 门槛是质量门, 只放行足以滋养健康段的富帧。
    KeyframeMeasurement        accumulated_measurement;
    const KeyframeMeasurement* effective_measurement = &measurement;
    const auto                 accumulate            = [ & ]() -> bool {
      for ( const StereoObservation& obs : measurement.observations )
      {
        if ( obs.disparity_px > 0.0 )
        {
          m_impl->pending_seed_obs[ obs.id ] = obs;  // latest wins
        }
      }
      if ( static_cast<int>( m_impl->pending_seed_obs.size() ) <
           m_impl->options.min_seed_observations )
      {
        return false;
      }
      accumulated_measurement.timestamp = measurement.timestamp;
      accumulated_measurement.observations.reserve(
          m_impl->pending_seed_obs.size() );
      for ( const auto& [ id, obs ] : m_impl->pending_seed_obs )
      {
        (void)id;
        accumulated_measurement.observations.push_back( obs );
      }
      effective_measurement = &accumulated_measurement;
      return true;
    };

    if ( overlap_broken )
    {
      const int stereo_count =
          static_cast<int>( m_impl->countStereoObservations( measurement ) );
      if ( stereo_count < m_impl->options.min_seed_observations )
      {
        result.status  = UpdateStatus::kRejected;
        result.message = "insufficient observations to seed new segment";
        result.diagnostics.window_size =
            static_cast<std::uint32_t>( m_impl->window.size() );
        if ( !m_impl->window.empty() )
        {
          result.diagnostics.prior_key = m_impl->window.front().frame_index;
        }
        return result;
      }
      m_impl->pending_seed_obs.clear();
    }
    result.diagnostics.low_connectivity =
        m_impl->initialized &&
        num_shared > 0 &&
        static_cast<int>( num_shared ) < m_impl->options.min_shared_landmarks;

    if ( !m_impl->initialized )
    {
      const int stereo_count =
          static_cast<int>( m_impl->countStereoObservations( measurement ) );
      if ( stereo_count >= m_impl->options.min_seed_observations )
      {
        m_impl->pending_seed_obs.clear();
      }
      else if ( m_impl->options.enable_accumulated_seed )
      {
        if ( !accumulate() )
        {
          result.status  = UpdateStatus::kRejected;
          result.message = "accumulating seed observations (first segment)";
          return result;
        }
      }
      else
      {
        result.status  = UpdateStatus::kRejected;
        result.message = "insufficient observations to seed first segment";
        return result;
      }
    }

    // Snapshot for transactional rollback. Both keyframes and accepted
    // track-only frames enter the graph and therefore mutate the window.
    decltype( m_impl->window )              window_backup;
    decltype( m_impl->landmarks_W )         landmarks_backup;
    decltype( m_impl->track_times )         track_times_backup;
    decltype( m_impl->last_stereo_pose_W )  last_stereo_pose_backup;
    decltype( m_impl->last_accepted_T_W_B ) last_backup;
    decltype( m_impl->prev_accepted_T_W_B ) prev_backup;
    decltype( m_impl->next_frame_index )    next_index_backup;
    bool                                    initialized_backup = false;
    decltype( m_impl->segment_id )          segment_id_backup;
    decltype( m_impl->culled_ids_ )         culled_ids_backup;
    window_backup           = m_impl->window;
    landmarks_backup        = m_impl->landmarks_W;
    track_times_backup      = m_impl->track_times;
    last_stereo_pose_backup = m_impl->last_stereo_pose_W;
    last_backup             = m_impl->last_accepted_T_W_B;
    prev_backup             = m_impl->prev_accepted_T_W_B;
    next_index_backup       = m_impl->next_frame_index;
    initialized_backup      = m_impl->initialized;
    segment_id_backup       = m_impl->segment_id;
    culled_ids_backup       = m_impl->culled_ids_;

    std::vector<LandmarkId> frame_culled;

    auto restore = [ & ]() {
      m_impl->window             = window_backup;
      m_impl->landmarks_W        = landmarks_backup;
      m_impl->track_times        = track_times_backup;
      m_impl->last_stereo_pose_W = last_stereo_pose_backup;
      // D9: IMU-on 时位姿链 (last/prev accepted) 与 pending 段不回滚 ——
      // 失败帧的段已在更新入口挂入 pending, 下帧从最后接受位姿继续预积分。
      // (当前实现二者等价, 此处按设计稿固化契约。)
      if ( !m_impl->options.enable_imu )
      {
        m_impl->last_accepted_T_W_B = last_backup;
        m_impl->prev_accepted_T_W_B = prev_backup;
      }
      m_impl->next_frame_index      = next_index_backup;
      m_impl->initialized           = initialized_backup;
      m_impl->segment_id            = segment_id_backup;
      m_impl->culled_ids_           = culled_ids_backup;
      result.diagnostics.segment_id = m_impl->segment_id;
      result.diagnostics.culled_landmark_ids.clear();
      frame_culled.clear();
    };

    if ( !m_impl->initialized )
    {
      // The first visual seed keeps the same identity gauge as VO. Static
      // gravity orientation is audit-only and must not re-anchor production.
      const Eigen::Isometry3d anchor = Eigen::Isometry3d::Identity();
      gyro_state.graph_initial_T_W_B = anchor;
      if ( !m_impl->seedSegment( anchor, *effective_measurement,
                                 result.diagnostics.probe_rejected_block_n,
                                 result.diagnostics.probe_new_lm_n ) )
      {
        restore();
        result.status  = UpdateStatus::kRejected;
        result.message = "failed to backproject landmark on first frame";
        return result;
      }
      m_impl->pending_seed_obs.clear();

      WindowFrame& seed = m_impl->window.back();
      seed.gyro_bias    = m_impl->init_phase == Impl::InitPhase::kReady
                              ? m_impl->init_gyro_bias0
                              : Eigen::Vector3d::Zero();
    }
    else if ( overlap_broken )
    {
      Eigen::Isometry3d anchor = m_impl->poseInitialValue();
      if ( m_impl->options.enable_imu )
      {
        Eigen::Isometry3d predicted      = Eigen::Isometry3d::Identity();
        Eigen::Vector3d   predicted_bias = Eigen::Vector3d::Zero();
        gyro_state.prediction_valid      = m_impl->gyroPredict(
            m_impl->pending_imu, m_impl->pending_gap, predicted,
            predicted_bias );
        if ( gyro_state.prediction_valid )
        {
          anchor = predicted;
        }
        gyro_state.imu_sample_count = static_cast<std::uint32_t>(
            m_impl->pending_imu.size() );
        if ( !m_impl->pending_imu.empty() )
        {
          gyro_state.imu_t_i_ns =
              m_impl->pending_imu.front().timestamp.nanoseconds();
          gyro_state.imu_t_j_ns =
              m_impl->pending_imu.back().timestamp.nanoseconds();
          gyro_state.imu_dt_s =
              static_cast<double>( gyro_state.imu_t_j_ns -
                                   gyro_state.imu_t_i_ns ) *
              1e-9;
        }
        gyro_state.predicted_T_W_B      = predicted;
        gyro_state.prediction_bias_gyro = predicted_bias;
      }
      if ( !isFinite( anchor ) )
      {
        restore();
        result.status  = UpdateStatus::kFailed;
        result.message = "non-finite pose initial value";
        return result;
      }
      gyro_state.graph_initial_T_W_B = anchor;
      if ( !m_impl->seedSegment( anchor, *effective_measurement,
                                 result.diagnostics.probe_rejected_block_n,
                                 result.diagnostics.probe_new_lm_n ) )
      {
        restore();
        result.status  = UpdateStatus::kRejected;
        result.message = "failed to backproject landmark on re-anchor frame";
        return result;
      }
      m_impl->window.back().gyro_bias = m_impl->last_reported_gyro_bias;
      m_impl->pending_seed_obs.clear();
      m_impl->imu_window_rebuilt = false;
      ++m_impl->segment_id;
      result.diagnostics.segment_id = m_impl->segment_id;
    }
    else
    {
      WindowFrame candidate;
      candidate.frame_index  = m_impl->next_frame_index;
      candidate.timestamp    = measurement.timestamp;
      candidate.observations = measurement.observations;
      candidate.is_keyframe  = keyframe;  // Slice ⑤c
      // M4.2: IMU 段 = pending 拼接段 (含本帧段)。gap = pending 累积 gap
      // 或段不可积分 (样本 < 2)。
      candidate.imu_samples = m_impl->pending_imu;
      candidate.t_prev      = m_impl->pending_from;
      candidate.imu_gap     = m_impl->pending_gap ||
                          m_impl->pending_imu.size() < 2;

      Eigen::Isometry3d guess_T_W_B = m_impl->poseInitialValue();
      Eigen::Vector3d   guess_bias =
          m_impl->window.empty() ? Eigen::Vector3d::Zero()
                                   : m_impl->window.back().gyro_bias;
      // Keep normal-frame initial-value ownership visual: gyro prediction is
      // recorded for diagnostics and supplies the shared bias seed, while
      // CV/PnP chooses the pose basin. Re-anchor above may still use gyro when
      // the visual chain is broken.
      if ( m_impl->options.enable_imu )
      {
        Eigen::Isometry3d pred_T    = Eigen::Isometry3d::Identity();
        Eigen::Vector3d   pred_b    = Eigen::Vector3d::Zero();
        gyro_state.prediction_valid = m_impl->gyroPredict(
            m_impl->pending_imu, candidate.imu_gap, pred_T, pred_b );
        if ( gyro_state.prediction_valid )
        {
          guess_bias = pred_b;
        }
        gyro_state.imu_sample_count = static_cast<std::uint32_t>(
            m_impl->pending_imu.size() );
        if ( !m_impl->pending_imu.empty() )
        {
          gyro_state.imu_t_i_ns =
              m_impl->pending_imu.front().timestamp.nanoseconds();
          gyro_state.imu_t_j_ns =
              m_impl->pending_imu.back().timestamp.nanoseconds();
          gyro_state.imu_dt_s =
              static_cast<double>( gyro_state.imu_t_j_ns -
                                   gyro_state.imu_t_i_ns ) *
              1e-9;
        }
        gyro_state.predicted_T_W_B      = pred_T;
        gyro_state.prediction_bias_gyro = pred_b;
      }
      candidate.T_W_B     = guess_T_W_B;
      candidate.gyro_bias = guess_bias;

      // PnP owns the normal-frame pose initialization on every sufficiently
      // supported frame; gyro enters the optimized graph through AHRS factors.
      const bool pnp_allowed = m_impl->options.enable_pnp_init;
      if ( pnp_allowed &&
           static_cast<int>( num_shared ) >=
               m_impl->options.min_pnp_inliers )
      {
        const Impl::PnpInitResult pnp =
            m_impl->tryPnpInit( measurement, guess_T_W_B );
        if ( pnp.success )
        {
          candidate.T_W_B = pnp.T_W_B;

          // Map inlier indices → shared LandmarkIds (same scan order as
          // tryPnpInit: zero-disparity observations skipped there), then drop
          // shared outliers; keep new ids.
          std::vector<LandmarkId> shared_ids;
          shared_ids.reserve( static_cast<std::size_t>( num_shared ) );
          for ( const StereoObservation& observation :
                measurement.observations )
          {
            if ( observation.disparity_px > 0.0 &&
                 m_impl->landmarks_W.find( observation.id ) !=
                     m_impl->landmarks_W.end() )
            {
              shared_ids.push_back( observation.id );
            }
          }
          std::unordered_set<LandmarkId> inlier_ids;
          inlier_ids.reserve( pnp.inlier_indices.size() );
          for ( const int index : pnp.inlier_indices )
          {
            if ( index < 0 ||
                 static_cast<std::size_t>( index ) >= shared_ids.size() )
            {
              continue;
            }
            inlier_ids.insert(
                shared_ids[ static_cast<std::size_t>( index ) ] );
          }
          std::vector<StereoObservation> filtered;
          filtered.reserve( candidate.observations.size() );
          for ( const StereoObservation& observation :
                candidate.observations )
          {
            const bool is_shared =
                m_impl->landmarks_W.find( observation.id ) !=
                m_impl->landmarks_W.end();
            // Slice ⑦: zero-disparity observations never enter the PnP
            // inlier set, so never drop them here either — they stay in the
            // window for when stereo returns.
            if ( observation.disparity_px > 0.0 && is_shared &&
                 inlier_ids.find( observation.id ) == inlier_ids.end() )
            {
              continue;
            }
            filtered.push_back( observation );
          }
          candidate.observations = std::move( filtered );

          result.diagnostics.pnp_success = true;
          result.diagnostics.pnp_inliers =
              static_cast<std::uint32_t>( pnp.inlier_indices.size() );
        }
      }

      if ( !isFinite( candidate.T_W_B ) )
      {
        if ( keyframe ) restore();
        result.status  = UpdateStatus::kFailed;
        result.message = "non-finite pose initial value";
        return result;
      }
      gyro_state.graph_initial_T_W_B = candidate.T_W_B;
      // CRITICAL: track_times must see the full measurement (including
      // shared outliers masked out of candidate.observations).
      for ( const StereoObservation& observation : measurement.observations )
      {
        m_impl->track_times[ observation.id ].push_back(
            measurement.timestamp );
      }
      // Slice ⑦ (E12g, final: part of the E13-composed gate): a far-return
      // landmark — one whose last stereo observation left the window while
      // it hung on zero-disparity observations — carries a 3D frozen at
      // that old pose. E10/E11 showed this population is the sole carrier
      // of the Slice ⑦ effect, good (V2_02 -39%: the stale multi-frame
      // point beats unreliable single-frame SAD backprojects) or bad
      // (MH_03/V2_01 +36%/+15%-gain: the return must not be anchored to a
      // drifted point). E13's hang-distance gate (hanging_landmark_gate_m)
      // drops the far band where the damage concentrates; the near band
      // survives and is resolved here: keep the stale 3D while it still
      // projects to the observed left pixel; refresh to the current
      // backproject when the stale point is grossly off (drifted track) or
      // behind the camera. Threshold and on/off via
      // options.far_return_refresh_px (bench CLI --far-refresh-px;
      // <= 0 disables the refresh — pure-gate runs).
      for ( const StereoObservation& observation : candidate.observations )
      {
        if ( observation.disparity_px <= 0.0 )
        {
          continue;
        }
        auto landmark_it = m_impl->landmarks_W.find( observation.id );
        if ( landmark_it == m_impl->landmarks_W.end() )
        {
          continue;  // new id — seeded below
        }
        if ( m_impl->findLastStereoFrameInWindow( observation.id )
                 .has_value() )
        {
          continue;  // fresh return — untouched (zero-impact population)
        }
        if ( m_impl->options.far_return_refresh_px <= 0.0 )
        {
          continue;  // E13 pure: refresh disabled — gate-only runs
        }
        const gtsam::Pose3 T_W_left =
            toPose3( candidate.T_W_B ) * m_impl->body_P_sensor;
        const gtsam::Point3 point_W( landmark_it->second.x(),
                                     landmark_it->second.y(),
                                     landmark_it->second.z() );
        gtsam::StereoPoint2 projected;
        double              proj_error = -1.0;
        try
        {
          projected = gtsam::StereoCamera( T_W_left, m_impl->K ).project( point_W );
          proj_error =
              std::sqrt( std::pow( projected.uL() - observation.left_pixel.x(),
                                   2 ) +
                         std::pow( projected.v() - observation.left_pixel.y(),
                                   2 ) );
        }
        catch ( const gtsam::StereoCheiralityException& )
        {
          proj_error = -1.0;  // stale point behind the camera — refresh
        }
        const bool refresh =
            proj_error < 0.0 ||
            proj_error > m_impl->options.far_return_refresh_px;
        if ( !refresh )
        {
          continue;  // stale 3D still projects to the observed pixel — e9 path
        }
        const std::optional<Eigen::Vector3d> point_W_new =
            m_impl->backprojectWorld( candidate.T_W_B, observation );
        if ( !point_W_new.has_value() || !isFinite( *point_W_new ) )
        {
          continue;
        }
        landmark_it->second = *point_W_new;  // drifted track — refresh (ck path)
      }
      // New ids only: backproject from masked candidate.observations.
      // Slice ⑤c: only keyframes seed new landmarks; non-keyframes enter
      // the window/BA but do not grow the map.
      // Slice ⑥b: require the track to have >= 3 observations before
      // seeding — a single-frame disparity can be a SAD mismatch under
      // fast motion, producing a bad-depth anchor that drags the BA.
      if ( keyframe )
      {
        for ( const StereoObservation& observation : candidate.observations )
        {
          if ( m_impl->landmarks_W.find( observation.id ) !=
               m_impl->landmarks_W.end() )
          {
            continue;
          }
          if ( m_impl->options.block_culled_rebirth &&
               m_impl->culled_ids_.find( observation.id ) !=
                   m_impl->culled_ids_.end() )
          {
            ++result.diagnostics.probe_rejected_block_n;
            continue;
          }
          const auto        track_it    = m_impl->track_times.find( observation.id );
          const std::size_t seed_thresh = static_cast<std::size_t>(
              m_impl->options.min_track_observations_for_seed );
          if ( track_it == m_impl->track_times.end() ||
               track_it->second.size() < seed_thresh )
          {
            std::cerr << "[6b] skip seed id=" << observation.id
                      << " times="
                      << ( track_it == m_impl->track_times.end()
                               ? -1
                               : static_cast<int>( track_it->second.size() ) )
                      << " thresh=" << seed_thresh << "\n";
            continue;  // ⑥b: not yet stable enough to seed
          }
          // Slice ⑦: a zero-disparity observation has no stereo depth, and
          // multi-frame triangulation seeding is disabled (Slice ⑦ gate
          // outcome — see docs/benchmark/m3.3/slice-7_*.md). The landmark is
          // seeded later by a stereo (disparity > 0) observation instead.
          if ( observation.disparity_px <= 0.0 )
          {
            continue;
          }
          const std::optional<Eigen::Vector3d> point_W =
              m_impl->backprojectWorld( candidate.T_W_B, observation );
          if ( !point_W.has_value() )
          {
            continue;
          }
          const gtsam::Pose3 T_W_left =
              toPose3( candidate.T_W_B ) * m_impl->body_P_sensor;
          const gtsam::Point3 point_left = T_W_left.transformTo(
              gtsam::Point3( point_W->x(), point_W->y(), point_W->z() ) );
          if ( !isFinite( *point_W ) || point_left.z() <= 0.0 )
          {
            continue;
          }
          m_impl->landmarks_W[ observation.id ] = *point_W;
          ++result.diagnostics.probe_new_lm_n;
        }
      }  // keyframe-only landmark seeding

      m_impl->window.push_back( std::move( candidate ) );
      ++m_impl->next_frame_index;
    }

    // Slice ⑤c: Basalt-style eviction — cap keyframes at 7, then total at
    // window_size (10), preferring to evict the oldest non-keyframe so the
    // 3 most recent frames stay as temporal states.
    // M4.3: IMU-on 下驱逐只允许发生在最老帧 (pop_front) —— IMU 链每帧段
    // [t_prev, t_cur] 只在窗口帧连续时构成正确预积分 (buildGraph 用
    // cur.imu_samples 重建); 逐出中间帧 (关键帧上限的首关键帧 / 非关键帧
    // 优先) 会时段错配。M4.3 实测: 全关键帧合成链的关键帧上限逐出种子帧
    // → (virtual, f1) 对 dT 0.05 vs 0.55, LM 用错误速度妥协, 轨迹冻结。
    // 虚拟帧无观测在最老位, 自然先被逐出 (C3 重积分安全)。
    // IMU-off 保持原行为逐字节 (c1d3481 参考回归)。
    if ( m_impl->gyro_fusion_active )
    {
      std::size_t keyframe_count = 0;
      for ( const WindowFrame& frame : m_impl->window )
      {
        if ( frame.is_keyframe ) ++keyframe_count;
      }
      while ( static_cast<int>( m_impl->window.size() ) >
                  m_impl->options.window_size ||
              keyframe_count > 7U )
      {
        if ( m_impl->window.front().is_keyframe )
        {
          --keyframe_count;
        }
        m_impl->window.pop_front();
      }
    }
    else
    {
      std::size_t keyframe_count = 0;
      for ( const WindowFrame& frame : m_impl->window )
      {
        if ( frame.is_keyframe ) ++keyframe_count;
      }
      while ( keyframe_count > 7U )
      {
        for ( auto it = m_impl->window.begin(); it != m_impl->window.end();
              ++it )
        {
          if ( it->is_keyframe )
          {
            m_impl->window.erase( it );
            --keyframe_count;
            break;
          }
        }
      }
      while ( static_cast<int>( m_impl->window.size() ) >
              m_impl->options.window_size )
      {
        // Evict the oldest non-keyframe first; fall back to pop_front.
        bool evicted = false;
        for ( auto it = m_impl->window.begin(); it != m_impl->window.end();
              ++it )
        {
          if ( !it->is_keyframe &&
               it->frame_index != m_impl->window.back().frame_index )
          {
            m_impl->window.erase( it );
            evicted = true;
            break;
          }
        }
        if ( !evicted )
        {
          m_impl->window.pop_front();
        }
      }
    }
    m_impl->pruneLandmarksNotInWindow();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               values;
    std::uint64_t               prior_key     = 0;
    std::uint32_t               num_landmarks = 0;
    try
    {
      m_impl->buildGraph( graph, values, prior_key, num_landmarks );
    }
    catch ( const std::exception& exception )
    {
      restore();
      result.status  = UpdateStatus::kFailed;
      result.message = std::string( "graph build exception: " ) +
                       exception.what();
      return result;
    }
    result.diagnostics.gyro_factor_count = m_impl->gyroFactorCount();
    result.diagnostics.num_landmarks     = num_landmarks;
    result.diagnostics.prior_key         = prior_key;
    result.diagnostics.window_size =
        static_cast<std::uint32_t>( m_impl->window.size() );
    result.diagnostics.reproj_rms_before_px =
        stereoReprojRms( graph, values );
    const std::uint32_t cheirality_before = std::max(
        countCheiralityFactors(
            graph, values, m_impl->calibration.fxPixels() ),
        countBehindCameraLandmarks( m_impl->window, m_impl->landmarks_W,
                                    m_impl->body_P_sensor, values ) );
    std::unordered_map<std::uint64_t, Eigen::Vector3d> poses_before;
    for ( const WindowFrame& frame : m_impl->window )
    {
      if ( frame.frame_index != m_impl->window.back().frame_index )
      {
        poses_before.emplace( frame.frame_index, frame.T_W_B.translation() );
      }
    }

    gtsam::Values optimized;
    try
    {
      gtsam::LevenbergMarquardtOptimizer optimizer( graph, values );
      optimized = optimizer.optimize();
      result.diagnostics.lm_iterations =
          static_cast<std::uint32_t>( optimizer.iterations() );
    }
    catch ( const gtsam::IndeterminantLinearSystemException& exception )
    {
      restore();
      result.status  = UpdateStatus::kFailed;
      result.message = std::string( "indeterminant linear system: " ) +
                       exception.what();
      return result;
    }
    catch ( const std::exception& exception )
    {
      restore();
      result.status  = UpdateStatus::kFailed;
      result.message = std::string( "optimizer exception: " ) +
                       exception.what();
      return result;
    }

    for ( const WindowFrame& frame : m_impl->window )
    {
      if ( !optimized.exists( X( frame.frame_index ) ) )
      {
        restore();
        result.status  = UpdateStatus::kFailed;
        result.message = "optimized values missing a window pose";
        return result;
      }
      const Eigen::Isometry3d T_W_B =
          toIsometry( optimized.at<gtsam::Pose3>( X( frame.frame_index ) ) );
      if ( !isFinite( T_W_B ) )
      {
        restore();
        result.status  = UpdateStatus::kFailed;
        result.message = "non-finite optimized pose";
        return result;
      }
    }

    double                max_shift_m     = 0.0;
    constexpr std::size_t kProbeShiftTopK = 3;
    if ( m_impl->options.enable_probe_b )
    {
      result.diagnostics.probe_shift_top.reserve( kProbeShiftTopK );
    }
    for ( WindowFrame& frame : m_impl->window )
    {
      const Eigen::Isometry3d T_W_B =
          toIsometry( optimized.at<gtsam::Pose3>( X( frame.frame_index ) ) );
      const auto before_it = poses_before.find( frame.frame_index );
      if ( before_it != poses_before.end() )
      {
        const double dt_m =
            ( T_W_B.translation() - before_it->second ).norm();
        max_shift_m = std::max( max_shift_m, dt_m );
        if ( m_impl->options.enable_probe_b )
        {
          considerShiftTopK( result.diagnostics.probe_shift_top,
                             frame.frame_index, dt_m, kProbeShiftTopK );
        }
      }
      frame.T_W_B = T_W_B;
    }
    if ( m_impl->gyro_fusion_active )
    {
      if ( !optimized.exists( W( 0 ) ) )
      {
        restore();
        result.status  = UpdateStatus::kFailed;
        result.message = "optimized values missing shared gyro bias";
        return result;
      }
      const Eigen::Vector3d bias_gyr =
          optimized.at<gtsam::Vector3>( W( 0 ) );
      if ( !bias_gyr.allFinite() )
      {
        restore();
        result.status  = UpdateStatus::kFailed;
        result.message = "non-finite optimized shared gyro bias";
        return result;
      }
      for ( WindowFrame& frame : m_impl->window )
      {
        frame.gyro_bias = bias_gyr;
      }
    }

    // E13: record the BA-refined pose of every stereo observation of the new
    // frame, so the hang-distance gate keeps measuring after this frame
    // leaves the window. (Older frames' entries were written when each was
    // the back frame, with the pose refined up to that point — later BA
    // shifts are sub-cm, negligible against meter-scale gates.)
    {
      const Eigen::Isometry3d& T_W_B = m_impl->window.back().T_W_B;
      for ( const StereoObservation& observation :
            m_impl->window.back().observations )
      {
        if ( observation.disparity_px > 0.0 )
        {
          m_impl->last_stereo_pose_W[ observation.id ] = T_W_B;
        }
      }
    }
    if ( m_impl->options.enable_probe_b )
    {
      std::sort( result.diagnostics.probe_shift_top.begin(),
                 result.diagnostics.probe_shift_top.end(),
                 []( const std::pair<std::uint64_t, double>& a,
                     const std::pair<std::uint64_t, double>& b ) {
                   return a.second > b.second;
                 } );
    }

    for ( auto& [ id, point_W ] : m_impl->landmarks_W )
    {
      const gtsam::Key key = L( id );
      if ( !optimized.exists( key ) )
      {
        continue;
      }
      const gtsam::Point3 point = optimized.at<gtsam::Point3>( key );
      point_W                   = Eigen::Vector3d( point.x(), point.y(), point.z() );
      if ( !isFinite( point_W ) )
      {
        restore();
        result.status  = UpdateStatus::kFailed;
        result.message = "non-finite optimized landmark";
        return result;
      }
    }

    if ( m_impl->options.enable_vio_state_probe &&
         m_impl->gyro_fusion_active )
    {
      result.diagnostics.gyro_graph_cost =
          summarizeGyroGraphCosts( graph, values, optimized );
    }

    result.diagnostics.num_cheirality = cheirality_before;

    std::uint32_t outliers_culled        = 0;
    std::uint32_t outliers_culled_unique = 0;
    std::uint32_t culled_round           = m_impl->runCheiralityAndMeanCull(
        optimized, graph, frame_culled, outliers_culled, outliers_culled_unique,
        result.diagnostics.num_cheirality );

    // Full-graph RMS on LM₁ graph/values (includes just-culled ids) =
    // pre-cull quality; contract unchanged by multi-round reopt.
    result.diagnostics.reproj_rms_after_px =
        stereoReprojRms( graph, optimized );
    // Slice ④ after_cull initial value (final when reopt is skipped / fails).
    double after_cull = result.diagnostics.reproj_rms_after_px;
    if ( m_impl->options.enable_outlier_cull )
    {
      after_cull = stereoReprojRmsSkippingMissingLandmarks(
          graph, optimized, m_impl->landmarks_W );
    }

    std::uint32_t rounds               = 0;
    bool          outlier_reopt_failed = false;
    while ( m_impl->options.enable_outlier_reopt &&
            rounds <
                static_cast<std::uint32_t>( m_impl->options.max_outlier_reopts ) &&
            culled_round >= 4U )
    {
      // Snapshot: failed round rolls back to pre-round window/landmarks.
      const auto window_snap    = m_impl->window;
      const auto landmarks_snap = m_impl->landmarks_W;

      gtsam::NonlinearFactorGraph g_r;
      gtsam::Values               v_r;
      std::uint64_t               prior_key_r = 0;
      std::uint32_t               n_lm_r      = 0;
      try
      {
        m_impl->buildGraph( g_r, v_r, prior_key_r, n_lm_r );
        (void)prior_key_r;
        (void)n_lm_r;
        gtsam::LevenbergMarquardtOptimizer opt( g_r, v_r );
        const gtsam::Values                optimized_r = opt.optimize();

        for ( const WindowFrame& frame : m_impl->window )
        {
          if ( !optimized_r.exists( X( frame.frame_index ) ) )
          {
            throw std::runtime_error( "optimized values missing a window pose" );
          }
          const Eigen::Isometry3d T_W_B = toIsometry(
              optimized_r.at<gtsam::Pose3>( X( frame.frame_index ) ) );
          if ( !isFinite( T_W_B ) )
          {
            throw std::runtime_error( "non-finite optimized pose" );
          }
        }

        for ( WindowFrame& frame : m_impl->window )
        {
          frame.T_W_B = toIsometry(
              optimized_r.at<gtsam::Pose3>( X( frame.frame_index ) ) );
        }
        if ( m_impl->gyro_fusion_active )
        {
          if ( !optimized_r.exists( W( 0 ) ) )
          {
            throw std::runtime_error(
                "optimized values missing shared gyro bias" );
          }
          const Eigen::Vector3d bias_gyr =
              optimized_r.at<gtsam::Vector3>( W( 0 ) );
          if ( !bias_gyr.allFinite() )
          {
            throw std::runtime_error(
                "non-finite optimized shared gyro bias" );
          }
          for ( WindowFrame& frame : m_impl->window )
          {
            frame.gyro_bias = bias_gyr;
          }
        }

        for ( auto& [ id, point_W ] : m_impl->landmarks_W )
        {
          const gtsam::Key key = L( id );
          if ( !optimized_r.exists( key ) )
          {
            continue;
          }
          const gtsam::Point3 point = optimized_r.at<gtsam::Point3>( key );
          point_W                   = Eigen::Vector3d( point.x(), point.y(), point.z() );
          if ( !isFinite( point_W ) )
          {
            throw std::runtime_error( "non-finite optimized landmark" );
          }
        }

        result.diagnostics.lm_iterations +=
            static_cast<std::uint32_t>( opt.iterations() );
        ++rounds;
        after_cull   = stereoReprojRms( g_r, optimized_r );
        culled_round = m_impl->runCheiralityAndMeanCull(
            optimized_r, g_r, frame_culled, outliers_culled,
            outliers_culled_unique, result.diagnostics.num_cheirality );
        if ( m_impl->options.enable_vio_state_probe &&
             m_impl->gyro_fusion_active )
        {
          result.diagnostics.gyro_graph_cost =
              summarizeGyroGraphCosts( g_r, v_r, optimized_r );
        }
      }
      catch ( const gtsam::IndeterminantLinearSystemException& )
      {
        m_impl->window       = window_snap;
        m_impl->landmarks_W  = landmarks_snap;
        outlier_reopt_failed = true;
        break;
      }
      catch ( const std::exception& )
      {
        m_impl->window       = window_snap;
        m_impl->landmarks_W  = landmarks_snap;
        outlier_reopt_failed = true;
        break;
      }
    }

    result.diagnostics.outliers_culled          = outliers_culled;
    result.diagnostics.outliers_culled_unique   = outliers_culled_unique;
    result.diagnostics.outlier_reopt_rounds     = rounds;
    result.diagnostics.outlier_reopt            = ( rounds > 0U );
    result.diagnostics.outlier_reopt_failed     = outlier_reopt_failed;
    result.diagnostics.reproj_rms_after_cull_px = after_cull;
    result.diagnostics.max_window_pose_shift_m  = max_shift_m;
    result.diagnostics.culled_landmark_ids      = std::move( frame_culled );

    if ( m_impl->options.enable_probe_b )
    {
      // Final window/landmarks snapshot (same stage as reproj_rms_after_*).
      gtsam::NonlinearFactorGraph probe_graph;
      gtsam::Values               probe_values;
      std::uint64_t               probe_prior_key = 0;
      std::uint32_t               probe_num_lm    = 0;
      m_impl->buildGraph( probe_graph, probe_values, probe_prior_key,
                          probe_num_lm );
      (void)probe_prior_key;
      (void)probe_num_lm;
      fillProbeLandmarkResiduals( probe_graph, probe_values,
                                  result.diagnostics );
    }

    if ( m_impl->options.enable_fixed_lag_shadow )
    {
      FixedLagShadowDiagnostics shadow_diagnostics;
      if ( result.diagnostics.fusion_mode == FusionMode::kGyroVisual )
      {
        try
        {
          shadow_diagnostics = m_impl->updateFixedLagShadow();
        }
        catch ( const std::exception& exception )
        {
          restore();
          result.status = UpdateStatus::kFailed;
          result.message =
              std::string( "fixed-lag shadow exception: " ) +
              exception.what();
          return result;
        }
      }
      result.diagnostics.fixed_lag_shadow = shadow_diagnostics;
    }

    // M4.4 staged activation: use only poses produced by a pure visual graph
    // as calibration evidence.  A re-anchor has no cross-segment visual
    // rotation constraint, and a gap has no valid gyro integral, so neither
    // contributes.  Failure is explicit and transactional; the current IMU
    // interval remains pending for the next call.
    if ( m_impl->options.enable_imu && !m_impl->gyro_fusion_active &&
         initialized_backup && !overlap_broken )
    {
      const std::size_t evidence_size_before =
          m_impl->gyro_bias_evidence.size();
      const double       support_before = m_impl->gyro_align_support_s;
      const WindowFrame& current        = m_impl->window.back();
      if ( !current.imu_gap && current.imu_samples.size() >= 2 &&
           m_impl->last_accepted_T_W_B.has_value() )
      {
        const double duration_s =
            static_cast<double>(
                current.imu_samples.back().timestamp.nanoseconds() -
                current.imu_samples.front().timestamp.nanoseconds() ) *
            1e-9;
        if ( std::isfinite( duration_s ) && duration_s > 0.0 )
        {
          m_impl->gyro_bias_evidence.push_back( GyroBiasEvidence{
              gtsam::Rot3( m_impl->last_accepted_T_W_B->linear() ),
              gtsam::Rot3( current.T_W_B.linear() ), current.imu_samples,
              duration_s } );
          m_impl->gyro_align_support_s += duration_s;
        }
      }

      if ( m_impl->init_phase == Impl::InitPhase::kReady &&
           m_impl->gyro_align_support_s + 1e-12 >=
               m_impl->options.imu_gyro_align_window_s )
      {
        Eigen::Vector3d aligned_bias     = Eigen::Vector3d::Zero();
        double          residual_rms_rad = 0.0;
        std::string     alignment_error;
        if ( !m_impl->estimateGyroBias(
                 aligned_bias, residual_rms_rad, alignment_error ) )
        {
          m_impl->gyro_bias_evidence.resize( evidence_size_before );
          m_impl->gyro_align_support_s = support_before;
          restore();
          result.status  = UpdateStatus::kFailed;
          result.message = alignment_error;
          return result;
        }
        m_impl->gyro_bias_prior                 = aligned_bias;
        m_impl->gyro_alignment_residual_rms_rad = residual_rms_rad;
        m_impl->gyro_fusion_active              = true;
        m_impl->retainContiguousGyroSuffix();
        for ( WindowFrame& frame : m_impl->window )
        {
          frame.gyro_bias = aligned_bias;
        }
        m_impl->gyro_bias_evidence.clear();
        m_impl->gyro_align_support_s = 0.0;
      }
    }

    m_impl->initialized         = true;
    m_impl->prev_accepted_T_W_B = m_impl->last_accepted_T_W_B;
    m_impl->last_accepted_T_W_B = m_impl->window.back().T_W_B;
    // M4.2 (D9): 段已入窗 → 消费 pending (被拒帧的段此前保留, 下帧续积)。
    m_impl->pending_imu.clear();
    m_impl->pending_gap  = false;
    m_impl->pending_from = common::Timestamp{ 0 };

    // Accepted IMU-on frames publish the shared gyro bias. Rejected frames
    // keep the last accepted value; IMU-off remains zero.
    if ( m_impl->options.enable_imu )
    {
      m_impl->last_reported_gyro_bias = m_impl->window.back().gyro_bias;
      result.diagnostics.bias_acc.setZero();
      gyro_state.state_key          = m_impl->window.back().frame_index;
      result.diagnostics.gyro_state = gyro_state;
    }
    result.diagnostics.bias_gyro = m_impl->last_reported_gyro_bias;

    result.status   = UpdateStatus::kOk;
    result.estimate = VioEstimate{ measurement.timestamp,
                                   m_impl->window.back().T_W_B };
    return result;
  }

}  // namespace phad::estimator
