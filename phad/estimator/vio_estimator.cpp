#include "phad/estimator/vio_estimator.hpp"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include "phad/estimator/internal/imu_interval.hpp"
#include "phad/estimator/internal/initial_moving_root_math.hpp"
#include "phad/estimator/internal/initial_visual_evidence.hpp"
#include "phad/estimator/internal/initialization_diagnostics.hpp"
#include "phad/estimator/internal/vio_update_transaction.hpp"
#ifdef PHAD_ESTIMATOR_TESTING
#include "phad/estimator/internal/vio_estimator_test_access.hpp"
#endif

namespace phad::estimator
{
  namespace
  {

    // GTSAM 4.3 exports uppercase Symbol helpers (X/L); older docs used x/l.
    using gtsam::symbol_shorthand::B;
    using gtsam::symbol_shorthand::L;
    using gtsam::symbol_shorthand::V;
    using gtsam::symbol_shorthand::X;
    using internal::classifyInitialVisualFactorFinal;
    using internal::classifyInitialVisualFactorMask;
    using internal::ImuIntervalError;
    using internal::ImuIntervalResult;
    using internal::ImuIntervalStep;
    using internal::InitialMovingCandidateFailure;
    using internal::InitialMovingCandidateResult;
    using internal::InitialMovingCandidateSolution;
    using internal::InitialMovingMathError;
    using internal::InitialMovingMathInput;
    using internal::InitialMovingPimConfig;
    using internal::InitialMovingPose;
    using internal::InitialVisualFactorFinalDecision;
    using internal::InitialVisualFactorMaskDecision;
    using internal::initialVisualUnderlyingGaussianQ;
    using internal::kInitialVisualFactorChiSquareLimit;
    using internal::NavigationState;
    using internal::NormalizedImuInterval;
    using internal::normalizeRawImuInterval;
    using internal::solveInitialMovingCandidate;
    using internal::spliceNormalizedImuIntervals;
    using internal::VerifiedVisualPose;
    using internal::VioUpdateTransaction;
    using internal::VisualPoseSeed;
    using internal::WindowFrame;

    using StereoVisualFactor =
        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;
    using MonoVisualFactor = gtsam::GenericProjectionFactor<
        gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;

    enum class VisualFactorModality
    {
      kStereo,
      kMono,
    };

    struct VisualFactorView
    {
      const gtsam::NoiseModelFactor* m_factor;
      gtsam::Key                     m_pose_key;
      gtsam::Key                     m_landmark_key;
      VisualFactorModality           m_modality;

      [[nodiscard]] gtsam::Vector unwhitenedError(
          const gtsam::Values& values ) const
      {
        return m_factor->unwhitenedError( values );
      }
    };

    [[nodiscard]] std::optional<VisualFactorView> asVisualFactor(
        const gtsam::NonlinearFactor::shared_ptr& factor )
    {
      if ( factor == nullptr )
      {
        return std::nullopt;
      }
      if ( const auto* stereo =
               dynamic_cast<const StereoVisualFactor*>( factor.get() ) )
      {
        return VisualFactorView{ stereo, stereo->key1(), stereo->key2(),
                                 VisualFactorModality::kStereo };
      }
      if ( const auto* mono =
               dynamic_cast<const MonoVisualFactor*>( factor.get() ) )
      {
        return VisualFactorView{ mono, mono->key1(), mono->key2(),
                                 VisualFactorModality::kMono };
      }
      return std::nullopt;
    }

    struct VioGraphInfo
    {
      std::uint32_t               m_nav_states                  = 0;
      std::uint32_t               m_imu_factors                 = 0;
      std::uint32_t               m_bias_rw_factors             = 0;
      std::uint32_t               m_visual_factors              = 0;
      std::uint32_t               m_current_visual_factors      = 0;
      std::uint32_t               m_current_mono_visual_factors = 0;
      std::uint32_t               m_root_prior_sets             = 0;
      std::uint32_t               m_integration_steps           = 0;
      std::int64_t                m_integrated_duration_ns      = 0;
      Eigen::Matrix<double, 6, 1> m_last_bias_rw_sigmas =
          Eigen::Matrix<double, 6, 1>::Zero();
    };

    enum class GraphSolveErrorCode
    {
      kIndeterminant,
      kOptimizer,
    };

    struct GraphSolveError
    {
      GraphSolveErrorCode m_code;
      std::string         m_detail;
      std::uint32_t       m_lm_iterations = 0;
    };

    struct GraphSolveSuccess
    {
      gtsam::Values m_values;
      std::uint32_t m_lm_iterations = 0;
    };

    using GraphSolveResult =
        std::variant<GraphSolveSuccess, GraphSolveError>;

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

    [[nodiscard]] gtsam::Point2 toMonoPoint(
        const StereoObservation& observation )
    {
      return gtsam::Point2( observation.left_pixel.x(),
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

    [[nodiscard]] const NavigationState& requireNavigationState(
        const WindowFrame& frame, const char* context )
    {
      if ( !frame.m_navigation.has_value() )
      {
        throw std::runtime_error( std::string( context ) +
                                  " requires a committed navigation state" );
      }
      return *frame.m_navigation;
    }

    [[nodiscard]] NavigationState& requireNavigationState(
        WindowFrame& frame, const char* context )
    {
      if ( !frame.m_navigation.has_value() )
      {
        throw std::runtime_error( std::string( context ) +
                                  " requires a committed navigation state" );
      }
      return *frame.m_navigation;
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

    [[nodiscard]] gtsam::SharedNoiseModel makeMonoNoise(
        const EstimatorOptions& options )
    {
      const auto gaussian =
          gtsam::noiseModel::Isotropic::Sigma( 2, options.stereo_sigma_px );
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

    [[nodiscard]] gtsam::SharedNoiseModel makeVelocityPriorNoise(
        const EstimatorOptions& options )
    {
      if ( !std::isfinite( options.m_velocity_prior_sigma_mps ) ||
           options.m_velocity_prior_sigma_mps <= 0.0 )
      {
        throw std::invalid_argument(
            "EstimatorOptions.m_velocity_prior_sigma_mps must be finite and > 0" );
      }
      return gtsam::noiseModel::Isotropic::Sigma(
          3, options.m_velocity_prior_sigma_mps );
    }

    [[nodiscard]] gtsam::SharedNoiseModel makeBiasPriorNoise(
        const EstimatorOptions& options )
    {
      if ( !std::isfinite( options.m_acc_bias_prior_sigma_mps2 ) ||
           options.m_acc_bias_prior_sigma_mps2 <= 0.0 ||
           !std::isfinite( options.m_gyr_bias_prior_sigma_radps ) ||
           options.m_gyr_bias_prior_sigma_radps <= 0.0 )
      {
        throw std::invalid_argument(
            "EstimatorOptions bias prior sigmas must be finite and > 0" );
      }
      gtsam::Vector6 sigmas;
      sigmas << options.m_acc_bias_prior_sigma_mps2,
          options.m_acc_bias_prior_sigma_mps2,
          options.m_acc_bias_prior_sigma_mps2,
          options.m_gyr_bias_prior_sigma_radps,
          options.m_gyr_bias_prior_sigma_radps,
          options.m_gyr_bias_prior_sigma_radps;
      return gtsam::noiseModel::Diagonal::Sigmas( sigmas );
    }

    [[nodiscard]] std::shared_ptr<gtsam::PreintegrationParams>
    makePreintegrationParams( const sensor::ImuParameters& imu,
                              const EstimatorOptions&      options )
    {
      if ( !std::isfinite( options.m_gravity_mps2 ) ||
           options.m_gravity_mps2 <= 0.0 ||
           !std::isfinite( options.m_q_int ) || options.m_q_int < 0.0 )
      {
        throw std::invalid_argument(
            "EstimatorOptions gravity/q_int must be finite with gravity > 0 and q_int >= 0" );
      }
      auto params =
          gtsam::PreintegrationParams::MakeSharedU( options.m_gravity_mps2 );
      params->setAccelerometerCovariance(
          imu.accNd() * imu.accNd() * Eigen::Matrix3d::Identity() );
      params->setGyroscopeCovariance(
          imu.gyrNd() * imu.gyrNd() * Eigen::Matrix3d::Identity() );
      params->setIntegrationCovariance(
          options.m_q_int * Eigen::Matrix3d::Identity() );
      return params;
    }

    [[nodiscard]] gtsam::imuBias::ConstantBias toGtsamBias(
        const ImuBias& bias )
    {
      return gtsam::imuBias::ConstantBias( bias.m_acc_mps2,
                                           bias.m_gyr_radps );
    }

    [[nodiscard]] ImuBias toImuBias(
        const gtsam::imuBias::ConstantBias& bias )
    {
      return ImuBias{ .m_acc_mps2  = bias.accelerometer(),
                      .m_gyr_radps = bias.gyroscope() };
    }

    [[nodiscard]] Eigen::Vector3d sampleAcc(
        const sensor::ImuMeasurement& sample )
    {
      return { sample.accel_mps2[ 0 ], sample.accel_mps2[ 1 ],
               sample.accel_mps2[ 2 ] };
    }

    [[nodiscard]] Eigen::Vector3d sampleGyr(
        const sensor::ImuMeasurement& sample )
    {
      return { sample.gyro_radps[ 0 ], sample.gyro_radps[ 1 ],
               sample.gyro_radps[ 2 ] };
    }

    [[nodiscard]] bool sameImuValue(
        const sensor::ImuMeasurement& lhs,
        const sensor::ImuMeasurement& rhs )
    {
      return lhs.accel_mps2 == rhs.accel_mps2 &&
             lhs.gyro_radps == rhs.gyro_radps;
    }

    [[nodiscard]] std::optional<std::int64_t> checkedPositiveDurationNs(
        common::Timestamp begin, common::Timestamp end )
    {
      const std::int64_t begin_ns = begin.nanoseconds();
      const std::int64_t end_ns   = end.nanoseconds();
      if ( begin_ns >= end_ns ||
           ( begin_ns < 0 &&
             end_ns > std::numeric_limits<std::int64_t>::max() + begin_ns ) )
      {
        return std::nullopt;
      }
      return end_ns - begin_ns;
    }

    struct BootstrapStats
    {
      Eigen::Vector3d m_acc_mean = Eigen::Vector3d::Zero();
      Eigen::Vector3d m_gyr_mean = Eigen::Vector3d::Zero();
      Eigen::Vector3d m_acc_std  = Eigen::Vector3d::Zero();
      Eigen::Vector3d m_gyr_std  = Eigen::Vector3d::Zero();
    };

    [[nodiscard]] BootstrapStats bootstrapStats(
        const std::vector<sensor::ImuMeasurement>& samples,
        std::size_t                                begin = 0U )
    {
      BootstrapStats stats;
      for ( std::size_t index = begin; index < samples.size(); ++index )
      {
        const sensor::ImuMeasurement& sample = samples[ index ];
        stats.m_acc_mean += sampleAcc( sample );
        stats.m_gyr_mean += sampleGyr( sample );
      }
      const double count = static_cast<double>( samples.size() - begin );
      stats.m_acc_mean /= count;
      stats.m_gyr_mean /= count;
      for ( std::size_t index = begin; index < samples.size(); ++index )
      {
        const sensor::ImuMeasurement& sample = samples[ index ];
        stats.m_acc_std +=
            ( sampleAcc( sample ) - stats.m_acc_mean ).array().square().matrix();
        stats.m_gyr_std +=
            ( sampleGyr( sample ) - stats.m_gyr_mean ).array().square().matrix();
      }
      stats.m_acc_std = ( stats.m_acc_std / count ).array().sqrt().matrix();
      stats.m_gyr_std = ( stats.m_gyr_std / count ).array().sqrt().matrix();
      return stats;
    }

    [[nodiscard]] Eigen::Matrix3d minimalRotationToWorldUp(
        const Eigen::Vector3d& acc_mean )
    {
      const Eigen::Vector3d source             = acc_mean.normalized();
      const Eigen::Vector3d target             = Eigen::Vector3d::UnitZ();
      const double          dot                = std::clamp( source.dot( target ), -1.0, 1.0 );
      constexpr double      kParallelTolerance = 1e-12;
      if ( dot >= 1.0 - kParallelTolerance )
      {
        return Eigen::Matrix3d::Identity();
      }
      if ( dot <= -1.0 + kParallelTolerance )
      {
        return Eigen::AngleAxisd( std::numbers::pi,
                                  Eigen::Vector3d::UnitX() )
            .toRotationMatrix();
      }
      const Eigen::Vector3d axis = source.cross( target ).normalized();
      return Eigen::AngleAxisd( std::acos( dot ), axis ).toRotationMatrix();
    }

    [[nodiscard]] Eigen::Isometry3d makeMovingWorldAlignment(
        const Eigen::Vector3d& gravity_V, const double gravity_mps2 )
    {
      constexpr double kRelativeTolerance = 1e-9;
      const double     scale              = std::max( 1.0, gravity_mps2 );
      if ( !gravity_V.allFinite() || !std::isfinite( gravity_mps2 ) ||
           !( gravity_mps2 > 0.0 ) || !( gravity_V.norm() > 0.0 ) ||
           std::abs( gravity_V.norm() - gravity_mps2 ) >
               kRelativeTolerance * scale )
      {
        throw std::runtime_error(
            "moving candidate gravity violates materialization contract" );
      }

      const Eigen::Matrix3d R_W_V =
          minimalRotationToWorldUp( -gravity_V );
      const Eigen::Matrix3d should_be_identity =
          R_W_V.transpose() * R_W_V;
      if ( !R_W_V.allFinite() ||
           !should_be_identity.isApprox( Eigen::Matrix3d::Identity(),
                                         kRelativeTolerance ) ||
           std::abs( R_W_V.determinant() - 1.0 ) > kRelativeTolerance )
      {
        throw std::runtime_error(
            "moving world alignment is not a finite proper rotation" );
      }
      const Eigen::Vector3d gravity_W( 0.0, 0.0, -gravity_mps2 );
      if ( !( R_W_V * gravity_V )
                .isApprox( gravity_W, kRelativeTolerance * scale ) )
      {
        throw std::runtime_error(
            "moving world alignment does not produce canonical gravity" );
      }

      Eigen::Isometry3d T_W_V = Eigen::Isometry3d::Identity();
      T_W_V.linear()          = R_W_V;
      return T_W_V;
    }

    [[nodiscard]] double visualReprojRms(
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values&               values )
    {
      double      sum_sq = 0.0;
      std::size_t count  = 0;
      for ( const auto& factor : graph )
      {
        const std::optional<VisualFactorView> visual =
            asVisualFactor( factor );
        if ( !visual.has_value() )
        {
          continue;
        }
        const gtsam::Vector error = visual->unwhitenedError( values );
        sum_sq += error.squaredNorm();
        ++count;
      }
      if ( count == 0 )
      {
        return 0.0;
      }
      return std::sqrt( sum_sq / static_cast<double>( count ) );
    }

    [[nodiscard]] std::optional<double> probeGraphError(
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values&               values ) noexcept
    {
      try
      {
        const double value = graph.error( values );
        return std::isfinite( value ) ? std::optional<double>{ value }
                                      : std::nullopt;
      }
      catch ( const std::exception& )
      {
        return std::nullopt;
      }
    }

    [[nodiscard]] std::optional<double> probeImuFactorError(
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values&               values ) noexcept
    {
      double      sum   = 0.0;
      std::size_t count = 0U;
      try
      {
        for ( const auto& factor : graph )
        {
          if ( factor == nullptr ||
               dynamic_cast<const gtsam::ImuFactor*>( factor.get() ) ==
                   nullptr )
          {
            continue;
          }
          sum += factor->error( values );
          ++count;
        }
      }
      catch ( const std::exception& )
      {
        return std::nullopt;
      }
      if ( count == 0U || !std::isfinite( sum ) )
      {
        return std::nullopt;
      }
      return sum;
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

    // Per-landmark mean visual residual (unwhitened L2), then mean/max/max_id.
    void fillProbeLandmarkResiduals( const gtsam::NonlinearFactorGraph& graph,
                                     const gtsam::Values&               values,
                                     UpdateDiagnostics&                 diagnostics )
    {
      std::unordered_map<LandmarkId, std::pair<double, std::size_t>> per_lm;
      for ( const auto& factor : graph )
      {
        const std::optional<VisualFactorView> visual =
            asVisualFactor( factor );
        if ( !visual.has_value() )
        {
          continue;
        }
        const LandmarkId id =
            static_cast<LandmarkId>(
                gtsam::Symbol( visual->m_landmark_key ).index() );
        auto& entry = per_lm[ id ];
        entry.first += visual->unwhitenedError( values ).norm();
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

    // Same RMS as visualReprojRms but skips factors whose landmark is no longer
    // in landmarks_W (cheirality / mean-reproj cull). Does not rebuild the graph.
    [[nodiscard]] double visualReprojRmsSkippingMissingLandmarks(
        const gtsam::NonlinearFactorGraph&                     graph,
        const gtsam::Values&                                   values,
        const std::unordered_map<LandmarkId, Eigen::Vector3d>& landmarks_W )
    {
      double      sum_sq = 0.0;
      std::size_t count  = 0;
      for ( const auto& factor : graph )
      {
        const std::optional<VisualFactorView> visual =
            asVisualFactor( factor );
        if ( !visual.has_value() )
        {
          continue;
        }
        const gtsam::Key lkey = visual->m_landmark_key;
        const LandmarkId id =
            static_cast<LandmarkId>( gtsam::Symbol( lkey ).index() );
        if ( landmarks_W.find( id ) == landmarks_W.end() )
        {
          continue;
        }
        const gtsam::Vector error = visual->unwhitenedError( values );
        sum_sq += error.squaredNorm();
        ++count;
      }
      if ( count == 0 )
      {
        return 0.0;
      }
      return std::sqrt( sum_sq / static_cast<double>( count ) );
    }

    // Both visual factor types return 2*fx on each residual axis when the point
    // is behind the camera (throwCheirality=false).
    [[nodiscard]] std::uint32_t countCheiralityFactors(
        const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
        double fx_pixels )
    {
      const double  sentinel = 2.0 * fx_pixels;
      std::uint32_t count    = 0;
      for ( const auto& factor : graph )
      {
        const std::optional<VisualFactorView> visual =
            asVisualFactor( factor );
        if ( !visual.has_value() )
        {
          continue;
        }
        const gtsam::Vector error = visual->unwhitenedError( values );
        const Eigen::Index  expected_size =
            visual->m_modality == VisualFactorModality::kStereo ? 3 : 2;
        if ( error.size() == expected_size &&
             ( error.array() - sentinel ).abs().maxCoeff() < 1e-6 )
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
          for ( const StereoObservation& observation : frame.m_observations )
          {
            if ( observation.id == id )
            {
              observes = true;
              break;
            }
          }
          if ( !observes || !values.exists( X( frame.m_frame_index ) ) )
          {
            continue;
          }
          const gtsam::Pose3 T_W_left =
              values.at<gtsam::Pose3>( X( frame.m_frame_index ) ) *
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

  struct VioEstimator::Impl
  {
    camera::RectifiedStereoCalibration           calibration;
    sensor::ImuParameters                        imu;
    EstimatorOptions                             options;
    gtsam::Cal3_S2Stereo::shared_ptr             K;
    gtsam::Cal3_S2::shared_ptr                   K_mono;
    gtsam::Pose3                                 body_P_sensor;
    gtsam::SharedNoiseModel                      stereo_noise;
    gtsam::SharedNoiseModel                      mono_noise;
    gtsam::SharedNoiseModel                      prior_noise;
    gtsam::SharedNoiseModel                      velocity_prior_noise;
    gtsam::SharedNoiseModel                      bias_prior_noise;
    std::shared_ptr<gtsam::PreintegrationParams> pim_params;
    std::unique_ptr<internal::VioUpdateState>    m_state;
    std::uint64_t                                m_next_attempt_id = 1U;
#ifdef PHAD_ESTIMATOR_TESTING
    std::optional<InitialMovingCandidateResult>
        m_test_next_moving_candidate_result;
    mutable std::optional<InitializationReason>
                          m_test_next_current_graph_result;
    mutable bool          m_test_next_current_graph_contract_error = false;
    std::optional<double> m_test_next_initial_visual_depth;
    bool                  m_test_next_initial_visual_whitening_error   = false;
    std::size_t           m_test_last_initial_visual_solve_window_size = 0U;
    mutable std::optional<PnpProbeResult>
        m_test_next_pnp_proposal_failure;
    mutable std::optional<PnpGuessRetryOutcome>
        m_test_next_pnp_guess_retry_outcome;
#endif

    explicit Impl( camera::RectifiedStereoCalibration calibration_in,
                   sensor::ImuParameters              imu_in,
                   EstimatorOptions                   options_in )
        : calibration( std::move( calibration_in ) ),
          imu( std::move( imu_in ) ),
          options( std::move( options_in ) ),
          K( std::make_shared<gtsam::Cal3_S2Stereo>(
              calibration.fxPixels(), calibration.fyPixels(), 0.0,
              calibration.cxPixels(), calibration.cyPixels(),
              calibration.baselineM() ) ),
          K_mono( std::make_shared<gtsam::Cal3_S2>( K->calibration() ) ),
          body_P_sensor(
              toPose3( toIsometry( calibration.T_B_left_rectified() ) ) ),
          stereo_noise( makeStereoNoise( options ) ),
          mono_noise( makeMonoNoise( options ) ),
          prior_noise( makePriorNoise( options ) ),
          velocity_prior_noise( makeVelocityPriorNoise( options ) ),
          bias_prior_noise( makeBiasPriorNoise( options ) ),
          pim_params( makePreintegrationParams( imu, options ) ),
          m_state( std::make_unique<internal::VioUpdateState>() )
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
      if ( options.m_bootstrap_min_duration_ns <= 0 ||
           options.m_bootstrap_min_samples < 2U ||
           !std::isfinite( options.m_bootstrap_max_acc_std_mps2 ) ||
           options.m_bootstrap_max_acc_std_mps2 < 0.0 ||
           !std::isfinite( options.m_bootstrap_max_gyr_std_radps ) ||
           options.m_bootstrap_max_gyr_std_radps < 0.0 ||
           !std::isfinite( options.m_bootstrap_acc_norm_tol_mps2 ) ||
           options.m_bootstrap_acc_norm_tol_mps2 < 0.0 ||
           options.m_bootstrap_timeout_ns <
               options.m_bootstrap_min_duration_ns ||
           options.m_visual_coast_horizon_ns <= 0 )
      {
        throw std::invalid_argument(
            "EstimatorOptions bootstrap/coast values are invalid" );
      }
      m_state->m_vio_diagnostics =
          makeVioDiagnostics( VioGraphInfo{}, *m_state );
    }

    [[nodiscard]] std::uint32_t completeActiveSegment(
        const common::Timestamp next_continuity_anchor )
    {
      const std::uint32_t completed_segment = m_state->m_segment_id;
      m_state->m_window.clear();
      m_state->m_landmarks_w.clear();
      m_state->m_track_times.clear();
      m_state->m_T_W_B_last_stereo.clear();
      m_state->m_culled_ids.clear();
      m_state->m_bootstrap_nodes.clear();
      m_state->m_static_candidate_ended           = false;
      m_state->m_evidence_revision                = 0U;
      m_state->m_last_attempted_evidence_revision = 0U;
      m_state->m_continuity_anchor                = next_continuity_anchor;
      m_state->m_visual_coast_duration_ns         = 0;
      m_state->m_initialized                      = false;
      m_state->m_vio_diagnostics =
          makeVioDiagnostics( VioGraphInfo{}, *m_state );
      ++m_state->m_segment_id;
      return completed_segment;
    }

    void clearProvisionalMovingLifecycleForStaticRoot()
    {
      m_state->m_window.clear();
      m_state->m_landmarks_w.clear();
      m_state->m_track_times.clear();
      m_state->m_T_W_B_last_stereo.clear();
      m_state->m_culled_ids.clear();
      m_state->m_static_candidate_ended           = false;
      m_state->m_evidence_revision                = 0U;
      m_state->m_last_attempted_evidence_revision = 0U;
      m_state->m_non_keyframe_evictions           = 0U;
      m_state->m_imu_reintegrations               = 0U;
      m_state->m_vio_diagnostics =
          makeVioDiagnostics( VioGraphInfo{}, *m_state );
    }

    static void eraseLandmarkFromWindow(
        internal::VioUpdateState& graph_state, LandmarkId id )
    {
      for ( WindowFrame& frame : graph_state.m_window )
      {
        auto& obs = frame.m_observations;
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
    bool seedRoot( const Eigen::Isometry3d& anchor_T_W_B,
                   const Eigen::Vector3d&   v_W_B,
                   const ImuBias&           bias,
                   const VioMeasurement&    measurement,
                   std::uint32_t&           probe_rejected_block_n,
                   std::uint32_t&           probe_new_lm_n )
    {
      m_state->m_landmarks_w.clear();

      WindowFrame candidate;
      candidate.m_frame_index  = m_state->m_next_frame_index;
      candidate.m_timestamp    = measurement.m_timestamp;
      candidate.m_observations = measurement.m_observations;
      candidate.m_is_keyframe  = true;
      candidate.m_navigation   = NavigationState{
            .m_T_W_B = anchor_T_W_B, .m_v_W_B = v_W_B, .m_bias = bias };
      const NavigationState& navigation = *candidate.m_navigation;

      for ( const StereoObservation& observation : measurement.m_observations )
      {
        if ( options.block_culled_rebirth &&
             m_state->m_culled_ids.find( observation.id ) != m_state->m_culled_ids.end() )
        {
          ++probe_rejected_block_n;
          continue;
        }
        // A root has no window history, so a zero-disparity
        // observation cannot be backprojected (infinite depth); it is
        // seeded on a later keyframe once the window is rebuilt.
        if ( observation.disparity_px <= 0.0 )
        {
          continue;
        }
        const Eigen::Vector3d point_W =
            backprojectWorld( navigation.m_T_W_B, observation );
        const gtsam::Pose3 T_W_left =
            toPose3( navigation.m_T_W_B ) * body_P_sensor;
        const gtsam::Point3 point_left = T_W_left.transformTo( gtsam::Point3(
            point_W.x(), point_W.y(), point_W.z() ) );
        if ( !isFinite( point_W ) || point_left.z() <= 0.0 )
        {
          return false;
        }
        m_state->m_landmarks_w[ observation.id ] = point_W;
        m_state->m_track_times[ observation.id ].push_back(
            measurement.m_timestamp );
        ++probe_new_lm_n;
      }

      if ( m_state->m_landmarks_w.empty() )
      {
        return false;
      }

      m_state->m_window.clear();
      m_state->m_window.push_back( std::move( candidate ) );
      ++m_state->m_next_frame_index;
      return true;
    }

    struct PnpInitResult
    {
      bool                           success             = false;
      Eigen::Isometry3d              T_W_B               = Eigen::Isometry3d::Identity();
      std::uint32_t                  num_correspondences = 0;
      std::uint32_t                  num_inliers         = 0;
      std::unordered_set<LandmarkId> inlier_ids;
      PnpProbeResult                 probe_result = PnpProbeResult::kRansacFailed;
      std::optional<double>          proposal_rms_px;
      std::optional<double>          guess_rms_px;
      std::optional<std::uint32_t>   proposal_inlier_count;
      std::optional<std::uint32_t>   proposal_cheiral_inlier_count;
      std::optional<double>          proposal_min_inlier_depth_m;
      std::optional<LandmarkId>      proposal_min_inlier_landmark_id;
      std::optional<double>          proposal_guess_translation_delta_m;
      std::optional<double>          proposal_guess_rotation_delta_rad;
      PnpGuessRetryOutcome           guess_retry_outcome =
          PnpGuessRetryOutcome::kNotAttempted;
      std::optional<double>        guess_retry_proposal_rms_px;
      std::optional<double>        guess_retry_guess_rms_px;
      std::optional<std::uint32_t> guess_retry_cheiral_inlier_count;
      std::optional<double>        guess_retry_min_inlier_depth_m;
      std::optional<double>        guess_retry_translation_delta_m;
      std::optional<double>        guess_retry_rotation_delta_rad;
    };

    struct PnpInlierGeometrySummary
    {
      std::uint32_t m_inlier_count         = 0U;
      std::uint32_t m_cheiral_inlier_count = 0U;
      double        m_min_inlier_depth_m   = 0.0;
      LandmarkId    m_min_inlier_landmark_id{};
    };

    [[nodiscard]] std::optional<PnpInlierGeometrySummary>
    summarizePnpInlierGeometry(
        const Eigen::Isometry3d&                     T_W_B,
        const std::vector<const StereoObservation*>& mapped_observations,
        const std::vector<int>&                      inlier_indices ) const
    {
      if ( !isFinite( T_W_B ) || inlier_indices.empty() )
      {
        return std::nullopt;
      }

      const Eigen::Isometry3d T_W_left =
          T_W_B * toIsometry( body_P_sensor );
      const Eigen::Isometry3d  T_left_W = T_W_left.inverse();
      PnpInlierGeometrySummary summary;
      double                   min_depth = std::numeric_limits<double>::infinity();
      for ( const int index : inlier_indices )
      {
        if ( index < 0 ||
             static_cast<std::size_t>( index ) >= mapped_observations.size() )
        {
          return std::nullopt;
        }
        const StereoObservation* observation =
            mapped_observations[ static_cast<std::size_t>( index ) ];
        if ( observation == nullptr )
        {
          return std::nullopt;
        }
        const auto landmark_it = m_state->m_landmarks_w.find( observation->id );
        if ( landmark_it == m_state->m_landmarks_w.end() ||
             !isFinite( landmark_it->second ) )
        {
          return std::nullopt;
        }
        const Eigen::Vector3d point_left = T_left_W * landmark_it->second;
        if ( !point_left.allFinite() )
        {
          return std::nullopt;
        }
        const double depth_m = point_left.z();
        ++summary.m_inlier_count;
        if ( depth_m <= 0.0 )
        {
          ++summary.m_cheiral_inlier_count;
        }
        if ( depth_m < min_depth )
        {
          min_depth                        = depth_m;
          summary.m_min_inlier_depth_m     = depth_m;
          summary.m_min_inlier_landmark_id = observation->id;
        }
      }
      return summary;
    }

    struct PoseRmsResult
    {
      std::optional<double> m_rms_px;
      PnpProbeResult        m_failure = PnpProbeResult::kProposalRmsNonFinite;
    };

    [[nodiscard]] PoseRmsResult poseRmsAtPose(
        const Eigen::Isometry3d&                     T_W_B,
        const std::vector<const StereoObservation*>& mapped_observations,
        const std::vector<int>&                      inlier_indices ) const
    {
      if ( !isFinite( T_W_B ) )
      {
        return { .m_rms_px  = std::nullopt,
                 .m_failure = PnpProbeResult::kProposalPoseNonFinite };
      }
      if ( inlier_indices.empty() )
      {
        return { .m_rms_px = std::nullopt,
                 .m_failure =
                     PnpProbeResult::kProposalRmsInvalidCorrespondence };
      }

      const gtsam::Pose3        T_W_left = toPose3( T_W_B ) * body_P_sensor;
      const gtsam::StereoCamera camera( T_W_left, K );
      double                    sum_sq = 0.0;
      for ( const int index : inlier_indices )
      {
        if ( index < 0 ||
             static_cast<std::size_t>( index ) >= mapped_observations.size() )
        {
          return { .m_rms_px = std::nullopt,
                   .m_failure =
                       PnpProbeResult::kProposalRmsInvalidCorrespondence };
        }
        const StereoObservation* observation =
            mapped_observations[ static_cast<std::size_t>( index ) ];
        if ( observation == nullptr ||
             !observation->left_pixel.allFinite() ||
             !std::isfinite( observation->disparity_px ) )
        {
          return { .m_rms_px = std::nullopt,
                   .m_failure =
                       PnpProbeResult::kProposalRmsInvalidCorrespondence };
        }
        const auto landmark_it = m_state->m_landmarks_w.find( observation->id );
        if ( landmark_it == m_state->m_landmarks_w.end() ||
             !isFinite( landmark_it->second ) )
        {
          return { .m_rms_px = std::nullopt,
                   .m_failure =
                       PnpProbeResult::kProposalRmsInvalidCorrespondence };
        }

        const gtsam::Point3 point_W( landmark_it->second.x(),
                                     landmark_it->second.y(),
                                     landmark_it->second.z() );
        const gtsam::Point3 point_left = T_W_left.transformTo( point_W );
        if ( !point_left.allFinite() )
        {
          return { .m_rms_px  = std::nullopt,
                   .m_failure = PnpProbeResult::kProposalRmsNonFinite };
        }
        if ( point_left.z() <= 0.0 )
        {
          return { .m_rms_px = std::nullopt,
                   .m_failure =
                       PnpProbeResult::kProposalRmsCheirality };
        }

        double residual_sq = 0.0;
        if ( observation->disparity_px > 0.0 )
        {
          gtsam::StereoPoint2 projected;
          try
          {
            projected = camera.project( point_W );
          }
          catch ( const gtsam::StereoCheiralityException& )
          {
            return { .m_rms_px = std::nullopt,
                     .m_failure =
                         PnpProbeResult::kProposalRmsCheirality };
          }
          const gtsam::Vector residual =
              ( projected - toStereoPoint( *observation ) ).vector();
          if ( !residual.allFinite() )
          {
            return { .m_rms_px = std::nullopt,
                     .m_failure =
                         PnpProbeResult::kProposalRmsNonFinite };
          }
          residual_sq = residual.squaredNorm();
        }
        else
        {
          const double u = calibration.fxPixels() * point_left.x() /
                               point_left.z() +
                           calibration.cxPixels();
          const double v = calibration.fyPixels() * point_left.y() /
                               point_left.z() +
                           calibration.cyPixels();
          const Eigen::Vector2d residual =
              Eigen::Vector2d( u, v ) - observation->left_pixel;
          if ( !residual.allFinite() )
          {
            return { .m_rms_px = std::nullopt,
                     .m_failure =
                         PnpProbeResult::kProposalRmsNonFinite };
          }
          residual_sq = residual.squaredNorm();
        }
        if ( !std::isfinite( residual_sq ) )
        {
          return { .m_rms_px = std::nullopt,
                   .m_failure =
                       PnpProbeResult::kProposalRmsNonFinite };
        }
        sum_sq += residual_sq;
        if ( !std::isfinite( sum_sq ) )
        {
          return { .m_rms_px = std::nullopt,
                   .m_failure =
                       PnpProbeResult::kProposalRmsNonFinite };
        }
      }
      const double rms = std::sqrt(
          sum_sq / static_cast<double>( inlier_indices.size() ) );
      if ( !std::isfinite( rms ) )
      {
        return { .m_rms_px  = std::nullopt,
                 .m_failure = PnpProbeResult::kProposalRmsNonFinite };
      }
      return { .m_rms_px  = rms,
               .m_failure = PnpProbeResult::kProposalRmsNonFinite };
    }

    struct PnpGuessRetryResult
    {
      PnpGuessRetryOutcome             m_outcome = PnpGuessRetryOutcome::kNotAttempted;
      std::optional<double>            m_proposal_rms_px;
      std::optional<double>            m_guess_rms_px;
      std::optional<std::uint32_t>     m_cheiral_inlier_count;
      std::optional<double>            m_min_inlier_depth_m;
      std::optional<double>            m_translation_delta_m;
      std::optional<double>            m_rotation_delta_rad;
      std::optional<Eigen::Isometry3d> m_T_W_B;
    };

    [[nodiscard]] PnpGuessRetryResult runGuessConsistentPnpRetry(
        const std::vector<cv::Point3d>&              pts3d,
        const std::vector<cv::Point2d>&              pts2d,
        const std::vector<const StereoObservation*>& mapped_observations,
        const std::vector<int>&                      inlier_indices,
        const cv::Mat&                               camera_matrix,
        const cv::Mat&                               guess_rvec,
        const cv::Mat&                               guess_tvec,
        const Eigen::Isometry3d&                     T_B_left,
        const Eigen::Isometry3d&                     guess_T_W_B ) const
    {
      PnpGuessRetryResult retry;
#ifdef PHAD_ESTIMATOR_TESTING
      if ( m_test_next_pnp_guess_retry_outcome.has_value() )
      {
        retry.m_outcome = *m_test_next_pnp_guess_retry_outcome;
        m_test_next_pnp_guess_retry_outcome.reset();
        return retry;
      }
#endif
      std::vector<cv::Point3d> retry_pts3d;
      std::vector<cv::Point2d> retry_pts2d;
      retry_pts3d.reserve( inlier_indices.size() );
      retry_pts2d.reserve( inlier_indices.size() );
      for ( const int index : inlier_indices )
      {
        if ( index < 0 || static_cast<std::size_t>( index ) >= pts3d.size() ||
             static_cast<std::size_t>( index ) >= pts2d.size() )
        {
          retry.m_outcome =
              PnpGuessRetryOutcome::kProposalRmsInvalidCorrespondence;
          return retry;
        }
        retry_pts3d.push_back( pts3d[ static_cast<std::size_t>( index ) ] );
        retry_pts2d.push_back( pts2d[ static_cast<std::size_t>( index ) ] );
      }

      cv::Mat retry_rvec = guess_rvec.clone();
      cv::Mat retry_tvec = guess_tvec.clone();
      bool    solved     = false;
      try
      {
        solved = cv::solvePnP(
            retry_pts3d, retry_pts2d, camera_matrix, cv::noArray(),
            retry_rvec, retry_tvec, /*useExtrinsicGuess=*/true,
            cv::SOLVEPNP_ITERATIVE );
      }
      catch ( const cv::Exception& )
      {
        retry.m_outcome = PnpGuessRetryOutcome::kSolveFailed;
        return retry;
      }
      if ( !solved )
      {
        retry.m_outcome = PnpGuessRetryOutcome::kSolveFailed;
        return retry;
      }

      cv::Mat R_left_W;
      try
      {
        cv::Rodrigues( retry_rvec, R_left_W );
      }
      catch ( const cv::Exception& )
      {
        retry.m_outcome = PnpGuessRetryOutcome::kSolveFailed;
        return retry;
      }
      Eigen::Matrix3d retry_rotation;
      Eigen::Vector3d retry_translation;
      for ( int row = 0; row < 3; ++row )
      {
        retry_translation( row ) = retry_tvec.at<double>( row, 0 );
        for ( int col = 0; col < 3; ++col )
        {
          retry_rotation( row, col ) = R_left_W.at<double>( row, col );
        }
      }
      Eigen::Quaterniond retry_quat( retry_rotation );
      retry_quat.normalize();
      Eigen::Isometry3d retry_T_left_W = Eigen::Isometry3d::Identity();
      retry_T_left_W.linear()          = retry_quat.toRotationMatrix();
      retry_T_left_W.translation()     = retry_translation;
      const Eigen::Isometry3d retry_T_W_B =
          retry_T_left_W.inverse() * T_B_left.inverse();
      if ( !isFinite( retry_T_W_B ) )
      {
        retry.m_outcome = PnpGuessRetryOutcome::kProposalPoseNonFinite;
        return retry;
      }

      const auto geometry = summarizePnpInlierGeometry(
          retry_T_W_B, mapped_observations, inlier_indices );
      if ( geometry.has_value() )
      {
        retry.m_cheiral_inlier_count =
            geometry->m_cheiral_inlier_count;
        retry.m_min_inlier_depth_m = geometry->m_min_inlier_depth_m;
      }
      if ( isFinite( guess_T_W_B ) )
      {
        const Eigen::Isometry3d T_guess_retry =
            guess_T_W_B.inverse() * retry_T_W_B;
        const double translation_delta_m =
            T_guess_retry.translation().norm();
        const double rotation_delta_rad =
            Eigen::AngleAxisd( T_guess_retry.linear() ).angle();
        if ( std::isfinite( translation_delta_m ) &&
             std::isfinite( rotation_delta_rad ) )
        {
          retry.m_translation_delta_m = translation_delta_m;
          retry.m_rotation_delta_rad  = rotation_delta_rad;
        }
      }

      const PoseRmsResult proposal_score = poseRmsAtPose(
          retry_T_W_B, mapped_observations, inlier_indices );
      retry.m_proposal_rms_px = proposal_score.m_rms_px;
      if ( !proposal_score.m_rms_px.has_value() )
      {
        switch ( proposal_score.m_failure )
        {
          case PnpProbeResult::kProposalPoseNonFinite:
            retry.m_outcome =
                PnpGuessRetryOutcome::kProposalPoseNonFinite;
            break;
          case PnpProbeResult::kProposalRmsInvalidCorrespondence:
            retry.m_outcome =
                PnpGuessRetryOutcome::kProposalRmsInvalidCorrespondence;
            break;
          case PnpProbeResult::kProposalRmsCheirality:
            retry.m_outcome =
                PnpGuessRetryOutcome::kProposalRmsCheirality;
            break;
          case PnpProbeResult::kProposalRmsNonFinite:
            retry.m_outcome =
                PnpGuessRetryOutcome::kProposalRmsNonFinite;
            break;
          default:
            throw std::logic_error(
                "PnP retry score returned a non-failure result" );
        }
        return retry;
      }
      const PoseRmsResult guess_score = poseRmsAtPose(
          guess_T_W_B, mapped_observations, inlier_indices );
      retry.m_guess_rms_px = guess_score.m_rms_px;
      if ( guess_score.m_rms_px.has_value() &&
           *proposal_score.m_rms_px >
               *guess_score.m_rms_px + options.stereo_sigma_px )
      {
        retry.m_outcome = PnpGuessRetryOutcome::kProposalRmsRejected;
        return retry;
      }
      retry.m_outcome =
          PnpGuessRetryOutcome::kWouldPassExistingPnpGates;
      retry.m_T_W_B = retry_T_W_B;
      return retry;
    }

    // Mapped landmarks ∩ current left observations → solvePnPRansac.
    // A finite, sufficiently supported proposal is accepted only when its
    // modality-aware RMS on the PnP inlier set is no more than one
    // observation sigma worse than the existing guess.
    [[nodiscard]] PnpInitResult tryPnpInit(
        const VioMeasurement&    measurement,
        const Eigen::Isometry3d& guess_T_W_B ) const
    {
      PnpInitResult result;

      std::vector<cv::Point3d>              pts3d;
      std::vector<cv::Point2d>              pts2d;
      std::vector<const StereoObservation*> mapped_observations;
      pts3d.reserve( measurement.m_observations.size() );
      pts2d.reserve( measurement.m_observations.size() );
      mapped_observations.reserve( measurement.m_observations.size() );
      for ( const StereoObservation& observation : measurement.m_observations )
      {
        const auto landmark_it = m_state->m_landmarks_w.find( observation.id );
        if ( landmark_it == m_state->m_landmarks_w.end() )
        {
          continue;
        }
        const Eigen::Vector3d& point_W = landmark_it->second;
        pts3d.emplace_back( point_W.x(), point_W.y(), point_W.z() );
        pts2d.emplace_back( observation.left_pixel.x(),
                            observation.left_pixel.y() );
        mapped_observations.push_back( &observation );
      }
      result.num_correspondences =
          static_cast<std::uint32_t>( pts3d.size() );
      if ( static_cast<int>( pts3d.size() ) < options.min_pnp_inliers )
      {
        result.probe_result = PnpProbeResult::kInsufficientInliers;
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
      cv::Mat original_guess_rvec;
      cv::Mat original_guess_tvec;
      if ( options.enable_probe_b || options.enable_pnp_cheirality_retry )
      {
        original_guess_rvec = rvec.clone();
        original_guess_tvec = tvec.clone();
      }

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
        result.num_inliers =
            static_cast<std::uint32_t>( inlier_indices.size() );
        result.probe_result = PnpProbeResult::kInsufficientInliers;
        return result;
      }
      result.num_inliers =
          static_cast<std::uint32_t>( inlier_indices.size() );

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
        result.probe_result = PnpProbeResult::kProposalPoseNonFinite;
        return result;
      }

      const auto retryAfterCheirality = [ & ]() {
        const PnpGuessRetryResult retry = runGuessConsistentPnpRetry(
            pts3d, pts2d, mapped_observations, inlier_indices, K,
            original_guess_rvec, original_guess_tvec, T_B_left,
            guess_T_W_B );
        if ( options.enable_probe_b )
        {
          result.guess_retry_outcome         = retry.m_outcome;
          result.guess_retry_proposal_rms_px = retry.m_proposal_rms_px;
          result.guess_retry_guess_rms_px    = retry.m_guess_rms_px;
          result.guess_retry_cheiral_inlier_count =
              retry.m_cheiral_inlier_count;
          result.guess_retry_min_inlier_depth_m =
              retry.m_min_inlier_depth_m;
          result.guess_retry_translation_delta_m =
              retry.m_translation_delta_m;
          result.guess_retry_rotation_delta_rad =
              retry.m_rotation_delta_rad;
        }
        if ( !options.enable_pnp_cheirality_retry ||
             retry.m_outcome !=
                 PnpGuessRetryOutcome::kWouldPassExistingPnpGates ||
             !retry.m_T_W_B.has_value() )
        {
          return false;
        }

        std::unordered_set<LandmarkId> retry_inlier_ids;
        retry_inlier_ids.reserve( inlier_indices.size() );
        for ( const int index : inlier_indices )
        {
          if ( index < 0 ||
               static_cast<std::size_t>( index ) >=
                   mapped_observations.size() ||
               mapped_observations[ static_cast<std::size_t>( index ) ] ==
                   nullptr )
          {
            if ( options.enable_probe_b )
            {
              result.guess_retry_outcome =
                  PnpGuessRetryOutcome::kProposalRmsInvalidCorrespondence;
            }
            return false;
          }
          retry_inlier_ids.insert(
              mapped_observations[ static_cast<std::size_t>( index ) ]->id );
        }

        result.success         = true;
        result.T_W_B           = *retry.m_T_W_B;
        result.inlier_ids      = std::move( retry_inlier_ids );
        result.proposal_rms_px = retry.m_proposal_rms_px;
        result.guess_rms_px    = retry.m_guess_rms_px;
        result.probe_result    = PnpProbeResult::kAcceptedAfterGuessRetry;
        return true;
      };

#ifdef PHAD_ESTIMATOR_TESTING
      if ( m_test_next_pnp_proposal_failure.has_value() )
      {
        result.probe_result = *m_test_next_pnp_proposal_failure;
        m_test_next_pnp_proposal_failure.reset();
        if ( ( options.enable_probe_b ||
               options.enable_pnp_cheirality_retry ) &&
             result.probe_result ==
                 PnpProbeResult::kProposalRmsCheirality )
        {
          static_cast<void>( retryAfterCheirality() );
        }
        return result;
      }
#endif

      if ( options.enable_probe_b )
      {
        const auto geometry = summarizePnpInlierGeometry(
            T_W_B, mapped_observations, inlier_indices );
        if ( geometry.has_value() )
        {
          result.proposal_inlier_count = geometry->m_inlier_count;
          result.proposal_cheiral_inlier_count =
              geometry->m_cheiral_inlier_count;
          result.proposal_min_inlier_depth_m =
              geometry->m_min_inlier_depth_m;
          result.proposal_min_inlier_landmark_id =
              geometry->m_min_inlier_landmark_id;
        }
        if ( isFinite( guess_T_W_B ) )
        {
          const Eigen::Isometry3d T_guess_proposal =
              guess_T_W_B.inverse() * T_W_B;
          const double translation_delta_m =
              T_guess_proposal.translation().norm();
          const double rotation_delta_rad =
              Eigen::AngleAxisd( T_guess_proposal.linear() ).angle();
          if ( std::isfinite( translation_delta_m ) &&
               std::isfinite( rotation_delta_rad ) )
          {
            result.proposal_guess_translation_delta_m =
                translation_delta_m;
            result.proposal_guess_rotation_delta_rad = rotation_delta_rad;
          }
        }
      }

      const PoseRmsResult proposal_score = poseRmsAtPose(
          T_W_B, mapped_observations, inlier_indices );
      result.proposal_rms_px = proposal_score.m_rms_px;
      if ( !proposal_score.m_rms_px.has_value() )
      {
        result.probe_result = proposal_score.m_failure;
        if ( ( options.enable_probe_b ||
               options.enable_pnp_cheirality_retry ) &&
             result.probe_result ==
                 PnpProbeResult::kProposalRmsCheirality )
        {
          static_cast<void>( retryAfterCheirality() );
        }
        return result;
      }
      const PoseRmsResult guess_score = poseRmsAtPose(
          guess_T_W_B, mapped_observations, inlier_indices );
      result.guess_rms_px = guess_score.m_rms_px;
      if ( guess_score.m_rms_px.has_value() &&
           *proposal_score.m_rms_px >
               *guess_score.m_rms_px + options.stereo_sigma_px )
      {
        result.probe_result = PnpProbeResult::kProposalRmsRejected;
        return result;
      }

      result.inlier_ids.reserve( inlier_indices.size() );
      for ( const int index : inlier_indices )
      {
        if ( index < 0 ||
             static_cast<std::size_t>( index ) >= mapped_observations.size() )
        {
          result.probe_result =
              PnpProbeResult::kProposalRmsInvalidCorrespondence;
          return result;
        }
        result.inlier_ids.insert(
            mapped_observations[ static_cast<std::size_t>( index ) ]->id );
      }
      result.success      = true;
      result.T_W_B        = T_W_B;
      result.probe_result = PnpProbeResult::kAccepted;
      return result;
    }

    // Slice ⑦: most recent frame in the window that observed `id` with
    // stereo (disparity > 0), if any.
    [[nodiscard]] std::optional<Eigen::Isometry3d> findLastStereoFrameInWindow(
        LandmarkId id ) const
    {
      std::optional<Eigen::Isometry3d> last_stereo_pose;
      for ( auto it = m_state->m_window.rbegin(); it != m_state->m_window.rend(); ++it )
      {
        for ( const StereoObservation& observation : it->m_observations )
        {
          if ( observation.id == id && observation.disparity_px > 0.0 )
          {
            last_stereo_pose =
                requireNavigationState( *it, "last-stereo lookup" ).m_T_W_B;
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
      const auto it = m_state->m_T_W_B_last_stereo.find( id );
      if ( it == m_state->m_T_W_B_last_stereo.end() )
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
      for ( const WindowFrame& frame : m_state->m_window )
      {
        for ( const StereoObservation& observation : frame.m_observations )
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
      for ( auto it = m_state->m_landmarks_w.begin(); it != m_state->m_landmarks_w.end(); )
      {
        if ( live.find( it->first ) == live.end() )
        {
          if ( zero_live.find( it->first ) != zero_live.end() &&
               m_state->m_window.back().m_navigation.has_value() &&
               !hangingLandmarkStale( it->first,
                                      m_state->m_window.back()
                                          .m_navigation->m_T_W_B ) )
          {
            // Fresh hanging landmark (Slice ⑦): keep — it constrains the
            // BA once stereo returns and stabilizes the track.
            ++it;
            continue;
          }
          // Keep track timestamps for diagnostics across the whole run.
          m_state->m_T_W_B_last_stereo.erase( it->first );
          it = m_state->m_landmarks_w.erase( it );
        }
        else
        {
          ++it;
        }
      }
    }

    // Count only stereo observations in a measurement — those are the ones
    // that can seed a root / constrain the BA graph. Zero-disparity
    // observations must not inflate the initialization gate (a gate that
    // passes on ~180 no-depth observations but seeds ~18 weak landmarks leaves
    // an almost factor-free graph that drifts freely).
    [[nodiscard]] std::size_t countStereoObservations(
        const VioMeasurement& measurement ) const
    {
      return static_cast<std::size_t>( std::count_if(
          measurement.m_observations.begin(),
          measurement.m_observations.end(),
          []( const StereoObservation& observation ) {
            return observation.disparity_px > 0.0;
          } ) );
    }

    // Factor admission is owned by the staged map. A left-only observation
    // can constrain an existing landmark but cannot create landmark depth.
    [[nodiscard]] std::unordered_map<LandmarkId, int> countObservations(
        const internal::VioUpdateState& graph_state ) const
    {
      std::unordered_map<LandmarkId, int> counts;
      for ( const WindowFrame& frame : graph_state.m_window )
      {
        for ( const StereoObservation& observation : frame.m_observations )
        {
          if ( graph_state.m_landmarks_w.find( observation.id ) !=
               graph_state.m_landmarks_w.end() )
          {
            ++counts[ observation.id ];
          }
        }
      }
      return counts;
    }

    void rebaseWindowFront()
    {
      if ( m_state->m_window.empty() )
      {
        return;
      }
      m_state->m_window.front().m_predecessor_frame_index.reset();
      m_state->m_window.front().m_imu.reset();
    }

    void validateNavigationWindow() const
    {
      if ( m_state->m_window.empty() )
      {
        return;
      }

      const WindowFrame& root = m_state->m_window.front();
      if ( root.m_predecessor_frame_index.has_value() ||
           root.m_imu.has_value() )
      {
        throw std::runtime_error(
            "navigation window root carries a predecessor or IMU interval" );
      }

      for ( std::size_t index = 1U; index < m_state->m_window.size();
            ++index )
      {
        const WindowFrame& predecessor = m_state->m_window[ index - 1U ];
        const WindowFrame& frame       = m_state->m_window[ index ];
        if ( !frame.m_predecessor_frame_index.has_value() ||
             *frame.m_predecessor_frame_index !=
                 predecessor.m_frame_index ||
             !frame.m_imu.has_value() )
        {
          throw std::runtime_error(
              "navigation window contains a dangling predecessor link" );
        }
        const std::optional<std::int64_t> duration_ns =
            checkedPositiveDurationNs( predecessor.m_timestamp,
                                       frame.m_timestamp );
        if ( predecessor.m_frame_index >= frame.m_frame_index ||
             !duration_ns.has_value() ||
             frame.m_imu->m_raw.m_t_begin != predecessor.m_timestamp ||
             frame.m_imu->m_raw.m_t_end != frame.m_timestamp ||
             frame.m_imu->m_nodes.empty() ||
             frame.m_imu->m_nodes.front().timestamp !=
                 predecessor.m_timestamp ||
             frame.m_imu->m_nodes.back().timestamp != frame.m_timestamp ||
             frame.m_imu->m_duration_ns != *duration_ns )
        {
          throw std::runtime_error(
              "navigation window provenance does not match linked states" );
        }
      }
    }

    [[nodiscard]] bool evictOldestNonKeyframe()
    {
      if ( m_state->m_window.size() < 3U )
      {
        return false;
      }

      const auto last    = std::prev( m_state->m_window.end() );
      const auto evicted = std::find_if(
          std::next( m_state->m_window.begin() ), last,
          []( const WindowFrame& frame ) { return !frame.m_is_keyframe; } );
      if ( evicted == last )
      {
        return false;
      }

      const auto predecessor = std::prev( evicted );
      const auto successor   = std::next( evicted );
      if ( !evicted->m_predecessor_frame_index.has_value() ||
           *evicted->m_predecessor_frame_index !=
               predecessor->m_frame_index ||
           !evicted->m_imu.has_value() ||
           !successor->m_predecessor_frame_index.has_value() ||
           *successor->m_predecessor_frame_index !=
               evicted->m_frame_index ||
           !successor->m_imu.has_value() )
      {
        throw std::runtime_error(
            "non-keyframe eviction requires adjacent navigation provenance" );
      }

      ImuIntervalResult joined = spliceNormalizedImuIntervals(
          *evicted->m_imu, *successor->m_imu,
          predecessor->m_timestamp, evicted->m_timestamp,
          successor->m_timestamp );
      if ( std::holds_alternative<ImuIntervalError>( joined ) )
      {
        throw std::runtime_error(
            std::get<ImuIntervalError>( std::move( joined ) ).m_detail );
      }

      successor->m_predecessor_frame_index = predecessor->m_frame_index;
      successor->m_imu =
          std::get<NormalizedImuInterval>( std::move( joined ) );
      m_state->m_window.erase( evicted );
      ++m_state->m_non_keyframe_evictions;
      ++m_state->m_imu_reintegrations;
      return true;
    }

    void enforceWindowCapacity()
    {
      while ( static_cast<int>( m_state->m_window.size() ) >
              options.window_size )
      {
        if ( evictOldestNonKeyframe() )
        {
          continue;
        }
        m_state->m_window.pop_front();
        rebaseWindowFront();
      }
      validateNavigationWindow();
    }

    [[nodiscard]] gtsam::PreintegratedImuMeasurements preintegrate(
        const NormalizedImuInterval& interval,
        const ImuBias&               bias ) const
    {
      gtsam::PreintegratedImuMeasurements pim( pim_params,
                                               toGtsamBias( bias ) );
      for ( const ImuIntervalStep& step : interval.m_steps )
      {
        pim.integrateMeasurement( step.m_acc_mean_mps2,
                                  step.m_gyr_mean_radps, step.m_dt_s );
      }
      return pim;
    }

    struct MovingEvidenceSummary
    {
      std::uint32_t                           m_connected_poses     = 0;
      std::int64_t                            m_duration_ns         = 0;
      std::uint32_t                           m_connected_landmarks = 0;
      std::optional<std::uint32_t>            m_visual_support;
      std::optional<InitializationFitSummary> m_visual_fit;
      bool                                    m_latest_verified = false;
      InitializationReason                    m_reason =
          InitializationReason::kVisualConnectivityInsufficient;
    };

    struct MovingVisualFactorRef
    {
      std::size_t   m_graph_index = 0;
      std::uint64_t m_frame_index = 0;
      LandmarkId    m_landmark_id{};
      std::size_t   m_observation_index = 0;
    };

    [[nodiscard]] MovingEvidenceSummary stageMovingVisualEvidence(
        const VioMeasurement&        measurement,
        const NormalizedImuInterval& normalized_imu,
        const bool                   keyframe )
    {
      const bool first_frame = m_state->m_window.empty();
      if ( first_frame )
      {
        m_state->m_landmarks_w.clear();
      }

      WindowFrame frame;
      frame.m_frame_index  = m_state->m_next_frame_index;
      frame.m_timestamp    = measurement.m_timestamp;
      frame.m_observations = measurement.m_observations;
      frame.m_is_keyframe  = keyframe;
      VisualPoseSeed visual_seed;
      for ( auto it = m_state->m_window.rbegin();
            it != m_state->m_window.rend(); ++it )
      {
        if ( it->m_verified_visual.has_value() )
        {
          visual_seed.m_T_W_B = it->m_verified_visual->m_T_W_B;
          break;
        }
      }
      frame.m_visual_seed = visual_seed;
      if ( !first_frame )
      {
        frame.m_predecessor_frame_index =
            m_state->m_window.back().m_frame_index;
        frame.m_imu = normalized_imu;
      }
      m_state->m_window.push_back( std::move( frame ) );
      ++m_state->m_next_frame_index;
      enforceWindowCapacity();
      pruneLandmarksNotInWindow();

      for ( const StereoObservation& observation :
            measurement.m_observations )
      {
        m_state->m_track_times[ observation.id ].push_back(
            measurement.m_timestamp );
      }

      WindowFrame& current = m_state->m_window.back();
      if ( current.m_is_keyframe )
      {
        for ( const StereoObservation& observation : current.m_observations )
        {
          if ( observation.disparity_px <= 0.0 ||
               m_state->m_landmarks_w.contains( observation.id ) )
          {
            continue;
          }
          const Eigen::Vector3d point_W =
              backprojectWorld( current.m_visual_seed->m_T_W_B,
                                observation );
          if ( !isFinite( point_W ) )
          {
            throw std::runtime_error(
                "moving visual landmark seed is non-finite" );
          }
          m_state->m_landmarks_w.emplace( observation.id, point_W );
        }
      }

      const auto landmarks_before_visual_solve = m_state->m_landmarks_w;
      std::unordered_map<std::uint64_t, std::optional<VerifiedVisualPose>>
          verified_before_visual_solve;
      for ( const WindowFrame& candidate : m_state->m_window )
      {
        verified_before_visual_solve.emplace(
            candidate.m_frame_index, candidate.m_verified_visual );
      }
      const auto restoreVisualAuthority = [ & ]() {
        m_state->m_landmarks_w = landmarks_before_visual_solve;
        for ( WindowFrame& candidate : m_state->m_window )
        {
          const auto before = verified_before_visual_solve.find(
              candidate.m_frame_index );
          candidate.m_verified_visual =
              before == verified_before_visual_solve.end()
                  ? std::nullopt
                  : before->second;
        }
        pruneLandmarksNotInWindow();
      };

      validateNavigationWindow();
#ifdef PHAD_ESTIMATOR_TESTING
      m_test_last_initial_visual_solve_window_size =
          m_state->m_window.size();
#endif
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values               values;
      for ( const WindowFrame& candidate : m_state->m_window )
      {
        if ( !candidate.m_visual_seed.has_value() )
        {
          throw std::runtime_error(
              "moving visual frame is missing its typed seed" );
        }
        values.insert( X( candidate.m_frame_index ),
                       toPose3( candidate.m_visual_seed->m_T_W_B ) );
      }
      std::unordered_set<LandmarkId>     inserted_landmarks;
      std::vector<MovingVisualFactorRef> factor_refs;
      for ( const WindowFrame& candidate : m_state->m_window )
      {
        for ( std::size_t observation_index = 0U;
              observation_index < candidate.m_observations.size();
              ++observation_index )
        {
          const StereoObservation& observation =
              candidate.m_observations[ observation_index ];
          if ( observation.disparity_px <= 0.0 )
          {
            continue;
          }
          const auto landmark_it =
              m_state->m_landmarks_w.find( observation.id );
          if ( landmark_it == m_state->m_landmarks_w.end() )
          {
            continue;
          }
          if ( inserted_landmarks.insert( observation.id ).second )
          {
            values.insert( L( observation.id ),
                           gtsam::Point3( landmark_it->second.x(),
                                          landmark_it->second.y(),
                                          landmark_it->second.z() ) );
          }
          factor_refs.push_back( MovingVisualFactorRef{
              .m_graph_index       = graph.size(),
              .m_frame_index       = candidate.m_frame_index,
              .m_landmark_id       = observation.id,
              .m_observation_index = observation_index } );
          graph.emplace_shared<StereoVisualFactor>(
              toStereoPoint( observation ), stereo_noise,
              X( candidate.m_frame_index ), L( observation.id ), K,
              body_P_sensor );
        }
      }

      if ( factor_refs.empty() )
      {
        enforceWindowCapacity();
        pruneLandmarksNotInWindow();
        return {};
      }

      std::unordered_map<LandmarkId, std::vector<std::uint64_t>>
          frames_by_landmark;
      std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
                                        frame_adjacency;
      std::unordered_set<std::uint64_t> frames_with_factors;
      for ( const MovingVisualFactorRef& ref : factor_refs )
      {
        frames_by_landmark[ ref.m_landmark_id ].push_back(
            ref.m_frame_index );
        frames_with_factors.insert( ref.m_frame_index );
      }
      for ( auto& [ id, frame_indices ] : frames_by_landmark )
      {
        (void)id;
        std::sort( frame_indices.begin(), frame_indices.end() );
        frame_indices.erase(
            std::unique( frame_indices.begin(), frame_indices.end() ),
            frame_indices.end() );
        for ( std::size_t index = 1U; index < frame_indices.size(); ++index )
        {
          frame_adjacency[ frame_indices.front() ].push_back(
              frame_indices[ index ] );
          frame_adjacency[ frame_indices[ index ] ].push_back(
              frame_indices.front() );
        }
      }

      const std::uint64_t staged_latest_index =
          m_state->m_window.back().m_frame_index;
      if ( !frames_with_factors.contains( staged_latest_index ) )
      {
        enforceWindowCapacity();
        pruneLandmarksNotInWindow();
        return {};
      }

      std::unordered_set<std::uint64_t> latest_factor_frames{
          staged_latest_index };
      std::vector<std::uint64_t> latest_frontier{ staged_latest_index };
      for ( std::size_t index = 0U; index < latest_frontier.size();
            ++index )
      {
        for ( const std::uint64_t neighbor :
              frame_adjacency[ latest_frontier[ index ] ] )
        {
          if ( latest_factor_frames.insert( neighbor ).second )
          {
            latest_frontier.push_back( neighbor );
          }
        }
      }

      std::vector<MovingVisualFactorRef> component_factor_refs;
      std::unordered_set<LandmarkId>     latest_factor_landmarks;
      gtsam::NonlinearFactorGraph        component_graph;
      for ( const MovingVisualFactorRef& ref : factor_refs )
      {
        if ( latest_factor_frames.contains( ref.m_frame_index ) )
        {
          component_factor_refs.push_back( ref );
          latest_factor_landmarks.insert( ref.m_landmark_id );
          component_graph.push_back( graph[ ref.m_graph_index ] );
        }
      }
      std::uint32_t latest_factor_component_landmarks =
          static_cast<std::uint32_t>( latest_factor_landmarks.size() );

      gtsam::Values                component_values;
      std::optional<std::uint64_t> visual_gauge_index;
      for ( const WindowFrame& candidate : m_state->m_window )
      {
        if ( !latest_factor_frames.contains( candidate.m_frame_index ) )
        {
          continue;
        }
        component_values.insert(
            X( candidate.m_frame_index ),
            values.at<gtsam::Pose3>( X( candidate.m_frame_index ) ) );
        if ( !visual_gauge_index.has_value() )
        {
          visual_gauge_index = candidate.m_frame_index;
          component_graph.emplace_shared<
              gtsam::PriorFactor<gtsam::Pose3>>(
              X( candidate.m_frame_index ),
              toPose3( candidate.m_visual_seed->m_T_W_B ), prior_noise );
        }
      }
      for ( const LandmarkId id : latest_factor_landmarks )
      {
        component_values.insert(
            L( id ), values.at<gtsam::Point3>( L( id ) ) );
      }
      if ( !visual_gauge_index.has_value() )
      {
        throw std::runtime_error(
            "moving latest visual component has no gauge frame" );
      }
      std::unordered_set<std::uint64_t> visual_gauge_indices{
          *visual_gauge_index };

      GraphSolveResult solved =
          solveGraph( component_graph, component_values );
      if ( std::holds_alternative<GraphSolveError>( solved ) )
      {
        throw std::runtime_error(
            std::get<GraphSolveError>( solved ).m_detail );
      }
      gtsam::Values optimized =
          std::get<GraphSolveSuccess>( std::move( solved ) ).m_values;

      const auto visualFactorQ =
          [ & ]( const MovingVisualFactorRef& ref,
                 const gtsam::Values&         evaluated ) {
            const std::optional<VisualFactorView> visual =
                asVisualFactor( graph[ ref.m_graph_index ] );
            if ( !visual.has_value() )
            {
              throw std::runtime_error(
                  "moving visual factor provenance is invalid" );
            }
            const gtsam::Vector error =
                visual->unwhitenedError( evaluated );
#ifdef PHAD_ESTIMATOR_TESTING
            if ( m_test_next_initial_visual_whitening_error )
            {
              m_test_next_initial_visual_whitening_error = false;
              throw std::runtime_error(
                  "injected initial visual whitening error" );
            }
#endif
            return initialVisualUnderlyingGaussianQ(
                error, visual->m_factor->noiseModel() );
          };

      std::vector<MovingVisualFactorRef> retained_mask;
      retained_mask.reserve( component_factor_refs.size() );
      for ( const MovingVisualFactorRef& ref : component_factor_refs )
      {
        switch ( classifyInitialVisualFactorMask(
            visualFactorQ( ref, optimized ) ) )
        {
          case InitialVisualFactorMaskDecision::kRetained:
            retained_mask.push_back( ref );
            break;
          case InitialVisualFactorMaskDecision::kMasked:
            break;
          case InitialVisualFactorMaskDecision::kContractError:
            throw std::runtime_error(
                "moving visual mask statistic violates its contract" );
        }
      }

      std::unordered_map<
          std::uint64_t,
          std::vector<VerifiedVisualPose::RetainedStereoFactor>>
                                              retained;
      std::optional<InitializationFitSummary> visual_fit;
      bool                                    final_fit_rejected        = false;
      bool                                    final_cheirality_rejected = false;
      if ( !retained_mask.empty() )
      {
        std::unordered_map<LandmarkId, std::vector<std::uint64_t>>
            masked_frames_by_landmark;
        std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
                                          masked_adjacency;
        std::unordered_set<std::uint64_t> masked_frames;
        for ( const MovingVisualFactorRef& ref : retained_mask )
        {
          masked_frames_by_landmark[ ref.m_landmark_id ].push_back(
              ref.m_frame_index );
          masked_frames.insert( ref.m_frame_index );
        }
        if ( !masked_frames.contains( staged_latest_index ) )
        {
          enforceWindowCapacity();
          pruneLandmarksNotInWindow();
          return {};
        }
        for ( auto& [ id, frame_indices ] : masked_frames_by_landmark )
        {
          (void)id;
          std::sort( frame_indices.begin(), frame_indices.end() );
          frame_indices.erase(
              std::unique( frame_indices.begin(), frame_indices.end() ),
              frame_indices.end() );
          for ( std::size_t index = 1U; index < frame_indices.size();
                ++index )
          {
            masked_adjacency[ frame_indices.front() ].push_back(
                frame_indices[ index ] );
            masked_adjacency[ frame_indices[ index ] ].push_back(
                frame_indices.front() );
          }
        }

        std::unordered_set<std::uint64_t> final_mask_frames{
            staged_latest_index };
        std::vector<std::uint64_t> final_frontier{ staged_latest_index };
        for ( std::size_t index = 0U; index < final_frontier.size();
              ++index )
        {
          for ( const std::uint64_t neighbor :
                masked_adjacency[ final_frontier[ index ] ] )
          {
            if ( final_mask_frames.insert( neighbor ).second )
            {
              final_frontier.push_back( neighbor );
            }
          }
        }

        std::vector<MovingVisualFactorRef> final_mask_refs;
        std::unordered_set<LandmarkId>     final_mask_landmarks;
        gtsam::NonlinearFactorGraph        masked_graph;
        for ( const MovingVisualFactorRef& ref : retained_mask )
        {
          if ( final_mask_frames.contains( ref.m_frame_index ) )
          {
            final_mask_refs.push_back( ref );
            final_mask_landmarks.insert( ref.m_landmark_id );
            masked_graph.push_back( graph[ ref.m_graph_index ] );
          }
        }
        gtsam::Values masked_values;
        visual_gauge_indices.clear();
        for ( const WindowFrame& candidate : m_state->m_window )
        {
          if ( !final_mask_frames.contains( candidate.m_frame_index ) )
          {
            continue;
          }
          masked_values.insert(
              X( candidate.m_frame_index ),
              optimized.at<gtsam::Pose3>( X( candidate.m_frame_index ) ) );
          if ( visual_gauge_indices.empty() )
          {
            visual_gauge_indices.insert( candidate.m_frame_index );
            masked_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                X( candidate.m_frame_index ),
                toPose3( candidate.m_visual_seed->m_T_W_B ), prior_noise );
          }
        }
        for ( const LandmarkId id : final_mask_landmarks )
        {
          masked_values.insert(
              L( id ), optimized.at<gtsam::Point3>( L( id ) ) );
        }
        retained_mask        = std::move( final_mask_refs );
        latest_factor_frames = std::move( final_mask_frames );
        latest_factor_component_landmarks =
            static_cast<std::uint32_t>( final_mask_landmarks.size() );
        GraphSolveResult masked_solved =
            solveGraph( masked_graph, masked_values );
        if ( std::holds_alternative<GraphSolveError>( masked_solved ) )
        {
          throw std::runtime_error(
              std::get<GraphSolveError>( masked_solved ).m_detail );
        }
        optimized =
            std::get<GraphSolveSuccess>( std::move( masked_solved ) )
                .m_values;

        double worst_q = 0.0;
        for ( const MovingVisualFactorRef& ref : retained_mask )
        {
          const double q = visualFactorQ( ref, optimized );
          worst_q        = std::max( worst_q, q );
          const gtsam::Pose3 T_W_left =
              optimized.at<gtsam::Pose3>( X( ref.m_frame_index ) ) *
              body_P_sensor;
          const gtsam::Point3 point_W =
              optimized.at<gtsam::Point3>( L( ref.m_landmark_id ) );
          double depth = T_W_left.transformTo( point_W ).z();
#ifdef PHAD_ESTIMATOR_TESTING
          if ( m_test_next_initial_visual_depth.has_value() )
          {
            depth = *m_test_next_initial_visual_depth;
            m_test_next_initial_visual_depth.reset();
          }
#endif
          switch ( classifyInitialVisualFactorFinal( q, depth ) )
          {
            case InitialVisualFactorFinalDecision::kAccepted:
              retained[ ref.m_frame_index ].push_back(
                  VerifiedVisualPose::RetainedStereoFactor{
                      .m_landmark_id       = ref.m_landmark_id,
                      .m_observation_index = ref.m_observation_index } );
              break;
            case InitialVisualFactorFinalDecision::kVisualFitRejected:
              final_fit_rejected = true;
              break;
            case InitialVisualFactorFinalDecision::kCheiralityRejected:
              final_cheirality_rejected = true;
              break;
            case InitialVisualFactorFinalDecision::kContractError:
              throw std::runtime_error(
                  "moving visual final factor violates its contract" );
          }
        }
        visual_fit = InitializationFitSummary{
            .m_statistic     = worst_q,
            .m_limit         = kInitialVisualFactorChiSquareLimit,
            .m_effective_dof = 3 };
      }
      else
      {
        enforceWindowCapacity();
        pruneLandmarksNotInWindow();
        return {};
      }
      InitializationReason latest_failure_reason =
          InitializationReason::kVisualConnectivityInsufficient;
      std::uint32_t                latest_retained_landmarks = 0;
      std::optional<std::uint32_t> latest_support;
      const std::uint64_t          latest_index =
          m_state->m_window.back().m_frame_index;
      for ( WindowFrame& candidate : m_state->m_window )
      {
        if ( !latest_factor_frames.contains( candidate.m_frame_index ) )
        {
          continue;
        }
        std::vector<VerifiedVisualPose::RetainedStereoFactor>& factors =
            retained[ candidate.m_frame_index ];
        std::sort(
            factors.begin(), factors.end(),
            []( const VerifiedVisualPose::RetainedStereoFactor& lhs,
                const VerifiedVisualPose::RetainedStereoFactor& rhs ) {
              if ( lhs.m_observation_index != rhs.m_observation_index )
              {
                return lhs.m_observation_index < rhs.m_observation_index;
              }
              return lhs.m_landmark_id < rhs.m_landmark_id;
            } );
        factors.erase(
            std::unique(
                factors.begin(), factors.end(),
                []( const VerifiedVisualPose::RetainedStereoFactor& lhs,
                    const VerifiedVisualPose::RetainedStereoFactor& rhs ) {
                  return lhs.m_observation_index == rhs.m_observation_index &&
                         lhs.m_landmark_id == rhs.m_landmark_id;
                } ),
            factors.end() );
        if ( final_fit_rejected &&
             candidate.m_frame_index == latest_index )
        {
          latest_failure_reason =
              InitializationReason::kVisualFitRejected;
          candidate.m_verified_visual.reset();
          continue;
        }
        if ( final_cheirality_rejected &&
             candidate.m_frame_index == latest_index )
        {
          latest_failure_reason =
              InitializationReason::kCheiralityRejected;
          candidate.m_verified_visual.reset();
          continue;
        }
        constexpr int required = 1;
        if ( static_cast<int>( factors.size() ) < required )
        {
          if ( candidate.m_frame_index == latest_index )
          {
            if ( visual_gauge_indices.contains(
                     candidate.m_frame_index ) )
            {
              latest_failure_reason =
                  InitializationReason::kVisualPopulationInsufficient;
              std::unordered_set<LandmarkId> unique_ids;
              for ( const auto& factor : factors )
              {
                unique_ids.insert( factor.m_landmark_id );
              }
              latest_retained_landmarks =
                  static_cast<std::uint32_t>( unique_ids.size() );
            }
            else
            {
              if ( factors.empty() )
              {
                latest_failure_reason =
                    InitializationReason::kVisualConnectivityInsufficient;
              }
              else
              {
                latest_failure_reason =
                    InitializationReason::kVisualSupportInsufficient;
                latest_support =
                    static_cast<std::uint32_t>( factors.size() );
              }
            }
          }
          candidate.m_verified_visual.reset();
          continue;
        }
        const Eigen::Isometry3d T_W_B = toIsometry(
            optimized.at<gtsam::Pose3>( X( candidate.m_frame_index ) ) );
        if ( !isFinite( T_W_B ) )
        {
          throw std::runtime_error(
              "moving verified visual pose is non-finite" );
        }
        candidate.m_verified_visual = VerifiedVisualPose{
            .m_T_W_B = T_W_B, .m_stereo_factors = std::move( factors ) };
      }
      for ( auto& [ id, point_W ] : m_state->m_landmarks_w )
      {
        if ( !optimized.exists( L( id ) ) )
        {
          continue;
        }
        const gtsam::Point3 point =
            optimized.at<gtsam::Point3>( L( id ) );
        point_W = Eigen::Vector3d( point.x(), point.y(), point.z() );
        if ( !isFinite( point_W ) )
        {
          throw std::runtime_error(
              "moving optimized visual landmark is non-finite" );
        }
      }

      enforceWindowCapacity();
      pruneLandmarksNotInWindow();

      MovingEvidenceSummary summary;
      summary.m_visual_fit = visual_fit;
      summary.m_latest_verified =
          m_state->m_window.back().m_verified_visual.has_value();
      if ( !summary.m_latest_verified )
      {
        summary.m_reason = latest_failure_reason;
        summary.m_connected_landmarks =
            latest_factor_component_landmarks > 0U
                ? latest_factor_component_landmarks
                : latest_retained_landmarks;
        summary.m_visual_support = latest_support;
        restoreVisualAuthority();
        return summary;
      }
      summary.m_reason = InitializationReason::kPoseCountInsufficient;

      std::unordered_map<std::uint64_t, const WindowFrame*> verified_by_index;
      std::unordered_map<LandmarkId, std::vector<std::uint64_t>>
          verified_frames_by_landmark;
      for ( const WindowFrame& candidate : m_state->m_window )
      {
        if ( !candidate.m_verified_visual.has_value() )
        {
          continue;
        }
        verified_by_index.emplace( candidate.m_frame_index, &candidate );
        for ( const auto& factor :
              candidate.m_verified_visual->m_stereo_factors )
        {
          verified_frames_by_landmark[ factor.m_landmark_id ].push_back(
              candidate.m_frame_index );
        }
      }
      const std::uint64_t latest_frame_index =
          m_state->m_window.back().m_frame_index;
      std::unordered_set<std::uint64_t> latest_component{
          latest_frame_index };
      std::vector<std::uint64_t> frontier{ latest_frame_index };
      for ( std::size_t index = 0U; index < frontier.size(); ++index )
      {
        const WindowFrame& candidate =
            *verified_by_index.at( frontier[ index ] );
        for ( const auto& factor :
              candidate.m_verified_visual->m_stereo_factors )
        {
          for ( const std::uint64_t neighbor :
                verified_frames_by_landmark[ factor.m_landmark_id ] )
          {
            if ( latest_component.insert( neighbor ).second )
            {
              frontier.push_back( neighbor );
            }
          }
        }
      }

      std::unordered_set<LandmarkId>   connected_landmarks;
      std::optional<common::Timestamp> first_verified;
      std::optional<common::Timestamp> last_verified;
      std::uint32_t                    weakest_support =
          std::numeric_limits<std::uint32_t>::max();
      for ( const WindowFrame& candidate : m_state->m_window )
      {
        if ( !latest_component.contains( candidate.m_frame_index ) )
        {
          continue;
        }
        ++summary.m_connected_poses;
        if ( !first_verified.has_value() )
        {
          first_verified = candidate.m_timestamp;
        }
        else
        {
          weakest_support = std::min(
              weakest_support,
              static_cast<std::uint32_t>(
                  candidate.m_verified_visual->m_stereo_factors.size() ) );
        }
        last_verified = candidate.m_timestamp;
        for ( const auto& factor :
              candidate.m_verified_visual->m_stereo_factors )
        {
          connected_landmarks.insert( factor.m_landmark_id );
        }
      }
      summary.m_connected_landmarks =
          static_cast<std::uint32_t>( connected_landmarks.size() );
      if ( summary.m_connected_poses > 1U )
      {
        const std::optional<std::int64_t> duration_ns =
            checkedPositiveDurationNs( *first_verified, *last_verified );
        if ( !duration_ns.has_value() )
        {
          throw std::runtime_error(
              "moving verified visual duration is invalid" );
        }
        summary.m_duration_ns    = *duration_ns;
        summary.m_visual_support = weakest_support;
      }
      if ( summary.m_connected_landmarks <
           static_cast<std::uint32_t>( options.min_seed_observations ) )
      {
        m_state->m_window.back().m_verified_visual.reset();
        summary.m_latest_verified = false;
        summary.m_reason =
            InitializationReason::kVisualPopulationInsufficient;
        summary.m_connected_poses = 0U;
        summary.m_duration_ns     = 0;
        summary.m_visual_support.reset();
        restoreVisualAuthority();
        return summary;
      }
      if ( summary.m_visual_support.has_value() &&
           *summary.m_visual_support <
               static_cast<std::uint32_t>( options.min_pnp_inliers ) )
      {
        m_state->m_window.back().m_verified_visual.reset();
        summary.m_latest_verified = false;
        summary.m_reason =
            InitializationReason::kVisualSupportInsufficient;
        summary.m_connected_poses = 0U;
        summary.m_duration_ns     = 0;
        restoreVisualAuthority();
        return summary;
      }
      if ( m_state->m_evidence_revision ==
           std::numeric_limits<std::uint64_t>::max() )
      {
        throw std::runtime_error(
            "moving evidence revision overflows uint64" );
      }
      ++m_state->m_evidence_revision;
      return summary;
    }

    struct PreparedMovingCandidate
    {
      InitialMovingMathInput         m_math;
      Eigen::MatrixXd                m_visual_pose_covariance;
      std::vector<std::uint64_t>     m_frame_indices;
      std::unordered_set<LandmarkId> m_landmark_ids;
    };

    [[nodiscard]] PreparedMovingCandidate prepareMovingCandidate() const
    {
      if ( m_state->m_window.empty() ||
           !m_state->m_window.back().m_verified_visual.has_value() )
      {
        throw std::runtime_error(
            "moving candidate requires a verified latest frame" );
      }

      std::unordered_map<LandmarkId, std::vector<std::uint64_t>>
                                                            frames_by_landmark;
      std::unordered_map<std::uint64_t, const WindowFrame*> frames_by_index;
      for ( const WindowFrame& frame : m_state->m_window )
      {
        if ( !frame.m_verified_visual.has_value() )
        {
          continue;
        }
        frames_by_index.emplace( frame.m_frame_index, &frame );
        for ( const auto& factor :
              frame.m_verified_visual->m_stereo_factors )
        {
          frames_by_landmark[ factor.m_landmark_id ].push_back(
              frame.m_frame_index );
        }
      }
      const std::uint64_t latest_index =
          m_state->m_window.back().m_frame_index;
      std::unordered_set<std::uint64_t> component{ latest_index };
      std::vector<std::uint64_t>        frontier{ latest_index };
      for ( std::size_t index = 0U; index < frontier.size(); ++index )
      {
        const WindowFrame& frame = *frames_by_index.at( frontier[ index ] );
        for ( const auto& factor :
              frame.m_verified_visual->m_stereo_factors )
        {
          for ( const std::uint64_t neighbor :
                frames_by_landmark[ factor.m_landmark_id ] )
          {
            if ( component.insert( neighbor ).second )
            {
              frontier.push_back( neighbor );
            }
          }
        }
      }

      PreparedMovingCandidate  prepared;
      std::vector<std::size_t> selected_positions;
      for ( std::size_t position = 0U;
            position < m_state->m_window.size(); ++position )
      {
        const WindowFrame& frame = m_state->m_window[ position ];
        if ( !component.contains( frame.m_frame_index ) )
        {
          continue;
        }
        selected_positions.push_back( position );
        prepared.m_frame_indices.push_back( frame.m_frame_index );
        prepared.m_math.m_poses.push_back( InitialMovingPose{
            .m_frame_index = frame.m_frame_index,
            .m_timestamp   = frame.m_timestamp,
            .m_T_W_B       = frame.m_verified_visual->m_T_W_B } );
      }
      if ( selected_positions.size() < 2U )
      {
        throw std::runtime_error(
            "moving candidate component has too few selected poses" );
      }

      for ( std::size_t selected_index = 1U;
            selected_index < selected_positions.size(); ++selected_index )
      {
        const std::size_t begin_position =
            selected_positions[ selected_index - 1U ];
        const std::size_t end_position =
            selected_positions[ selected_index ];
        if ( end_position <= begin_position ||
             begin_position + 1U >= m_state->m_window.size() ||
             !m_state->m_window[ begin_position + 1U ].m_imu.has_value() )
        {
          throw std::runtime_error(
              "moving candidate interval chain is incomplete" );
        }
        NormalizedImuInterval joined =
            *m_state->m_window[ begin_position + 1U ].m_imu;
        for ( std::size_t position = begin_position + 2U;
              position <= end_position; ++position )
        {
          const WindowFrame& current = m_state->m_window[ position ];
          if ( !current.m_imu.has_value() )
          {
            throw std::runtime_error(
                "moving candidate interval splice input is missing" );
          }
          ImuIntervalResult splice = spliceNormalizedImuIntervals(
              joined, *current.m_imu,
              m_state->m_window[ begin_position ].m_timestamp,
              m_state->m_window[ position - 1U ].m_timestamp,
              current.m_timestamp );
          if ( std::holds_alternative<ImuIntervalError>( splice ) )
          {
            throw std::runtime_error(
                std::get<ImuIntervalError>( splice ).m_detail );
          }
          joined =
              std::get<NormalizedImuInterval>( std::move( splice ) );
        }
        prepared.m_math.m_intervals.push_back( std::move( joined ) );
      }
      prepared.m_math.m_pim = InitialMovingPimConfig{
          .m_acc_cov         = pim_params->accelerometerCovariance,
          .m_gyr_cov         = pim_params->gyroscopeCovariance,
          .m_integration_cov = pim_params->integrationCovariance };
      prepared.m_math.m_gravity_mps2 = options.m_gravity_mps2;

      gtsam::NonlinearFactorGraph visual_graph;
      gtsam::Values               visual_values;
      for ( const std::size_t position : selected_positions )
      {
        const WindowFrame& frame = m_state->m_window[ position ];
        visual_values.insert(
            X( frame.m_frame_index ),
            toPose3( frame.m_verified_visual->m_T_W_B ) );
      }
      const WindowFrame& gauge =
          m_state->m_window[ selected_positions.front() ];
      visual_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          X( gauge.m_frame_index ),
          toPose3( gauge.m_verified_visual->m_T_W_B ), prior_noise );

      for ( const std::size_t position : selected_positions )
      {
        const WindowFrame& frame = m_state->m_window[ position ];
        for ( const auto& factor :
              frame.m_verified_visual->m_stereo_factors )
        {
          if ( factor.m_observation_index >= frame.m_observations.size() )
          {
            throw std::runtime_error(
                "moving retained factor observation index is invalid" );
          }
          const StereoObservation& observation =
              frame.m_observations[ factor.m_observation_index ];
          if ( observation.id != factor.m_landmark_id ||
               observation.disparity_px <= 0.0 )
          {
            throw std::runtime_error(
                "moving retained factor provenance does not match observation" );
          }
          const auto landmark =
              m_state->m_landmarks_w.find( factor.m_landmark_id );
          if ( landmark == m_state->m_landmarks_w.end() )
          {
            throw std::runtime_error(
                "moving retained factor landmark is missing" );
          }
          if ( prepared.m_landmark_ids.insert(
                                          factor.m_landmark_id )
                   .second )
          {
            visual_values.insert(
                L( factor.m_landmark_id ),
                gtsam::Point3( landmark->second.x(), landmark->second.y(),
                               landmark->second.z() ) );
          }
          visual_graph.emplace_shared<StereoVisualFactor>(
              toStereoPoint( observation ), stereo_noise,
              X( frame.m_frame_index ), L( factor.m_landmark_id ), K,
              body_P_sensor );
        }
      }

      const gtsam::Marginals marginals(
          visual_graph, visual_values, gtsam::Marginals::QR );
      gtsam::KeyVector non_root_keys;
      for ( std::size_t index = 1U;
            index < prepared.m_frame_indices.size(); ++index )
      {
        non_root_keys.push_back( X( prepared.m_frame_indices[ index ] ) );
      }
      const gtsam::JointMarginal joint =
          marginals.jointMarginalCovariance( non_root_keys );
      const Eigen::Index covariance_dimension =
          static_cast<Eigen::Index>( 6U * non_root_keys.size() );
      prepared.m_visual_pose_covariance =
          Eigen::MatrixXd::Zero( covariance_dimension,
                                 covariance_dimension );
      for ( std::size_t row = 0U; row < non_root_keys.size(); ++row )
      {
        for ( std::size_t column = 0U;
              column < non_root_keys.size(); ++column )
        {
          prepared.m_visual_pose_covariance.block<6, 6>(
              static_cast<Eigen::Index>( 6U * row ),
              static_cast<Eigen::Index>( 6U * column ) ) =
              joint.at( non_root_keys[ row ], non_root_keys[ column ] );
        }
      }
      if ( !prepared.m_visual_pose_covariance.allFinite() )
      {
        throw std::runtime_error(
            "moving visual joint covariance is non-finite" );
      }
      return prepared;
    }

    struct CurrentGraphValidation
    {
      internal::VioUpdateState                      m_candidate;
      VioGraphInfo                                  m_info;
      std::uint64_t                                 m_prior_key               = 0;
      std::uint32_t                                 m_num_landmarks           = 0;
      std::uint32_t                                 m_lm_iterations           = 0;
      double                                        m_reproj_rms_before       = 0.0;
      double                                        m_reproj_rms              = 0.0;
      double                                        m_reproj_rms_after_cull   = 0.0;
      double                                        m_max_window_pose_shift_m = 0.0;
      std::uint32_t                                 m_num_cheirality          = 0;
      std::uint32_t                                 m_outliers_culled         = 0;
      std::uint32_t                                 m_outliers_culled_unique  = 0;
      std::uint32_t                                 m_outlier_reopt_rounds    = 0;
      bool                                          m_outlier_reopt_failed    = false;
      std::vector<LandmarkId>                       m_culled_landmark_ids;
      std::vector<std::pair<std::uint64_t, double>> m_probe_shift_top;
      double                                        m_probe_res_mean_px = 0.0;
      double                                        m_probe_res_max_px  = 0.0;
      LandmarkId                                    m_probe_res_max_id{};
      bool                                          m_probe_detail_valid = false;
      std::optional<double>                         m_probe_current_pose_translation_delta_m;
      std::optional<double>                         m_probe_current_pose_rotation_delta_rad;
      std::optional<double>                         m_probe_current_velocity_delta_mps;
      std::optional<double>                         m_probe_current_acc_bias_delta_mps2;
      std::optional<double>                         m_probe_current_gyr_bias_delta_radps;
      std::optional<double>                         m_probe_primary_graph_error_seed;
      std::optional<double>                         m_probe_primary_graph_error_optimized;
      std::optional<double>                         m_probe_primary_imu_factor_error_seed;
      std::optional<double>                         m_probe_primary_imu_factor_error_optimized;
    };

    using CurrentGraphLifecycleResult =
        std::variant<CurrentGraphValidation, InitializationReason>;

    [[nodiscard]] internal::VioUpdateState materializeMovingCandidate(
        const PreparedMovingCandidate&        prepared,
        const InitialMovingCandidateSolution& solution ) const
    {
      if ( prepared.m_frame_indices.size() !=
               solution.m_candidate.m_v_W.size() ||
           prepared.m_frame_indices.size() < 2U )
      {
        throw std::runtime_error(
            "moving candidate state dimensions are invalid" );
      }

      const Eigen::Isometry3d T_W_V = makeMovingWorldAlignment(
          solution.m_candidate.m_g_W, options.m_gravity_mps2 );
      const Eigen::Matrix3d R_W_V = T_W_V.rotation();

      internal::VioUpdateState candidate = *m_state;
      candidate.m_window.clear();
      for ( std::size_t index = 0U;
            index < prepared.m_frame_indices.size(); ++index )
      {
        const std::uint64_t frame_index = prepared.m_frame_indices[ index ];
        const auto          source      = std::find_if(
            m_state->m_window.begin(), m_state->m_window.end(),
            [ frame_index ]( const WindowFrame& frame ) {
              return frame.m_frame_index == frame_index;
            } );
        if ( source == m_state->m_window.end() ||
             !source->m_verified_visual.has_value() )
        {
          throw std::runtime_error(
              "moving candidate selected frame is missing" );
        }
        WindowFrame frame = *source;
        if ( frame.m_visual_seed.has_value() )
        {
          frame.m_visual_seed->m_T_W_B =
              T_W_V * frame.m_visual_seed->m_T_W_B;
        }
        frame.m_verified_visual->m_T_W_B =
            T_W_V * frame.m_verified_visual->m_T_W_B;
        NavigationState navigation;
        navigation.m_T_W_B = frame.m_verified_visual->m_T_W_B;
        navigation.m_v_W_B =
            R_W_V * solution.m_candidate.m_v_W[ index ];
        navigation.m_bias.m_acc_mps2.setZero();
        navigation.m_bias.m_gyr_radps =
            solution.m_candidate.m_bg_radps;
        if ( !isFinite( navigation.m_T_W_B ) ||
             !navigation.m_v_W_B.allFinite() )
        {
          throw std::runtime_error(
              "moving materialized navigation state is non-finite" );
        }
        frame.m_navigation = navigation;
        if ( index == 0U )
        {
          frame.m_predecessor_frame_index.reset();
          frame.m_imu.reset();
        }
        else
        {
          frame.m_predecessor_frame_index =
              prepared.m_frame_indices[ index - 1U ];
          frame.m_imu = prepared.m_math.m_intervals[ index - 1U ];
        }
        candidate.m_window.push_back( std::move( frame ) );
      }
      for ( auto it = candidate.m_landmarks_w.begin();
            it != candidate.m_landmarks_w.end(); )
      {
        if ( !prepared.m_landmark_ids.contains( it->first ) )
        {
          it = candidate.m_landmarks_w.erase( it );
        }
        else
        {
          ++it;
        }
      }
      for ( auto& [ id, point_W ] : candidate.m_landmarks_w )
      {
        (void)id;
        point_W = T_W_V * point_W;
        if ( !point_W.allFinite() )
        {
          throw std::runtime_error(
              "moving materialized landmark is non-finite" );
        }
      }
      for ( auto& [ id, T_W_B ] : candidate.m_T_W_B_last_stereo )
      {
        (void)id;
        T_W_B = T_W_V * T_W_B;
        if ( !isFinite( T_W_B ) )
        {
          throw std::runtime_error(
              "moving materialized stereo pose cache is non-finite" );
        }
      }
      candidate.m_bootstrap_nodes.clear();
      candidate.m_static_candidate_ended = false;
      candidate.m_continuity_anchor.reset();
      candidate.m_initialized = false;
      return candidate;
    }

    void applyOptimizedGraphValues(
        internal::VioUpdateState& state,
        const gtsam::Values&      optimized,
        const char*               context ) const
    {
      for ( const WindowFrame& frame : state.m_window )
      {
        (void)requireNavigationState( frame, context );
        if ( !optimized.exists( X( frame.m_frame_index ) ) ||
             !optimized.exists( V( frame.m_frame_index ) ) ||
             !optimized.exists( B( frame.m_frame_index ) ) )
        {
          throw std::runtime_error(
              std::string( context ) + " omitted a navigation state" );
        }
        const Eigen::Isometry3d T_W_B = toIsometry(
            optimized.at<gtsam::Pose3>( X( frame.m_frame_index ) ) );
        const Eigen::Vector3d v_W_B =
            optimized.at<gtsam::Vector3>( V( frame.m_frame_index ) );
        const gtsam::imuBias::ConstantBias bias =
            optimized.at<gtsam::imuBias::ConstantBias>(
                B( frame.m_frame_index ) );
        if ( !isFinite( T_W_B ) || !v_W_B.allFinite() ||
             !bias.vector().allFinite() )
        {
          throw std::runtime_error(
              std::string( context ) + " state is non-finite" );
        }
      }
      for ( const auto& [ id, point_W ] : state.m_landmarks_w )
      {
        (void)point_W;
        if ( !optimized.exists( L( id ) ) )
        {
          continue;
        }
        const gtsam::Point3 point =
            optimized.at<gtsam::Point3>( L( id ) );
        if ( !Eigen::Vector3d( point.x(), point.y(), point.z() ).allFinite() )
        {
          throw std::runtime_error(
              std::string( context ) + " landmark is non-finite" );
        }
      }

      for ( WindowFrame& frame : state.m_window )
      {
        NavigationState& navigation =
            requireNavigationState( frame, context );
        navigation.m_T_W_B = toIsometry(
            optimized.at<gtsam::Pose3>( X( frame.m_frame_index ) ) );
        navigation.m_v_W_B =
            optimized.at<gtsam::Vector3>( V( frame.m_frame_index ) );
        navigation.m_bias = toImuBias(
            optimized.at<gtsam::imuBias::ConstantBias>(
                B( frame.m_frame_index ) ) );
      }
      for ( auto& [ id, point_W ] : state.m_landmarks_w )
      {
        if ( !optimized.exists( L( id ) ) )
        {
          continue;
        }
        const gtsam::Point3 point =
            optimized.at<gtsam::Point3>( L( id ) );
        point_W = Eigen::Vector3d( point.x(), point.y(), point.z() );
      }
    }

    [[nodiscard]] CurrentGraphLifecycleResult runCurrentGraphLifecycle(
        internal::VioUpdateState candidate ) const
    {
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values               values;
      VioGraphInfo                info;
      std::uint64_t               prior_key     = 0U;
      std::uint32_t               num_landmarks = 0U;
      buildGraph( candidate, graph, values, prior_key, num_landmarks,
                  nullptr, &info );
      UpdateDiagnostics probe_diagnostics;
      if ( options.enable_probe_b )
      {
        probe_diagnostics.probe_primary_graph_error_seed =
            probeGraphError( graph, values );
        probe_diagnostics.probe_primary_imu_factor_error_seed =
            probeImuFactorError( graph, values );
      }
      const double reproj_rms_before = visualReprojRms( graph, values );
      if ( !std::isfinite( reproj_rms_before ) )
      {
        throw std::runtime_error(
            "current graph initial reprojection RMS is non-finite" );
      }
#ifdef PHAD_ESTIMATOR_TESTING
      if ( m_test_next_current_graph_contract_error )
      {
        m_test_next_current_graph_contract_error = false;
        throw std::runtime_error(
            "injected current graph contract error" );
      }
      if ( m_test_next_current_graph_result.has_value() )
      {
        const InitializationReason injected =
            *m_test_next_current_graph_result;
        m_test_next_current_graph_result.reset();
        return injected;
      }
#endif
      std::unordered_map<std::uint64_t, Eigen::Vector3d> poses_before;
      for ( const WindowFrame& frame : candidate.m_window )
      {
        const NavigationState& navigation = requireNavigationState(
            frame, "current graph pose-shift baseline" );
        if ( frame.m_frame_index !=
             candidate.m_window.back().m_frame_index )
        {
          poses_before.emplace(
              frame.m_frame_index,
              navigation.m_T_W_B.translation() );
        }
      }
      constexpr std::size_t                         kProbeShiftTopK = 3U;
      std::vector<std::pair<std::uint64_t, double>> probe_shift_top;
      if ( options.enable_probe_b )
      {
        probe_shift_top.reserve( kProbeShiftTopK );
      }

      GraphSolveResult solved = solveGraph( graph, values );
      if ( const auto* solve_error =
               std::get_if<GraphSolveError>( &solved ) )
      {
        if ( solve_error->m_code == GraphSolveErrorCode::kIndeterminant )
        {
          return InitializationReason::kCurrentGraphUnderconstrained;
        }
        throw std::runtime_error(
            std::string( "current graph optimizer failed: " ) +
            solve_error->m_detail );
      }
      GraphSolveSuccess success =
          std::get<GraphSolveSuccess>( std::move( solved ) );
      gtsam::Values optimized               = std::move( success.m_values );
      std::uint32_t lm_iterations           = success.m_lm_iterations;
      double        max_window_pose_shift_m = 0.0;
      if ( options.enable_probe_b )
      {
        probe_diagnostics.probe_primary_graph_error_optimized =
            probeGraphError( graph, optimized );
        probe_diagnostics.probe_primary_imu_factor_error_optimized =
            probeImuFactorError( graph, optimized );

        const std::uint64_t current_index =
            candidate.m_window.back().m_frame_index;
        const gtsam::Pose3 seed_pose =
            values.at<gtsam::Pose3>( X( current_index ) );
        const gtsam::Pose3 optimized_pose =
            optimized.at<gtsam::Pose3>( X( current_index ) );
        const Eigen::Vector3d seed_velocity =
            values.at<gtsam::Vector3>( V( current_index ) );
        const Eigen::Vector3d optimized_velocity =
            optimized.at<gtsam::Vector3>( V( current_index ) );
        const gtsam::imuBias::ConstantBias seed_bias =
            values.at<gtsam::imuBias::ConstantBias>( B( current_index ) );
        const gtsam::imuBias::ConstantBias optimized_bias =
            optimized.at<gtsam::imuBias::ConstantBias>( B( current_index ) );

        const double pose_translation_delta =
            ( optimized_pose.translation() - seed_pose.translation() ).norm();
        const double pose_rotation_delta = Eigen::AngleAxisd(
                                               seed_pose.rotation().matrix().transpose() *
                                               optimized_pose.rotation().matrix() )
                                               .angle();
        const double velocity_delta =
            ( optimized_velocity - seed_velocity ).norm();
        const double acc_bias_delta =
            ( optimized_bias.accelerometer() - seed_bias.accelerometer() )
                .norm();
        const double gyr_bias_delta =
            ( optimized_bias.gyroscope() - seed_bias.gyroscope() ).norm();
        if ( std::isfinite( pose_translation_delta ) &&
             std::isfinite( pose_rotation_delta ) &&
             std::isfinite( velocity_delta ) &&
             std::isfinite( acc_bias_delta ) &&
             std::isfinite( gyr_bias_delta ) )
        {
          probe_diagnostics.probe_current_pose_translation_delta_m =
              pose_translation_delta;
          probe_diagnostics.probe_current_pose_rotation_delta_rad =
              pose_rotation_delta;
          probe_diagnostics.probe_current_velocity_delta_mps =
              velocity_delta;
          probe_diagnostics.probe_current_acc_bias_delta_mps2 =
              acc_bias_delta;
          probe_diagnostics.probe_current_gyr_bias_delta_radps =
              gyr_bias_delta;
        }
      }
      applyOptimizedGraphValues(
          candidate, optimized, "current graph writeback" );
      for ( const WindowFrame& frame : candidate.m_window )
      {
        const NavigationState& navigation = requireNavigationState(
            frame, "current graph writeback" );
        const auto before = poses_before.find( frame.m_frame_index );
        if ( before != poses_before.end() )
        {
          const double shift_m =
              ( navigation.m_T_W_B.translation() - before->second ).norm();
          max_window_pose_shift_m =
              std::max( max_window_pose_shift_m, shift_m );
          if ( options.enable_probe_b )
          {
            considerShiftTopK( probe_shift_top, frame.m_frame_index,
                               shift_m, kProbeShiftTopK );
          }
        }
      }
      const double reproj_rms = visualReprojRms( graph, optimized );
      if ( !std::isfinite( reproj_rms ) )
      {
        throw std::runtime_error(
            "current graph reprojection RMS is non-finite" );
      }
      std::uint32_t num_cheirality = std::max(
          countCheiralityFactors(
              graph, optimized, calibration.fxPixels() ),
          countBehindCameraLandmarks(
              candidate.m_window, candidate.m_landmarks_w,
              body_P_sensor, optimized ) );

      std::vector<LandmarkId> frame_culled;
      std::uint32_t           outliers_culled        = 0U;
      std::uint32_t           outliers_culled_unique = 0U;
      std::uint32_t           culled_round           = runCheiralityAndMeanCull(
          candidate, optimized, graph, frame_culled, outliers_culled,
          outliers_culled_unique, num_cheirality );
      double after_cull = reproj_rms;
      if ( options.enable_outlier_cull && info.m_visual_factors > 0U )
      {
        after_cull = visualReprojRmsSkippingMissingLandmarks(
            graph, optimized, candidate.m_landmarks_w );
      }

      std::uint32_t rounds       = 0U;
      bool          reopt_failed = false;
      while ( info.m_visual_factors > 0U &&
              options.enable_outlier_reopt &&
              rounds <
                  static_cast<std::uint32_t>(
                      options.max_outlier_reopts ) &&
              culled_round >= 4U )
      {
        const auto                  window_snapshot   = candidate.m_window;
        const auto                  landmark_snapshot = candidate.m_landmarks_w;
        gtsam::NonlinearFactorGraph reopt_graph;
        gtsam::Values               reopt_values;
        std::uint64_t               reopt_prior     = 0U;
        std::uint32_t               reopt_landmarks = 0U;
        VioGraphInfo                reopt_info;
        buildGraph( candidate, reopt_graph, reopt_values, reopt_prior,
                    reopt_landmarks, &optimized, &reopt_info );
        (void)reopt_prior;
        (void)reopt_landmarks;
        GraphSolveResult reopt_result =
            solveGraph( reopt_graph, reopt_values );
        if ( std::holds_alternative<GraphSolveError>( reopt_result ) )
        {
          candidate.m_window      = window_snapshot;
          candidate.m_landmarks_w = landmark_snapshot;
          reopt_failed            = true;
          break;
        }
        GraphSolveSuccess reopt_success =
            std::get<GraphSolveSuccess>( std::move( reopt_result ) );
        lm_iterations += reopt_success.m_lm_iterations;
        gtsam::Values optimized_reopt =
            std::move( reopt_success.m_values );
        try
        {
          applyOptimizedGraphValues(
              candidate, optimized_reopt,
              "current graph reoptimization writeback" );
        }
        catch ( const std::exception& )
        {
          candidate.m_window      = window_snapshot;
          candidate.m_landmarks_w = landmark_snapshot;
          reopt_failed            = true;
          break;
        }
        optimized = std::move( optimized_reopt );
        info      = reopt_info;
        ++rounds;
        after_cull   = visualReprojRms( reopt_graph, optimized );
        culled_round = runCheiralityAndMeanCull(
            candidate, optimized, reopt_graph, frame_culled,
            outliers_culled, outliers_culled_unique, num_cheirality );
      }
      if ( !std::isfinite( after_cull ) )
      {
        throw std::runtime_error(
            "current graph post-cull reprojection RMS is non-finite" );
      }
      const Eigen::Isometry3d& latest_pose = requireNavigationState(
                                                 candidate.m_window.back(),
                                                 "last-stereo committed-pose update" )
                                                 .m_T_W_B;
      for ( const StereoObservation& observation :
            candidate.m_window.back().m_observations )
      {
        if ( observation.disparity_px > 0.0 )
        {
          candidate.m_T_W_B_last_stereo[ observation.id ] =
              latest_pose;
        }
      }
      candidate.m_initialized = true;
      if ( options.enable_probe_b )
      {
        std::sort(
            probe_shift_top.begin(), probe_shift_top.end(),
            []( const std::pair<std::uint64_t, double>& lhs,
                const std::pair<std::uint64_t, double>& rhs ) {
              return lhs.second > rhs.second;
            } );
        gtsam::NonlinearFactorGraph probe_graph;
        gtsam::Values               probe_values;
        std::uint64_t               probe_prior_key = 0U;
        std::uint32_t               probe_landmarks = 0U;
        buildGraph( candidate, probe_graph, probe_values,
                    probe_prior_key, probe_landmarks );
        (void)probe_prior_key;
        (void)probe_landmarks;
        fillProbeLandmarkResiduals(
            probe_graph, probe_values, probe_diagnostics );
      }
      return CurrentGraphValidation{
          .m_candidate               = std::move( candidate ),
          .m_info                    = info,
          .m_prior_key               = prior_key,
          .m_num_landmarks           = num_landmarks,
          .m_lm_iterations           = lm_iterations,
          .m_reproj_rms_before       = reproj_rms_before,
          .m_reproj_rms              = reproj_rms,
          .m_reproj_rms_after_cull   = after_cull,
          .m_max_window_pose_shift_m = max_window_pose_shift_m,
          .m_num_cheirality          = num_cheirality,
          .m_outliers_culled         = outliers_culled,
          .m_outliers_culled_unique  = outliers_culled_unique,
          .m_outlier_reopt_rounds    = rounds,
          .m_outlier_reopt_failed    = reopt_failed,
          .m_culled_landmark_ids     = std::move( frame_culled ),
          .m_probe_shift_top         = std::move( probe_shift_top ),
          .m_probe_res_mean_px       = probe_diagnostics.probe_res_mean_px,
          .m_probe_res_max_px        = probe_diagnostics.probe_res_max_px,
          .m_probe_res_max_id        = probe_diagnostics.probe_res_max_id,
          .m_probe_detail_valid      = probe_diagnostics.probe_detail_valid,
          .m_probe_current_pose_translation_delta_m =
              probe_diagnostics.probe_current_pose_translation_delta_m,
          .m_probe_current_pose_rotation_delta_rad =
              probe_diagnostics.probe_current_pose_rotation_delta_rad,
          .m_probe_current_velocity_delta_mps =
              probe_diagnostics.probe_current_velocity_delta_mps,
          .m_probe_current_acc_bias_delta_mps2 =
              probe_diagnostics.probe_current_acc_bias_delta_mps2,
          .m_probe_current_gyr_bias_delta_radps =
              probe_diagnostics.probe_current_gyr_bias_delta_radps,
          .m_probe_primary_graph_error_seed =
              probe_diagnostics.probe_primary_graph_error_seed,
          .m_probe_primary_graph_error_optimized =
              probe_diagnostics.probe_primary_graph_error_optimized,
          .m_probe_primary_imu_factor_error_seed =
              probe_diagnostics.probe_primary_imu_factor_error_seed,
          .m_probe_primary_imu_factor_error_optimized =
              probe_diagnostics.probe_primary_imu_factor_error_optimized };
    }

    void buildGraph( const internal::VioUpdateState& graph_state,
                     gtsam::NonlinearFactorGraph&    graph,
                     gtsam::Values&                  values,
                     std::uint64_t&                  prior_key_out,
                     std::uint32_t&                  num_landmarks_out,
                     const gtsam::Values*            round_start  = nullptr,
                     VioGraphInfo*                   vio_info_out = nullptr ) const
    {
      graph.resize( 0 );
      values.clear();
      num_landmarks_out = 0;
      VioGraphInfo vio_info;
      if ( graph_state.m_window.empty() )
      {
        prior_key_out = 0;
        if ( vio_info_out != nullptr )
        {
          *vio_info_out = vio_info;
        }
        return;
      }

      prior_key_out = graph_state.m_window.front().m_frame_index;
      for ( const WindowFrame& frame : graph_state.m_window )
      {
        const NavigationState& navigation = requireNavigationState(
            frame, "current graph construction" );
        const gtsam::Key   pose_key     = X( frame.m_frame_index );
        const gtsam::Key   velocity_key = V( frame.m_frame_index );
        const gtsam::Key   bias_key     = B( frame.m_frame_index );
        const gtsam::Pose3 pose =
            round_start != nullptr && round_start->exists( pose_key )
                ? round_start->at<gtsam::Pose3>( pose_key )
                : toPose3( navigation.m_T_W_B );
        const Eigen::Vector3d velocity =
            round_start != nullptr && round_start->exists( velocity_key )
                ? round_start->at<gtsam::Vector3>( velocity_key )
                : navigation.m_v_W_B;
        const gtsam::imuBias::ConstantBias bias =
            round_start != nullptr && round_start->exists( bias_key )
                ? round_start->at<gtsam::imuBias::ConstantBias>( bias_key )
                : toGtsamBias( navigation.m_bias );
        if ( !velocity.allFinite() || !bias.vector().allFinite() )
        {
          throw std::runtime_error(
              "navigation state initial value is non-finite" );
        }
        values.insert( pose_key, pose );
        values.insert( velocity_key, velocity );
        values.insert( bias_key, bias );
        ++vio_info.m_nav_states;
      }

      for ( const WindowFrame& frame : graph_state.m_window )
      {
        const NavigationState& navigation = requireNavigationState(
            frame, "current graph factor construction" );
        if ( !frame.m_predecessor_frame_index.has_value() )
        {
          graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
              X( frame.m_frame_index ), toPose3( navigation.m_T_W_B ),
              prior_noise );
          graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
              V( frame.m_frame_index ), navigation.m_v_W_B,
              velocity_prior_noise );
          graph.emplace_shared<
              gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
              B( frame.m_frame_index ), toGtsamBias( navigation.m_bias ),
              bias_prior_noise );
          ++vio_info.m_root_prior_sets;
          continue;
        }
        if ( !frame.m_imu.has_value() )
        {
          throw std::runtime_error(
              "linked navigation state is missing its IMU interval" );
        }
        const std::uint64_t predecessor =
            *frame.m_predecessor_frame_index;
        const auto predecessor_it = std::find_if(
            graph_state.m_window.begin(), graph_state.m_window.end(),
            [ predecessor ]( const WindowFrame& candidate ) {
              return candidate.m_frame_index == predecessor;
            } );
        if ( predecessor_it == graph_state.m_window.end() )
        {
          throw std::runtime_error(
              "linked navigation predecessor is outside the window" );
        }

        const ImuBias bias_hat = toImuBias(
            values.at<gtsam::imuBias::ConstantBias>( B( predecessor ) ) );
        const auto pim = preintegrate( *frame.m_imu, bias_hat );
        graph.emplace_shared<gtsam::ImuFactor>(
            X( predecessor ), V( predecessor ), X( frame.m_frame_index ),
            V( frame.m_frame_index ), B( predecessor ), pim );

        const double dt_s =
            static_cast<double>( frame.m_imu->m_duration_ns ) * 1e-9;
        const double   sqrt_dt_s = std::sqrt( dt_s );
        gtsam::Vector6 rw_sigmas;
        rw_sigmas << imu.accRw() * sqrt_dt_s,
            imu.accRw() * sqrt_dt_s, imu.accRw() * sqrt_dt_s,
            imu.gyrRw() * sqrt_dt_s, imu.gyrRw() * sqrt_dt_s,
            imu.gyrRw() * sqrt_dt_s;
        if ( !rw_sigmas.allFinite() || ( rw_sigmas.array() <= 0.0 ).any() )
        {
          throw std::runtime_error( "bias random-walk sigma is invalid" );
        }
        graph.emplace_shared<
            gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            B( predecessor ), B( frame.m_frame_index ),
            gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Diagonal::Sigmas( rw_sigmas ) );
        ++vio_info.m_imu_factors;
        ++vio_info.m_bias_rw_factors;
        vio_info.m_integration_steps += static_cast<std::uint32_t>(
            frame.m_imu->m_steps.size() );
        if ( frame.m_imu->m_duration_ns >
             std::numeric_limits<std::int64_t>::max() -
                 vio_info.m_integrated_duration_ns )
        {
          throw std::runtime_error(
              "integrated IMU duration overflows int64 diagnostics" );
        }
        vio_info.m_integrated_duration_ns += frame.m_imu->m_duration_ns;
        vio_info.m_last_bias_rw_sigmas = rw_sigmas;
      }

      const auto                     counts = countObservations( graph_state );
      std::unordered_set<LandmarkId> inserted;
      for ( const WindowFrame& frame : graph_state.m_window )
      {
        for ( const StereoObservation& observation : frame.m_observations )
        {
          const auto count_it = counts.find( observation.id );
          if ( count_it == counts.end() ||
               count_it->second < options.min_landmark_observations )
          {
            continue;
          }
          const auto landmark_it =
              graph_state.m_landmarks_w.find( observation.id );
          if ( landmark_it == graph_state.m_landmarks_w.end() )
          {
            continue;
          }
          if ( inserted.insert( observation.id ).second )
          {
            const gtsam::Key    landmark_key = L( observation.id );
            const gtsam::Point3 point =
                round_start != nullptr && round_start->exists( landmark_key )
                    ? round_start->at<gtsam::Point3>( landmark_key )
                    : gtsam::Point3( landmark_it->second.x(),
                                     landmark_it->second.y(),
                                     landmark_it->second.z() );
            values.insert(
                landmark_key, point );
            ++num_landmarks_out;
          }
          if ( observation.disparity_px > 0.0 )
          {
            graph.emplace_shared<
                gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
                toStereoPoint( observation ), stereo_noise,
                X( frame.m_frame_index ), L( observation.id ), K,
                body_P_sensor );
          }
          else
          {
            graph.emplace_shared<gtsam::GenericProjectionFactor<
                gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                toMonoPoint( observation ), mono_noise,
                X( frame.m_frame_index ), L( observation.id ), K_mono,
                body_P_sensor );
            if ( frame.m_frame_index ==
                 graph_state.m_window.back().m_frame_index )
            {
              ++vio_info.m_current_mono_visual_factors;
            }
          }
          ++vio_info.m_visual_factors;
          if ( frame.m_frame_index ==
               graph_state.m_window.back().m_frame_index )
          {
            ++vio_info.m_current_visual_factors;
          }
        }
      }
      if ( vio_info_out != nullptr )
      {
        *vio_info_out = vio_info;
      }
    }

    [[nodiscard]] GraphSolveResult solveGraph(
        const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values&               values ) const
    {
      try
      {
        gtsam::LevenbergMarquardtOptimizer optimizer( graph, values );
        gtsam::Values                      optimized = optimizer.optimize();
        return GraphSolveSuccess{
            std::move( optimized ),
            static_cast<std::uint32_t>( optimizer.iterations() ) };
      }
      catch ( const gtsam::IndeterminantLinearSystemException& exception )
      {
        return GraphSolveError{ GraphSolveErrorCode::kIndeterminant,
                                exception.what(), 0U };
      }
      catch ( const std::exception& exception )
      {
        return GraphSolveError{ GraphSolveErrorCode::kOptimizer,
                                exception.what(), 0U };
      }
    }

    [[nodiscard]] VioDiagnostics makeVioDiagnostics(
        const VioGraphInfo&             info,
        const internal::VioUpdateState& graph_state ) const
    {
      VioDiagnostics diagnostics;
      diagnostics.m_nav_states             = info.m_nav_states;
      diagnostics.m_imu_factors            = info.m_imu_factors;
      diagnostics.m_bias_rw_factors        = info.m_bias_rw_factors;
      diagnostics.m_visual_factors         = info.m_visual_factors;
      diagnostics.m_root_prior_sets        = info.m_root_prior_sets;
      diagnostics.m_integration_steps      = info.m_integration_steps;
      diagnostics.m_integrated_duration_ns = info.m_integrated_duration_ns;
      diagnostics.m_visual_coast_duration_ns =
          graph_state.m_visual_coast_duration_ns;
      diagnostics.m_non_keyframe_evictions =
          graph_state.m_non_keyframe_evictions;
      diagnostics.m_imu_reintegrations =
          graph_state.m_imu_reintegrations;
      diagnostics.m_acc_cov_diag =
          pim_params->accelerometerCovariance.diagonal();
      diagnostics.m_gyr_cov_diag =
          pim_params->gyroscopeCovariance.diagonal();
      diagnostics.m_integration_cov_diag =
          pim_params->integrationCovariance.diagonal();
      diagnostics.m_bias_rw_sigmas = info.m_last_bias_rw_sigmas;
      return diagnostics;
    }

    [[nodiscard]] std::uint32_t dropCheiralityLandmarks(
        internal::VioUpdateState& graph_state,
        const gtsam::Values&      values,
        std::vector<LandmarkId>&  frame_culled ) const
    {
      std::uint32_t           dropped = 0;
      std::vector<LandmarkId> to_drop;
      for ( const auto& [ id, point_W ] : graph_state.m_landmarks_w )
      {
        const gtsam::Key key = L( id );
        if ( !values.exists( key ) )
        {
          continue;
        }
        const gtsam::Point3 optimized = values.at<gtsam::Point3>( key );
        bool                behind    = false;
        for ( const WindowFrame& frame : graph_state.m_window )
        {
          bool observes = false;
          for ( const StereoObservation& observation : frame.m_observations )
          {
            if ( observation.id == id )
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
              values.exists( X( frame.m_frame_index ) )
                  ? values.at<gtsam::Pose3>( X( frame.m_frame_index ) )
                  : toPose3( requireNavigationState(
                                 frame, "cheirality cull" )
                                 .m_T_W_B );
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
        graph_state.m_landmarks_w.erase( id );
        eraseLandmarkFromWindow( graph_state, id );
        graph_state.m_culled_ids.insert( id );
        frame_culled.push_back( id );
        ++dropped;
      }
      return dropped;
    }

    // Returns mean-cull count this pass. Cheirality ids are appended to
    // frame_culled but do not count toward the reopt trigger.
    [[nodiscard]] std::uint32_t runCheiralityAndMeanCull(
        internal::VioUpdateState&          graph_state,
        const gtsam::Values&               optimized,
        const gtsam::NonlinearFactorGraph& graph_for_scoring,
        std::vector<LandmarkId>&           frame_culled,
        std::uint32_t&                     outliers_culled_total,
        std::uint32_t&                     outliers_culled_unique_total,
        std::uint32_t&                     num_cheirality_inout ) const
    {
      const std::uint32_t cheirality_after = countCheiralityFactors(
          graph_for_scoring, optimized, calibration.fxPixels() );
      const std::uint32_t dropped =
          dropCheiralityLandmarks(
              graph_state, optimized, frame_culled );
      num_cheirality_inout = std::max(
          num_cheirality_inout, std::max( cheirality_after, dropped ) );

      std::uint32_t culled_round = 0;
      if ( !options.enable_outlier_cull )
      {
        return culled_round;
      }

      std::vector<LandmarkId> candidate_ids;
      candidate_ids.reserve( graph_state.m_landmarks_w.size() );
      for ( const auto& [ id, _unused ] : graph_state.m_landmarks_w )
      {
        candidate_ids.push_back( id );
      }
      for ( const LandmarkId id : candidate_ids )
      {
        double      sum_norm  = 0.0;
        std::size_t n_factors = 0;
        for ( const auto& factor : graph_for_scoring )
        {
          const std::optional<VisualFactorView> visual =
              asVisualFactor( factor );
          if ( !visual.has_value() )
          {
            continue;
          }
          if ( visual->m_landmark_key != L( id ) )
          {
            continue;
          }
          sum_norm += visual->unwhitenedError( optimized ).norm();
          ++n_factors;
        }
        if ( n_factors < 4 )
        {
          continue;
        }
        const double score = sum_norm / static_cast<double>( n_factors );
        if ( score > options.outlier_avg_reproj_px )
        {
          graph_state.m_landmarks_w.erase( id );
          eraseLandmarkFromWindow( graph_state, id );
          frame_culled.push_back( id );
          ++culled_round;
          ++outliers_culled_total;
          if ( graph_state.m_culled_ids.insert( id ).second )
          {
            ++outliers_culled_unique_total;
          }
        }
      }
      return culled_round;
    }
  };

  VioEstimator::VioEstimator(
      camera::RectifiedStereoCalibration calibration,
      sensor::ImuParameters              imu,
      EstimatorOptions                   options )
      : m_impl( std::make_unique<Impl>( std::move( calibration ),
                                        std::move( imu ),
                                        std::move( options ) ) )
  {
  }

  VioEstimator::~VioEstimator() = default;

  VioEstimator::VioEstimator( VioEstimator&& ) noexcept =
      default;

  VioEstimator& VioEstimator::operator=( VioEstimator&& ) noexcept = default;

  std::vector<common::Timestamp> VioEstimator::observationTimestamps(
      LandmarkId id ) const
  {
    const auto it = m_impl->m_state->m_track_times.find( id );
    if ( it == m_impl->m_state->m_track_times.end() )
    {
      return {};
    }
    return it->second;
  }

  VioUpdateResult VioEstimator::update( const VioMeasurement& measurement,
                                        const bool            keyframe )
  {
    VioUpdateResult result;
    result.diagnostics.culled_landmark_ids.clear();
    result.diagnostics.num_observations =
        static_cast<std::uint32_t>( measurement.m_observations.size() );
    result.diagnostics.segment_id = m_impl->m_state->m_segment_id;
    result.diagnostics.window_size =
        static_cast<std::uint32_t>( m_impl->m_state->m_window.size() );
    result.diagnostics.prior_key = m_impl->m_state->m_window.empty()
                                       ? 0U
                                       : m_impl->m_state->m_window.front()
                                             .m_frame_index;
    result.diagnostics.m_vio     = m_impl->m_state->m_vio_diagnostics;
    result.diagnostics.unsupported_span_ns =
        m_impl->m_state->m_unsupported_span_ns;
    const auto finalizePreStagingHardResult =
        [ & ]( UpdateStatus status, std::string message ) -> VioUpdateResult {
      result.status  = status;
      result.message = std::move( message );
      return result;
    };
    const auto stageUnsupportedSpanForCurrentTimestamp = [ & ]() -> bool {
      if ( !m_impl->m_state->m_last_visual_support_timestamp.has_value() )
      {
        m_impl->m_state->m_unsupported_span_ns = 0;
        result.diagnostics.unsupported_span_ns = 0;
        return true;
      }
      const std::optional<std::int64_t> unsupported_span_ns =
          checkedPositiveDurationNs(
              *m_impl->m_state->m_last_visual_support_timestamp,
              measurement.m_timestamp );
      if ( !unsupported_span_ns.has_value() )
      {
        return false;
      }
      m_impl->m_state->m_unsupported_span_ns = *unsupported_span_ns;
      result.diagnostics.unsupported_span_ns = *unsupported_span_ns;
      return true;
    };

    const std::optional<common::Timestamp> expected_t_begin =
        m_impl->m_state->m_initialized
            ? std::optional<common::Timestamp>(
                  m_impl->m_state->m_window.back().m_timestamp )
            : m_impl->m_state->m_continuity_anchor;

    if ( std::holds_alternative<sensor::MeasurementDiscontinuity>(
             measurement.m_imu ) )
    {
      const sensor::MeasurementDiscontinuity& discontinuity =
          std::get<sensor::MeasurementDiscontinuity>( measurement.m_imu );
      if ( discontinuity.m_t_begin >= discontinuity.m_t_end ||
           discontinuity.m_t_end != measurement.m_timestamp ||
           ( expected_t_begin.has_value() &&
             discontinuity.m_t_begin != *expected_t_begin ) )
      {
        return finalizePreStagingHardResult(
            UpdateStatus::kInvalidInput,
            "measurement discontinuity endpoints do not match the continuity anchor" );
      }

      VioUpdateTransaction transaction( m_impl->m_state );
      m_impl->m_state->m_bootstrap_nodes.clear();
      m_impl->m_state->m_static_candidate_ended           = false;
      m_impl->m_state->m_evidence_revision                = 0U;
      m_impl->m_state->m_last_attempted_evidence_revision = 0U;
      m_impl->m_state->m_visual_coast_duration_ns         = 0;
      m_impl->m_state->m_continuity_anchor                = discontinuity.m_t_end;
      m_impl->m_state->m_last_visual_support_timestamp.reset();
      m_impl->m_state->m_unsupported_span_ns = 0;
      result.diagnostics.unsupported_span_ns = 0;
      if ( !m_impl->m_state->m_initialized )
      {
        result.status  = UpdateStatus::kInitializing;
        result.message = "measurement discontinuity reset bootstrap evidence";
        transaction.commit();
        return result;
      }

      const std::uint32_t completed_segment =
          m_impl->completeActiveSegment( discontinuity.m_t_end );
      result.diagnostics.segment_id                   = m_impl->m_state->m_segment_id;
      result.diagnostics.window_size                  = 0;
      result.diagnostics.prior_key                    = 0;
      result.diagnostics.m_vio                        = m_impl->m_state->m_vio_diagnostics;
      result.diagnostics.m_vio.m_completed_segment_id = completed_segment;
      result.status                                   = UpdateStatus::kDiscontinuity;
      result.message                                  = "measurement discontinuity completed the active segment";
      transaction.commit();
      return result;
    }

    const sensor::RawImuInterval& raw =
        std::get<sensor::RawImuInterval>( measurement.m_imu );
    ImuIntervalResult normalized_result = normalizeRawImuInterval(
        raw, expected_t_begin, measurement.m_timestamp );
    if ( std::holds_alternative<ImuIntervalError>( normalized_result ) )
    {
      return finalizePreStagingHardResult(
          UpdateStatus::kInvalidInput,
          std::get<ImuIntervalError>( normalized_result ).m_detail );
    }
    NormalizedImuInterval normalized_imu =
        std::get<NormalizedImuInterval>( std::move( normalized_result ) );

    // disparity_px == 0 is legal and represents unavailable stereo depth.
    std::unordered_set<LandmarkId> packet_landmark_ids;
    std::uint32_t                  current_num_disparity = 0;
    for ( const StereoObservation& observation : measurement.m_observations )
    {
      if ( observation.disparity_px < 0.0 ||
           !observation.left_pixel.allFinite() ||
           !std::isfinite( observation.disparity_px ) )
      {
        return finalizePreStagingHardResult(
            UpdateStatus::kInvalidInput,
            "non-finite pixel or negative disparity" );
      }
      if ( !packet_landmark_ids.insert( observation.id ).second )
      {
        return finalizePreStagingHardResult(
            UpdateStatus::kInvalidInput,
            "duplicate landmark id in one measurement" );
      }
      if ( observation.disparity_px > 0.0 )
      {
        ++current_num_disparity;
      }
    }
    result.diagnostics.num_disparity = current_num_disparity;

    std::vector<sensor::ImuMeasurement> bootstrap_nodes;
    BootstrapStats                      bootstrap_stats;
    Eigen::Isometry3d                   bootstrap_T_W_B =
        Eigen::Isometry3d::Identity();
    ImuBias bootstrap_bias;
    bool    static_root_selected = false;
    if ( !m_impl->m_state->m_initialized )
    {
      std::optional<std::int64_t> bootstrap_duration_ns;
      bool                        static_ready = false;
      if ( !m_impl->m_state->m_static_candidate_ended )
      {
        bootstrap_nodes = m_impl->m_state->m_bootstrap_nodes;
        for ( const sensor::ImuMeasurement& node : normalized_imu.m_nodes )
        {
          if ( !bootstrap_nodes.empty() &&
               node.timestamp == bootstrap_nodes.back().timestamp )
          {
            if ( !sameImuValue( node, bootstrap_nodes.back() ) )
            {
              return finalizePreStagingHardResult(
                  UpdateStatus::kInvalidInput,
                  "shared bootstrap endpoint has inconsistent IMU values" );
            }
            continue;
          }
          bootstrap_nodes.push_back( node );
        }

        bootstrap_duration_ns = checkedPositiveDurationNs(
            bootstrap_nodes.front().timestamp,
            bootstrap_nodes.back().timestamp );
        if ( !bootstrap_duration_ns.has_value() )
        {
          return finalizePreStagingHardResult(
              UpdateStatus::kInvalidInput,
              "static bootstrap timestamp span is not representable" );
        }
        bootstrap_stats = bootstrapStats( bootstrap_nodes );
        const bool stationary =
            bootstrap_stats.m_acc_std.maxCoeff() <=
                m_impl->options.m_bootstrap_max_acc_std_mps2 &&
            bootstrap_stats.m_gyr_std.maxCoeff() <=
                m_impl->options.m_bootstrap_max_gyr_std_radps &&
            std::abs( bootstrap_stats.m_acc_mean.norm() -
                      m_impl->options.m_gravity_mps2 ) <=
                m_impl->options.m_bootstrap_acc_norm_tol_mps2;
        static_ready =
            *bootstrap_duration_ns >=
                m_impl->options.m_bootstrap_min_duration_ns &&
            bootstrap_nodes.size() >=
                m_impl->options.m_bootstrap_min_samples &&
            stationary;
      }
      if ( m_impl->options.m_enable_moving_bootstrap )
      {
        VioUpdateTransaction transaction( m_impl->m_state );
        m_impl->m_state->m_continuity_anchor = measurement.m_timestamp;
        if ( !static_ready && bootstrap_duration_ns.has_value() &&
             *bootstrap_duration_ns >=
                 m_impl->options.m_bootstrap_timeout_ns )
        {
          m_impl->m_state->m_static_candidate_ended = true;
          m_impl->m_state->m_bootstrap_nodes.clear();
          bootstrap_nodes.clear();
        }
        else if ( !m_impl->m_state->m_static_candidate_ended )
        {
          m_impl->m_state->m_bootstrap_nodes = bootstrap_nodes;
        }
        Impl::MovingEvidenceSummary evidence;
        try
        {
          evidence = m_impl->stageMovingVisualEvidence(
              measurement, normalized_imu, keyframe );
        }
        catch ( const std::exception& exception )
        {
          InitializationDiagnostics initialization;
          initialization.m_path    = InitializationPath::kMoving;
          initialization.m_phase   = InitializationPhase::kVisualSolve;
          initialization.m_reason  = InitializationReason::kContractError;
          initialization.m_outcome = InitializationOutcome::kFatalError;
          initialization.m_transaction =
              InitializationTransactionResult::kRolledBack;
          transaction.rollback();
          return internal::finalizeInitializationResult(
              std::move( result ), initialization, std::nullopt,
              std::string( "moving visual evidence failed: " ) +
                  exception.what() );
        }
        result.diagnostics.m_vio =
            m_impl->makeVioDiagnostics(
                VioGraphInfo{}, *m_impl->m_state );
        m_impl->m_state->m_vio_diagnostics = result.diagnostics.m_vio;
        if ( evidence.m_reason ==
                 InitializationReason::kPoseCountInsufficient &&
             evidence.m_connected_poses >= 6U &&
             evidence.m_duration_ns < 250'000'000 )
        {
          evidence.m_reason = InitializationReason::kDurationInsufficient;
        }
        if ( !stageUnsupportedSpanForCurrentTimestamp() )
        {
          return finalizePreStagingHardResult(
              UpdateStatus::kFailed,
              "unsupported visual span is not representable" );
        }
        const auto makeMovingInitialization =
            [ & ]( const InitializationPhase             phase,
                   const InitializationReason            reason,
                   const InitializationOutcome           outcome,
                   const InitializationTransactionResult transaction_result ) {
              InitializationDiagnostics initialization;
              initialization.m_path    = InitializationPath::kMoving;
              initialization.m_phase   = phase;
              initialization.m_reason  = reason;
              initialization.m_outcome = outcome;
              initialization.m_connected_poses.m_actual =
                  evidence.m_connected_poses;
              initialization.m_connected_poses.m_required = 6U;
              initialization.m_duration.m_actual_ns =
                  evidence.m_duration_ns;
              initialization.m_duration.m_required_ns = 250'000'000;
              initialization.m_connected_landmarks.m_actual =
                  evidence.m_connected_landmarks;
              initialization.m_connected_landmarks.m_required =
                  static_cast<std::uint32_t>(
                      m_impl->options.min_seed_observations );
              if ( evidence.m_visual_support.has_value() )
              {
                initialization.m_visual_support =
                    InitializationCountSummary{
                        .m_actual   = *evidence.m_visual_support,
                        .m_required = static_cast<std::uint32_t>(
                            m_impl->options.min_pnp_inliers ) };
              }
              initialization.m_visual_fit  = evidence.m_visual_fit;
              initialization.m_transaction = transaction_result;
              return initialization;
            };

        const bool eligible =
            evidence.m_reason ==
                InitializationReason::kPoseCountInsufficient &&
            evidence.m_connected_poses >= 6U &&
            evidence.m_duration_ns >= 250'000'000;
        if ( !static_ready && eligible &&
             m_impl->m_state->m_evidence_revision !=
                 m_impl->m_state->m_last_attempted_evidence_revision )
        {
          if ( m_impl->m_next_attempt_id ==
               std::numeric_limits<std::uint64_t>::max() )
          {
            InitializationDiagnostics fatal = makeMovingInitialization(
                InitializationPhase::kCollectingEvidence,
                InitializationReason::kContractError,
                InitializationOutcome::kFatalError,
                InitializationTransactionResult::kRolledBack );
            transaction.rollback();
            return internal::finalizeInitializationResult(
                std::move( result ), fatal, std::nullopt,
                "moving attempt id overflows uint64" );
          }
          const std::uint64_t attempt_id = m_impl->m_next_attempt_id++;
          m_impl->m_state->m_last_attempted_evidence_revision =
              m_impl->m_state->m_evidence_revision;

          const auto fatalAttempt =
              [ & ]( const InitializationPhase phase,
                     std::string               message ) -> VioUpdateResult {
            InitializationDiagnostics fatal = makeMovingInitialization(
                phase, InitializationReason::kContractError,
                InitializationOutcome::kFatalError,
                InitializationTransactionResult::kRolledBack );
            fatal.m_attempt_id = attempt_id;
            transaction.rollback();
            result.diagnostics.segment_id =
                m_impl->m_state->m_segment_id;
            result.diagnostics.window_size =
                static_cast<std::uint32_t>(
                    m_impl->m_state->m_window.size() );
            result.diagnostics.prior_key =
                m_impl->m_state->m_window.empty()
                    ? 0U
                    : m_impl->m_state->m_window.front().m_frame_index;
            result.diagnostics.m_vio =
                m_impl->m_state->m_vio_diagnostics;
            return internal::finalizeInitializationResult(
                std::move( result ), fatal, std::nullopt,
                std::move( message ) );
          };

          Impl::PreparedMovingCandidate prepared;
          try
          {
            prepared = m_impl->prepareMovingCandidate();
          }
          catch ( const std::exception& exception )
          {
            return fatalAttempt(
                InitializationPhase::kJointConsistency,
                std::string( "moving candidate preparation failed: " ) +
                    exception.what() );
          }
          InitialMovingCandidateResult candidate_result = [ & ]()
              -> InitialMovingCandidateResult {
#ifdef PHAD_ESTIMATOR_TESTING
            if ( m_impl->m_test_next_moving_candidate_result.has_value() )
            {
              InitialMovingCandidateResult injected = std::move(
                  *m_impl->m_test_next_moving_candidate_result );
              m_impl->m_test_next_moving_candidate_result.reset();
              return injected;
            }
#endif
            return solveInitialMovingCandidate(
                prepared.m_math,
                prepared.m_visual_pose_covariance );
          }();
          if ( const auto* math_error =
                   std::get_if<InitialMovingMathError>(
                       &candidate_result ) )
          {
            return fatalAttempt(
                InitializationPhase::kJointConsistency,
                std::string( "moving candidate math failed: " ) +
                    math_error->m_detail );
          }
          if ( const auto* failure =
                   std::get_if<InitialMovingCandidateFailure>(
                       &candidate_result ) )
          {
            InitializationReason reason = InitializationReason::kRankDeficient;
            InitializationPhase  phase  = InitializationPhase::kGyroAlignment;
            switch ( failure->m_code )
            {
              case internal::InitialMovingCandidateFailureCode::kRankDeficient:
                reason = InitializationReason::kRankDeficient;
                break;
              case internal::InitialMovingCandidateFailureCode::kIllConditioned:
                reason = InitializationReason::kIllConditioned;
                break;
              case internal::InitialMovingCandidateFailureCode::kNoConvergence:
                reason = InitializationReason::kNoConvergence;
                phase  = InitializationPhase::kJointConsistency;
                break;
              case internal::InitialMovingCandidateFailureCode::
                  kJointDofInsufficient:
                reason = InitializationReason::kJointDofInsufficient;
                phase  = InitializationPhase::kJointConsistency;
                break;
              case internal::InitialMovingCandidateFailureCode::
                  kJointInconsistent:
                reason = InitializationReason::kJointInconsistent;
                phase  = InitializationPhase::kJointConsistency;
                break;
            }
            InitializationDiagnostics rejected = makeMovingInitialization(
                phase, reason,
                InitializationOutcome::kRecoverableRejected,
                InitializationTransactionResult::kEvidenceCommitted );
            rejected.m_attempt_id = attempt_id;
            if ( failure->m_linear_failure.has_value() )
            {
              const auto&               linear = *failure->m_linear_failure;
              InitializationRankSummary rank{
                  .m_actual   = linear.m_rank,
                  .m_required = linear.m_required_rank };
              if ( linear.m_required_rank == 3U )
              {
                rejected.m_gyro_rank = rank;
                phase                = InitializationPhase::kGyroAlignment;
                if ( linear.m_normalized_condition.has_value() )
                {
                  rejected.m_gyro_normalized_condition =
                      InitializationThresholdSummary{
                          .m_value = *linear.m_normalized_condition,
                          .m_limit =
                              internal::kInitialMovingNormalizedConditionLimit };
                }
              }
              else
              {
                rejected.m_phase =
                    InitializationPhase::kGravityVelocitySolve;
                rejected.m_gravity_velocity_rank = rank;
                if ( linear.m_normalized_condition.has_value() )
                {
                  rejected.m_gravity_velocity_normalized_condition =
                      InitializationThresholdSummary{
                          .m_value = *linear.m_normalized_condition,
                          .m_limit =
                              internal::kInitialMovingNormalizedConditionLimit };
                }
              }
            }
            if ( failure->m_joint_fit.has_value() )
            {
              rejected.m_joint_fit = InitializationFitSummary{
                  .m_statistic = failure->m_joint_fit->m_statistic,
                  .m_limit     = failure->m_joint_limit.value_or( 0.0 ),
                  .m_effective_dof =
                      failure->m_joint_fit->m_effective_dof };
            }
            if ( failure->m_gyro.has_value() )
            {
              rejected.m_gyro_rank = InitializationRankSummary{
                  .m_actual   = failure->m_gyro->m_rank,
                  .m_required = 3U };
              if ( failure->m_gyro->m_normalized_condition.has_value() )
              {
                rejected.m_gyro_normalized_condition =
                    InitializationThresholdSummary{
                        .m_value = *failure->m_gyro
                                        ->m_normalized_condition,
                        .m_limit = internal::
                            kInitialMovingNormalizedConditionLimit };
              }
            }
            if ( failure->m_gravity_velocity.has_value() )
            {
              rejected.m_gravity_velocity_rank =
                  InitializationRankSummary{
                      .m_actual =
                          failure->m_gravity_velocity->m_rank,
                      .m_required = static_cast<std::uint32_t>(
                          3U * prepared.m_frame_indices.size() + 2U ) };
              if ( failure->m_gravity_velocity
                       ->m_normalized_condition.has_value() )
              {
                rejected.m_gravity_velocity_normalized_condition =
                    InitializationThresholdSummary{
                        .m_value = *failure->m_gravity_velocity
                                        ->m_normalized_condition,
                        .m_limit = internal::
                            kInitialMovingNormalizedConditionLimit };
              }
            }
            transaction.commit();
            return internal::finalizeInitializationResult(
                std::move( result ), rejected, std::nullopt,
                "moving initialization candidate was rejected" );
          }

          InitialMovingCandidateSolution candidate =
              std::get<InitialMovingCandidateSolution>(
                  std::move( candidate_result ) );
          const auto attachCandidateSummaries =
              [ & ]( InitializationDiagnostics& diagnostics ) {
                diagnostics.m_gyro_rank = InitializationRankSummary{
                    .m_actual   = candidate.m_gyro.m_rank,
                    .m_required = 3U };
                if ( candidate.m_gyro.m_normalized_condition.has_value() )
                {
                  diagnostics.m_gyro_normalized_condition =
                      InitializationThresholdSummary{
                          .m_value =
                              *candidate.m_gyro.m_normalized_condition,
                          .m_limit = internal::
                              kInitialMovingNormalizedConditionLimit };
                }
                diagnostics.m_gravity_velocity_rank =
                    InitializationRankSummary{
                        .m_actual   = candidate.m_gravity_velocity.m_rank,
                        .m_required = static_cast<std::uint32_t>(
                            3U * prepared.m_frame_indices.size() + 2U ) };
                if ( candidate.m_gravity_velocity
                         .m_normalized_condition.has_value() )
                {
                  diagnostics.m_gravity_velocity_normalized_condition =
                      InitializationThresholdSummary{
                          .m_value = *candidate.m_gravity_velocity
                                          .m_normalized_condition,
                          .m_limit = internal::
                              kInitialMovingNormalizedConditionLimit };
                }
                diagnostics.m_joint_fit = InitializationFitSummary{
                    .m_statistic = candidate.m_joint_fit.m_statistic,
                    .m_limit     = candidate.m_joint_limit,
                    .m_effective_dof =
                        candidate.m_joint_fit.m_effective_dof };
              };
          Impl::CurrentGraphLifecycleResult graph_result;
          try
          {
            internal::VioUpdateState candidate_state =
                m_impl->materializeMovingCandidate(
                    prepared, candidate );
            graph_result = m_impl->runCurrentGraphLifecycle(
                std::move( candidate_state ) );
          }
          catch ( const std::exception& exception )
          {
            return fatalAttempt(
                InitializationPhase::kCurrentGraph,
                std::string( "moving current graph failed: " ) +
                    exception.what() );
          }
          if ( const auto* graph_reason =
                   std::get_if<InitializationReason>( &graph_result ) )
          {
            InitializationDiagnostics rejected = makeMovingInitialization(
                InitializationPhase::kCurrentGraph, *graph_reason,
                InitializationOutcome::kRecoverableRejected,
                InitializationTransactionResult::kEvidenceCommitted );
            rejected.m_attempt_id = attempt_id;
            rejected.m_current_graph =
                InitializationGraphResult::kRejected;
            attachCandidateSummaries( rejected );
            transaction.commit();
            return internal::finalizeInitializationResult(
                std::move( result ), rejected, std::nullopt,
                "moving current graph rejected the candidate" );
          }

          Impl::CurrentGraphValidation validated =
              std::get<Impl::CurrentGraphValidation>(
                  std::move( graph_result ) );
          if ( validated.m_info.m_visual_factors == 0U )
          {
            InitializationDiagnostics rejected = makeMovingInitialization(
                InitializationPhase::kCurrentGraph,
                InitializationReason::kCurrentGraphRejected,
                InitializationOutcome::kRecoverableRejected,
                InitializationTransactionResult::kEvidenceCommitted );
            rejected.m_attempt_id = attempt_id;
            rejected.m_current_graph =
                InitializationGraphResult::kRejected;
            attachCandidateSummaries( rejected );
            transaction.commit();
            return internal::finalizeInitializationResult(
                std::move( result ), rejected, std::nullopt,
                "moving current graph has no admitted visual factor" );
          }
          try
          {
            auto validated_owner =
                std::make_unique<internal::VioUpdateState>(
                    std::move( validated.m_candidate ) );
            result.diagnostics.window_size =
                static_cast<std::uint32_t>(
                    validated_owner->m_window.size() );
            result.diagnostics.prior_key     = validated.m_prior_key;
            result.diagnostics.num_landmarks = validated.m_num_landmarks;
            result.diagnostics.lm_iterations = validated.m_lm_iterations;
            result.diagnostics.reproj_rms_before_px =
                validated.m_reproj_rms_before;
            result.diagnostics.reproj_rms_after_px =
                validated.m_reproj_rms;
            result.diagnostics.reproj_rms_after_cull_px =
                validated.m_reproj_rms_after_cull;
            result.diagnostics.max_window_pose_shift_m =
                validated.m_max_window_pose_shift_m;
            result.diagnostics.num_cheirality =
                validated.m_num_cheirality;
            result.diagnostics.outliers_culled =
                validated.m_outliers_culled;
            result.diagnostics.outliers_culled_unique =
                validated.m_outliers_culled_unique;
            result.diagnostics.outlier_reopt_rounds =
                validated.m_outlier_reopt_rounds;
            result.diagnostics.outlier_reopt =
                validated.m_outlier_reopt_rounds > 0U;
            result.diagnostics.outlier_reopt_failed =
                validated.m_outlier_reopt_failed;
            result.diagnostics.culled_landmark_ids =
                std::move( validated.m_culled_landmark_ids );
            result.diagnostics.probe_shift_top =
                std::move( validated.m_probe_shift_top );
            result.diagnostics.probe_res_mean_px =
                validated.m_probe_res_mean_px;
            result.diagnostics.probe_res_max_px =
                validated.m_probe_res_max_px;
            result.diagnostics.probe_res_max_id =
                validated.m_probe_res_max_id;
            result.diagnostics.probe_detail_valid =
                validated.m_probe_detail_valid;
            result.diagnostics.probe_current_pose_translation_delta_m =
                validated.m_probe_current_pose_translation_delta_m;
            result.diagnostics.probe_current_pose_rotation_delta_rad =
                validated.m_probe_current_pose_rotation_delta_rad;
            result.diagnostics.probe_current_velocity_delta_mps =
                validated.m_probe_current_velocity_delta_mps;
            result.diagnostics.probe_current_acc_bias_delta_mps2 =
                validated.m_probe_current_acc_bias_delta_mps2;
            result.diagnostics.probe_current_gyr_bias_delta_radps =
                validated.m_probe_current_gyr_bias_delta_radps;
            result.diagnostics.probe_primary_graph_error_seed =
                validated.m_probe_primary_graph_error_seed;
            result.diagnostics.probe_primary_graph_error_optimized =
                validated.m_probe_primary_graph_error_optimized;
            result.diagnostics.probe_primary_imu_factor_error_seed =
                validated.m_probe_primary_imu_factor_error_seed;
            result.diagnostics.probe_primary_imu_factor_error_optimized =
                validated.m_probe_primary_imu_factor_error_optimized;
            result.diagnostics.num_retained_observations =
                static_cast<std::uint32_t>(
                    validated_owner->m_window.back().m_observations.size() );
            result.diagnostics.num_current_visual_factors =
                validated.m_info.m_current_visual_factors;
            result.diagnostics.num_current_mono_visual_factors =
                validated.m_info.m_current_mono_visual_factors;
            result.diagnostics.m_vio =
                m_impl->makeVioDiagnostics(
                    validated.m_info, *validated_owner );
            validated_owner->m_vio_diagnostics =
                result.diagnostics.m_vio;
            const WindowFrame& latest =
                validated_owner->m_window.back();
            const NavigationState& latest_navigation =
                requireNavigationState( latest,
                                        "moving-root committed estimate" );
            const VioEstimate estimate{
                .timestamp    = latest.m_timestamp,
                .T_W_B        = latest_navigation.m_T_W_B,
                .m_v_W_B      = latest_navigation.m_v_W_B,
                .m_bias       = latest_navigation.m_bias,
                .m_segment_id = validated_owner->m_segment_id };
            InitializationDiagnostics committed = makeMovingInitialization(
                InitializationPhase::kCommit, InitializationReason::kNone,
                InitializationOutcome::kCommitted,
                InitializationTransactionResult::kRootCommitted );
            committed.m_attempt_id = attempt_id;
            committed.m_current_graph =
                InitializationGraphResult::kValidated;
            attachCandidateSummaries( committed );
            VioUpdateResult finalized =
                internal::finalizeInitializationResult(
                    std::move( result ), committed, estimate, {} );
            internal::installVioUpdateStateOwner(
                m_impl->m_state, validated_owner );
            transaction.commit();
            return finalized;
          }
          catch ( const std::exception& exception )
          {
            return fatalAttempt(
                InitializationPhase::kCommit,
                std::string( "moving root install failed: " ) +
                    exception.what() );
          }
        }

        if ( !static_ready )
        {
          InitializationDiagnostics initialization =
              makeMovingInitialization(
                  evidence.m_reason ==
                              InitializationReason::kPoseCountInsufficient ||
                          evidence.m_reason ==
                              InitializationReason::kDurationInsufficient
                      ? InitializationPhase::kCollectingEvidence
                      : InitializationPhase::kVisualSolve,
                  evidence.m_reason, InitializationOutcome::kPending,
                  InitializationTransactionResult::kEvidenceCommitted );
          transaction.commit();
          return internal::finalizeInitializationResult(
              std::move( result ), initialization, std::nullopt,
              "collecting moving initialization evidence" );
        }

        const bool static_visual_seed_ready =
            keyframe &&
            static_cast<int>(
                m_impl->countStereoObservations( measurement ) ) >=
                m_impl->options.min_seed_observations;
        if ( static_visual_seed_ready )
        {
          transaction.rollback();
        }
        else
        {
          transaction.commit();
        }
      }
      if ( !static_ready )
      {
        VioUpdateTransaction transaction( m_impl->m_state );
        if ( *bootstrap_duration_ns >=
             m_impl->options.m_bootstrap_timeout_ns )
        {
          InitializationDiagnostics initialization;
          initialization.m_path = InitializationPath::kStatic;
          initialization.m_phase =
              InitializationPhase::kCollectingEvidence;
          initialization.m_reason  = InitializationReason::kStaticTimeout;
          initialization.m_outcome = InitializationOutcome::kFatalError;
          initialization.m_transaction =
              InitializationTransactionResult::kRolledBack;
          transaction.rollback();
          return internal::finalizeInitializationResult(
              std::move( result ), initialization, std::nullopt,
              "static bootstrap timed out" );
        }
        m_impl->m_state->m_continuity_anchor = measurement.m_timestamp;
        m_impl->m_state->m_bootstrap_nodes   = std::move( bootstrap_nodes );
        if ( !stageUnsupportedSpanForCurrentTimestamp() )
        {
          return finalizePreStagingHardResult(
              UpdateStatus::kFailed,
              "unsupported visual span is not representable" );
        }
        InitializationDiagnostics initialization;
        initialization.m_path = InitializationPath::kStatic;
        initialization.m_phase =
            InitializationPhase::kCollectingEvidence;
        initialization.m_reason =
            InitializationReason::kStaticEvidenceInsufficient;
        initialization.m_outcome = InitializationOutcome::kPending;
        initialization.m_transaction =
            InitializationTransactionResult::kEvidenceCommitted;
        transaction.commit();
        return internal::finalizeInitializationResult(
            std::move( result ), initialization, std::nullopt,
            "collecting static bootstrap evidence" );
      }

      static_root_selected       = true;
      bootstrap_bias.m_gyr_radps = bootstrap_stats.m_gyr_mean;
      bootstrap_T_W_B.linear() =
          minimalRotationToWorldUp( bootstrap_stats.m_acc_mean );
      if ( measurement.m_observations.empty() )
      {
        VioUpdateTransaction transaction( m_impl->m_state );
        m_impl->m_state->m_bootstrap_nodes   = std::move( bootstrap_nodes );
        m_impl->m_state->m_continuity_anchor = measurement.m_timestamp;
        result.status                        = UpdateStatus::kInitializing;
        result.message                       = "static bootstrap ready; waiting for visual seed";
        if ( !stageUnsupportedSpanForCurrentTimestamp() )
        {
          return finalizePreStagingHardResult(
              UpdateStatus::kFailed,
              "unsupported visual span is not representable" );
        }
        InitializationDiagnostics initialization;
        initialization.m_path = InitializationPath::kStatic;
        initialization.m_phase =
            InitializationPhase::kCollectingEvidence;
        initialization.m_reason =
            InitializationReason::kStaticEvidenceInsufficient;
        initialization.m_outcome = InitializationOutcome::kPending;
        initialization.m_transaction =
            InitializationTransactionResult::kEvidenceCommitted;
        transaction.commit();
        return internal::finalizeInitializationResult(
            std::move( result ), initialization, std::nullopt,
            "static bootstrap ready; waiting for visual seed" );
      }
    }

    const auto retainBootstrapAndReturn =
        [ & ]( std::string message ) -> VioUpdateResult {
      VioUpdateTransaction transaction( m_impl->m_state );
      m_impl->m_state->m_bootstrap_nodes   = bootstrap_nodes;
      m_impl->m_state->m_continuity_anchor = measurement.m_timestamp;
      if ( !stageUnsupportedSpanForCurrentTimestamp() )
      {
        return finalizePreStagingHardResult(
            UpdateStatus::kFailed,
            "unsupported visual span is not representable" );
      }
      InitializationDiagnostics initialization;
      initialization.m_path = InitializationPath::kStatic;
      initialization.m_phase =
          InitializationPhase::kCollectingEvidence;
      initialization.m_reason =
          InitializationReason::kStaticEvidenceInsufficient;
      initialization.m_outcome = InitializationOutcome::kPending;
      initialization.m_transaction =
          InitializationTransactionResult::kEvidenceCommitted;
      transaction.commit();
      return internal::finalizeInitializationResult(
          std::move( result ), initialization, std::nullopt,
          std::move( message ) );
    };

    std::uint32_t num_shared              = 0;
    std::uint32_t num_mapped_observations = 0;
    for ( const StereoObservation& observation : measurement.m_observations )
    {
      const bool mapped =
          m_impl->m_state->m_landmarks_w.find( observation.id ) !=
          m_impl->m_state->m_landmarks_w.end();
      if ( mapped )
      {
        ++num_mapped_observations;
      }
      // Keep num_shared as the historical positive-disparity overlap.
      // PnP, full support, and low-connectivity use the mapped population.
      if ( observation.disparity_px > 0.0 )
      {
        if ( mapped )
        {
          ++num_shared;
        }
      }
    }
    result.diagnostics.num_shared              = num_shared;
    result.diagnostics.num_mapped_observations = num_mapped_observations;

    const bool active_segment = m_impl->m_state->m_initialized;
    const bool full_visual_support =
        !active_segment ||
        static_cast<int>( num_mapped_observations ) >=
            m_impl->options.min_pnp_inliers;
    const bool low_visual_support =
        active_segment && !full_visual_support;
    std::int64_t next_visual_coast_duration_ns = 0;

    if ( low_visual_support )
    {
      const std::int64_t coast_duration_ns =
          m_impl->m_state->m_visual_coast_duration_ns;
      const std::int64_t horizon_ns =
          m_impl->options.m_visual_coast_horizon_ns;
      if ( coast_duration_ns < 0 || coast_duration_ns > horizon_ns )
      {
        return finalizePreStagingHardResult(
            UpdateStatus::kFailed,
            "visual coast duration violates estimator invariant" );
      }
      if ( normalized_imu.m_duration_ns > horizon_ns - coast_duration_ns )
      {
        VioUpdateTransaction transaction( m_impl->m_state );
        if ( !stageUnsupportedSpanForCurrentTimestamp() )
        {
          return finalizePreStagingHardResult(
              UpdateStatus::kFailed,
              "unsupported visual span is not representable" );
        }
        const std::uint32_t completed_segment =
            m_impl->completeActiveSegment( measurement.m_timestamp );
        result.diagnostics.segment_id  = m_impl->m_state->m_segment_id;
        result.diagnostics.window_size = 0;
        result.diagnostics.prior_key   = 0;
        result.diagnostics.m_vio =
            m_impl->m_state->m_vio_diagnostics;
        result.diagnostics.m_vio.m_completed_segment_id = completed_segment;
        result.status                                   = UpdateStatus::kVisualOutage;
        result.message                                  = "visual support outage completed the active segment";
        transaction.commit();
        return result;
      }
      next_visual_coast_duration_ns =
          coast_duration_ns + normalized_imu.m_duration_ns;
    }

    if ( !m_impl->m_state->m_initialized && !keyframe )
    {
      return retainBootstrapAndReturn(
          "static bootstrap ready; waiting for a keyframe visual seed" );
    }
    result.diagnostics.low_connectivity =
        m_impl->m_state->m_initialized && full_visual_support &&
        static_cast<int>( num_mapped_observations ) <
            m_impl->options.min_shared_landmarks;

    if ( !m_impl->m_state->m_initialized )
    {
      const int stereo_count =
          static_cast<int>( m_impl->countStereoObservations( measurement ) );
      if ( stereo_count < m_impl->options.min_seed_observations )
      {
        return retainBootstrapAndReturn(
            "insufficient observations to seed first segment" );
      }
    }

    std::vector<LandmarkId> frame_culled;

    VioUpdateTransaction transaction( m_impl->m_state );
    m_impl->m_state->m_visual_coast_duration_ns =
        next_visual_coast_duration_ns;
    auto finalizePostStagingHardResult =
        [ & ]( UpdateStatus status, std::string message ) -> VioUpdateResult {
      transaction.rollback();
      result.status                 = status;
      result.message                = std::move( message );
      result.diagnostics.segment_id = m_impl->m_state->m_segment_id;
      result.diagnostics.window_size =
          static_cast<std::uint32_t>( m_impl->m_state->m_window.size() );
      result.diagnostics.prior_key = m_impl->m_state->m_window.empty()
                                         ? 0U
                                         : m_impl->m_state->m_window.front()
                                               .m_frame_index;
      result.diagnostics.m_vio     = m_impl->m_state->m_vio_diagnostics;
      result.diagnostics.culled_landmark_ids.clear();
      return result;
    };
    Eigen::Isometry3d propagated_T_W_B = bootstrap_T_W_B;
    Eigen::Vector3d   propagated_v_W_B = Eigen::Vector3d::Zero();
    ImuBias           propagated_bias  = bootstrap_bias;
    if ( m_impl->m_state->m_initialized )
    {
      const WindowFrame& predecessor = m_impl->m_state->m_window.back();
      try
      {
        const NavigationState& predecessor_navigation =
            requireNavigationState( predecessor, "IMU propagation" );
        const auto pim = m_impl->preintegrate(
            normalized_imu, predecessor_navigation.m_bias );
        const gtsam::NavState predicted = pim.predict(
            gtsam::NavState( toPose3( predecessor_navigation.m_T_W_B ),
                             predecessor_navigation.m_v_W_B ),
            toGtsamBias( predecessor_navigation.m_bias ) );
        propagated_T_W_B = toIsometry( predicted.pose() );
        propagated_v_W_B = predicted.v();
        propagated_bias  = predecessor_navigation.m_bias;
      }
      catch ( const std::exception& exception )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed,
            std::string( "IMU propagation failed: " ) + exception.what() );
      }
    }

    if ( !m_impl->m_state->m_initialized )
    {
      if ( static_root_selected &&
           m_impl->options.m_enable_moving_bootstrap )
      {
        m_impl->clearProvisionalMovingLifecycleForStaticRoot();
      }
      if ( !m_impl->seedRoot( bootstrap_T_W_B, Eigen::Vector3d::Zero(),
                              bootstrap_bias, measurement,
                              result.diagnostics.probe_rejected_block_n,
                              result.diagnostics.probe_new_lm_n ) )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kRejected,
            "failed to backproject landmark on first frame" );
      }
      m_impl->m_state->m_bootstrap_nodes.clear();
      m_impl->m_state->m_static_candidate_ended = false;
      m_impl->m_state->m_continuity_anchor.reset();
    }
    else
    {
      WindowFrame candidate;
      candidate.m_frame_index  = m_impl->m_state->m_next_frame_index;
      candidate.m_timestamp    = measurement.m_timestamp;
      candidate.m_observations = measurement.m_observations;
      candidate.m_is_keyframe  = keyframe;

      const Eigen::Isometry3d guess_T_W_B = propagated_T_W_B;
      candidate.m_navigation              = NavigationState{
                       .m_T_W_B = guess_T_W_B,
                       .m_v_W_B = propagated_v_W_B,
                       .m_bias  = propagated_bias };
      NavigationState& candidate_navigation = *candidate.m_navigation;
      candidate.m_predecessor_frame_index =
          m_impl->m_state->m_window.back().m_frame_index;
      candidate.m_imu = normalized_imu;

      if ( m_impl->options.enable_pnp_init &&
           static_cast<int>( num_mapped_observations ) >=
               m_impl->options.min_pnp_inliers )
      {
        const Impl::PnpInitResult pnp =
            m_impl->tryPnpInit( measurement, guess_T_W_B );
        if ( m_impl->options.enable_probe_b )
        {
          result.diagnostics.probe_pnp_attempted = true;
          result.diagnostics.probe_pnp_result    = pnp.probe_result;
          result.diagnostics.probe_pnp_correspondences =
              pnp.num_correspondences;
          result.diagnostics.probe_pnp_inliers = pnp.num_inliers;
          result.diagnostics.probe_pnp_proposal_rms_px =
              pnp.proposal_rms_px;
          result.diagnostics.probe_pnp_guess_rms_px = pnp.guess_rms_px;
          result.diagnostics.probe_pnp_proposal_inlier_count =
              pnp.proposal_inlier_count;
          result.diagnostics.probe_pnp_proposal_cheiral_inlier_count =
              pnp.proposal_cheiral_inlier_count;
          result.diagnostics.probe_pnp_proposal_min_inlier_depth_m =
              pnp.proposal_min_inlier_depth_m;
          result.diagnostics.probe_pnp_proposal_min_inlier_landmark_id =
              pnp.proposal_min_inlier_landmark_id;
          result.diagnostics.probe_pnp_proposal_guess_translation_delta_m =
              pnp.proposal_guess_translation_delta_m;
          result.diagnostics.probe_pnp_proposal_guess_rotation_delta_rad =
              pnp.proposal_guess_rotation_delta_rad;
          result.diagnostics.probe_pnp_guess_retry_outcome =
              pnp.guess_retry_outcome;
          result.diagnostics.probe_pnp_guess_retry_proposal_rms_px =
              pnp.guess_retry_proposal_rms_px;
          result.diagnostics.probe_pnp_guess_retry_guess_rms_px =
              pnp.guess_retry_guess_rms_px;
          result.diagnostics.probe_pnp_guess_retry_cheiral_inlier_count =
              pnp.guess_retry_cheiral_inlier_count;
          result.diagnostics.probe_pnp_guess_retry_min_inlier_depth_m =
              pnp.guess_retry_min_inlier_depth_m;
          result.diagnostics.probe_pnp_guess_retry_translation_delta_m =
              pnp.guess_retry_translation_delta_m;
          result.diagnostics.probe_pnp_guess_retry_rotation_delta_rad =
              pnp.guess_retry_rotation_delta_rad;
        }
        if ( pnp.success )
        {
          candidate_navigation.m_T_W_B = pnp.T_W_B;

          // tryPnpInit maps the accepted RANSAC indices back to the exact
          // pre-intake mapped-observation scan used to create correspondences.
          std::vector<StereoObservation> filtered;
          filtered.reserve( candidate.m_observations.size() );
          for ( const StereoObservation& observation :
                candidate.m_observations )
          {
            const bool is_mapped =
                m_impl->m_state->m_landmarks_w.find( observation.id ) !=
                m_impl->m_state->m_landmarks_w.end();
            if ( is_mapped &&
                 pnp.inlier_ids.find( observation.id ) ==
                     pnp.inlier_ids.end() )
            {
              continue;
            }
            filtered.push_back( observation );
          }
          candidate.m_observations = std::move( filtered );

          result.diagnostics.pnp_success = true;
          result.diagnostics.pnp_inliers = pnp.num_inliers;
        }
      }

      if ( !isFinite( candidate_navigation.m_T_W_B ) )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed, "non-finite pose initial value" );
      }
      // Track history uses the full validated measurement, including mapped
      // PnP outliers masked from candidate.m_observations.
      for ( const StereoObservation& observation :
            measurement.m_observations )
      {
        m_impl->m_state->m_track_times[ observation.id ].push_back(
            measurement.m_timestamp );
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
      for ( const StereoObservation& observation : candidate.m_observations )
      {
        if ( observation.disparity_px <= 0.0 )
        {
          continue;
        }
        auto landmark_it = m_impl->m_state->m_landmarks_w.find( observation.id );
        if ( landmark_it == m_impl->m_state->m_landmarks_w.end() )
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
            toPose3( candidate_navigation.m_T_W_B ) *
            m_impl->body_P_sensor;
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
            m_impl->backprojectWorld( candidate_navigation.m_T_W_B,
                                      observation );
        if ( !point_W_new.has_value() || !isFinite( *point_W_new ) )
        {
          continue;
        }
        landmark_it->second = *point_W_new;  // drifted track — refresh (ck path)
      }
      // New ids only: backproject from masked candidate.m_observations.
      // Slice ⑤c: only keyframes seed new landmarks; non-keyframes enter
      // the window/BA but do not grow the map.
      // Slice ⑥b: require the track to have >= 3 observations before
      // seeding — a single-frame disparity can be a SAD mismatch under
      // fast motion, producing a bad-depth anchor that drags the BA.
      if ( keyframe )
      {
        for ( const StereoObservation& observation : candidate.m_observations )
        {
          if ( m_impl->m_state->m_landmarks_w.find( observation.id ) !=
               m_impl->m_state->m_landmarks_w.end() )
          {
            continue;
          }
          if ( m_impl->options.block_culled_rebirth &&
               m_impl->m_state->m_culled_ids.find( observation.id ) !=
                   m_impl->m_state->m_culled_ids.end() )
          {
            ++result.diagnostics.probe_rejected_block_n;
            continue;
          }
          const auto        track_it    = m_impl->m_state->m_track_times.find( observation.id );
          const std::size_t seed_thresh = static_cast<std::size_t>(
              m_impl->options.min_track_observations_for_seed );
          if ( track_it == m_impl->m_state->m_track_times.end() ||
               track_it->second.size() < seed_thresh )
          {
            std::cerr << "[6b] skip seed id=" << observation.id
                      << " times="
                      << ( track_it == m_impl->m_state->m_track_times.end()
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
              m_impl->backprojectWorld( candidate_navigation.m_T_W_B,
                                        observation );
          if ( !point_W.has_value() )
          {
            continue;
          }
          const gtsam::Pose3 T_W_left =
              toPose3( candidate_navigation.m_T_W_B ) *
              m_impl->body_P_sensor;
          const gtsam::Point3 point_left = T_W_left.transformTo(
              gtsam::Point3( point_W->x(), point_W->y(), point_W->z() ) );
          if ( !isFinite( *point_W ) || point_left.z() <= 0.0 )
          {
            continue;
          }
          m_impl->m_state->m_landmarks_w[ observation.id ] = *point_W;
          ++result.diagnostics.probe_new_lm_n;
        }
      }  // keyframe-only landmark seeding

      m_impl->m_state->m_window.push_back( std::move( candidate ) );
      ++m_impl->m_state->m_next_frame_index;
    }

    try
    {
      m_impl->enforceWindowCapacity();
    }
    catch ( const std::exception& exception )
    {
      return finalizePostStagingHardResult(
          UpdateStatus::kFailed,
          std::string( "navigation eviction reintegration failed: " ) +
              exception.what() );
    }
    m_impl->pruneLandmarksNotInWindow();

    Impl::CurrentGraphLifecycleResult lifecycle_result;
    try
    {
      lifecycle_result = m_impl->runCurrentGraphLifecycle(
          *m_impl->m_state );
    }
    catch ( const std::exception& exception )
    {
      return finalizePostStagingHardResult(
          UpdateStatus::kFailed,
          std::string( "current graph lifecycle failed: " ) +
              exception.what() );
    }
    if ( const auto* lifecycle_reason =
             std::get_if<InitializationReason>( &lifecycle_result ) )
    {
      return finalizePostStagingHardResult(
          UpdateStatus::kFailed,
          *lifecycle_reason ==
                  InitializationReason::kCurrentGraphUnderconstrained
              ? "current graph is underconstrained"
              : "current graph quality rejected the update" );
    }
    Impl::CurrentGraphValidation lifecycle =
        std::get<Impl::CurrentGraphValidation>(
            std::move( lifecycle_result ) );
    internal::installVioUpdateStateInPlace(
        m_impl->m_state, std::move( lifecycle.m_candidate ) );
    result.diagnostics.num_landmarks = lifecycle.m_num_landmarks;
    result.diagnostics.prior_key     = lifecycle.m_prior_key;
    result.diagnostics.window_size =
        static_cast<std::uint32_t>( m_impl->m_state->m_window.size() );
    result.diagnostics.reproj_rms_before_px =
        lifecycle.m_reproj_rms_before;
    result.diagnostics.reproj_rms_after_px = lifecycle.m_reproj_rms;
    result.diagnostics.reproj_rms_after_cull_px =
        lifecycle.m_reproj_rms_after_cull;
    result.diagnostics.lm_iterations   = lifecycle.m_lm_iterations;
    result.diagnostics.num_cheirality  = lifecycle.m_num_cheirality;
    result.diagnostics.outliers_culled = lifecycle.m_outliers_culled;
    result.diagnostics.outliers_culled_unique =
        lifecycle.m_outliers_culled_unique;
    result.diagnostics.outlier_reopt_rounds =
        lifecycle.m_outlier_reopt_rounds;
    result.diagnostics.outlier_reopt =
        lifecycle.m_outlier_reopt_rounds > 0U;
    result.diagnostics.outlier_reopt_failed =
        lifecycle.m_outlier_reopt_failed;
    result.diagnostics.max_window_pose_shift_m =
        lifecycle.m_max_window_pose_shift_m;
    result.diagnostics.culled_landmark_ids =
        std::move( lifecycle.m_culled_landmark_ids );
    result.diagnostics.probe_shift_top =
        std::move( lifecycle.m_probe_shift_top );
    result.diagnostics.probe_res_mean_px =
        lifecycle.m_probe_res_mean_px;
    result.diagnostics.probe_res_max_px =
        lifecycle.m_probe_res_max_px;
    result.diagnostics.probe_res_max_id =
        lifecycle.m_probe_res_max_id;
    result.diagnostics.probe_detail_valid =
        lifecycle.m_probe_detail_valid;
    result.diagnostics.probe_current_pose_translation_delta_m =
        lifecycle.m_probe_current_pose_translation_delta_m;
    result.diagnostics.probe_current_pose_rotation_delta_rad =
        lifecycle.m_probe_current_pose_rotation_delta_rad;
    result.diagnostics.probe_current_velocity_delta_mps =
        lifecycle.m_probe_current_velocity_delta_mps;
    result.diagnostics.probe_current_acc_bias_delta_mps2 =
        lifecycle.m_probe_current_acc_bias_delta_mps2;
    result.diagnostics.probe_current_gyr_bias_delta_radps =
        lifecycle.m_probe_current_gyr_bias_delta_radps;
    result.diagnostics.probe_primary_graph_error_seed =
        lifecycle.m_probe_primary_graph_error_seed;
    result.diagnostics.probe_primary_graph_error_optimized =
        lifecycle.m_probe_primary_graph_error_optimized;
    result.diagnostics.probe_primary_imu_factor_error_seed =
        lifecycle.m_probe_primary_imu_factor_error_seed;
    result.diagnostics.probe_primary_imu_factor_error_optimized =
        lifecycle.m_probe_primary_imu_factor_error_optimized;
    result.diagnostics.num_retained_observations =
        static_cast<std::uint32_t>(
            m_impl->m_state->m_window.back().m_observations.size() );
    result.diagnostics.num_seeded_landmarks =
        result.diagnostics.probe_new_lm_n;
    result.diagnostics.num_current_visual_factors =
        lifecycle.m_info.m_current_visual_factors;
    result.diagnostics.num_current_mono_visual_factors =
        lifecycle.m_info.m_current_mono_visual_factors;
    if ( active_segment && full_visual_support )
    {
      m_impl->m_state->m_last_visual_support_timestamp =
          measurement.m_timestamp;
      m_impl->m_state->m_unsupported_span_ns = 0;
    }
    else if ( !stageUnsupportedSpanForCurrentTimestamp() )
    {
      return finalizePostStagingHardResult(
          UpdateStatus::kFailed,
          "unsupported visual span is not representable" );
    }
    result.diagnostics.unsupported_span_ns =
        m_impl->m_state->m_unsupported_span_ns;
    result.diagnostics.m_vio = m_impl->makeVioDiagnostics(
        lifecycle.m_info, *m_impl->m_state );
    m_impl->m_state->m_vio_diagnostics = result.diagnostics.m_vio;
    const WindowFrame& lifecycle_estimate_frame =
        m_impl->m_state->m_window.back();
    const NavigationState& lifecycle_estimate_navigation =
        requireNavigationState( lifecycle_estimate_frame,
                                "active committed estimate" );
    const VioEstimate estimate{
        .timestamp    = measurement.m_timestamp,
        .T_W_B        = lifecycle_estimate_navigation.m_T_W_B,
        .m_v_W_B      = lifecycle_estimate_navigation.m_v_W_B,
        .m_bias       = lifecycle_estimate_navigation.m_bias,
        .m_segment_id = m_impl->m_state->m_segment_id };
    if ( !active_segment && static_root_selected )
    {
      InitializationDiagnostics initialization;
      initialization.m_path    = InitializationPath::kStatic;
      initialization.m_phase   = InitializationPhase::kCommit;
      initialization.m_outcome = InitializationOutcome::kCommitted;
      initialization.m_current_graph =
          InitializationGraphResult::kValidated;
      initialization.m_transaction =
          InitializationTransactionResult::kRootCommitted;
      VioUpdateResult finalized =
          internal::finalizeInitializationResult(
              std::move( result ), initialization, estimate, {} );
      transaction.commit();
      return finalized;
    }
    result.status   = UpdateStatus::kOk;
    result.estimate = estimate;
    transaction.commit();
    return result;
  }

#ifdef PHAD_ESTIMATOR_TESTING
  void internal::VioEstimatorTestAccess::setNextMovingCandidateResult(
      VioEstimator& estimator, InitialMovingCandidateResult result )
  {
    if ( !estimator.m_impl )
    {
      throw std::invalid_argument(
          "test access requires a live VioEstimator" );
    }
    if ( estimator.m_impl->m_test_next_moving_candidate_result.has_value() )
    {
      throw std::logic_error(
          "next moving candidate test result is already set" );
    }
    estimator.m_impl->m_test_next_moving_candidate_result =
        std::move( result );
  }

  void internal::VioEstimatorTestAccess::setNextCurrentGraphResult(
      VioEstimator& estimator, const InitializationReason reason )
  {
    if ( !estimator.m_impl )
    {
      throw std::invalid_argument(
          "test access requires a live VioEstimator" );
    }
    if ( reason != InitializationReason::kCurrentGraphUnderconstrained &&
         reason != InitializationReason::kCurrentGraphRejected )
    {
      throw std::invalid_argument(
          "test current graph result must be a graph rejection" );
    }
    if ( estimator.m_impl->m_test_next_current_graph_result.has_value() ||
         estimator.m_impl->m_test_next_current_graph_contract_error )
    {
      throw std::logic_error(
          "next current graph test result is already set" );
    }
    estimator.m_impl->m_test_next_current_graph_result = reason;
  }

  void internal::VioEstimatorTestAccess::setNextCurrentGraphContractError(
      VioEstimator& estimator )
  {
    if ( !estimator.m_impl )
    {
      throw std::invalid_argument(
          "test access requires a live VioEstimator" );
    }
    if ( estimator.m_impl->m_test_next_current_graph_contract_error ||
         estimator.m_impl->m_test_next_current_graph_result.has_value() )
    {
      throw std::logic_error(
          "next current graph test result is already set" );
    }
    estimator.m_impl->m_test_next_current_graph_contract_error = true;
  }

  void internal::VioEstimatorTestAccess::setNextInitialVisualDepth(
      VioEstimator& estimator, const double depth )
  {
    if ( !estimator.m_impl )
    {
      throw std::invalid_argument(
          "test access requires a live VioEstimator" );
    }
    if ( estimator.m_impl->m_test_next_initial_visual_depth.has_value() )
    {
      throw std::logic_error(
          "next initial visual depth is already set" );
    }
    estimator.m_impl->m_test_next_initial_visual_depth = depth;
  }

  void internal::VioEstimatorTestAccess::setNextInitialVisualWhiteningError(
      VioEstimator& estimator )
  {
    if ( !estimator.m_impl )
    {
      throw std::invalid_argument(
          "test access requires a live VioEstimator" );
    }
    if ( estimator.m_impl->m_test_next_initial_visual_whitening_error )
    {
      throw std::logic_error(
          "next initial visual whitening error is already set" );
    }
    estimator.m_impl->m_test_next_initial_visual_whitening_error = true;
  }

  void internal::VioEstimatorTestAccess::setNextPnpProposalFailure(
      VioEstimator& estimator, const PnpProbeResult failure )
  {
    if ( !estimator.m_impl )
    {
      throw std::invalid_argument(
          "test access requires a live VioEstimator" );
    }
    switch ( failure )
    {
      case PnpProbeResult::kProposalPoseNonFinite:
      case PnpProbeResult::kProposalRmsInvalidCorrespondence:
      case PnpProbeResult::kProposalRmsCheirality:
      case PnpProbeResult::kProposalRmsNonFinite:
        break;
      default:
        throw std::invalid_argument(
            "test PnP proposal failure must be a typed proposal failure" );
    }
    if ( estimator.m_impl->m_test_next_pnp_proposal_failure.has_value() )
    {
      throw std::logic_error(
          "next PnP proposal failure is already set" );
    }
    estimator.m_impl->m_test_next_pnp_proposal_failure = failure;
  }

  void internal::VioEstimatorTestAccess::setNextPnpGuessRetryOutcome(
      VioEstimator& estimator, const PnpGuessRetryOutcome outcome )
  {
    if ( !estimator.m_impl )
    {
      throw std::invalid_argument(
          "test access requires a live VioEstimator" );
    }
    switch ( outcome )
    {
      case PnpGuessRetryOutcome::kSolveFailed:
      case PnpGuessRetryOutcome::kProposalPoseNonFinite:
      case PnpGuessRetryOutcome::kProposalRmsInvalidCorrespondence:
      case PnpGuessRetryOutcome::kProposalRmsCheirality:
      case PnpGuessRetryOutcome::kProposalRmsNonFinite:
      case PnpGuessRetryOutcome::kProposalRmsRejected:
        break;
      default:
        throw std::invalid_argument(
            "test PnP guess retry outcome must be a retry failure" );
    }
    if ( estimator.m_impl->m_test_next_pnp_guess_retry_outcome.has_value() )
    {
      throw std::logic_error(
          "next PnP guess retry outcome is already set" );
    }
    estimator.m_impl->m_test_next_pnp_guess_retry_outcome = outcome;
  }

  std::size_t
  internal::VioEstimatorTestAccess::lastInitialVisualSolveWindowSize(
      const VioEstimator& estimator )
  {
    if ( !estimator.m_impl )
    {
      throw std::invalid_argument(
          "test access requires a live VioEstimator" );
    }
    return estimator.m_impl->m_test_last_initial_visual_solve_window_size;
  }
#endif

}  // namespace phad::estimator
