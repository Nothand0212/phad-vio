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
#include "phad/estimator/internal/vio_update_transaction.hpp"

namespace phad::estimator
{
  namespace
  {

    // GTSAM 4.3 exports uppercase Symbol helpers (X/L); older docs used x/l.
    using gtsam::symbol_shorthand::B;
    using gtsam::symbol_shorthand::L;
    using gtsam::symbol_shorthand::V;
    using gtsam::symbol_shorthand::X;
    using internal::ImuIntervalError;
    using internal::ImuIntervalResult;
    using internal::ImuIntervalStep;
    using internal::NormalizedImuInterval;
    using internal::normalizeRawImuInterval;
    using internal::spliceNormalizedImuIntervals;
    using internal::VioUpdateTransaction;
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

    [[nodiscard]] std::optional<std::size_t> recentBootstrapBegin(
        const std::vector<sensor::ImuMeasurement>& samples,
        std::int64_t                               min_duration_ns,
        std::uint32_t                              min_samples )
    {
      const std::size_t required_samples =
          static_cast<std::size_t>( min_samples );
      if ( samples.size() < required_samples )
      {
        return std::nullopt;
      }

      std::size_t begin = samples.size() - required_samples;
      while ( true )
      {
        const std::optional<std::int64_t> duration_ns =
            checkedPositiveDurationNs( samples[ begin ].timestamp,
                                       samples.back().timestamp );
        if ( duration_ns.has_value() && *duration_ns >= min_duration_ns )
        {
          return begin;
        }
        if ( begin == 0U )
        {
          return std::nullopt;
        }
        --begin;
      }
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
      m_state->m_vio_diagnostics = makeVioDiagnostics( VioGraphInfo{} );
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
      m_state->m_pending_seed_obs.clear();
      m_state->m_bootstrap_nodes.clear();
      m_state->m_continuity_anchor        = next_continuity_anchor;
      m_state->m_visual_coast_duration_ns = 0;
      m_state->m_initialized              = false;
      m_state->m_vio_diagnostics          = makeVioDiagnostics( VioGraphInfo{} );
      ++m_state->m_segment_id;
      return completed_segment;
    }

    void eraseLandmarkFromWindow( LandmarkId id )
    {
      for ( WindowFrame& frame : m_state->m_window )
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
      candidate.m_T_W_B        = anchor_T_W_B;
      candidate.m_is_keyframe  = true;
      candidate.m_v_W_B        = v_W_B;
      candidate.m_bias         = bias;

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
            backprojectWorld( candidate.m_T_W_B, observation );
        const gtsam::Pose3 T_W_left =
            toPose3( candidate.m_T_W_B ) * body_P_sensor;
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
        const auto landmark_it = m_state->m_landmarks_w.find( observation->id );
        if ( landmark_it == m_state->m_landmarks_w.end() ||
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

    // Shared landmarks ∩ current left observations → solvePnPRansac.
    // A finite, sufficiently supported proposal is accepted only when its
    // stereo RMS on the PnP inlier set is no more than one observation sigma
    // worse than the existing guess.
    [[nodiscard]] PnpInitResult tryPnpInit(
        const VioMeasurement&    measurement,
        const Eigen::Isometry3d& guess_T_W_B ) const
    {
      PnpInitResult result;

      std::vector<cv::Point3d>              pts3d;
      std::vector<cv::Point2d>              pts2d;
      std::vector<const StereoObservation*> shared_observations;
      pts3d.reserve( measurement.m_observations.size() );
      pts2d.reserve( measurement.m_observations.size() );
      shared_observations.reserve( measurement.m_observations.size() );
      for ( const StereoObservation& observation : measurement.m_observations )
      {
        const auto landmark_it = m_state->m_landmarks_w.find( observation.id );
        if ( landmark_it == m_state->m_landmarks_w.end() )
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
      for ( auto it = m_state->m_window.rbegin(); it != m_state->m_window.rend(); ++it )
      {
        for ( const StereoObservation& observation : it->m_observations )
        {
          if ( observation.id == id && observation.disparity_px > 0.0 )
          {
            last_stereo_pose = it->m_T_W_B;
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
               !hangingLandmarkStale( it->first, m_state->m_window.back().m_T_W_B ) )
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
    [[nodiscard]] std::unordered_map<LandmarkId, int> countObservations()
        const
    {
      std::unordered_map<LandmarkId, int> counts;
      for ( const WindowFrame& frame : m_state->m_window )
      {
        for ( const StereoObservation& observation : frame.m_observations )
        {
          if ( m_state->m_landmarks_w.find( observation.id ) !=
               m_state->m_landmarks_w.end() )
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

    void buildGraph( gtsam::NonlinearFactorGraph& graph,
                     gtsam::Values&               values,
                     std::uint64_t&               prior_key_out,
                     std::uint32_t&               num_landmarks_out,
                     const gtsam::Values*         round_start  = nullptr,
                     VioGraphInfo*                vio_info_out = nullptr ) const
    {
      graph.resize( 0 );
      values.clear();
      num_landmarks_out = 0;
      VioGraphInfo vio_info;
      if ( m_state->m_window.empty() )
      {
        prior_key_out = 0;
        if ( vio_info_out != nullptr )
        {
          *vio_info_out = vio_info;
        }
        return;
      }

      prior_key_out = m_state->m_window.front().m_frame_index;
      for ( const WindowFrame& frame : m_state->m_window )
      {
        const gtsam::Key   pose_key     = X( frame.m_frame_index );
        const gtsam::Key   velocity_key = V( frame.m_frame_index );
        const gtsam::Key   bias_key     = B( frame.m_frame_index );
        const gtsam::Pose3 pose =
            round_start != nullptr && round_start->exists( pose_key )
                ? round_start->at<gtsam::Pose3>( pose_key )
                : toPose3( frame.m_T_W_B );
        const Eigen::Vector3d velocity =
            round_start != nullptr && round_start->exists( velocity_key )
                ? round_start->at<gtsam::Vector3>( velocity_key )
                : frame.m_v_W_B;
        const gtsam::imuBias::ConstantBias bias =
            round_start != nullptr && round_start->exists( bias_key )
                ? round_start->at<gtsam::imuBias::ConstantBias>( bias_key )
                : toGtsamBias( frame.m_bias );
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

      for ( const WindowFrame& frame : m_state->m_window )
      {
        if ( !frame.m_predecessor_frame_index.has_value() )
        {
          graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
              X( frame.m_frame_index ), toPose3( frame.m_T_W_B ), prior_noise );
          graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
              V( frame.m_frame_index ), frame.m_v_W_B,
              velocity_prior_noise );
          graph.emplace_shared<
              gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
              B( frame.m_frame_index ), toGtsamBias( frame.m_bias ),
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
            m_state->m_window.begin(), m_state->m_window.end(),
            [ predecessor ]( const WindowFrame& candidate ) {
              return candidate.m_frame_index == predecessor;
            } );
        if ( predecessor_it == m_state->m_window.end() )
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

      const auto                     counts = countObservations();
      std::unordered_set<LandmarkId> inserted;
      for ( const WindowFrame& frame : m_state->m_window )
      {
        for ( const StereoObservation& observation : frame.m_observations )
        {
          const auto count_it = counts.find( observation.id );
          if ( count_it == counts.end() ||
               count_it->second < options.min_landmark_observations )
          {
            continue;
          }
          const auto landmark_it = m_state->m_landmarks_w.find( observation.id );
          if ( landmark_it == m_state->m_landmarks_w.end() )
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
                 m_state->m_window.back().m_frame_index )
            {
              ++vio_info.m_current_mono_visual_factors;
            }
          }
          ++vio_info.m_visual_factors;
          if ( frame.m_frame_index ==
               m_state->m_window.back().m_frame_index )
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
        const VioGraphInfo& info ) const
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
          m_state->m_visual_coast_duration_ns;
      diagnostics.m_non_keyframe_evictions =
          m_state->m_non_keyframe_evictions;
      diagnostics.m_imu_reintegrations = m_state->m_imu_reintegrations;
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
        const gtsam::Values&     values,
        std::vector<LandmarkId>& frame_culled )
    {
      std::uint32_t           dropped = 0;
      std::vector<LandmarkId> to_drop;
      for ( const auto& [ id, point_W ] : m_state->m_landmarks_w )
      {
        const gtsam::Key key = L( id );
        if ( !values.exists( key ) )
        {
          continue;
        }
        const gtsam::Point3 optimized = values.at<gtsam::Point3>( key );
        bool                behind    = false;
        for ( const WindowFrame& frame : m_state->m_window )
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
                  : toPose3( frame.m_T_W_B );
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
        m_state->m_landmarks_w.erase( id );
        eraseLandmarkFromWindow( id );
        m_state->m_culled_ids.insert( id );
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
      candidate_ids.reserve( m_state->m_landmarks_w.size() );
      for ( const auto& [ id, _unused ] : m_state->m_landmarks_w )
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
          m_state->m_landmarks_w.erase( id );
          eraseLandmarkFromWindow( id );
          frame_culled.push_back( id );
          ++culled_round;
          ++outliers_culled_total;
          if ( m_state->m_culled_ids.insert( id ).second )
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
      m_impl->m_state->m_visual_coast_duration_ns = 0;
      m_impl->m_state->m_continuity_anchor        = discontinuity.m_t_end;
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
    }

    std::vector<sensor::ImuMeasurement> bootstrap_nodes;
    BootstrapStats                      bootstrap_stats;
    Eigen::Isometry3d                   bootstrap_T_W_B =
        Eigen::Isometry3d::Identity();
    ImuBias bootstrap_bias;
    if ( !m_impl->m_state->m_initialized )
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

      const std::optional<std::int64_t> bootstrap_duration_ns =
          checkedPositiveDurationNs( bootstrap_nodes.front().timestamp,
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
      const bool static_ready =
          *bootstrap_duration_ns >=
              m_impl->options.m_bootstrap_min_duration_ns &&
          bootstrap_nodes.size() >=
              m_impl->options.m_bootstrap_min_samples &&
          stationary;
      bool moving_bootstrap = false;
      if ( !static_ready &&
           m_impl->options.m_enable_moving_bootstrap )
      {
        const std::optional<std::size_t> recent_begin = recentBootstrapBegin(
            bootstrap_nodes, m_impl->options.m_bootstrap_min_duration_ns,
            m_impl->options.m_bootstrap_min_samples );
        if ( recent_begin.has_value() )
        {
          const BootstrapStats recent_stats =
              bootstrapStats( bootstrap_nodes, *recent_begin );
          if ( recent_stats.m_acc_mean.norm() >
               std::numeric_limits<double>::epsilon() )
          {
            bootstrap_stats  = recent_stats;
            moving_bootstrap = true;
          }
        }
      }
      const bool ready = static_ready || moving_bootstrap;
      if ( !ready )
      {
        VioUpdateTransaction transaction( m_impl->m_state );
        m_impl->m_state->m_continuity_anchor = measurement.m_timestamp;
        if ( *bootstrap_duration_ns >=
             m_impl->options.m_bootstrap_timeout_ns )
        {
          m_impl->m_state->m_bootstrap_nodes.clear();
          result.status  = UpdateStatus::kFailed;
          result.message = "static bootstrap timed out";
        }
        else
        {
          m_impl->m_state->m_bootstrap_nodes = std::move( bootstrap_nodes );
          result.status                      = UpdateStatus::kInitializing;
          result.message                     = "collecting static bootstrap evidence";
          if ( !stageUnsupportedSpanForCurrentTimestamp() )
          {
            return finalizePreStagingHardResult(
                UpdateStatus::kFailed,
                "unsupported visual span is not representable" );
          }
        }
        transaction.commit();
        return result;
      }

      if ( !moving_bootstrap )
      {
        bootstrap_bias.m_gyr_radps = bootstrap_stats.m_gyr_mean;
      }
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
        transaction.commit();
        return result;
      }
    }

    auto       pending_seed_obs = m_impl->m_state->m_pending_seed_obs;
    const auto retainBootstrapAndReturn =
        [ & ]( std::string message ) -> VioUpdateResult {
      VioUpdateTransaction transaction( m_impl->m_state );
      m_impl->m_state->m_bootstrap_nodes   = bootstrap_nodes;
      m_impl->m_state->m_continuity_anchor = measurement.m_timestamp;
      m_impl->m_state->m_pending_seed_obs  = pending_seed_obs;
      result.status                        = UpdateStatus::kInitializing;
      result.message                       = std::move( message );
      if ( !stageUnsupportedSpanForCurrentTimestamp() )
      {
        return finalizePreStagingHardResult(
            UpdateStatus::kFailed,
            "unsupported visual span is not representable" );
      }
      transaction.commit();
      return result;
    };

    std::uint32_t num_shared              = 0;
    std::uint32_t num_mapped_observations = 0;
    std::uint32_t num_disparity           = 0;
    for ( const StereoObservation& observation : measurement.m_observations )
    {
      const bool mapped =
          m_impl->m_state->m_landmarks_w.find( observation.id ) !=
          m_impl->m_state->m_landmarks_w.end();
      if ( mapped )
      {
        ++num_mapped_observations;
      }
      // E3 experiment: zero-disparity observations cannot constrain PnP or
      // BA, so exclude them from the shared-overlap accounting that gates PnP
      // and low-connectivity.
      if ( observation.disparity_px > 0.0 )
      {
        ++num_disparity;  // diag: frontend stereo matching health, independent
                          // of landmark-table overlap (num_shared below)
        if ( mapped )
        {
          ++num_shared;
        }
      }
    }
    result.diagnostics.num_shared              = num_shared;
    result.diagnostics.num_mapped_observations = num_mapped_observations;
    result.diagnostics.num_disparity           = num_disparity;

    const bool active_segment = m_impl->m_state->m_initialized;
    const bool full_visual_support =
        !active_segment ||
        static_cast<int>( num_shared ) >= m_impl->options.min_pnp_inliers;
    const bool low_visual_support =
        active_segment && !full_visual_support;
    std::int64_t next_visual_coast_duration_ns = 0;

    bool clear_pending_seed_obs =
        m_impl->m_state->m_initialized && full_visual_support;
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
    // 首段跨帧累积播种。enable_accumulated_seed 时,
    // 未初始化帧的视差观测跨帧累积进 pending_seed_obs (最新覆盖), 累积到
    // min_seed_observations 个唯一 track 后用合成 measurement 播种 —— 只
    // 用于 Gate F (首段), V2_03 启动段暗帧 1-7 obs/帧饿死的突破手段。
    VioMeasurement        accumulated_measurement = measurement;
    const VioMeasurement* effective_measurement   = &measurement;
    const auto            accumulate              = [ & ]() -> bool {
      for ( const StereoObservation& obs : measurement.m_observations )
      {
        if ( obs.disparity_px > 0.0 )
        {
          pending_seed_obs[ obs.id ] = obs;  // latest wins
        }
      }
      if ( static_cast<int>( pending_seed_obs.size() ) <
           m_impl->options.min_seed_observations )
      {
        return false;
      }
      accumulated_measurement.m_observations.clear();
      accumulated_measurement.m_observations.reserve(
          pending_seed_obs.size() );
      for ( const auto& [ id, obs ] : pending_seed_obs )
      {
        (void)id;
        accumulated_measurement.m_observations.push_back( obs );
      }
      effective_measurement = &accumulated_measurement;
      return true;
    };

    result.diagnostics.low_connectivity =
        m_impl->m_state->m_initialized && full_visual_support &&
        static_cast<int>( num_shared ) < m_impl->options.min_shared_landmarks;

    if ( !m_impl->m_state->m_initialized )
    {
      const int stereo_count =
          static_cast<int>( m_impl->countStereoObservations( measurement ) );
      if ( stereo_count >= m_impl->options.min_seed_observations )
      {
        clear_pending_seed_obs = true;
      }
      else if ( m_impl->options.enable_accumulated_seed )
      {
        if ( !accumulate() )
        {
          return retainBootstrapAndReturn(
              "accumulating seed observations (first segment)" );
        }
      }
      else
      {
        return retainBootstrapAndReturn(
            "insufficient observations to seed first segment" );
      }
    }

    std::vector<LandmarkId> frame_culled;

    VioUpdateTransaction transaction( m_impl->m_state );
    m_impl->m_state->m_pending_seed_obs = pending_seed_obs;
    if ( clear_pending_seed_obs )
    {
      m_impl->m_state->m_pending_seed_obs.clear();
    }
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
        const auto            pim       = m_impl->preintegrate( normalized_imu,
                                                                predecessor.m_bias );
        const gtsam::NavState predicted = pim.predict(
            gtsam::NavState( toPose3( predecessor.m_T_W_B ),
                             predecessor.m_v_W_B ),
            toGtsamBias( predecessor.m_bias ) );
        propagated_T_W_B = toIsometry( predicted.pose() );
        propagated_v_W_B = predicted.v();
        propagated_bias  = predecessor.m_bias;
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
      if ( !m_impl->seedRoot( bootstrap_T_W_B, Eigen::Vector3d::Zero(),
                              bootstrap_bias, *effective_measurement,
                              result.diagnostics.probe_rejected_block_n,
                              result.diagnostics.probe_new_lm_n ) )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kRejected,
            "failed to backproject landmark on first frame" );
      }
      m_impl->m_state->m_pending_seed_obs.clear();
      m_impl->m_state->m_bootstrap_nodes.clear();
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
      candidate.m_T_W_B                   = guess_T_W_B;
      candidate.m_v_W_B                   = propagated_v_W_B;
      candidate.m_bias                    = propagated_bias;
      candidate.m_predecessor_frame_index =
          m_impl->m_state->m_window.back().m_frame_index;
      candidate.m_imu = normalized_imu;

      if ( m_impl->options.enable_pnp_init &&
           static_cast<int>( num_shared ) >=
               m_impl->options.min_pnp_inliers )
      {
        const Impl::PnpInitResult pnp =
            m_impl->tryPnpInit( measurement, guess_T_W_B );
        if ( pnp.success )
        {
          candidate.m_T_W_B = pnp.T_W_B;

          // Map inlier indices → shared LandmarkIds (same scan order as
          // tryPnpInit: zero-disparity observations skipped there), then drop
          // shared outliers; keep new ids.
          std::vector<LandmarkId> shared_ids;
          shared_ids.reserve( static_cast<std::size_t>( num_shared ) );
          for ( const StereoObservation& observation :
                measurement.m_observations )
          {
            if ( observation.disparity_px > 0.0 &&
                 m_impl->m_state->m_landmarks_w.find( observation.id ) !=
                     m_impl->m_state->m_landmarks_w.end() )
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
          filtered.reserve( candidate.m_observations.size() );
          for ( const StereoObservation& observation :
                candidate.m_observations )
          {
            const bool is_shared =
                m_impl->m_state->m_landmarks_w.find( observation.id ) !=
                m_impl->m_state->m_landmarks_w.end();
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
          candidate.m_observations = std::move( filtered );

          result.diagnostics.pnp_success = true;
          result.diagnostics.pnp_inliers =
              static_cast<std::uint32_t>( pnp.inlier_indices.size() );
        }
      }

      if ( !isFinite( candidate.m_T_W_B ) )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed, "non-finite pose initial value" );
      }
      // Track history uses the full validated measurement, including shared
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
            toPose3( candidate.m_T_W_B ) * m_impl->body_P_sensor;
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
            m_impl->backprojectWorld( candidate.m_T_W_B, observation );
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
              m_impl->backprojectWorld( candidate.m_T_W_B, observation );
          if ( !point_W.has_value() )
          {
            continue;
          }
          const gtsam::Pose3 T_W_left =
              toPose3( candidate.m_T_W_B ) * m_impl->body_P_sensor;
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

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               values;
    std::uint64_t               prior_key     = 0;
    std::uint32_t               num_landmarks = 0;
    VioGraphInfo                vio_graph_info;
    try
    {
      m_impl->buildGraph( graph, values, prior_key, num_landmarks, nullptr,
                          &vio_graph_info );
    }
    catch ( const std::exception& exception )
    {
      return finalizePostStagingHardResult(
          UpdateStatus::kFailed,
          std::string( "graph build failed: " ) + exception.what() );
    }
    result.diagnostics.num_landmarks = num_landmarks;
    result.diagnostics.prior_key     = prior_key;
    result.diagnostics.window_size =
        static_cast<std::uint32_t>( m_impl->m_state->m_window.size() );
    result.diagnostics.reproj_rms_before_px =
        visualReprojRms( graph, values );
    if ( !std::isfinite( result.diagnostics.reproj_rms_before_px ) )
    {
      return finalizePostStagingHardResult(
          UpdateStatus::kFailed,
          "initial graph reprojection error is non-finite" );
    }
    const bool graph_has_visual_factors =
        vio_graph_info.m_visual_factors > 0U;
    const std::uint32_t cheirality_before = std::max(
        countCheiralityFactors(
            graph, values, m_impl->calibration.fxPixels() ),
        countBehindCameraLandmarks( m_impl->m_state->m_window, m_impl->m_state->m_landmarks_w,
                                    m_impl->body_P_sensor, values ) );
    std::unordered_map<std::uint64_t, Eigen::Vector3d> poses_before;
    for ( const WindowFrame& frame : m_impl->m_state->m_window )
    {
      if ( frame.m_frame_index != m_impl->m_state->m_window.back().m_frame_index )
      {
        poses_before.emplace( frame.m_frame_index, frame.m_T_W_B.translation() );
      }
    }

    GraphSolveResult solved = m_impl->solveGraph( graph, values );
    if ( std::holds_alternative<GraphSolveError>( solved ) )
    {
      const GraphSolveError& error = std::get<GraphSolveError>( solved );
      result.diagnostics.lm_iterations += error.m_lm_iterations;
      switch ( error.m_code )
      {
        case GraphSolveErrorCode::kIndeterminant:
          return finalizePostStagingHardResult(
              UpdateStatus::kFailed,
              std::string( "indeterminant linear system: " ) +
                  error.m_detail );
        case GraphSolveErrorCode::kOptimizer:
          return finalizePostStagingHardResult(
              UpdateStatus::kFailed,
              std::string( "optimizer exception: " ) + error.m_detail );
      }
      return finalizePostStagingHardResult(
          UpdateStatus::kFailed, "unknown graph solve error" );
    }
    GraphSolveSuccess solve_success =
        std::get<GraphSolveSuccess>( std::move( solved ) );
    result.diagnostics.lm_iterations += solve_success.m_lm_iterations;
    gtsam::Values optimized = std::move( solve_success.m_values );

    for ( const WindowFrame& frame : m_impl->m_state->m_window )
    {
      if ( !optimized.exists( X( frame.m_frame_index ) ) ||
           !optimized.exists( V( frame.m_frame_index ) ) ||
           !optimized.exists( B( frame.m_frame_index ) ) )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed,
            "optimized values missing a navigation state" );
      }
      const Eigen::Isometry3d T_W_B =
          toIsometry( optimized.at<gtsam::Pose3>( X( frame.m_frame_index ) ) );
      const Eigen::Vector3d v_W_B =
          optimized.at<gtsam::Vector3>( V( frame.m_frame_index ) );
      const gtsam::imuBias::ConstantBias bias =
          optimized.at<gtsam::imuBias::ConstantBias>(
              B( frame.m_frame_index ) );
      if ( !isFinite( T_W_B ) || !v_W_B.allFinite() ||
           !bias.vector().allFinite() )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed, "non-finite optimized navigation state" );
      }
    }

    double                max_shift_m     = 0.0;
    constexpr std::size_t kProbeShiftTopK = 3;
    if ( m_impl->options.enable_probe_b )
    {
      result.diagnostics.probe_shift_top.reserve( kProbeShiftTopK );
    }
    for ( WindowFrame& frame : m_impl->m_state->m_window )
    {
      const Eigen::Isometry3d T_W_B =
          toIsometry( optimized.at<gtsam::Pose3>( X( frame.m_frame_index ) ) );
      const auto before_it = poses_before.find( frame.m_frame_index );
      if ( before_it != poses_before.end() )
      {
        const double dt_m =
            ( T_W_B.translation() - before_it->second ).norm();
        max_shift_m = std::max( max_shift_m, dt_m );
        if ( m_impl->options.enable_probe_b )
        {
          considerShiftTopK( result.diagnostics.probe_shift_top,
                             frame.m_frame_index, dt_m, kProbeShiftTopK );
        }
      }
      frame.m_T_W_B = T_W_B;
      frame.m_v_W_B = optimized.at<gtsam::Vector3>(
          V( frame.m_frame_index ) );
      frame.m_bias = toImuBias(
          optimized.at<gtsam::imuBias::ConstantBias>(
              B( frame.m_frame_index ) ) );
    }

    // E13: record the BA-refined pose of every stereo observation of the new
    // frame, so the hang-distance gate keeps measuring after this frame
    // leaves the window. (Older frames' entries were written when each was
    // the back frame, with the pose refined up to that point — later BA
    // shifts are sub-cm, negligible against meter-scale gates.)
    {
      const Eigen::Isometry3d& T_W_B = m_impl->m_state->m_window.back().m_T_W_B;
      for ( const StereoObservation& observation :
            m_impl->m_state->m_window.back().m_observations )
      {
        if ( observation.disparity_px > 0.0 )
        {
          m_impl->m_state->m_T_W_B_last_stereo[ observation.id ] = T_W_B;
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

    for ( auto& [ id, point_W ] : m_impl->m_state->m_landmarks_w )
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
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed, "non-finite optimized landmark" );
      }
    }

    result.diagnostics.num_cheirality = cheirality_before;

    std::uint32_t outliers_culled        = 0;
    std::uint32_t outliers_culled_unique = 0;
    std::uint32_t culled_round           = 0;
    if ( graph_has_visual_factors )
    {
      culled_round = m_impl->runCheiralityAndMeanCull(
          optimized, graph, frame_culled, outliers_culled,
          outliers_culled_unique, result.diagnostics.num_cheirality );
    }

    // Full-graph RMS on LM₁ graph/values (includes just-culled ids) =
    // pre-cull quality; contract unchanged by multi-round reopt.
    result.diagnostics.reproj_rms_after_px =
        visualReprojRms( graph, optimized );
    // Slice ④ after_cull initial value (final when reopt is skipped / fails).
    double after_cull = result.diagnostics.reproj_rms_after_px;
    if ( m_impl->options.enable_outlier_cull &&
         graph_has_visual_factors )
    {
      after_cull = visualReprojRmsSkippingMissingLandmarks(
          graph, optimized, m_impl->m_state->m_landmarks_w );
    }

    std::uint32_t rounds               = 0;
    bool          outlier_reopt_failed = false;
    while ( graph_has_visual_factors &&
            m_impl->options.enable_outlier_reopt &&
            rounds <
                static_cast<std::uint32_t>( m_impl->options.max_outlier_reopts ) &&
            culled_round >= 4U )
    {
      // Snapshot: failed round rolls back to pre-round window/landmarks.
      const auto window_snap    = m_impl->m_state->m_window;
      const auto landmarks_snap = m_impl->m_state->m_landmarks_w;

      gtsam::NonlinearFactorGraph g_r;
      gtsam::Values               v_r;
      std::uint64_t               prior_key_r = 0;
      std::uint32_t               n_lm_r      = 0;
      VioGraphInfo                reopt_vio_info;
      try
      {
        m_impl->buildGraph( g_r, v_r, prior_key_r, n_lm_r,
                            &optimized, &reopt_vio_info );
      }
      catch ( const std::exception& exception )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed,
            std::string( "reopt graph build failed: " ) +
                exception.what() );
      }
      (void)prior_key_r;
      (void)n_lm_r;

      GraphSolveResult reopt_solved = m_impl->solveGraph( g_r, v_r );
      if ( std::holds_alternative<GraphSolveError>( reopt_solved ) )
      {
        m_impl->m_state->m_window      = window_snap;
        m_impl->m_state->m_landmarks_w = landmarks_snap;
        outlier_reopt_failed           = true;
        break;
      }

      GraphSolveSuccess reopt_success =
          std::get<GraphSolveSuccess>( std::move( reopt_solved ) );
      gtsam::Values optimized_r = std::move( reopt_success.m_values );
      try
      {
        for ( const WindowFrame& frame : m_impl->m_state->m_window )
        {
          if ( !optimized_r.exists( X( frame.m_frame_index ) ) ||
               !optimized_r.exists( V( frame.m_frame_index ) ) ||
               !optimized_r.exists( B( frame.m_frame_index ) ) )
          {
            throw std::runtime_error(
                "optimized values missing a navigation state" );
          }
          const Eigen::Isometry3d T_W_B = toIsometry(
              optimized_r.at<gtsam::Pose3>( X( frame.m_frame_index ) ) );
          const Eigen::Vector3d v_W_B =
              optimized_r.at<gtsam::Vector3>( V( frame.m_frame_index ) );
          const gtsam::imuBias::ConstantBias bias =
              optimized_r.at<gtsam::imuBias::ConstantBias>(
                  B( frame.m_frame_index ) );
          if ( !isFinite( T_W_B ) || !v_W_B.allFinite() ||
               !bias.vector().allFinite() )
          {
            throw std::runtime_error(
                "non-finite optimized navigation state" );
          }
        }

        for ( WindowFrame& frame : m_impl->m_state->m_window )
        {
          frame.m_T_W_B = toIsometry(
              optimized_r.at<gtsam::Pose3>( X( frame.m_frame_index ) ) );
          frame.m_v_W_B = optimized_r.at<gtsam::Vector3>(
              V( frame.m_frame_index ) );
          frame.m_bias = toImuBias(
              optimized_r.at<gtsam::imuBias::ConstantBias>(
                  B( frame.m_frame_index ) ) );
        }

        for ( auto& [ id, point_W ] : m_impl->m_state->m_landmarks_w )
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
            reopt_success.m_lm_iterations;
        optimized      = optimized_r;
        vio_graph_info = reopt_vio_info;
        ++rounds;
        after_cull   = visualReprojRms( g_r, optimized_r );
        culled_round = m_impl->runCheiralityAndMeanCull(
            optimized_r, g_r, frame_culled, outliers_culled,
            outliers_culled_unique, result.diagnostics.num_cheirality );
      }
      catch ( const gtsam::IndeterminantLinearSystemException& )
      {
        m_impl->m_state->m_window      = window_snap;
        m_impl->m_state->m_landmarks_w = landmarks_snap;
        outlier_reopt_failed           = true;
        break;
      }
      catch ( const std::exception& )
      {
        m_impl->m_state->m_window      = window_snap;
        m_impl->m_state->m_landmarks_w = landmarks_snap;
        outlier_reopt_failed           = true;
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
      try
      {
        m_impl->buildGraph( probe_graph, probe_values, probe_prior_key,
                            probe_num_lm );
      }
      catch ( const std::exception& exception )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed,
            std::string( "probe graph build failed: " ) +
                exception.what() );
      }
      (void)probe_prior_key;
      (void)probe_num_lm;
      fillProbeLandmarkResiduals( probe_graph, probe_values,
                                  result.diagnostics );
    }

    m_impl->m_state->m_initialized = true;

    result.diagnostics.num_retained_observations =
        static_cast<std::uint32_t>(
            m_impl->m_state->m_window.back().m_observations.size() );
    result.diagnostics.num_seeded_landmarks =
        result.diagnostics.probe_new_lm_n;
    result.diagnostics.num_current_visual_factors =
        vio_graph_info.m_current_visual_factors;
    result.diagnostics.num_current_mono_visual_factors =
        vio_graph_info.m_current_mono_visual_factors;
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
    result.status                     = UpdateStatus::kOk;
    const WindowFrame& estimate_frame = m_impl->m_state->m_window.back();
    result.estimate                   = VioEstimate{
                          .timestamp    = measurement.m_timestamp,
                          .T_W_B        = estimate_frame.m_T_W_B,
                          .m_v_W_B      = estimate_frame.m_v_W_B,
                          .m_bias       = estimate_frame.m_bias,
                          .m_segment_id = m_impl->m_state->m_segment_id };
    result.diagnostics.m_vio           = m_impl->makeVioDiagnostics( vio_graph_info );
    m_impl->m_state->m_vio_diagnostics = result.diagnostics.m_vio;
    transaction.commit();
    return result;
  }

}  // namespace phad::estimator
