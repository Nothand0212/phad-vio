#include "phad/estimator/stereo_vo_estimator.hpp"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
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

#include "phad/estimator/internal/gyro_bias_initial_value.hpp"
#include "phad/estimator/internal/gyro_rotation_factor.hpp"
#include "phad/estimator/internal/stereo_vo_update_transaction.hpp"

namespace phad::estimator
{
  namespace
  {

    // GTSAM 4.3 exports uppercase Symbol helpers (X/L); older docs used x/l.
    using gtsam::symbol_shorthand::L;
    using gtsam::symbol_shorthand::X;
    using internal::deriveGyroNoiseScales;
    using internal::GyroBiasInitialKind;
    using internal::GyroFrameState;
    using internal::GyroIntervalError;
    using internal::GyroNoiseScales;
    using internal::GyroRotationFactor;
    using internal::preintegrateGyroInterval;
    using internal::selectGyroBiasInitialValue;
    using internal::StereoVoUpdateTransaction;
    using internal::ValidatedGyroInterval;
    using internal::validateGyroInterval;
    using internal::WindowFrame;

    constexpr double      kGyroBiasLinearizationDomain = 1e-3;
    constexpr std::size_t kMaxFixedPimRounds           = 3U;

    [[nodiscard]] gtsam::Key G( std::uint64_t index )
    {
      return gtsam::Symbol( 'g', index );
    }

    struct GyroGraphInfo
    {
      std::uint32_t                                      m_rotation_factors = 0;
      std::uint32_t                                      m_rw_factors       = 0;
      std::uint32_t                                      m_root_priors      = 0;
      std::unordered_map<std::uint64_t, Eigen::Vector3d> m_bias_hats;
    };

    enum class FixedPimSolveErrorCode
    {
      kIndeterminant,
      kOptimizer,
      kGraphBuild,
      kGyroIntegrity,
      kRelinearizationLimit,
    };

    struct FixedPimSolveError
    {
      FixedPimSolveErrorCode m_code;
      std::string            m_detail;
      std::uint32_t          m_lm_iterations = 0;
      std::uint32_t          m_extra_rounds  = 0;
    };

    struct FixedPimSolveSuccess
    {
      gtsam::Values m_values;
      std::uint32_t m_lm_iterations = 0;
      std::uint32_t m_extra_rounds  = 0;
    };

    using FixedPimSolveResult =
        std::variant<FixedPimSolveSuccess, FixedPimSolveError>;

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
          for ( const StereoObservation& observation : frame.m_observations )
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

  struct StereoVoEstimator::Impl
  {
    camera::RectifiedStereoCalibration             calibration;
    EstimatorOptions                               options;
    gtsam::Cal3_S2Stereo::shared_ptr               K;
    gtsam::Pose3                                   body_P_sensor;
    gtsam::SharedNoiseModel                        stereo_noise;
    gtsam::SharedNoiseModel                        prior_noise;
    std::unique_ptr<internal::StereoVoUpdateState> m_state;

    explicit Impl( camera::RectifiedStereoCalibration calibration_in,
                   EstimatorOptions                   options_in )
        : calibration( std::move( calibration_in ) ), options( std::move( options_in ) ), K( std::make_shared<gtsam::Cal3_S2Stereo>( calibration.fxPixels(), calibration.fyPixels(), 0.0, calibration.cxPixels(), calibration.cyPixels(), calibration.baselineM() ) ), body_P_sensor( toPose3( toIsometry( calibration.T_B_left_rectified() ) ) ), stereo_noise( makeStereoNoise( options ) ), prior_noise( makePriorNoise( options ) ), m_state( std::make_unique<internal::StereoVoUpdateState>() )
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
      if ( options.m_gyro_bias.has_value() )
      {
        const GyroBiasOptions& gyro = *options.m_gyro_bias;
        const double           prior_variance =
            gyro.m_prior_sigma_radps * gyro.m_prior_sigma_radps;
        if ( !std::isfinite( gyro.m_gyr_nd ) || gyro.m_gyr_nd <= 0.0 ||
             !std::isfinite( gyro.m_gyr_rw ) || gyro.m_gyr_rw <= 0.0 ||
             !gyro.m_prior_mean_radps.allFinite() ||
             !std::isfinite( gyro.m_prior_sigma_radps ) ||
             gyro.m_prior_sigma_radps <= 0.0 ||
             !std::isfinite( prior_variance ) || prior_variance <= 0.0 )
        {
          throw std::invalid_argument(
              "EstimatorOptions.m_gyro_bias must contain finite positive "
              "noise scales and a finite prior" );
        }
      }
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
    bool seedSegment( const Eigen::Isometry3d&   anchor_T_W_B,
                      const KeyframeMeasurement& measurement,
                      std::uint32_t&             probe_rejected_block_n,
                      std::uint32_t&             probe_new_lm_n )
    {
      m_state->m_landmarks_w.clear();

      WindowFrame candidate;
      candidate.m_frame_index  = m_state->m_next_frame_index;
      candidate.m_timestamp    = measurement.timestamp;
      candidate.m_observations = measurement.observations;
      candidate.m_T_W_B        = anchor_T_W_B;
      candidate.m_is_keyframe  = true;  // seed/re-anchor frames are keyframes

      for ( const StereoObservation& observation : measurement.observations )
      {
        if ( options.block_culled_rebirth &&
             m_state->m_culled_ids.find( observation.id ) != m_state->m_culled_ids.end() )
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
        m_state->m_track_times[ observation.id ].push_back( measurement.timestamp );
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

    [[nodiscard]] Eigen::Isometry3d poseInitialValue() const
    {
      if ( !m_state->m_T_W_B_last_accepted.has_value() )
      {
        return Eigen::Isometry3d::Identity();
      }
      if ( !options.use_constant_velocity_init ||
           !m_state->m_T_W_B_prev_accepted.has_value() )
      {
        return *m_state->m_T_W_B_last_accepted;
      }
      const Eigen::Isometry3d& T_prev     = *m_state->m_T_W_B_last_accepted;
      const Eigen::Isometry3d& T_prevprev = *m_state->m_T_W_B_prev_accepted;
      const Eigen::Isometry3d  predicted =
          T_prev * ( T_prevprev.inverse() * T_prev );
      if ( !isFinite( predicted ) )
      {
        return T_prev;
      }
      return predicted;
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
      for ( const WindowFrame& frame : m_state->m_window )
      {
        for ( const StereoObservation& observation : frame.m_observations )
        {
          if ( observation.disparity_px > 0.0 )
          {
            ++counts[ observation.id ];
          }
        }
      }
      return counts;
    }

    [[nodiscard]] bool resetEvictedGyroLinks()
    {
      if ( !options.m_gyro_bias.has_value() )
      {
        return false;
      }

      std::unordered_set<std::uint64_t> surviving_frames;
      surviving_frames.reserve( m_state->m_window.size() );
      for ( const WindowFrame& frame : m_state->m_window )
      {
        surviving_frames.insert( frame.m_frame_index );
      }

      bool                                             reset_any = false;
      std::unordered_map<std::uint64_t, std::uint64_t> component_by_frame;
      component_by_frame.reserve( m_state->m_window.size() );
      for ( WindowFrame& frame : m_state->m_window )
      {
        if ( !frame.m_gyro.has_value() )
        {
          throw std::runtime_error(
              "gyro-enabled window frame is missing bias state" );
        }
        GyroFrameState& gyro = *frame.m_gyro;
        if ( !gyro.m_predecessor_frame_index.has_value() )
        {
          component_by_frame.emplace( frame.m_frame_index,
                                      gyro.m_component_id );
          continue;
        }

        const std::uint64_t predecessor =
            *gyro.m_predecessor_frame_index;
        const auto component_it = component_by_frame.find( predecessor );
        if ( component_it != component_by_frame.end() )
        {
          gyro.m_component_id = component_it->second;
          component_by_frame.emplace( frame.m_frame_index,
                                      gyro.m_component_id );
          continue;
        }
        if ( surviving_frames.contains( predecessor ) )
        {
          throw std::runtime_error(
              "gyro predecessor does not precede its linked state" );
        }

        const auto initial = selectGyroBiasInitialValue(
            GyroBiasInitialKind::kComponentRoot, std::nullopt, std::nullopt,
            options.m_gyro_bias->m_prior_mean_radps );
        if ( !std::holds_alternative<Eigen::Vector3d>( initial ) )
        {
          throw std::runtime_error(
              "gyro bias initial value is non-finite" );
        }
        gyro.m_bias_radps   = std::get<Eigen::Vector3d>( initial );
        gyro.m_component_id = m_state->m_next_gyro_component_id++;
        gyro.m_predecessor_frame_index.reset();
        gyro.m_interval.reset();
        component_by_frame.emplace( frame.m_frame_index,
                                    gyro.m_component_id );
        reset_any = true;
      }
      return reset_any;
    }

    void buildGraph( gtsam::NonlinearFactorGraph& graph,
                     gtsam::Values&               values,
                     std::uint64_t&               prior_key_out,
                     std::uint32_t&               num_landmarks_out,
                     const gtsam::Values*         round_start   = nullptr,
                     GyroGraphInfo*               gyro_info_out = nullptr ) const
    {
      graph.resize( 0 );
      values.clear();
      num_landmarks_out = 0;
      GyroGraphInfo gyro_info;
      if ( m_state->m_window.empty() )
      {
        prior_key_out = 0;
        if ( gyro_info_out != nullptr )
        {
          *gyro_info_out = std::move( gyro_info );
        }
        return;
      }

      const std::uint64_t oldest = m_state->m_window.front().m_frame_index;
      prior_key_out              = oldest;
      for ( const WindowFrame& frame : m_state->m_window )
      {
        const gtsam::Key   pose_key = X( frame.m_frame_index );
        const gtsam::Pose3 pose =
            round_start != nullptr && round_start->exists( pose_key )
                ? round_start->at<gtsam::Pose3>( pose_key )
                : toPose3( frame.m_T_W_B );
        values.insert( pose_key, pose );
      }
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          X( oldest ), toPose3( m_state->m_window.front().m_T_W_B ), prior_noise );

      if ( options.m_gyro_bias.has_value() )
      {
        const GyroBiasOptions& gyro_options = *options.m_gyro_bias;
        for ( const WindowFrame& frame : m_state->m_window )
        {
          if ( !frame.m_gyro.has_value() )
          {
            throw std::runtime_error(
                "gyro-enabled window frame is missing bias state" );
          }
          const gtsam::Key      bias_key = G( frame.m_frame_index );
          const Eigen::Vector3d bias =
              round_start != nullptr && round_start->exists( bias_key )
                  ? round_start->at<gtsam::Vector3>( bias_key )
                  : frame.m_gyro->m_bias_radps;
          if ( !bias.allFinite() )
          {
            throw std::runtime_error(
                "gyro bias initial value is non-finite" );
          }
          values.insert( bias_key, bias );
        }

        for ( const WindowFrame& frame : m_state->m_window )
        {
          const GyroFrameState& gyro = *frame.m_gyro;
          if ( !gyro.m_predecessor_frame_index.has_value() )
          {
            graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
                G( frame.m_frame_index ), gyro_options.m_prior_mean_radps,
                gtsam::noiseModel::Isotropic::Sigma(
                    3, gyro_options.m_prior_sigma_radps ) );
            ++gyro_info.m_root_priors;
            continue;
          }
          if ( !gyro.m_interval.has_value() )
          {
            throw std::runtime_error(
                "linked gyro state is missing its validated interval" );
          }

          const std::uint64_t predecessor =
              *gyro.m_predecessor_frame_index;
          const auto predecessor_it = std::find_if(
              m_state->m_window.begin(), m_state->m_window.end(),
              [ predecessor ]( const WindowFrame& candidate ) {
                return candidate.m_frame_index == predecessor;
              } );
          if ( predecessor_it == m_state->m_window.end() ||
               !predecessor_it->m_gyro.has_value() ||
               predecessor_it->m_gyro->m_component_id != gyro.m_component_id )
          {
            throw std::runtime_error(
                "linked gyro predecessor is not in the current component" );
          }

          const auto scales_result = deriveGyroNoiseScales(
              gyro.m_interval->m_duration_ns, gyro_options.m_gyr_nd,
              gyro_options.m_gyr_rw, gyro_options.m_prior_sigma_radps );
          if ( !std::holds_alternative<GyroNoiseScales>( scales_result ) )
          {
            throw std::invalid_argument(
                std::get<GyroIntervalError>( scales_result ).m_detail );
          }
          const GyroNoiseScales& scales =
              std::get<GyroNoiseScales>( scales_result );
          const Eigen::Vector3d bias_hat =
              values.at<gtsam::Vector3>( G( predecessor ) );
          const auto pim = preintegrateGyroInterval(
              *gyro.m_interval, bias_hat, gyro_options.m_gyr_nd );
          graph.emplace_shared<GyroRotationFactor>(
              X( predecessor ), X( frame.m_frame_index ), G( predecessor ),
              pim );
          graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
              G( predecessor ), G( frame.m_frame_index ),
              Eigen::Vector3d::Zero(),
              gtsam::noiseModel::Isotropic::Sigma( 3, scales.m_rw_sigma ) );
          ++gyro_info.m_rotation_factors;
          ++gyro_info.m_rw_factors;
          gyro_info.m_bias_hats.emplace( predecessor, bias_hat );
        }
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
          // Slice ⑦: zero-disparity observations are not stereo measurements
          // — building a StereoFactor from them would project a degenerate
          // right pixel.
          if ( observation.disparity_px > 0.0 )
          {
            graph.emplace_shared<
                gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
                toStereoPoint( observation ), stereo_noise,
                X( frame.m_frame_index ), L( observation.id ), K,
                body_P_sensor );
          }
        }
      }
      if ( gyro_info_out != nullptr )
      {
        *gyro_info_out = std::move( gyro_info );
      }
    }

    [[nodiscard]] FixedPimSolveResult solveFixedPimGraph(
        gtsam::NonlinearFactorGraph& graph, gtsam::Values& values,
        std::uint64_t& prior_key, std::uint32_t& num_landmarks,
        GyroGraphInfo& gyro_info ) const
    {
      std::uint32_t lm_iterations    = 0U;
      std::uint32_t extra_rounds     = 0U;
      std::size_t   completed_rounds = 0U;
      while ( true )
      {
        gtsam::Values optimized;
        try
        {
          gtsam::LevenbergMarquardtOptimizer optimizer( graph, values );
          optimized = optimizer.optimize();
          lm_iterations +=
              static_cast<std::uint32_t>( optimizer.iterations() );
        }
        catch ( const gtsam::IndeterminantLinearSystemException& exception )
        {
          return FixedPimSolveError{
              FixedPimSolveErrorCode::kIndeterminant, exception.what(),
              lm_iterations, extra_rounds };
        }
        catch ( const std::exception& exception )
        {
          return FixedPimSolveError{ FixedPimSolveErrorCode::kOptimizer,
                                     exception.what(), lm_iterations,
                                     extra_rounds };
        }

        bool relinearization_required = false;
        if ( options.m_gyro_bias.has_value() )
        {
          std::unordered_map<std::uint64_t, Eigen::Vector3d> biases;
          biases.reserve( m_state->m_window.size() );
          try
          {
            for ( const WindowFrame& frame : m_state->m_window )
            {
              const gtsam::Key bias_key = G( frame.m_frame_index );
              if ( !optimized.exists( bias_key ) )
              {
                return FixedPimSolveError{
                    FixedPimSolveErrorCode::kGyroIntegrity,
                    "optimized values missing a gyro bias", lm_iterations,
                    extra_rounds };
              }
              const Eigen::Vector3d bias =
                  optimized.at<gtsam::Vector3>( bias_key );
              if ( !bias.allFinite() )
              {
                return FixedPimSolveError{
                    FixedPimSolveErrorCode::kGyroIntegrity,
                    "non-finite optimized gyro bias", lm_iterations,
                    extra_rounds };
              }
              biases.emplace( frame.m_frame_index, bias );
            }
          }
          catch ( const std::exception& exception )
          {
            return FixedPimSolveError{
                FixedPimSolveErrorCode::kGyroIntegrity,
                std::string( "invalid optimized gyro bias: " ) +
                    exception.what(),
                lm_iterations, extra_rounds };
          }

          for ( const auto& [ frame_index, bias_hat ] :
                gyro_info.m_bias_hats )
          {
            const auto bias_it = biases.find( frame_index );
            if ( bias_it == biases.end() )
            {
              return FixedPimSolveError{
                  FixedPimSolveErrorCode::kGyroIntegrity,
                  "optimized values missing a gyro bias", lm_iterations,
                  extra_rounds };
            }
            if ( ( bias_it->second - bias_hat ).cwiseAbs().maxCoeff() >
                 kGyroBiasLinearizationDomain )
            {
              relinearization_required = true;
              break;
            }
          }
        }

        if ( !relinearization_required )
        {
          return FixedPimSolveSuccess{ std::move( optimized ), lm_iterations,
                                       extra_rounds };
        }

        ++completed_rounds;
        if ( completed_rounds >= kMaxFixedPimRounds )
        {
          return FixedPimSolveError{
              FixedPimSolveErrorCode::kRelinearizationLimit,
              "gyro bias relinearization did not converge", lm_iterations,
              extra_rounds };
        }

        const gtsam::Values round_start = optimized;
        try
        {
          buildGraph( graph, values, prior_key, num_landmarks, &round_start,
                      &gyro_info );
        }
        catch ( const std::exception& exception )
        {
          return FixedPimSolveError{
              FixedPimSolveErrorCode::kGraphBuild, exception.what(),
              lm_iterations, extra_rounds };
        }
        ++extra_rounds;
      }
    }

    [[nodiscard]] GyroDiagnostics makeGyroDiagnostics(
        GyroBreakReason break_reason, std::uint32_t extra_rounds ) const
    {
      GyroDiagnostics diagnostics;
      diagnostics.m_break_reason           = break_reason;
      diagnostics.m_relinearization_rounds = extra_rounds;
      std::unordered_set<std::uint64_t> observed_components;
      for ( const WindowFrame& frame : m_state->m_window )
      {
        if ( !frame.m_gyro.has_value() )
        {
          continue;
        }
        if ( frame.m_gyro->m_predecessor_frame_index.has_value() )
        {
          ++diagnostics.m_rotation_factors;
          ++diagnostics.m_rw_factors;
          observed_components.insert( frame.m_gyro->m_component_id );
        }
        else
        {
          ++diagnostics.m_root_priors;
        }
      }

      diagnostics.m_window_biases.reserve( m_state->m_window.size() );
      for ( const WindowFrame& frame : m_state->m_window )
      {
        GyroWindowBias window_bias;
        window_bias.m_frame_index = frame.m_frame_index;
        window_bias.m_timestamp   = frame.m_timestamp;
        if ( frame.m_gyro.has_value() &&
             observed_components.contains( frame.m_gyro->m_component_id ) )
        {
          window_bias.m_bias_radps = frame.m_gyro->m_bias_radps;
        }
        diagnostics.m_window_biases.push_back( std::move( window_bias ) );
      }
      if ( !diagnostics.m_window_biases.empty() )
      {
        diagnostics.m_bias_radps =
            diagnostics.m_window_biases.back().m_bias_radps;
      }
      return diagnostics;
    }

    [[nodiscard]] GyroDiagnostics materializeHardGyroDiagnostics() const
    {
      GyroDiagnostics diagnostics =
          makeGyroDiagnostics( GyroBreakReason::kNone, 0U );
      diagnostics.m_bias_radps.reset();
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
    const auto it = m_impl->m_state->m_track_times.find( id );
    if ( it == m_impl->m_state->m_track_times.end() )
    {
      return {};
    }
    return it->second;
  }

  VioUpdateResult StereoVoEstimator::update(
      const KeyframeMeasurement& measurement,
      const bool                 keyframe )
  {
    VioUpdateResult result;
    result.diagnostics.culled_landmark_ids.clear();
    result.diagnostics.num_observations =
        static_cast<std::uint32_t>( measurement.observations.size() );
    result.diagnostics.segment_id = m_impl->m_state->m_segment_id;
    const auto finalizePreStagingHardResult =
        [ & ]( UpdateStatus status, std::string message ) -> VioUpdateResult {
      result.status  = status;
      result.message = std::move( message );
      if ( m_impl->options.m_gyro_bias.has_value() )
      {
        result.diagnostics.m_gyro =
            m_impl->materializeHardGyroDiagnostics();
      }
      return result;
    };

    if ( measurement.observations.empty() )
    {
      return finalizePreStagingHardResult( UpdateStatus::kRejected,
                                           "empty observations" );
    }
    // Slice ⑦: disparity_px == 0 is legal — it marks "stereo failed, no
    // depth"; negative or non-finite is not.
    for ( const StereoObservation& observation : measurement.observations )
    {
      if ( observation.disparity_px < 0.0 ||
           !observation.left_pixel.allFinite() ||
           !std::isfinite( observation.disparity_px ) )
      {
        return finalizePreStagingHardResult(
            UpdateStatus::kRejected,
            "non-finite pixel or negative disparity" );
      }
    }
    if ( m_impl->m_state->m_initialized && !m_impl->m_state->m_window.empty() &&
         measurement.timestamp <= m_impl->m_state->m_window.back().m_timestamp )
    {
      return finalizePreStagingHardResult(
          UpdateStatus::kRejected, "timestamp not strictly increasing" );
    }

    std::optional<std::uint64_t>         gyro_predecessor_index;
    std::optional<common::Timestamp>     gyro_predecessor_timestamp;
    std::optional<GyroFrameState>        gyro_predecessor_state;
    std::optional<ValidatedGyroInterval> validated_gyro_interval;
    bool                                 gyro_interval_missing = false;
    bool                                 gyro_interval_gap     = false;
    GyroBreakReason                      gyro_break_reason     = GyroBreakReason::kNone;
    if ( m_impl->options.m_gyro_bias.has_value() )
    {
      if ( !m_impl->m_state->m_window.empty() )
      {
        const WindowFrame& predecessor = m_impl->m_state->m_window.back();
        gyro_predecessor_index         = predecessor.m_frame_index;
        gyro_predecessor_timestamp     = predecessor.m_timestamp;
        gyro_predecessor_state         = predecessor.m_gyro;
      }

      if ( measurement.m_gyro_interval.has_value() )
      {
        const GyroInterval& interval = *measurement.m_gyro_interval;
        gyro_interval_gap            = interval.m_imu_gap;
        if ( !gyro_interval_gap )
        {
          const auto reduced = validateGyroInterval(
              interval.m_samples, interval.m_t_prev, measurement.timestamp );
          if ( !std::holds_alternative<ValidatedGyroInterval>( reduced ) )
          {
            return finalizePreStagingHardResult(
                UpdateStatus::kRejected,
                std::get<GyroIntervalError>( reduced ).m_detail );
          }
          validated_gyro_interval =
              std::get<ValidatedGyroInterval>( reduced );
          const auto scales = deriveGyroNoiseScales(
              validated_gyro_interval->m_duration_ns,
              m_impl->options.m_gyro_bias->m_gyr_nd,
              m_impl->options.m_gyro_bias->m_gyr_rw,
              m_impl->options.m_gyro_bias->m_prior_sigma_radps );
          if ( !std::holds_alternative<GyroNoiseScales>( scales ) )
          {
            return finalizePreStagingHardResult(
                UpdateStatus::kRejected,
                std::get<GyroIntervalError>( scales ).m_detail );
          }
        }
      }
      else
      {
        gyro_interval_missing = true;
      }
    }

    const auto recordEligibleVisualRejection = [ & ]() {
      if ( m_impl->options.m_gyro_bias.has_value() )
      {
        m_impl->m_state->m_eligible_visual_rejected_timestamp =
            measurement.timestamp;
      }
    };

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
        if ( m_impl->m_state->m_landmarks_w.find( observation.id ) !=
             m_impl->m_state->m_landmarks_w.end() )
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
      m_impl->m_state->m_pending_seed_obs.clear();
    }

    const bool overlap_broken = m_impl->m_state->m_initialized && num_shared == 0;

    if ( overlap_broken && !m_impl->options.enable_reanchor )
    {
      recordEligibleVisualRejection();
      result.diagnostics.window_size =
          static_cast<std::uint32_t>( m_impl->m_state->m_window.size() );
      if ( !m_impl->m_state->m_window.empty() )
      {
        result.diagnostics.prior_key = m_impl->m_state->m_window.front().m_frame_index;
      }
      return finalizePreStagingHardResult(
          UpdateStatus::kRejected, "zero shared landmarks with window" );
    }
    // Non-keyframe cannot re-anchor; reject when overlap is broken.
    if ( overlap_broken && !keyframe )
    {
      recordEligibleVisualRejection();
      result.diagnostics.window_size =
          static_cast<std::uint32_t>( m_impl->m_state->m_window.size() );
      if ( !m_impl->m_state->m_window.empty() )
      {
        result.diagnostics.prior_key = m_impl->m_state->m_window.front().m_frame_index;
      }
      return finalizePreStagingHardResult(
          UpdateStatus::kRejected, "zero shared landmarks (non-keyframe)" );
    }
    // Non-keyframe cannot initialise the first segment; reject.
    if ( !m_impl->m_state->m_initialized && !keyframe )
    {
      recordEligibleVisualRejection();
      return finalizePreStagingHardResult(
          UpdateStatus::kRejected, "not initialised (non-keyframe)" );
    }
    // Non-keyframe with too few shared landmarks cannot run PnP; a raw CV
    // guess would pollute the pose chain. Reject (Slice ⑤b gate).
    if ( !keyframe && static_cast<int>( num_shared ) <
                          m_impl->options.min_pnp_inliers )
    {
      recordEligibleVisualRejection();
      result.diagnostics.window_size =
          static_cast<std::uint32_t>( m_impl->m_state->m_window.size() );
      if ( !m_impl->m_state->m_window.empty() )
      {
        result.diagnostics.prior_key = m_impl->m_state->m_window.front().m_frame_index;
      }
      return finalizePreStagingHardResult(
          UpdateStatus::kRejected,
          "insufficient shared landmarks (non-keyframe)" );
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
          m_impl->m_state->m_pending_seed_obs[ obs.id ] = obs;  // latest wins
        }
      }
      if ( static_cast<int>( m_impl->m_state->m_pending_seed_obs.size() ) <
           m_impl->options.min_seed_observations )
      {
        return false;
      }
      accumulated_measurement.timestamp = measurement.timestamp;
      accumulated_measurement.observations.reserve(
          m_impl->m_state->m_pending_seed_obs.size() );
      for ( const auto& [ id, obs ] : m_impl->m_state->m_pending_seed_obs )
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
        recordEligibleVisualRejection();
        result.diagnostics.window_size =
            static_cast<std::uint32_t>( m_impl->m_state->m_window.size() );
        if ( !m_impl->m_state->m_window.empty() )
        {
          result.diagnostics.prior_key = m_impl->m_state->m_window.front().m_frame_index;
        }
        return finalizePreStagingHardResult(
            UpdateStatus::kRejected,
            "insufficient observations to seed new segment" );
      }
      m_impl->m_state->m_pending_seed_obs.clear();
    }
    result.diagnostics.low_connectivity =
        m_impl->m_state->m_initialized &&
        num_shared > 0 &&
        static_cast<int>( num_shared ) < m_impl->options.min_shared_landmarks;

    if ( !m_impl->m_state->m_initialized )
    {
      const int stereo_count =
          static_cast<int>( m_impl->countStereoObservations( measurement ) );
      if ( stereo_count >= m_impl->options.min_seed_observations )
      {
        m_impl->m_state->m_pending_seed_obs.clear();
      }
      else if ( m_impl->options.enable_accumulated_seed )
      {
        if ( !accumulate() )
        {
          recordEligibleVisualRejection();
          return finalizePreStagingHardResult(
              UpdateStatus::kRejected,
              "accumulating seed observations (first segment)" );
        }
      }
      else
      {
        recordEligibleVisualRejection();
        return finalizePreStagingHardResult(
            UpdateStatus::kRejected,
            "insufficient observations to seed first segment" );
      }
    }

    std::vector<LandmarkId> frame_culled;

    // PHAD_M4_ONLINE_BIAS_TRANSACTION_CTOR
    StereoVoUpdateTransaction transaction( m_impl->m_state );
    auto                      finalizePostStagingHardResult =
        [ & ]( UpdateStatus status, std::string message ) -> VioUpdateResult {
      // PHAD_M4_ONLINE_BIAS_ROLLBACK_BEFORE_DIAGNOSTICS
      transaction.rollback();
      result.status                 = status;
      result.message                = std::move( message );
      result.diagnostics.segment_id = m_impl->m_state->m_segment_id;
      result.diagnostics.culled_landmark_ids.clear();
      if ( m_impl->options.m_gyro_bias.has_value() )
      {
        result.diagnostics.m_gyro =
            m_impl->materializeHardGyroDiagnostics();
      }
      return result;
    };
    const auto finalizePostStagingVisualRejection =
        [ & ]( std::string message ) -> VioUpdateResult {
      VioUpdateResult rejected = finalizePostStagingHardResult(
          UpdateStatus::kRejected, std::move( message ) );
      recordEligibleVisualRejection();
      return rejected;
    };

    if ( !m_impl->m_state->m_initialized )
    {
      if ( !m_impl->seedSegment( Eigen::Isometry3d::Identity(),
                                 *effective_measurement,
                                 result.diagnostics.probe_rejected_block_n,
                                 result.diagnostics.probe_new_lm_n ) )
      {
        return finalizePostStagingVisualRejection(
            "failed to backproject landmark on first frame" );
      }
      m_impl->m_state->m_pending_seed_obs.clear();
    }
    else if ( overlap_broken )
    {
      const Eigen::Isometry3d anchor = m_impl->poseInitialValue();
      if ( !isFinite( anchor ) )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed, "non-finite pose initial value" );
      }
      if ( !m_impl->seedSegment( anchor, *effective_measurement,
                                 result.diagnostics.probe_rejected_block_n,
                                 result.diagnostics.probe_new_lm_n ) )
      {
        return finalizePostStagingVisualRejection(
            "failed to backproject landmark on re-anchor frame" );
      }
      m_impl->m_state->m_pending_seed_obs.clear();
      ++m_impl->m_state->m_segment_id;
      result.diagnostics.segment_id = m_impl->m_state->m_segment_id;
    }
    else
    {
      WindowFrame candidate;
      candidate.m_frame_index  = m_impl->m_state->m_next_frame_index;
      candidate.m_timestamp    = measurement.timestamp;
      candidate.m_observations = measurement.observations;
      candidate.m_is_keyframe  = keyframe;  // Slice ⑤c

      const Eigen::Isometry3d guess_T_W_B = m_impl->poseInitialValue();
      candidate.m_T_W_B                   = guess_T_W_B;

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
                measurement.observations )
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
      // CRITICAL: track_times must see the full measurement (including
      // shared outliers masked out of candidate.m_observations).
      for ( const StereoObservation& observation : measurement.observations )
      {
        m_impl->m_state->m_track_times[ observation.id ].push_back(
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

    if ( m_impl->options.m_gyro_bias.has_value() )
    {
      WindowFrame&           current                = m_impl->m_state->m_window.back();
      const GyroBiasOptions& gyro_options           = *m_impl->options.m_gyro_bias;
      const bool             starts_first_component = !gyro_predecessor_index.has_value();
      if ( starts_first_component && validated_gyro_interval.has_value() )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kRejected,
            "gyro interval endpoints do not match pose timestamps" );
      }
      const bool exact_predecessor =
          validated_gyro_interval.has_value() &&
          gyro_predecessor_timestamp.has_value() &&
          validated_gyro_interval->m_t_prev ==
              *gyro_predecessor_timestamp;
      const bool rejected_endpoint =
          !exact_predecessor && validated_gyro_interval.has_value() &&
          m_impl->m_state->m_eligible_visual_rejected_timestamp.has_value() &&
          validated_gyro_interval->m_t_prev ==
              *m_impl->m_state->m_eligible_visual_rejected_timestamp;
      const bool starts_new_component =
          starts_first_component || overlap_broken || gyro_interval_missing ||
          gyro_interval_gap || rejected_endpoint;
      if ( starts_new_component )
      {
        const auto initial = selectGyroBiasInitialValue(
            GyroBiasInitialKind::kComponentRoot,
            gyro_predecessor_state.has_value()
                ? std::optional<Eigen::Vector3d>(
                      gyro_predecessor_state->m_bias_radps )
                : std::nullopt,
            gyro_predecessor_state.has_value()
                ? std::optional<Eigen::Vector3d>(
                      gyro_predecessor_state->m_bias_radps )
                : std::nullopt,
            gyro_options.m_prior_mean_radps );
        if ( !std::holds_alternative<Eigen::Vector3d>( initial ) )
        {
          return finalizePostStagingHardResult(
              UpdateStatus::kFailed,
              "gyro bias initial value is non-finite" );
        }

        GyroFrameState gyro;
        gyro.m_bias_radps   = std::get<Eigen::Vector3d>( initial );
        gyro.m_segment_id   = m_impl->m_state->m_segment_id;
        gyro.m_component_id = m_impl->m_state->m_next_gyro_component_id++;
        current.m_gyro      = std::move( gyro );
        if ( !starts_first_component )
        {
          gyro_break_reason = overlap_broken
                                  ? GyroBreakReason::kSegmentChange
                              : gyro_interval_gap
                                  ? GyroBreakReason::kDeclaredGap
                              : rejected_endpoint
                                  ? GyroBreakReason::kRejectedEndpoint
                                  : GyroBreakReason::kMissingInterval;
        }
      }
      else
      {
        if ( !exact_predecessor || !gyro_predecessor_state.has_value() )
        {
          return finalizePostStagingHardResult(
              UpdateStatus::kRejected,
              "gyro interval endpoints do not match pose timestamps" );
        }

        const auto initial = selectGyroBiasInitialValue(
            GyroBiasInitialKind::kExactLink, std::nullopt,
            gyro_predecessor_state->m_bias_radps,
            gyro_options.m_prior_mean_radps );
        if ( !std::holds_alternative<Eigen::Vector3d>( initial ) )
        {
          return finalizePostStagingHardResult(
              UpdateStatus::kFailed,
              "gyro bias initial value is non-finite" );
        }

        GyroFrameState gyro;
        gyro.m_bias_radps              = std::get<Eigen::Vector3d>( initial );
        gyro.m_segment_id              = m_impl->m_state->m_segment_id;
        gyro.m_component_id            = gyro_predecessor_state->m_component_id;
        gyro.m_predecessor_frame_index = *gyro_predecessor_index;
        gyro.m_interval                = std::move( validated_gyro_interval );
        current.m_gyro                 = std::move( gyro );
      }
      m_impl->m_state->m_eligible_visual_rejected_timestamp.reset();
    }

    // Slice ⑤c: Basalt-style eviction — cap keyframes at 7, then total at
    // window_size (10), preferring to evict the oldest non-keyframe so the
    // 3 most recent frames stay as temporal states.
    {
      std::size_t keyframe_count = 0;
      for ( const WindowFrame& frame : m_impl->m_state->m_window )
      {
        if ( frame.m_is_keyframe ) ++keyframe_count;
      }
      while ( keyframe_count > 7U )
      {
        for ( auto it = m_impl->m_state->m_window.begin(); it != m_impl->m_state->m_window.end();
              ++it )
        {
          if ( it->m_is_keyframe )
          {
            m_impl->m_state->m_window.erase( it );
            --keyframe_count;
            break;
          }
        }
      }
      while ( static_cast<int>( m_impl->m_state->m_window.size() ) >
              m_impl->options.window_size )
      {
        // Evict the oldest non-keyframe first; fall back to pop_front.
        bool evicted = false;
        for ( auto it = m_impl->m_state->m_window.begin(); it != m_impl->m_state->m_window.end();
              ++it )
        {
          if ( !it->m_is_keyframe &&
               it->m_frame_index != m_impl->m_state->m_window.back().m_frame_index )
          {
            m_impl->m_state->m_window.erase( it );
            evicted = true;
            break;
          }
        }
        if ( !evicted )
        {
          m_impl->m_state->m_window.pop_front();
        }
      }
    }
    if ( m_impl->options.m_gyro_bias.has_value() )
    {
      try
      {
        if ( m_impl->resetEvictedGyroLinks() &&
             gyro_break_reason == GyroBreakReason::kNone )
        {
          gyro_break_reason = GyroBreakReason::kEvictedEndpoint;
        }
      }
      catch ( const std::exception& exception )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed,
            std::string( "gyro eviction repair failed: " ) +
                exception.what() );
      }
    }
    m_impl->pruneLandmarksNotInWindow();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               values;
    std::uint64_t               prior_key     = 0;
    std::uint32_t               num_landmarks = 0;
    GyroGraphInfo               gyro_graph_info;
    try
    {
      m_impl->buildGraph( graph, values, prior_key, num_landmarks, nullptr,
                          &gyro_graph_info );
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
        stereoReprojRms( graph, values );
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

    FixedPimSolveResult solved = m_impl->solveFixedPimGraph(
        graph, values, prior_key, num_landmarks, gyro_graph_info );
    if ( std::holds_alternative<FixedPimSolveError>( solved ) )
    {
      const FixedPimSolveError& error =
          std::get<FixedPimSolveError>( solved );
      result.diagnostics.lm_iterations += error.m_lm_iterations;
      switch ( error.m_code )
      {
        case FixedPimSolveErrorCode::kIndeterminant:
          return finalizePostStagingHardResult(
              UpdateStatus::kFailed,
              std::string( "indeterminant linear system: " ) +
                  error.m_detail );
        case FixedPimSolveErrorCode::kOptimizer:
          return finalizePostStagingHardResult(
              UpdateStatus::kFailed,
              std::string( "optimizer exception: " ) + error.m_detail );
        case FixedPimSolveErrorCode::kGraphBuild:
          return finalizePostStagingHardResult(
              UpdateStatus::kFailed,
              std::string( "gyro graph rebuild failed: " ) +
                  error.m_detail );
        case FixedPimSolveErrorCode::kGyroIntegrity:
          return finalizePostStagingHardResult( UpdateStatus::kFailed,
                                                error.m_detail );
        case FixedPimSolveErrorCode::kRelinearizationLimit:
          return finalizePostStagingHardResult( UpdateStatus::kRejected,
                                                error.m_detail );
      }
      return finalizePostStagingHardResult(
          UpdateStatus::kFailed, "unknown fixed-PIM solve error" );
    }
    FixedPimSolveSuccess solve_success =
        std::get<FixedPimSolveSuccess>( std::move( solved ) );
    result.diagnostics.lm_iterations += solve_success.m_lm_iterations;
    std::uint32_t gyro_extra_rounds = solve_success.m_extra_rounds;
    gtsam::Values optimized         = std::move( solve_success.m_values );

    for ( const WindowFrame& frame : m_impl->m_state->m_window )
    {
      if ( !optimized.exists( X( frame.m_frame_index ) ) )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed,
            "optimized values missing a window pose" );
      }
      const Eigen::Isometry3d T_W_B =
          toIsometry( optimized.at<gtsam::Pose3>( X( frame.m_frame_index ) ) );
      if ( !isFinite( T_W_B ) )
      {
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed, "non-finite optimized pose" );
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
      if ( m_impl->options.m_gyro_bias.has_value() )
      {
        frame.m_gyro->m_bias_radps =
            optimized.at<gtsam::Vector3>( G( frame.m_frame_index ) );
      }
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
          graph, optimized, m_impl->m_state->m_landmarks_w );
    }

    std::uint32_t rounds               = 0;
    bool          outlier_reopt_failed = false;
    while ( m_impl->options.enable_outlier_reopt &&
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
      GyroGraphInfo               reopt_gyro_info;
      const gtsam::Values*        reopt_round_start =
          m_impl->options.m_gyro_bias.has_value() ? &optimized : nullptr;
      try
      {
        m_impl->buildGraph( g_r, v_r, prior_key_r, n_lm_r,
                            reopt_round_start, &reopt_gyro_info );
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

      FixedPimSolveResult reopt_solved = m_impl->solveFixedPimGraph(
          g_r, v_r, prior_key_r, n_lm_r, reopt_gyro_info );
      if ( std::holds_alternative<FixedPimSolveError>( reopt_solved ) )
      {
        const FixedPimSolveError& error =
            std::get<FixedPimSolveError>( reopt_solved );
        if ( error.m_code == FixedPimSolveErrorCode::kIndeterminant ||
             error.m_code == FixedPimSolveErrorCode::kOptimizer )
        {
          gyro_extra_rounds += error.m_extra_rounds;
          m_impl->m_state->m_window      = window_snap;
          m_impl->m_state->m_landmarks_w = landmarks_snap;
          outlier_reopt_failed           = true;
          break;
        }
        if ( error.m_code ==
             FixedPimSolveErrorCode::kRelinearizationLimit )
        {
          return finalizePostStagingHardResult( UpdateStatus::kRejected,
                                                error.m_detail );
        }
        const std::string prefix =
            error.m_code == FixedPimSolveErrorCode::kGraphBuild
                ? "gyro graph rebuild failed: "
                : "";
        return finalizePostStagingHardResult(
            UpdateStatus::kFailed, prefix + error.m_detail );
      }

      FixedPimSolveSuccess reopt_success =
          std::get<FixedPimSolveSuccess>( std::move( reopt_solved ) );
      gyro_extra_rounds += reopt_success.m_extra_rounds;
      gtsam::Values optimized_r = std::move( reopt_success.m_values );
      try
      {
        for ( const WindowFrame& frame : m_impl->m_state->m_window )
        {
          if ( !optimized_r.exists( X( frame.m_frame_index ) ) )
          {
            throw std::runtime_error( "optimized values missing a window pose" );
          }
          const Eigen::Isometry3d T_W_B = toIsometry(
              optimized_r.at<gtsam::Pose3>( X( frame.m_frame_index ) ) );
          if ( !isFinite( T_W_B ) )
          {
            throw std::runtime_error( "non-finite optimized pose" );
          }
        }

        for ( WindowFrame& frame : m_impl->m_state->m_window )
        {
          frame.m_T_W_B = toIsometry(
              optimized_r.at<gtsam::Pose3>( X( frame.m_frame_index ) ) );
          if ( m_impl->options.m_gyro_bias.has_value() )
          {
            frame.m_gyro->m_bias_radps =
                optimized_r.at<gtsam::Vector3>( G( frame.m_frame_index ) );
          }
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
        if ( m_impl->options.m_gyro_bias.has_value() )
        {
          optimized = optimized_r;
        }
        ++rounds;
        after_cull   = stereoReprojRms( g_r, optimized_r );
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

    m_impl->m_state->m_initialized         = true;
    m_impl->m_state->m_T_W_B_prev_accepted = m_impl->m_state->m_T_W_B_last_accepted;
    m_impl->m_state->m_T_W_B_last_accepted = m_impl->m_state->m_window.back().m_T_W_B;

    if ( m_impl->options.m_gyro_bias.has_value() )
    {
      result.diagnostics.m_gyro =
          m_impl->makeGyroDiagnostics( gyro_break_reason,
                                       gyro_extra_rounds );
    }

    result.status   = UpdateStatus::kOk;
    result.estimate = VioEstimate{ measurement.timestamp,
                                   m_impl->m_state->m_window.back().m_T_W_B };
    // PHAD_M4_ONLINE_BIAS_TRANSACTION_COMMIT
    transaction.commit();
    return result;
  }

}  // namespace phad::estimator
