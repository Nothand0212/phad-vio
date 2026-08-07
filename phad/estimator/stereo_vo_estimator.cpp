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
    using gtsam::symbol_shorthand::X;

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

    struct WindowFrame
    {
      std::uint64_t                  frame_index = 0;
      common::Timestamp              timestamp{ 0 };
      Eigen::Isometry3d              T_W_B = Eigen::Isometry3d::Identity();
      std::vector<StereoObservation> observations;
      bool                           is_keyframe = true;  // Slice ⑤c
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
    std::optional<Eigen::Isometry3d> last_accepted_T_W_B;
    std::optional<Eigen::Isometry3d> prev_accepted_T_W_B;
    std::uint64_t                    next_frame_index = 0;
    bool                             initialized      = false;
    std::uint32_t                    segment_id       = 0;
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
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          X( oldest ), toPose3( window.front().T_W_B ), prior_noise );

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
    VioUpdateResult result;
    result.diagnostics.culled_landmark_ids.clear();
    result.diagnostics.num_observations =
        static_cast<std::uint32_t>( measurement.observations.size() );
    result.diagnostics.segment_id = m_impl->segment_id;

    if ( measurement.observations.empty() )
    {
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

    std::uint32_t num_shared     = 0;
    std::uint32_t num_disparity  = 0;
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
    // Non-keyframe with too few shared landmarks cannot run PnP; a raw CV
    // guess would pollute the pose chain. Reject (Slice ⑤b gate).
    if ( !keyframe && static_cast<int>( num_shared ) <
                          m_impl->options.min_pnp_inliers )
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
    KeyframeMeasurement       accumulated_measurement;
    const KeyframeMeasurement* effective_measurement = &measurement;
    const auto accumulate = [ & ]() -> bool {
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
        ( void )id;
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

    // Snapshot for transactional rollback (keyframe only; non-keyframe
    // path does not modify window/landmarks and returns before LM).
    decltype( m_impl->window )          window_backup;
    decltype( m_impl->landmarks_W )     landmarks_backup;
    decltype( m_impl->track_times )     track_times_backup;
    decltype( m_impl->last_stereo_pose_W ) last_stereo_pose_backup;
    decltype( m_impl->last_accepted_T_W_B ) last_backup;
    decltype( m_impl->prev_accepted_T_W_B ) prev_backup;
    decltype( m_impl->next_frame_index )    next_index_backup;
    bool        initialized_backup = false;
    decltype( m_impl->segment_id )  segment_id_backup;
    decltype( m_impl->culled_ids_ ) culled_ids_backup;
    if ( keyframe )
    {
      window_backup      = m_impl->window;
      landmarks_backup   = m_impl->landmarks_W;
      track_times_backup = m_impl->track_times;
      last_stereo_pose_backup = m_impl->last_stereo_pose_W;
      last_backup        = m_impl->last_accepted_T_W_B;
      prev_backup        = m_impl->prev_accepted_T_W_B;
      next_index_backup  = m_impl->next_frame_index;
      initialized_backup = m_impl->initialized;
      segment_id_backup  = m_impl->segment_id;
      culled_ids_backup  = m_impl->culled_ids_;
    }

    std::vector<LandmarkId> frame_culled;

    auto restore = [ & ]() {
      m_impl->window                = window_backup;
      m_impl->landmarks_W           = landmarks_backup;
      m_impl->track_times           = track_times_backup;
      m_impl->last_stereo_pose_W    = last_stereo_pose_backup;
      m_impl->last_accepted_T_W_B   = last_backup;
      m_impl->prev_accepted_T_W_B   = prev_backup;
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
      if ( !m_impl->seedSegment( Eigen::Isometry3d::Identity(),
                                 *effective_measurement,
                                 result.diagnostics.probe_rejected_block_n,
                                 result.diagnostics.probe_new_lm_n ) )
      {
        restore();
        result.status  = UpdateStatus::kRejected;
        result.message = "failed to backproject landmark on first frame";
        return result;
      }
      m_impl->pending_seed_obs.clear();
    }
    else if ( overlap_broken )
    {
      const Eigen::Isometry3d anchor = m_impl->poseInitialValue();
      if ( !isFinite( anchor ) )
      {
        restore();
        result.status  = UpdateStatus::kFailed;
        result.message = "non-finite pose initial value";
        return result;
      }
      if ( !m_impl->seedSegment( anchor, *effective_measurement,
                                 result.diagnostics.probe_rejected_block_n,
                                 result.diagnostics.probe_new_lm_n ) )
      {
        restore();
        result.status  = UpdateStatus::kRejected;
        result.message = "failed to backproject landmark on re-anchor frame";
        return result;
      }
      m_impl->pending_seed_obs.clear();
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

      const Eigen::Isometry3d guess_T_W_B = m_impl->poseInitialValue();
      candidate.T_W_B                     = guess_T_W_B;

      if ( m_impl->options.enable_pnp_init &&
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
        double proj_error = -1.0;
        try
        {
          projected = gtsam::StereoCamera( T_W_left, m_impl->K ).project(
              point_W );
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
        const auto track_it = m_impl->track_times.find( observation.id );
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
    m_impl->buildGraph( graph, values, prior_key, num_landmarks );
    result.diagnostics.num_landmarks = num_landmarks;
    result.diagnostics.prior_key     = prior_key;
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
      m_impl->buildGraph( g_r, v_r, prior_key_r, n_lm_r );
      (void)prior_key_r;
      (void)n_lm_r;

      try
      {
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

    m_impl->initialized         = true;
    m_impl->prev_accepted_T_W_B = m_impl->last_accepted_T_W_B;
    m_impl->last_accepted_T_W_B = m_impl->window.back().T_W_B;

    result.status   = UpdateStatus::kOk;
    result.estimate = VioEstimate{ measurement.timestamp,
                                   m_impl->window.back().T_W_B };
    return result;
  }

}  // namespace phad::estimator
