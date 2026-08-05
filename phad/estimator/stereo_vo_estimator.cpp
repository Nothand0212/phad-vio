#include "phad/estimator/stereo_vo_estimator.hpp"

#include <gtsam/geometry/Cal3_S2Stereo.h>
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

    // Normalize rotation of a raw Eigen isometry (PnP/guess results may
    // drift slightly off SO(3)); Trajectory::create requires proper rotation.
    [[nodiscard]] Eigen::Isometry3d toIsometry(
        const Eigen::Isometry3d& transform )
    {
      Eigen::Quaterniond rotation( transform.linear() );
      rotation.normalize();
      Eigen::Isometry3d T_a_b = Eigen::Isometry3d::Identity();
      T_a_b.linear()          = rotation.toRotationMatrix();
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
            if ( observation.id == id )
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
    std::optional<Eigen::Isometry3d> last_accepted_T_W_B;
    std::optional<Eigen::Isometry3d> prev_accepted_T_W_B;
    std::uint64_t                    next_frame_index = 0;
    bool                             initialized      = false;
    std::uint32_t                    segment_id       = 0;
    // Cross-segment reject set: mean-cull ∪ cheirality erasures (block rebirth).
    std::unordered_set<LandmarkId> culled_ids_;

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

    void pruneLandmarksNotInWindow()
    {
      std::unordered_set<LandmarkId> live;
      for ( const WindowFrame& frame : window )
      {
        for ( const StereoObservation& observation : frame.observations )
        {
          live.insert( observation.id );
        }
      }
      for ( auto it = landmarks_W.begin(); it != landmarks_W.end(); )
      {
        if ( live.find( it->first ) == live.end() )
        {
          // Keep track_times for diagnostics across the whole run.
          it = landmarks_W.erase( it );
        }
        else
        {
          ++it;
        }
      }
    }

    [[nodiscard]] std::unordered_map<LandmarkId, int> countObservations()
        const
    {
      std::unordered_map<LandmarkId, int> counts;
      for ( const WindowFrame& frame : window )
      {
        for ( const StereoObservation& observation : frame.observations )
        {
          ++counts[ observation.id ];
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
          graph.emplace_shared<
              gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
              toStereoPoint( observation ), stereo_noise,
              X( frame.frame_index ), L( observation.id ), K,
              body_P_sensor );
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
    for ( const StereoObservation& observation : measurement.observations )
    {
      if ( !( observation.disparity_px > 0.0 ) ||
           !observation.left_pixel.allFinite() ||
           !std::isfinite( observation.disparity_px ) )
      {
        result.status  = UpdateStatus::kRejected;
        result.message = "non-finite pixel or non-positive disparity";
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

    std::uint32_t num_shared = 0;
    for ( const StereoObservation& observation : measurement.observations )
    {
      if ( m_impl->landmarks_W.find( observation.id ) !=
           m_impl->landmarks_W.end() )
      {
        ++num_shared;
      }
    }
    result.diagnostics.num_shared = num_shared;

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
    if ( overlap_broken &&
         static_cast<int>( measurement.observations.size() ) <
             m_impl->options.min_seed_observations )
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
    result.diagnostics.low_connectivity =
        m_impl->initialized &&
        num_shared > 0 &&
        static_cast<int>( num_shared ) < m_impl->options.min_shared_landmarks;

    if ( !m_impl->initialized &&
         static_cast<int>( measurement.observations.size() ) <
             m_impl->options.min_seed_observations )
    {
      result.status  = UpdateStatus::kRejected;
      result.message = "insufficient observations to seed first segment";
      return result;
    }

    // Snapshot for transactional rollback (keyframe only; non-keyframe
    // path does not modify window/landmarks and returns before LM).
    decltype( m_impl->window )          window_backup;
    decltype( m_impl->landmarks_W )     landmarks_backup;
    decltype( m_impl->track_times )     track_times_backup;
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
      if ( !m_impl->seedSegment( Eigen::Isometry3d::Identity(), measurement,
                                 result.diagnostics.probe_rejected_block_n,
                                 result.diagnostics.probe_new_lm_n ) )
      {
        restore();
        result.status  = UpdateStatus::kRejected;
        result.message = "failed to backproject landmark on first frame";
        return result;
      }
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
      if ( !m_impl->seedSegment( anchor, measurement,
                                 result.diagnostics.probe_rejected_block_n,
                                 result.diagnostics.probe_new_lm_n ) )
      {
        restore();
        result.status  = UpdateStatus::kRejected;
        result.message = "failed to backproject landmark on re-anchor frame";
        return result;
      }
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
          // tryPnpInit), then drop shared outliers; keep new ids.
          std::vector<LandmarkId> shared_ids;
          shared_ids.reserve( static_cast<std::size_t>( num_shared ) );
          for ( const StereoObservation& observation :
                measurement.observations )
          {
            if ( m_impl->landmarks_W.find( observation.id ) !=
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
            if ( is_shared &&
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
      // New ids only: backproject from masked candidate.observations.
      // Slice ⑤c: only keyframes seed new landmarks; non-keyframes enter
      // the window/BA but do not grow the map.
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
        const Eigen::Vector3d point_W =
            m_impl->backprojectWorld( candidate.T_W_B, observation );
        const gtsam::Pose3 T_W_left =
            toPose3( candidate.T_W_B ) * m_impl->body_P_sensor;
        const gtsam::Point3 point_left =
            T_W_left.transformTo( gtsam::Point3(
                point_W.x(), point_W.y(), point_W.z() ) );
        if ( !isFinite( point_W ) || point_left.z() <= 0.0 )
        {
          continue;
        }
        m_impl->landmarks_W[ observation.id ] = point_W;
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
