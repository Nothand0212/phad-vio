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
                      const KeyframeMeasurement& measurement )
    {
      landmarks_W.clear();

      WindowFrame candidate;
      candidate.frame_index  = next_frame_index;
      candidate.timestamp    = measurement.timestamp;
      candidate.observations = measurement.observations;
      candidate.T_W_B        = anchor_T_W_B;

      for ( const StereoObservation& observation : measurement.observations )
      {
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
        const gtsam::Values& values )
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
        ++dropped;
      }
      return dropped;
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
      const KeyframeMeasurement& measurement )
  {
    VioUpdateResult result;
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

    // Snapshot for transactional rollback.
    const auto window_backup      = m_impl->window;
    const auto landmarks_backup   = m_impl->landmarks_W;
    const auto track_times_backup = m_impl->track_times;
    const auto last_backup        = m_impl->last_accepted_T_W_B;
    const auto prev_backup        = m_impl->prev_accepted_T_W_B;
    const auto next_index_backup  = m_impl->next_frame_index;
    const bool initialized_backup = m_impl->initialized;
    const auto segment_id_backup  = m_impl->segment_id;

    auto restore = [ & ]() {
      m_impl->window              = window_backup;
      m_impl->landmarks_W         = landmarks_backup;
      m_impl->track_times         = track_times_backup;
      m_impl->last_accepted_T_W_B = last_backup;
      m_impl->prev_accepted_T_W_B = prev_backup;
      m_impl->next_frame_index    = next_index_backup;
      m_impl->initialized         = initialized_backup;
      m_impl->segment_id          = segment_id_backup;
    };

    if ( !m_impl->initialized )
    {
      if ( !m_impl->seedSegment( Eigen::Isometry3d::Identity(),
                                 measurement ) )
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
      if ( !m_impl->seedSegment( anchor, measurement ) )
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

      candidate.T_W_B = m_impl->poseInitialValue();
      if ( !isFinite( candidate.T_W_B ) )
      {
        restore();
        result.status  = UpdateStatus::kFailed;
        result.message = "non-finite pose initial value";
        return result;
      }
      for ( const StereoObservation& observation : measurement.observations )
      {
        m_impl->track_times[ observation.id ].push_back(
            measurement.timestamp );
        if ( m_impl->landmarks_W.find( observation.id ) !=
             m_impl->landmarks_W.end() )
        {
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
      }

      m_impl->window.push_back( std::move( candidate ) );
      ++m_impl->next_frame_index;
    }

    while ( static_cast<int>( m_impl->window.size() ) >
            m_impl->options.window_size )
    {
      m_impl->window.pop_front();
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

    double max_shift_m = 0.0;
    for ( WindowFrame& frame : m_impl->window )
    {
      const Eigen::Isometry3d T_W_B =
          toIsometry( optimized.at<gtsam::Pose3>( X( frame.frame_index ) ) );
      const auto before_it = poses_before.find( frame.frame_index );
      if ( before_it != poses_before.end() )
      {
        max_shift_m = std::max(
            max_shift_m,
            ( T_W_B.translation() - before_it->second ).norm() );
      }
      frame.T_W_B = T_W_B;
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

    const std::uint32_t cheirality_after = countCheiralityFactors(
        graph, optimized, m_impl->calibration.fxPixels() );
    const std::uint32_t dropped =
        m_impl->dropCheiralityLandmarks( optimized );
    result.diagnostics.num_cheirality =
        std::max( cheirality_before, std::max( cheirality_after, dropped ) );
    result.diagnostics.reproj_rms_after_px =
        stereoReprojRms( graph, optimized );
    result.diagnostics.max_window_pose_shift_m = max_shift_m;

    m_impl->initialized         = true;
    m_impl->prev_accepted_T_W_B = m_impl->last_accepted_T_W_B;
    m_impl->last_accepted_T_W_B = m_impl->window.back().T_W_B;

    result.status   = UpdateStatus::kOk;
    result.estimate = VioEstimate{ measurement.timestamp,
                                   m_impl->window.back().T_W_B };
    return result;
  }

}  // namespace phad::estimator
