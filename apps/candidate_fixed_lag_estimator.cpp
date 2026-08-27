#include "apps/candidate_fixed_lag_estimator.hpp"

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace phad::apps
{

  namespace
  {

    using gtsam::symbol_shorthand::B;
    using gtsam::symbol_shorthand::L;
    using gtsam::symbol_shorthand::X;
    using Pose3 = gtsam::Pose3;
    using StereoFactor =
        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;

    /// 复刻 production `PoseAhrsFactor`（Pose3/Pose3/Vector3 包装 gtsam
    /// AHRSFactor，只约束旋转）。candidate 无 gyro-alignment 流程，因此
    /// 噪声只取 preintMeasCov()（对齐残差项恒 0）。
    class CandidatePoseAhrsFactor final
        : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3,
                                          gtsam::Vector3>
    {
      using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3,
                                            gtsam::Vector3>;

    public:
      CandidatePoseAhrsFactor(
          gtsam::Key pose_i, gtsam::Key pose_j, gtsam::Key bias_gyr,
          const gtsam::PreintegratedAhrsMeasurements& preintegration )
          : Base( ahrsNoise( preintegration ), pose_i, pose_j, bias_gyr ),
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
        const gtsam::Rot3 rotation_i =
            pose_i.rotation( H1 ? &D_rotation_i_pose_i : nullptr );
        const gtsam::Rot3 rotation_j =
            pose_j.rotation( H2 ? &D_rotation_j_pose_j : nullptr );

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
      [[nodiscard]] static gtsam::SharedNoiseModel ahrsNoise(
          const gtsam::PreintegratedAhrsMeasurements& preintegration )
      {
        return gtsam::noiseModel::Gaussian::Covariance(
            preintegration.preintMeasCov() );
      }

      gtsam::PreintegratedAhrsMeasurements m_preintegration;
    };

    [[nodiscard]] std::shared_ptr<gtsam::PreintegratedAhrsMeasurements>
    rebuildAhrsPreintegration(
        const std::shared_ptr<gtsam::PreintegratedRotationParams>& params,
        const std::vector<sensor::ImuMeasurement>&                 samples,
        const Eigen::Vector3d&                                     bias_gyr )
    {
      auto preint = std::make_shared<gtsam::PreintegratedAhrsMeasurements>(
          params, bias_gyr );
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

    gtsam::SharedNoiseModel posePriorNoise(
        const estimator::EstimatorOptions& options )
    {
      // 与 production prior 语义一致：旋转/平移分别定权。
      gtsam::Vector6 sigmas;
      sigmas << options.prior_rotation_sigma_rad * gtsam::Vector3::Ones(),
          options.prior_translation_sigma_m * gtsam::Vector3::Ones();
      return gtsam::noiseModel::Diagonal::Sigmas( sigmas );
    }

    // 骨架期占位 Between 已删除（slice ③ 起 landmark 观测提供 KF 间
    // 约束；占位 identity/σ=1e-3 会压制真实运动，见 P2b 结果文档 §4）。
    // 复刻 production 的 stereo 噪声语义（Isotropic sigma + 可选 Huber）。
    gtsam::SharedNoiseModel makeStereoNoise(
        const estimator::EstimatorOptions& options )
    {
      const auto gaussian = gtsam::noiseModel::Isotropic::Sigma(
          3, options.stereo_sigma_px );
      if ( options.huber_k_px <= 0.0 )
      {
        return gaussian;
      }
      return gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(
              options.huber_k_px ),
          gaussian );
    }

    gtsam::Pose3 toPose3( const Eigen::Isometry3d& T_a_b )
    {
      return gtsam::Pose3( T_a_b.matrix() );
    }

    Eigen::Isometry3d toIsometry(
        const sensor::RigidTransform& transform )
    {
      Eigen::Isometry3d T_a_b = Eigen::Isometry3d::Identity();
      T_a_b.linear()          = transform.rotation();
      T_a_b.translation()     = transform.translation();
      return T_a_b;
    }

    Eigen::Isometry3d toIsometry3d( const Pose3& pose )
    {
      // GTSAM Rot3 矩阵可轻微漂移离 SO(3)；Trajectory::create 要求
      // 正交旋转（与 production 的 toIsometry 同款 normalize）。
      Eigen::Quaterniond rotation( pose.rotation().matrix() );
      rotation.normalize();
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.linear()          = rotation.toRotationMatrix();
      T.translation()     = pose.translation();
      return T;
    }

    gtsam::StereoPoint2 toStereoPoint(
        const estimator::StereoObservation& observation )
    {
      return gtsam::StereoPoint2(
          observation.left_pixel.x(),
          observation.left_pixel.x() - observation.disparity_px,
          observation.left_pixel.y() );
    }

    Eigen::Vector3d backprojectWorld(
        const Pose3& T_W_B, const gtsam::Cal3_S2Stereo::shared_ptr& K,
        const Pose3&                        body_P_sensor,
        const estimator::StereoObservation& observation )
    {
      const gtsam::StereoCamera camera( T_W_B * body_P_sensor, K );
      const gtsam::Point3       point_W =
          camera.backproject( toStereoPoint( observation ) );
      return Eigen::Vector3d( point_W.x(), point_W.y(), point_W.z() );
    }

    std::uint32_t toU32( std::size_t value )
    {
      return static_cast<std::uint32_t>( value );
    }

  }  // namespace

  struct CandidateFixedLagEstimator::Impl
  {
    Impl( camera::RectifiedStereoCalibration calibration,
          estimator::EstimatorOptions        estimator_options )
        : options( std::move( estimator_options ) ),
          smoother( static_cast<double>( options.window_size ) ),
          K( std::make_shared<gtsam::Cal3_S2Stereo>(
              calibration.fxPixels(), calibration.fyPixels(), 0.0,
              calibration.cxPixels(), calibration.cyPixels(),
              calibration.baselineM() ) ),
          body_P_sensor(
              toPose3( toIsometry( calibration.T_B_left_rectified() ) ) ),
          stereo_noise( makeStereoNoise( options ) ),
          gating_threshold_px(
              // 进图前预测残差（常量速度）在 MH_01 加速/旋转段可达
              // 50+ px；threshold 只拦明显 outlier（>100 px），小残差
              // 交给 stereo_noise 的 Huber 核（production 同语义）。
              options.outlier_avg_reproj_px > 100.0
                  ? options.outlier_avg_reproj_px
                  : 100.0 )
    {
      // 与 production StereoVoEstimator 的 create 校验一致：lag < 1 时
      // cutoff 行为未定义（每帧立即边缘化）。
      if ( options.window_size < 1 )
      {
        throw std::invalid_argument(
            "EstimatorOptions.window_size must be >= 1" );
      }
      // PnP 参数校验（与 production tryPnpInit 的构造约束一致，NaN 也拒绝）。
      if ( !( options.pnp_reproj_px > 0.0 ) || options.min_pnp_inliers < 4 ||
           !( options.pnp_confidence > 0.0 &&
              options.pnp_confidence < 1.0 ) )
      {
        throw std::invalid_argument(
            "invalid PnP options: pnp_reproj_px > 0, min_pnp_inliers >= 4, "
            "0 < pnp_confidence < 1" );
      }
      ahrs_params = std::make_shared<gtsam::PreintegratedRotationParams>();
      ahrs_params->setGyroscopeCovariance(
          Eigen::Matrix3d::Identity() * options.imu_gyr_noise_nd *
          options.imu_gyr_noise_nd );
      bias_prior_noise = gtsam::noiseModel::Isotropic::Sigma(
          3, options.imu_prior_bias_gyro_sigma );
    }

    estimator::EstimatorOptions                         options;
    gtsam::IncrementalFixedLagSmoother                  smoother;
    gtsam::Cal3_S2Stereo::shared_ptr                    K;
    gtsam::Pose3                                        body_P_sensor;
    gtsam::SharedNoiseModel                             stereo_noise;
    double                                              gating_threshold_px;
    std::shared_ptr<gtsam::PreintegratedRotationParams> ahrs_params;
    gtsam::SharedNoiseModel                             bias_prior_noise;
    std::vector<std::pair<std::size_t, std::string>>    ledger;
    std::uint64_t                                       frame_count = 0;
    std::uint64_t                                       kf_count    = 0;
    bool                                                has_root    = false;
    std::optional<estimator::VioEstimate>               last_estimate;
    std::optional<Pose3>                                last_pose;
    /// 倒数第二个 KF 位姿（常量速度外推用）。
    std::optional<Pose3>             prev_pose;
    std::optional<common::Timestamp> last_kf_timestamp;
    estimator::FusionMode            last_fusion_mode = estimator::FusionMode::kVisionOnly;
    /// slice ④：KF 间累积的 IMU 段（相邻段共享右端样本已去重）。
    std::vector<sensor::ImuMeasurement> pending_imu;
    /// 区间被 gap 打断（sticky）：下一 KF 不建部分区间的 gyro factor。
    bool imu_interval_broken = false;
    /// 图内 landmark key 集（L 前缀；slice ③ 生命周期账本）。
    std::unordered_set<gtsam::Key> active_landmarks;
    /// 已观测一次但尚未进图的 landmark id（延迟初始化：首观测只登记，
    /// 二次观测才三角化进图——单观测 landmark 与 pose 的联合系统缺秩，
    /// 难序列实测 Indeterminant）。tracker id 单调不重用。
    std::unordered_set<common::LandmarkId> observed_once;

    /// 进图前 gating：已有 landmark 的新观测相对当前 estimate 的重投影
    /// 残差超阈值 → 拒收（观测不入图、不刷 timestamp）。
    [[nodiscard]] bool gated( const Pose3&                        T_W_B,
                              const estimator::StereoObservation& observation,
                              gtsam::Key                          landmark_key ) const
    {
      gtsam::Point3 point_W;
      try
      {
        point_W = smoother.calculateEstimate<gtsam::Point3>( landmark_key );
      }
      catch ( const std::exception& )
      {
        // key 已出窗/刚边缘化（末帧实测 "Requested variable"）：拒收。
        return true;
      }
      gtsam::StereoPoint2 predicted;
      try
      {
        const gtsam::StereoCamera camera( T_W_B * body_P_sensor, K );
        predicted = camera.project( point_W );
      }
      catch ( const gtsam::StereoCheiralityException& )
      {
        // landmark 在相机后：该观测无法验证，按拒收处理（不入图）。
        return true;
      }
      const double residual_px =
          ( predicted.vector() - toStereoPoint( observation ).vector() )
              .norm();
      return residual_px > gating_threshold_px;
    }

    struct LandmarkCollectResult
    {
      std::uint32_t           inserted = 0;
      std::uint32_t           anchored = 0;
      std::vector<gtsam::Key> new_keys;
    };

    /// 观测进图（slice ③ 语义）：gating + 三角化 + stereo factor 连到
    /// anchor pose key（KF 帧=新 pose，非 KF 帧=最新 KF pose）。新
    /// landmark 延迟初始化（VINS/ORB-SLAM3）：首观测只登记不建 factor，
    /// 二次观测才三角化进图（单观测 landmark 与 pose 联合缺秩）；root
    /// 帧由 pose prior 锚定，首观测即可进图。
    [[nodiscard]] LandmarkCollectResult collectLandmarkFactors(
        const estimator::KeyframeMeasurement& measurement,
        const Pose3&                          pose_for_triangulation,
        gtsam::Key anchor_pose_key, double ts, bool allow_new_landmarks,
        bool first_observation_allowed, gtsam::NonlinearFactorGraph& factors,
        gtsam::Values&                            values,
        gtsam::FixedLagSmoother::KeyTimestampMap& timestamps )
    {
      LandmarkCollectResult result;
      for ( const estimator::StereoObservation& observation :
            measurement.observations )
      {
        if ( observation.disparity_px <= 0.0 )
        {
          continue;
        }
        const gtsam::Key landmark_key = L( observation.id );
        if ( active_landmarks.count( landmark_key ) != 0U )
        {
          if ( gated( pose_for_triangulation, observation, landmark_key ) )
          {
            continue;
          }
          factors.emplace_shared<StereoFactor>(
              toStereoPoint( observation ), stereo_noise, anchor_pose_key,
              landmark_key, K, body_P_sensor );
          timestamps.emplace( landmark_key, ts );
          ++result.anchored;
        }
        else if ( allow_new_landmarks )
        {
          if ( !first_observation_allowed &&
               observed_once.count( observation.id ) == 0U )
          {
            // 首观测登记（延迟初始化）：不进图，等待二次观测。
            observed_once.insert( observation.id );
            continue;
          }
          const Eigen::Vector3d point_W = backprojectWorld(
              pose_for_triangulation, K, body_P_sensor, observation );
          // cheirality：深度用左相机系（与 production seedSegment 一致）。
          const gtsam::Pose3 T_W_left =
              pose_for_triangulation * body_P_sensor;
          const gtsam::Point3 point_left = T_W_left.transformTo(
              gtsam::Point3( point_W.x(), point_W.y(), point_W.z() ) );
          // 深度下限（与 tracker min_depth_m 一致）：过近的三角化点在
          // LM 一步就会越过相机面（Stereo Cheirality Exception）。
          if ( !point_W.allFinite() || point_left.z() <= 0.3 )
          {
            continue;
          }
          values.insert( landmark_key,
                         gtsam::Point3( point_W.x(), point_W.y(),
                                        point_W.z() ) );
          factors.emplace_shared<StereoFactor>(
              toStereoPoint( observation ), stereo_noise, anchor_pose_key,
              landmark_key, K, body_P_sensor );
          timestamps.emplace( landmark_key, ts );
          result.new_keys.push_back( landmark_key );
          observed_once.erase( observation.id );
        }
        ++result.inserted;
      }
      return result;
    }

    /// 自然退休：从 timestamps() 消失的 L 前缀 key（最后观测出窗被
    /// cutoff 边缘化并自动清理）。返回本帧退休数并更新账本。
    std::uint32_t retireExpiredLandmarks()
    {
      std::uint32_t                  retired = 0;
      std::unordered_set<gtsam::Key> still_active;
      for ( const auto& [ key, key_timestamp ] : smoother.timestamps() )
      {
        if ( gtsam::Symbol( key ).chr() == 'l' )
        {
          still_active.insert( key );
        }
      }
      for ( const gtsam::Key key : active_landmarks )
      {
        if ( still_active.count( key ) == 0U )
        {
          ++retired;
        }
      }
      active_landmarks = std::move( still_active );
      return retired;
    }

    /// 新 KF 初值：常量速度外推（PnP 失败时的兜底）。
    [[nodiscard]] Pose3 predictKfPose() const
    {
      if ( !last_pose.has_value() )
      {
        return Pose3::Identity();
      }
      if ( !prev_pose.has_value() )
      {
        return *last_pose;
      }
      const Pose3 delta = last_pose->between( *prev_pose );
      return last_pose->compose( delta );
    }

    /// 当前图中 X 前缀 pose 数。
    [[nodiscard]] std::uint32_t poseCount() const
    {
      std::uint32_t count = 0;
      for ( const auto& [ key, key_timestamp ] : smoother.timestamps() )
      {
        if ( gtsam::Symbol( key ).chr() == 'x' )
        {
          ++count;
        }
      }
      return count;
    }

    FixedLagGraphStats stats() const
    {
      FixedLagGraphStats graph;
      for ( const auto& [ key, key_timestamp ] : smoother.timestamps() )
      {
        if ( gtsam::Symbol( key ).chr() == 'x' )
        {
          ++graph.active_pose_count;
        }
      }
      graph.active_factor_count = toU32( smoother.getFactors().size() );
      return graph;
    }

    /// 非 KF 帧 PnP（solvePnPRansac，production tryPnpInit 语义）：图内
    /// landmark 3D ∩ 当前观测。失败返回 nullopt（调用方按 kRejected）。
    [[nodiscard]] std::optional<Eigen::Isometry3d> tryPnp(
        const estimator::KeyframeMeasurement& measurement ) const
    {
      std::vector<cv::Point3d> pts3d;
      std::vector<cv::Point2d> pts2d;
      pts3d.reserve( measurement.observations.size() );
      pts2d.reserve( measurement.observations.size() );
      for ( const estimator::StereoObservation& observation :
            measurement.observations )
      {
        if ( observation.disparity_px <= 0.0 )
        {
          continue;
        }
        const gtsam::Key landmark_key = L( observation.id );
        // 只匹配真正在图内的 landmark（本帧未提交的 id 不在 timestamps）。
        if ( smoother.timestamps().count( landmark_key ) == 0U )
        {
          continue;
        }
        gtsam::Point3 point_W;
        try
        {
          point_W = smoother.calculateEstimate<gtsam::Point3>( landmark_key );
        }
        catch ( const std::exception& )
        {
          // key 已出窗/刚边缘化：跳过该观测。
          continue;
        }
        pts3d.emplace_back( point_W.x(), point_W.y(), point_W.z() );
        pts2d.emplace_back( observation.left_pixel.x(),
                            observation.left_pixel.y() );
      }
      // 非 KF 帧的图内匹配受 track 存活率限制（2-9 个/帧）；production
      // 的 min_pnp_inliers=10 是每帧全观测语义。candidate 用 4 点最小
      // 充分（solvePnP 下界），成功率优先，精度阶段再调。
      const int min_pnp_inliers =
          std::min( options.min_pnp_inliers, 4 );
      if ( static_cast<int>( pts3d.size() ) < min_pnp_inliers )
      {
        return std::nullopt;
      }
      if ( !last_estimate.has_value() )
      {
        return std::nullopt;
      }

      const Eigen::Isometry3d T_W_left_guess =
          last_estimate->T_W_B * toIsometry3d( body_P_sensor );
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

      const cv::Mat camera_matrix =
          ( cv::Mat_<double>( 3, 3 ) << K->fx(), 0.0, K->px(), 0.0,
            K->fy(), K->py(), 0.0, 0.0, 1.0 );

      cv::Mat inliers;
      bool    solved = false;
      try
      {
        solved = cv::solvePnPRansac(
            pts3d, pts2d, camera_matrix, cv::noArray(), rvec, tvec,
            /*useExtrinsicGuess=*/true, /*iterationsCount=*/100,
            static_cast<float>( options.pnp_reproj_px ),
            options.pnp_confidence, inliers, cv::SOLVEPNP_ITERATIVE );
      }
      catch ( const cv::Exception& )
      {
        return std::nullopt;
      }
      if ( !solved || inliers.rows < min_pnp_inliers )
      {
        return std::nullopt;
      }

      cv::Mat R;
      cv::Rodrigues( rvec, R );
      Eigen::Isometry3d T_left_W = Eigen::Isometry3d::Identity();
      for ( int row = 0; row < 3; ++row )
      {
        for ( int col = 0; col < 3; ++col )
        {
          T_left_W.linear()( row, col ) = R.at<double>( row, col );
        }
      }
      T_left_W.translation() =
          Eigen::Vector3d( tvec.at<double>( 0 ), tvec.at<double>( 1 ),
                           tvec.at<double>( 2 ) );
      Eigen::Isometry3d T_W_B =
          T_left_W.inverse() * toIsometry3d( body_P_sensor ).inverse();
      if ( !T_W_B.matrix().allFinite() )
      {
        return std::nullopt;
      }
      // cv::Rodrigues/求逆累积数值漂移：旋转正交化（否则 Trajectory
      // ::create 拒绝非正交旋转）。
      Eigen::Quaterniond rotation( T_W_B.linear() );
      rotation.normalize();
      T_W_B.linear() = rotation.toRotationMatrix();
      return T_W_B;
    }
  };

  CandidateFixedLagEstimator::CandidateFixedLagEstimator(
      camera::RectifiedStereoCalibration calibration,
      estimator::EstimatorOptions        options )
      : m_impl( std::make_unique<Impl>( std::move( calibration ),
                                        std::move( options ) ) )
  {
  }

  CandidateFixedLagEstimator::~CandidateFixedLagEstimator() = default;

  CandidateFixedLagEstimator::CandidateFixedLagEstimator(
      CandidateFixedLagEstimator&& ) noexcept = default;

  CandidateFixedLagEstimator& CandidateFixedLagEstimator::operator=(
      CandidateFixedLagEstimator&& ) noexcept = default;

  FixedLagUpdateResult CandidateFixedLagEstimator::update(
      const estimator::KeyframeMeasurement& measurement,
      bool                                  keyframe )
  {
    FixedLagUpdateResult result;
    result.vio.diagnostics.fusion_mode = m_impl->last_fusion_mode;

    // slice ④：非-gap 段累积 KF 间的 IMU 样本（相邻段共享右端插值样本，
    // 去重），gap 段清空缓冲（区间断裂）。
    if ( !measurement.imu_gap )
    {
      for ( const sensor::ImuMeasurement& sample : measurement.imu_samples )
      {
        if ( m_impl->pending_imu.empty() ||
             sample.timestamp != m_impl->pending_imu.back().timestamp )
        {
          m_impl->pending_imu.push_back( sample );
        }
      }
    }
    else
    {
      m_impl->pending_imu.clear();
      m_impl->imu_interval_broken = true;
    }

    // 帧级 snapshot：update 无 strong exception guarantee，任何异常
    // （KF 与非 KF 都调用 update）恢复 snapshot 后原样 rethrow。
    struct Snapshot
    {
      gtsam::IncrementalFixedLagSmoother               smoother;
      std::vector<std::pair<std::size_t, std::string>> ledger;
      std::unordered_set<gtsam::Key>                   active_landmarks;
      std::unordered_set<common::LandmarkId>           observed_once;
      std::uint64_t                                    frame_count = 0;
      std::uint64_t                                    kf_count    = 0;
      bool                                             has_root    = false;
      std::optional<estimator::VioEstimate>            last_estimate;
      std::optional<Pose3>                             last_pose;
      std::optional<Pose3>                             prev_pose;
      std::optional<common::Timestamp>                 last_kf_timestamp;
      estimator::FusionMode                            last_fusion_mode = estimator::FusionMode::kVisionOnly;
      std::vector<sensor::ImuMeasurement>              pending_imu;
      bool                                             imu_interval_broken = false;
    };
    Snapshot snapshot{
        m_impl->smoother, m_impl->ledger,
        m_impl->active_landmarks, m_impl->observed_once,
        m_impl->frame_count, m_impl->kf_count, m_impl->has_root,
        m_impl->last_estimate, m_impl->last_pose, m_impl->prev_pose,
        m_impl->last_kf_timestamp, m_impl->last_fusion_mode,
        m_impl->pending_imu, m_impl->imu_interval_broken };

    try
    {
      ++m_impl->frame_count;
      const double ts = static_cast<double>( m_impl->frame_count );

      if ( !keyframe )
      {
        // 非 KF 帧：不进图（观测时刻与最新 KF pose 错配会污染图并触发
        // Stereo Cheirality Exception；图连接由短 KF 间隔保证）。本帧只
        // 推进图老化（刷新最新 KF timestamp），位姿用 PnP。
        if ( m_impl->kf_count == 0U || measurement.observations.empty() )
        {
          result.vio.status  = estimator::UpdateStatus::kRejected;
          result.vio.message = "no keyframe yet or no observations";
          result.graph       = m_impl->stats();
          return result;
        }
        const std::uint32_t prev_poses = m_impl->poseCount();
        std::uint32_t       retired    = 0;
        try
        {
          gtsam::FixedLagSmoother::KeyTimestampMap aging;
          aging.emplace( X( m_impl->kf_count - 1U ),
                         static_cast<double>( m_impl->frame_count ) );
          m_impl->smoother.update( {}, {}, aging );
          retired = m_impl->retireExpiredLandmarks();
        }
        catch ( const std::exception& )
        {
          // GTSAM 边缘化边界问题（V1_02 实测 "not unused"）：本帧跳过
          // 老化（图保持上一帧状态，暂停一帧无害），PnP 兜底。仍对齐
          // 账本：retire 只读 timestamps()，update 抛后图可能已部分
          // 更新（否则 active_landmarks 残留已边缘化的 landmark，
          // gated() 的 calculateEstimate 会抛 "not in VectorValues"）。
          retired = m_impl->retireExpiredLandmarks();
        }
        if ( const std::optional<Eigen::Isometry3d> pnp =
                 m_impl->tryPnp( measurement ) )
        {
          estimator::VioEstimate estimate;
          estimate.timestamp    = measurement.timestamp;
          estimate.T_W_B        = *pnp;
          m_impl->last_estimate = estimate;
          result.vio.status     = estimator::UpdateStatus::kOk;
          result.vio.estimate   = estimate;
        }
        else
        {
          result.vio.status = estimator::UpdateStatus::kRejected;
          result.vio.message =
              "PnP failed or insufficient shared landmarks";
        }
        result.vio.diagnostics.num_observations = toU32(
            measurement.observations.size() );
        result.vio.diagnostics.num_disparity = toU32(
            static_cast<std::size_t>( std::count_if(
                measurement.observations.begin(),
                measurement.observations.end(),
                []( const estimator::StereoObservation& obs ) { return obs.disparity_px > 0.0; } ) ) );
        result.graph                        = m_impl->stats();
        result.graph.retired_landmark_count = retired;
        result.graph.marginalized_pose_count =
            prev_poses > result.graph.active_pose_count
                ? prev_poses - result.graph.active_pose_count
                : 0U;
        return result;
      }

      const gtsam::Key pose_key = X( m_impl->kf_count );
      const bool       is_root  = !m_impl->has_root;
      if ( is_root )
      {
        // root 帧先确定锚 pose：landmark 三角化/gating 需要 last_pose。
        m_impl->last_pose = Pose3::Identity();
      }
      // 新 KF 初值：优先 PnP（图内 landmark ∩ 当前观测，production
      // tryPnpInit 同款——视觉真值初值让 LM 收敛），否则常量速度外推。
      const std::optional<Eigen::Isometry3d> kf_pnp =
          is_root ? std::nullopt : m_impl->tryPnp( measurement );
      const Pose3 kf_initial =
          kf_pnp.has_value() ? toPose3( *kf_pnp )
                             : m_impl->predictKfPose();

      gtsam::NonlinearFactorGraph              factors;
      gtsam::Values                            values;
      gtsam::FixedLagSmoother::KeyTimestampMap timestamps;

      const auto collected = m_impl->collectLandmarkFactors(
          measurement, kf_initial, pose_key, ts, /*allow_new=*/true,
          /*first_observation_allowed=*/is_root, factors, values,
          timestamps );
      const std::uint32_t inserted_stereo       = collected.inserted;
      const std::uint32_t anchored_observations = collected.anchored;
      const auto&         new_landmark_keys     = collected.new_keys;

      // 帧级降级（数据问题，非 invariant 破坏）：只老化 + PnP 兜底，
      // 不 terminal。用于 cheirality / 边缘化边界 / 数值病态。
      const auto degrade = [ & ]() -> FixedLagUpdateResult {
        FixedLagUpdateResult degraded;
        degraded.vio.diagnostics.fusion_mode =
            result.vio.diagnostics.fusion_mode;
        std::uint32_t retired = 0;
        try
        {
          gtsam::FixedLagSmoother::KeyTimestampMap aging;
          aging.emplace( X( m_impl->kf_count - 1U ),
                         static_cast<double>( m_impl->frame_count ) );
          m_impl->smoother.update( {}, {}, aging );
          retired = m_impl->retireExpiredLandmarks();
        }
        catch ( const std::exception& )
        {
          // 边缘化边界/数值问题：跳过本帧老化，仍对齐账本。
          retired = m_impl->retireExpiredLandmarks();
        }
        if ( const std::optional<Eigen::Isometry3d> pnp =
                 m_impl->tryPnp( measurement ) )
        {
          estimator::VioEstimate estimate;
          estimate.timestamp    = measurement.timestamp;
          estimate.T_W_B        = *pnp;
          m_impl->last_estimate = estimate;
          degraded.vio.status   = estimator::UpdateStatus::kOk;
          degraded.vio.estimate = estimate;
        }
        else
        {
          degraded.vio.status = estimator::UpdateStatus::kRejected;
          degraded.vio.message =
              "PnP failed or insufficient shared landmarks";
        }
        degraded.vio.diagnostics.num_observations = toU32(
            measurement.observations.size() );
        degraded.vio.diagnostics.num_disparity = toU32(
            static_cast<std::size_t>( std::count_if(
                measurement.observations.begin(),
                measurement.observations.end(),
                []( const estimator::StereoObservation& obs ) { return obs.disparity_px > 0.0; } ) ) );
        degraded.graph                        = m_impl->stats();
        degraded.graph.retired_landmark_count = retired;
        std::uint32_t prev_poses              = 0;
        for ( const auto& [ key, key_timestamp ] :
              snapshot.smoother.timestamps() )
        {
          if ( gtsam::Symbol( key ).chr() == 'x' )
          {
            ++prev_poses;
          }
        }
        degraded.graph.marginalized_pose_count =
            prev_poses > degraded.graph.active_pose_count
                ? prev_poses - degraded.graph.active_pose_count
                : 0U;
        return degraded;
      };

      // 观测不足的 KF 不进图（孤立 pose 欠约束 → GTSAM 奇异）。非 root
      // KF 需要 ≥2 个已锚定 landmark 观测（单观测 landmark 的联合缺秩
      // 已由延迟初始化消除；图连接由短 KF 间隔保证）。锚定门槛 =
      // clamp(min_shared_landmarks, 2, 4)。
      const std::uint32_t min_anchored =
          is_root ? 1U
                  : static_cast<std::uint32_t>( std::clamp(
                        m_impl->options.min_shared_landmarks, 2, 4 ) );
      if ( inserted_stereo == 0U ||
           ( !is_root && anchored_observations < min_anchored ) )
      {
        return degrade();
      }

      if ( is_root )
      {
        factors.emplace_shared<gtsam::PriorFactor<Pose3>>(
            pose_key, Pose3::Identity(),
            posePriorNoise( m_impl->options ) );
        values.insert( pose_key, Pose3::Identity() );
        m_impl->has_root = true;
      }
      else
      {
        // 无占位 Between：KF 间约束来自共享 landmark 观测（slice ③）；
        // 新 KF 初值 = PnP 或常量速度外推。
        values.insert( pose_key, kf_initial );
      }
      timestamps.emplace( pose_key, ts );

      // slice ④ per-KF bias 链：每 accepted KF 一个 B(kf_count)；首 KF
      // bias prior，相邻 KF random-walk edge（σ = imu_gyr_noise_nd·√dt）；
      // enable_imu=false 时整链跳过。
      estimator::FusionMode fusion_now = estimator::FusionMode::kVisionOnly;
      if ( m_impl->options.enable_imu )
      {
        const gtsam::Key     bias_key  = B( m_impl->kf_count );
        const gtsam::Vector3 zero_bias = gtsam::Vector3::Zero();
        values.insert( bias_key, zero_bias );
        timestamps.emplace( bias_key, ts );
        if ( is_root )
        {
          factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
              bias_key, zero_bias, m_impl->bias_prior_noise );
        }
        else
        {
          const double dt_s =
              m_impl->last_kf_timestamp.has_value()
                  ? static_cast<double>(
                        measurement.timestamp.nanoseconds() -
                        m_impl->last_kf_timestamp->nanoseconds() ) *
                        1e-9
                  : 0.0;
          factors.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
              B( m_impl->kf_count - 1 ), bias_key, zero_bias,
              gtsam::noiseModel::Isotropic::Sigma(
                  3, m_impl->options.imu_gyr_noise_nd *
                         std::sqrt( std::max( dt_s, 1e-9 ) ) ) );
          if ( !m_impl->imu_interval_broken &&
               m_impl->pending_imu.size() >= 2U )
          {
            try
            {
              const gtsam::Vector3 prev_bias =
                  m_impl->smoother.calculateEstimate<gtsam::Vector3>(
                      B( m_impl->kf_count - 1 ) );
              const auto preint = rebuildAhrsPreintegration(
                  m_impl->ahrs_params, m_impl->pending_imu, prev_bias );
              factors.emplace_shared<CandidatePoseAhrsFactor>(
                  X( m_impl->kf_count - 1 ), X( m_impl->kf_count ),
                  B( m_impl->kf_count - 1 ), *preint );
              fusion_now = estimator::FusionMode::kGyroVisual;
            }
            catch ( const std::exception& )
            {
              // B 出窗边界：跳过本区间 gyro factor（视觉照常）。
            }
          }
        }
        m_impl->last_fusion_mode = fusion_now;
      }
      // 每个 KF 都是新区间起点：无论是否建 factor、无论 is_root。
      m_impl->pending_imu.clear();
      m_impl->imu_interval_broken = false;
      m_impl->last_kf_timestamp   = measurement.timestamp;

      try
      {
        m_impl->smoother.update( factors, values, timestamps );
      }
      catch ( const gtsam::StereoCheiralityException& )
      {
        // 观测质量差（KF 初值误差大 → LM 中 landmark 越过相机面）。
        return degrade();
      }
      catch ( const std::out_of_range& )
      {
        // GTSAM 数值病态（"Index out of requested size"）：帧级降级。
        return degrade();
      }
      catch ( const std::invalid_argument& )
      {
        // GTSAM 边缘化边界（"not unused"）：帧级降级。
        return degrade();
      }
      catch ( const std::runtime_error& )
      {
        // GTSAM 数值/数据问题（Indeterminant 等）：帧级降级。
        return degrade();
      }
      catch ( const gtsam::IndeterminantLinearSystemException& )
      {
        // GTSAM 秩亏（ThreadsafeException 链，非 runtime_error 子类）：
        // 帧级降级。
        return degrade();
      }

      // 新 landmark 只在 update 成功后进入账本（退化/异常路径不污染）。
      for ( const gtsam::Key key : new_landmark_keys )
      {
        m_impl->active_landmarks.insert( key );
      }

      for ( const std::size_t slot :
            m_impl->smoother.getISAM2Result().newFactorsIndices )
      {
        m_impl->ledger.emplace_back(
            slot, is_root ? "pose_prior" : "between" );
      }
      ++m_impl->kf_count;

      // 自然退休：本帧后从 timestamps() 消失的 L 前缀 key。
      const std::uint32_t retired_landmarks =
          m_impl->retireExpiredLandmarks();

      const Pose3 newest =
          m_impl->smoother.calculateEstimate<Pose3>( pose_key );
      m_impl->prev_pose = m_impl->last_pose;
      m_impl->last_pose = newest;

      estimator::VioEstimate estimate;
      estimate.timestamp    = measurement.timestamp;
      estimate.T_W_B        = toIsometry3d( newest );
      m_impl->last_estimate = estimate;

      result.vio.status                       = estimator::UpdateStatus::kOk;
      result.vio.estimate                     = estimate;
      result.vio.diagnostics.fusion_mode      = fusion_now;
      result.vio.diagnostics.num_observations = toU32(
          measurement.observations.size() );
      result.vio.diagnostics.num_disparity = toU32(
          static_cast<std::size_t>( std::count_if(
              measurement.observations.begin(),
              measurement.observations.end(),
              []( const estimator::StereoObservation& obs ) { return obs.disparity_px > 0.0; } ) ) );
      result.vio.diagnostics.num_landmarks =
          toU32( m_impl->active_landmarks.size() );

      result.graph                        = m_impl->stats();
      result.graph.retired_landmark_count = retired_landmarks;
      std::uint32_t prev_active_poses     = 0;
      for ( const auto& [ key, key_timestamp ] :
            snapshot.smoother.timestamps() )
      {
        if ( gtsam::Symbol( key ).chr() == 'x' )
        {
          ++prev_active_poses;
        }
      }
      result.graph.marginalized_pose_count =
          prev_active_poses + 1U > result.graph.active_pose_count
              ? prev_active_poses + 1U - result.graph.active_pose_count
              : 0U;
      return result;
    }
    catch ( const std::exception& )
    {
      m_impl->smoother            = std::move( snapshot.smoother );
      m_impl->ledger              = std::move( snapshot.ledger );
      m_impl->active_landmarks    = std::move( snapshot.active_landmarks );
      m_impl->observed_once       = std::move( snapshot.observed_once );
      m_impl->frame_count         = snapshot.frame_count;
      m_impl->kf_count            = snapshot.kf_count;
      m_impl->has_root            = snapshot.has_root;
      m_impl->last_estimate       = std::move( snapshot.last_estimate );
      m_impl->last_pose           = std::move( snapshot.last_pose );
      m_impl->prev_pose           = std::move( snapshot.prev_pose );
      m_impl->last_kf_timestamp   = snapshot.last_kf_timestamp;
      m_impl->last_fusion_mode    = snapshot.last_fusion_mode;
      m_impl->pending_imu         = std::move( snapshot.pending_imu );
      m_impl->imu_interval_broken = snapshot.imu_interval_broken;
      throw;
    }
  }

}  // namespace phad::apps
