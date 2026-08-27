// M4.4 gyro-visual 合成对拍：视觉观测与 IMU 测量由同一地面真值运动
// 生成，验证 AHRS 预积分、ΣΔt、staged activation、pending 拼接、gap
// 退化，以及 IMU-on / IMU-off 的行为分界。
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <functional>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include "phad/camera/rectified_stereo_calibration.hpp"
#include "phad/estimator/stereo_vo_estimator.hpp"
#include "phad/sensor/imu_measurement.hpp"
#include "phad/sensor/rigid_transform.hpp"

namespace
{

  using phad::camera::RectifiedStereoCalibration;
  using phad::estimator::EstimatorOptions;
  using phad::estimator::FusionMode;
  using phad::estimator::KeyframeMeasurement;
  using phad::estimator::LandmarkId;
  using phad::estimator::StereoObservation;
  using phad::estimator::StereoVoEstimator;
  using phad::estimator::UpdateStatus;
  using phad::sensor::ImuMeasurement;
  using phad::sensor::RigidTransform;

  // Z-up 世界系, 伪初始化重力 (C4/C5): g_w = [0, 0, -g]。
  constexpr double      kGravity = 9.81007;
  const Eigen::Vector3d kGravityWorld{ 0.0, 0.0, -kGravity };
  const Eigen::Vector3d kZero3 = Eigen::Vector3d::Zero();

  // 运动学: 位置/旋转/世界系加速度/body 系角速度的时间函数 (秒)。
  struct ImuMotion
  {
    std::function<Eigen::Isometry3d( double )> pose_at;
    std::function<Eigen::Vector3d( double )>   accel_world;  // a_w(t)
    std::function<Eigen::Vector3d( double )>   omega_body;   // ω_b(t)
  };

  RectifiedStereoCalibration makeCalibration()
  {
    auto rigid = RigidTransform::create( Eigen::Isometry3d::Identity().matrix() )
                     .value();
    return RectifiedStereoCalibration::create(
               400.0, 400.0, 320.0, 240.0, 0.12, 640, 480, std::move( rigid ) )
        .value();
  }

  [[nodiscard]] StereoObservation projectLandmark(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d& T_W_B, LandmarkId id,
      const Eigen::Vector3d& point_W )
  {
    Eigen::Isometry3d T_B_C          = Eigen::Isometry3d::Identity();
    T_B_C.linear()                   = calibration.T_B_left_rectified().rotation();
    T_B_C.translation()              = calibration.T_B_left_rectified().translation();
    const Eigen::Vector3d point_left = ( T_W_B * T_B_C ).inverse() * point_W;
    const double          z          = point_left.z();
    EXPECT_GT( z, 0.0 );
    const double u_l =
        calibration.fxPixels() * point_left.x() / z + calibration.cxPixels();
    const double v =
        calibration.fyPixels() * point_left.y() / z + calibration.cyPixels();
    const double disparity =
        calibration.fxPixels() * calibration.baselineM() / z;
    return StereoObservation{ id, Eigen::Vector2d( u_l, v ), disparity };
  }

  // IMU 测量模型 (设计稿 §2.3): a_m = R^T (a_w - g_w), ω_m = ω_b。
  // bias 叠加在测量上; noise 用固定种子保证确定性。
  [[nodiscard]] ImuMeasurement imuAt(
      const ImuMotion& motion, double t_s, std::int64_t t_ns,
      const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyro,
      std::mt19937* rng = nullptr, double noise_acc = 0.0,
      double noise_gyr = 0.0 )
  {
    const Eigen::Vector3d a_w = motion.accel_world( t_s );
    const Eigen::Matrix3d R   = motion.pose_at( t_s ).linear();
    Eigen::Vector3d       a_m = R.transpose() * ( a_w - kGravityWorld ) +
                          bias_acc;
    Eigen::Vector3d omega_m = motion.omega_body( t_s ) + bias_gyro;
    if ( rng != nullptr )
    {
      std::normal_distribution<double> acc_noise( 0.0, noise_acc );
      std::normal_distribution<double> gyr_noise( 0.0, noise_gyr );
      for ( int axis = 0; axis < 3; ++axis )
      {
        a_m[ axis ] += acc_noise( *rng );
        omega_m[ axis ] += gyr_noise( *rng );
      }
    }
    return ImuMeasurement{ phad::common::Timestamp{ t_ns },
                           { a_m.x(), a_m.y(), a_m.z() },
                           { omega_m.x(), omega_m.y(), omega_m.z() } };
  }

  // 段生成 (M4.1 切段语义): n_samples 个样本均匀覆盖 [t_prev, t_cur] 且含
  // 两端; 样本 i 覆盖区间 [t_i, t_{i+1}], 右端样本不积分, ΣΔt ≡ t_cur - t_prev。
  // 相邻段共享右端样本 (右端 = 下段左端)。t_prev/t_cur 为运动学采样场景时间;
  // t_ns_abs_offset_s 使样本的 ns 时间戳落在绝对帧时间上 (场景时间 = 绝对
  // 时间 − kInitFrames·dt 平移, M4.3 缓冲拼接要求段与帧时间戳一致; M4.2
  // 预积分只取 Δt, 平移不影响, 故默认 0 保持原行为)。
  [[nodiscard]] std::vector<ImuMeasurement> makeImuSegment(
      const ImuMotion& motion, double t_prev_s, double t_cur_s,
      int n_samples, const Eigen::Vector3d& bias_acc = kZero3,
      const Eigen::Vector3d& bias_gyro = kZero3, std::mt19937* rng = nullptr,
      double noise_acc = 0.0, double noise_gyr = 0.0,
      double t_ns_abs_offset_s = 0.0 )
  {
    std::vector<ImuMeasurement> segment;
    segment.reserve( static_cast<std::size_t>( n_samples ) );
    for ( int i = 0; i < n_samples; ++i )
    {
      const double frac = static_cast<double>( i ) /
                          static_cast<double>( n_samples - 1 );
      const double       t_s  = t_prev_s + frac * ( t_cur_s - t_prev_s );
      const std::int64_t t_ns = static_cast<std::int64_t>(
          ( t_ns_abs_offset_s + t_prev_s ) * 1e9 +
          frac * ( t_cur_s - t_prev_s ) * 1e9 );
      segment.push_back(
          imuAt( motion, t_s, t_ns, bias_acc, bias_gyro, rng, noise_acc,
                 noise_gyr ) );
    }
    return segment;
  }

  [[nodiscard]] KeyframeMeasurement makeFrameWithImu(
      const RectifiedStereoCalibration& calibration,
      const Eigen::Isometry3d& T_W_B, std::int64_t t_prev_ns,
      std::int64_t t_cur_ns, const std::vector<Eigen::Vector3d>& landmarks_W,
      const std::vector<LandmarkId>&     ids,
      const std::vector<ImuMeasurement>& segment, bool imu_gap = false )
  {
    KeyframeMeasurement measurement;
    measurement.timestamp   = phad::common::Timestamp{ t_cur_ns };
    measurement.t_prev      = phad::common::Timestamp{ t_prev_ns };
    measurement.imu_samples = segment;
    measurement.imu_gap     = imu_gap;
    for ( std::size_t index = 0; index < landmarks_W.size(); ++index )
    {
      measurement.observations.push_back( projectLandmark(
          calibration, T_W_B, ids[ index ], landmarks_W[ index ] ) );
    }
    return measurement;
  }

  [[nodiscard]] std::vector<LandmarkId> sequentialIds( std::size_t count,
                                                       LandmarkId  start = 1 )
  {
    std::vector<LandmarkId> ids;
    ids.reserve( count );
    for ( std::size_t index = 0; index < count; ++index )
    {
      ids.push_back( start + static_cast<LandmarkId>( index ) );
    }
    return ids;
  }

  // 与 reanchor 测试同源的路标集 (z ≈ 4.2-6 m, 保证全程在视锥内)。
  const std::vector<Eigen::Vector3d> kLandmarks{
      { 0.4, 0.1, 5.0 },
      { -0.3, 0.2, 4.5 },
      { 0.1, -0.25, 6.0 },
      { 0.6, -0.1, 5.5 },
      { -0.5, -0.2, 4.8 },
      { 0.0, 0.3, 5.2 },
      { 0.25, 0.15, 4.2 },
      { -0.2, -0.15, 5.8 },
      { 0.35, -0.05, 5.3 },
      { -0.15, 0.25, 4.6 },
  };

  struct RunResult
  {
    std::vector<Eigen::Isometry3d>                accepted;
    std::vector<phad::estimator::VioUpdateResult> results;
  };

  // ── M4.3 静止 init 前缀语义 ──
  // 静止运动定义见下方运动学场景节。
  const ImuMotion& stationaryMotion();
  // 帧 0..kInitFrames 静止，视觉从首帧正常接受；前 kInitFrames 帧在
  // diagnostics 中标记 init_pending，末帧发布一次 audit snapshot。运动场景
  // 从末帧的同一 identity pose（τ=0）继续。
  constexpr int kInitFrames = 10;  // 0.5 s @ 20 Hz

  // 场景帧输入 (前缀语义): 场景帧 j (≥ 1) 的绝对时间 = (kInitFrames + j)·dt,
  // 场景时间 τ = j·dt; 段样本按场景时间生成。
  struct ScenarioInput
  {
    std::int64_t                t_prev_ns;
    std::int64_t                t_cur_ns;
    std::vector<ImuMeasurement> segment;
    Eigen::Isometry3d           T_W_B;
  };

  [[nodiscard]] ScenarioInput scenarioInput(
      const ImuMotion& motion, double dt_img_s, int j, int n_samples,
      const Eigen::Vector3d& bias_acc  = kZero3,
      const Eigen::Vector3d& bias_gyro = kZero3, std::mt19937* rng = nullptr,
      double noise_acc = 0.0, double noise_gyr = 0.0 )
  {
    ScenarioInput input;
    input.t_cur_ns = static_cast<std::int64_t>(
        static_cast<double>( kInitFrames + j ) * dt_img_s * 1e9 );
    input.t_prev_ns = static_cast<std::int64_t>(
        static_cast<double>( kInitFrames + j - 1 ) * dt_img_s * 1e9 );
    const double tau = static_cast<double>( j ) * dt_img_s;
    input.segment    = makeImuSegment( motion, tau - dt_img_s, tau, n_samples,
                                       bias_acc, bias_gyro, rng, noise_acc,
                                       noise_gyr,
                                       static_cast<double>( kInitFrames ) *
                                           dt_img_s );
    input.T_W_B      = motion.pose_at( tau );
    return input;
  }

  // 静止 audit 前缀：视觉帧全部接受；末帧发布 snapshot。返回末帧位姿
  // （运动场景 τ=0 的 identity anchor）。
  [[nodiscard]] Eigen::Isometry3d runStaticInitPrefix(
      StereoVoEstimator& estimator, const RectifiedStereoCalibration& calibration,
      double dt_img_s, const Eigen::Vector3d& bias_acc = kZero3,
      const Eigen::Vector3d& bias_gyro = kZero3 )
  {
    const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
    for ( int i = 0; i <= kInitFrames; ++i )
    {
      const double       t_cur_s = static_cast<double>( i ) * dt_img_s;
      const std::int64_t t_cur_ns =
          static_cast<std::int64_t>( t_cur_s * 1e9 );
      const std::int64_t          t_prev_ns = i == 0
                                                  ? 0
                                                  : static_cast<std::int64_t>(
                                               ( static_cast<double>( i ) -
                                                 1.0 ) *
                                               dt_img_s * 1e9 );
      std::vector<ImuMeasurement> segment;
      if ( i > 0 )
      {
        segment = makeImuSegment( stationaryMotion(), t_cur_s - dt_img_s,
                                  t_cur_s, 6, bias_acc, bias_gyro );
      }
      auto result = estimator.update( makeFrameWithImu(
                                          calibration, stationaryMotion().pose_at( t_cur_s ), t_prev_ns,
                                          t_cur_ns, kLandmarks, ids, segment ),
                                      true );
      EXPECT_EQ( result.status, UpdateStatus::kOk )
          << "init frame " << i << ": " << result.message;
      EXPECT_TRUE( result.estimate.has_value() ) << "init frame " << i;
      if ( i < kInitFrames )
      {
        EXPECT_TRUE( result.diagnostics.init_pending ) << "init frame " << i;
      }
      else
      {
        EXPECT_TRUE( result.diagnostics.imu_init.has_value() );
        return result.estimate->T_W_B;
      }
    }
    ADD_FAILURE() << "unreachable: seed frame not reached";
    return Eigen::Isometry3d::Identity();
  }

  // 帧时间: 首帧 t=0, 之后每帧 dt_img_s。IMU 段 6 样本/帧 (10 ms 步长, 含两端)。
  // 首帧 t_prev=0。imu_init=true 时先跑 non-blocking 静止 audit 前缀；
  // 为让运动学断言继续以 τ=0 编号，只把前缀末帧纳入返回的 RunResult，
  // 之前已接受的视觉 warm-up 帧由专门的 lifecycle 测试覆盖。场景帧
  // j ≥ 1 的 τ = j·dt；imu_init=false 时无前缀。
  [[nodiscard]] RunResult runImuSegment(
      StereoVoEstimator& estimator, const RectifiedStereoCalibration& calibration,
      const ImuMotion& motion, double dt_img_s, int frames,
      const Eigen::Vector3d& bias_acc  = kZero3,
      const Eigen::Vector3d& bias_gyro = kZero3, double noise_acc = 0.0,
      double noise_gyr = 0.0, bool imu_init = true )
  {
    const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
    RunResult                     run;
    std::mt19937                  rng( 42 );  // 固定种子, 确定性噪声
    if ( imu_init )
    {
      for ( int i = 0; i <= kInitFrames; ++i )
      {
        const double       t_cur_s = static_cast<double>( i ) * dt_img_s;
        const std::int64_t t_cur_ns =
            static_cast<std::int64_t>( t_cur_s * 1e9 );
        const std::int64_t          t_prev_ns = i == 0
                                                    ? 0
                                                    : static_cast<std::int64_t>(
                                                 ( static_cast<double>( i ) -
                                                   1.0 ) *
                                                 dt_img_s * 1e9 );
        std::vector<ImuMeasurement> segment;
        if ( i > 0 )
        {
          segment = makeImuSegment( stationaryMotion(),
                                    t_cur_s - dt_img_s, t_cur_s, 6, bias_acc,
                                    bias_gyro, &rng, noise_acc, noise_gyr );
        }
        auto result = estimator.update( makeFrameWithImu(
                                            calibration, stationaryMotion().pose_at( t_cur_s ), t_prev_ns,
                                            t_cur_ns, kLandmarks, ids, segment ),
                                        true );
        if ( i == kInitFrames )
        {
          run.results.push_back( result );
          if ( result.status == UpdateStatus::kOk &&
               result.estimate.has_value() )
          {
            run.accepted.push_back( result.estimate->T_W_B );
          }
        }
      }
    }
    for ( int j = imu_init ? 1 : 0; j < frames; ++j )
    {
      const double       tau      = static_cast<double>( j ) * dt_img_s;
      const std::int64_t t_cur_ns = static_cast<std::int64_t>(
          static_cast<double>( imu_init ? kInitFrames + j : j ) *
          dt_img_s * 1e9 );
      const std::int64_t t_prev_ns = static_cast<std::int64_t>(
          static_cast<double>( imu_init ? kInitFrames + j - 1 : j - 1 ) *
          dt_img_s * 1e9 );
      std::vector<ImuMeasurement> segment;
      if ( j > 0 )
      {
        segment = makeImuSegment( motion, tau - dt_img_s, tau, 6, bias_acc,
                                  bias_gyro, &rng, noise_acc, noise_gyr,
                                  static_cast<double>( kInitFrames ) *
                                      dt_img_s );
      }
      const Eigen::Isometry3d T_W_B  = motion.pose_at( tau );
      auto                    result = estimator.update( makeFrameWithImu(
                                          calibration, T_W_B, t_prev_ns, t_cur_ns, kLandmarks, ids, segment ),
                                                         true );
      run.results.push_back( result );
      if ( result.status == UpdateStatus::kOk && result.estimate.has_value() )
      {
        run.accepted.push_back( result.estimate->T_W_B );
      }
    }
    return run;
  }

  // 所有返回帧必须 ok 且逐帧贴合真值；accepted[j] 对应 τ=j·dt。
  void expectAllOkAndClose( const RunResult& run, const ImuMotion& motion,
                            double dt_img_s, double tol_trans_m,
                            double tol_rot_rad       = 1e-2,
                            bool   expect_gyro_state = true )
  {
    ASSERT_EQ( run.accepted.size(), run.results.size() );
    for ( std::size_t index = 0; index < run.accepted.size(); ++index )
    {
      const auto& result = run.results[ index ];
      EXPECT_EQ( result.status, UpdateStatus::kOk ) << "frame " << index
                                                    << ": " << result.message;
      EXPECT_EQ( result.diagnostics.gyro_state.has_value(), expect_gyro_state )
          << "frame " << index;
      if ( expect_gyro_state && result.diagnostics.gyro_state.has_value() )
      {
        const auto& state = *result.diagnostics.gyro_state;
        const bool  expects_prediction =
            result.diagnostics.fusion_mode == FusionMode::kGyroVisual;
        EXPECT_EQ( state.prediction_valid, expects_prediction )
            << "frame " << index;
        EXPECT_GE( state.imu_sample_count, 2U ) << "frame " << index;
        EXPECT_LT( state.imu_t_i_ns, state.imu_t_j_ns ) << "frame " << index;
        EXPECT_NEAR(
            state.imu_dt_s,
            static_cast<double>( state.imu_t_j_ns - state.imu_t_i_ns ) * 1e-9,
            1e-12 )
            << "frame " << index;
        EXPECT_TRUE( state.predicted_T_W_B.matrix().allFinite() )
            << "frame " << index;
        EXPECT_TRUE( state.prediction_bias_gyro.allFinite() )
            << "frame " << index;
      }
      const double             t_s   = static_cast<double>( index ) * dt_img_s;
      const Eigen::Isometry3d& est   = run.accepted[ index ];
      const Eigen::Isometry3d& truth = motion.pose_at( t_s );
      const double             d_trans =
          ( est.translation() - truth.translation() ).norm();
      EXPECT_NEAR( d_trans, 0.0, tol_trans_m ) << "frame " << index;
      const Eigen::Matrix3d dR = est.linear() * truth.linear().transpose();
      EXPECT_NEAR( Eigen::AngleAxisd( dR ).angle(), 0.0, tol_rot_rad )
          << "frame " << index;
    }
  }

  // ---- 运动学场景 ----

  // 静止: 位姿恒等, 速度 0 → a_m = -g_w 常量, ω_m = 0。
  const ImuMotion& stationaryMotion()
  {
    static const ImuMotion motion{
        []( double ) { return Eigen::Isometry3d::Identity(); },
        []( double ) { return Eigen::Vector3d::Zero(); },
        []( double ) { return Eigen::Vector3d::Zero(); },
    };
    return motion;
  }

  // 匀速直线 (带起动斜坡): p(τ) = ½aτ² (τ ≤ τ_ramp), 之后 ½aτ_ramp² +
  // v(τ−τ_ramp), R = I; a_w = a 常量 (τ ≤ τ_ramp), 之后 0。
  // M4.3 前缀语义下场景帧从 τ=0 起步 (播种帧速度 = 0), 瞬时速度阶跃无法被
  // 离散样本预积分表示 (pair1 静止缓冲约束 v_seed = v_0, 真值却为 v) ——
  // 斜坡使加速度出现在样本中, 预积分与真值一致。
  ImuMotion constantVelocityMotion( const Eigen::Vector3d& v )
  {
    constexpr double      tau_ramp = 0.1;  // s, 加速期
    const Eigen::Vector3d a        = v / tau_ramp;
    return ImuMotion{
        [ v, a, tau_ramp ]( double t ) {
          Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
          if ( t <= tau_ramp )
          {
            T.translation() = 0.5 * a * ( t * t );
          }
          else
          {
            T.translation() =
                0.5 * a * ( tau_ramp * tau_ramp ) + v * ( t - tau_ramp );
          }
          return T;
        },
        [ a, tau_ramp ]( double t ) {
          return t < tau_ramp ? a : Eigen::Vector3d::Zero();
        },
        []( double ) { return Eigen::Vector3d::Zero(); },
    };
  }

  // 恒定角速度 (绕 body/world y 轴) + 匀速平移 (带起动斜坡, 同
  // constantVelocityMotion): θ(t) = ω t, R(t) = RotY(θ); ω_b = [0, ω, 0]。
  ImuMotion constantRotationMotion( double omega_y, const Eigen::Vector3d& v )
  {
    constexpr double      tau_ramp = 0.1;
    const Eigen::Vector3d a        = v / tau_ramp;
    return ImuMotion{
        [ omega_y, v, a, tau_ramp ]( double t ) {
          Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
          T.linear() =
              Eigen::AngleAxisd( omega_y * t, Eigen::Vector3d::UnitY() )
                  .toRotationMatrix();
          if ( t <= tau_ramp )
          {
            T.translation() = 0.5 * a * ( t * t );
          }
          else
          {
            T.translation() =
                0.5 * a * ( tau_ramp * tau_ramp ) + v * ( t - tau_ramp );
          }
          return T;
        },
        [ a, tau_ramp ]( double t ) {
          return t < tau_ramp ? a : Eigen::Vector3d::Zero();
        },
        [ omega_y ]( double ) {
          return Eigen::Vector3d( 0.0, omega_y, 0.0 );
        },
    };
  }

  struct DiscreteAlignmentState
  {
    Eigen::Isometry3d T_W_B  = Eigen::Isometry3d::Identity();
    Eigen::Vector3d   acc_W  = Eigen::Vector3d::Zero();
    Eigen::Vector3d   rate_B = Eigen::Vector3d::Zero();
  };

  [[nodiscard]] Eigen::Matrix3d richRotation( const double t_s )
  {
    const double roll  = 0.4 * std::sin( 0.9 * t_s );
    const double pitch = 0.35 * std::sin( 0.7 * t_s );
    const double yaw   = 0.45 * std::sin( 0.6 * t_s );
    return ( Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ() ) *
             Eigen::AngleAxisd( pitch, Eigen::Vector3d::UnitY() ) *
             Eigen::AngleAxisd( roll, Eigen::Vector3d::UnitX() ) )
        .toRotationMatrix();
  }

  [[nodiscard]] std::vector<DiscreteAlignmentState> alignmentStates(
      const int intervals, const double dt_s )
  {
    std::vector<DiscreteAlignmentState> states(
        static_cast<std::size_t>( intervals + 1 ) );
    const Eigen::Vector3d frequency{ 2.0, 2.4, 2.8 };
    Eigen::Vector3d       position = Eigen::Vector3d::Zero();
    Eigen::Vector3d       velocity{ 0.6, 0.4, 0.25 };
    for ( int index = 0; index <= intervals; ++index )
    {
      const double t_s          = static_cast<double>( index ) * dt_s;
      auto&        state        = states[ static_cast<std::size_t>( index ) ];
      state.T_W_B.linear()      = richRotation( t_s );
      state.T_W_B.translation() = position;
      state.acc_W =
          -frequency.cwiseProduct( frequency ).cwiseProduct( position );
      if ( index < intervals )
      {
        const Eigen::Matrix3d   R_next = richRotation( t_s + dt_s );
        const Eigen::AngleAxisd delta(
            state.T_W_B.linear().transpose() * R_next );
        state.rate_B = delta.axis() * delta.angle() / dt_s;
      }
      else
      {
        state.rate_B = states[ static_cast<std::size_t>( index - 1 ) ].rate_B;
      }
      position += velocity * dt_s + 0.5 * state.acc_W * dt_s * dt_s;
      velocity += state.acc_W * dt_s;
    }
    return states;
  }

  [[nodiscard]] ImuMeasurement alignmentSample(
      const DiscreteAlignmentState& state, const std::int64_t timestamp_ns,
      const Eigen::Vector3d& bias_acc, const Eigen::Vector3d& bias_gyro )
  {
    const Eigen::Vector3d accel =
        state.T_W_B.linear().transpose() * ( state.acc_W - kGravityWorld ) +
        bias_acc;
    const Eigen::Vector3d gyro = state.rate_B + bias_gyro;
    return ImuMeasurement{ phad::common::Timestamp{ timestamp_ns },
                           { accel.x(), accel.y(), accel.z() },
                           { gyro.x(), gyro.y(), gyro.z() } };
  }

  [[nodiscard]] EstimatorOptions imuOptions()
  {
    EstimatorOptions options;
    options.min_track_observations_for_seed = 1;  // tests seed at 2 frames
    options.window_size                     = 10;
    options.min_shared_landmarks            = 3;
    options.min_seed_observations           = 10;
    // 合成观测零噪声, 像素 sigma 保持默认即可; IMU 参数取默认 EuRoC 值。
    return options;
  }

}  // namespace

// ── §4.7-a 静止: 只有重力的 IMU 段 + 零运动视觉 → 位姿保持。 ──
TEST( StereoVoImu, StationaryKeepsPose )
{
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  const auto        run = runImuSegment( estimator, makeCalibration(),
                                         stationaryMotion(), 0.05, 10 );
  expectAllOkAndClose( run, stationaryMotion(), 0.05, 2e-2 );
}

// ── §4.7-b 匀速: 视觉位移 = IMU 积分位移。 ──
TEST( StereoVoImu, ConstantVelocityMatchesGroundTruth )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  StereoVoEstimator     estimator( makeCalibration(), imuOptions() );
  const auto            run = runImuSegment( estimator, makeCalibration(), motion, 0.05,
                                             12 );
  expectAllOkAndClose( run, motion, 0.05, 2e-2 );
  // ΣΔt ≡ 图像间隔: 12 帧后累计位移 = p(0.55) ≈ 0.15 m (0.3 m/s 巡航
  // 0.45 s + 斜坡 0.015 m; 相对误差 < 1%)。
  const double d = run.accepted.back().translation().norm();
  EXPECT_NEAR( d, motion.pose_at( 0.55 ).translation().norm(), 1.0e-2 );
}

// ── §4.7-c 恒定角速度 (rad-vs-deg): 姿态轨迹贴合真值。 ──
TEST( StereoVoImu, ConstantRotationMatchesGroundTruth )
{
  const Eigen::Vector3d v{ 0.2, 0.0, 0.0 };
  const ImuMotion       motion = constantRotationMotion( 0.2, v );
  StereoVoEstimator     estimator( makeCalibration(), imuOptions() );
  const auto            run = runImuSegment( estimator, makeCalibration(), motion, 0.05,
                                             12 );
  expectAllOkAndClose( run, motion, 0.05, 3e-2, 2e-2 );
}

// ── §4.7-d 已知 bias: 静止 + 恒定偏置 → 偏置被吸收, 位姿不漂移。 ──
TEST( StereoVoImu, KnownBiasIsAbsorbed )
{
  const Eigen::Vector3d bias_acc{ 0.3, -0.2, 0.1 };
  const Eigen::Vector3d bias_gyro{ 0.02, -0.01, 0.015 };
  const ImuMotion       motion = stationaryMotion();
  StereoVoEstimator     estimator( makeCalibration(), imuOptions() );
  // 0.5 s 静止。bias 若无估计, 预积分漂移 ≈ 0.5 * 0.25/2 = 6 cm —— 容差
  // 3 cm 区分「偏置被图吸收」与「伪初始化完全不估计」。
  // M4.3: init 的 R_W_I0 用含 bias 的 a_mean 定 roll/pitch → 姿态偏
  // |bias_acc|/g ≈ 0.038 rad (旋转容差放宽到 6e-2);平移仍 ≈ 0 (bias 被
  // 图吸收, 姿态倾斜与地图一致)。
  const auto run =
      runImuSegment( estimator, makeCalibration(), motion, 0.05, 10, bias_acc,
                     bias_gyro );
  expectAllOkAndClose( run, motion, 0.05, 3e-2, 6e-2 );
}

TEST( StereoVoImu, GyroFactorUsesAbsoluteBiasWithoutDoubleCorrection )
{
  EstimatorOptions options        = imuOptions();
  options.imu_gyro_align_window_s = 0.2;
  options.enable_vio_state_probe  = true;
  const Eigen::Vector3d bias_gyro{ 0.02, -0.01, 0.015 };
  const ImuMotion       motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator estimator( makeCalibration(), options );

  const RunResult run = runImuSegment(
      estimator, makeCalibration(), motion, 0.05, 12, kZero3, bias_gyro );
  expectAllOkAndClose( run, motion, 0.05, 3e-2, 2e-2 );

  bool saw_active = false;
  for ( const auto& result : run.results )
  {
    if ( result.diagnostics.fusion_mode != FusionMode::kGyroVisual )
    {
      continue;
    }
    saw_active = true;
    EXPECT_TRUE( result.diagnostics.bias_gyro.isApprox( bias_gyro, 1e-8 ) );
    EXPECT_GT( result.diagnostics.gyro_factor_count, 0U );
    ASSERT_TRUE( result.diagnostics.gyro_graph_cost.has_value() );
    const auto& cost = *result.diagnostics.gyro_graph_cost;
    EXPECT_EQ( cost.gyro_factor_count,
               result.diagnostics.gyro_factor_count );
    EXPECT_EQ( cost.interior_factor_count + 1U,
               cost.gyro_factor_count );
    EXPECT_GT( cost.stereo_factor_count, 0U );
    EXPECT_GE( cost.boundary_initial_residual_norm_rad, 0.0 );
    EXPECT_GE( cost.boundary_posterior_residual_norm_rad, 0.0 );
    EXPECT_GE( cost.boundary_initial_whitened_norm, 0.0 );
    EXPECT_GE( cost.boundary_posterior_whitened_norm, 0.0 );
    EXPECT_GE( cost.newest_initial_residual_norm_rad, 0.0 );
    EXPECT_GE( cost.newest_posterior_residual_norm_rad, 0.0 );
    EXPECT_GE( cost.newest_initial_whitened_norm, 0.0 );
    EXPECT_GE( cost.newest_posterior_whitened_norm, 0.0 );
  }
  EXPECT_TRUE( saw_active );
}

TEST( StereoVoImu, StaticInitDoesNotBlockVisualUpdatesBeforeGyroActivation )
{
  EstimatorOptions vio_options        = imuOptions();
  vio_options.imu_gyro_align_window_s = 100.0;
  EstimatorOptions vo_options         = vio_options;
  vo_options.enable_imu               = false;
  StereoVoEstimator             vio( makeCalibration(), vio_options );
  StereoVoEstimator             vo( makeCalibration(), vo_options );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );

  int init_snapshot_count = 0;
  for ( int i = 0; i <= kInitFrames + 2; ++i )
  {
    const double       t_cur_s  = static_cast<double>( i ) * 0.05;
    const std::int64_t t_cur_ns = static_cast<std::int64_t>( t_cur_s * 1e9 );
    const std::int64_t t_prev_ns =
        i == 0 ? 0 : static_cast<std::int64_t>( ( t_cur_s - 0.05 ) * 1e9 );
    std::vector<ImuMeasurement> segment;
    if ( i > 0 )
    {
      segment = makeImuSegment( stationaryMotion(), t_cur_s - 0.05,
                                t_cur_s, 6 );
    }
    const KeyframeMeasurement measurement = makeFrameWithImu(
        makeCalibration(), stationaryMotion().pose_at( t_cur_s ), t_prev_ns,
        t_cur_ns, kLandmarks, ids, segment );
    const auto vio_result = vio.update( measurement, true );
    const auto vo_result  = vo.update( measurement, true );

    EXPECT_EQ( vio_result.status, vo_result.status ) << "frame " << i;
    ASSERT_EQ( vio_result.estimate.has_value(),
               vo_result.estimate.has_value() )
        << "frame " << i;
    if ( vio_result.estimate.has_value() )
    {
      EXPECT_TRUE( vio_result.estimate->T_W_B.matrix().isApprox(
          vo_result.estimate->T_W_B.matrix(), 1e-12 ) )
          << "frame " << i;
    }
    EXPECT_EQ( vio_result.diagnostics.fusion_mode, FusionMode::kVisionOnly );
    EXPECT_EQ( vio_result.diagnostics.gyro_factor_count, 0U );
    if ( i < kInitFrames )
    {
      EXPECT_TRUE( vio_result.diagnostics.init_pending ) << "frame " << i;
    }
    if ( vio_result.diagnostics.imu_init.has_value() )
    {
      ++init_snapshot_count;
    }
  }
  EXPECT_EQ( init_snapshot_count, 1 );
}

TEST( StereoVoImu, AccelerometerDoesNotAffectGyroVisualTrajectory )
{
  const ImuMotion motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator nominal( makeCalibration(), imuOptions() );
  StereoVoEstimator adversarial( makeCalibration(), imuOptions() );

  const RunResult nominal_run = runImuSegment(
      nominal, makeCalibration(), motion, 0.05, 12 );
  const RunResult adversarial_run = runImuSegment(
      adversarial, makeCalibration(), motion, 0.05, 12,
      Eigen::Vector3d{ 1.0, -0.7, 0.4 } );

  ASSERT_EQ( nominal_run.accepted.size(), adversarial_run.accepted.size() );
  ASSERT_FALSE( nominal_run.accepted.empty() );
  const Eigen::Isometry3d T_N0_Ni = nominal_run.accepted.front().inverse();
  const Eigen::Isometry3d T_A0_Ai = adversarial_run.accepted.front().inverse();
  for ( std::size_t i = 0; i < nominal_run.accepted.size(); ++i )
  {
    const Eigen::Isometry3d T_N0_Nj = T_N0_Ni * nominal_run.accepted[ i ];
    const Eigen::Isometry3d T_A0_Aj = T_A0_Ai * adversarial_run.accepted[ i ];
    EXPECT_NEAR(
        ( T_N0_Nj.translation() - T_A0_Aj.translation() ).norm(), 0.0,
        1e-9 )
        << "frame " << i;
    EXPECT_NEAR(
        Eigen::AngleAxisd( T_N0_Nj.linear().transpose() *
                           T_A0_Aj.linear() )
            .angle(),
        0.0, 1e-9 )
        << "frame " << i;
  }
}

TEST( StereoVoImu, StaticInitPublishesOneTruthSnapshot )
{
  const Eigen::Vector3d bias_acc{ 0.3, -0.2, 0.1 };
  const Eigen::Vector3d bias_gyro{ 0.02, -0.01, 0.015 };
  StereoVoEstimator     estimator( makeCalibration(), imuOptions() );
  const auto            run = runImuSegment(
      estimator, makeCalibration(), stationaryMotion(), 0.05, 4, bias_acc,
      bias_gyro );

  const phad::estimator::ImuInitDiagnostics* snapshot    = nullptr;
  std::size_t                                event_count = 0;
  for ( const auto& result : run.results )
  {
    if ( result.diagnostics.imu_init.has_value() )
    {
      ++event_count;
      snapshot = &*result.diagnostics.imu_init;
    }
  }
  ASSERT_EQ( event_count, 1U );
  ASSERT_NE( snapshot, nullptr );

  const Eigen::Vector3d expected_acc =
      Eigen::Vector3d( 0.0, 0.0, kGravity ) + bias_acc;
  const Eigen::Vector3d expected_acc_bias =
      expected_acc - kGravity * expected_acc.normalized();
  EXPECT_EQ( snapshot->imu_sample_count, 51U );
  EXPECT_EQ( snapshot->imu_t_i_ns, 0 );
  EXPECT_EQ( snapshot->imu_t_j_ns, 500'000'000 );
  EXPECT_NEAR( snapshot->imu_dt_s, 0.5, 1e-12 );
  EXPECT_TRUE( snapshot->gyro_mean.isApprox( bias_gyro, 1e-12 ) );
  EXPECT_TRUE( snapshot->gyro_std.isZero( 1e-12 ) );
  EXPECT_TRUE( snapshot->acc_mean.isApprox( expected_acc, 1e-12 ) );
  EXPECT_TRUE( snapshot->acc_std.isZero( 1e-12 ) );
  EXPECT_TRUE(
      ( snapshot->T_W_B0.linear() * expected_acc.normalized() )
          .isApprox( Eigen::Vector3d::UnitZ(), 1e-12 ) );
  EXPECT_TRUE( snapshot->velocity_W.isZero( 1e-12 ) );
  EXPECT_TRUE( snapshot->bias_gyro.isApprox( bias_gyro, 1e-12 ) );
  EXPECT_TRUE(
      snapshot->bias_acc.isApprox( expected_acc_bias, 1e-12 ) );
  EXPECT_NEAR( snapshot->acc_mean_norm, expected_acc.norm(), 1e-12 );
  EXPECT_DOUBLE_EQ( snapshot->gravity_model_magnitude, kGravity );
  EXPECT_DOUBLE_EQ( snapshot->gyro_std_limit,
                    imuOptions().imu_init_gyro_std );
  EXPECT_DOUBLE_EQ( snapshot->acc_std_limit,
                    imuOptions().imu_init_accel_std );
}

// ── §4.7-e covariance: 带高斯噪声 (密度量级) 的 IMU 段轨迹仍准确 ——
// 协方差 = 密度平方的换算若错误, LM 权重失当 → 漂移/发散。 ──
TEST( StereoVoImu, NoisyImuStaysAccurate )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  StereoVoEstimator     estimator( makeCalibration(), imuOptions() );
  // 噪声 σ = 密度量级 (2e-3 m/s², 1.7e-4 rad/s) 的 5 倍, 仍远小于运动
  // 信号;accel σ = 1e-2 保持在静止检测阈值 (2e-2) 以下带余量, init 在
  // 播种帧确定性通过。
  const auto run = runImuSegment( estimator, makeCalibration(), motion, 0.05,
                                  12, kZero3, kZero3, 1e-2, 1.7e-3 );
  expectAllOkAndClose( run, motion, 0.05, 5e-2 );
}

// ── §4.7-f ΣΔt: 两样本最小段 (左端 + 右端, 整段一步积分) 仍正确 ——
// rebuildPreintegration 用 samples[0] 覆盖整个 [t_prev, t_cur]。 ──
TEST( StereoVoImu, TwoSampleSegmentIntegratesWholeInterval )
{
  const Eigen::Vector3d         v{ 0.3, 0.0, 0.0 };
  const ImuMotion               motion = constantVelocityMotion( v );
  StereoVoEstimator             estimator( makeCalibration(), imuOptions() );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
  // M4.3: 静止 init 前缀 (播种帧 = accepted[0], τ=0), 场景帧 j ≥ 1 两样本段。
  std::vector<Eigen::Isometry3d> accepted{
      runStaticInitPrefix( estimator, makeCalibration(), 0.05 ) };
  for ( int j = 1; j < 12; ++j )
  {
    const ScenarioInput in     = scenarioInput( motion, 0.05, j, 2 );
    auto                result = estimator.update( makeFrameWithImu(
                                        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns, kLandmarks,
                                        ids, in.segment ),
                                                   true );
    EXPECT_EQ( result.status, UpdateStatus::kOk ) << "frame " << j
                                                  << ": " << result.message;
    if ( result.estimate.has_value() )
    {
      accepted.push_back( result.estimate->T_W_B );
    }
  }
  // 播种帧 + 11 场景帧 (末帧 τ = 0.55)。
  ASSERT_EQ( accepted.size(), 12U );
  const double d = accepted.back().translation().norm();
  EXPECT_NEAR( d, motion.pose_at( 0.55 ).translation().norm(), 1.5e-2 );
}

// ── §4.7-g imu_gap: gap 帧无 IMU 因子 (视觉照常, PnP 兜底), 链断恢复
// (weak priors 防 indeterminant), 后续帧继续正常。 ──
TEST( StereoVoImu, ImuGapFallsBackToVision )
{
  const Eigen::Vector3d         v{ 0.3, 0.0, 0.0 };
  const ImuMotion               motion = constantVelocityMotion( v );
  StereoVoEstimator             estimator( makeCalibration(), imuOptions() );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
  // M4.3: 静止 init 前缀 (播种帧 = accepted[0]), 场景帧 j ≥ 1; 场景
  // j=5 段缺失 → imu_gap (初始化已完成, gap 不再重置 init)。
  std::vector<phad::estimator::VioUpdateResult> results;
  std::vector<Eigen::Isometry3d>                accepted{
      runStaticInitPrefix( estimator, makeCalibration(), 0.05 ) };
  for ( int j = 1; j < 12; ++j )
  {
    const ScenarioInput         in = scenarioInput( motion, 0.05, j, 6 );
    std::vector<ImuMeasurement> segment =
        j == 5 ? std::vector<ImuMeasurement>{} : in.segment;
    auto result = estimator.update( makeFrameWithImu(
                                        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns, kLandmarks,
                                        ids, segment, j == 5 ),
                                    true );
    results.push_back( result );
    if ( result.status == UpdateStatus::kOk && result.estimate.has_value() )
    {
      accepted.push_back( result.estimate->T_W_B );
    }
  }
  // 全部 ok (播种帧 + 11 场景帧): gap 帧 PnP 兜底 (共享 ≥ min_pnp_inliers),
  // 后续帧链断恢复。
  ASSERT_EQ( accepted.size(), 12U );
  EXPECT_TRUE( results[ 4 ].diagnostics.pnp_success ) << "gap 帧应走 PnP";
  ASSERT_TRUE( results[ 4 ].diagnostics.gyro_state.has_value() );
  EXPECT_FALSE( results[ 4 ].diagnostics.gyro_state->prediction_valid );
  for ( std::size_t j = 1; j < 12; ++j )
  {
    const std::size_t index = j - 1;
    EXPECT_EQ( results[ index ].status, UpdateStatus::kOk )
        << "frame " << j << ": " << results[ index ].message;
    const double d_trans =
        ( accepted[ j ].translation() - motion.pose_at(
                                                  static_cast<double>( j ) * 0.05 )
                                            .translation() )
            .norm();
    EXPECT_NEAR( d_trans, 0.0, 1e-1 ) << "frame " << j;
  }
}

// ── §4.7-h pending 拼接: 被拒帧的段保留, 成功帧拼接后 ΣΔt 跨越两个图像
// 间隔, 位姿从最后接受位姿外推仍贴合真值。M4.3c (plan G) 起空观测非 gap
// 帧已放行, 故改用畸形观测触发拒绝 (段同样已入 pending)。 ──
TEST( StereoVoImu, PendingAppendsOnRejectAndConsumesOnAccept )
{
  const Eigen::Vector3d         v{ 0.3, 0.0, 0.0 };
  const ImuMotion               motion = constantVelocityMotion( v );
  StereoVoEstimator             estimator( makeCalibration(), imuOptions() );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
  // M4.3: 静止 init 前缀 (播种帧 = accepted[0]), 场景帧 j ≥ 1; 场景
  // j=5 被拒 (畸形观测: 负 disparity → rejected), 其段 (已入 pending) 保留。
  std::vector<Eigen::Isometry3d> accepted{
      runStaticInitPrefix( estimator, makeCalibration(), 0.05 ) };
  for ( int j = 1; j < 10; ++j )
  {
    const ScenarioInput in = scenarioInput( motion, 0.05, j, 6 );
    if ( j == 5 )  // 被拒帧: 畸形观测 → rejected, 段 (已入 pending) 保留
    {
      KeyframeMeasurement rejected;
      rejected.timestamp   = phad::common::Timestamp{ in.t_cur_ns };
      rejected.t_prev      = phad::common::Timestamp{ in.t_prev_ns };
      rejected.imu_samples = in.segment;
      rejected.imu_gap     = false;
      StereoObservation bad{};
      bad.left_pixel   = Eigen::Vector2d{ 0.0, 0.0 };
      bad.disparity_px = -1.0;  // 触发 "negative disparity" 拒绝
      rejected.observations.push_back( bad );
      const auto result = estimator.update( rejected, true );
      EXPECT_EQ( result.status, UpdateStatus::kRejected );
      continue;
    }
    auto result = estimator.update( makeFrameWithImu(
                                        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns, kLandmarks,
                                        ids, in.segment ),
                                    true );
    EXPECT_EQ( result.status, UpdateStatus::kOk )
        << "frame " << j << ": " << result.message;
    if ( result.estimate.has_value() )
    {
      accepted.push_back( result.estimate->T_W_B );
    }
  }
  // 9 个接受帧 (播种帧 + 场景 1..4,6..9); 末帧 (场景 j=9, 拼接段 100 ms)
  // 位姿 ≈ 真值 (τ = 9 * 0.05)。
  ASSERT_EQ( accepted.size(), 9U );
  const Eigen::Isometry3d& est   = accepted.back();
  const Eigen::Isometry3d& truth = motion.pose_at( 9 * 0.05 );
  EXPECT_NEAR( ( est.translation() - truth.translation() ).norm(), 0.0,
               2e-2 );
}

// Gyro-only 无法跨 zero-overlap 约束 translation。关键帧满足 seed 质量门
// 时必须显式 re-anchor 为新 segment，然后只在新地图上继续。
TEST( StereoVoImu, OverlapBreakReanchorsWithGyroVisual )
{
  const Eigen::Vector3d         v{ 0.3, 0.0, 0.0 };
  const ImuMotion               motion = constantVelocityMotion( v );
  StereoVoEstimator             estimator( makeCalibration(), imuOptions() );
  const std::vector<LandmarkId> ids_a = sequentialIds( kLandmarks.size() );
  // 全新 id 集合 + 稍远的路标 (覆盖平移后仍在视锥内)。
  const std::vector<Eigen::Vector3d> landmarks_b{
      { 1.4, -0.3, 5.4 },
      { 0.9, 0.35, 4.9 },
      { 1.1, -0.15, 6.2 },
      { 1.6, 0.05, 5.1 },
      { 0.65, -0.4, 4.7 },
      { 1.0, 0.2, 5.6 },
      { 1.25, -0.2, 4.4 },
      { 0.8, 0.3, 5.9 },
      { 1.35, -0.05, 5.0 },
      { 0.85, 0.15, 4.8 },
  };
  const std::vector<LandmarkId> ids_b = sequentialIds( kLandmarks.size(), 1000 );

  // 静止 init 后，j=5 起切换到全新 id 并持续跟踪新地图。
  std::vector<Eigen::Isometry3d> accepted{
      runStaticInitPrefix( estimator, makeCalibration(), 0.05 ) };
  for ( int j = 1; j < 10; ++j )
  {
    const ScenarioInput in          = scenarioInput( motion, 0.05, j, 6 );
    const bool          new_segment = j >= 5;
    const auto          landmarks   = new_segment ? landmarks_b : kLandmarks;
    const auto          ids         = new_segment ? ids_b : ids_a;
    auto                result      = estimator.update( makeFrameWithImu(
                                        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns, landmarks,
                                        ids, in.segment ),
                                                        true );
    ASSERT_EQ( result.status, UpdateStatus::kOk )
        << "frame " << j << ": " << result.message;
    EXPECT_EQ( result.diagnostics.segment_id, new_segment ? 1U : 0U )
        << "frame " << j;
    if ( result.estimate.has_value() )
    {
      accepted.push_back( result.estimate->T_W_B );
    }
  }
  // 播种帧 + 9 场景帧 (末帧 τ = 9 * 0.05)；re-anchor 不得打断
  // world pose 连续性。
  ASSERT_EQ( accepted.size(), 10U );
  const Eigen::Isometry3d& est   = accepted.back();
  const Eigen::Isometry3d& truth = motion.pose_at( 9 * 0.05 );
  EXPECT_NEAR( ( est.translation() - truth.translation() ).norm(), 0.0,
               5e-2 );
}

// ── IMU-off 零回归: 同一条合成链 enable_imu=false → 每帧走原 M3.3 语义
// (非 gap 帧 PnP 运行), 轨迹仍贴合真值。 ──
TEST( StereoVoImu, DisabledImuReproducesVisionChain )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion  = constantVelocityMotion( v );
  EstimatorOptions      options = imuOptions();
  options.enable_imu            = false;
  StereoVoEstimator estimator( makeCalibration(), options );
  // imu_init=false: IMU-off 无 init 相位 (kNone 不激活), 保持原时序
  // (帧 i → τ = i·dt, 无前缀)。
  const auto run = runImuSegment( estimator, makeCalibration(), motion, 0.05,
                                  12, kZero3, kZero3, 0.0, 0.0, false );
  expectAllOkAndClose( run, motion, 0.05, 2e-2, 1e-2, false );
  // IMU-off: 非 gap 帧走 PnP 初值。
  EXPECT_TRUE( run.results[ 3 ].diagnostics.pnp_success );
  for ( const auto& result : run.results )
  {
    EXPECT_FALSE( result.diagnostics.gyro_state.has_value() );
    EXPECT_FALSE( result.diagnostics.imu_init.has_value() );
  }
}

TEST( StereoVoImu, EmptyObservationsAreRejectedWithoutTranslationConstraint )
{
  const ImuMotion motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  (void)runStaticInitPrefix( estimator, makeCalibration(), 0.05 );

  const ScenarioInput input  = scenarioInput( motion, 0.05, 1, 6 );
  const auto          result = estimator.update(
      makeFrameWithImu( makeCalibration(), input.T_W_B, input.t_prev_ns,
                                 input.t_cur_ns, {}, {}, input.segment ),
      true );

  EXPECT_EQ( result.status, UpdateStatus::kRejected );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.message, "empty observations" );
}

TEST( StereoVoImu, ZeroOverlapNonKeyframeIsRejected )
{
  const ImuMotion motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  (void)runStaticInitPrefix( estimator, makeCalibration(), 0.05 );

  const ScenarioInput           input = scenarioInput( motion, 0.05, 1, 6 );
  const std::vector<LandmarkId> new_ids =
      sequentialIds( kLandmarks.size(), 1000 );
  const auto result = estimator.update(
      makeFrameWithImu( makeCalibration(), input.T_W_B, input.t_prev_ns,
                        input.t_cur_ns, kLandmarks, new_ids, input.segment ),
      false );

  EXPECT_EQ( result.status, UpdateStatus::kRejected );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.message, "zero shared landmarks (non-keyframe)" );
}

TEST( StereoVoImu, InsufficientSharedNonKeyframeIsRejected )
{
  const ImuMotion motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  (void)runStaticInitPrefix( estimator, makeCalibration(), 0.05 );

  const ScenarioInput     input = scenarioInput( motion, 0.05, 1, 6 );
  std::vector<LandmarkId> mostly_new_ids =
      sequentialIds( kLandmarks.size(), 1000 );
  mostly_new_ids.front() = 1;
  const auto result      = estimator.update(
      makeFrameWithImu( makeCalibration(), input.T_W_B, input.t_prev_ns,
                             input.t_cur_ns, kLandmarks, mostly_new_ids,
                             input.segment ),
      false );

  EXPECT_EQ( result.status, UpdateStatus::kRejected );
  EXPECT_FALSE( result.estimate.has_value() );
  EXPECT_EQ( result.message, "insufficient shared landmarks (non-keyframe)" );
}

TEST( StereoVoImu, VisualPnpRemainsAvailableWithGyroFactor )
{
  EstimatorOptions options        = imuOptions();
  options.imu_gyro_align_window_s = 0.2;
  const ImuMotion motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator estimator( makeCalibration(), options );
  (void)runStaticInitPrefix( estimator, makeCalibration(), 0.05 );

  phad::estimator::VioUpdateResult result;
  for ( int j = 1; j <= 5; ++j )
  {
    const ScenarioInput input = scenarioInput( motion, 0.05, j, 6 );
    result                    = estimator.update(
        makeFrameWithImu( makeCalibration(), input.T_W_B, input.t_prev_ns,
                                             input.t_cur_ns, kLandmarks,
                                             sequentialIds( kLandmarks.size() ), input.segment ),
        true );
    ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  }

  EXPECT_EQ( result.diagnostics.fusion_mode, FusionMode::kGyroVisual );
  EXPECT_GT( result.diagnostics.gyro_factor_count, 0U );
  EXPECT_TRUE( result.diagnostics.pnp_success );
  EXPECT_GE( result.diagnostics.pnp_inliers,
             static_cast<std::uint32_t>( imuOptions().min_pnp_inliers ) );
}

TEST( StereoVoImu, NormalFrameKeepsVisualInitialValueWithGyroPrediction )
{
  EstimatorOptions options        = imuOptions();
  options.imu_gyro_align_window_s = 0.2;
  options.enable_pnp_init         = false;
  StereoVoEstimator estimator( makeCalibration(), options );
  (void)runStaticInitPrefix( estimator, makeCalibration(), 0.05 );

  ScenarioInput input = scenarioInput( stationaryMotion(), 0.05, 1, 6 );
  for ( ImuMeasurement& sample : input.segment )
  {
    sample.gyro_radps[ 2 ] += 1.0;
  }
  const auto result = estimator.update(
      makeFrameWithImu( makeCalibration(), input.T_W_B, input.t_prev_ns,
                        input.t_cur_ns, kLandmarks,
                        sequentialIds( kLandmarks.size() ), input.segment ),
      true );

  ASSERT_EQ( result.status, UpdateStatus::kOk ) << result.message;
  ASSERT_TRUE( result.diagnostics.gyro_state.has_value() );
  const auto& state = *result.diagnostics.gyro_state;
  ASSERT_EQ( result.diagnostics.fusion_mode, FusionMode::kGyroVisual );
  ASSERT_TRUE( state.prediction_valid );
  EXPECT_GT( Eigen::AngleAxisd( state.predicted_T_W_B.linear() ).angle(),
             0.04 );
  EXPECT_NEAR(
      Eigen::AngleAxisd( state.graph_initial_T_W_B.linear() ).angle(), 0.0,
      1e-10 );
}

TEST( StereoVoImu, GyroFusionWaitsForStaticReadyAndVisualBiasAlignment )
{
  EstimatorOptions options = imuOptions();
  // Static prefix contributes 0.5 s visual/gyro evidence. Requiring 0.6 s
  // makes activation occur at the end of scenario frame 2; factors therefore
  // first appear on frame 3.
  options.imu_gyro_align_window_s = 0.6;
  const ImuMotion motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator estimator( makeCalibration(), options );
  (void)runStaticInitPrefix( estimator, makeCalibration(), 0.05 );

  for ( int j = 1; j <= 6; ++j )
  {
    ScenarioInput input = scenarioInput( motion, 0.05, j, 6 );
    for ( ImuMeasurement& sample : input.segment )
    {
      sample.gyro_radps[ 0 ] += j % 2 == 0 ? 2e-3 : -2e-3;
    }
    const auto result = estimator.update(
        makeFrameWithImu( makeCalibration(), input.T_W_B, input.t_prev_ns,
                          input.t_cur_ns, kLandmarks,
                          sequentialIds( kLandmarks.size() ), input.segment ),
        true );
    ASSERT_EQ( result.status, UpdateStatus::kOk )
        << "frame " << j << ": " << result.message;
    if ( j <= 2 )
    {
      EXPECT_EQ( result.diagnostics.fusion_mode, FusionMode::kVisionOnly );
      EXPECT_EQ( result.diagnostics.gyro_factor_count, 0U );
    }
    else
    {
      EXPECT_EQ( result.diagnostics.fusion_mode, FusionMode::kGyroVisual );
      EXPECT_GT( result.diagnostics.gyro_factor_count, 0U );
      EXPECT_GT(
          result.diagnostics.gyro_alignment_residual_rms_rad, 0.0 );
    }
  }
}

TEST( StereoVoImu, LongRunRemainsGyroOnlyAfterActivation )
{
  constexpr double       kDtS       = 0.05;
  constexpr int          kIntervals = 225;
  constexpr std::int64_t kDtNs      = 50'000'000;
  constexpr std::int64_t kOffsetNs =
      static_cast<std::int64_t>( kInitFrames ) * kDtNs;
  const Eigen::Vector3d bias_acc{ 0.08, -0.12, 0.05 };
  const Eigen::Vector3d bias_gyro{ 0.012, -0.018, 0.027 };
  EstimatorOptions      options   = imuOptions();
  options.imu_gyro_align_window_s = 0.6;
  options.enable_vio_state_probe  = true;
  StereoVoEstimator estimator( makeCalibration(), options );
  (void)runStaticInitPrefix( estimator, makeCalibration(), kDtS, bias_acc,
                             bias_gyro );

  const auto                                    states = alignmentStates( kIntervals, kDtS );
  const auto                                    ids    = sequentialIds( kLandmarks.size() );
  std::vector<phad::estimator::VioUpdateResult> results;
  results.reserve( static_cast<std::size_t>( kIntervals ) );
  ImuMeasurement previous =
      alignmentSample( states.front(), kOffsetNs, bias_acc, bias_gyro );
  for ( int index = 1; index <= kIntervals; ++index )
  {
    const std::int64_t t_cur_ns =
        kOffsetNs + static_cast<std::int64_t>( index ) * kDtNs;
    const ImuMeasurement current = alignmentSample(
        states[ static_cast<std::size_t>( index ) ], t_cur_ns, bias_acc,
        bias_gyro );
    results.push_back( estimator.update(
        makeFrameWithImu(
            makeCalibration(), states[ static_cast<std::size_t>( index ) ].T_W_B,
            t_cur_ns - kDtNs, t_cur_ns, kLandmarks, ids,
            { previous, current } ),
        true ) );
    ASSERT_EQ( results.back().status, UpdateStatus::kOk )
        << "frame " << index << ": " << results.back().message;
    previous = current;
  }

  std::size_t gyro_rows = 0U;
  for ( std::size_t index = 0U; index < results.size(); ++index )
  {
    const auto& result = results[ index ];
    if ( result.diagnostics.fusion_mode == FusionMode::kVisionOnly )
    {
      continue;
    }
    ASSERT_EQ( result.diagnostics.fusion_mode, FusionMode::kGyroVisual )
        << "frame " << index;
    EXPECT_GT( result.diagnostics.gyro_factor_count, 0U )
        << "frame " << index;
    ASSERT_TRUE( result.diagnostics.gyro_graph_cost.has_value() )
        << "frame " << index;
    EXPECT_EQ( result.diagnostics.gyro_graph_cost->gyro_factor_count,
               result.diagnostics.gyro_factor_count )
        << "frame " << index;
    ++gyro_rows;
  }
  EXPECT_GT( gyro_rows, 200U );
}

TEST( StereoVoImu, FixedLagShadowIsReadOnlyBoundedAndExplicit )
{
  EstimatorOptions control_options        = imuOptions();
  control_options.imu_gyro_align_window_s = 0.2;
  EstimatorOptions shadow_options         = control_options;
  shadow_options.enable_fixed_lag_shadow  = true;

  const ImuMotion motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator control( makeCalibration(), control_options );
  StereoVoEstimator shadow( makeCalibration(), shadow_options );
  const RunResult   control_run =
      runImuSegment( control, makeCalibration(), motion, 0.05, 40 );
  const RunResult shadow_run =
      runImuSegment( shadow, makeCalibration(), motion, 0.05, 40 );

  ASSERT_EQ( control_run.results.size(), shadow_run.results.size() );
  std::size_t reset_count      = 0U;
  std::size_t active_count     = 0U;
  bool        saw_fixed_bias   = false;
  bool        saw_marginalized = false;
  for ( std::size_t index = 0; index < shadow_run.results.size(); ++index )
  {
    const auto& control_result = control_run.results[ index ];
    const auto& shadow_result  = shadow_run.results[ index ];
    EXPECT_FALSE( control_result.diagnostics.fixed_lag_shadow.has_value() );
    ASSERT_EQ( shadow_result.status, control_result.status )
        << "frame " << index << ": " << shadow_result.message;
    ASSERT_EQ( shadow_result.estimate.has_value(),
               control_result.estimate.has_value() )
        << "frame " << index;
    if ( shadow_result.estimate.has_value() )
    {
      EXPECT_TRUE( shadow_result.estimate->T_W_B.matrix().isApprox(
          control_result.estimate->T_W_B.matrix(), 1e-12 ) )
          << "frame " << index;
    }
    EXPECT_TRUE( shadow_result.diagnostics.bias_gyro.isApprox(
        control_result.diagnostics.bias_gyro, 1e-12 ) )
        << "frame " << index;
    EXPECT_EQ( shadow_result.diagnostics.gyro_factor_count,
               control_result.diagnostics.gyro_factor_count )
        << "frame " << index;

    ASSERT_TRUE( shadow_result.diagnostics.fixed_lag_shadow.has_value() )
        << "frame " << index;
    const auto& diagnostics =
        *shadow_result.diagnostics.fixed_lag_shadow;
    EXPECT_TRUE( diagnostics.update_ok ) << "frame " << index;
    if ( shadow_result.diagnostics.fusion_mode == FusionMode::kVisionOnly )
    {
      EXPECT_FALSE( diagnostics.active ) << "frame " << index;
      continue;
    }

    ++active_count;
    EXPECT_TRUE( diagnostics.active ) << "frame " << index;
    EXPECT_EQ( diagnostics.smoother_pose_count,
               diagnostics.batch_window_size )
        << "frame " << index;
    EXPECT_LE( diagnostics.smoother_pose_count, 10U ) << "frame " << index;
    EXPECT_GT( diagnostics.smoother_landmark_count, 0U )
        << "frame " << index;
    EXPECT_TRUE( diagnostics.bias_present ) << "frame " << index;
    EXPECT_EQ( diagnostics.missing_owned_slot_count, 0U )
        << "frame " << index;
    EXPECT_EQ( diagnostics.timestamp_without_value_count, 0U )
        << "frame " << index;
    EXPECT_LE( diagnostics.cutoff_epoch, diagnostics.current_epoch )
        << "frame " << index;
    EXPECT_TRUE( diagnostics.newest_T_W_B.matrix().allFinite() )
        << "frame " << index;
    EXPECT_TRUE( std::isfinite( diagnostics.bias_delta_norm ) )
        << "frame " << index;
    EXPECT_TRUE( std::isfinite( diagnostics.newest_rotation_delta_rad ) )
        << "frame " << index;
    EXPECT_TRUE( std::isfinite( diagnostics.newest_translation_delta_m ) )
        << "frame " << index;
    if ( diagnostics.reset )
    {
      ++reset_count;
      EXPECT_EQ( diagnostics.reset_reason,
                 phad::estimator::FixedLagShadowReset::kBootstrap );
    }
    saw_fixed_bias = saw_fixed_bias || diagnostics.bias_fixed;
    saw_marginalized =
        saw_marginalized || diagnostics.marginalized_pose_count > 0U;
  }

  EXPECT_GT( active_count, 20U );
  EXPECT_EQ( reset_count, 1U );
  EXPECT_TRUE( saw_fixed_bias );
  EXPECT_TRUE( saw_marginalized );
}

TEST( StereoVoImu, FixedLagShadowRetiresAndRegeneratesLandmarkKeys )
{
  EstimatorOptions options                     = imuOptions();
  options.imu_gyro_align_window_s              = 0.2;
  options.enable_fixed_lag_shadow              = true;
  const RectifiedStereoCalibration calibration = makeCalibration();
  const ImuMotion                  motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator estimator( calibration, options );
  const RunResult   warmup =
      runImuSegment( estimator, calibration, motion, 0.05, 40 );
  ASSERT_EQ( warmup.results.back().status, UpdateStatus::kOk );
  ASSERT_EQ( warmup.results.back().diagnostics.fusion_mode,
             FusionMode::kGyroVisual );

  const std::vector<Eigen::Vector3d> persistent_landmarks(
      kLandmarks.begin() + 1, kLandmarks.end() );
  const std::vector<LandmarkId> persistent_ids =
      sequentialIds( persistent_landmarks.size(), 2 );
  std::uint32_t retired = 0U;
  for ( int frame = 40; frame < 49; ++frame )
  {
    const ScenarioInput input  = scenarioInput( motion, 0.05, frame, 6 );
    const auto          result = estimator.update(
        makeFrameWithImu( calibration, input.T_W_B, input.t_prev_ns,
                                   input.t_cur_ns, persistent_landmarks,
                                   persistent_ids, input.segment ),
        true );
    ASSERT_EQ( result.status, UpdateStatus::kOk )
        << "frame " << frame << ": " << result.message;
    ASSERT_TRUE( result.diagnostics.fixed_lag_shadow.has_value() );
    retired +=
        result.diagnostics.fixed_lag_shadow->retired_landmark_count;
  }
  EXPECT_GE( retired, 1U );

  const std::vector<LandmarkId> all_ids     = sequentialIds( kLandmarks.size() );
  std::uint32_t                 regenerated = 0U;
  for ( int frame = 49; frame < 51; ++frame )
  {
    const ScenarioInput input  = scenarioInput( motion, 0.05, frame, 6 );
    const auto          result = estimator.update(
        makeFrameWithImu( calibration, input.T_W_B, input.t_prev_ns,
                                   input.t_cur_ns, kLandmarks, all_ids,
                                   input.segment ),
        true );
    ASSERT_EQ( result.status, UpdateStatus::kOk )
        << "frame " << frame << ": " << result.message;
    ASSERT_TRUE( result.diagnostics.fixed_lag_shadow.has_value() );
    regenerated += result.diagnostics.fixed_lag_shadow
                       ->new_landmark_generation_count;
  }
  EXPECT_GE( regenerated, 1U );
}

TEST( StereoVoImu, FixedLagShadowResetsExplicitlyAtSegmentBoundary )
{
  EstimatorOptions options                     = imuOptions();
  options.imu_gyro_align_window_s              = 0.2;
  options.enable_fixed_lag_shadow              = true;
  const RectifiedStereoCalibration calibration = makeCalibration();
  const ImuMotion                  motion =
      constantRotationMotion( 0.2, Eigen::Vector3d{ 0.2, 0.0, 0.0 } );
  StereoVoEstimator estimator( calibration, options );
  const RunResult   warmup =
      runImuSegment( estimator, calibration, motion, 0.05, 40 );
  ASSERT_EQ( warmup.results.back().status, UpdateStatus::kOk );

  const std::vector<LandmarkId> new_ids =
      sequentialIds( kLandmarks.size(), 1'000 );
  const ScenarioInput reset_input  = scenarioInput( motion, 0.05, 40, 6 );
  const auto          reset_result = estimator.update(
      makeFrameWithImu( calibration, reset_input.T_W_B,
                                 reset_input.t_prev_ns, reset_input.t_cur_ns,
                                 kLandmarks, new_ids, reset_input.segment ),
      true );
  ASSERT_EQ( reset_result.status, UpdateStatus::kOk )
      << reset_result.message;
  ASSERT_TRUE( reset_result.diagnostics.fixed_lag_shadow.has_value() );
  const auto& reset = *reset_result.diagnostics.fixed_lag_shadow;
  EXPECT_TRUE( reset.active );
  EXPECT_TRUE( reset.reset );
  EXPECT_EQ( reset.reset_reason,
             phad::estimator::FixedLagShadowReset::kSegment );
  EXPECT_EQ( reset.smoother_pose_count, 1U );
  EXPECT_EQ( reset.smoother_landmark_count, 0U );

  const ScenarioInput next_input  = scenarioInput( motion, 0.05, 41, 6 );
  const auto          next_result = estimator.update(
      makeFrameWithImu( calibration, next_input.T_W_B,
                                 next_input.t_prev_ns, next_input.t_cur_ns,
                                 kLandmarks, new_ids, next_input.segment ),
      true );
  ASSERT_EQ( next_result.status, UpdateStatus::kOk ) << next_result.message;
  ASSERT_TRUE( next_result.diagnostics.fixed_lag_shadow.has_value() );
  const auto& next = *next_result.diagnostics.fixed_lag_shadow;
  EXPECT_FALSE( next.reset );
  EXPECT_EQ( next.reset_reason,
             phad::estimator::FixedLagShadowReset::kNone );
  EXPECT_GT( next.smoother_landmark_count, 0U );
}

TEST( StereoVoImu, FixedLagShadowRequiresImu )
{
  EstimatorOptions options;
  options.enable_imu              = false;
  options.enable_fixed_lag_shadow = true;
  EXPECT_THROW( StereoVoEstimator estimator( makeCalibration(), options ),
                std::invalid_argument );
}

// Gyro-only 不能在视觉 dropout 期约束 translation：注入帧必须拒绝且
// 不产生轨迹点，但 pending gyro interval 保留并在视觉恢复帧一次消费。
TEST( StereoVoImu, DropoutInjectionFreezesWithGyroVisual )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion    = constantVelocityMotion( v );
  EstimatorOptions      options   = imuOptions();
  options.imu_gyro_align_window_s = 0.2;
  StereoVoEstimator             estimator( makeCalibration(), options );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
  // M4.3: 静止 init 前缀 (播种帧 = accepted[0], τ=0), 场景帧 j ≥ 1。
  (void)runStaticInitPrefix( estimator, makeCalibration(), 0.05 );

  constexpr int kDropoutStart   = 5;  // 场景帧号 (播种帧之后)
  constexpr int kDropoutEnd     = 9;  // 含
  std::size_t   accepted_before = 0;
  std::size_t   accepted_after  = 0;
  for ( int j = 1; j <= 16; ++j )
  {
    const ScenarioInput in      = scenarioInput( motion, 0.05, j, 6 );
    const bool          dropped = j >= kDropoutStart && j <= kDropoutEnd;
    auto                result  = estimator.update( makeFrameWithImu(
                                        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns,
                                        dropped ? std::vector<Eigen::Vector3d>{} : kLandmarks,
                                        dropped ? std::vector<LandmarkId>{} : ids, in.segment ),
                                                    true );
    if ( dropped )
    {
      EXPECT_EQ( result.status, UpdateStatus::kRejected )
          << "frame " << j << ": " << result.message;
      EXPECT_FALSE( result.estimate.has_value() );
      EXPECT_EQ( result.message, "empty observations" );
      continue;
    }

    ASSERT_EQ( result.status, UpdateStatus::kOk )
        << "frame " << j << ": " << result.message;
    EXPECT_EQ( result.diagnostics.segment_id, 0U );
    ASSERT_TRUE( result.estimate.has_value() );
    if ( j < kDropoutStart )
    {
      ++accepted_before;
    }
    else
    {
      ++accepted_after;
      if ( j == kDropoutEnd + 1 )
      {
        ASSERT_TRUE( result.diagnostics.gyro_state.has_value() );
        EXPECT_TRUE( result.diagnostics.gyro_state->prediction_valid );
        EXPECT_NEAR( result.diagnostics.gyro_state->imu_dt_s, 0.30, 1e-12 );
      }
    }
    const Eigen::Isometry3d& truth =
        motion.pose_at( static_cast<double>( j ) * 0.05 );
    EXPECT_LE( ( result.estimate->T_W_B.translation() - truth.translation() ).norm(),
               2e-2 )
        << "frame " << j;
  }
  EXPECT_EQ( accepted_before, 4U );
  EXPECT_EQ( accepted_after, 7U );
}

// ── M4.3 dropout 注入 (合成断言 2): IMU-off 对照 —— 观测全清 → PnP 无点
// 可解 → 注入期拒帧冻结语义 (无新轨迹点: 不产生 accepted 位姿); 恢复后
// 重锚回到 kOk。与 M3.3 同源行为, 只断言「冻结 = 无新轨迹点」。 ──
TEST( StereoVoImu, DropoutInjectionFreezesWithoutImu )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion  = constantVelocityMotion( v );
  EstimatorOptions      options = imuOptions();
  options.enable_imu            = false;
  StereoVoEstimator             estimator( makeCalibration(), options );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );

  constexpr int kDropoutStart   = 5;
  constexpr int kDropoutEnd     = 9;  // 含
  std::size_t   accepted_before = 0;
  std::size_t   accepted_after  = 0;
  for ( int j = 0; j <= 14; ++j )
  {
    const std::int64_t t_cur_ns =
        static_cast<std::int64_t>( static_cast<double>( j ) * 0.05 * 1e9 );
    const std::int64_t t_prev_ns = j == 0
                                       ? 0
                                       : static_cast<std::int64_t>(
                                             ( static_cast<double>( j ) -
                                               1.0 ) *
                                             0.05 * 1e9 );
    const bool         dropped   = j >= kDropoutStart && j <= kDropoutEnd;
    auto               result    = estimator.update( makeFrameWithImu(
                                        makeCalibration(), motion.pose_at( static_cast<double>( j ) * 0.05 ),
                                        t_prev_ns, t_cur_ns,
                                        dropped ? std::vector<Eigen::Vector3d>{} : kLandmarks,
                                        dropped ? std::vector<LandmarkId>{} : ids, {} ),
                                                     true );
    if ( dropped )
    {
      // 冻结语义: 注入期全部拒帧, 无新轨迹点。
      EXPECT_EQ( result.status, UpdateStatus::kRejected )
          << "frame " << j << ": " << result.message;
      EXPECT_FALSE( result.estimate.has_value() );
    }
    else if ( j < kDropoutStart )
    {
      ASSERT_EQ( result.status, UpdateStatus::kOk )
          << "frame " << j << ": " << result.message;
      ++accepted_before;
    }
    else
    {
      ASSERT_EQ( result.status, UpdateStatus::kOk )
          << "frame " << j << ": " << result.message;
      ++accepted_after;
    }
  }
  // 注入期 5 帧全冻结: 14 帧中只接受 10 帧 (0..4 与 10..14)。
  EXPECT_EQ( accepted_before, 5U );
  EXPECT_EQ( accepted_after, 5U );
}
