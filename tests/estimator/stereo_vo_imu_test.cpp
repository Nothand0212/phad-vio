// M4.2 合成对拍测试 (§4.7): 视觉观测与 IMU 测量由同一地面真值运动生成,
// 断言估计轨迹贴合真值 —— 验证预积分机制/ΣΔt 语义/初值链/伪初始化/
// pending 拼接/imu_gap 退化, 以及 IMU-on 与 IMU-off 的行为分界。
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
  using phad::estimator::KeyframeMeasurement;
  using phad::estimator::LandmarkId;
  using phad::estimator::StereoObservation;
  using phad::estimator::StereoVoEstimator;
  using phad::estimator::UpdateStatus;
  using phad::sensor::ImuMeasurement;
  using phad::sensor::RigidTransform;

  // Z-up 世界系, 伪初始化重力 (C4/C5): g_w = [0, 0, -g]。
  constexpr double kGravity = 9.81007;
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
      const double t_s  = t_prev_s + frac * ( t_cur_s - t_prev_s );
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
      const std::vector<LandmarkId>& ids,
      const std::vector<ImuMeasurement>& segment, bool imu_gap = false )
  {
    KeyframeMeasurement measurement;
    measurement.timestamp = phad::common::Timestamp{ t_cur_ns };
    measurement.t_prev    = phad::common::Timestamp{ t_prev_ns };
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
                                                       LandmarkId start = 1 )
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
      { 0.4, 0.1, 5.0 },  { -0.3, 0.2, 4.5 }, { 0.1, -0.25, 6.0 },
      { 0.6, -0.1, 5.5 }, { -0.5, -0.2, 4.8 }, { 0.0, 0.3, 5.2 },
      { 0.25, 0.15, 4.2 }, { -0.2, -0.15, 5.8 }, { 0.35, -0.05, 5.3 },
      { -0.15, 0.25, 4.6 },
  };

  struct RunResult
  {
    std::vector<Eigen::Isometry3d>            accepted;
    std::vector<phad::estimator::VioUpdateResult> results;
  };

  // ── M4.3 静止 init 前缀语义 ──
  // 静止运动定义见下方运动学场景节。
  const ImuMotion& stationaryMotion();
  // 帧 0..kInitFrames−1 静止 (init 期全部被拒, init_pending);帧
  // kInitFrames 为播种帧 (静止, 被接受, 即 accepted[0], τ=0);场景帧
  // j ≥ 1 的 τ = j·dt —— 播种帧之后运动才启动, 检测窗口 [t0, t0+0.5 s]
  // 恒为静止样本 (plan 决策 A/C: init 样本 = 被拒帧段累积, t0 = 缓冲
  // 首样本时刻)。
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
      const Eigen::Vector3d& bias_acc = kZero3,
      const Eigen::Vector3d& bias_gyro = kZero3, std::mt19937* rng = nullptr,
      double noise_acc = 0.0, double noise_gyr = 0.0 )
  {
    ScenarioInput input;
    input.t_cur_ns  = static_cast<std::int64_t>(
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

  // 静止 init 前缀 (手写循环测试用): 帧 0..kInitFrames−1 被拒
  // (init_pending), 帧 kInitFrames 播种 (静止, 接受)。返回播种帧位姿
  // (accepted[0], τ=0)。
  [[nodiscard]] Eigen::Isometry3d runStaticInitPrefix(
      StereoVoEstimator& estimator, const RectifiedStereoCalibration& calibration,
      double dt_img_s )
  {
    const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
    for ( int i = 0; i <= kInitFrames; ++i )
    {
      const double t_cur_s = static_cast<double>( i ) * dt_img_s;
      const std::int64_t t_cur_ns =
          static_cast<std::int64_t>( t_cur_s * 1e9 );
      const std::int64_t t_prev_ns = i == 0
                                         ? 0
                                         : static_cast<std::int64_t>(
                                               ( static_cast<double>( i ) -
                                                 1.0 ) *
                                               dt_img_s * 1e9 );
      std::vector<ImuMeasurement> segment;
      if ( i > 0 )
      {
        segment = makeImuSegment( stationaryMotion(), t_cur_s - dt_img_s,
                                  t_cur_s, 6 );
      }
      auto result = estimator.update( makeFrameWithImu(
          calibration, stationaryMotion().pose_at( t_cur_s ), t_prev_ns,
          t_cur_ns, kLandmarks, ids, segment ), true );
      if ( i < kInitFrames )
      {
        EXPECT_EQ( result.status, UpdateStatus::kRejected )
            << "init frame " << i << ": " << result.message;
        EXPECT_TRUE( result.diagnostics.init_pending ) << "init frame " << i;
      }
      else
      {
        EXPECT_EQ( result.status, UpdateStatus::kOk )
            << "seed frame: " << result.message;
        EXPECT_TRUE( result.estimate.has_value() );
        return result.estimate->T_W_B;
      }
    }
    ADD_FAILURE() << "unreachable: seed frame not reached";
    return Eigen::Isometry3d::Identity();
  }

  // 帧时间: 首帧 t=0, 之后每帧 dt_img_s。IMU 段 6 样本/帧 (10 ms 步长, 含两端)。
  // 首帧 t_prev=0 (seed 路径不使用段)。imu_init=true (M4.3): 先跑静止
  // init 前缀 (kInitFrames 被拒帧 + 播种帧 = accepted[0], τ=0), 场景帧
  // j ≥ 1 的 τ = j·dt; imu_init=false (IMU-off): 保持原时序 (帧 i →
  // τ = i·dt, 无前缀)。
  [[nodiscard]] RunResult runImuSegment(
      StereoVoEstimator& estimator, const RectifiedStereoCalibration& calibration,
      const ImuMotion& motion, double dt_img_s, int frames,
      const Eigen::Vector3d& bias_acc = kZero3,
      const Eigen::Vector3d& bias_gyro = kZero3, double noise_acc = 0.0,
      double noise_gyr = 0.0, bool imu_init = true )
  {
    const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
    RunResult                     run;
    std::mt19937 rng( 42 );  // 固定种子, 确定性噪声
    if ( imu_init )
    {
      for ( int i = 0; i <= kInitFrames; ++i )
      {
        const double t_cur_s = static_cast<double>( i ) * dt_img_s;
        const std::int64_t t_cur_ns =
            static_cast<std::int64_t>( t_cur_s * 1e9 );
        const std::int64_t t_prev_ns = i == 0
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
            t_cur_ns, kLandmarks, ids, segment ), true );
        run.results.push_back( result );
        if ( result.status == UpdateStatus::kOk &&
             result.estimate.has_value() )
        {
          run.accepted.push_back( result.estimate->T_W_B );
        }
      }
    }
    for ( int j = imu_init ? 1 : 0; j < frames; ++j )
    {
      const double             tau = static_cast<double>( j ) * dt_img_s;
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
      const Eigen::Isometry3d T_W_B = motion.pose_at( tau );
      auto result = estimator.update( makeFrameWithImu(
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

  // 所有接受帧必须 ok 且逐帧贴合真值 (容差逐测试给定)。前缀 init 帧
  // (若有, 如 M4.3 的 kInitFrames 帧) 全部被拒且 init_pending; accepted[j]
  // 对应 τ = j·dt (播种帧 = accepted[0])。
  void expectAllOkAndClose( const RunResult& run, const ImuMotion& motion,
                            double dt_img_s, double tol_trans_m,
                            double tol_rot_rad = 1e-2 )
  {
    const std::size_t rejected = run.results.size() - run.accepted.size();
    for ( std::size_t index = 0; index < rejected; ++index )
    {
      EXPECT_EQ( run.results[ index ].status, UpdateStatus::kRejected )
          << "init frame " << index << ": " << run.results[ index ].message;
      EXPECT_TRUE( run.results[ index ].diagnostics.init_pending )
          << "init frame " << index;
    }
    ASSERT_EQ( run.accepted.size(), run.results.size() - rejected );
    for ( std::size_t index = 0; index < run.accepted.size(); ++index )
    {
      const auto& result = run.results[ rejected + index ];
      EXPECT_EQ( result.status, UpdateStatus::kOk ) << "frame " << index
                                                    << ": " << result.message;
      const double t_s = static_cast<double>( index ) * dt_img_s;
      const Eigen::Isometry3d& est    = run.accepted[ index ];
      const Eigen::Isometry3d& truth  = motion.pose_at( t_s );
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
  const auto run = runImuSegment( estimator, makeCalibration(),
                                  stationaryMotion(), 0.05, 10 );
  expectAllOkAndClose( run, stationaryMotion(), 0.05, 2e-2 );
  // D8: IMU-on 非 gap 帧初值 = preint.Predict, PnP 不运行。
  for ( const auto& result : run.results )
  {
    EXPECT_FALSE( result.diagnostics.pnp_success );
    EXPECT_EQ( result.diagnostics.pnp_inliers, 0U );
  }
}

// ── §4.7-b 匀速: 视觉位移 = IMU 积分位移。 ──
TEST( StereoVoImu, ConstantVelocityMatchesGroundTruth )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  const auto run = runImuSegment( estimator, makeCalibration(), motion, 0.05,
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
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  const auto run = runImuSegment( estimator, makeCalibration(), motion, 0.05,
                                  12 );
  expectAllOkAndClose( run, motion, 0.05, 3e-2, 2e-2 );
}

// ── §4.7-d 已知 bias: 静止 + 恒定偏置 → 偏置被吸收, 位姿不漂移。 ──
TEST( StereoVoImu, KnownBiasIsAbsorbed )
{
  const Eigen::Vector3d bias_acc{ 0.3, -0.2, 0.1 };
  const Eigen::Vector3d bias_gyro{ 0.02, -0.01, 0.015 };
  const ImuMotion       motion = stationaryMotion();
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
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

// ── §4.7-e covariance: 带高斯噪声 (密度量级) 的 IMU 段轨迹仍准确 ——
// 协方差 = 密度平方的换算若错误, LM 权重失当 → 漂移/发散。 ──
TEST( StereoVoImu, NoisyImuStaysAccurate )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
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
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
  // M4.3: 静止 init 前缀 (播种帧 = accepted[0], τ=0), 场景帧 j ≥ 1 两样本段。
  std::vector<Eigen::Isometry3d> accepted{
    runStaticInitPrefix( estimator, makeCalibration(), 0.05 )
  };
  for ( int j = 1; j < 12; ++j )
  {
    const ScenarioInput in = scenarioInput( motion, 0.05, j, 2 );
    auto result = estimator.update( makeFrameWithImu(
        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns, kLandmarks,
        ids, in.segment ), true );
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
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
  // M4.3: 静止 init 前缀 (播种帧 = accepted[0]), 场景帧 j ≥ 1; 场景
  // j=5 段缺失 → imu_gap (初始化已完成, gap 不再重置 init)。
  std::vector<phad::estimator::VioUpdateResult> results;
  std::vector<Eigen::Isometry3d> accepted{
    runStaticInitPrefix( estimator, makeCalibration(), 0.05 )
  };
  for ( int j = 1; j < 12; ++j )
  {
    const ScenarioInput in = scenarioInput( motion, 0.05, j, 6 );
    std::vector<ImuMeasurement> segment =
        j == 5 ? std::vector<ImuMeasurement>{} : in.segment;
    auto result = estimator.update( makeFrameWithImu(
        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns, kLandmarks,
        ids, segment, j == 5 ), true );
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
  for ( std::size_t j = 1; j < 12; ++j )
  {
    const std::size_t index = j - 1;
    EXPECT_EQ( results[ index ].status, UpdateStatus::kOk )
        << "frame " << j << ": " << results[ index ].message;
    const double d_trans =
        ( accepted[ j ].translation() - motion.pose_at(
             static_cast<double>( j ) * 0.05 ).translation() )
            .norm();
    EXPECT_NEAR( d_trans, 0.0, 1e-1 ) << "frame " << j;
  }
}

// ── §4.7-h pending 拼接: 被拒帧的段保留, 成功帧拼接后 ΣΔt 跨越两个图像
// 间隔, 位姿从最后接受位姿外推仍贴合真值。M4.3c (plan G) 起空观测非 gap
// 帧已放行, 故改用畸形观测触发拒绝 (段同样已入 pending)。 ──
TEST( StereoVoImu, PendingAppendsOnRejectAndConsumesOnAccept )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
  // M4.3: 静止 init 前缀 (播种帧 = accepted[0]), 场景帧 j ≥ 1; 场景
  // j=5 被拒 (畸形观测: 负 disparity → rejected), 其段 (已入 pending) 保留。
  std::vector<Eigen::Isometry3d> accepted{
    runStaticInitPrefix( estimator, makeCalibration(), 0.05 )
  };
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
      StereoObservation bad;
      bad.left_pixel   = Eigen::Vector2d{ 0.0, 0.0 };
      bad.disparity_px = -1.0;  // 触发 "negative disparity" 拒绝
      rejected.observations.push_back( bad );
      const auto result = estimator.update( rejected, true );
      EXPECT_EQ( result.status, UpdateStatus::kRejected );
      continue;
    }
    auto result = estimator.update( makeFrameWithImu(
        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns, kLandmarks,
        ids, in.segment ), true );
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

// ── §4.7-i D12: overlap 断裂 (全新 id) 时 IMU-on 不清位姿链 —— 帧仍 ok,
// segment_id 不增加, 新路标进窗口, 后续帧继续。 ──
TEST( StereoVoImu, OverlapBreakKeepsChainWithImu )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  const std::vector<LandmarkId> ids_a = sequentialIds( kLandmarks.size() );
  // 全新 id 集合 + 稍远的路标 (覆盖平移后仍在视锥内)。
  const std::vector<Eigen::Vector3d> landmarks_b{
      { 1.4, -0.3, 5.4 }, { 0.9, 0.35, 4.9 }, { 1.1, -0.15, 6.2 },
      { 1.6, 0.05, 5.1 }, { 0.65, -0.4, 4.7 }, { 1.0, 0.2, 5.6 },
      { 1.25, -0.2, 4.4 }, { 0.8, 0.3, 5.9 }, { 1.35, -0.05, 5.0 },
      { 0.85, 0.15, 4.8 },
  };
  const std::vector<LandmarkId> ids_b = sequentialIds( kLandmarks.size(), 1000 );

  // M4.3: 静止 init 前缀 (播种帧 = accepted[0]), 场景帧 j ≥ 1; 场景
  // j=5 换成全新 id (overlap 断裂, D12)。
  std::vector<Eigen::Isometry3d> accepted{
    runStaticInitPrefix( estimator, makeCalibration(), 0.05 )
  };
  for ( int j = 1; j < 10; ++j )
  {
    const ScenarioInput in = scenarioInput( motion, 0.05, j, 6 );
    const bool          broken = j == 5;
    const auto          landmarks = broken ? landmarks_b : kLandmarks;
    const auto          ids       = broken ? ids_b : ids_a;
    auto result = estimator.update( makeFrameWithImu(
        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns, landmarks,
        ids, in.segment ), true );
    ASSERT_EQ( result.status, UpdateStatus::kOk )
        << "frame " << j << ": " << result.message;
    EXPECT_EQ( result.diagnostics.segment_id, 0U )
        << "IMU-on 不再 re-anchor (D12), segment 恒为 0";
    if ( result.estimate.has_value() )
    {
      accepted.push_back( result.estimate->T_W_B );
    }
  }
  // 播种帧 + 9 场景帧 (末帧 τ = 9 * 0.05)。
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
  const ImuMotion       motion = constantVelocityMotion( v );
  EstimatorOptions      options = imuOptions();
  options.enable_imu            = false;
  StereoVoEstimator estimator( makeCalibration(), options );
  // imu_init=false: IMU-off 无 init 相位 (kNone 不激活), 保持原时序
  // (帧 i → τ = i·dt, 无前缀)。
  const auto run = runImuSegment( estimator, makeCalibration(), motion, 0.05,
                                  12, kZero3, kZero3, 0.0, 0.0, false );
  expectAllOkAndClose( run, motion, 0.05, 2e-2 );
  // IMU-off: 非 gap 帧走 PnP 初值。
  EXPECT_TRUE( run.results[ 3 ].diagnostics.pnp_success );
}

// ── M4.3 dropout 注入 (plan 定案 G, 合成断言 1): IMU-on 观测全清 →
// 图 = 仅 IMU 因子 + priors 有解 → 注入期每帧 accepted, 轨迹连续 (预积分
// 外推, 相对真值偏差 ≤ 阈值);恢复后 50 帧内 ATE 回基线 (无永久损伤)。
// 等价于 session 侧 --dropout-keep-ratio 0 对观测子采样后的估计器行为。 ──
TEST( StereoVoImu, DropoutInjectionKeepsChainWithImu )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  StereoVoEstimator estimator( makeCalibration(), imuOptions() );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );
  // M4.3: 静止 init 前缀 (播种帧 = accepted[0], τ=0), 场景帧 j ≥ 1。
  ( void )runStaticInitPrefix( estimator, makeCalibration(), 0.05 );

  constexpr int kDropoutStart = 5;  // 场景帧号 (播种帧之后)
  constexpr int kDropoutEnd   = 9;  // 含
  for ( int j = 1; j <= 16; ++j )
  {
    const ScenarioInput in = scenarioInput( motion, 0.05, j, 6 );
    const bool          dropped = j >= kDropoutStart && j <= kDropoutEnd;
    auto result = estimator.update( makeFrameWithImu(
        makeCalibration(), in.T_W_B, in.t_prev_ns, in.t_cur_ns,
        dropped ? std::vector<Eigen::Vector3d>{} : kLandmarks,
        dropped ? std::vector<LandmarkId>{} : ids, in.segment ), true );
    ASSERT_EQ( result.status, UpdateStatus::kOk )
        << "frame " << j << ": " << result.message;
    EXPECT_EQ( result.diagnostics.segment_id, 0U )
        << "IMU-on 注入期与恢复期均不 re-anchor (D12)";
    ASSERT_TRUE( result.estimate.has_value() );
    const Eigen::Isometry3d& est   = result.estimate->T_W_B;
    const Eigen::Isometry3d& truth = motion.pose_at(
        static_cast<double>( j ) * 0.05 );
    const double err_m = ( est.translation() - truth.translation() ).norm();
    if ( dropped )
    {
      // 注入期: 仅 IMU 因子 + priors, 位姿由预积分外推 → 连续但不逐帧
      // 视觉锚定, 允许小漂移 (0.25 s 窗口)。
      EXPECT_LE( err_m, 5e-2 ) << "frame " << j;
    }
    else
    {
      // 恢复后: 视觉回到图里, ATE 回基线 (50 帧内; 合成 16 帧内即达)。
      EXPECT_LE( err_m, 2e-2 ) << "frame " << j;
    }
  }
}

// ── M4.3 dropout 注入 (合成断言 2): IMU-off 对照 —— 观测全清 → PnP 无点
// 可解 → 注入期拒帧冻结语义 (无新轨迹点: 不产生 accepted 位姿); 恢复后
// 重锚回到 kOk。与 M3.3 同源行为, 只断言「冻结 = 无新轨迹点」。 ──
TEST( StereoVoImu, DropoutInjectionFreezesWithoutImu )
{
  const Eigen::Vector3d v{ 0.3, 0.0, 0.0 };
  const ImuMotion       motion = constantVelocityMotion( v );
  EstimatorOptions      options = imuOptions();
  options.enable_imu            = false;
  StereoVoEstimator estimator( makeCalibration(), options );
  const std::vector<LandmarkId> ids = sequentialIds( kLandmarks.size() );

  constexpr int kDropoutStart = 5;
  constexpr int kDropoutEnd   = 9;  // 含
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
    const bool dropped = j >= kDropoutStart && j <= kDropoutEnd;
    auto result = estimator.update( makeFrameWithImu(
        makeCalibration(), motion.pose_at( static_cast<double>( j ) * 0.05 ),
        t_prev_ns, t_cur_ns,
        dropped ? std::vector<Eigen::Vector3d>{} : kLandmarks,
        dropped ? std::vector<LandmarkId>{} : ids, {} ), true );
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
