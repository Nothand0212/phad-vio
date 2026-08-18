# M4 VIO 在线陀螺仪 bias state 调研

日期：2026-08-18
状态：Research complete；只为重写下一份协议提供依据，不授予实现或真实序列资格化权限

## 结论先行

Q3 的一次冻结运行只证明：`visual_posterior_aligned_nuisance` 的 early/late 半窗差异超过既定阈值，因此按原协议必须 `STOP`；它没有证明所有离线常量 bias 方法在数学上都不可能，也没有把该 nuisance estimate 认证为物理陀螺仪 bias。[Q3 result](m4-minimal-gyro-q3-offline-bias-alignment-result.md) 与 [Q3 design](m4-minimal-gyro-q3-offline-bias-alignment-design.md) 已明确禁止把这次失败解释为 Q4 实现授权。

成熟实现的共同点不是“先从完整数据拟合一个常量，再冻结到 VIO”，而是：

1. bias 是在线状态；
2. IMU/旋转预积分在一个线性化 bias 处建立，并保留对 bias 的一阶敏感度；
3. bias 有初值先验和随时间演化的 process model；
4. 优化后的 bias 会用于下一段预积分，或用于重传播/一阶修正。

这一共同模式可直接从 [GTSAM `ImuFactor`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/ImuFactor.h#L158-L206)、[GTSAM `CombinedImuFactor`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/CombinedImuFactor.h#L191-L247)、[Kimera-VIO backend](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/backend/VioBackend.cpp#L899-L958)、[ORB-SLAM3 optimizer](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L2528-L2660)、[VINS-Fusion estimator](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L1015-L1061) 和 [OpenVINS IMU state](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/types/IMU.h#L30-L90) 交叉确认。

因此，下一最小 gyro→VIO vertical slice 的推荐候选是 **A：显式三维 gyro bias state + random-walk process factor**。对当前仅有 `Pose3` 的后端，最小且长期可演进的内部形态是：

- 每条合格的相邻图状态 IMU 区间连接 `X_i, X_j, G_i` 的三维旋转 residual，其中 `G(k)` 是本项目选择的 gyro-only bias key；
- 相邻 bias 以零增量 `BetweenFactor<Vector3>(G_i, G_j, 0, noise)` 连接；
- 首个 `G_0` 有独立初值与 prior；window 逐出该节点后，新的 oldest-bias prior 如何继承信息必须另行冻结；
- 使用 `PreintegratedAhrsMeasurements` 保存 gyro covariance、bias Jacobian 与 bias-corrected prediction；
- 因官方 `AHRSFactor` 的端点类型是 `Rot3` 而当前图节点是 `Pose3`，应在 `StereoVoEstimator::Impl` 内做一个 Pose3 端点的薄 factor，复用其 residual 语义，而不为复用类本身额外复制一套 `Rot3` 状态。

上述 GTSAM 类型合同分别见 [`PreintegratedAhrsMeasurements`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.h#L32-L106)、[`AHRSFactor`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.h#L108-L190) 及其[残差实现](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.cpp#L92-L168)。“Pose3 端点薄 factor”是本项目工程推断，不是 GTSAM 官方直接提供的类型。

**范围判断（工程）：** 这一步仍是 **gyro-aided stereo VO**，不是含速度、重力、加速度与 accelerometer bias 的完整 VIO。它是完整 VIO 的下一层基础；后续再单独资格化 `V_i`、重力、三维 accelerometer bias，以及 `ImuFactor + bias BetweenFactor` 或 `CombinedImuFactor`。

## 1. 本地证据与当前边界

### 1.1 Q3 的确切含义

**事实：** Q3 的 support、fit 和 effect gates 通过，但 half-stability gate 失败；最大 early/late 轴差为 `0.001227119335357879 rad/s`，高于冻结阈值 `0.001 rad/s`，最终结论为 `HYPOTHESIS_FAIL`、权限为 `STOP`。[Q3 result](m4-minimal-gyro-q3-offline-bias-alignment-result.md)

**事实：** Q3 的被估参数被协议定义成 `visual_posterior_aligned_nuisance`，不是 IMU 静态标定得到的 physical sensor bias；协议还明确排除了 online initializer、factor/posterior 接入、阈值重调和失败后继续 Q4。[Q3 design](m4-minimal-gyro-q3-offline-bias-alignment-design.md)

**工程推断：** 现有“Q3 PASS 后把单个 offline bias 冻结进 Q4/Q5”的路线已被该协议关闭。正确动作不是围绕旧实现继续调阈值或换窗口，而是新建一份以在线 bias state 为机制对象的协议，继承 Q1/Q2 已合格的同步与 SO(3) 积分合同，同时把 Q3 当作“全局常量 nuisance 假设未获资格”的反证。

### 1.2 当前 estimator 与 helper 能复用什么

**事实：** `StereoVoEstimator` 的公开入口仍只有 `update(KeyframeMeasurement, bool keyframe)`，GTSAM 类型被 PIMPL 隔离；当前 `Impl` 构造 `Pose3` 与 landmark 状态、prior 和 stereo factors，没有 velocity 或 bias state。[public header](../../phad/estimator/stereo_vo_estimator.hpp)、[implementation](../../phad/estimator/stereo_vo_estimator.cpp)、[estimator contract](../../phad/estimator/README.md)

**工程推断：** 在线 bias 与 gyro factor 应留在 `StereoVoEstimator::Impl`，继续保持 `phad_estimator` 的 GTSAM `PRIVATE` 边界；公开层只暴露项目自己的测量/诊断 POD。没有理由为本 slice 让 GTSAM 类型泄漏到 public header。

**事实：** 当前 graph 不是跨 update 持久维护的增量图：每轮在局部 `NonlinearFactorGraph`/`Values` 中调用 `buildGraph`，该函数先清空容器，再从 `window` 的持久 `T_W_B` 和 `landmarks_W` 重建 `X(k)+L(id)` 初值；优化成功后把各 `X(k)` 写回 `WindowFrame::T_W_B`。[graph rebuild and writeback](../../phad/estimator/stereo_vo_estimator.cpp)

**本项目选择：** 沿用同一生命周期，在 `WindowFrame` 中持久保存 gyro bias，重建时将其插入 `G(k): Vector3`，优化成功且 finite/type/existence 检查通过后再回写；rollback snapshot、window eviction 与 outlier re-optimization 也必须和 pose 一样覆盖 bias。成熟实现支持“优化后 bias 必须进入下一段/下一轮”的原则——例如 Kimera 的 backend→frontend bias handoff 与 ORB-SLAM3 的 keyframe bias writeback——但 `G(k)` 命名、存入 `WindowFrame`、每轮 batch rebuild 是针对本项目当前架构的选择，不是这些上游项目的原样实现。[Kimera handoff](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/include/kimera-vio/imu-frontend/ImuFrontend.h#L115-L148) [ORB-SLAM3 writeback](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L2918-L2934)

**事实：** Q2 helper 对相邻样本做 endpoint average，检查样本数、有限值和严格递增时间戳，再调用 GTSAM `PreintegratedRotation::integrateGyroMeasurement(mean, known_bias, dt)`；它只返回 `delta_R_i_j` 与 duration，不返回 covariance、bias Jacobian 或 factor。[Q2 helper implementation](../../phad/estimator/gyro_rotation_predictor.cpp) 与 [header](../../phad/estimator/gyro_rotation_predictor.hpp)

**工程推断：** 应复用 Q2 的时间区间、端点梯形、frame convention、SO(3) 符号和异常合同作为 oracle/test contract；不应把其“传入已知常量 bias、只返回结果”的 API 直接当在线优化内核。在线内核需要保留 covariance 与 bias sensitivity，`PreintegratedAhrsMeasurements` 已提供这两项。[GTSAM declaration](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.h#L32-L106)

**事实：** 本地 `ImuParameters` 已分别提供 `gyrNd()` 与 `gyrRw()`；注释将其单位区分为 gyro measurement noise density `rad/s/√Hz` 和 gyro bias random-walk density `rad/s²/√Hz`。[local parameter contract](../../phad/sensor/imu_parameters.hpp)

**工程推断：** 下一协议应复用这两个既有参数，但语义不能互换：`gyr_nd² I` 进入 gyro preintegration 的 continuous-time measurement covariance；`gyr_rw` 决定 bias increment process noise。

## 2. GTSAM 官方语义

### 2.1 bias state 不是固定校正常量

**事实：** `imuBias::ConstantBias` 是一个六维值对象，前三维为 accelerometer bias、后三维为 gyroscope bias；`correctGyroscope` 从测量中减去 gyro bias。[`ImuBias.h`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/ImuBias.h#L32-L90)

**事实：** `PreintegrationBase` 保存 `biasHat`，允许以新 bias 重置积分，并在 `biasCorrectedDelta`/`predict` 中接受当前 bias 做一阶修正。[`PreintegrationBase.h`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegrationBase.h#L35-L83) [bias-corrected API](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegrationBase.h#L140-L173)

**事实：** 五元 `ImuFactor(X_i,V_i,X_j,V_j,B_i)` 本身不约束跨时刻 bias 一致性，官方注释要求调用者另行建模；六元 `CombinedImuFactor(X_i,V_i,X_j,V_j,B_i,B_j)` 则在内部加入 slowly-varying/random-walk bias，并保留 bias 与预积分测量的不确定性相关。[`ImuFactor`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/ImuFactor.h#L158-L206) [`CombinedImuFactor`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/CombinedImuFactor.h#L191-L247)

**事实：** GTSAM 官方 `ImuFactorsExample` 对 classic `ImuFactor` 显式加入零测量的 `BetweenFactor<ConstantBias>`，优化后读取当前 bias，并用该结果重置下一段预积分。[official example](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/examples/ImuFactorsExample.cpp#L241-L288)

### 2.2 random walk 与 noise 参数

令连续时间 gyro bias random-walk density 为 `σ_bg = gyr_rw`，相邻 bias 节点时间跨度为 `Δt`。常用离散模型为：

```text
b_g,j = b_g,i + w_bg
w_bg ~ N(0, σ_bg² Δt I)
```

于是传给 `noiseModel::Diagonal::Sigmas` 的是标准差：

```text
sigma_between = gyr_rw * sqrt(Δt)
```

**事实：** `BetweenFactor` 的误差是“预测的 `between(p1,p2)` 与测量之间的局部坐标”；对 `Vector3` 和零测量，它就是相邻 bias 的增量残差。[`BetweenFactor.h`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/slam/BetweenFactor.h#L110-L123)

**事实：** `Diagonal::Sigmas` 接收的是标准差而不是方差。[`NoiseModel.h`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/linear/NoiseModel.h#L301-L337)

**事实：** Kimera-VIO 的 regular IMU 路径正是用 `sqrt(pim.deltaTij()) * gyro_random_walk` 构造 bias `BetweenFactor`；combined 路径则把 bias random-walk covariance 交给 `PreintegratedCombinedMeasurements`。[Kimera-VIO backend](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/backend/VioBackend.cpp#L899-L958) [Kimera IMU frontend](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/imu-frontend/ImuFrontend.cpp#L107-L120)

**事实：** `PreintegratedCombinedParams` 将 `biasOmegaCovariance` 定义为描述 gyro bias random walk 的 continuous-time covariance，并另设初始 bias covariance。[`PreintegrationCombinedParams.h`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegrationCombinedParams.h#L33-L84)

**工程推断：** classic bias `BetweenFactor` 使用 `gyr_rw * sqrt(Δt)` 的 sigma；combined preintegration 使用 `gyr_rw² I` 的 continuous-time covariance。把 variance 传给 `Sigmas`、把 `gyr_nd` 当 bias random walk、或忽略不同长度区间的 `sqrt(Δt)`，都会改变模型量纲，必须成为协议级失败条件。

### 2.3 gyro-only 可直接借鉴的官方形态

**事实：** `PreintegratedAhrsMeasurements` 累积 gyro、预积分 covariance 和对 gyro bias 的 Jacobian，并以当前三维 bias 生成 bias-corrected prediction。[declaration](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.h#L32-L106)

**事实：** `AHRSFactor(R_i,R_j,B_i)` 使用 PIM covariance 作为 factor noise；其 residual 是 bias-corrected delta rotation 与 `R_i.between(R_j)` 之差的 SO(3) Logmap。[constructor and residual](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.cpp#L92-L168)

**事实：** GTSAM 官方测试建立一串 `AHRSFactor` 和 `BetweenFactor<Vector3>(B_i,B_j,0,noise)`，并从优化结果读取末端三维 bias；这给出了 gyro-only“旋转因子 + 显式 bias random walk”的完整拓扑例证。[official test](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/tests/testAHRSFactor.cpp#L410-L468)

**工程推断：** 本项目下一 slice 用 `Vector3` gyro bias 比直接使用六维 `ConstantBias` 更小：当前没有 accel factor，额外的 accel-bias 三维只能依赖人为 prior，不能从本 slice 的观测获得约束。完整 VIO 引入 accel/velocity/gravity时，再在 PIMPL 内升级为 `ConstantBias`，不会改变 public API。

## 3. 成熟 VIO 的 state 与更新模式

| 实现 | 官方源码中的事实 | 对本项目的工程含义 |
|---|---|---|
| Kimera-VIO | 后端为每个 backend frame id 插入 `Pose`、`Velocity`、`ImuBias`；regular 路径用 `ImuFactor + bias BetweenFactor`，combined 路径用 `CombinedImuFactor`。[factor construction](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/backend/VioBackend.cpp#L899-L958) 后端读取优化 bias 并回调前端，[backend update](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/backend/VioBackend.cpp#L1365-L1380)；前端在新 keyframe 以缓存的最新 bias 重置预积分。[frontend handoff](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/include/kimera-vio/imu-frontend/ImuFrontend.h#L115-L148) | bias 是图状态，预积分 bias 是线性化点；关键帧边界是“结束一段、吸收后端估计、开始下一段”的自然 seam。 |
| ORB-SLAM3 | 显式定义三维 gyro/acc bias vertices 和 `EdgeGyroRW`/`EdgeAccRW`；RW residual 为 `b_j-b_i`，Jacobian 为 `-I/+I`。[vertices and edges](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/G2oTypes.h#L212-L253) [RW implementation](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/G2oTypes.h#L635-L689) Local inertial BA 为连续 keyframes 加 pose、velocity、gyro bias、acc bias 与 RW edges。[optimizer](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L2528-L2660) 优化后写回 keyframe bias，[writeback](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L2918-L2934)；较大 bias 改变量会触发 reintegration。[reintegration](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L3188-L3221) | 不能把“PIM 建立时的 bias”当最终常量；必须定义优化更新后的 correction/reintegration 生命周期。 |
| VINS-Fusion | 滑窗为各帧保存 `Bas`、`Bgs` 和 `para_SpeedBias`。[state storage](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.h#L116-L153) IMU residual 同时连接 `pose_i/speedbias_i/pose_j/speedbias_j`，[factor wiring](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L1015-L1061)；预积分 residual 包含 `Baj-Bai`、`Bgj-Bgi` 并做一阶 bias correction。[integration residual](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/factor/integration_base.h#L169-L194) 优化前后在 `Bgs/Bas` 与 solver parameter blocks 间双向写入，[state write/read](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L820-L917)；滑窗移动时 bias 状态随状态一起搬移，新槽以相邻状态初始化。[window slide](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L1340-L1368) | bias 与窗口生命周期绑定，而不是另跑一个完整序列离线拟合器；warm start 可来自相邻 bias，但仍需 prior/process covariance。 |
| OpenVINS | 活跃 IMU state 是 `q,p,v,b_g,b_a` 的 15-DOF state；相机 clones 只保留 pose。[IMU state](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/types/IMU.h#L30-L90) [state/clones](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/State.h#L140-L150) Propagator 从测量减去当前 bias，并在状态转移/协方差中注入 gyro 与 accel bias random walk。[propagation](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp#L163-L231) 其 noise manager 明确分开 gyro white noise 与 gyro bias random walk。[noise contract](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/utils/NoiseManager.h#L34-L58) 通用 EKF update 为每个 active variable 构造 Kalman-gain block，并把 `dx` 应用到全部 active variables。[state update](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/StateHelper.cpp#L116-L188) | “显式在线 bias state”是共同点；“每个 keyframe 都有独立 bias node”不是跨算法的唯一做法。OpenVINS 是 EKF active-state 模式，视觉 residual 可通过 cross-covariance 更新 active bias；当前项目是 batch factor graph，不能机械照搬它的单 active bias 拓扑。 |

**交叉实现结论（事实归纳）：** 四个系统都把 bias 放在在线估计状态或 active covariance state 中，并把 gyro measurement noise 与 bias random walk 分开；没有一个把完整序列的 posterior-aligned 常量当运行时唯一 bias。该归纳仅覆盖上述固定 commit 的源码范围。[Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/backend/VioBackend.cpp#L899-L958) [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L2528-L2660) [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/factor/integration_base.h#L18-L60) [OpenVINS](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/utils/NoiseManager.h#L34-L58)

**工程推断：** 对当前 GTSAM batch graph，项目内的 `G_i` chain 比引入一个独立 EKF bias tracker 更符合现有 optimizer 边界；它也为未来的完整 `ImuFactor`/`CombinedImuFactor` 保留直接演进路径。上游源码中的 `B_i` 仍指其各自完整 IMU bias key，二者不应混称。

## 4. 三种候选路线比较

| 候选 | 因果性与状态模型 | 与现有证据的关系 | 判断 |
|---|---|---|---|
| **A. explicit bias state + random walk** | `G_0` prior；每个合格图时间节点有 `G_i`；相邻节点以 `g_j-g_i ~ N(0, gyr_rw² Δt I)` 连接；gyro factor 对 `G_i` 有 Jacobian。 | 与 GTSAM AHRS 官方拓扑、Kimera/ORB/VINS 的 factor-graph/sliding-window 形态一致；OpenVINS 也支持“bias 是在线 state”这一更一般结论。[GTSAM test](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/tests/testAHRSFactor.cpp#L410-L468) | **推荐。** 最小新增 3D state，又能自然演进到完整 VIO。 |
| **B. freeze early/full bias** | full-window estimate 使用未来观测，在线运行非因果；early estimate 只在启动窗结束后因果，但之后不再跟踪 bias 或模型误差。 | Q3 已将此次 early/full 估计限定为 nuisance，并因 half stability 失败而 `STOP`。[Q3 result](m4-minimal-gyro-q3-offline-bias-alignment-result.md) | **否决为产品路径。** 可保留“零 bias/冻结 bias”作为受控对照臂，但不得用 Q3 结果初始化产品 state。 |
| **C. piecewise offline bias** | 先选窗/分段，再各段拟合常量；若分段依据未来数据则非因果，段边界还会产生未经 process covariance 解释的跳变。 | Q3 只资格化了固定 early/full 半窗检验，没有资格化自适应分段、变点检测或片段拼接。[Q3 design](m4-minimal-gyro-q3-offline-bias-alignment-design.md) | **否决为下一 slice。** 可作诊断；若把它改成因果滑窗并显式传递 covariance，本质上已接近较粗糙的在线 state estimator。 |

**事实：** 成熟实现可以先做 windowed bias initialization，但不会因此删除运行期 bias state。VINS-Fusion 的 initializer 解 gyro-bias increment、更新 `Bgs` 并 repropagate 预积分，[initial alignment](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/initial/initial_aligment.cpp#L14-L46)；之后主滑窗仍优化并回写每帧 `Bgs`。[online state write/read](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L820-L917) ORB-SLAM3 的 inertial initialization 也会把优化 bias 写入 keyframes，并在变化较大时 reintegrate；后续 local inertial BA 仍保留各 keyframe bias vertices 与 RW edges。[initialization writeback](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L3188-L3221) [local inertial BA](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L2528-L2660)

**工程推断：** 一个独立合格的 causal initializer 日后可以为 `G_0` 提供 mean/covariance，但它只能初始化 A，不能替代 A。Q3 的 nuisance estimate 既未通过稳定性门，也未提供 physical prior covariance，因此当前不能承担这个角色。

**工程推断：** B/C 看似少一个图变量，实际把 observability、时间演化和 uncertainty 藏进窗口选择与手工交接；这会重演 Q3 的 identifiability 问题。A 把这些假设放进可检查的 factor、prior 和 process noise，失败时也有明确诊断面。

## 5. 推荐最小 slice 的边界

### 5.1 状态与 factor 拓扑

候选内部拓扑：

```text
prior(G0)
   │
X0 ── gyro(X0, X1, G0) ── X1 ── gyro(X1, X2, G1) ── X2
│                            │
G0 ── rw(G0, G1, Δt01) ── G1 ── rw(G1, G2, Δt12) ── G2

现有 stereo factors / landmarks 仍连接各 Xi；gyro factor 只约束旋转 residual。
```

**本项目选择：** 在 PIMPL 内实现一个 `Pose3/Pose3/Vector3` 三元 rotation factor，使用 `X_i, X_j, G_i`；不额外创建 `Rot3` state，也不直接接入端点类型不匹配的 `AHRSFactor`。成熟源码支持的是其 residual、PIM covariance/bias correction 以及显式 bias process topology；这个具体 factor 类型是本项目为避免重复 rotation state 而选的适配层。[GTSAM AHRS residual](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.cpp#L92-L168) [GTSAM AHRS endpoint types](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.h#L108-L190)

**工程推断：** bias node 应属于每条 inertial edge 的实际图状态端点，而不是每个原始 IMU sample。状态邻接、IMU 区间 `[t_i,t_j]`、`Δt` 和 bias RW edge 必须一一对应。

**事实：** 当前 estimator 的 `update` 接受 `keyframe` 标记，但内部 window/graph 才是 `Pose3` 节点的实际所有者。[public API](../../phad/estimator/stereo_vo_estimator.hpp) [implementation](../../phad/estimator/stereo_vo_estimator.cpp)

**工程推断：** 新协议必须冻结以下二选一，而不能含糊称作“每 keyframe bias”：

- A1：每个进入图的相邻 `Pose3` temporal state 都有 bias state 与精确 IMU 区间；这是当前 batch topology 最直接的候选；
- A2：只在真正 keyframe 间建 inertial edge；则 session/sync 必须先明确如何在多个非 keyframe 更新上无损拼接连续 IMU 区间，并保证没有跨 segment/gap。

在当前实现尚未冻结这项选择前，本研究不假定 `keyframe == graph state`。无论选择哪项，keyframe policy 仍属于 composition root，factor/state 生命周期属于 estimator。

**工程推断：** 当前 window 会优先逐出旧 non-keyframe；若逐出的是两个仍存状态之间的内部节点，下一轮重建不能仅凭两个 pose/bias 数值臆造跨越区间。协议必须要求 window 持有足以重建该 surviving adjacency 的连续 IMU provenance/PIM，或明确不建这条 inertial edge 并处理 bias-chain 连通性；任何跨 gap/segment 的隐式拼接都应失败。[current eviction/rebuild implementation](../../phad/estimator/stereo_vo_estimator.cpp)

**工程推断：** transient rebuild 还有一个上游实现不能替本项目回答的问题：原始 `G_0` 被逐出后，初始 bias prior 也随旧图消失。下一协议必须冻结新 oldest `G(k)` 的 prior handoff（mean、covariance、与 pose 的相关性如何处理）。只回写 point estimate 而不声明 uncertainty，会把一个启发式 warm start 伪装成 posterior transfer；每轮重新套用固定 prior 也可能重复利用重叠窗口的信息。当前 estimator 对 pose 的 point re-anchor 方式是本地既有事实，但不能自动证明它对 bias 足够。[current graph rebuild](../../phad/estimator/stereo_vo_estimator.cpp)

### 5.2 应复用

- **Q1/sync：** 精确覆盖、同 segment、缺口/越界拒绝、样本所有权与端点约定；不得为凑 factor 跨 gap 或跨 segment 拼接。[Q3 对 Q1 输入合同的引用](m4-minimal-gyro-q3-offline-bias-alignment-design.md)
- **Q2：** endpoint trapezoid、严格递增时间戳、`R_i_j` 方向、SO(3) 积分、异常与零运动等定向测试。[Q2 helper](../../phad/estimator/gyro_rotation_predictor.cpp)
- **sensor calibration：** `gyr_nd` 与 `gyr_rw` 两个既有量及单位合同。[ImuParameters](../../phad/sensor/imu_parameters.hpp)
- **estimator PIMPL：** GTSAM keys、PIM、factor 与优化后 bias 都留在 `Impl`；public API 不暴露 GTSAM。[estimator AGENTS](../../phad/estimator/AGENTS.md)
- **GTSAM：** `PreintegratedAhrsMeasurements` 的 covariance/bias correction 与官方 `AHRSFactor` residual 作为实现 oracle；`BetweenFactor<Vector3>` 和 `Diagonal::Sigmas` 直接复用。[AHRS API](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.h#L32-L190) [`BetweenFactor`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/slam/BetweenFactor.h#L110-L123)

### 5.3 不应复用

- 不复用 Q3 early/full nuisance estimate 作为 `G_0` 的 mean、prior covariance 或产品默认值；Q3 没有给它这项资格。[Q3 result](m4-minimal-gyro-q3-offline-bias-alignment-result.md)
- 不复用旧 Q4“固定 known bias 后只加 rotation constraint”的授权链；其前置条件是 Q3 PASS，而实际结果是 STOP。[Q3 result](m4-minimal-gyro-q3-offline-bias-alignment-result.md)
- 不直接在图中调用 Q2 result-only helper 代替 PIM；它没有 optimizer 所需 covariance 与 bias Jacobian。[Q2 header](../../phad/estimator/gyro_rotation_predictor.hpp)
- 不为直接使用 `AHRSFactor` 复制一套与 `Pose3` 平行的 `Rot3` state；这会新增同步一致性约束和 gauge 面。官方 `AHRSFactor` 的两个端点类型确为 `Rot3`。[GTSAM declaration](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.h#L108-L190)
- 不在 gyro-only slice 提前引入 `V_i`、重力、accel bias 或 `CombinedImuFactor`；后者合同明确需要 pose、velocity 与六维 bias 两端点。[CombinedImuFactor declaration](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/CombinedImuFactor.h#L220-L247)

## 6. 新协议必须先冻结的内容

本节是研究建议，不是实现授权。

### 6.1 机制合同

1. 图状态粒度：A1 每 graph state，或 A2 仅 keyframe；不得在看到真实数据结果后切换。
2. 首个 `G_0` mean/covariance 的独立来源，以及 original prior node 被逐出后的 oldest-bias prior handoff；未知时应明确标为待资格化，不能从 Q3 nuisance result 回填。
3. `gyr_nd`、`gyr_rw` 到 covariance/sigma 的精确公式和单位；`Δt` 来自每条实际区间。
4. PIM 的 `biasHat` 初始化、优化后 bias correction、重建 graph 时 reintegration/reset 的生命周期。
5. factor residual 的 frame convention、SO(3) 顺序、Pose3 rotation Jacobian、translation Jacobian 应为零。
6. gap、segment change、缺端点、非严格时间戳、空/单点区间的显式错误或 skip reason。
7. window eviction 时 `X_i/G_i/PIM` 的共同生命周期；不得留下孤立 bias 或把 RW edge 接到错误时间邻居。
8. 产品开关与纯视觉 control：关闭时必须不创建 bias/PIM/factor，现有输出与诊断合同不受隐式改变。

### 6.2 先于真实数据的 synthetic mechanism gates

- exact zero-bias 与已知 constant-bias：检查 residual、符号、单位及 bias recovery；
- 已知线性 random-walk/drift：检查 `sqrt(Δt)` scaling 与不等长区间；
- 数值 Jacobian 对拍：`X_i` rotation、`X_j` rotation、`G_i`，以及两个 Pose3 translation block 为零；
- biasHat 改变后的一阶 correction 与 full reintegration 对拍；超过冻结误差域时必须 reintegrate 或拒绝，不能静默继续；
- 无旋转激励、退化视觉姿态或过强/过弱 prior：应暴露 rank/conditioning/observability 诊断，不得声称 bias 已收敛；
- gap/segment/window eviction/keyframe policy 的拓扑测试；
- factor off 的纯视觉结果与诊断对照。

GTSAM 的 gyro-only 官方测试可作为最小 synthetic oracle，但其 prior/noise 数值只是测试数据，不能复制成本项目产品参数。[GTSAM test](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/tests/testAHRSFactor.cpp#L410-L468)

### 6.3 建议的新资格阶梯

1. **合同冻结：** state topology、prior source、noise semantics、PIM lifecycle、错误/诊断与 control arm。
2. **纯 synthetic 机制门：** 先证明 factor/Jacobian/process model 正确，且失败模式可见。
3. **冻结输入的受控图实验：** 在线 bias state 真正进入 posterior；比较 A 与“factor off/固定 bias control”，但不以 ATE 反向调 prior/RW。
4. **新协议下的真实序列资格化：** 数据集、序列、阈值、看数顺序与 STOP 条件必须预先冻结；这不是 Q3 rerun。
5. **完整 VIO 后续 slice：** 增加 velocity、gravity、accelerometer bias，并重新选择/资格化 classic `ImuFactor + BetweenFactor` 或 `CombinedImuFactor`。

**工程推断：** “shadow compute 一个 online bias、但完全不连接 posterior”不能验证可观测性，因为 bias 正是通过视觉 pose 与 gyro residual 的耦合获得信息。若需要 shadow 阶段，它只能验证数据流、PIM 数值与诊断，不得声称已验证 bias estimation。

## 7. 待确认项与明确非结论

以下没有足够本地证据，必须由下一协议冻结，而不是由本研究猜测：

- A1（每 graph state）还是 A2（仅 keyframe）的最终选择；
- `G_0` 的 mean、covariance 与独立证据来源；
- 真实序列的 bias observability 判据和数值阈值；
- 何种 bias delta/线性化误差触发 reintegration；
- gyro factor 的产品默认开关、完整 VIO 的 rollout 顺序和真实数据验收指标。

本研究没有运行 Q3、GT、ATE、RPE 或任何真实序列；没有生成新的数值证据。它支持的唯一决策是：**废止“从 Q3 nuisance estimate 继续固定 bias 调参”的实现路线，改写为 explicit online gyro bias state + random walk 的新资格协议；在该协议通过前不接入现有产品 posterior。**
