# M4 minimal gyro Q2：known-bias deterministic predict 设计

日期：2026-08-12

状态：**Q2 implementation provisional；独立资格 pending，尚未取得 Q2 final 结论**

固定点：`c0e214a04f8521dcf7f1c2769ebf10bf7ca06051`
（分支 `Nothand0212/m4-q2-gyro-predict`）

权限：GitHub issue
[#37](https://github.com/Nothand0212/phad-vio/issues/37) 授权 Q2 research、plan 与经评审的
Q2 deterministic implementation/qualification；当前实现结果仅为 provisional。前置 Q1 final PASS 见
[Q1 Observe 资格结果](m4-minimal-gyro-q1-observe-result.md)。上位设计与方法分别见
[M4 gyro measurement / factor 资格实验设计](m4-minimal-gyro-slice-design.md)和
[证据门控的信息接入](../agents/evidence-gated-integration.md)。

本文档描述当前约定，不是绝对约束，会随项目开发修订。

## 0. 结论、证据类别与权限边界

### 0.1 本片唯一问题

**项目决策：**Q2 只回答一个可证伪问题：给定 timestamped gyro point samples 与同一
body frame、同为 rad/s 的已知常值 bias，能否确定性地算出区间两端之间的

\[
\Delta R_{ij}=R_{WB}(t_i)^\top R_{WB}(t_j)
\]

并保留 exact `int64` nanosecond duration？Q2 结果只能是独立 open-loop prediction，
不得进入 visual estimator、factor graph、posterior、初始化或 feedback。

### 0.2 本文标签

- **来源事实**：由本仓库合同、一手源码或已经完成的 Q1 证据直接支持。
- **项目决策**：本项目为 Q2 冻结的行为；不能反推为外部项目的统一做法。
- **假设**：用于 synthetic qualification 的解析运动或数值环境，实施时必须按本文复现。
- **未授权范围**：本片不得实现或以 Q2 通过为由提前放行的能力。

### 0.3 已有事实与本片不声明的事情

**来源事实：**Q1 executable identity 是 commit
`74270572cc1fcc2eac82559117efd0951c800ab9`，其独立验证结果为 final PASS；gyro packet 与
逐样本 CSV 可独立重放，M3 的 `est.tum`、`kf.tum`、`diag.csv`
仍与 control byte-identical。当前 fixed point `c0e214a...` 只记录了这一资格结论；它没有
提供 gyro integration、bias fit、factor 或 posterior 证据。

**未授权范围：**本设计不读取 MH_01、ATE、visual pose 或 Q1 CSV，不估计 bias，不传播
covariance，不构造 factor，不运行 optimizer，不修改 config/hash，不接入
`StereoVoEstimator`、apps/session/bench，也不改变 M3 输出。Q3 Align、Q4 Constrain、
Q5 Feedback 均未授权。Q2 PASS 最多允许另行规划 Q3；不是 Q3/Q5 实现许可，更不是 VIO
完成声明。

## 1. 一手资料对照

### 1.1 GTSAM 4.3a0：实际依赖与调用语义

**来源事实：**当前构建从 `/usr/local/lib/cmake/GTSAM` 找到 GTSAM，安装头中的
`GTSAM_VERSION_STRING` 为 `4.3a0`；对应上游 tag `4.3a0`、commit
[`3ad4b4c3cb28394c9597f48fa02dad361c8450e3`](https://github.com/borglab/gtsam/tree/3ad4b4c3cb28394c9597f48fa02dad361c8450e3)。本地同 commit 源码用于以下核对。

- [`PreintegratedRotation.h`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegratedRotation.h#L31-L50)
  定义 `internal::IncrementalRotation{measuredOmega, deltaT, body_P_sensor}`；同文件将
  `deltaRij_` 定义为 frame i 中的 preintegrated relative orientation，并公开
  [`integrateGyroMeasurement(measuredOmega, biasHat, deltaT)`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegratedRotation.h#L112-L188)。
- [`IncrementalRotation::operator()`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegratedRotation.cpp#L71-L94)
  先且只先执行 `correctedOmega = measuredOmega - bias`，再对可选的
  `body_P_sensor` 旋转，最后执行 `Rot3::Expmap(correctedOmega * deltaT)`。
- [`integrateGyroMeasurement`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegratedRotation.cpp#L97-L111)
  执行 `deltaTij_ += deltaT` 与 `deltaRij_ = deltaRij_.compose(incrR)`。因此按输入
  chronological iteration 调用时，增量为 right-compose：
  `DeltaR <- DeltaR * Exp(...)`。
- 上述 API 不检查 `deltaT > 0`、timestamp overflow 或所有输入/output finite；这些检查
  必须由项目 helper 在调用 GTSAM 前后完成，不能把 GTSAM 的正常返回当成合同验证。
- GTSAM 自有
  [`testAHRSFactor.cpp`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/tests/testAHRSFactor.cpp#L55-L102)
  按容器顺序累积 measurement，并检查 rotation、duration 与 covariance；其
  [bias 用例](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/tests/testAHRSFactor.cpp#L158-L183)
  把含 bias 的 measured omega 交给 PIM、再用 bias 求零 residual。长序列 predict、graph
  optimization 与 bias posterior 见
  [同文件的 optimizer/posterior 用例](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/tests/testAHRSFactor.cpp#L374-L407)，
  它们只用于理解 API，均超出 Q2。

**项目决策：**DUT 使用 base `gtsam::PreintegratedRotation` 的 public
`integrateGyroMeasurement`，不使用 deprecated `integrateMeasurement`。DUT 将未经 bias
预减的 interval representative 和 `known_bias` 分别传入，令 GTSAM **恰减 bias 一次**。
`body_P_sensor` 保持 unset，因为 Q2 输入与 bias 已在项目 body/IMU-aligned frame。

**项目决策：**不使用 `PreintegratedAhrsMeasurements`。`AHRSFactor` 的声明明确绑定
Rot3/bias keys，并公开 factor error/Jacobian 接口；参见
[`AHRSFactor.h`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.h#L135-L165)。其实现从
PIM covariance 构造 noise model（[`AHRSFactor.cpp`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.cpp#L92-L98)），并计算带 bias correction 的
factor error/Jacobians（[同文件](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.cpp#L122-L168)）。posterior 只有在另行构造 graph 并运行 optimizer 后才产生；对应
[GTSAM 测试](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/tests/testAHRSFactor.cpp#L374-L407)
超出 Q2。Q2 只需 deterministic rotation；选 base class 可复用 GTSAM 的 Expmap、bias 与
compose 语义，又不会把 covariance、factor 或 optimization 偷渡进本片。

### 1.2 Kimera-VIO

**来源事实：**以下链接固定在 Kimera-VIO commit
[`ce8c59b7b273ab5ac29db7e5572e1623760e19c7`](https://github.com/MIT-SPARK/Kimera-VIO/tree/ce8c59b7b273ab5ac29db7e5572e1623760e19c7)。本机快照没有 `.git`，但所引用文件的
SHA-256 已与该 commit 的 raw 内容核对一致。

- [`ThreadsafeImuBuffer`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/include/kimera-vio/utils/ThreadsafeImuBuffer-inl.h#L52-L67)
  以 integer-ns timestamped point samples 入队并要求严格递增；其
  [双端 interpolation](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/utils/ThreadsafeImuBuffer.cpp#L181-L226)
  为查询边界生成精确 timestamp 样本。
- [`ImuFrontend::preintegrateGyroMeasurements`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/imu-frontend/ImuFrontend.cpp#L200-L219)
  使用 cached known gyro bias、从相邻 integer-ns stamps 求 seconds，再按时间顺序调用
  GTSAM。
- 但该循环每段只传左端 `measured_omega`；
  [`StereoVisionImuFrontend`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/frontend/StereoVisionImuFrontend.cpp#L121-L130)
  也明确最后一个 interpolation value 不参与其 ZOH 积分。

**项目决策：**借鉴 Kimera 的 timestamped samples、严格顺序、integer-ns dt 和 exact
boundary seam；不照抄其左端 ZOH rule。

### 1.3 ORB-SLAM3

**来源事实：**ORB-SLAM3 commit
[`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4)
的 [`Tracking::PreintegrateIMU`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L1683-L1725)
对区间内部使用相邻 endpoints 的平均；frame 边界 off-grid 时，先从相邻原样本插值边界
value，再与区间另一端求平均，没有创建 midpoint timestamp。

[`IntegratedRotation`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/ImuTypes.cc#L84-L105)
内部减一次 gyro bias 并用 Rodrigues；
[`IntegrateNewMeasurement`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/ImuTypes.cc#L177-L235)
以 `dR = dR * deltaR` right-compose 并累计 `dT`。

**项目决策：**采用其 endpoint-average 与 right-compose 形态；不采用其 double-seconds
timestamp、float `dt`、自有 Rodrigues/covariance 或 full inertial state。

### 1.4 VINS-Fusion

**来源事实：**VINS-Fusion commit
[`be55a937a57436548ddfb1bd324bc1e9a9e828e0`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/tree/be55a937a57436548ddfb1bd324bc1e9a9e828e0)
的
[`midPointIntegration`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/factor/integration_base.h#L63-L80)
使用 `0.5 * (gyr_0 + gyr_1) - linearized_bg` 并 right-multiply quaternion；
[`propagate`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/factor/integration_base.h#L139-L165)
保存相邻 point samples、normalize 并累计 `sum_dt`。runtime
[`processIMU`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L375-L408)
也使用 endpoint trapezoid、一次 bias subtraction 与 right-compose。

**项目决策：**采用其相邻 endpoint trapezoid 语义；不采用其 `deltaQ` 小角近似、double
timestamp、acceleration/state/covariance/Jacobian 传播。

### 1.5 OpenVINS

**来源事实：**OpenVINS commit
[`69488123ed9362dd44b6f28e7f4680abbff1442b`](https://github.com/rpng/open_vins/tree/69488123ed9362dd44b6f28e7f4680abbff1442b)
的
[`Propagator::select_imu_readings`](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp#L269-L393)
从 timestamped IMU buffer 选取传播区间，并以
[`Propagator::interpolate_data`](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.h#L145-L164)
在查询边界构造 endpoint value；同文件的
[`Propagator::predict_and_compute`](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp#L395-L447)
路径对相邻 IMU endpoints 求平均、扣除 gyro bias，并进入 propagation/predict 计算。

**项目决策：**借鉴其 off-grid boundary endpoint 与相邻 endpoint average 的职责分离；不采用
其 double timestamp、EKF/MSCKF state、RK4/discrete state propagation、covariance、Jacobian
或 quaternion state convention。OpenVINS 的 interpolation 在 sync/propagator 边界完成，不能
据此要求 Q2 helper 再插值一次。

### 1.6 对照结论

**受来源支持的推断：**成熟实现共同支持 timestamped point samples、chronological
composition 与 known bias 单次扣除；ORB-SLAM3/VINS-Fusion/OpenVINS 支持相邻 endpoint
average，Kimera/OpenVINS 支持 boundary interpolation seam。各项目 frame convention 与 state
scope 不同，right-compose 的本地合同仍以 GTSAM base class 与本项目 `Delta R_ij` 定义为准。

**项目决策：**Q2 把上述模式组合为一个更严格的本地合同：checked integer-ns duration、
相邻 endpoint trapezoid、GTSAM 4.3a0 base exact Expmap、exact integer duration output。
没有任何单一外部参考完整实现了这一组合，不能把本项目决定写成“照搬某库”。

## 2. 冻结数学合同

### 2.1 输入与 interval representative

输入是：

```text
samples: chronological span<sensor::ImuMeasurement>, size >= 2
known_bias_radps: finite Eigen::Vector3d, 与 sample gyro 同一 body frame
```

**项目决策：**`ImuMeasurement` 是 timestamped **point sample**；不是 sample-and-hold
区间值。对相邻 endpoints `k,k+1`：

\[
d_k^{ns}=t_{k+1}^{ns}-t_k^{ns},\qquad
dt_k=\operatorname{double}(d_k^{ns})\times10^{-9},
\]

\[
\bar\omega_k=\tfrac12\omega_k+\tfrac12\omega_{k+1}.
\]

必须先在 integer domain 检查 subtraction，确认 `d_k^{ns}>0` 且可表示，再转换为
seconds。严格递增 timestamp 经 checked subtraction 得到的正 `int64` duration，在转换为
double seconds 后于可达域内必然 finite 且为正，因此不为该不可达情形设置 runtime gate 或
typed error。平均只形成传给 integrator 的 interval representative；它不是在两端已由
production sync 插值之后再做一次时间插值，即 **endpoint average 不是 double
interpolation**。DUT 不创建 synthetic midpoint timestamp 或 midpoint `ImuMeasurement`。

### 2.2 GTSAM 调用、bias 与 compose 顺序

对每个 interval，严格执行：

```cpp
pim.integrateGyroMeasurement(omega_bar_k, known_bias_radps, dt_k);
```

不得把项目代码预减 bias 后的 omega 传给 GTSAM；否则 GTSAM 会再次减 bias。按 samples
的输入顺序迭代，GTSAM 产生：

\[
\Delta R_{k+1}=\Delta R_k
\operatorname{Exp}((\bar\omega_k-b_g)dt_k),\qquad \Delta R_0=I.
\]

这一定义是 chronological **right-compose**；reverse iteration、left-compose、rotation
inverse 或 sign flip 都是合同错误。

**项目决策：**调用 GTSAM 前，项目代码必须另外计算一次仅用于 validation 的
`corrected_omega = omega_bar_k - known_bias_radps` 并检查其每轴 finite；validation 不改变
传给 GTSAM 的两个实参，仍由 GTSAM 恰减 bias 一次。endpoint average 必须写成
`0.5 * omega_k + 0.5 * omega_k_plus_1`，避免两个有限同号大数先相加而溢出。

### 2.3 输出

成功输出只包含：

```text
delta_R_i_j: Eigen::Matrix3d  // R_WB(t_i)^T R_WB(t_j)
duration_ns: int64            // samples.back.timestamp - samples.front.timestamp
```

`duration_ns` 来自 checked integer arithmetic，是时间权威值；GTSAM 的 double
`deltaTij()` 只可作内部一致性诊断，不能替代 exact output。返回前必须检查 matrix finite、
`R^T R` 与 `det(R)`；不得把 invalid result normalize 成“成功”。

## 3. Module seam 与 API 取舍

### 3.1 选择：独立 deterministic helper

**项目决策：**未来实现放在 `phad::estimator` 的独立 deep module，例如：

```cpp
struct GyroRotationPrediction {
  Eigen::Matrix3d delta_R_i_j;
  std::int64_t duration_ns;
};

GyroRotationResult integrateGyroRotation(
    std::span<const sensor::ImuMeasurement> samples,
    const Eigen::Vector3d& known_bias_radps);
```

确切命名可在 Q2 plan review 时按项目 C++ naming 收敛，但语义不得变化：public header
只暴露 Eigen、`sensor::ImuMeasurement`、project result/error types，不暴露 `gtsam::Rot3`、
  PIM、Params、factor 或 optimizer。GTSAM 只在 `.cpp` 中 PRIVATE 使用；`phad_estimator`
  对 `phad_sensor` 建立直接依赖，不能依赖 `phad_sync` 或其他 target 偶然转递
  `sensor::ImuMeasurement`。

helper 接收 samples 而不是 `StereoImuPacket`，因为 point-sample integration 不应知道 image、
`imu_gap`、segment 或 visual topology。packet/gap eligibility 属未来调用方边界。

### 3.2 被否决的备选

| 备选 | 否决原因 |
|---|---|
| 在 `StereoVoEstimator::Impl` 内写循环 | Q2 无 production caller；会把独立预测与 visual posterior seam 耦合，Q3 也难以复用 |
| helper 接收 `StereoImuPacket` | 把 image/gap/sync policy 泄漏进纯积分；synthetic 与 Q3 offline 使用不自然 |
| 放 `phad::sync` | sync 只拥有切段与 interpolation，不知道 bias/GTSAM；违反现有依赖方向 |
| 放 `apps` 或 `phad::bench` | 形成第二套数学实现；bench 还要求零 `phad::*` 数据处理职责 |
| 返回 `gtsam::Rot3` 或 PIM object | 泄漏 PRIVATE 依赖并预留未经验证的 covariance/bias-correction 权限 |
| 使用 `PreintegratedAhrsMeasurements` | 同时引入 covariance 容器，诱导 Q2 之后绕过资格门直接构造 AHRS factor |
| 自研 Rodrigues / quaternion integrator | 项目已有 GTSAM exact Expmap；重复实现增加 oracle 与 DUT 同错的风险 |

### 3.3 Q3/Q5 复用与 deletion test

**项目决策：**Q3 offline bias alignment 和 Q5 gyro-enabled path 若日后分别获授权，必须
调用**同一个** deterministic helper；不得在 script、apps、factor 或 session 再写一套
trapezoid/bias/compose。Q3 可以多次以 candidate bias 调 helper，Q5 可以把 helper 输出交给
已取得资格的 factor，但本片不实现两者。

Q2 实施评审应包含 deletion test：

1. 删除 helper 的 `.cpp/.hpp` 与其 Q2 tests 后，现有 production libraries/apps 的 source
   不需要修改，证明 Q2 尚未接入产品路径；
2. 保留 tests 而删除 helper 时，Q2 target 必须 link/compile fail，证明 tests 实际约束的是
   该 helper，不是测试内副本；
3. `rg` 应只有 helper tests（以及未来显式授权的 Q3/Q5 caller）引用 API；Q2 完成时不得出现
   `StereoVoEstimator`、apps/session/bench caller。

## 4. Errors 与 hard-fail matrix

### 4.1 失败语义

所有合同错误都返回带稳定 code、sample index/timestamp 与 detail 的失败结果；不得 throw
裸字符串、skip interval、返回 identity、clamp dt、排序输入、去重、静默关闭或报告部分成功。
建议 error codes：

```text
kInsufficientSamples
kNonFiniteGyro
kNonFiniteBias
kDuplicateTimestamp
kOutOfOrderTimestamp
kTimestampDeltaOverflow
kDurationOverflow
kNonFiniteComputation
kNonFinitePrediction
kInvalidRotation
```

`std::variant` 风格 project result 足以满足 C++20；不为本片引入新依赖或通用 error
framework。

### 4.2 预注册矩阵

| 输入/边界 | 期望 | 说明 |
|---|---|---|
| 0 或 1 sample | hard fail `kInsufficientSamples` | 不返回 identity success |
| 任一 gyro axis NaN/±Inf | hard fail `kNonFiniteGyro` | 报首个坏 sample index |
| bias 任一 axis NaN/±Inf | hard fail `kNonFiniteBias` | 在任何 GTSAM 调用前 |
| duplicate timestamp | hard fail `kDuplicateTimestamp` | `dt=0` 不可积分 |
| decreasing timestamp | hard fail `kOutOfOrderTimestamp` | 不自动排序 |
| `next_ns-current_ns` 超 `int64` | hard fail `kTimestampDeltaOverflow` | 先 checked subtract，禁止 signed UB |
| 正 interval 的累计 duration 超 `int64` | hard fail `kDurationOverflow` | output 无法精确表示 |
| endpoint mean、corrected omega 或 rotation vector 在调用 GTSAM 前非 finite | hard fail `kNonFiniteComputation` | 首个坏 interval；不调用 GTSAM、不回 identity/partial |
| GTSAM 最终 output matrix 非 finite | hard fail `kNonFinitePrediction` | 不把 nonfinite prediction 返回为 success |
| finite output matrix 的 orthogonality/determinant 超门 | hard fail `kInvalidRotation` | 不 normalize 掩盖错误 |
| timestamp/gyro/bias 相同，仅 accel 在正常值、任意有限值、NaN、`+Inf`、`-Inf` 间变化 | 所有变体 success，duration 与 matrix 每元素 exact-equal | hard gate：证明 helper 完全不读取 accel |
| negative epoch、严格递增且差值安全 | success | epoch 正负不影响 duration |
| 单个 1 ns interval | success，`duration_ns==1` | 不以“太小”为由丢段 |
| `[INT64_MIN, INT64_MIN+1]` | success，duration 1 ns | lower bound safe case |
| `[INT64_MAX-1, INT64_MAX]` | success，duration 1 ns | upper bound safe case |
| `[INT64_MIN, INT64_MAX]` | hard fail overflow | 不能先 subtract 再检查 |

`kNonFiniteComputation` 的预注册可达 fixture 使用合法、严格递增 timestamp，所有 gyro/bias
输入在进入运算前均 finite；两个 endpoints 的同一 gyro axis 都取 `DBL_MAX`，bias 同轴取
`-DBL_MAX`，其余轴取有限正常值。production 以
`0.5 * DBL_MAX + 0.5 * DBL_MAX` 得到 finite `DBL_MAX`，随后仅用于 validation 的
`DBL_MAX - (-DBL_MAX)` 必须自然 overflow 为 `Inf`；DUT 必须在把该 interval 交给 GTSAM 前
返回 `kNonFiniteComputation`，并报告 `interval_index=0`、不返回 partial prediction。不得通过
注入、临时改常量或新增 test seam 制造该错误；若 verifier 平台上这个已预注册 fixture 不能
自然触发，Q2 整体判为 **INCONCLUSIVE 并停止**。

final-output defensive branches 另冻结两个无 injection 的 public-input fixture；两者在进入
GTSAM 前的 safe endpoint mean、corrected omega 与 rotation vector 均须逐元素 finite：

- `kNonFinitePrediction`：timestamps `[0,1]` ns，两个 endpoints gyro 均为
  `[DBL_MAX,0,0]` rad/s，bias 为 zero。当前冻结环境下 GTSAM output 为 3x3 全 NaN；public
  result 必须为 `kNonFinitePrediction`，`sample_index`、`interval_index`、`timestamp_ns`
  均为 `nullopt`。
- `kInvalidRotation`：471 samples，timestamps 为 `0..470` ns，每点 gyro 均为
  `[-2.4664720996731243e54,1.9669167514335542e54,-2.5705088372089910e54]` rad/s，bias
  为 zero。当前冻结环境下 470 intervals 的 output matrix finite，
  `||R^T R-I||_F=1.0012190690733831e-12`、`|det(R)-1|=7.5839334812144443e-13`；public
  result 必须为 `kInvalidRotation`，三个 optional 字段均为 `nullopt`。

这两组 actual 依赖 GTSAM `4.3a0`、Eigen `3.4.0`、GCC `13.3.0`、glibc `2.39`、x86_64、
Release matrix-mode Rot3/Expmap 路径；不是 GTSAM 跨平台合同。换平台后任一 fixture 无法自然
复现时，Q2 为 **INCONCLUSIVE 并停止**，不得更换常量或增加 injection seam。独立资格除运行
这两个 runtime 门外，还须审计 production 的 output finite 与 SO(3) defensive checks。

**未授权范围：**`imu_gap` 是 packet-level eligibility，不是 helper 输入。未来 caller 对 gap
可按其资格层显式 skip factor，但不得把 malformed sample span 当 gap 静默处理。

## 5. 独立数值 oracle 与冻结环境

### 5.1 Oracle identity

**假设/预注册：**oracle 使用 CPython `3.12.3`、`mpmath 1.3.0`、`mp.dps=80`。oracle
不得 import/call GTSAM、Eigen rotation utility、DUT helper 或复制 DUT C++；它只用 mpmath
scalar/matrix 运算实现 Rodrigues：

\[
\operatorname{Exp}(\phi)=I+\frac{\sin\theta}{\theta}[\phi]_\times
+\frac{1-\cos\theta}{\theta^2}[\phi]^2_\times,
\quad \theta=\|\phi\|,
\]

并为 `theta=0` 使用解析极限。所有 oracle constants、版本、precision 和输出 full-precision
文本须在 RED/qualification transcript 固定 hash。

SO(3) geodesic 不用 trace-only `acos`，而用：

\[
A=R_{ref}^\top R_{dut},\quad
s=\tfrac12\sqrt{(A_{32}-A_{23})^2+(A_{13}-A_{31})^2+(A_{21}-A_{12})^2},
\]

\[
c=\tfrac12(\operatorname{tr}A-1),\qquad
e_R=\operatorname{atan2}(s,c).
\]

这样小角误差不因 `acos` 接近 1 的 conditioning 被抹掉。DUT test 读取冻结的数值常量；
不能在同一个 C++ test 中用 DUT/GTSAM 重新生成 expected。

### 5.2 主非交换解析轨迹与二阶门

**假设：**

\[
R(t)=R_x(1.1t)R_y(0.7t),\quad
\omega_B(t)=[1.1\cos(0.7t),\ 0.7,\ 1.1\sin(0.7t)]^\top,
\]

\[
T=0.4\ \mathrm{s},\qquad b_g=[0.12,-0.08,0.05]^\top\ \mathrm{rad/s}.
\]

sample measurement 为 `omega_B(t)+b_g`，timestamps 从 exact integer ns grid 生成；参考
relative rotation直接取解析 `R(0)^T R(T)`。三组 grid：

| `h` (s) | step (ns) | 80-dps oracle geodesic error (rad) | absolute upper `U=0.06 h²` |
|---:|---:|---:|---:|
| 0.04 | 40,000,000 | `7.288791970389558e-5` | `9.6e-5` |
| 0.02 | 20,000,000 | `1.822294989221110e-5` | `2.4e-5` |
| 0.01 | 10,000,000 | `4.555798094738877e-6` | `6.0e-6` |

正常 DUT 每格须同时满足 `error <= U`。相邻误差 ratio
`e(h)/e(h/2)` 均须在 `[3.8,4.2]`；冻结 oracle ratios 约为
`3.9997870890842715`、`3.9999467740362142`。absolute 与 ratio 门缺一不可：前者防统一
大误差仍呈二阶，后者防单点偶合。

### 5.3 Irregular exact-ns cases

所有下列 case 使用 timestamps：

```text
[0, 7000000, 31000000, 60000000,
 113000000, 191000000, 260000000, 400000000]
```

每个成功结果必须 `duration_ns == 400000000`。

| case | measured gyro | exact expected | geodesic upper |
|---|---|---|---:|
| stationary + nonzero bias | `b_g` | `I` | `2e-12 rad` |
| constant 3-axis rate | `[0.35,-0.42,0.27] + b_g` | `Exp([0.35,-0.42,0.27] * 0.4)` | `2e-12 rad` |
| linear commuting | `u(0.3+1.4t)+b_g`, `u=[2,-1,2]/3` | `Exp(u*(0.3T+0.7T²))` | `2e-12 rad` |

trapezoid 对 linear commuting case 在每个 irregular interval 上积分标量线性函数应为 exact；
该 case 专门隔离 quadrature rule，不用非交换误差替错误方法兜底。

所有正常 success case（含主轨迹三 grid、1 ns、near-bounds 与 fixtures）另须：

```text
||R^T R - I||_F <= 1e-12
abs(det(R) - 1) <= 1e-12
```

## 6. Mutant/reject lower bounds

mutant 必须由 test-only 独立实现或预先生成常量表达；不得把 mutant branch 留进 production
helper。`L` 是错误实现相对 exact reference 的 geodesic **lower bound**：

| mutant | fixture | reject lower `L` | 预注册 actual / 说明 |
|---|---|---:|---|
| reverse compose/order | 主非交换轨迹，三组 grid | 每格 `>= 8e-3 rad` | `h=0.04/0.02/0.01` actual 分别为 `0.010562749192115032`、`0.010591148277551452`、`0.010598253194581134 rad` |
| left-ZOH | irregular linear commuting | `>= 2e-2 rad` | actual `0.024304 rad` |
| double bias subtraction | stationary + nonzero bias | `>= 5e-2 rad` | actual `0.061057350089895 rad` |
| rad/s 被当作 deg/s（乘 `pi/180`） | constant rate | `>= 0.20 rad` | actual `0.239644733199408 rad` |
| deg/s 被当作 rad/s（乘 `180/pi`） | constant rate | `>= 1.0 rad` | actual `1.164261180505555 rad` |
| angular-rate sign flip | constant rate | `>= 0.40 rad` | actual `0.4878032390 rad` |

这些 lower bounds 不是正常 DUT 的 tolerance。每个 mutant test 同时要求：对应正常 DUT 通过
其 upper bound，mutant 错误达到 `L`；否则不能宣称测试具有区分力。

## 7. 既有 sync 前提与 shared endpoint fixture

Q2 helper 不重做 interpolation。fixed point 上既有 sync tests 是 Q2 的上游资格前提；Q2
只在 estimator test 中证明 helper 能消费 endpoint-aligned samples。

### 7.1 Fixed-point sync 上游前提

**来源事实：**fixed point 上 sync 首包是零段；Q2 不声称首包发生 interpolation。既有
`TEST InterpolatedRightBoundaryChainsToNextSegment` 已覆盖 interpolation 产生的右端点被下一
segment 链式复用。该测试与现有 sync 合同共同构成 Q2 上游前提；Q2 不新增、修改或链接
sync test，也不修改 sync production policy。

### 7.2 Estimator helper-owned fixture

estimator test 预构造 endpoint-aligned samples，使用 linear commuting
`omega(t)=u(0.3+1.4t)+b_g`；两段分别覆盖 `[t0,t1]`、`[t1,t2]`，且两段输入的 shared
endpoint `t1` timestamp/gyro 完全相同；whole-span 输入覆盖 `[t0,t2]`，其中 shared endpoint
`t1` 只保留一次。known bias 不变，分别进行三次 helper 调用，得到 `(R01,d01)`、
`(R12,d12)` 与 `(R02_whole,d02_whole)`，断言：

- `d01 + d12 == t2 - t0`，且加法在 checked integer domain 中完成；
- segmented result `R01 * R12` 与 `R02_whole` 的 geodesic error `<= 2e-12 rad`；
- segmented result `R01 * R12` 和 whole result `R02_whole` 各自与 linear commuting 在
  `[t0,t2]` 上的 analytic relative rotation geodesic error `<= 2e-12 rad`；
- 三次调用各自的 result duration 都 exact-equal 于其 endpoint timestamp difference。

该 fixture 防止未来 Q3/Q5 在 shared endpoint 处重复一个 interval、丢 interval，或另造
midpoint semantics。它不调用、链接或修改 sync，也不授权 packet merge。

## 8. 正常 DUT / mutant 判定与整体 stop/go

### 8.1 单项判定定义

| 对象 | PASS | FAIL | INCONCLUSIVE |
|---|---|---|---|
| 正常 DUT | 本文所有 upper bounds、exact duration、SO(3)、unused-accel invariance、自然 runtime 门与 defensive code audit 全满足 | 任一 deterministic expectation 不满足，或错误被 skip/fallback/identity 掩盖 | 仅资格前提、工具版本、冻结 oracle identity，或任一冻结自然 fixture 无法复现，尚未得到有效 DUT observation |
| 单个 mutant gate | 正常 DUT 先通过对应正例，且 mutant error `>= L`，证明 test 能拒绝它 | mutant error `< L`、mutant 被正常门接受，或 test 实际没有执行 mutant | 仅因 oracle/tool identity 无法复现而没有有效比较；不得因 mutant 结果难看而使用 |

**项目决策：**“mutant PASS”表示 rejection test 有区分力，不表示 mutant 实现正确。

### 8.2 Q2 整体结论

- **Q2 PASS**：正常 DUT 全部 PASS（包括 unused-accel invariance、三个自然 defensive
  runtime 门与 final-output defensive code audit），六类 mutant gates
  全部 PASS，RED 证据证明 tests 在 helper
  实现前失败，且 allowlist/deletion test 成立。
- **Q2 FAIL**：任何 deterministic 正常门、hard-fail、duration、SO(3) 或 mutant rejection
  失败。必须修合同/实现或记录负结果；不得进入 Q3。
- **Q2 INCONCLUSIVE**：仅当 fixed point/Q1 前提、CPython/mpmath 版本、80-dps oracle
  identity、编译工具链、必需一手 dependency 无法复现，或 verifier 平台不能由任一冻结
  自然 fixture 触发对应 branch，导致没有有效 observation；后一情形
  必须立即停止，不得临时改常量、注入错误或增加 seam。
  `inconclusive` 不能掩盖已经观测到的 deterministic failure；某条正常 DUT 已超门时，整体
  就是 FAIL，而不是“环境可能有差异”。

## 9. Fixed-seed noise 删除与未来 stochastic qualification

**项目决策：**从 Q2 删除 fixed-seed noise。固定随机种子只能让一次 stochastic sample 可
重放，不能证明 covariance propagation、whitening 或噪声校准正确；把它与 deterministic
quadrature 混在一起还会让失败无法归因。

**未授权范围：**gyro noise density、Monte Carlo distribution、covariance propagation、
PSD/condition、factor noise model、whitened residual 都属于未来独立资格片。该片必须另行
预注册 random generator/seed set、样本量、置信区间与 coverage；不得把本 Q2 的
deterministic PASS 外推为 stochastic propagation 已通过。

## 10. 建议的未来实施 allowlist / forbidden paths

本节只是给后续 Q2 plan review 的范围建议，不是当前实施授权。

### 10.1 建议 allowlist

```text
CMakeLists.txt
phad/estimator/gyro_rotation_predictor.hpp
phad/estimator/gyro_rotation_predictor.cpp
phad/estimator/README.md
tests/estimator/gyro_rotation_predictor_test.cpp
```

§7 的 shared-endpoint 输入必须作为 estimator test 内预构造 endpoint-aligned fixture；fixed
point 既有 sync tests 只作为上游前提，不把 sync test/source 加入 Q2 implementation
allowlist，也不让 Q2 test target 链接 sync。若 error/result type 能完全封装在
`gyro_rotation_predictor.hpp`，不要新增通用文件。

### 10.2 Forbidden paths / effects

```text
phad/estimator/stereo_vo_estimator.*
phad/estimator/types.hpp
phad/sync/stereo_pair_synchronizer.{hpp,cpp}
tests/sync/*
phad/sensor/*
phad/bench/*
phad/frontend/*
apps/*
scripts/*
tests/apps/*
docs/roadmap.md
任何 runtime config / config_hash / CLI / artifact schema
```

不得新增 `PreintegratedAhrsMeasurements`、AHRS/rotation factor、noise model、optimizer、
posterior state、MH_01 run 或 visual comparison。若实现发现必须改 forbidden path，说明
当前 seam/合同与仓库实际冲突，应停止并重新评审，而不是扩大 allowlist。

## 11. 后续实施的最小验收账本

后续独立 Q2 plan 至少应冻结并实际记录：

1. fixed point、clean status、GTSAM `4.3a0` identity 与 compiler/build identity；
2. CPython/mpmath oracle script/hash、版本、80 dps、完整 constants/output；
3. RED：helper 未实现时 Q2 test target 的预期 compile/link/test failure；
4. GREEN：只构建/运行 estimator 定向 unit tests；fixed point 既有 sync tests 仅记录为
   上游前提，Q2 不新增、修改、链接或运行 sync test，也不运行 MH_01 或长测试；
5. §5 正常数值、§6 mutant lower bounds、§4 hard-error matrix（含 unused-accel exact
   invariance、三个自然 defensive fixtures 的实际 code/optional fields、precondition finite
   checks、final output finite/orthogonality/determinant evidence 与 defensive code audit）、
   §7 三次调用 endpoint fixture 的逐项 actual；
6. `git diff --check`、受托 diff review、allowlist/forbidden-path audit 与 deletion test；
7. PASS/FAIL/INCONCLUSIVE 按 §8 判定，不以“测试进程 exit 0”替代逐门 evidence。

当前 helper/test/CMake implementation 与本次两个自然 fixture 的实施者定向结果仅为
provisional；独立 clean qualification 与 Q2 final PASS/FAIL/INCONCLUSIVE 仍 pending。本 note
不授予 Q3–Q5 权限。
