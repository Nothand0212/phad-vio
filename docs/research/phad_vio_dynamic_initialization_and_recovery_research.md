# phad-vio 正式动态视觉惯性初始化与失跟恢复技术调研

> 项目：[`Nothand0212/phad-vio`](https://github.com/Nothand0212/phad-vio)  
> 分支：`Nothand0212/m4-online-gyro-bias-synthetic-replay2`  
> 基准提交：`c999f5891d8efbc353e2e4001df7656a9e19907a`  
> 调研日期：2026-08-26  
> 目标：为下一步 C++ 实施提供可直接执行的初始化与视觉失跟恢复设计。

## 调研范围与结论标记

本报告针对 `phad-vio` 固定提交 `c999f5891d8efbc353e2e4001df7656a9e19907a` 的现有 `VioEstimator::update()`、PIMPL、raw IMU interval、transaction/rollback 和 segment 生命周期进行实施映射。EuRoC 数字按项目当前基线使用，本次没有重新运行 11 条序列。

固定提交中的 estimator、公开类型、offline session、双目同步器和 M5/M6 路线图共同构成本文的接口边界：

- [`phad/estimator/vio_estimator.cpp`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/estimator/vio_estimator.cpp)
- [`phad/estimator/types.hpp`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/estimator/types.hpp)
- [`apps/offline_vo_session.cpp`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/apps/offline_vo_session.cpp)
- [`phad/sync/stereo_pair_synchronizer.cpp`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/sync/stereo_pair_synchronizer.cpp)
- [`docs/roadmap.md`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/docs/roadmap.md)

文中使用以下标记：

- **[论文明确]**：论文明确描述的方法。
- **[源码行为]**：官方开源实现实际执行的路径。
- **[工程推断]**：基于论文、源码和 phad-vio 当前合同给出的实施建议。

---

# A. 一页式结论

## A.1 推荐路线

推荐采用：

> **VINS 风格的分阶段 metric stereo–IMU 对齐作为主求解器，加一个缩小版 ORB-SLAM3 风格的 inertial MAP refinement 和验收步骤。**

正式 moving initialization 数据流应为：

```text
metric stereo 多帧位姿
        ↓
常量 gyro bias 求解
        ↓
基于 raw IMU interval 全量 fresh reintegration
        ↓
每帧 velocity + gravity direction 线性求解
        ↓
重力 2-DOF tangent refinement
        ↓
acc bias 使用固定或继承先验
        ↓
小型 scratch GTSAM MAP refinement
        ↓
可观测性、残差、物理合理性、先验一致性验收
        ↓
一次性原子提交 X / V / B / gravity alignment / segment
```

其中：

- 冷启动和 visual-outage recovery 共用上述求解器。
- 二者只在 `InitializationPrior` 和坐标系处理上不同。
- 冷启动估计 gravity direction，保留 yaw gauge。
- recovery 在 IMU 连续时继承 gravity、bias 和传播后的 `NavState` 先验，并估计新视觉局部坐标系到旧 world frame 的桥接。
- static bootstrap 保留为独立快速路径。
- **moving 路径不得再退化到 mean(acc) + 零速度 + 零 bias。**

**[源码行为]** VINS-Mono 的初始化实现采用：

1. visual structure；
2. gyro bias；
3. PIM repropagation；
4. velocity、gravity、scale 线性对齐；
5. gravity tangent-space refinement。

参考：

- [VINS-Mono 论文](https://arxiv.org/abs/1708.03852)
- [VINS-Mono `initial_aligment.cpp` 固定提交](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/initial/initial_aligment.cpp)

**[论文明确 / 源码行为]** ORB-SLAM3 把视觉结构作为已知或固定部分，使用惯性约束估计 velocity、gravity、gyro bias、accelerometer bias；单目模式还估计 scale，stereo/RGB-D 模式 scale 固定。初始化后继续进行 full inertial BA/refinement。

参考：

- [ORB-SLAM3 论文](https://arxiv.org/abs/2007.11898)
- [ORB-SLAM3 `LocalMapping::InitializeIMU()`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc)
- [ORB-SLAM3 `Optimizer.cc`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc)

已知双目尺度后，VINS 的 scale 变量和 ORB-SLAM3 的 monocular scale 变量都应删除。

## A.2 为什么现在做这条路线

**[工程推断]** 当前最需要解决的是“错误或弱约束的初始 X/V/B 被提交”，而不是 optimizer 是否能运行：

- 所有序列 `estimator failed=0、rejected=0`，说明事务和 optimizer 没有显式失败。
- 但 MH_02、V2_02、V2_03 出现大量 segment、initializing frame 和 visual outage。
- 11 条平均 ATE 改善，但 7/11 RPE 上升，说明问题具有明显序列依赖，不像单一全局 noise 参数失配。
- 当前 moving bootstrap 将运动加速度混入重力方向，并把 velocity、gyro bias、acc bias 全部设零，恰好会造成“optimizer 能收敛，但从错误 basin 开始”。

VINS-Fusion 的 stereo+IMU 源码路径比 VINS-Mono 的 monocular 初始化简单：metric stereo 建图后先求 gyro bias、重积分，再进入 nonlinear optimization；其首姿态仍依赖平均加速度。因此它适合作为最低实现基线，但不能作为 phad-vio 正式动态初始化的终点。

参考：

- [VINS-Fusion 论文](https://arxiv.org/abs/1901.03638)
- [VINS-Fusion stereo+IMU 初始化分支](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp)
- [VINS-Fusion `initial_aligment.cpp`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/initial/initial_aligment.cpp)

## A.3 明确不建议现在做什么

当前不建议：

1. 不把 accelerometer bias 作为无先验自由变量直接塞进短窗口线性系统。
2. 不完整移植 ORB-SLAM3 初始化、OpenVINS DynamicInitializer 或 Basalt backend。
3. 不新增第二个 estimator、backend、wrapper 或第二条 update seam。
4. 不先调 IMU noise 或 root prior。
5. 不现在进入 fixed-lag marginalization。
6. 不通过延长 coast、放宽 optimizer 或 residual 阈值掩盖初始化失败。
7. 不在证据不足时用 mean(acc) fallback 冒充 moving initialization 成功。

ORB-SLAM3 的完整实现依赖 Atlas、Map、KeyFrame 和全局/局部 BA 生命周期；OpenVINS 动态初始化与其 feature-state、MSCKF/MLE 状态表示结合；Basalt 与自身 square-root marginalization 和 factor recovery 强绑定。直接搬运都会扩大当前 M5 范围。

## A.4 结论置信度

| 结论 | 置信度 |
|---|---:|
| `bg → reintegrate → g/v → MAP gate` 顺序 | 高 |
| 双目初始化不应再估计 scale | 高 |
| 首个 vertical slice 固定或继承 `b_a` | 高 |
| 冷启动与 recovery 共用求解器、使用不同先验 | 高 |
| 6–10 个 informative states、约 1–2 秒作为首轮工作区间 | 中 |
| 具体 condition、NIS、激励阈值 | 中低，必须由 synthetic replay 和 EuRoC 诊断确定 |

可能推翻当前路线的证据有三类：

- metric stereo pose 增量噪声大到不能作为初始化约束，需要提前引入联合 pose refinement；
- 时间同步、IMU 标定、相机到 body 外参存在系统误差，使所有可观窗口的 IMU residual 都呈结构性偏差；
- tracker 在恢复后给出的局部位姿不是一致刚体坐标系，无法由单个 `T_WV` 桥接。

---

# B. 成熟实现对比表

| 系统 | 初始化输入 | 求解变量 | 优化形式 | 所需时间/激励 | 失败判据 | Failure recovery | 对 phad-vio 适配成本 |
|---|---|---|---|---|---|---|---|
| **VINS-Mono** | monocular SFM poses、结构、IMU PIM | `b_g`、各帧 `v_i`、gravity、scale；初始化中不求 `b_a` | gyro bias LS；重积分；gravity/velocity/scale 线性 LS；重力切空间 refinement | 完整滑窗、足够视觉视差和 IMU 激励 | relative-pose/SFM/线性对齐失败、gravity/scale 不合理；运行期另有 failure detection | 清空 estimator 后重新进入同一初始化路径 | **低—中**：删除 SFM 和 scale 后，核心 `bg+g/v` 可直接借鉴。 |
| **VINS-Fusion stereo+IMU** | metric stereo poses/三角化结果、IMU | 源码先求 `b_g`，之后由 joint optimization 求 X/V/B；初始重力姿态仍依赖平均加速度 | `b_g` LS → PIM repropagate → nonlinear optimization | 填满窗口；没有单独正式 moving gravity/velocity linear alignment | 通用 optimizer/failure-detection 路径 | reset 后重新进入 INITIAL 路径 | **低**，但只适合作为最低基线，不能解决动态 mean(acc) 污染。 |
| **ORB-SLAM3** | 已建立 visual map/KF poses、IMU；stereo/RGB-D 已有 metric scale | 各 KF velocity、gravity direction、`b_g`、`b_a`；scale 仅 monocular | 固定视觉结构的 inertial-only MAP，随后 visual-inertial BA 和延迟 refinement | 秒级证据窗口、足够运动；源码中还有后续阶段性 refinement | KF/时间/运动不足、优化与物理检查失败；初始化后很快失跟会重置 active map | 重置或创建 active map，再重新初始化；不是轻量 NavState bridge | **中**：目标函数和 prior 策略适合，Atlas/Map 生命周期不适合移植。 |
| **OpenVINS** | feature tracks、IMU history、静止/动态初始化配置 | 动态 batch/MLE 状态，按选项可涉及姿态、速度、bias、部分标定量 | 显式 `StaticInitializer` 与 `DynamicInitializer`；动态路径为 batch/MLE | 可配置历史窗口和可观测性检查 | initializer 返回状态、条件和优化失败 | reset 后可再次调用 initializer；没有 phad 所需 segment bridge 合同 | **高**：架构思想值得借鉴，状态表示和 feature/MSCKF 耦合较重。 |
| **Kimera-VIO** | frontend bootstrap 状态、初始 pose/velocity/bias、IMU | backend X/V/B、landmarks | GTSAM factor graph；ImuFrontEnd 持有 bias-cached PIM，backend 从初始 state/prior 启动 | 依赖 frontend/bootstrap 和正常 backend 优化 | frontend/backend 状态和优化失败 | 主要依赖 pipeline 重启或重新 bootstrap；未提供独立动态对齐模块 | **中—高**：GTSAM/PIM 使用模式可参考，但初始化本身不是目标模板。 |
| **Basalt** | stereo/optical flow、IMU、初始 estimator state | pose、velocity、bias 等紧耦合状态 | 非线性 VIO + square-root marginalization / factor recovery | 依赖完整 estimator 生命周期 | backend-specific tracking/optimization 状态 | estimator reset/reseed，与 backend 耦合 | **高**：主要优势属于后续 marginalization/backend 数值架构，不是当前初始化 vertical slice。 |
| **OKVIS** | 多相机 keyframes、IMU、初始化 priors/state | poses、speed/bias、landmarks | keyframe-based nonlinear optimization | 依赖正常关键帧和 optimizer 收敛 | estimator/track/optimization 检查 | 通常重新建立 estimator 状态；没有独立 staged moving initializer | **高**：factor/prior 形式可参考，初始化代码直接复用价值低。 |

## B.1 对比结论

1. VINS-Mono 的顺序最适合成为 phad 的可诊断主解。
2. VINS-Fusion stereo 路径说明 metric stereo 下先校准 `b_g` 再进入 nonlinear graph 是成熟做法，但它没有解决正式 moving gravity/velocity 初始化。
3. ORB-SLAM3 最值得吸收的是：
   - 固定或强约束视觉结构；
   - bias prior；
   - velocity/gravity/bias 的 MAP refinement；
   - 初始化后继续做延迟 refinement；
   - 不把 optimizer 成功等同于初始化成功。
4. OpenVINS 最值得吸收的是静态/动态 initializer 共用统一输出和明确失败状态，而不是其 feature-state 实现。
5. Kimera、Basalt、OKVIS 更适合作为 GTSAM/PIM、backend prior 和后续 marginalization 的参考，不是当前 M5 主模板。

---

# C. 推荐算法的逐步数据流

## C.1 坐标、状态和 gauge

定义：

- `V`：当前 metric stereo visual local frame。
- `W`：gravity-aligned world frame。
- `B`：body frame；当前项目中 `B=I`，即 body frame 与 IMU frame 相同。
- `T^V_{B_i}=(R_i,p_i)`：第 `i` 个 evidence state 的 body pose，`R_i` 将 body 向量旋转到 `V`。
- `ΔR_ij, Δv_ij, Δp_ij`：区间 `[t_i,t_j]` 的 IMU preintegration，表达在 `B_i`。
- `g_0`：配置中的重力大小，不作为自由变量。

双目尺度已知后，最小初始化变量应为：

| 变量 | 是否必须求 | 建议 |
|---|---:|---|
| Gyroscope bias `b_g` | 是 | 初始化窗口内使用一个常量 3D bias |
| Gravity direction | 是，冷启动 | 2 DOF；大小固定为 `g_0` |
| 各 evidence state velocity `v_i` | 是 | 求每帧 velocity，而非只求 `v_0` |
| Accelerometer bias `b_a` | 首版不自由求解 | 冷启动固定为 prior；恢复继承 prior |
| Scale | 否 | 固定为 1 |
| Global yaw | 不可观 | 冷启动选择 gauge；恢复由旧世界先验约束 |
| Global translation | gauge | 冷启动令首帧位置为原点；恢复由旧世界先验约束 |

求每帧 `v_i` 而不是只求初始速度的原因是：它可以利用每个 interval 的 position/velocity equation 形成冗余，直接暴露异常区间，并避免把所有积分误差累积到单个 `v_0`。VINS 和 ORB-SLAM3 初始化都保留关键帧级 velocity 状态。

## C.2 输入缓存

每个 evidence node 至少包含：

```cpp
struct InitEvidenceNode {
  Timestamp timestamp;
  MetricBodyPose visual_pose;      // T_V_B，已经应用相机到 body 外参
  VisualQualitySummary visual_quality;
  RawImuInterval imu_from_previous;
};
```

要求：

1. 视觉 pose 必须先转换为 body pose，不能直接把 camera pose 当成 IMU pose。
2. 每个相邻 node 之间必须保存完整 raw IMU provenance。
3. interval 必须经过现有 endpoint normalization。
4. node 被移除时，必须复用现有 raw interval 拼接和 fresh reintegration 语义。
5. evidence buffer 只保存 pose summary、quality 和 raw interval，不建立第二个长期 factor graph。

建议首轮采用：

- 冷启动：**6–10 个 informative states，目标跨度约 1–2 秒**。
- 连续 IMU 的恢复：可以从 4–6 个 states 开始尝试，但只有完整通过信息矩阵和 prior-consistency gate 才能提前接受。
- hard restart：与冷启动相同。
- 窗口已满但不可观时：滑动 evidence，不得因为 timeout 强制成功。

这是 phad-vio 的首轮工程工作区间，不是论文中的通用物理阈值。

## C.3 多帧 stereo pose 增量估计 gyro bias

视觉相对旋转为：

\[
R^{vis}_{ij}=R_i^\top R_j
\]

定义 IMU rotation residual：

\[
r^R_{ij}(b_g)
=
\operatorname{Log}
\left(
\Delta R_{ij}(b_g)^\top R_i^\top R_j
\right)
\]

在当前 bias `b_g^0` 处线性化：

\[
r^R_{ij}(b_g^0+\delta b_g)
\approx
r^R_{ij}(b_g^0)
+
J^R_{b_g,ij}\delta b_g
\]

求解：

\[
\min_{\delta b_g}
\sum_{(i,j)}
\rho\left(
\left\|
r^R_{ij,0}
+
J^R_{b_g,ij}\delta b_g
\right\|_{\Sigma^{-1}_{R,ij}}^2
\right)
+
\left\|
b_g^0+\delta b_g-b_g^{prior}
\right\|_{P^{-1}_{bg}}^2
\]

其中：

- 冷启动的 `bg_prior` 来自配置标定或零均值宽先验。
- recovery 的 `bg_prior` 来自上一 committed segment，并按 bias random walk 和 outage 时间膨胀协方差。
- `ρ` 建议使用轻量 Huber/IRLS，防止一个视觉旋转异常污染整个 3D bias。
- 不要直接复制 VINS 中 quaternion `2*vec()` 的符号约定；应从当前 GTSAM 版本的 bias-corrected delta 或 `computeErrorAndJacobians()` 获取 Jacobian，封装在一个有单元测试的 helper 中。

VINS-Mono 源码使用 PIM rotation 对 gyro bias 的 Jacobian 构造小型正规方程，求出 `delta_bg` 后更新所有 bias，并对全部预积分执行 repropagation。

### 必须执行 fresh reintegration

每次接受新的 `b_g` 后：

1. 从 raw interval 重新构造 candidate PIM。
2. 使用统一的 candidate `b_g` 和当前 `b_a^{prior}`。
3. 再计算 rotation residual。
4. 允许少量外层迭代，直到 bias update 和 residual 改善收敛。
5. 最终验收必须使用 fresh-reintegrated PIM，不能只依赖大 bias correction 的一阶近似。

这可以直接复用 phad-vio 已有 raw interval provenance、eviction interval splice 和 predecessor fresh reintegration 合同。

## C.4 Gravity 与 velocity 线性系统

完成 gyro bias 校准并重新积分后，标准预积分关系为：

\[
p_j
=
p_i + v_i\Delta t
+\frac{1}{2}g_V\Delta t^2
+R_i\Delta p_{ij}
\]

\[
v_j
=
v_i+g_V\Delta t+R_i\Delta v_{ij}
\]

因此每个 interval 提供：

\[
\Delta t\,v_i+
\frac{1}{2}\Delta t^2 g_V
=
p_j-p_i-R_i\Delta p_{ij}
\]

\[
-v_i+v_j-\Delta t\,g_V
=
R_i\Delta v_{ij}
\]

第一轮线性状态为：

\[
x=
\begin{bmatrix}
v_0^\top &
v_1^\top &
\cdots &
v_N^\top &
g_V^\top
\end{bmatrix}^{\top}
\]

构造加权系统：

\[
\min_x \|Ax-b\|^2_W
\]

实施要求：

- 使用 PIM 中 position/velocity covariance 的联合块进行 whitening。
- 如果暂时没有可靠 visual pose covariance，应明确记录“视觉 pose 当作确定量”的模型近似，不能把 PIM-only covariance 解释为完整 uncertainty。
- 使用 QR/SVD 求解和检查 rank，避免通过 `AᵀA` 计算 condition number。
- 在 whitening 后再做列尺度归一化，否则 velocity、gravity 不同单位会使 condition number 无解释性。
- 不仅检查 solver success，还要记录最小奇异值、reciprocal condition number 和 residual。

VINS-Mono 的 `LinearAlignment` 同时求 velocity、gravity 和 monocular scale；phad-vio 应删除 scale 列，只保留 velocity 和 gravity。

### 重力大小和方向 refinement

先由线性系统得到 `g_V^{raw}`，然后：

1. 检查其未归一化模长是否与 `g_0` 在统计上相容。
2. 令 `ĝ=g_V^{raw}/||g_V^{raw}||`。
3. 构造与 `ĝ` 正交的 2D tangent basis `B(ĝ)∈R^{3×2}`。
4. 使用：

\[
g_V(\delta\theta)
\approx
g_0\hat g+B(\hat g)\delta\theta
\]

迭代求解 2D gravity correction。

VINS-Mono 源码使用相同的重力切空间思想，避免把 gravity 当作 unconstrained 3D vector 长期优化。

### 冷启动 world gauge

冷启动时：

- 令 `p^W_0=0`。
- 构造仅对齐 gravity 的 `R_WV`，将 `g_V` 旋转到配置的 `g_W`。
- 不额外估计 global yaw。
- 选择“零附加 yaw”或“保留第一视觉帧 yaw”的确定性 convention。
- 所有 `p_i、R_i、v_i` 一起变换到 `W`。

### Recovery gauge

若视觉重建在新局部坐标系 `V'` 中重新开始，并且 IMU 连续，则利用传播到首个恢复帧的先验：

\[
T_{WV'}^{0}
=
T_{WB_0}^{-}
\left(T^{V'}_{B_0}\right)^{-1}
\]

随后由整个 evidence window 的视觉相对约束和 IMU residual refinement。此时旧世界先验约束 translation 和 yaw，不再创建新的任意 yaw gauge。

## C.5 Accelerometer bias 如何处理

### 结论

首个 vertical slice：

- **moving cold start：固定 `b_a=b_a^{prior}`。**
- 如果没有离线标定，零只能称为“先验均值”，不能称为“已知真值”。
- **continuous-IMU recovery：继承上一 committed `b_a`，并按 random walk 与 outage 时间膨胀 covariance。**
- 初始 linear alignment 不自由求解 `b_a`。
- 在线 active graph 继续使用现有 bias random-walk factor 逐步估计。
- 后续只有在明确通过 accelerometer-bias observability gate 后，才可在 MAP refinement 中加入小量 `δb_a`。

### 原因

若 orientation 几乎不变，accelerometer bias 在 world frame 中表现为近似固定加速度：

\[
R_i b_a \approx R b_a
\]

它与 gravity correction 在 position/velocity equation 中几乎同列，短窗口内无法可靠区分。只有足够多轴旋转和 specific-force 变化，`R_i b_a` 才会随时间变化，从而和 gravity 分离。

VINS-Mono 的初始化线性状态没有 accelerometer bias，初始化重积分显式使用零 `ba`；ORB-SLAM3 的 MAP 初始化允许估计 `ba`，但同时引入 bias prior，并在初始化后继续 refinement。二者结合说明：`ba` 可以在成熟 MAP 中估计，但不应在弱激励短窗口内无先验自由求解。

### 可选 `ba` refinement 的 observability gate

后续若加入 `δb_a`，应检查消去 velocity/gravity 后的条件信息矩阵：

\[
H_{ba\mid rest}
=
H_{ba,ba}
-
H_{ba,r}
H_{r,r}^{-1}
H_{r,ba}
\]

只有以下条件同时成立时，才接受 `ba` refinement：

- `H_ba|rest` 满秩；
- reciprocal condition 足够；
- gravity norm 未被 `ba` 人为修正；
- `ba` update 在 prior Mahalanobis 范围内；
- MAP cost 和 normalized residual 明显改善。

否则保持 prior。

## C.6 小型 MAP refinement

推荐先完成 staged seed，再建立一个**局部 scratch graph**。它只是 `update()` 内部临时值对象，不是第二个 backend。

### 推荐状态

首版：

\[
\mathcal Z=
\{X_0,\ldots,X_N,\;
V_0,\ldots,V_N,\;
B_{init}\}
\]

其中：

- `X_i` 由 gravity-aligned metric stereo poses 初始化。
- `V_i` 由线性系统初始化。
- 所有 interval 共享一个 `B_init`，表示初始化窗口内常量 bias。
- gravity 已由外层 2-DOF refinement 固定，不在 GTSAM graph 中另建变量。
- 冷启动时 `ba` 固定或使用强 prior。
- recovery 时增加 first NavState 和 bias 的传播先验。

### 因子

可以直接复用：

- 当前视觉 relative-pose factor 或已有视觉约束；
- `gtsam::ImuFactor`；
- `gtsam::PriorFactor<gtsam::Pose3>`；
- `gtsam::PriorFactor<gtsam::Vector3>`；
- `gtsam::PriorFactor<gtsam::imuBias::ConstantBias>`；
- `gtsam::NavState`；
- `gtsam::PreintegratedImuMeasurements`；
- `gtsam::NonlinearFactorGraph`；
- `gtsam::Values`；
- 当前已经使用的 optimizer。

GTSAM 官方文档：

- [`NavState`](https://borglab.github.io/gtsam/navstate/)
- [`PreintegratedImuMeasurements`](https://borglab.github.io/gtsam/preintegratedimumeasurements/)
- [`ImuFactor`](https://borglab.github.io/gtsam/imufactor/)
- [`gtsam/navigation/ImuFactor.h`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/ImuFactor.h)

`NavState` 表示 pose 和 velocity；`ImuFactor` 连接前后 pose/velocity 以及 bias；bias 演化通常由单独 factor 表示。当前初始化窗口使用一个共享 bias，接受后再广播到各个 `B(k)`，并恢复现有 bias random-walk factors。

目标函数可写为：

\[
\begin{aligned}
J=&
\sum_{ij}\rho_v(\|r^{vis}_{ij}\|^2_{\Sigma_{vis}^{-1}})
+
\sum_{ij}\|r^{imu}_{ij}\|^2_{\Sigma_{imu}^{-1}}\\
&+
\|B_{init}-B^{prior}\|^2_{P_B^{-1}}
+
\mathbb{1}_{recovery}
\|X_0-X_0^{-}\|^2_{P_X^{-1}}\\
&+
\mathbb{1}_{recovery}
\|V_0-V_0^{-}\|^2_{P_V^{-1}}
\end{aligned}
\]

第一版不需要新增自定义 `StereoInitializationFactor`。如果现有视觉约束是 relative pose factor，直接复用；如果只有绝对 stereo pose，可使用有限 covariance 的 Pose3 priors。不要用未经验证的极小协方差把 pose “钉死”。

### MAP refinement 不是 acceptance 本身

optimizer 返回成功只说明找到局部极值。仍需检查：

- cost 是否下降；
- whitened rotation/position/velocity residual；
- bias update；
- gravity consistency；
- velocity；
- recovery prior consistency；
- candidate 是否导致很快再次 outage。

## C.7 Evidence window 与可观测性要求

以下阈值策略比“固定旋转 10°、平移 0.1 m”更可靠：

| 指标 | 作用 |
|---|---|
| `H_bg=ΣJ_RᵀW_RJ_R` 的最小特征值、rcond | gyro bias 是否可观 |
| whitened/scaled `A_gv` 的最小奇异值、rcond | gravity/velocity 是否可解 |
| accumulated rotation | 解释弱 `bg` 信息 |
| rotation-axis spread | 识别单轴退化 |
| metric translation / parallax | 识别无位置约束 |
| specific-force covariance | 识别匀速或弱动态 |
| visual pose quality/inlier ratio | 防止把视觉离群当 IMU bias |
| residual before/after `bg` reintegration | 检查 bias 解是否真正改善模型 |

阈值制定方式：

1. 先把上述指标作为 log-only diagnostics。
2. 用 synthetic replay 覆盖静止、纯平移、纯单轴旋转、匀速、充分激励。
3. 找到退化样本和可接受样本在 rcond/NIS 上的分界。
4. 再将阈值写入现有 estimator options。
5. 不直接复制 VINS、ORB 或 OpenVINS 的数字，因为其状态尺度、相机模型、窗口和 noise 不同。

## C.8 Rejection、继续累积和 hard clear

| 结果 | 条件 | 对状态的处理 |
|---|---|---|
| **Continue accumulating** | node/时间不足；`bg` 或 `g/v` 信息弱但数据合法；视觉质量暂时不足；窗口仍可增加信息 | 保留或按现有 eviction 策略滑动 evidence；不提交 X/V/B |
| **Reject candidate / slide** | 某视觉 interval residual 明显离群；重积分后 rotation residual 不改善；MAP cost 不下降；bias 或 velocity 与 prior/物理范围不一致 | 丢弃可疑最新 node 或最老低信息 node；保留合法历史；重新积累 |
| **Hard clear** | timestamp regression；IMU sensor epoch/reset 未知；标定、单位或外参变化；raw provenance 破坏；interval endpoint 不可构造；显式 session reset | 清空 evidence、candidate PIM、传播 NavState 和动态 prior；只保留静态配置 |
| **Accept** | 数据完整、信息矩阵通过、物理检查通过、MAP 收敛、normalized residual 通过、recovery prior 一致 | 一次性安装 X/V/B、gravity/world transform 和 segment |

绝对 bias、velocity 和 residual 限制不应使用论文中的魔法数字：

- bias 应以 sensor 配置和 prior Mahalanobis distance 检查；
- residual 应使用 NIS/whitened residual；
- velocity 上限应来自机器人平台运动范围；
- gravity 原始模长应按估计 covariance 检查，而不是简单复制 `9～10 m/s²`。

## C.9 Visual outage 后恢复

### 情况 1：短 outage，IMU 连续，视觉坐标系未重置

- outage 不超过现有 500 ms coast 合同；
- tracker 恢复后仍在同一视觉/world frame；
- 继续现有 segment；
- 不重新运行完整 initializer；
- 使用 coast 后传播的 X/V/B 继续优化。

### 情况 2：视觉局部地图重置，但 IMU 连续

- 进入 `moving_recovery`。
- 创建 pending segment，但不立即公开提交。
- 继承：
  - `b_g`：继承均值，按 random walk 膨胀 covariance；
  - `b_a`：同上；
  - gravity：作为旧 world frame 中固定量或强先验；
  - velocity：由最后 committed NavState 通过完整 raw IMU 传播到恢复帧；
  - attitude/yaw：作为有 covariance 的传播先验。
- 新视觉局部系通过 `T_WV'` 与旧世界桥接。
- evidence window 通过完整 acceptance 后才原子提交新 segment。

### 情况 3：超过 500 ms，但 raw IMU 完整连续

500 ms 是输出 coast 合同，不等于 IMU 物理连续性失效：

- 可以停止公开 coast 输出；
- 仍可利用完整 IMU 将旧状态传播到恢复时间；
- 传播 covariance 会随时间增长；
- 新 visual segment 接受后仍可保持 world-frame 连续性。

### 情况 4：IMU 数据存在缺口或 epoch 不可靠

- 不能继承传播后的 pose/velocity。
- 如果确认传感器未重启，只是少量样本缺失，可将 bias 作为非常弱 prior，但不能声称 NavState 连续。
- timestamp 回退、sensor restart、单位/标定变化、extrinsic 变化必须完全冷启动。

VINS 和 ORB-SLAM3 官方实现主要通过 estimator/map reset 再次进入初始化；phad-vio 的 raw IMU provenance 和 transaction 使其有条件在明确 IMU 连续时保留带 covariance 的动态先验，而不是无条件清零。

---

# D. phad-vio 实施映射

## D.1 文件级修改

| 文件 | 建议修改 |
|---|---|
| `phad/estimator/vio_estimator.cpp` | 在 `Impl` 内增加 evidence accumulator、mode-specific prior、staged solver、candidate evaluator 和 atomic commit；复用当前 PIM、transaction、window 和 factor 创建逻辑 |
| `phad/estimator/types.hpp` | 增加公开 POD diagnostics、enum 和 rejection reason；不得出现 GTSAM/PIM 类型 |
| `apps/offline_vo_session.cpp` | 输出初始化 mode/phase/reason、observability、residual、inheritance、candidate/segment 信息；不加入初始化决策逻辑 |
| `phad/sync/stereo_pair_synchronizer.cpp` | 初始化算法不应进入同步器；最多补充 paired/left-only/right-only/timestamp-delta counters |
| `docs/roadmap.md` | 将 M5 拆成 staged initialization、MAP gate、recovery 和定向验证；保持 formal marginalization 在 M6 |

## D.2 最小私有状态

```cpp
enum class InitializationMode {
  kStaticCold,
  kMovingCold,
  kMovingRecovery,
};

enum class InitializationPhase {
  kAccumulating,
  kSolvingGyroBias,
  kSolvingGravityVelocity,
  kRefining,
  kAccepted,
};

struct InitializationPrior {
  InitializationMode mode;

  gtsam::imuBias::ConstantBias bias_mean;
  BiasCovarianceSummary bias_covariance;

  std::optional<gtsam::NavState> propagated_state;
  NavStateCovarianceSummary propagated_covariance;

  Eigen::Vector3d gravity_world;
  bool has_valid_world_frame = false;
};

struct InitializationCandidate {
  std::vector<gtsam::Pose3> poses;
  std::vector<gtsam::Vector3> velocities;
  gtsam::imuBias::ConstantBias bias;
  Eigen::Vector3d gravity;

  CandidateQuality quality;
};

struct InitializationEvidence {
  std::deque<InitEvidenceNode> nodes;
  std::uint64_t candidate_id = 0;
  std::uint32_t attempt_count = 0;
};
```

这些类型全部位于 `VioEstimator::Impl` 或 `.cpp` 私有区域。

## D.3 最小私有 helper

```cpp
void appendInitializationEvidence(...);

InitializationPrior buildInitializationPrior(
    InitializationMode mode,
    Timestamp target_time) const;

GyroBiasSolveResult solveInitializationGyroBias(
    const InitializationEvidence& evidence,
    const InitializationPrior& prior) const;

CandidatePimSet reintegrateInitializationIntervals(
    const InitializationEvidence& evidence,
    const gtsam::imuBias::ConstantBias& bias) const;

GravityVelocitySolveResult solveGravityAndVelocities(
    const InitializationEvidence& evidence,
    const CandidatePimSet& pims,
    const InitializationPrior& prior) const;

RefinementResult refineInitializationCandidate(
    const InitializationEvidence& evidence,
    const InitializationPrior& prior,
    InitializationCandidate seed) const;

InitializationDecision evaluateInitializationCandidate(
    const InitializationEvidence& evidence,
    const InitializationPrior& prior,
    const InitializationCandidate& candidate) const;

void commitInitializationCandidate(
    InitializationCandidate candidate,
    VioUpdateTransaction& transaction);

void slideInitializationEvidence(InitSlideReason reason);

void clearInitializationEvidence(HardDiscontinuity reason);
```

不要增加：

```text
DynamicVioEstimator
RecoveryEstimator
InitializationBackend
InitializationWrapper
updateForRecovery()
updateForInitialization()
```

## D.4 可以直接复用的代码

应复用而不是重写：

- raw IMU endpoint normalization；
- raw interval provenance；
- interval splice；
- fresh PIM reintegration；
- predecessor bias 语义；
- `X(k)/V(k)/B(k)` key 创建；
- `ImuFactor`；
- bias random-walk factor；
- root prior 构造；
- 当前 optimizer；
- window node 创建和 eviction；
- transaction snapshot/rollback；
- segment/discontinuity 管理。

GTSAM PIM、`NavState` 和 `ImuFactor` 已覆盖 candidate state propagation、bias correction、residual 和 Jacobian 计算；当前工作不需要 `IncrementalFixedLagSmoother` 或新的 marginalization API。

## D.5 Transaction / rollback 语义

需要区分两种“成功”。

### 证据累积成功

当前 stereo/IMU 输入合法，但证据不足：

- evidence append 可以作为一次合法 `update()` 提交；
- active X/V/B 不发生变化；
- 返回 `kInitializing`；
- `estimator rejected` 不增加；
- diagnostics 给出 `kNeedMoreEvidence`。

### Candidate 初始化成功

只有所有 gate 通过后：

1. 在 scratch 数据结构中完成全部 PIM、X/V/B 和 graph refinement。
2. 创建新的 active-window 状态。
3. 安装 gravity/world alignment。
4. 安装所有 active PIM/factors。
5. 安装 segment id 和 lifecycle 状态。
6. 一次 transaction commit。

任何一步异常：

- active graph、X/V/B、segment、raw interval ownership 必须保持调用前状态；
- candidate scratch 直接销毁；
- 不允许部分写入 bias、gravity 或 root pose。

Candidate 被正常拒绝不等于 transaction error；它是合法的 initializing/recovery 状态。

## D.6 最小 public diagnostics

推荐在 `types.hpp` 增加稳定 POD：

```cpp
enum class InitializationModeDiag {
  kNone,
  kStaticCold,
  kMovingCold,
  kMovingRecovery,
};

enum class InitializationStatus {
  kAccumulating,
  kCandidateRejected,
  kAccepted,
  kHardReset,
};

enum class InitializationReason {
  kNone,
  kNeedMoreFrames,
  kNeedMoreDuration,
  kInsufficientGyroInformation,
  kInsufficientGravityVelocityInformation,
  kVisualPoseInconsistent,
  kInvalidImuInterval,
  kGyroBiasPriorViolation,
  kGravityInconsistent,
  kVelocityInconsistent,
  kResidualTooLarge,
  kNonlinearRefinementFailed,
  kRecoveryPriorInconsistent,
  kHardDiscontinuity,
};
```

公开 scalar diagnostics：

| 类别 | 字段 |
|---|---|
| Evidence | keyframe/node count、duration、IMU interval/sample count |
| Visual excitation | accumulated rotation、axis-spread summary、translation、visual quality |
| Observability | `gyro_min_sv`、`gyro_rcond`、`gravity_velocity_min_sv`、`gravity_velocity_rcond` |
| Bias | prior source、`bg_update_norm`、`ba_update_norm`、是否 inherited/refined |
| Gravity/velocity | raw gravity norm、tilt update、initial/max speed |
| Residual | rotation/position/velocity normalized RMS、initial/final MAP cost |
| Lifecycle | outage duration、candidate id、attempt count、pending/committed segment id |
| Decision | mode、status、primary reason、reason bitmask |

不公开：

- `gtsam::Values`；
- factor 指针；
- PIM；
- Hessian/Jacobian 矩阵；
- GTSAM key；
- optimizer 对象；
- raw covariance matrix。

---

# E. 首轮验证矩阵

## E.1 Public seam 单元测试

| 测试 | 注入条件 | 必须观察到的公开行为 |
|---|---|---|
| 1. Moving metric nominal | 已知 stereo poses、非零 `bg`、`ba=prior`、充分 3D 运动 | 正确接受；X/V/B 接近真值；segment 原子建立 |
| 2. Zero rotation | 有平移、无有效旋转 | `gyro_rcond` 低；持续 accumulating；不得用零 `bg` 强制成功 |
| 3. Pure single-axis rotation | 无足够 translation/specific-force 变化 | 可得到部分 `bg` 信息，但 `g/v` gate 不通过 |
| 4. Constant velocity | metric translation 近似匀速 | velocity 可解但 `ba` 不可自由 refine；不得伪造高置信 `ba` |
| 5. Moving acceleration contaminates mean(acc) | 运动加速度明显 | 动态路径不使用 mean(acc)；gravity 由 multi-frame equation 得到 |
| 6. Visual rotation outlier | 单个 stereo increment 被扰动 | candidate reject/slide；active X/V/B 不变 |
| 7. Raw interval endpoint 缺失 | 缺首/尾 endpoint 或 timestamp 非单调 | hard reject/clear；transaction 完整 rollback |
| 8. Recovery with continuous IMU | 视觉 local frame 重置，IMU 连续 | 继承 bias/gravity/velocity prior；估计 world bridge；原子提交新 segment |
| 9. Recovery with sensor epoch reset | timestamp 或 IMU epoch 变化 | 所有动态 prior 清空；进入 cold initialization |
| 10. Repeated weak candidates | 多次窗口不可观，随后出现充分运动 | 前期不误接受；后期使用滑动 evidence 正常接受 |
| 11. Candidate optimizer exception | 人工抛出异常或生成 non-finite state | 输出和内部状态与调用前一致 |
| 12. Evidence eviction | 删除中间 non-keyframe | raw intervals 正确拼接，并用 candidate predecessor bias fresh reintegrate |

最重要的 acceptance 测试不是“估计误差足够小”，而是：

> 所有退化 synthetic case 中，**false accept 必须为 0**。

## E.2 定向 EuRoC 矩阵

### MH_01：static bootstrap 回归基线

当前：

- ATE 0.07054 m；
- completion 99.97%；
- segments=1。

检查：

- 首次启动仍选择 static path；
- dynamic initializer 不被误触发；
- mean(acc)/mean(gyro) 静止结果不回归；
- segment 保持 1；
- completion 不下降；
- 首个 X/V/B 和 gravity 不出现明显跳变。

建议再加一条起始阶段较静止的 MH 序列作为交叉检查，避免只针对 MH_01 过拟合。

### MH_02：优先定位初始化/恢复问题

当前：

- ATE 0.29054 m；
- segments=6；
- 旧 VO ATE 0.08937 m、segments=1。

这是首轮最重要序列，因为视觉 VO 基线本身较好。

重点看：

- 每个 segment 的 initialization mode；
- candidate attempt 数；
- acceptance latency；
- `bg/gv rcond`；
- commit 后 1 秒内的 IMU residual；
- `time_to_next_visual_outage`；
- segment 是否因错误 initialization 很快再次中断。

判别：

- 若 formal initializer 后 segment 显著减少，说明旧 moving bootstrap/recovery 是主因。
- 若 candidate 质量很好但视觉仍频繁 outage，则转向视觉 tracking。
- 若 good visual + good observability 下 normalized IMU residual 系统性偏高，才怀疑 noise、时间同步或标定。

### V2_02：区分初始化与动态视觉 tracking

当前：

- ATE 1.98198 m；
- segments=9；
- 旧 VO ATE 0.85285 m、segments=1。

检查：

- 强运动时 `bg` 是否真正可观；
- visual pose increment 是否有离群；
- candidate 被拒绝的主因是视觉质量还是 IMU residual；
- accepted candidate 的 gravity tilt、velocity 和 bias 是否合理；
- outage 前是否先出现 visual inlier/pose-quality 下降。

判别：

- initialization observability 和 residual 均好，但仍失跟：视觉 tracking/blur 主导。
- gyro solve 在视觉旋转 residual 上不稳定：视觉 rotation increment 或外参问题。
- 多个良好视觉窗口均出现一致的 IMU normalized residual 偏大：IMU noise/标定候选。

### V2_03：生命周期压力测试

当前：

- ATE 1.70751 m；
- completion 50.65%；
- segments=52；
- initializing frame=896；
- visual-outage frame=52。

先不要以 ATE 为唯一目标。应先把 896 个 initializing frame 分解为：

```text
need_more_frames
need_more_duration
insufficient_gyro_information
insufficient_gravity_velocity_information
visual_pose_inconsistent
recovery_prior_inconsistent
candidate_refinement_failed
hard_discontinuity
```

重点指标：

- initializing 累积时间占比；
- 每次 outage 后恢复成功率；
- recovery acceptance latency；
- repeated candidate reject 次数；
- false accept 后短时间再次 outage 的次数；
- pending segment 与 committed segment 数；
- completion。

关于同步：

- `cam0=1922`；
- 共同 timestamp=1921；
- 因而最多只有 1 个 left frame 因缺少 right 无法组成共同 timestamp stereo pair。
- 415 个 right-only frame 本身不能解释 896 个 initializing frame。
- 仍应通过 synchronizer diagnostics 检查 common timestamp 是否被完整消费，但不应把 right-only 数量当作首要修复目标。
- 同步逻辑应继续局限在 synchronizer，不进入 initializer。

## E.3 值得跑 11 条全表的门槛

满足以下条件后再跑全表：

1. synthetic 退化测试 false accept=0。
2. 所有 transaction/rollback 和 raw interval fault-injection 测试通过。
3. MH_01 static path、segment 和 completion 无回归。
4. MH_02、V2_02、V2_03 中至少两条出现：
   - initializing frame 减少；
   - segment 数减少；
   - recovery latency 减少；
   - commit 后短时再次失跟减少。
5. 三条定向序列均没有明显 completion 回退。
6. ATE/RPE 没有被“更少 segment 但错误桥接”显著恶化。
7. diagnostics 能解释每次 candidate 未成功的原因。

不建议先设任意“ATE 必须提高 20%”阈值。先重复运行当前 commit 得到 deterministic/run-to-run variation，再定义实际回归容限。

## E.4 三类问题的诊断签名

| 主要问题 | 典型 diagnostics |
|---|---|
| 初始化问题 | good visual quality；candidate `bg/gv` 条件差或 residual 高；commit 后很快再次 outage；gravity/bias update 大 |
| 视觉 tracking 问题 | initializer 条件和 residual 良好；accepted state 连续；outage 前 visual inlier、parallax、pose-quality 明显下降 |
| IMU noise/标定问题 | 多条序列、多个 good-visual 且 well-conditioned window 中，whitened IMU residual 一致偏大；bias 被持续推向 prior 边界；误差与运动方向相关 |

---

# F. 风险排序

| 排序 | 风险类别 | 主要失败方式 | 最小验证 |
|---:|---|---|---|
| 1 | **Correctness** | camera/body pose 方向错误；gravity 符号错误；PIM residual block 顺序错误；endpoint 或 bias predecessor 错误；candidate 部分提交 | 无噪声 synthetic exact recovery；frame-convention 单元测试；PIM equation closure；fault injection 后状态 bitwise/semantic equality |
| 2 | **Observability** | 无旋转、单轴旋转、匀速时误接受；`ba` 吸收 gravity；timeout 强制成功 | 退化运动矩阵；whitened SVD/rcond；false-accept=0；窗口满但不可观时持续 accumulating |
| 3 | **数值稳定性** | 直接解 normal equation；单位尺度导致 condition 失真；gravity unconstrained；强 Pose prior 导致 LM 病态 | QR/SVD 与 truth 对比；列尺度测试；finite-difference Jacobian；不同初值重复求解 |
| 4 | **生命周期** | outage 和 hard discontinuity 混淆；错误继承 bias/velocity；segment 在 candidate 未验收时提前发布 | 短 outage、长但连续 IMU、IMU gap、timestamp reset、calibration change 五类状态机测试 |
| 5 | **性能** | 每帧重复全量 reintegration 或多次 scratch optimization；候选缓存复制过多 | 10-node 上限 benchmark；记录 reintegration 次数、solver iterations、p50/p95 `update()` latency |

性能风险排在最后：10 个 evidence states、若干小型 SVD 和一次 bounded scratch optimization 的规模很小。当前首先应确保不能错误提交。

---

# G. 来源清单

## G.1 论文

1. **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**  
   <https://arxiv.org/abs/1708.03852>

2. **VINS-Fusion: A General Optimization-based Framework for Multi-sensor State Estimation**  
   <https://arxiv.org/abs/1901.03638>

3. **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**  
   <https://arxiv.org/abs/2007.11898>

4. **OpenVINS: A Research Platform for Visual-Inertial Estimation**  
   DOI: <https://doi.org/10.1109/ICRA40945.2020.9196524>  
   官方文档：<https://docs.openvins.com/>

5. **Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping**  
   <https://arxiv.org/abs/1910.02490>

6. **Basalt: Visual-Inertial Mapping with Non-Linear Factor Recovery**  
   <https://arxiv.org/abs/2109.09653>

7. **OKVIS: Keyframe-Based Visual-Inertial Odometry Using Nonlinear Optimization**  
   <https://doi.org/10.1177/0278364914554813>

## G.2 phad-vio 固定提交源码

以下均固定到 `c999f5891d8efbc353e2e4001df7656a9e19907a`：

- [`phad/estimator/vio_estimator.cpp`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/estimator/vio_estimator.cpp)
- [`phad/estimator/types.hpp`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/estimator/types.hpp)
- [`apps/offline_vo_session.cpp`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/apps/offline_vo_session.cpp)
- [`phad/sync/stereo_pair_synchronizer.cpp`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/sync/stereo_pair_synchronizer.cpp)
- [`docs/roadmap.md`](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/docs/roadmap.md)

## G.3 官方仓库初始化源码

### VINS-Mono

- [`initial_aligment.cpp`](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/initial/initial_aligment.cpp)
- [`estimator.cpp`](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/estimator.cpp)

### VINS-Fusion

- [`initial_aligment.cpp`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/initial/initial_aligment.cpp)
- [`estimator.cpp`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp)

### ORB-SLAM3

- [`LocalMapping.cc`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc)
- [`Optimizer.cc`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc)
- [`Tracking.cc`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc)

### OpenVINS

- [`Initializer.cpp`](https://github.com/rpng/open_vins/blob/master/ov_init/src/Initializer.cpp)
- [`DynamicInitializer.cpp`](https://github.com/rpng/open_vins/blob/master/ov_init/src/dynamic/DynamicInitializer.cpp)
- [`DynamicInitializerOptions.h`](https://github.com/rpng/open_vins/blob/master/ov_init/src/dynamic/DynamicInitializerOptions.h)

### Kimera-VIO

- [`ImuFrontEnd.cpp`](https://github.com/MIT-SPARK/Kimera-VIO/blob/master/src/imu-frontend/ImuFrontEnd.cpp)
- [`VioBackend.cpp`](https://github.com/MIT-SPARK/Kimera-VIO/blob/master/src/backend/VioBackend.cpp)
- [`InitializationBackend.cpp`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/InitializationBackend.cpp)
- [`InitializationFromImu.cpp`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/InitializationFromImu.cpp)

### Basalt / OKVIS

- [Basalt 官方仓库](https://gitlab.com/VladyslavUsenko/basalt)
- [OKVIS 官方仓库](https://github.com/ethz-asl/okvis)

## G.4 GTSAM 官方文档

- [`NavState`](https://borglab.github.io/gtsam/navstate/)
- [`PreintegratedImuMeasurements`](https://borglab.github.io/gtsam/preintegratedimumeasurements/)
- [`ImuFactor`](https://borglab.github.io/gtsam/imufactor/)
- [`gtsam/navigation/ImuFactor.h`](https://github.com/borglab/gtsam/blob/develop/gtsam/navigation/ImuFactor.h)

---

# 最终实施顺序

建议严格按以下顺序推进：

1. 增加 initialization diagnostics，不改变算法。
2. 实现 evidence buffer 和 `InitializationPrior/InitializationCandidate`。
3. 实现多帧 gyro bias solve。
4. 基于 raw intervals fresh reintegrate。
5. 实现 metric stereo gravity/velocity linear alignment。
6. 固定或继承 accelerometer bias。
7. 加入 observability、residual 和物理 acceptance gate。
8. 加入 scratch GTSAM refinement。
9. 加入 continuous-IMU recovery prior 和 visual-world bridge。
10. 完成 synthetic public-seam 测试。
11. 定向运行 MH_01、MH_02、V2_02、V2_03。
12. 只有在 well-conditioned、good-visual 窗口仍表现出系统性 normalized IMU residual 失配时，才调 IMU noise。
13. 只有 gauge/root diagnostics 明确指向 prior conditioning 时，才调整 root prior。
14. 全 11 序列验证稳定后，再进入 M6 formal fixed-lag marginalization。

## 明确结论

当前不应先调 IMU noise，不应先调 root prior，也不应进入 fixed-lag marginalization。

初始化错误会污染 residual，使 noise 调优失去统计意义；marginalization 又会固定已有错误线性化点并放大 recovery 生命周期复杂度。应先建立“可观测、可拒绝、可回滚、可诊断”的正式 initialization transaction，才能保持后续参数调优和 M6 的因果可归因性。
