# 坐标系、时间与单位合同

本文档是 `phad-vio` 中所有传感器适配器、前端、预积分和后端优化共同
遵守的合同。代码与本文档不一致时，应先判断是实现错误还是合同需要通过
ADR 修改，不能只在局部添加转换来掩盖不一致。

## 1. 记号

使用

\[
{}^A\mathbf T_B =
\begin{bmatrix}
{}^A\mathbf R_B & {}^A\mathbf p_B \\
\mathbf 0^\top & 1
\end{bmatrix}
\]

表示把在坐标系 \(B\) 中表达的点转换到坐标系 \(A\)：

\[
{}^A\mathbf p = {}^A\mathbf T_B\,{}^B\mathbf p
\]

因此，\({}^W\mathbf T_B\) 表示 body/IMU 在 world 中的位姿，而不是
world 在 body 中的位姿。

复合与求逆遵守：

\[
{}^A\mathbf T_C = {}^A\mathbf T_B\,{}^B\mathbf T_C
\]

\[
{}^B\mathbf T_A = ({}^A\mathbf T_B)^{-1}
\]

## 2. 坐标系

### 2.1 World：\(W\)

第一版采用局部重力对齐的右手世界坐标系：

- \(+Z\) 向上；
- 重力向量为
  \[
  {}^W\mathbf g = [0, 0, -g]^\top
  \]
- 默认标准重力大小 \(g=9.81\ \mathrm{m/s^2}\)，实际数据集可由配置覆盖；
- 原点和 yaw 由初始化时的第一个 body 状态确定。

这是 ENU 风格的局部坐标约定，但第一版不声称 world 已与地理 east/north
方向对齐。

### 2.2 Body/IMU：\(B\)

\(B\) 固连于 IMU。后端的主 pose 状态始终是
\({}^W\mathbf T_{B_k}\)。

IMU 标定文件必须明确原始传感器轴到 \(B\) 的变换。传感器适配器负责在
数据进入核心模块前完成该转换；核心模块不接受“沿用数据集原始轴但未
标注”的测量。

### 2.3 左、右相机：\(C_l, C_r\)

相机坐标遵循常见针孔模型约定：

- \(+X\) 向图像右方；
- \(+Y\) 向图像下方；
- \(+Z\) 沿光轴向前。

固定外参保存为：

\[
{}^B\mathbf T_{C_l},\quad {}^B\mathbf T_{C_r}
\]

因此：

\[
{}^W\mathbf T_{C_l}
  = {}^W\mathbf T_B\,{}^B\mathbf T_{C_l}
\]

外参的存储方向不得由变量名外的注释或调用上下文推断。配置读取完成后，
必须通过一个可见的规范化步骤转换为上述方向。

## 3. 状态定义

关键帧 \(k\) 的最小后端状态为：

\[
\mathcal X_k =
\left\{
{}^W\mathbf T_{B_k},
{}^W\mathbf v_{B_k},
\mathbf b_k^a,
\mathbf b_k^g
\right\}
\]

其中：

- pose：GTSAM `Pose3`，语义为 \({}^W\mathbf T_{B_k}\)；
- velocity：body 原点的速度，以 \(W\) 表达，单位 \(\mathrm{m/s}\)；
- accelerometer bias：以 \(B\) 表达，单位 \(\mathrm{m/s^2}\)；
- gyroscope bias：以 \(B\) 表达，单位 \(\mathrm{rad/s}\)。

GTSAM key 约定：

```text
X(k) -> Pose3
V(k) -> Vector3
B(k) -> imuBias::ConstantBias
```

### 3.1 阶段性 gyro-only 资格 key（ADR-0002）

上表仍是完整 VIO 的唯一 canonical state/key 合同。仅在
`PHAD-M4-ONLINE-GYRO-BIAS-SYNTHETIC-V1` 的 default-off、无 real caller synthetic
资格阶段，允许 estimator PIMPL/private implementation 内部使用：

```text
G(k) = gtsam::Symbol('g', frame_index) -> Vector3  # 仅 b^g，B frame，rad/s
```

这里的 `k` 是实际 `Pose3` graph state 的 `frame_index`，包含 accepted non-keyframe；每个
实际 pose graph state 都有且只有一个对应的 `G(k)`。`G(k)` 的 GTSAM key、type 与 ownership
不得进入 public API；允许按本协议通过不含 GTSAM key/type 的 project-owned POD 公开
in-memory bias value 与 diagnostics。它们不得成为 config key 或持久 artifact schema，也不得
形成真实数据或 app caller。

`B(k)` 继续且只用于 `imuBias::ConstantBias`；同一 factor graph 中不得用 `G(k)` 与 `B(k)`
重复表示同一个 physical gyro bias。full `X/V/B` slice 另行获授权时，必须直接删除 `G(k)`
state、gyro-only factor 及其 lifecycle 并改用 `B(k)`；不得增加 key alias、compatibility
adapter、dual-write 或 migration fallback。

本协议只允许 test code 通过 in-memory project-owned POD
`EstimatorOptions::m_gyro_bias` 激活该阶段性机制；它保持 default-off，且不得接入 CLI/config
parser、`flattenConfig()`、`config_hash`、persistent artifact schema、apps/session 或 real-data
caller。该 public value seam 不改变 private ownership：`G(k)` 的 GTSAM key/type、PIM、factor 与
lifecycle 仍只能存在于 estimator PIMPL/private implementation。

Q3 的 `visual_posterior_aligned_nuisance` 不得用于初始化或约束 `G(k)`；Q3 的
`HYPOTHESIS_FAIL/STOP` 结论保持不变。完整边界与退役条件见
[ADR-0002](adr/0002-stage-gated-gyro-only-bias-state.md)。

项目数据结构中的 bias 字段顺序明确写成 accelerometer、gyroscope；
禁止依靠长度为 6 的裸向量位置约定在模块间传递 bias。

## 4. IMU 测量模型

加速度计输出是 specific force，而不是直接的世界系线加速度：

\[
\tilde{\mathbf a}
=
{}^B\mathbf R_W
\left(
{}^W\mathbf a - {}^W\mathbf g
\right)
+ \mathbf b^a + \mathbf n^a
\]

陀螺仪模型为：

\[
\tilde{\boldsymbol\omega}
= \boldsymbol\omega + \mathbf b^g + \mathbf n^g
\]

静止、水平且 bias 已去除时，加速度计应测得约
\([0,0,+g]^\top\)。这是第一批数据与预积分测试必须验证的符号约定。

预积分调用的参数顺序固定为：

```text
integrateMeasurement(accel_mps2, gyro_radps, dt_seconds)
```

## 5. 时间合同

### 5.1 表示

- 外部输入时间戳首先保存为整数纳秒；
- 时间戳的 epoch 可以由数据集决定，但同一运行内必须一致；
- 只有在计算时间差时才转换为 `double` 秒；
- 不使用浮点绝对秒作为传感器队列的排序键。

### 5.2 顺序

每种传感器流必须严格单调递增。重复或逆序时间戳是输入错误，应携带
传感器名称和问题时间戳显式报告。

第 \(k-1\) 与第 \(k\) 个图像时刻之间的预积分区间定义为：

\[
(t_{k-1}, t_k]
\]

但仅用区间筛选样本不足以完成精确积分。同步模块必须在两个图像边界处
拥有测量值；当边界位于相邻 IMU 样本之间时，对 accelerometer 和
gyroscope 分别做线性插值，再按分段常值测量积分。

必须满足：

\[
\sum_i \Delta t_i = t_k - t_{k-1}
\]

允许浮点舍入误差，但不允许通过修改最后一个 \(\Delta t\) 静默填补真实
数据缺口。

### 5.3 Camera–IMU 时间偏移

第一版假设输入已使用标定得到的 camera–IMU time offset 完成时间对齐。
在线估计 time offset 不属于第一版状态。

适配器应保留原始时间戳，并产生规范化后的核心时间戳，以便诊断偏移方向
和单位错误。

## 6. 单位

核心模块只接受 SI 单位：

| 量 | 单位 |
|---|---|
| 时间戳 | ns（整数） |
| 时间差 | s |
| 位置 | m |
| 速度 | m/s |
| 加速度计测量与 bias | m/s² |
| 角度 | rad |
| 角速度与 gyro bias | rad/s |
| 相机焦距、主点、观测 | pixel |

度、毫秒、微秒、重力加速度倍数 `g` 等外部表示只能存在于数据适配器和
配置解析层。转换后应通过具有单位后缀的字段名传入核心模块。

## 7. IMU 噪声配置

以下量必须分开配置，禁止都命名为模糊的 `imu_noise`：

- accelerometer white-noise density；
- gyroscope white-noise density；
- accelerometer bias random-walk density；
- gyroscope bias random-walk density；
- integration uncertainty。

配置文件必须注明每一项的单位，以及它是连续时间 density、离散标准差
还是方差。GTSAM 参数构造模块负责唯一一次转换；调用方不得自行平方、
乘除 \(\Delta t\) 或重复离散化。

具体转换公式在引入第一份目标数据集并核对其标定格式后写入测试和实现。
在此之前不写“看似合理”的默认噪声。

## 8. 数值与错误合同

进入核心模块的数据必须满足：

- 时间戳有效且顺序正确；
- 三维向量各元素有限；
- \(\Delta t>0\)；
- 相机内参有限且焦距为正；
- 刚体变换旋转合法；
- noise sigma/density 为有限正数；
- covariance 对称，并在适用场景下为正定或半正定。

违反合同返回带上下文的错误；不使用 NaN 继续计算，不静默丢弃样本，
不将无效值替换为零。

## 9. 待确认事项

以下事项需要在接入第一个数据集前确认：

1. 第一目标数据集是 EuRoC、KITTI raw 还是自采数据；
2. 数据集给出的 camera–IMU 外参方向和时间偏移符号；
3. IMU 噪声字段的统计含义和单位；
4. stereo 图像是否硬件同步；
5. 初始静止时间和允许的数据间断阈值；
6. world 原点与初始 yaw 的具体初始化策略。
