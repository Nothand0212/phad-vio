# ADR-0001：采用 GTSAM 构建 VIO 后端

- 状态：已接受
- 日期：2026-07-28

## 背景

本项目的首要目标是深入学习完整的 VIO pipeline、IMU 预积分、状态与因子
合同、初始化、边缘化和增量平滑，而不是最快复用已有视觉 SLAM 后端。

候选方式包括：

- g2o 自定义 pose/velocity/bias 顶点和 IMU 边；
- Ceres 参数块、残差块和自研边缘化；
- GTSAM 导航状态、IMU factor 和 factor graph/smoother。

## 决策

局部 VIO 后端采用 GTSAM。

第一版使用：

- `Pose3`、`NavState` 和 `imuBias::ConstantBias`；
- `PreintegratedImuMeasurements`；
- `ImuFactor`；
- `BetweenFactor<imuBias::ConstantBias>`；
- 显式 landmark 和 stereo projection factor；
- batch optimizer。

在 batch stereo VIO 验证后，再迁移到：

- `IncrementalFixedLagSmoother`；
- smart stereo factor。

不创建一个允许调用方逐项添加变量和 factor 的通用 `Optimizer` interface。
GTSAM 对象、key、factor 生命周期和更新顺序由 `VioEstimator` 模块内部拥有。

## 理由

GTSAM 的状态和因子语义与本项目的学习目标直接对应：

- IMU 预积分和 bias correction 有成熟实现；
- \(X/V/B\) 变量关系可以从 factor graph 直接检查；
- batch graph、fixed-lag smoothing 和增量推断可按阶段演进；
- 概率噪声模型和边缘化信息不会隐藏在自研先验残差中。

选择 GTSAM 不是因为视觉问题只能使用 GTSAM，也不意味着它在所有实时
VIO 中优于 Ceres 或定制求解器。

## 后果

正面影响：

- 可以聚焦 VIO 状态、测量和因子合同；
- 无需在第一阶段实现 IMU Jacobian 和滑窗边缘化求解器；
- 后续能自然学习 fixed-lag smoother 和 smart factor。

代价与风险：

- 必须准确理解 GTSAM 的 pose、gravity、bias 和 covariance 约定；
- high-level factor 可能隐藏部分数学细节，因此需要合成测试和诊断工具；
- smart factor 和 smoother 的生命周期有额外复杂度；
- GTSAM 版本和构建依赖需要固定并记录。

## 被拒绝的方案

### 继续基于旧项目的 g2o 后端堆叠 IMU 边

旧项目的 pose-only/BA 状态模型无法自然容纳 velocity、bias、时间同步和
真正的边缘化。从空项目建立清晰合同更符合当前学习目标。

### 以 Ceres 实现第一版滑窗

Ceres 很适合异构残差和参数块，但完整 VIO 仍需自行管理状态布局、
linearization point 和 marginalization prior。它适合后续作为对照实现，
不适合作为本项目的首条学习路径。

### 同时使用多个后端

首版同时维护 g2o/Ceres/GTSAM 会引入重复状态定义、噪声转换和坐标系
适配，降低问题定位能力。只有出现明确且经测试证明的独立问题时，才通过
新的 ADR 评估第二个求解框架。

## 验证方式

该决策通过路线图中的阶段性对照验证：

- 纯 IMU 合成解析解；
- 纯双目 batch BA；
- batch stereo VIO；
- batch 与 fixed-lag 短窗口结果对照；
- 显式 landmark 与 smart factor pose 结果对照。

