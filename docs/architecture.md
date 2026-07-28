# 目标架构与模块职责

## 1. 目标与非目标

第一版目标是建立一条可解释、可测试的双目 VIO pipeline：

```text
dataset / sensors
        |
        v
sensor adapters
        |
        v
sensor synchronizer ----> StereoImuPacket
        |
        v
visual-inertial frontend
        |
        v
KeyframeMeasurement
        |
        v
GTSAM VIO estimator ----> VioEstimate
```

第一版非目标：

- 不复制 Kimera-VIO 的全部线程与故障恢复框架；
- 不实现回环、全局地图、mesh 或语义模块；
- 不在线估计相机内参、外参或 camera–IMU time offset；
- 不在首个闭环中使用 smart factor 或 fixed-lag smoother；
- 不同时维护 g2o、Ceres 和 GTSAM 三套后端。

## 2. 核心数据合同

### 2.1 `ImuMeasurement`

表示一个已转换到 body 坐标系、使用 SI 单位的 IMU 样本：

```cpp
struct ImuMeasurement {
  Timestamp timestamp;
  Vector3 accel_mps2;
  Vector3 gyro_radps;
};
```

它不携带数据集特有的轴、单位和时间偏移语义。

### 2.2 `StereoFrame`

表示同一图像时刻的一对双目图像和标定引用。图像缓冲区的所有权策略在
实现前确定，但接口必须保证 frontend 处理期间数据有效。

### 2.3 `StereoImuPacket`

```cpp
struct StereoImuPacket {
  StereoFrame stereo;
  std::vector<ImuMeasurement> imu_segment;
};
```

`imu_segment` 必须覆盖上一图像时刻到当前图像时刻的完整积分区间，并包含
同步模块产生的边界测量。首帧没有前一区间，应以独立的初始化事件表达，
而不是传入空 segment 后让下游猜测语义。

### 2.4 `KeyframeMeasurement`

frontend 只在产生关键帧时向 estimator 提交：

```cpp
struct KeyframeMeasurement {
  Timestamp timestamp;
  std::vector<StereoObservation> observations;
  PreintegratedImu pim;
  TrackingQuality tracking_quality;
};
```

这是 frontend 与 estimator 之间的主要 seam。调用方不负责分配 GTSAM
keys、添加 factor、维护 `Values` 或重置 bias。

### 2.5 `VioEstimate`

estimator 返回完整的当前导航状态、估计时间戳和诊断摘要。返回值必须能
区分：

- 成功估计；
- 输入被拒绝；
- 初始化尚未完成；
- 优化失败。

不能用 identity pose 或上一帧状态冒充成功结果。

## 3. 模块

### 3.1 Sensor adapters

职责：

- 读取数据集或实时驱动输出；
- 转换单位、轴和外参方向；
- 应用已标定的时间偏移；
- 保留原始时间信息供诊断；
- 产出核心数据类型。

数据集特有知识只存在于 adapter 内。核心模块不得出现 `euroc`、`kitti`
等条件分支。

离线双目惯性数据统一产出 `StereoImuDataset`。该 module 拥有规范化标定、
严格有序的 IMU 测量、双目索引和惰性图像解码；EuRoC 与 TUM VI adapter
只负责格式专属的目录、CSV/YAML 字段、外参方向和像素类型转换。格式由
调用方显式选择，不进行目录猜测，也不通过虚基类暴露 parser 生命周期。

`Image` 保留数据集原始无符号灰度深度，当前支持 `uint8_t` 与 `uint16_t`。
调用方必须使用与 `PixelType` 一致的 typed pixel view，不能把 16-bit 图像
静默截断为 8-bit。

### 3.2 Sensor synchronizer

对外接口应保持小而完整，例如：

```cpp
SyncResult pushImu(ImuMeasurement);
SyncResult pushStereo(StereoFrame);
```

内部负责：

- 缓冲和顺序检查；
- 双目图像配对检查；
- 图像边界 IMU 插值；
- 构造无间断的 `StereoImuPacket`；
- 限制队列长度并显式报告溢出。

调用方不应自行从 IMU 队列切片；否则时间边界逻辑会散落到数据集、
frontend 和测试中。

### 3.3 Visual-inertial frontend

职责：

- 双目特征检测、LK 跟踪与匹配；
- 几何外点剔除；
- 维护稳定的 `LandmarkId`；
- 使用 IMU rotation prediction 辅助视觉跟踪；
- 关键帧判断；
- 累积关键帧间预积分；
- 形成 `KeyframeMeasurement`。

frontend 可以使用 estimator 最近返回的 bias 和状态预测，但不拥有最终
导航状态，也不执行紧耦合图优化。

### 3.4 GTSAM VIO estimator

外部 interface 以一次关键帧更新为中心：

```cpp
VioUpdateResult update(const KeyframeMeasurement& measurement);
```

模块内部拥有：

- GTSAM key 分配；
- factor graph 与 `Values`；
- pose、velocity、bias 生命周期；
- landmark 生命周期；
- 初值预测；
- noise model；
- 优化器或 smoother；
- 优化结果提取；
- bias 反馈。

这是一个 deep module：调用方提交规范化测量并得到估计，不学习 GTSAM
内部更新顺序。首版只有一个 GTSAM 实现，因此不预先创建通用
`Optimizer` seam。

### 3.5 Initialization

初始化是 estimator 内部明确的状态机，不是散落在 frontend 的布尔标志。
第一版针对双目和起始静止数据：

1. 收集一段静止 IMU；
2. 估计初始 gyro bias；
3. 估计重力方向；
4. 初始化 roll/pitch；
5. 将初始 velocity 设为零；
6. 建立 \(X_0,V_0,B_0\) priors；
7. 进入 tracking 状态。

初始化失败必须返回原因和所需的下一步数据。

## 4. 第一版因子图

关键帧状态：

```text
X(k): body pose in world
V(k): body velocity in world
B(k): accelerometer and gyroscope bias
L(j): explicit 3D landmark in world
```

初始因子：

```text
PriorFactor<Pose3>                 X(0)
PriorFactor<Vector3>               V(0)
PriorFactor<ConstantBias>          B(0)
```

相邻状态因子：

```text
ImuFactor                          X(k), V(k), X(k+1), V(k+1), B(k)
BetweenFactor<ConstantBias>        B(k), B(k+1)
```

视觉因子首版使用显式 landmark 的 stereo projection factor。这样可以在
引入 structureless smart factor 前，直接检查 landmark 初值、cheirality
和重投影误差。

## 5. 状态与所有权

- sensor adapter 拥有外部资源句柄；
- synchronizer 拥有尚未消费的传感器缓冲；
- frontend 拥有 feature track 和关键帧判定状态；
- estimator 独占因子图、状态和 GTSAM 对象；
- 可视化与日志只消费不可变快照，不反向修改估计状态。

不在模块间传递拥有不明的裸指针。GTSAM PIM 的具体传递方式需要在实现
阶段结合复制成本确定，但所有权必须由类型表达。

## 6. 线程模型

阶段 1 至阶段 4 使用单线程确定性执行：

```text
read -> synchronize -> frontend -> estimator -> record
```

只有在 batch VIO 正确、可复现并通过数据集回放后才引入线程。未来线程化
时保持相同数据合同，通过有界队列连接模块，不让模块共享可变 frame、
track 或 factor graph。

shutdown、队列溢出和后台异常都必须是可观察事件。

## 7. 错误与诊断

错误按产生模块报告，并携带足够上下文：

- 输入错误：传感器、时间戳、字段和值；
- 同步错误：缺失的时间区间和已有样本范围；
- frontend 错误：track 数、内点数和几何检查结果；
- estimator 错误：关键帧 id、factor 类型、初值状态和 GTSAM 原始原因。

核心诊断至少记录：

- 每个 packet 的 IMU 样本数和总 \(\Delta t\)；
- frontend track/内点/新 landmark 数；
- 每类 factor 数量；
- 优化前后总 error；
- 当前 pose、velocity、bias；
- 初始化和 tracking 状态变化。

## 8. 受控演进

基础 batch VIO 验证后，按以下顺序演进：

1. batch optimizer 替换为 fixed-lag smoother；
2. 显式 landmark 替换为 smart stereo factor；
3. 接入真实 LK frontend；
4. 引入 bounded queues 和线程；
5. 增加独立 global pose graph 与回环。

每次只替换一个主要机制，并保留上一阶段的合成测试作为回归证据。
