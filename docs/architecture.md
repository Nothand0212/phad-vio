# 目标架构与模块职责

## 1. 目标与非目标

第一版目标是建立一条可解释、可测试的双目 VIO pipeline：

```text
io: dataset / live sensors
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

本文档描述的是**完整 VIO 的目标形态**。按
[实现里程碑与验收标准](roadmap.md)，第一条打通的闭环（M2）是它的纯视觉
子集：不经过 sensor synchronizer，`KeyframeMeasurement` 不携带
preintegration，估计器状态只有 `X(k)` 与 `L(j)`。M4 接入 IMU 时补齐
synchronizer 与 `V/B` 状态，视觉侧的数据合同不变。

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

M2 的纯视觉闭环中不存在 `pim` 字段；它在 M4 接入 IMU 时加入，其余字段
保持不变。

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

`phad::io` 是外部数据进入或离开系统的顶层 module。离线数据集 adapter
归属 `phad::io::dataset`；未来串口和 ROS 输入也归属 `phad::io`，并各自
定义符合来源特性的输入合同。`phad::sensor` 只定义与数据来源无关的测量、
图像和标定类型。

```text
phad::io
├── dataset
│   └── DatasetReplaySource
├── serial        # future
├── ros           # future
└── SensorSource
```

数据集特有知识只存在于 adapter 内。核心模块不得出现 `euroc`、`kitti`
等条件分支。

离线双目惯性数据统一产出可廉价复制的不可变 `StereoImuDataset` handle。
完整校验后的 IMU 与双目 metadata、filesystem path、预期 pixel type 和
解码配置只存在于共享只读 implementation。dataset 对外按值提供 calibration
和 summary，并为每次回放创建 move-only、single-pass 的独立 reader。
reader 拥有自己的双流消费位置与 sticky terminal error，通过 `takeImu()`、
`peekStereoTimestamp()` 和 `takeStereo()` 顺序消费；观察 timestamp 不触发
图像 I/O，取得 stereo 时才惰性解码。EuRoC 与 TUM VI adapter 只负责格式
专属的目录、CSV/YAML 字段、外参方向和像素类型转换，并只通过各自 concrete
`open()` 产生通用 dataset handle。格式由调用方显式选择，不进行目录猜测，
也不通过虚基类或转发 facade 暴露 parser 生命周期。

`Image` 保留数据集原始无符号灰度深度，当前支持 `uint8_t` 与 `uint16_t`。
调用方必须使用与 `PixelType` 一致的 typed pixel view，不能把 16-bit 图像
静默截断为 8-bit。

`SensorSource` 是来源无关的 pull-based 输入 seam，只公开稳定标定和下一个
规范化 sensor event。`DatasetReplaySource` 从 dataset 创建并拥有独立
reader，同时自持有 calibration、一条 IMU lookahead 和 terminal state；
它比较 lookahead 与 reader 观察到的下一帧 stereo timestamp，timestamp
相同时先输出 IMU，并仅在 stereo 成为下一事件时解码图像。正常耗尽与读取
失败是不同的 terminal 状态。该 seam 不执行 IMU 分段、边界插值或 packet
构造。

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

M2 的纯视觉闭环使用视觉初始化（首帧建立 gauge-fixing prior 并三角化初始
landmark）。上述惯性初始化在 M4 加入，M5 用更鲁棒的方案替换起始静止假设。

### 3.6 评估与可视化

评估模块是估计链路之外的独立模块，只消费不可变结果，不参与优化：

- 真值轨迹加载与时间关联；
- TUM 格式轨迹导出；
- SE3 对齐（双目尺度已知，不估 scale）与 ATE、RPE 计算；
- 实时 2D 面板与离线绘图。

它必须能被单独测试：把真值当作估计输入时 ATE 为 0，是该模块自身正确性的
判据。轨迹导出格式与 `evo` 兼容，以便交叉验证自实现的指标。

估计器不依赖评估模块，评估模块也不回写估计状态。

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

M2 的纯视觉图只包含 `X(k)`、`L(j)`、一个 gauge-fixing `PriorFactor<Pose3>`
和 stereo projection factor。M4 在此基础上加入 `V(k)`、`B(k)` 与
`ImuFactor`、`BetweenFactor<ConstantBias>`，视觉部分不变。

## 5. 状态与所有权

- sensor adapter 拥有外部资源句柄；
- synchronizer 拥有尚未消费的传感器缓冲；
- frontend 拥有 feature track 和关键帧判定状态；
- estimator 独占因子图、状态和 GTSAM 对象；
- 可视化与日志只消费不可变快照，不反向修改估计状态。

不在模块间传递拥有不明的裸指针。GTSAM PIM 的具体传递方式需要在实现
阶段结合复制成本确定，但所有权必须由类型表达。

## 6. 线程模型

M1 至 M6 使用单线程确定性执行：

```text
read -> synchronize -> frontend -> estimator -> record
```

只有在 VIO 在真实序列上正确、可复现且 ATE 稳定后（M7）才引入线程。
线程化时保持相同数据合同，通过有界队列连接模块，不让模块共享可变
frame、track 或 factor graph。

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

先用最短路径打通端到端可观察行为，再按 ATE 指标逐层加深。顺序为：

1. 建立评估与可视化底座（轨迹导出、ATE/RPE、实时面板）；
2. 接入 LK frontend，打通纯双目 VO 闭环并测出第一个真实 ATE；
3. 以 ATE 为准绳加固 VO（几何验证、关键帧策略、landmark 生命周期）；
4. 加入 IMU 预积分、synchronizer 与 `V/B` 状态，视觉因子不变；
5. batch optimizer 替换为 fixed-lag smoother 并引入边缘化；
6. 视需要将显式 landmark 替换为 smart stereo factor；
7. 引入 bounded queues 和线程；
8. 增加解耦的 global pose graph 与回环。

两条约束贯穿全过程：

- **每次只替换一个主要机制**，并保留上一阶段的测试作为回归证据；
- **每次替换都伴随同一组序列的 ATE 前后数字**。无法测量的改动不构成
  演进。

前端与后端均先使用成熟库（OpenCV、GTSAM）。任何自研替换以调库版本为
oracle，通过 ATE 与中间量分布的对拍验收，不满足对拍时两者并存。

阶段划分与出口条件的权威来源是
[实现里程碑与验收标准](roadmap.md)。
