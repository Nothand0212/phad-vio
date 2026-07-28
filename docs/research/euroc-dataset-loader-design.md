# EuRoC 数据集加载器设计调研

日期：2026-07-28

> 本文保留当时的调研结论。顶层 `phad::dataset` 布局已被后续架构诊断取代；
> 当前实现归属 `phad::io::dataset`，见
> [`dataset_io_module_relocation.plan.md`](../plans/dataset_io_module_relocation.plan.md)。

## 1. 结论

建议把新的第一个里程碑定义为：

> 从原生 EuRoC/ASL 目录中加载并严格校验双目、IMU 和标定数据，建立不可变
> 索引，并按需解码双目图像；暂不负责 IMU 分段、边界插值、初始化或 VIO。

公开模块应是具体的 `phad::dataset::EurocDataset`，而不是提前设计通用
`DatasetInterface`。它隐藏 CSV、YAML、filesystem 和 OpenCV 的细节，对外
只提供满足项目时间、单位、坐标系和所有权合同的数据。

推荐采用两阶段加载：

1. `Open()` 解析配置和 CSV、校验整个清单并建立索引；
2. `loadStereo(index)` 在消费某帧时才解码左右 PNG。

这比一次性预加载图像更适合本地 `MH_01_easy`：双目灰度图完全解码后约
2.658 GB，而 CSV、标定、路径索引和 IMU 测量只需很小的常驻内存。

## 2. 范围和现有路线图的冲突

### 事实

当前 [roadmap](../roadmap.md) 把阶段 1 定义为纯 IMU 预积分，把“目标数据集
adapter、sensor synchronizer、边界插值和数据集片段回放”一起放在阶段 4。
[architecture](../architecture.md) 又明确要求数据集特有知识只存在于 adapter，
同步模块独占 IMU 切片和图像边界插值。

### 推断

如果现在把“数据集加载”提升为第一个里程碑，又照搬原阶段 4 的全部范围，
将同时引入文件格式、同步、初始化和回放，无法单独判断错误来自哪一层。

### 建议

将加载器作为新的基础设施 M1，但保留以下边界：

- M1 负责原始数据到规范化测量和标定的转换；
- 后续 synchronizer 负责缓冲、IMU 区间和边界插值；
- 后续 replay runner 负责事件调度、暂停、倍速和生命周期；
- frontend/estimator 不接触路径、CSV、YAML 或 EuRoC 字段名。

采纳后需要另行更新 `docs/roadmap.md`。本调研不直接改写已接受的路线图。

## 3. 一手资料

### 3.1 EuRoC 官方合同

EuRoC 官方页面说明数据包含两路 20 FPS WVGA 单色图像、200 Hz IMU、
shutter-centric temporal alignment、相机内外参和时空对齐的 ground truth：

- [ETHZ ASL：EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/)
- [数据集论文 DOI](https://doi.org/10.1177/0278364915620033)

论文的数据格式章节规定：每个 sensor 目录包含 `sensor.yaml`、`data.csv`
以及可选的 `data/`；CSV 首行包含字段名和 SI 单位；`T_BS` 表示 sensor
相对于 body/sensor-system 的外参，即把 sensor 表达的量转换到 body 的
变换。这与项目使用的 \({}^B T_S\) 方向一致。

官方页面也明确提示：左右相机使用独立自动曝光，所以一对双目图像可能有
亮度差；loader 不应把左右图像灰度一致性当作有效性条件。

### 3.2 ORB-SLAM3

[ORB-SLAM3 的 EuRoC stereo-inertial 示例（调研快照）](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
先加载图像路径和 IMU 数组，再在逐帧主循环中选取当前相机时刻之前的 IMU，
按需调用 `cv::imread()`，最后按数据集时间间隔 sleep。

可借鉴：

- 图像按需解码，而不是预解码整个序列；
- IMU 和图像加载失败必须终止当前运行；
- 离线数据可以按时间顺序确定性消费。

不应照搬：

- 示例把整数纳秒除以 \(10^9\) 后保存为绝对 `double` 秒；
- 图像路径由时间戳字符串拼出，而没有把 `data.csv` 作为清单真源；
- 文件解析、IMU 切片、算法调用和实时节流耦合在一个 executable 中；
- 左右目来自同一个时间列表，未系统验证两份相机清单。

这些做法适合作为项目 demo，不适合作为 `phad-vio` 的长期数据合同。

### 3.3 VINS-Fusion

VINS-Fusion 的官方 EuRoC 示例不是原生目录 loader：README 要求播放
EuRoC rosbag，节点从 ROS topics 接收 IMU 和图像
（[README 调研快照](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/README.md)；
[节点源码](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/rosNodeTest.cpp)）。
节点为左右图建立独立队列，以 3 ms 容差配对，超出容差时丢弃较早一侧；
图像通过 `cv_bridge` 转为 `MONO8` 后 clone，再提交 estimator。

事实表明它的同步策略针对可能有 jitter 的实时/ROS 输入。对已知为硬同步、
且拥有两份完整 CSV 清单的原生 EuRoC，M1 应 exact join 并在不一致时失败，
不应静默套用“最近邻 + 丢帧”。可借鉴的是 estimator 只消费已解码测量，
不理解数据集目录。

### 3.4 Basalt

Basalt 明确定义了 `VioDataset` 与 `DatasetIoInterface`，EuRoC 实现把
timestamp、IMU 和 ground truth 索引常驻内存，而
`get_image_data(timestamp)` 才调用 `cv::imread()`：
[通用接口](https://github.com/VladyslavUsenko/basalt/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/include/basalt/io/dataset_io.h)、
[EuRoC 实现](https://github.com/VladyslavUsenko/basalt/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/include/basalt/io/dataset_io_euroc.h)。
它还提供独立脚本检查传感器时间间隔、双目 timestamp 覆盖和图像缺失：
[验证脚本](https://github.com/VladyslavUsenko/basalt/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/scripts/basalt_verify_dataset.py)。

值得采用的是“元数据索引与像素解码分离”以及独立全清单验证。不能直接
照搬其假设和错误模型：该 EuRoC 实现只读 cam0 的 `data.csv`，随后用同一个
timestamp/filename 查找两路图像；部分错误路径只是打印或 `abort()`。
`phad-vio` 应实际解析两份相机清单并返回结构化错误。对本项目而言，第一种
格式就建立多态 hierarchy 收益不足；第二种格式出现后再抽取公共接口。

### 3.5 OpenVINS

[OpenVINS DatasetReader（调研快照）](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/utils/dataset_reader.h)
只负责 ASL/EuRoC ground truth 和仿真轨迹读取；正常相机/IMU 测量则由
[ROS 订阅入口](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/run_subscribe_msckf.cpp)
送入系统。源码注释明确 ground truth reader 用于初始化、绘图和 RMSE。

这一分工支持把 ground truth 保持为独立 evaluator adapter；M1 不把它包装成
普通 `SensorEvent` 送入 VIO。其 reader 在文件或格式错误时直接退出进程，
而库级 `EurocDataset::Open()` 应把错误返回调用方。

## 4. 本地 `MH_01_easy` 审计

数据根目录：

```text
/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy
```

实际读取和检查结果：

| 流 | 记录数 | 首时间戳 ns | 末时间戳 ns | 顺序 |
|---|---:|---:|---:|---|
| cam0 | 3682 | 1403636579763555584 | 1403636763813555456 | 严格递增 |
| cam1 | 3682 | 1403636579763555584 | 1403636763813555456 | 严格递增 |
| imu0 | 36820 | 1403636579758555392 | 1403636763853555456 | 严格递增 |
| ground truth | 36382 | 1403636580838555648 | 1403636762743555584 | 严格递增 |

进一步事实：

- cam0/cam1 的 3682 行清单逐行一致；
- 每个 camera timestamp 都精确对应一个 IMU timestamp；
- 没有重复或逆序 timestamp；
- 7364 个 CSV 引用的 PNG 全部存在；
- 文件名均是 basename，没有绝对路径或 `..`；
- PNG 是 752×480、8-bit grayscale；
- camera 相邻时间约 50 ms，IMU 相邻时间约 5 ms；
- `cam*/sensor.yaml` 为 pinhole + radial-tangential；
- `imu0/sensor.yaml` 给出四种连续时间 noise density/random walk；
- `imu0` 的 `T_BS` 为 identity。

这些是该序列的实测事实，不应被提升为所有 ASL 数据必然满足的假设。

## 5. 推荐模块边界

```text
phad/
  common/
    timestamp.hpp
  sensor/
    imu_measurement.hpp
    stereo_frame.hpp
    calibration.hpp
  dataset/
    dataset_error.hpp
    euroc/
      euroc_dataset.hpp
      euroc_dataset.cpp
      csv_reader.cpp
      calibration_reader.cpp
tests/
  dataset/euroc/
apps/
  phad_euroc_inspect.cpp
```

职责：

- `common::Timestamp`：有符号 64-bit 整数纳秒；排序不转浮点。
- `sensor::*`：不含 `euroc`、文件路径或 YAML 字段名的核心类型。
- `dataset::EurocDataset`：唯一公开的 EuRoC 加载 facade。
- CSV/YAML parser：作为 `euroc` 内部实现，不公开成通用框架。
- `phad_euroc_inspect`：打印摘要并解码抽样帧，作为人工验收入口。

当前 `phad/io/image.hpp` 只是未完成的用户工作。实现时应先明确它究竟是
拥有像素的核心图像值类型，还是文件 codec；不要让名为 `Image` 的对象同时
承担图像内存、路径、解码器和 dataset record 四种职责。

## 6. 推荐公开 API

以下是接口形状，不是要求逐字采用的最终 C++：

```cpp
class EurocDataset {
 public:
  static Result<EurocDataset, DatasetError> Open(
      const std::filesystem::path& sequence_root);

  const EurocCalibration& calibration() const;
  std::span<const ImuMeasurement> imuMeasurements() const;
  std::span<const StereoFrameRef> stereoIndex() const;

  Result<StereoFrame, DatasetError> loadStereo(std::size_t index) const;
};

struct StereoFrameRef {
  Timestamp timestamp;
  std::filesystem::path left_path;
  std::filesystem::path right_path;
};
```

`sequence_root` 应接受 `.../MH_01_easy`，由 facade 统一解析其 `mav0`；
不要让调用方分别拼 `cam0`、`cam1` 和 `imu0` 路径。

如果首个构建环境没有 `std::expected`，应在确定 C++ 标准后选择一个已有的
Result/Status 方案；不要仅为 loader 仓促发明全项目通用错误框架。

暂不提供：

- 虚基类 `Dataset`；
- callback/threaded provider；
- merged `SensorEvent`；
- `nextPacket()` 或 `getImuBetweenFrames()`；
- 隐式缓存和预取。

这些接口会提前决定同步、线程和背压策略。

## 7. 时间与同步

- CSV timestamp 必须完整解析为 `int64_t` ns；
- 禁止用浮点绝对秒作为索引或相等性判断依据；
- 每个 sensor 流必须严格单调递增；
- cam0/cam1 以 timestamp 做 exact join；
- 缺左帧、缺右帧、重复或错序均返回带上下文的错误；
- 同 timestamp 时如何调度 IMU 和 stereo 属于后续 replay/synchronizer；
- M1 不构造 `(t_{k-1}, t_k]` IMU segment，也不插值。

虽然当前 MH_01 每个图像时刻都精确命中 IMU，后续 synchronizer 的测试仍
必须覆盖图像边界落在两个 IMU 样本之间的情况，不能让 loader 偶然替代
同步合同。

## 8. 标定和单位

`Open()` 从原生 `sensor.yaml` 读取：

- 两相机 `T_BS`、分辨率、模型、内参和畸变；
- IMU `T_BS`、频率和四类 noise 参数。

规范化输出中：

- `T_BS` 命名为方向明确的 `T_B_camera` / \({}^B T_C\)；
- gyro 保持 rad/s，accel 保持 m/s²；
- noise 字段保留 density/random-walk 语义和单位；
- 不在 loader 中平方、离散化或构造 GTSAM covariance；
- 不在 loader 中去畸变或立体校正图像。

M1 可针对目标序列验证 IMU `T_BS` 为 identity。若以后支持非 identity
IMU 外参，不能只旋转 acceleration 就声称完成任意 lever-arm 转换；该扩展
需要单独的测量模型和测试。

不建议像部分示例项目一样复制一份 EuRoC 标定到项目私有配置，因为会产生
两个真源。算法调参应与 dataset intrinsic/extrinsic calibration 分开。

## 9. 图像和所有权

- manifest 永久保存 timestamp 和已解析的安全路径；
- `Open()` 检查所有引用文件存在，但不解码所有像素；
- `loadStereo()` 同时解码左右图，返回拥有或共享明确的不可变 buffer；
- 解码后验证非空、752×480、单通道、8-bit；
- 左右帧任一失败时整个 stereo load 失败，不返回半帧；
- M1 不实现 LRU cache；测量证明需要后再增加；
- frontend 处理期间图像 buffer 必须有效，不能返回指向临时 `cv::Mat` 的
  裸视图。

## 10. CSV、路径与错误合同

解析器至少检查：

- 文件可打开且 header 与预期 schema/单位一致；
- 每行列数正确且没有尾随未解析文本；
- timestamp 可无损表示为 `int64_t`（epoch 由数据集决定，不额外要求大于零）；
- 浮点测量均有限；
- 每个流严格递增且无重复；
- camera filename 非空、为 basename，解析结果位于 `data/` 下；
- 左右 timestamp 集完全匹配；
- `sensor_type`、矩阵维度、旋转和相机参数有效；
- 所有引用文件存在。

错误至少携带：

```text
error_code
sensor_id
source_path
line_or_record_index
timestamp (if available)
field
original cause
```

不允许静默跳行、排序修复、左右最近邻配对、NaN 置零、缺图跳过或返回空
图像冒充成功。若未来需要容错模式，应是显式的独立 policy，并记录所有
丢弃，不属于 M1。

## 11. M1 交付物

1. CMake 和测试骨架；
2. `Timestamp`、`ImuMeasurement`、相机/IMU calibration 等最小核心类型；
3. 具体 `EurocDataset` facade；
4. 原生 YAML、camera CSV、IMU CSV 解析；
5. 全清单校验和双目 exact join；
6. 双目 PNG 惰性解码；
7. `phad_euroc_inspect <sequence-root>`；
8. 小型、自生成/自有的测试 fixture；
9. 可选的本地 MH_01 integration test。

## 12. 验收标准

### 正常路径

- 打开给定 `MH_01_easy` 后得到 3682 个 stereo refs 和 36820 个 IMU；
- 首末 timestamp 与本调研审计值完全一致；
- 两路相机 calibration、IMU noise 和 `T_BS` 与 YAML 一致；
- 第一、中间、最后一对图像都解码为 752×480 `CV_8UC1`；
- 遍历索引具有确定性，两次运行摘要完全一致；
- 常驻内存不随已遍历图像总数线性增长。

### 失败路径

用独立的小 fixture 覆盖：

- 根目录或必需文件缺失；
- header/列数错误；
- 非整数/溢出 timestamp；
- NaN/Inf IMU；
- 重复和逆序 timestamp；
- 左右目缺帧或 timestamp 不一致；
- CSV 引用缺图、绝对路径或 `..`；
- PNG 损坏、尺寸或 pixel type 错误；
- 非法 `T_BS`、未知 camera/distortion model；
- 缺失或非法 noise 字段。

CI 测试不能硬编码用户机器上的绝对数据路径。完整 MH_01 测试可通过
`PHAD_EUROC_MH01_PATH` 或 CTest label 显式启用；默认单元测试使用仓库内
最小 fixture。

## 13. 非目标

M1 明确不包含：

- IMU 图像区间切片和边界插值；
- 起始静止检测和重力初始化；
- 去畸变、双目校正、特征提取；
- ground truth 对齐和 ATE/RPE；
- 实时 sleep、倍速、暂停；
- 多线程、prefetch、bounded queue；
- rosbag、KITTI、TUM-VI 或实时相机；
- GTSAM preintegration 和 estimator。

## 14. 后续演进

建议按以下顺序扩展：

1. 用 loader 输出构造独立 `SensorSynchronizer`；
2. 覆盖精确命中和边界插值两类时间用例；
3. 增加确定性顺序 replay runner；
4. 单独增加 ground-truth evaluator adapter；
5. 出现第二种数据集后，再评估 `Dataset` concept/interface；
6. 只有 profiler 证明解码成为瓶颈后再加 bounded prefetch/cache；
7. 算法闭环稳定后才引入线程和背压。

## 15. 待确认事项

- 首个构建标准是否允许直接使用 `std::expected`；
- YAML 采用 yaml-cpp，还是使用项目已经确定的配置方案；
- 核心图像类型是否直接持有 `cv::Mat`，以及只读语义如何表达；
- `Open()` 是否检查全部文件存在，或提供显式 strict validation 命令；
- 未知 camera model 是在 `Open()` 拒绝，还是允许保留但在规范化阶段拒绝。

上述事项会影响实现接口或依赖，应在编码前确认；它们不影响“具体 EuRoC
facade、整数纳秒、严格配对、索引与惰性解码分离”的总体方向。
