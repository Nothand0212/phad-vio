# `phad::io` 输入输出层

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录是外部数据进入（或离开）系统的顶层 module：把数据集、未来串口 /
ROS 等来源转成与格式无关的 `phad::sensor` 测量与标定。数据集特有知识
（目录布局、CSV/YAML 字段、外参方向、像素类型）只存在于各 adapter 内；
核心模块不得出现 `euroc`、`tum_vi` 等条件分支。

| CMake target | 内容 |
|---|---|
| `phad_io`（alias `phad::io`） | header-only：`SensorSource` 合同；依赖 `phad::sensor` |
| `phad_io_dataset`（alias `phad::io_dataset`） | 离线数据集加载与回放；公开依赖 `phad::io`、`phad::common`；私有依赖 OpenCV imgcodecs、yaml-cpp |

## 职责边界

| 做 | 不做 |
|---|---|
| 解析 EuRoC / TUM VI 目录与标定，产出 `StereoImuDataset` | ATE / RPE / TUM 轨迹评估（归 `phad::eval`） |
| 加载 EuRoC 真值为 `common::Trajectory`（`T_W_B`） | IMU 分段、边界插值、`StereoImuPacket`（归 synchronizer） |
| `SensorSource` / `DatasetReplaySource` 按时间拉事件 | 特征跟踪、估计、可视化 |
| 单位、轴、外参方向规范化到 `docs/conventions.md` | 猜测序列格式；调用方显式选 `euroc::open` 或 `tum_vi::open` |

测量与标定类型本身在 `phad::sensor`；本目录只负责「从磁盘/设备读出并适配」。

## 文件布局

```text
phad/io/
├── sensor_source.hpp              # SensorSource、SensorEvent、EndOfStream
└── dataset/
    ├── dataset_error.hpp          # DatasetError / DatasetResult
    ├── stereo_imu_dataset.hpp     # StereoImuDataset + Reader（按值 handle）
    ├── dataset_replay_source.hpp  # SensorSource 的数据集实现
    ├── internal/                  # 共享 builder、CSV/YAML 工具（不对外）
    ├── euroc/
    │   ├── euroc_dataset.hpp      # euroc::open(sequence_root)
    │   ├── euroc_groundtruth.hpp  # euroc::openGroundtruth → Trajectory
    │   └── internal/              # EuRoC YAML 变换解析
    └── tum_vi/
        └── tum_vi_dataset.hpp     # tum_vi::open(sequence_root)
```

## 数据流

```text
sequence_root
      │
      ├─ euroc::open / tum_vi::open ──► StereoImuDataset  (immutable handle)
      │                                      │
      │                         reader() ──► StereoImuDatasetReader
      │                                      │  takeImu / peekStereoTimestamp / takeStereo
      │                                      ▼
      │                         DatasetReplaySource ──► SensorSource::next()
      │                                      │
      │                                      ▼
      │                              SensorEvent (IMU | StereoFrame)
      │
      └─ euroc::openGroundtruth ──► common::Trajectory   (评估侧消费)
```

典型调用：

```cpp
auto dataset = phad::io::dataset::euroc::open(sequence_root);
if (!dataset) { /* DatasetError */ }

phad::io::dataset::DatasetReplaySource source(dataset.value());
while (true)
{
  auto result = source.next();
  if (std::holds_alternative<phad::io::EndOfStream>(result))
  {
    break;
  }
  // SensorEvent 或 SensorSourceError
}

auto gt = phad::io::dataset::euroc::openGroundtruth(sequence_root);
```

## 合同约定

### `StereoImuDataset`

- 打开时完整校验 metadata、路径与标定；校验后的实现放在共享只读 impl。
- 对外按值提供 `calibration()` 与 `summary()`；每次 `reader()` 得到独立的
  move-only、single-pass reader。
- `peekStereoTimestamp()` 不触发图像 I/O；`takeStereo()` 才惰性解码。
- `Image` 保留原始无符号灰度深度（EuRoC：`uint8_t`；TUM VI：`uint16_t`），
  调用方须用与 `PixelType` 一致的 typed view，禁止静默截断。

### `SensorSource`

来源无关的 pull-based seam：只公开稳定标定与 `next()`。
`DatasetReplaySource` 自持 calibration、一条 IMU lookahead 与 terminal
state；timestamp 相同时先输出 IMU，仅在 stereo 成为下一事件时解码图像。
正常耗尽（`EndOfStream`）与读取失败（`SensorSourceError`）是不同 terminal。

### 错误类型

| 类型 | 何时用 |
|---|---|
| `DatasetError` / `DatasetResult` | 打开序列 / 真值失败（缺文件、坏 CSV/YAML、标定不支持等） |
| `DatasetReaderError` | 回放中图像解码失败或格式不符 |
| `SensorSourceError` | `SensorSource::next` 路径上的读取失败 |

评估失败用 `phad::eval::EvalError`，不要与上述混用。

### 外部格式键

EuRoC / TUM VI 的 YAML、CSV **列名与字段按原文**读写（完整拼写）；缩写只用于
内部 C++ API。单位与坐标系转换在 adapter 内完成，产出 SI + `docs/conventions.md`
约定下的 `T_W_B` / body 系测量。

### EuRoC 真值

- 路径：`mav0/state_groundtruth_estimate0/`。
- CSV 位姿为 `T_W_S`；仅当同目录 `sensor.yaml` 的 `T_BS` 为 identity 时接受，
  从而产出的轨迹就是 `T_W_B`。
- 只读 timestamp 与 pose；velocity / bias 列参与列数校验但不产出。
- 四元数未严格单位（偏差可达约 `1.3e-4`）：用较松阈值识别损坏记录，读入后归一化。

### TUM VI

当前 adapter 加载双目 + IMU + 标定。`mocap0` 是 `T_W_marker` 而非 `T_W_B`，
真值导出延后；评估请用 EuRoC 真值或已是 `T_W_B` 的 TUM 文件。

## 相关入口

| 位置 | 用途 |
|---|---|
| `apps/phad_euroc_inspect` | 检查 EuRoC 摘要与抽样图像 |
| `apps/phad_euroc_runner` | 回放 + 可视化（人工确认 GUI） |
| `apps/phad_euroc_gt_export` | EuRoC 真值 → TUM |
| `apps/phad_traj_eval --gt-euroc` | 评估时直接读序列真值 |
| `tests/io/dataset/` | fixture 单测（`-L unit`）；真实序列 gated（`-L mh01` 等） |

模块边界的权威描述见 [`docs/architecture.md`](../../docs/architecture.md)
§3.1；坐标系与单位见 [`docs/conventions.md`](../../docs/conventions.md)。
