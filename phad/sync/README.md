# `phad::sync` 传感器同步

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录提供与数据源无关的队列式同步器。M3.2 落地
`StereoPairSynchronizer`（StereoOnly）：左右 B 族配对。M4.1 在同一模块扩展
`pushImu` 与 `StereoImuPacket`：IMU 独立队列，配对时切段 `[t_prev, t_cur]`
两端线性插值，大间断标 `imu_gap`。不读文件、不依赖 `phad::io`。

CMake target：`phad_sync`（alias `phad::sync`），公开依赖 `phad::sensor`、
`phad::common`。

## 职责边界

| 做 | 不做 |
|---|---|
| 左右队列 B 族配对（默认 `tol_ns=0`） | EuRoC / ROS 路径、读 PNG |
| IMU 队列切段：两端线性插值、`imu_gap` 标记（M4.1） | 预积分（estimator 构造，M4.2） |
| 有界队列与 drop-oldest 溢出计数 | 回调式输出；打日志 |
| 诊断计数与 sticky 校验错误 | 知道 bias / 重力 |

调用方（`apps/StereoPairStream` 等）负责 `push`/`tryPop`/`flush` 编排与
warning 字符串；本库只维护计数器。

## 文件布局

| 文件 | 作用 |
|---|---|
| `stereo_pair_synchronizer.hpp` / `.cpp` | `StereoPairSynchronizer`、选项、诊断、`PushStatus` |
| `phad/sensor/stereo_imu_packet.hpp` | `StereoImuPacket`（配对帧 + 帧间 IMU 段） |

## 数据流

```text
ImageFrameEvent (Left|Right)        ImuMeasurement
        │                                │
        ▼                                ▼
  pushImage → 单相机单调性校验      pushImu → IMU 单调性校验（独立 sticky）
        │                                │
        ▼                                ▼
  left_q / right_q（有界）          imu_q（有界；overflow → drop-oldest）
        │                                │
        └──────────┬─────────────────────┘
                   ▼
  drain：|tL-tR|<=tol → makePacket：切段 [t_prev, t_cur]
         （left stamp = t_cur）  左端/右端线性插值，ΣΔt ≡ t_cur − t_prev
         否则丢偏早一侧          大间断 → imu_gap
                   │
                   ▼
  tryPopPacket() → StereoImuPacket（tryPop() → 其中 frame）
  flush()  → 两侧 + IMU 剩余全部计入 dropped_*
```

## 诊断计数与结果码合同

### `StereoPairDiagnostics`

| 字段 | 含义 |
|---|---|
| `pushed_left` / `pushed_right` | 成功入队次数（含随后被 orphan/overflow 丢掉的） |
| `emitted_stereo` | 成功配对次数（含尚在 ready 队列未 `tryPop` 的） |
| `dropped_left` / `dropped_right` | B 族 orphan + `flush` 清出的剩余 |
| `dropped_*_overflow` | 超 `max_queue` 时 drop-oldest |
| `max_*_queue` | 观测到的峰值队列深度 |
| `pushed_imu` | 成功入队 IMU 样本数 |
| `dropped_imu` | 越界（早于段左端）+ `flush` 清出的 IMU 剩余 |
| `dropped_imu_overflow` | 超 `max_imu_queue` 时 drop-oldest |
| `imu_gap_count` | 构造出 `imu_gap` 段的次数 |
| `max_imu_queue` | IMU 队列观测峰值深度 |

### `PushStatus`（`pushImage` / `pushImu` 返回值）

| 值 | 含义 |
|---|---|
| `kOk` | 输入被接受（随后可能丢样本或成对，见计数 / `tryPop`） |
| `kOutOfOrder` | 同相机 / IMU stamp 回退；sticky |
| `kDuplicate` | 同相机 / IMU stamp 重复；sticky |
| `kInvalidStamp` | 非法 `CameraId`（及未来非法 stamp）；sticky |
| `kInvalidValue` | 非有限（NaN / Inf）IMU 测量；`pushImu` 专属；sticky |

sticky 置位后：后续 `pushImage`（或 `pushImu`）直接返回同一状态且不入队；
`tryPop()` / `tryPopPacket()` 仍可取出已配好的对。图像与 IMU 的 sticky
**相互独立**。

### 配置

- 默认 `tol_ns = 0`（exact）。soft 仅显式配置；`tol_ns > 0` 时为贪心配对，
  不一定是时间上最近的一对。
- `max_queue >= 1`；offline 宜用较大上界，避免与溢出策略搅在一起。
- `tol_ns < 0` 或 `max_queue == 0` → 构造抛 `std::invalid_argument`。
- `max_imu_queue >= 1`：IMU 队列上界（200 Hz 下 4096 ≈ 20 s 缓冲）；
  `== 0` → 抛 `std::invalid_argument`。
- `imu_gap_ns >= 0`：帧间 IMU 大间断阈值；段宽超过（或两端无法插值）→
  `StereoImuPacket::imu_gap = true`；`< 0` → 抛 `std::invalid_argument`。

### `StereoImuPacket`（`tryPopPacket` 返回）

- 段 `[t_prev, t_cur]`（`t_cur` = 配对帧 left stamp）：左端 = 恰在 `t_prev`
  的原样本（否则上段右端 / 插值）；中间 = 落在 `(t_prev, t_cur)` 的原样本；
  右端 = 恰在 `t_cur` 的原样本（否则与队列 front 线性插值）；
- 相邻段共享边界样本（右端 = 下段左端）；
- 非 `imu_gap` 段：段内相邻样本时间戳差之和 ΣΔt ≡ `t_cur − t_prev`；
- `imu_gap`：段不完整（两端无法构造 / 段宽超 `imu_gap_ns` / 段内无样本），
  消费方应跳过该段预积分因子（视觉照常）；
- 首帧 packet 为零段：`t_prev == t_cur`、`samples` 为空（早于首帧的样本在
  切段时计入 `dropped_imu`）。
