# `phad::sync` 传感器同步

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录提供与数据源无关的队列式同步器。M3.2 落地
`StereoPairSynchronizer`（StereoOnly）：左右 B 族配对。M4 在同一模块扩展
`pushImu` 与 `StereoImuPacket`。不读文件、不依赖 `phad::io`。

CMake target：`phad_sync`（alias `phad::sync`），公开依赖 `phad::sensor`、
`phad::common`。

## 职责边界

| 做 | 不做 |
|---|---|
| 左右队列 B 族配对（默认 `tol_ns=0`） | EuRoC / ROS 路径、读 PNG |
| 有界队列与 drop-oldest 溢出计数 | IMU 边界插值（M4） |
| 诊断计数与 sticky 校验错误 | 回调式输出；打日志 |

调用方（`apps/StereoPairStream` 等）负责 `push`/`tryPop`/`flush` 编排与
warning 字符串；本库只维护计数器。

## 文件布局

| 文件 | 作用 |
|---|---|
| `stereo_pair_synchronizer.hpp` / `.cpp` | `StereoPairSynchronizer`、选项、诊断、`PushStatus` |

## 数据流

```text
ImageFrameEvent (Left|Right)
        │
        ▼
  pushImage → 单相机单调性校验（sticky）
        │
        ▼
  left_q / right_q（有界；overflow → drop-oldest）
        │
        ▼
  drain：|tL-tR|<=tol → StereoFrame{tL, L, R}
         否则丢偏早一侧
        │
        ▼
  tryPop() → StereoFrame
  flush()  → 两侧剩余全部计入 dropped_*
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

### `PushStatus`（`pushImage` 返回值）

| 值 | 含义 |
|---|---|
| `kOk` | 输入被接受（随后可能丢帧或成对，见计数 / `tryPop`） |
| `kOutOfOrder` | 同相机 stamp 回退；sticky |
| `kDuplicate` | 同相机 stamp 重复；sticky |
| `kInvalidStamp` | 非法 `CameraId`（及未来非法 stamp）；sticky |

sticky 置位后：后续 `pushImage` 直接返回同一状态且不入队；`tryPop()` 仍可取出
已配好的对。

### 配置

- 默认 `tol_ns = 0`（exact）。soft 仅显式配置；`tol_ns > 0` 时为贪心配对，
  不一定是时间上最近的一对。
- `max_queue >= 1`；offline 宜用较大上界，避免与溢出策略搅在一起。
- `tol_ns < 0` 或 `max_queue == 0` → 构造抛 `std::invalid_argument`。
