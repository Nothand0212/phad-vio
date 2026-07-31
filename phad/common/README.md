# `phad::common` 公共类型

本文档描述当前约定，不是绝对约束，会随项目开发修订。

跨 module 共享、与数据来源无关的基础类型放在这里，避免 `io` ↔ `eval`
等方向出现反向依赖。当前只有时间戳与轨迹载体。

CMake target：`phad_common`（alias `phad::common`），公开依赖 Eigen。

## 职责边界

| 做 | 不做 |
|---|---|
| `Timestamp`、`Trajectory` / `TimedPose` | 数据集解析、评估指标、传感器标定 |
| 轨迹不变量的集中校验（`Trajectory::create`） | 坐标系变换语义之外的算法 |

## 文件布局

| 文件 | 作用 |
|---|---|
| `timestamp.hpp` | 整数纳秒时间戳（header-only） |
| `trajectory.hpp` / `.cpp` | `TimedPose`、`Trajectory`、校验工厂与错误类型 |

## 合同约定

### `Timestamp`

- 存储为 `std::int64_t` 纳秒；比较用默认 `<=>`。
- 不隐式与 `double` 秒互转：绝对时间（EuRoC 量级）经 `double` 会丢纳秒精度。
  需要秒表示时由调用方（如 TUM I/O）显式拆分。

### `Trajectory`

- 位姿为 `Eigen::Isometry3d` 的 `T_W_B`（body → world），见
  [`docs/conventions.md`](../../docs/conventions.md)。
- **只能**通过 `Trajectory::create` 构造：非空、时间戳严格递增、平移有限、
  旋转在容差内正交。消费方（关联、ATE、RPE）可依赖这些前提，不必再校一次。
- 失败返回 `TrajectoryError`（空、非有限、坏旋转、重复/乱序时间戳），带出错下标。

合成测试轨迹见 `tests/common/synthetic_trajectory.hpp`（`phad::testing`），
不进本库。

## 相关入口

| 位置 | 用途 |
|---|---|
| `phad::io` / `phad::eval` | 真值与估计轨迹的共同载体 |
| `tests/common/` | `Timestamp` / `Trajectory` 单测与合成 fixture |
