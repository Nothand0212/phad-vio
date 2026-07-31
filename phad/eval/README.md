# `phad::eval` 评估库

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录实现轨迹评估（trajectory evaluation）：把估计轨迹与真值轨迹对齐后
算出 ATE / RPE，并提供与 [evo](https://github.com/MichaelGrupp/evo) 兼容的
TUM 读写。库本身不含估计逻辑，也不依赖 OpenCV，便于无 GUI 环境单测。

CMake target：`phad_eval`（alias `phad::eval`），公开依赖 `phad::common`
（因而间接依赖 Eigen）。

## 职责边界

| 做 | 不做 |
|---|---|
| TUM 读写、时间关联、固定尺度 SE3 对齐 | 数据集真值解析（归 `phad::io`） |
| ATE / RPE 与误差统计 | GUI / 实时面板（归 `phad::viz`） |
| 评估失败用 `EvalError` / `EvalResult` | 估计器位姿输出格式（估计侧写出 TUM 后交给本库） |

轨迹载体是 `phad::common::Trajectory`（时间戳严格递增的 `T_W_B` 序列），
由 `Trajectory::create` 集中校验；本目录只消费与产出该类型。

## 文件布局

每个主题一对 `.hpp` / `.cpp`（`eval_error.hpp` 为 header-only）：

| 文件 | 作用 |
|---|---|
| `eval_error.hpp` | `EvalErrorCode`、`EvalError`、`EvalResult<T>` |
| `tum_io.hpp` | TUM 轨迹读写（纳秒时间戳无损往返） |
| `associate.hpp` | 估计 ↔ 真值时间最近邻关联 |
| `align.hpp` | 固定尺度 Umeyama SE3（`alignSe3`） |
| `error_stats.hpp` | RMSE / mean / median / std / max |
| `ate.hpp` | 关联 → 对齐 → 绝对轨迹误差 |
| `rpe.hpp` | 固定时间间隔相对位姿误差（无需对齐） |

## 数据流

```text
est.tum / gt.tum  ──readTum──►  Trajectory
                                    │
                     associate(est, gt)  ──► Association
                                    │
              ┌─────────────────────┴─────────────────────┐
              ▼                                           ▼
         alignSe3 + ATE                              computeRpe
              │                                           │
              ▼                                           ▼
          AteReport                                   RpeReport
     (ErrorStats + samples)                     (ErrorStats, 无对齐)
```

典型调用：

```cpp
auto est = phad::eval::readTum(est_path);
auto gt  = phad::eval::readTum(gt_path);  // 或由 io 加载 EuRoC 真值

auto ate = phad::eval::computeAte(est.value(), gt.value());
auto rpe = phad::eval::computeRpe(est.value(), gt.value());
```

## 格式约定

### TUM 轨迹文件

无 header、空格分隔，一行一位姿：

```text
timestamp tx ty tz qx qy qz qw
```

| 字段 | 约定 |
|---|---|
| `timestamp` | 秒；写出为整数秒 + 9 位定点小数，读入按字符串拆分秒与纳秒，避免 EuRoC 量级绝对时间经 `double` 丢精度 |
| `tx ty tz` | 米，`T_W_B` 的平移 |
| `qx qy qz qw` | 单位四元数，Hamilton，`w` 在末位（与 evo 一致） |

读入后经 `Trajectory::create` 校验（有限、旋转合法、时间戳严格递增）。
EuRoC 真值四元数可能略偏单位性，读入侧应归一化后再建轨迹。

### ATE 逐样本 CSV（CLI 合同）

`phad_traj_eval --errors-csv` 写出的列（本库 `AteReport::samples` 的扁平化）：

```text
timestamp_ns,dt_ns,err_trans_m,err_rot_deg,est_x,est_y,est_z,gt_x,gt_y,gt_z
```

`est_*` 为对齐后的估计位置；`scripts/plot_errors.py` 消费该合同。

### 错误类型

评估失败与数据集加载失败分开：本库用 `EvalError`，不用 `DatasetError`。
常见码：`kNoOverlap`、`kTooFewMatches`、`kMatchRateTooLow`、
`kDegenerateAlignment`、`kNoDeltaPairs`、`kInvalidOptions` 等。

## 指标约定

| 指标 | 含义 | 默认 |
|---|---|---|
| ATE | 时间关联后固定尺度 SE3 对齐，再算平移 / 旋转误差 | 关联阈值 `2.5 ms`（EuRoC 真值 200 Hz 半周期）；`min_match_rate = 0.5` |
| RPE | 固定间隔相对运动误差；左乘刚体下不变，**不对齐** | `delta = 1 s`，伙伴容差 `25 ms`（20 Hz 半周期） |
| 统计量 | 平移米、旋转度；RMSE + mean / median / std / max | 空序列 → 全零 |

双目提供可观尺度，对齐不估 scale。匹配率过低或点集退化时返回错误，
而不是基于少量样本给出漂亮数字。

## 相关入口

| 位置 | 用途 |
|---|---|
| `apps/phad_traj_eval` | 离线对比估计与真值（TUM 或 `--gt-euroc`） |
| `apps/phad_euroc_gt_export` | EuRoC 真值 → TUM，便于 evo 对拍 |
| `tests/eval/` | 合成轨迹单测（`-L unit`）；MH_01 gated 测在 `-L mh01` |
| `scripts/plot_trajectory.py` / `plot_errors.py` | 离线绘图（本地 venv，不进 CI） |

交叉验证：`evo_ape tum <gt> <est> -a` 与 `phad_traj_eval` 在 MH_01 上六位
有效数字一致。坐标系记号见 [`docs/conventions.md`](../../docs/conventions.md)。
