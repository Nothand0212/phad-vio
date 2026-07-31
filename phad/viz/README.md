# `phad::viz` 可视化

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录提供轨迹与图像的轻量可视化：可无显示测试的渲染，以及可选的
highgui 窗口。不参与估计，只消费已有轨迹 / 画布。

CMake target：`phad_viz`（alias `phad::viz`）。公开依赖 `phad::common`、
`opencv_core`；`highgui` / `imgproc` 为私有依赖，避免把窗口能力泄漏到
只渲染内存画布的测试链接面。

## 职责边界

| 做 | 不做 |
|---|---|
| 俯视 x-y 轨迹面板渲染到 `cv::Mat` | ATE / 对齐（归 `phad::eval`） |
| 单个 highgui 窗口的显示与退出键 | 3D 场景、mesh、语义叠加 |
| 回放时标出「当前时刻」位置 | 离线出图脚本（归 `scripts/`） |

## 文件布局

| 文件 | 作用 |
|---|---|
| `trajectory_panel.hpp` / `.cpp` | 固定视野俯视面板（不打开窗口） |
| `image_window.hpp` / `.cpp` | highgui 窗口生命周期与 `pump` |

## 合同约定

### `TrajectoryPanel`

- 构造时用整条轨迹的水平包围盒定视野；回放中不缩放、不平移。
- x / y 同一比例，形状不被拉伸；竖直分量不参与投影。
- `render(timestamp)` 画出折线，并在不晚于该时刻的最后一个位姿上标当前位置。
- **只依赖 OpenCV 绘图**，单测只断言 `cv::Mat`，不弹窗。

### `ImageWindow`

- 集中 highgui：`show(canvas)` + `pump(wait_ms)`（`q` / Esc / 关窗 → false）。
- `wait_ms` 至少 1 ms（OpenCV 把 0 当作无限等待）。
- GUI 端到端（如 `phad_euroc_runner`）需人工确认；**agent 不擅自弹窗**。

### 分层用意

```text
TrajectoryPanel  ──►  cv::Mat（可测）
                         │
ImageWindow      ──►  显示 / 事件泵（可选）
```

需要 CI 覆盖的路径只依赖 Panel；窗口是 runner 的薄壳。

## 相关入口

| 位置 | 用途 |
|---|---|
| `apps/phad_euroc_runner` | 回放图像旁显示真值轨迹面板 |
| `tests/viz/` | 仅渲染内存画布 |
| `scripts/plot_*.py` | 离线静态图（venv，不进本库） |
