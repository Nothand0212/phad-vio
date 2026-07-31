# `phad::viz` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- 拆成可无显示测试的 `TrajectoryPanel`（俯视 x-y → `cv::Mat`）与薄的 `ImageWindow`（highgui）
- `tests/viz` 只渲染内存画布，不开窗口
- GUI 端到端（如 `phad_euroc_runner`）需人工确认；agent 不擅自弹窗
- 历史原因：`phad_viz` 把 `opencv_core` 设为 PUBLIC；**新库不要照做**，重依赖应 PRIVATE + PIMPL
