# phad-vio

`phad-vio` 是一个以学习和验证为首要目标、从零实现的 GTSAM-based
stereo visual-inertial odometry 项目。

项目不以复刻 Kimera-VIO 为目标，而是借鉴它清晰的前端/后端数据流，
按可验证的小阶段实现一条完整 VIO pipeline。每个阶段先固定合同和验收
标准，再编写实现。

## 当前状态

当前已完成数据集 adapter 基础：

- `phad::io::dataset::StereoImuDataset` 提供统一的双目、IMU 与标定
  interface；
- EuRoC 与 TUM VI 目录和标定格式由独立 adapter 解析；
- EuRoC 8-bit 与 TUM VI 16-bit 灰度图均按需解码并保持原始像素类型；
- `phad_euroc_inspect` 可检查 EuRoC 数据摘要和抽样图像；
- 默认测试使用自生成 fixture，真实数据集测试通过 CMake option 显式启用。

后续 VIO 方向：

- 双目相机提供可观尺度；
- IMU 与左相机时间戳对齐；
- 后端状态为 pose、velocity 和 IMU bias；
- 后端使用 GTSAM；
- 第一版使用 batch optimization 和显式 landmark；
- 在基本视觉惯性图验证后，再引入 fixed-lag smoother、smart factor 和线程。

尚未确定的事项集中记录在各文档的“待确认事项”小节中。未确认项不得在
实现中被悄悄假设。

## 文档索引

- [坐标系、时间与单位合同](docs/conventions.md)
- [目标架构与模块职责](docs/architecture.md)
- [实现阶段与验收标准](docs/roadmap.md)
- [ADR-0001：采用 GTSAM 构建 VIO 后端](docs/adr/0001-gtsam-vio-backend.md)

## 第一条实现原则

传感器数据进入估计器以前必须拥有无歧义的时间、坐标系和单位语义。
任何无法满足这些合同的数据都应显式报错，不能通过默认值、静默截断或
坐标系猜测继续运行。

## 依赖

- GTSAM 4.3a1
- spdlog 1.17.0
