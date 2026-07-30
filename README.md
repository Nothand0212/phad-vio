# phad-vio

`phad-vio` 是一个以学习和验证为首要目标、从零实现的 GTSAM-based
stereo visual-inertial odometry 项目。

项目不复刻任何单一开源实现，而是从 lk-vio、Tassel、OpenVINS、
VINS-Fusion、ORB-SLAM3 和 Basalt 各取一处经过验证的设计，按可测量的小
里程碑实现一条完整 VIO pipeline。每个里程碑先固定合同和验收标准，再编写
实现，并以真实序列上的 ATE 作为出口条件。

## 当前状态

M1（离线 stereo-IMU 数据加载）已完成：

- `phad::io::dataset::StereoImuDataset` 提供统一的双目、IMU 与标定
  interface；
- EuRoC 与 TUM VI 目录和标定格式由独立 adapter 解析；
- EuRoC 8-bit 与 TUM VI 16-bit 灰度图均按需解码并保持原始像素类型；
- `phad::sensor` 提供标定与测量类型，`phad::camera` 提供运行时相机模型；
- `SensorSource` / `DatasetReplaySource` 提供来源无关的事件回放 seam；
- `phad_euroc_inspect` 可检查 EuRoC 数据摘要和抽样图像；
- 默认测试使用自生成 fixture，真实数据集测试通过 CMake option 显式启用。

下一个里程碑 M2 是**双目 VO 最小闭环**：先建立评估与可视化底座
（TUM 格式轨迹导出、ATE/RPE、实时 2D 面板），再接入 OpenCV LK 前端和
GTSAM 因子图后端，在 `MH_01_easy` 上产出第一条轨迹和第一个 ATE 数字。
该里程碑不追求精度，只建立后续所有改动的基线。

技术路线：

- 双目相机提供可观尺度，暂不支持单目；
- 后端从第一天起就是 GTSAM 因子图，因此 M4 接入 IMU 时视觉因子不变；
- 完整状态为 pose、velocity 和 IMU bias；
- 前端用 OpenCV、后端用 GTSAM，调库版本随后作为自研实现的对拍 oracle；
- 边缘化、smart factor、线程和回环在真实 ATE 基线建立之后才引入。

尚未确定的事项集中记录在各文档的“待确认事项”小节中。未确认项不得在
实现中被悄悄假设。

## 文档索引

- [坐标系、时间与单位合同](docs/conventions.md)
- [目标架构与模块职责](docs/architecture.md)
- [实现里程碑与验收标准](docs/roadmap.md)
- [ADR-0001：采用 GTSAM 构建 VIO 后端](docs/adr/0001-gtsam-vio-backend.md)

## 第一条实现原则

传感器数据进入估计器以前必须拥有无歧义的时间、坐标系和单位语义。
任何无法满足这些合同的数据都应显式报错，不能通过默认值、静默截断或
坐标系猜测继续运行。

## 依赖

当前构建所需：

- OpenCV 4（`core`、`highgui`、`imgcodecs`、`imgproc`）
- Eigen 3.4
- yaml-cpp 0.8
- GoogleTest 1.14（`PHAD_BUILD_TESTS=ON` 时）

已在本地 `thirdparty/`（gitignore）准备、但尚未接入 `CMakeLists.txt`，
将在 M2.3 引入后端时接入：

- GTSAM 4.3a1
- spdlog 1.17.0
