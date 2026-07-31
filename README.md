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

M2.1（评估与可视化底座）已完成：

- `phad::eval` 提供 TUM 轨迹读写、时间关联、固定尺度 SE3 对齐与 ATE、RPE；
- `phad_euroc_gt_export` 导出 EuRoC 真值，`phad_traj_eval` 比较两条轨迹并可
  写出逐样本误差 CSV；两者的数字与 `evo` 在六位有效数字上一致；
- `phad::viz` 提供俯视 x-y 轨迹面板与显示窗口，`phad_euroc_runner` 回放时在
  图像旁显示真值轨迹与当前位置；
- `scripts/` 下的 Python 脚本消费 TUM 与误差 CSV，产出 3D 轨迹与误差曲线
  （见 [离线绘图脚本](scripts/README.md)）。

M2.2（双目前端）与 M2.3（VO 后端）已完成：`phad_stereo_vo_probe` 在
`MH_01_easy` 上写出完整估计轨迹；ATE translation RMSE 约 0.15 m（基线，
不追求精度）。详见 [roadmap M2.3](docs/roadmap.md)。

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
- [离线绘图脚本](scripts/README.md)

## 第一条实现原则

传感器数据进入估计器以前必须拥有无歧义的时间、坐标系和单位语义。
任何无法满足这些合同的数据都应显式报错，不能通过默认值、静默截断或
坐标系猜测继续运行。

## 依赖

当前构建所需：

- OpenCV 4（`core`、`highgui`、`imgcodecs`、`imgproc`、`calib3d`、`video`）
- Eigen 3.4（须与构建 GTSAM 时使用的同一份 Eigen，避免 ODR 问题）
- yaml-cpp 0.8
- GTSAM 4.3（`find_package(GTSAM 4.3 REQUIRED)`；本仓库用系统/前缀安装，不
  把源码放进 git）
- GoogleTest 1.14（`PHAD_BUILD_TESTS=ON` 时）

已在本地 `thirdparty/`（gitignore）准备、但尚未接入 `CMakeLists.txt`：

- spdlog 1.17.0

### 安装 GTSAM 4.3

`thirdparty/` 被 gitignore，不能默认别人机器上已有安装。推荐从
[borglab/gtsam](https://github.com/borglab/gtsam) 的 4.3 发布标签构建并安装到
`/usr/local`（或任意 CMAKE_PREFIX_PATH）：

```bash
git clone --depth 1 --branch 4.3a1 https://github.com/borglab/gtsam.git
cmake -S gtsam -B gtsam/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
cmake --build gtsam/build -j"$(nproc)"
sudo cmake --install gtsam/build
```

要点：

- `GTSAM_USE_SYSTEM_EIGEN=ON`，与本仓库的 `Eigen3::Eigen` 3.4 对齐；
- 若 GTSAM 带 `-march=native` 构建，跨机器分发可能有对齐/ABI 问题；本项目
  当前只在本机构建与验收；
- 配置本仓库时若 GTSAM 不在默认前缀：  
  `cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/gtsam/prefix`。
