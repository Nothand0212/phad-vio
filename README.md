# phad-vio

`phad-vio` 是一个以学习和验证为首要目标、从零实现的 GTSAM-based
stereo visual-inertial odometry 项目。

项目不复刻任何单一开源实现，而是从 lk-vio、Tassel、OpenVINS、VINS-Fusion、
ORB-SLAM3 和 Basalt 各取一处经过验证的设计，按可测量的小里程碑实现一条完整的
VIO pipeline。每个里程碑先固定合同和验收标准，再编写实现，并以真实序列上的
ATE 作为出口条件。

## 当前状态（M4：最小 full-state VIO）

M1（数据加载）、M2（双目 VO 最小闭环）、M3.1（回归 benchmark）、M3.2（双目
配对同步器）和 M3.3（VO 加固）已完成。当前产品是最小 full-state VIO：单一
`VioEstimator::update()` 在同步图像时刻维护 pose `X(k)`、velocity `V(k)` 与
IMU bias `B(k)`。双目尺度已知、body≡IMU、滑窗 10，视觉短时丢失最多 500 ms
IMU coast。

**全量锚点**：[`c999f58` / `default_0337287b`](docs/benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md)。
EuRoC 11/11 均值 ATE ≈ **0.579 m**，段内加权 RMS ≈ **0.134 m**（相对
ORB-SLAM3 stereo 论文均值约 1.6×）。全局误差的主要来源是视觉中断后切段把
世界系重置到新原点。此后已合入视觉 overlap 不足时仍把已验证观测写入当前段
窗口的路径。

**下一步**：继续世界系连续（视觉中断后保住全局轨迹）；M5 正式动态初始化
排其后。

详细进展见 [`docs/design/roadmap.md`](docs/design/roadmap.md)。

## 技术路线

- 双目相机提供可观尺度，暂不支持单目
- 后端从第一天起就是 GTSAM 因子图；视觉侧的 stereo projection factor 与
  landmark 生命周期保持不变，IMU 只新增 `V(k)`、`B(k)`、`ImuFactor` 与
  bias random walk
- 完整状态为 pose、velocity 和 IMU bias
- 前端用 OpenCV、后端用 GTSAM；调库版本随后作为自研实现的对拍 oracle
- 边缘化、smart factor、线程和回环在真实 ATE 基线稳定之后才引入

### 设计的借鉴与不借鉴

| 项目 | 借鉴内容 | 不借鉴 |
|---|---|---|
| lk-vio | GFTT+LK 前端、滑窗后端、先 VO 后 IMU | — |
| Tassel | 事务式回环解耦、独立 viewer | 单目初始化 |
| OpenVINS | 轨迹仿真器与评估工具作为长期测试资产 | MSCKF 滤波架构 |
| VINS-Fusion | 视觉 SfM→惯性对齐初始化、关键帧策略 | — |
| ORB-SLAM3 | 初始化作为独立里程碑 | ATLAS 多地图 |
| Basalt | 边缘化零空间处理 | 完整 non-linear factor recovery 建图 |

## 快速开始

```bash
# 配置（GTSAM 4.3 须已在 CMAKE_PREFIX_PATH 中）
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j"$(nproc)"

# 测试
ctest --test-dir build

# 双目 VIO probe（MH_01 静止起步，2–5 min）
build/phad_stereo_vo_probe /path/to/euroc/MH_01_easy --tum est.tum --diag-csv diag.csv

# 评估 ATE
build/phad_traj_eval --est est.tum --gt-euroc /path/to/euroc/MH_01_easy

# 回归 benchmark（单序列；复现 M4 锚点时加 --estimator-enable-moving-bootstrap）
build/phad_vo_bench /path/to/euroc/MH_01_easy --bench-root /tmp/bench --sequence-name MH_01_easy --force
```

## 文档索引

完整地图（问题 / 寿命 / 命名）见 [`docs/README.md`](docs/README.md)。

| 目录 / 文档 | 回答的问题 |
|---|---|
| [`docs/design/`](docs/design/) | 系统是什么（roadmap / architecture / conventions） |
| [`docs/specs/`](docs/specs/) | 这一片必须交付什么 |
| [`docs/research/`](docs/research/) | 查到了什么（调研、诊断、历史切片笔记） |
| [`docs/plans/`](docs/plans/) | 怎么执行 |
| [`docs/benchmark/`](docs/benchmark/) | 跑出了什么数字（行为 checkpoint） |
| [`docs/adr/`](docs/adr/) | 为什么选 X |
| [`docs/agents/`](docs/agents/) | agent 怎么工作 |
| [`AGENTS.md`](AGENTS.md) | 根索引：原则、权威文档、模块表、现行事实 |
| [`CONTEXT.md`](CONTEXT.md) | 领域语言（传感器、标定、估计概念） |
| [`docs/historical-branch-evidence.md`](docs/historical-branch-evidence.md) | 历史分支与 evidence snapshot 入口 |

每个 `phad/` 子目录有 `README.md`（合同、边界、数据流）和 `AGENTS.md`（agent 提示）。
模块索引见根 [`AGENTS.md`](AGENTS.md) 的模块表格。

## 依赖

- **C++20**
- **OpenCV 4**（core、highgui、imgcodecs、imgproc、calib3d、video）
- **Eigen 3.4**（须与构建 GTSAM 时使用的同一份，避免 ODR）
- **yaml-cpp 0.8**
- **GTSAM 4.3**（系统/前缀安装；`find_package(GTSAM 4.3 REQUIRED)`）
- **nlohmann_json 3.11**
- **OpenSSL**（Crypto）
- **GoogleTest 1.14**（`PHAD_BUILD_TESTS=ON` 时）

已在本地 `thirdparty/`（gitignore）准备但尚未接入：spdlog 1.17.0。

### 安装 GTSAM 4.3

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

## 第一条实现原则

传感器数据进入估计器以前必须拥有无歧义的时间、坐标系和单位语义。任何无法
满足这些合同的数据都应显式报错，不能通过默认值、静默截断或坐标系猜测继续
运行。

## 参考

### 开源项目

| 项目 | 用途 |
|---|---|
| [lk-vio](https://github.com/Nothand0212/lk-vio) | 最小可行形态：GFTT+LK 前端、滑窗后端、DBoW 回环 |
| [Tassel](https://github.com/Ju-yzp/Tassel) | 回环事务式架构、选择性边缘化、独立 viewer |
| [OpenVINS](https://github.com/rpng/open_vins) | 轨迹仿真器与评估工具作为长期测试资产 |
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | 视觉 SfM→惯性对齐、关键帧策略、MARGIN_SECOND_NEW |
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | 初始化独立里程碑、`NeedNewKeyFrame` 决策 |
| [Basalt](https://github.com/VladyslavUsenko/basalt-mirror) | 边缘化零空间处理、VIO/mapping 分离评估 |
| [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO) | fixed-lag smoother、keyframe disparity 选择 |
| [GTSAM](https://github.com/borglab/gtsam) | 后端因子图库（`StereoVOExample`、`StereoFactor`、`PriorFactor`） |
| [gtsam-examples](https://github.com/gtbook/gtsam-examples) | `StereoVOExample_large` notebook |
| [evo](https://github.com/MichaelGrupp/evo) | ATE/RPE 交叉验证 oracle |

### 数据集

| 数据集 | 用途 |
|---|---|
| [EuRoC MAV](https://projects.asl.ethz.ch/datasets/euroc-mav/) | 11 条双目+IMU 序列基准 |
| [ETH ASL datasets](https://ethz-asl.github.io/datasets/) | TUM VI 等数据集入口 |

### 文献

| 文献 | 主题 |
|---|---|
| [Keyframe-based visual-inertial odometry using nonlinear optimization (IJRR 2015)](https://doi.org/10.1177/0278364915620033) | VINS 关键帧+非线性优化 VIO 架构 |
| [VINS-Fusion ar5iv 解读](https://ar5iv.labs.arxiv.org/html/1804.06120) | VINS 双目/单目融合 |
| [Basalt ICCV'21 边缘化论文](https://ar5iv.labs.arxiv.org/html/2303.11854) | square-root marginalization 与 nonlinear factor recovery |
| [OpenVINS 评估文档](https://docs.openvins.com/evaluation.html) | 轨迹评估流程 |
| [VINS-Fusion README](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/rosNodeTest.cpp) | 参数与启动说明 |
| [GTSAM StereoCamera](https://gtsam.org/doxygen/a05511.html) | 双目相机模型 |
| [GTSAM StereoFactor](https://gtsam.org/doxygen/a00458.html) | 双目投影因子 |

> 完整的设计对照、逐文件引用与决策依据见 `docs/research/*-open-source-refs.md`。
> 部分链接为 ar5iv 渲染（替代被墙的 arXiv 原文）。
