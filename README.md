# phad-vio

`phad-vio` 是一个以学习和验证为首要目标、从零实现的 GTSAM-based
stereo visual-inertial odometry 项目。

## 历史证据现场

此 checkout 对应 `m4-minimal-gyro-slice`。其已提交起点
`a8e892fcccbe061c5b69d7dabb150026ff19b341` 已进入后续产品历史；当前
dirty 内容保留最小 gyro-aided VO 的诊断、因子与测试续作。该路线的 natural
fused MH_01 结果曾出现 ATE/RPE 回退，因而这里的内容仅用于追溯问题、假设和
候选方案，不能直接迁入当前产品。

当前产品实现以 `main` 的 full-state `VioEstimator` 为准。任何后续复用须在
该边界重新实现并以当前门控重新验证。全仓分支定位见
`main:docs/historical-branch-evidence.md`。

项目不复刻任何单一开源实现，而是从 lk-vio、Tassel、OpenVINS、VINS-Fusion、
ORB-SLAM3 和 Basalt 各取一处经过验证的设计，按可测量的小里程碑实现一条完整的
VIO pipeline。每个里程碑先固定合同和验收标准，再编写实现，并以真实序列上的
ATE 作为出口条件。

## 当前状态（M4：最小 gyro 验证已收口）

M1（数据加载）、M2（双目 VO 最小闭环）、M3.1（回归 benchmark）、M3.2（双目
配对同步器）和 M3.3（VO 加固）已完成。M3.3 已消除 `num_shared==0` 永久拒帧
的吸收态，补齐 PnP 初值、外点剔除、多轮重优、skip-drop 和 zombie-age 生命周期
管理；当前最新工作是下述 M4 最小 gyro 验证。

**最新片：M4 最小 gyro-aided VO（#36）**。已在 production estimator 上实现
`off` / `shadow` / `fused`：off 与原 VO 三主产物逐字节一致，shadow 的 segment、
shared gyro bias 和 pose-only AHRS factor 机制成立；但 MH_01 natural fused 的
exact-common ATE 从 `0.080964 m` 恶化为 `0.090807 m`，1 s RPE 从
`0.017781 m` 恶化为 `0.018096 m`。因此本片以负结果停止，默认保持 `off`，
未扩跑 MH_05 或 EuRoC 11/11。

下一步不是自动继续实现：是否进入 full inertial `X/V/B + gravity` 需要另行评审。
结果与停止依据见
[`docs/research/m4-minimal-gyro-mh01-result.md`](docs/research/m4-minimal-gyro-mh01-result.md)。

详细进展见 [`docs/roadmap.md`](docs/roadmap.md)。

## 技术路线

- 双目相机提供可观尺度，暂不支持单目
- 后端从第一天起就是 GTSAM 因子图——VO 阶段建立的 stereo projection factor
  与 landmark 生命周期在接入 IMU 时完全不动
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

# 双目 VO probe（MH_01，2–5 min）
build/phad_stereo_vo_probe /path/to/euroc/MH_01_easy --tum est.tum --diag-csv diag.csv

# 评估 ATE
build/phad_traj_eval --est est.tum --gt-euroc /path/to/euroc/MH_01_easy

# 回归 benchmark（单序列）
build/phad_vo_bench /path/to/euroc/MH_01_easy --bench-root /tmp/bench --sequence-name MH_01_easy --force
```

## 文档索引

### 项目级

| 文档 | 内容 |
|---|---|
| [`docs/roadmap.md`](docs/roadmap.md) | 里程碑、出口条件、全序列基准数字 |
| [`docs/architecture.md`](docs/architecture.md) | 模块职责、数据流、目标架构 |
| [`docs/conventions.md`](docs/conventions.md) | 坐标系、时间、单位合同 |
| [`CONTEXT.md`](CONTEXT.md) | 领域语言（传感器、标定、估计概念） |

### 设计与诊断

| 目录 | 内容 |
|---|---|
| [`docs/research/`](docs/research/) | 设计文档、根因诊断、开源对照（~40 篇） |
| [`docs/plans/`](docs/plans/) | 实施计划（YAML frontmatter + 可执行步骤） |
| [`docs/benchmark/`](docs/benchmark/) | 关键 checkpoint 的全量 EuRoC 快照与对比 |
| [`docs/adr/`](docs/adr/) | 架构决策记录 |

### Agent 约定

| 文档 | 内容 |
|---|---|
| [`AGENTS.md`](AGENTS.md) | 项目级 agent 偏好、learned facts、模块索引 |
| [`docs/agents/`](docs/agents/) | 增量开发、issue 管理、C++ 命名/风格、Git 工作流 |

### 模块

每个 `phad/` 子目录有 `README.md`（合同、边界、数据流）和 `AGENTS.md`（agent 提示）。
模块索引见根 [`AGENTS.md`](AGENTS.md) 的模块表格。

## 依赖

- **OpenCV 4**（core、highgui、imgcodecs、imgproc、calib3d、video）
- **Eigen 3.4**（须与构建 GTSAM 时使用的同一份，避免 ODR）
- **yaml-cpp 0.8**
- **GTSAM 4.3**（系统/前缀安装；`find_package(GTSAM 4.3 REQUIRED)`）
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
