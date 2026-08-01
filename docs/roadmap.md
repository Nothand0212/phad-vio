# 实现里程碑与验收标准

本路线以「先跑通端到端可观察行为，再按指标演进」为原则：先用成熟库把
一条最短的双目 VO 闭环打通并测出 ATE，再逐层加入 IMU、初始化、边缘化
和回环。里程碑完成指实现、定向测试和本文档中的出口条件全部满足，不以
「代码已写」代替完成。

## 总原则

1. **每个里程碑以一个可填进表格的数字，或一条在 viewer 中可见的行为
   收尾。** 无法被外部观察的内部机制不单独构成里程碑。
2. **评估先于估计。** ATE/RPE 与轨迹导出在第一条估计轨迹之前就绪，
   否则「这次改动让系统变好还是变坏」无法回答。
3. **合成数据只作为测试资产，不作为里程碑。** 预积分与投影的解析验证
   写成单元测试，不占据独立阶段。
4. **调库优先。** 前端用 OpenCV，后端用 GTSAM。调库版本随后成为手写
   版本的 oracle：任何自研替换的验收标准是「同一序列上 ATE 与调库版本
   在容差内一致，且 track 数、内点率、重投影误差分布可比」。
5. **前端风险最先出清。** 决定 VIO 能否工作的是特征跟踪与初始化，
   不是后端公式；因此 LK frontend 属于第二个里程碑而非最后一个。
6. **后端从第一天就是 GTSAM 因子图。** VO 阶段建立的 stereo projection
   factor 与 landmark 生命周期在接入 IMU 时完全不动，加入 IMU 只是新增
   `V(k)`、`B(k)` 变量与 `ImuFactor`、`BetweenFactor<ConstantBias>`。

## 参考项目的定位

各参考项目回答本路线中的一个具体问题，不做整体复刻：

| 项目 | 借鉴内容 |
|---|---|
| [lk-vio](https://github.com/Nothand0212/lk-vio) | 最小可行形态：GFTT + LK 前端、滑窗后端、DBoW 回环；先 VO 后 IMU 的顺序 |
| [Tassel](https://github.com/Ju-yzp/Tassel) | 回环彻底解耦的事务式架构；选择性边缘化；早期就建独立 viewer 模块 |
| [OpenVINS](https://github.com/rpng/open_vins) | 把轨迹仿真器与评估工具当作长期测试资产，而非一次性阶段 |
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | 视觉 SfM → 惯性对齐的初始化；在线 time offset 与外参估计 |
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | 初始化作为独立里程碑；MAP-based inertial-only 初始化 |
| [Basalt](https://github.com/VladyslavUsenko/basalt-mirror) | 边缘化的零空间处理；VIO 与 mapping 分离并各自独立评估 |

明确不借鉴：OpenVINS 的 MSCKF 滤波架构（与 ADR-0001 冲突）、ORB-SLAM3
的 ATLAS 多地图、Basalt 的完整 non-linear factor recovery 建图、Tassel
的单目初始化路径。

## M1：离线 Stereo-IMU 数据加载（已完成）

范围：

- CMake 骨架与 GoogleTest 框架；
- `phad::sensor` 的 timestamp、IMU measurement、camera parameters、
  IMU parameters、rigid transform 与 stereo-IMU calibration 类型；
- `phad::camera` 的运行时 camera model（`project` / `backProject`）；
- `phad::io::dataset` 下的 `StereoImuDataset` 值类型，以及 EuRoC、
  TUM VI 两个 adapter；
- 整数纳秒时间戳、cam0/cam1 exact join、全清单校验、uint8/uint16
  惰性解码；
- 来源无关的 pull-based `SensorSource` seam 与 `DatasetReplaySource`；
- `phad_euroc_inspect` 数据集摘要工具。

出口（已满足）：

- 从原生 `MH_01_easy` 与 TUM VI `corridor1_512_16` 确定性取得标定、
  已审计 summary 与独立顺序 reader；
- adapter 之外不存在 EuRoC/TUM VI 路径、标定字段、坐标或单位转换；
- 常驻内存不随已遍历图像总数线性增长。

详细设计与证据见
[EuRoC 数据集加载器设计调研](research/euroc-dataset-loader-design.md)。

## M2：双目 VO 最小闭环

本里程碑的目标是拿到**第一个真实 ATE 数字**，不追求精度。拆为三个各自
产出可观察行为的小步。

### M2.1 评估与可视化底座（已完成）

范围：

- EuRoC `state_groundtruth_estimate0/data.csv` 真值轨迹加载，含
  timestamp 单位与外参方向的显式处理；
- TUM 格式（`timestamp tx ty tz qx qy qz qw`）轨迹导出；
- 估计轨迹与真值的时间关联（最近邻 + 最大时间差阈值）；
- SE3 对齐（Umeyama，双目尺度已知故不估 scale）与 ATE RMSE；
- 固定时间间隔的 RPE（默认 1 s）；距离间隔 RPE 有意不做，后续按需再加；
- OpenCV highgui 实时 2D 面板：俯视 x-y 轨迹叠加真值、图像窗口；
- Python 离线脚本：3D 轨迹、误差随时间曲线。

测试：

- **真值自比**：把真值当作估计输入，ATE 必须为 0（数值容差内）；
- 已知刚体扰动：对真值施加固定 SE3 后，对齐结果应恢复该变换且 ATE
  回到 0；
- 时间关联失败路径：估计时间戳超出真值范围、间隔过大、数量为零；
- 与 `evo` 的交叉验证（离线，非 CI）：同一对轨迹的 ATE RMSE 一致。

出口（已满足）：

- 真值自比 ATE 为 0；
- 轨迹文件可被 `evo` 直接读取；
- 实时面板可在回放 `MH_01_easy` 时显示真值轨迹与当前图像。

实施计划见
[M2.1 评估可视化底座](plans/2026-07-30_m2.1_eval_visualization_baseline_d818d653.plan.md)。

### M2.2 双目前端（已完成）

范围：

- 前置整幅立体校正（`StereoRectifier` + `RectifiedStereoCalibration`）；
- `goodFeaturesToTrack` 特征检测与按 track 长度涂 mask 补点；
- 左目时序 `calcOpticalFlowPyrLK` 与右目每帧重匹配；
- 前后向光流一致性与几何门限（行差 / 视差 / 深度）；
- 稳定且不复用的 `LandmarkId`；
- track 叠加可视化与无窗口 probe CSV。

指标与测试：

- track 数量随时间曲线，全序列不出现归零；
- track 长度分布（`--tracks-csv` 生命表）；
- 左右目匹配的 epipolar error 分布；
- 前后向一致性剔除率；
- `LandmarkId` 不复用；
- 低纹理与快速旋转片段的行为被记录，而不是被静默处理。

出口（已满足）：

- `MH_01_easy` 全序列跟踪不中断（门控 `phad_frontend_mh01_test`）；
- 上述指标可从 `phad_stereo_frontend_probe` 输出中直接读出；
- 尚不产出位姿。

实施计划见
[M2.2 双目前端](plans/2026-07-31_m2.2_stereo_frontend_38ddaa97.plan.md)。

### M2.3 VO 后端（已完成）

范围：

- GTSAM `Cal3_S2Stereo` 与 `GenericStereoFactor`；
- `StereoCamera::backproject` 建立 landmark 初值（单帧立体等价路径）；
- 显式 landmark 的固定窗口 batch BA，`LevenbergMarquardtOptimizer`；
- 窗口滑出的状态与 landmark 直接丢弃，不做边缘化；
- 单一明确的 gauge-fixing prior（最老帧 `PriorFactor<Pose3>`）。

测试：

- 合成轨迹与 landmark：扰动初值后恢复 pose 与 landmark；
- 优化后重投影 error 下降；
- behind-camera landmark 显式报告 cheirality；
- 错误外参用例产生显著位姿误差（仍为 `kOk`，不升格 `kFailed`）；
- 按 landmark id 可汇总完整 feature track。

出口（已满足）：

- `MH_01_easy` 跑完不崩，`phad_stereo_vo_probe` 写出完整 TUM（3682 / 3682
  `kOk`）；
- ATE translation RMSE **0.150 m**（`< 5 m` 米级门控）；RPE(1 s) translation
  RMSE 0.022 m；
- 拒帧比例 **0**；`low_connectivity` 帧数 **0**；重投影 RMS after 中位数
  **0.387 px**、p95 **1.091 px**；
- 轨迹形状需在 `phad_euroc_runner` 上人工确认（agent 不擅自弹窗）。

该组数字为后续里程碑的对比基线。设计见
[M2.3 VO 后端设计](research/m2.3-vo-backend-design.md)，开源对照见
[M2.3 VO backend open source references](research/m2.3-vo-backend-open-source-refs.md)。
实施计划见
[M2.3 VO 后端](plans/2026-07-31_m2.3_vo_backend_dcdbfc71.plan.md)。

## M3：让 VO 变稳

M3 拆成三步：**先有可复现对照表，再修数据入口，最后谈加固**。没有
benchmark，无法判断后续改动是优化还是退化；而 11 条 EuRoC 序列里有 3 条
在 `open()` 阶段就打不开，多序列对照表也无从谈起。因此 M3.1 阻塞 M3.2，
M3.2 阻塞 M3.3。

### M3.1 回归 Benchmark（已完成）

范围：

- 纯逻辑库 `phad::bench`（run 身份、config 快照与 hash、路径模板、
  summary schema），零 `phad::*` 依赖；
- `apps/offline_vo_session` 静态库（只跑 pipeline，不算 ATE/RPE、不落盘）；
- `phad_stereo_vo_probe` 与 `phad_vo_bench` 共用 session；
- `phad_vo_bench` 作为 composition root：session + `phad::eval` + 落盘；
- run 身份：运行时 git（dirty 只看 tracked）+ 完整 `config` 快照 +
  `config_hash`（FNV-1a 64 前 8 hex）；
- 路径模板：
  `<bench_root>/<sequence>/<commit_short>[_dirty]/<config_label>_<hash8>/`；
- `summary.json`：ATE / RPE(1 s) trans RMSE、completion_rate、coverage_rate、
  wall-clock / RTF；附带拒帧与阶段 timing；
- `scripts/bench_table.py` 把多个 `summary.json` 拼成 Markdown/CSV 对比表。

出口（MH_01_easy 复跑）：

- probe 迁移前后 `est.tum` / `diag.csv` 逐字节相同；
- `phad_vo_bench` ATE translation RMSE ≈ **0.150155 m**（与 M2.3 基线
  ≈ 0.150 m 一致；`phad_traj_eval` 交叉验证同值）；
- `completion_rate = 1.0`，`coverage_rate = 1.0`；
- clean 树无 `--force` 拒绝覆盖（exit 3）；dirty 树覆盖并警告。

v1 **只钉 MH_01**、只支持 EuRoC；多序列矩阵拆给 M3.2（先让 11 条都能打开）
与 M3.3（序列 × 版本质量对比）。设计见
[M3.1 VO 回归 Benchmark 设计](research/m3.1-vo-regression-benchmark-design.md)，
实施计划见
[M3.1 VO 回归 Benchmark](plans/2026-07-31_m3.1_vo_regression_benchmark_7c4e91a2.plan.md)。
Issue：[#21](https://github.com/Nothand0212/phad-vio/issues/21)。

### M3.2 双目配对同步器（已完成）

M3.1 的全序列 bench 只有 8/11 可跑：`MH_04_difficult`、`V1_02_medium`、
`V2_03_difficult` 在 `open()` 阶段即失败。已核实这不是本地数据损坏，而是
官方 ASL 与 rosbag 的左右清单本身不对称，而旧 `joinStereo` 要求「等长 +
下标 exact」，比任何开源实现都严。修法不是放松 loader，而是把左右配对从
adapter 移到独立的同步器——那里同时是 M4 的 IMU 包络与未来 ROS 输入的落点，
避免日后出现第二套配对策略。

范围：

- 新库 `phad::sync` 的 `StereoPairSynchronizer`（StereoOnly）：左右两队列
  比较 front，`|tL - tR| <= tol_ns` 则配对，否则丢弃偏早一侧并计数；默认
  `tol_ns = 0`（exact），soft 容差仅显式配置；输出用 left stamp；只提供
  `tryPop()`，不提供回调；有界队列与 drop-oldest 溢出显式计数；
- `phad::sensor` 新增 `CameraId` 与 `ImageFrameEvent`；`io::SensorEvent`
  的 `StereoFrame` 换成 `ImageFrameEvent`；
- dataset 改左右分路清单与 per-camera reader API
  （`peekImageTimestamp` / `takeImage`），`summary` 由 `stereo` 拆成
  `left` / `right`；`joinStereo` 与 `kStereoTimestampMismatch` 删除；
- `DatasetReplaySource` 按时间归并单路事件（同 stamp 顺序 IMU → Left →
  Right）；
- `apps/StereoPairStream`（source + sync 薄组合）供 `OfflineVoSession`、
  两个 probe、GUI runner 与真序列测试复用；`io` 与 `sync` 互不依赖。

不做：`pushImu`、IMU 边界插值与 `StereoImuPacket`（M4）；ROS adapter；
默认开启 soft 容差；frontend/estimator 算法改动。

测试：

- 纯内存单测：等长 exact 全配对；左首 orphan、右尾 orphan、大量右多余
  三型（断言配对集合等于时间戳交集，无下标错配）；交错 push 顺序；
  `tol_ns = 0` 拒绝 1 ns 差；单相机逆序/重复 → sticky error；有界队列
  溢出计数；`flush()` 把两侧全部剩余计入丢弃；
- adapter 回归：合成不等长序列 `open` 成功；单路非法（逆序、重复、缺
  PNG、坏 CSV/YAML）仍硬失败；TUM VI 等长路径不变；
- 诊断：`pushed_*`、`emitted_stereo`、`dropped_*`、`dropped_*_overflow`、
  `max_*_queue`，结束一条 summary，首次丢弃与首次溢出各 warning 一次。

出口（commit `4780660` 复跑）：

- `MH_01_easy` 的 `est.tum` 与 `diag.csv` 相对 M3.1 基线
  （`2b28616/default_0885385a`）**逐字节相同**，ATE 仍 ≈ 0.150155 m；
- 三条原失败序列 `open` 成功并产出 `summary.json`，`summary.sync` 精确匹配：
  `MH_04` emit 2032 / drop_left 1；`V1_02` emit 1710 / drop_right 1；
  `V2_03` emit 1921 / drop_left 1 / drop_right 415；
- 11 条 EuRoC 序列全部产出 `summary.json`（`scripts/bench_table.py` 可刷
  序列 × 版本表）；表中 VO 质量差（如 `V2_03` completion ≈ 0.03、
  `MH_02` / `V1_03` / `V2_02` 低 completion）归 M3.3。
- 全序列数字快照见
  [M3.2 EuRoC 全序列基线](research/m3.2-euroc-baseline.md)（钉 `4780660`）。

设计见
[Stereo Pair Synchronizer 设计](research/stereo-pair-synchronizer-design.md)，
根因诊断见
[EuRoC 双目 manifest 不等长 handoff](research/euroc-stereo-manifest-asymmetry-handoff.md)，
开源对照见
[EuRoC stereo manifest asymmetry open source refs](research/euroc-stereo-manifest-asymmetry-open-source-refs.md)。
实施计划见
[M3.2 双目配对同步器](plans/2026-07-31_m3.2_stereo_pair_synchronizer_5b7d1c93.plan.md)。
Issue：[#22](https://github.com/Nothand0212/phad-vio/issues/22)。

### M3.3 VO 加固（依赖 M3.2；Slice ① 已完成）

M3.2 全序列基线暴露的主导失败不是精度，而是一个**吸收态**：估计器一旦因
`num_shared == 0` 拒帧，事务回滚会冻结窗口与 landmark 表，而前端跟踪断裂后
重新检测会发放全新 `LandmarkId`，此后交集恒为空、永久拒帧。V1_03 上一帧异常
废掉了随后 1906 帧。因此本阶段按失败驱动排序，恢复先于精度。

范围（五片，Slice ① 已落地；②–⑤ 的范围按 ① 的 bench 数字推进）：

- **① 恢复（已完成）**：去掉 `num_shared == 0` 拒帧门，重叠断裂即以新 id 重建窗口，
  prior 打在最后一个被接受的位姿上（anchor 复用现有恒速位姿初值规则）；
  新增 `min_seed_observations`（初始化与 re-anchor 共用）与 `enable_reanchor`；
  `segment_id` 进 `diag.csv`，`reanchors` / `segments` 进 `summary.json`；
- **②** 前端右目匹配加固（约 25% `no_right_match` 的全局常态、坏视差写回
  `last_disp_px` 的种子中毒、新 track 零种子、单帧 LK 全灭）；
- **③** `solvePnPRansac` 位姿初值取代恒速初值 + 几何验证剔时序外点；
- **④** 优化后 chi2 外点剔除与 landmark 生命周期；
- **⑤** 关键帧策略（视差、跟踪数、时间间隔）。会改 `est.tum` 的输出语义且与 M4 的
  IMU 预积分边界耦合，开工前单独对齐。

Slice ① 出口（commit `0b0cd34` / `default_030a0197`，对照 `4780660`）：

- `MH_01_easy`：`segments=1`、`reanchors=0`；`est.tum` 相对
  `4780660/default_0885385a` **逐字节相同**；ATE 仍 ≈ 0.150155 m；
- completion 显著恢复：`MH_02` 0.131→0.997、`V1_03` 0.113→0.993、
  `V2_02` 0.113→0.980、`V2_01` 保持 0.952；`V2_03` 0.032→0.485（不门控）；
- `V1_02` completion 由 1.000 降至 0.946：新增的首段 seed 门限（默认
  `min_seed_observations=10`）拒掉了前 93 帧观测不足的帧；这是有意的
  门限行为，不是回归。真正对齐 M3.2 的 A/B 需要**同时**设
  `enable_reanchor=false` **与** `min_seed_observations=1`（只关
  `enable_reanchor` 不够，因为首段初始化也共用该门限）；
- ATE **只记录不门控**；跨 coverage 不可直接比较（如 V1_03 / V2_02）；
- 全序列数字快照见
  [M3.3 Slice ① 基线](research/m3.3-slice1-baseline.md)。

后续切片出口（不变）：

- 每次改动用 `phad_vo_bench` 产出前后数字；无法测量的改动不进入本阶段；
- **同等 coverage 下 ATE 不劣化**，`MH_01` 作不回归锚（不劣于 0.150155 m）；
- Slice ② 优先盯 `reanchors` 次数与每帧观测数，而不是再修吸收态。

设计见
[M3.3 VO 加固设计](research/m3.3-vo-hardening-design.md)，根因诊断见
[M3.3 VO 崩溃根因诊断](research/m3.3-vo-collapse-diagnosis.md)，开源对照见
[M3.3 VO 加固：开源实现对照](research/m3.3-vo-hardening-open-source-refs.md)。
实施计划见
[M3.3 VO 加固](plans/2026-07-31_m3.3_vo_hardening_a3f7d2e9.plan.md)。
Issue：[#23](https://github.com/Nothand0212/phad-vio/issues/23)。

## M4：接入 IMU

范围：

- IMU 输入校验与 bias 类型；
- GTSAM preintegration 参数构造，噪声单位转换只在此处发生一次；
- `PreintegratedCombinedMeasurements`；
- 在 M3.2 的 `StereoPairSynchronizer` 上扩展 `pushImu`：图像边界 IMU 线性
  插值与 `StereoImuPacket` 构造；队列长度限制与溢出报告沿用 M3.2，不新建
  synchronizer；
- 状态由 `X` 扩展为 `X/V/B`，新增 `ImuFactor` 与
  `BetweenFactor<ConstantBias>`；
- 起始静止初始化（收集静止段 → gyro bias → 重力方向 → roll/pitch →
  零初速 → 建立 priors）。

测试：

- 预积分解析验证作为单元测试：静止、匀速、恒定角速度、已知 bias 的
  first-order correction 与重新积分一致；
- rad/s 与 deg/s 错用能被测试发现；
- covariance 对称且特征值在容差内非负；
- packet 的 IMU \(\sum\Delta t\) 等于图像时间间隔；
- 图像时刻恰好落在 IMU 样本上、落在两样本之间、缺样、重复、逆序、
  大间断；
- 失败路径：逆序时间戳、非有限测量、非正 \(\Delta t\)、非法 noise 配置；
- 静止检测成功与运动中误初始化拒绝。

出口：

- `MH_01` 的 ATE 优于 M3.3 的纯 VO 结果；
- 视觉短时退化时轨迹连续，不出现跳变或发散；
- IMU bias 能从扰动初值收敛；
- 视觉与 IMU 时间错位用例产生可检测的误差增大。

## M5：正式初始化

在静止初始化的局限被真实序列暴露后再开始。双目尺度已知，核心是 gyro
bias、重力方向与初始速度。

范围（择一，决定时补 ADR）：

- 视觉 SfM → 顺序惯性对齐（VINS-Fusion 路线）；
- MAP-based inertial-only 优化（ORB-SLAM3 路线，更稳但更重）。

出口：

- TUM VI room 等手持起步、无静止段的序列可完成初始化；
- 初始化失败返回原因与所需的下一步数据，不返回 identity pose 冒充成功；
- 初始化后 ATE 不劣于静止初始化在 `MH_*` 上的结果。

## M6：边缘化与 fixed-lag

范围：

- `IncrementalFixedLagSmoother` 与 variable timestamps；
- 旧状态边缘化，bounded state 与 factor 生命周期；
- 参考 Basalt 的零空间处理与 Tassel 的选择性边缘化处理静止段。

出口：

- 长回放中状态数量保持有界，内存不随帧数线性增长；
- 边缘化时轨迹无不合理跳变，bias 与 velocity 连续；
- 与短片段 batch 结果在规定容差内一致；
- 静止状态下速度不漂移。

## M7：性能与线程

范围：

- `SmartStereoProjectionPoseFactor`（与显式 landmark 版本对照后决定是否
  采用）；
- frontend/backend bounded queues；
- lifecycle、shutdown 与不可变估计快照。

出口：

- `MH_01` 实时率 ≥ 1x；
- 单线程与多线程在确定性输入上的结果差异可解释；
- 队列满时显式背压或报错，后台异常不会变成假成功；
- shutdown 不丢失已接受但未处理的数据；
- ThreadSanitizer 或等价检查覆盖关键共享路径。

## M8：解耦回环与全局位姿图

采用 Tassel 的事务式架构：估计器提交关键帧与历史路标事务，回环组件在
独立任务中完成检索、几何验证与位姿图优化；全局轨迹接受修正，滑窗状态、
边缘化先验与既有线性化点**永不回写**。

范围：

- 关键帧描述子检索与候选管理；
- 几何验证（历史深度路标 PnP）；
- 独立的关键帧 global pose graph，odometry 与 loop constraints；
- world/odometry 校正变换。

出口：

- 关闭回环时局部 VIO 行为完全不变；
- 错误回环可拒绝，仅图像相似度不写入位姿图；
- 含回环序列的全局 ATE 下降；
- global correction 不破坏 fixed-lag graph 中 \(X/V/B\) 的一致性。

## M9：研究平台

在前述里程碑给出稳定基线后，才引入面向实验的可替换性。

范围：

- 在线 camera–IMU time offset 与外参标定；
- 算法可替换 seam（第二个真实实现出现后才提取）；
- 配置化实验与 benchmark 脚本；
- 全序列回归表的自动生成。

出口：

- 更换单个算法模块不需要修改其他模块；
- 一条命令产出全部目标序列的 ATE/RPE 表。

## 贯穿活动：手写深挖

任意模块在拥有调库 ATE 基线之后，可展开一次自研实现，作为学习活动而非
独立里程碑。验收标准固定为：

- 同一序列上 ATE 与调库版本在容差内一致；
- 中间量（track 数、内点率、重投影误差、covariance）分布可比；
- 差异超出容差时，能定位到具体的公式或实现分歧。

不满足对拍的自研实现不替换调库版本，两者可并存直到差异被解释清楚。

## 暂缓项

在 M8 完成前不排入实现：

- 单目支持与单目尺度初始化；
- rolling shutter；
- 多相机（两目以上）；
- GNSS、轮速计或 LiDAR 融合；
- 稠密地图、mesh 和语义；
- 自研非线性求解器；
- 实机与 ROS 2 接入。
