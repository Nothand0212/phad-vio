# M5 立体视觉–惯性初始化与视觉中断恢复

日期：2026-08-26

状态：Research complete。本文只为 M5 实施提供依据，不授予编码、配置改默认或全序列资格化权限。

工作区：`/home/lin/orca/workspaces/m4-minimal-gyro/tigerfish`  
对照 commit：[`c999f5891d8efbc353e2e4001df7656a9e19907a`](https://github.com/Nothand0212/phad-vio/commit/c999f5891d8efbc353e2e4001df7656a9e19907a)（branch `Nothand0212/m4-online-gyro-bias-synthetic-replay2`）  
EuRoC 锚点：[`docs/benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md`](../benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md)

证据分级：

- **论文明确声明**：作者在论文中写出的算法、变量或实验数字。
- **开源实现的实际行为**：官方仓库在钉死 commit 上的代码路径。
- **基于资料的工程推断**：把上游做法映射到 phad-vio 现有 `VioEstimator` 合同时的判断。

---

## A. 一页式结论

### 推荐路线

在现有单一 `VioEstimator::update()` 内，把当前 moving bootstrap 升级为 **立体度量位姿上的顺序惯性对齐**，并在同一 helper 上做视觉中断后的再初始化：

1. **证据窗**：在 `m_initialized == false` 期间继续用已有立体观测、PnP 与滑窗长出一条 **尺度已知** 的短轨迹，并保存相邻帧的 raw IMU interval；此阶段只返回 `kInitializing`，不提交成功的 `X/V/B`。
2. **陀螺仪 bias**：用视觉相对旋转与预积分旋转 Jacobian 做线性最小二乘（VINS / Kimera / Mur-Artal 同一残差），得到常值 \(\mathbf{b}_g\)，再按新 bias **重积**全部 PIM。
3. **重力方向 + 每帧速度**：在尺度固定为 1 的前提下，解 VINS 式线性对齐（去掉尺度未知数），再把重力约束到已知模长的 2-DOF 切空间。
4. **加速度计 bias**：短窗内钉在零，只带弱 prior；交给后续在线 `ImuFactor` + `BetweenFactor<ConstantBias>`。
5. **MAP 精化**：用已经存在的 `buildGraph()` / `LevenbergMarquardtOptimizer` 做一次小窗联合优化（位姿可弱约束或暂时固定），不引入第二套 backend。
6. **验收后原子提交**：通过验收才把对齐后的 `X/V/B` 写入 `VioUpdateState` 并 `transaction.commit()`；失败则 rollback，继续累积或保持 `kInitializing`。
7. **恢复**：`kVisualOutage` 后复用同一 helper；继承上一成功段末的 \(\mathbf{b}_g\)，并重新解重力方向与速度。第一刀不继承 \(\mathbf{b}_a\)（见 §G.3）。测量间断（IMU 流断裂）则 bias 也清掉。VINS estimator 的 reboot 是 `clearState()` 后冷启动同一套 init（Fusion 默认 `failureDetection` 关闭）。Kimera / OpenVINS / Basalt / OKVIS 的 **live** 路径在视觉丢失后并不重跑 stereo-VI init（OpenVINS 仅有 TODO；Kimera 用 IMU pose guess）。因此「同一模块服务起步与 outage」是 phad 的产品合同，不是这些仓库的现成接线。

这是 roadmap M5 两条候选的 **立体适配组合**：顺序对齐提供可观测、可拒绝的初值；既有 GTSAM 小窗提供 Campos 所说的 inertial-only / VI MAP 精化，而不搬 ORB-SLAM3 的 g2o 初始化器。

### 为什么现在做它

[`c999f58` / `default_0337287b`](../benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md) 已经证明 full-state VIO 的图、预积分、transaction 与 500 ms coast 能在 11 条上 `failed=0`、`rejected=0`。失败模式转到 **错误的 root `X/V/B` 被当成成功提交**：

- moving bootstrap 用最近约 20 ms IMU 的 `mean(acc)` 定 roll/pitch，并把 \(\mathbf{v}_0=\mathbf{0}\)、\(\mathbf{b}_g=\mathbf{0}\)、\(\mathbf{b}_a=\mathbf{0}\) 直接 `seedRoot`。见 [`vio_estimator.cpp` L1821–1868、L2065–2068](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/estimator/vio_estimator.cpp#L1821-L1868)。
- 一旦 `seedRoot` 成功且 LM 有限，`m_initialized = true` 并返回 `kOk`。[`vio_estimator.cpp` L2664–2666](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/estimator/vio_estimator.cpp#L2664-L2666)
- MH_02 ATE 从旧 VO `0.089` 升到 `0.291`、segments=6；V2_02 从 `0.853` 升到 `1.982`、segments=9。这与「带着错误速度/重力进入 IMU 传播」一致，而不是 estimator 崩溃。
- V2_03 的 896 帧 `initializing`、52 次 visual-outage、completion `50.65%` 说明 **同一初始化模块必须同时服务起步与中断恢复**；否则每次 `completeActiveSegment` 都会再种一个零速度 root。

M5 出口要求：无静止段可起步、失败返回原因而不是 identity pose、MH_* 不劣于静止初始化。[`docs/design/roadmap.md` M5](../design/roadmap.md)

### 明确不建议现在做什么

- 不新增第二套 estimator / backend / wrapper；不把 g2o、Ceres 初始化器或 OpenVINS `ov_init` 链进产品。
- 不在本片求解单目尺度、camera–IMU 外参、time offset。
- 不把加速度计 bias 当作短窗必解量。
- 不调 IMU noise、不改固定-lag / 边缘化（M6）、不做回环或全局地图。
- 不把 VINS-Fusion stereo 路径「凑满 10 帧就 `optimization()` 并宣称完成」原样搬来：那条路径 **没有** 独立的重力/速度线性解，依赖后续 Ceres 窗；phad 当前会在 **第 1 帧** 提交 `kOk`，缺的就是对齐与验收。

### 结论置信度及可能推翻结论的证据

| 判断 | 置信度 | 可能推翻的证据 |
|---|---|---|
| 立体尺度已知后，最小可解集是 \(\mathbf{b}_g\)、重力 2-DOF、每帧速度；yaw/原点是 gauge；\(\mathbf{b}_a\) 短窗不可靠 | 高 | 论文与 ORB-SLAM3 stereo `VertexScale` 固定互相独立证实 |
| 顺序 gyro → 线性 g/v → 既有 LM 精化 是当前代码上的最小 vertical slice | 高 | 若定向序列证明只做 gyro LS + 现有窗 LM、不解 g/v 已使 MH_02/V2_02 回到旧 VO 量级，可把线性 g/v 降为后续片 |
| 视觉中断后继承 \(\mathbf{b}_g\)、重解 g/v | 中 | 若 500 ms coast 后 \(\mathbf{b}_g\) 已漂到不可用，继承会劣于从零重估；需在 MH_02/V2_02 上 A/B |
| 本片不调 IMU noise / 边缘化 | 高 | 仅当对齐验收已过、MH_* 未回归、而在线 ATE 仍系统性差于对齐后的短窗残差时，才打开下一层 |

待确认（见 §G 证据缺口）：phad 窗口里非关键帧是否计入对齐帧；EuRoC 上 1 s vs 满窗 10 帧哪个先满足激励；继承 \(\mathbf{b}_a\) 是否值得做。

---

## B. 成熟实现对比表

| 系统 | 初始化输入 | 求解变量 | 优化形式 | 所需时间/激励 | 失败判据 | failure recovery | 对 phad-vio 的适配成本 |
|---|---|---|---|---|---|---|---|
| **VINS-Mono** | 滑窗视觉 SfM（任意尺度）+ 预积分 | \(\mathbf{b}_g\)；然后 \(\{\mathbf{v}_k\}, \mathbf{g}^{c_0}, s\)；\(\mathbf{b}_a\) 固定 0 | 顺序：5 点+SFM → `solveGyroscopeBias` → `LinearAlignment`（含 \(s\)）→ `RefineGravity` → 紧耦合 VIO | 满窗 `WINDOW_SIZE=10`（11 个 pose）；相对位姿视差 `average_parallax*460>30`；激励 `std(Δv/dt)<0.25` **只打日志** | 线性对齐：\(\lvert\\|\mathbf{g}\\|-G\rvert>1.0\) 或 \(s<0\)。[failureDetection 活阈值](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/estimator.cpp#L621-L667)：\(\\|\mathbf{b}_a\\|>2.5\)、\(\\|\mathbf{b}_g\\|>1.0\)、\(\Delta\mathbf{p}>5\)、\(\lvert\Delta z\rvert>1\)（\(\Delta\)angle / 跟踪数已被注释） | `clearState()` 后冷启动同一套 `INITIAL`；不复用旧图/旧 bias。`loop_fusion` 是独立进程 | 单目 SfM 与 \(s\) 对 phad 多余。`solveGyroscopeBias` 与去掉 \(s\) 的线性 g/v 可迁到度量位姿 |
| **VINS-Fusion stereo+IMU** | 立体三角化+PnP 得到 **度量** `Rs/Ps`；`initFirstIMUPose` 用 mean(acc) 定 `Rs[0]` | 窗满只解 \(\mathbf{b}_g\)；`Vs` 留自 `processIMU` 中点积分（PnP 覆盖 R/P）；\(\mathbf{b}_a\) 保持 0 直到第一次 `optimization()` | **不调用** `initialStructure` / `LinearAlignment` / `RefineGravity`。`LinearAlignment` 源码始终含 \(s\)，没有 stereo 分支。满窗：`solveGyroscopeBias` → `repropagate(ba=0)` → `optimization()` | `WINDOW_SIZE=10`（11 pose）；立体路径 **无** 激励检查、**无** 时长下限 | `failureDetection()` [第一行 `return false`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L955-L957)；后面的 Mono 式阈值全部不可达 | 若触发 reboot：`clearState()` + `setParameter()` 再攒满窗。`failure_occur` 在 clear 时被清零，`double2vector` 对齐 `last_R0` 的分支实际走不到 | 编排最近：度量位姿 + gyro LS + 非线性。phad 还缺独立 g/v 初值与验收，因为当前第 1 帧就 `kOk`。不要把满窗即 `NON_LINEAR` 当产品合同 |
| **ORB-SLAM3 / Campos MAP** | 视觉地图已起来 + Forster PIM。入口是 `LocalMapping::InitializeIMU`（无 `ImuInitializer` 类） | 立体：\(s\) 顶点仍在图里但 `setFixed(!bMono)`，\(s=1\)；估共享 \(\mathbf{b}_g,\mathbf{b}_a\)、\(R_{wg}\)（2-DOF）、每 KF 速度。位姿全部固定 | 惯性-only MAP（g2o `EdgeInertialGS`，无 Huber）→ `FullInertialBA`（放开位姿）。YAML **没有** `mMinTime`；`minTime`/`nMinKF` 是函数内局部变量 | 立体 `minTime=1.0`、`nMinKF=10`；单目 2.0 s / 10 KF。论文 5 s / 15 s VI-BA 在代码里是 `mTinit`：仅当相邻相机中心位移 `>0.05` m 才累加时间 | **代码门：** `mScale<1e-1`（立体几乎不触发）；视觉 stereo `N>500` 且 \(\\|\Delta\mathrm{avgA}\\|<0.5\) m/s² 则拒绝。**论文有、此 SHA 无：** ICRA 2020「平均加速度 < 0.5% \(G\)」；ICRA 2019 Hessian/consensus；墙钟 15 s 丢图（代码用 `!GetIniertialBA2()`） | Atlas `CreateMapInAtlas` / `ResetActiveMap`；新图必须重新视觉 init + `InitializeIMU`。这是 M8 范围。`ResetFrameIMU` 是空 TODO | 代价与 GTSAM `ImuFactor` 同构，精化复用现有 LM，不引入 g2o。立体已证明可去掉尺度。第一刀仍不把 \(\mathbf{b}_a\) 放进对齐状态（他们靠 `priorA=1e5`） |
| **Mur-Artal 2017 ORB-SLAM-VI** | 数秒单目 ORB-SLAM 轨迹 | 顺序：\(\mathbf{b}_g\) → \(s+\mathbf{g}\)（无 \(\mathbf{b}_a\)）→ 加入 \(\mathbf{b}_a\) 与 \(\\|\mathbf{g}\\|=G\) → 回代速度 | 三关键帧消元后的线性 SVD；用条件数检查可观测性 | 「数秒」；至少 4 个关键帧。\(\mathbf{b}_a\) 与重力耦合，靠 \(G\) 和条件数 | 病态则不可信 | 论文：reloc 后 20 帧重估 bias。ORB-SLAM3 `ResetFrameIMU` **未实现** | TRO 称这套太慢。gyro-first 分解仍成立；条件数是验收语言，不是现成阈值。不要把 2017 SVD 当 ORB-SLAM3 live 路径 |
| **Kimera-VIO live** | 首包 IMU；Euroc 默认 `autoInitialize: 0`（GT seed），否则 `1` = 静止 IMU | \(v=0\)；姿态由 \(-\mathrm{mean}(acc)\) 对齐世界重力；\(\mathbf{b}_g=\mathrm{mean}(gyr)\)；\(\mathbf{b}_a=\mathrm{mean}(acc)+R^\top g\) | 单次闭合解，无激励检查 | 第一包 IMU；假设静止且某一轴对准重力 | 非法 `autoInitialize` 直接 FATAL | 跟踪失败时用 IMU pose guess 继续；`Bootstrap→Nominal` 单向 | 静止路径与 phad static bootstrap 同类，但 phad **不**在静止窗解 \(\mathbf{b}_a\)。Euroc 默认走 GT，不能当「他们的运动 init」 |
| **Kimera OGA（源码在、pipeline 未接）** | 立体 BA 位姿 + PIM | \(\mathbf{b}_g\) LS；线性解 \(v,g_{b0}\)；**不含** \(\mathbf{b}_a\) | Qin–Shen / VINS 式对齐（`OnlineGravityAlignment`） | 未接入 live，无产品窗长 | gyro 残差 `>5e-2` 或 \(\lvert\\|\hat g\\|-G\rvert>0.1\) | 无 caller | 算法参考：GTSAM Jacobian 上的 VINS 对齐。`InitializationBackend` 仅自测/自调用 |
| **OpenVINS `ov_init`** | 特征库 + IMU 缓冲；动态路径对多相机 id 设 `have_stereo` | 静止：\(v=0\)，\(\mathbf{b}_g=\mathrm{mean}(gyr)\)，\(\mathbf{b}_a=\mathrm{mean}(acc)-R(0,0,G)\)；动态线性：feature+\(v\)+\(g\)，bias 保持配置猜测（默认 0）；随后 Ceres MLE 在 prior 下动 \(\mathbf{b}_g,\mathbf{b}_a\) | 默认静止（`init_dyn_use=false`）；动态为 Dong-Si 线性 + MLE | 默认 `init_window_time=1.0` s；动态 ≥4 pose、默认 5；累计转角 45°；半窗 ≥15 特征 | \(\lvert\\|\hat g\\|-G\rvert\)；动态协方差倒数条件数 `1e-15` | `try_to_initialize` 注释写可 repurpose 为失败后重初始化；**未实现** | 窗长/转角可参考。线性动态路径与推荐 g/v 解同族。不要链入 `ov_init` |
| **Basalt** | 第一帧 IMU 样本；`initialize(bg=0,ba=0)` | 姿态 `FromTwoVectors(accel, +Z)`；\(v=0\)；bias 为调用方零值，之后在窗内 RW 估计 | 无独立 stereo init 模块 | 单样本，无激励门 | 无 | 无 reset/re-init | 与当前 phad moving bootstrap 同类（单次加速度定姿态）。论文 NFR 属 M6/M8 |
| **OKVIS / OKVIS2** | 首帧 IMU deque 的 mean(acc)；YAML `a0`/`g0` prior | 姿态把 mean(acc) 对齐世界 +Z（注释「non-accelerating」）；\(v=0\)；\(\mathbf{b}_a,\mathbf{b}_g\) 不从数据解 | 无独立 init 模块；随后关键帧 MAP | 首帧 IMU 队列 | 前端 RANSAC 非纯旋转才标 `isInitialized_`（landmark/运动，不是 IMU init） | OKVIS1 跟踪差只打日志；OKVIS2 `trackingLost_` 从未置 true | 立体尺度直接进 VIO。第一帧零速度提交正是 phad 要升级的点 |
| **GTSAM 4.3a0** | 与 phad 现窗相同：PIM + `ImuFactor` + `BetweenFactor<ConstantBias>` | 窗内 `X/V/B`；重力是 `PreintegrationParams.n_gravity`，**不是** Values 变量 | `LevenbergMarquardtOptimizer` batch；官方 navigation.md：短实验可用常值 bias，Combined 非推荐路径 | phad 窗容量 10 | `IndeterminantLinearSystemException` → `kFailed` | 现有 transaction rollback | 精化直接复用。API：`biasCorrectedDelta`（迭代内一阶）+ 从 raw 重建 PIM（phad 已做）。不要 ISAM2 / `IncrementalFixedLagSmoother` / `CombinedImuFactor` / `ImuFactor2` / SmartStereo |

来源钉死：

- VINS-Mono 论文 §V、§VI-G：[arXiv:1708.03852](https://arxiv.org/abs/1708.03852)；源码 HEAD [`90dabb5`](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/commit/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d)：[`VisualIMUAlignment`](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/initial/initial_aligment.cpp#L199-L207)；重力模长门 `>1.0`：[L184](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/initial/initial_aligment.cpp#L184)；活着的 `failureDetection`：[estimator.cpp L621–667](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/estimator.cpp#L621-L667)
- VINS-Fusion stereo 初始化：[estimator.cpp L480–506](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L480-L506)；`initFirstIMUPose`：[L345–364](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L345-L364)；`failureDetection` 开头 return false：[L955–957](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L955-L957)；`LinearAlignment` 始终含 \(s\)、重力门 `>0.5`：[initial_aligment.cpp L135–207](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/initial/initial_aligment.cpp#L135-L207)；`WINDOW_SIZE=10`：[parameters.h L24](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/parameters.h#L24)
- ORB-SLAM3 HEAD [`4452a3c4`](https://github.com/UZ-SLAMLab/ORB_SLAM3/commit/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4)：论文立体固定尺度 [arXiv:2007.11898](https://arxiv.org/abs/2007.11898) §V-B；`VS->setFixed(!bMono)`：[Optimizer.cc L3121–3124](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L3121-L3124)；立体 `minTime=1.0`、`nMinKF=10`：[LocalMapping.cc L1178–1211](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc#L1178-L1211)；首次 `InitializeIMU(1e2, 1e5, true)`：[L180–240](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc#L180-L240)；惯性-only 同时加 gyro/acc 零先验：[Optimizer.cc L3101–3114](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L3101-L3114)；视觉 stereo \(\\|\Delta\mathrm{avgA}\\|<0.5\)：[Tracking.cc L2347–2350](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L2347-L2350)
- Campos inertial-only MAP：[arXiv:2003.05766](https://arxiv.org/abs/2003.05766)（论文只对 \(\mathbf{b}_a\) 加零先验；开源同时加 \(\mathbf{b}_g\) prior）
- Mur-Artal 顺序分解与条件数：[arXiv:1610.05949](https://arxiv.org/abs/1610.05949) §IV。该 SVD 路径与 ICRA 2019 Hessian/consensus **均不在** `4452a3c4` 源码中
- Kimera live 开关与静止 IMU：[VioBackend.h L143–194](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/include/kimera-vio/backend/VioBackend.h#L143-L194)；[Euroc BackendParams.yaml `autoInitialize: 0`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/params/Euroc/BackendParams.yaml#L7)；[InitializationFromImu.cpp](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/InitializationFromImu.cpp)
- Kimera OGA（未接入 pipeline）：[OnlineGravityAlignment.cpp](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/OnlineGravityAlignment.cpp)；[InitializationBackend.cpp L45–107](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/InitializationBackend.cpp#L45-L107)
- OpenVINS `ov_init`：[InertialInitializer.cpp L79–158](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_init/src/init/InertialInitializer.cpp#L79-L158)；[InertialInitializerOptions.h](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_init/src/init/InertialInitializerOptions.h)；动态线性不含 bias：[DynamicInitializer.cpp L166–170](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_init/src/dynamic/DynamicInitializer.cpp#L166-L170)；重初始化 TODO：[VioManager.cpp L308–311](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/core/VioManager.cpp#L308-L311)
- Basalt 单样本姿态：[sqrt_keypoint_vio.cpp L185–219](https://gitlab.com/VladyslavUsenko/basalt/-/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/src/vi_estimator/sqrt_keypoint_vio.cpp#L185-L219)
- OKVIS mean-acc 首姿态：[Estimator.cpp L811–839](https://github.com/ethz-asl/okvis/blob/f0c9ba472b89f85030cd58dfbae03ceec7817679/okvis_ceres/src/Estimator.cpp#L811-L839)；OKVIS2 同算法并注明 non-accelerating：[ImuError.cpp L781–807](https://github.com/ethz-mrl/okvis2/blob/a2ea00688cd10988aae7bd52ab7935ce9a657ec0/okvis_ceres/src/ImuError.cpp#L781-L807)
- GTSAM 4.3a0 [`3ad4b4c3cb28394c9597f48fa02dad361c8450e3`](https://github.com/borglab/gtsam/tree/3ad4b4c3cb28394c9597f48fa02dad361c8450e3)：[navigation.md Combined 非推荐](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/navigation.md#L202-L207)；[`MakeSharedU`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegrationParams.h#L41-L57)；[`ImuFactorsExample` LM + Between](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/examples/ImuFactorsExample.cpp#L241-L288)
- Forster PIM：[arXiv:1512.02363](https://arxiv.org/abs/1512.02363)

---

## C. 推荐算法的逐步数据流

记号与 phad 一致：body = IMU；世界系 \(z\) 轴沿重力（GTSAM `PreintegrationParams::MakeSharedU`）；立体尺度已知。帧 \(k=0\ldots n\)，\(n+1\) 等于证据窗帧数。预积分 \(\Delta\mathbf{R}_{k,k+1},\Delta\mathbf{v}_{k,k+1},\Delta\mathbf{p}_{k,k+1}\) 已对 \(\hat{\mathbf{b}}_g\) 线性化，旋转块 Jacobian 记 \(\mathbf{J}^\gamma_{b_g}\)。

### C.1 输入缓存

在 `m_initialized == false` 且 IMU payload 为 raw interval 时：

| 缓存 | 内容 | 现有落点 |
|---|---|---|
| IMU 节点 | 规范化 endpoint 样本 | 已有 `VioUpdateState::m_bootstrap_nodes` |
| 视觉帧 | `T_W_B`、观测、是否关键帧、raw interval | 应复用 `m_window`（`WindowFrame` 已有 `m_T_W_B` / `m_imu` / `m_v_W_B` / `m_bias`） |
| 连续性 | `m_continuity_anchor` | 已有 |
| 继承 bias | 上一段提交末状态（仅 visual-outage 路径） | **新增** optional，见 §D |

静止门仍走现有 static bootstrap：`mean(gyr)→b_g`，`mean(acc)` 定 roll/pitch，`v=0`。[`types.hpp` L87–97](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/estimator/types.hpp#L87-L97) 与 [`vio_estimator.cpp` L1806–1864](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/estimator/vio_estimator.cpp#L1806-L1864)。**工程推断：** MH_* 起飞前静止段应继续走这条，不对它做多帧对齐。

运动路径：首帧仍可用 `minimalRotationToWorldUp(mean(acc))` 给一个 **临时** 姿态（VINS-Fusion `initFirstIMUPose`、Campos「用平均加速度初始化重力方向」同一做法），但 **不** 把 `v=0,b_g=0` 标成成功 VIO。

视觉结构：phad 已有度量立体，不需要 Five-point SfM。用当前 `seedRoot` / PnP / `GenericStereoFactor` 长窗即可。VINS-Fusion stereo 路径同样是 PnP+triangulate，不是 `initialStructure()`。[`estimator.cpp` L477–502](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L477-L502)

每帧：若证据不足，`transaction.commit()` 只保存缓存，返回 `kInitializing` + 原因字符串（需要更多时间 / 更多关键帧 / 更大旋转激励）。**不**写 `VioEstimate`。roadmap 禁止 identity pose 冒充成功。

### C.2 gyro bias 估计

**论文明确声明（VINS-Mono eq. 15；Mur-Artal eq. 9）：** 常值 \(\delta\mathbf{b}_g\) 使视觉相对旋转与预积分旋转一致。

VINS-Fusion 实现（stereo 与 mono 共用 `solveGyroscopeBias`）：

```text
q_ij = R_i^T R_j                          # 视觉
A += J_bg^T J_bg
b += J_bg^T * 2 * (Δq_imu^{-1} ⊗ q_ij).vec()
δb_g = A.ldlt().solve(b)
Bgs[k] += δb_g
repropagate(ba=0, bg=Bgs[0])              # 全窗同一常值
```

Permalink：[initial_aligment.cpp L16–48](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/initial/initial_aligment.cpp#L16-L48)

Kimera 用 GTSAM `preintegrated_H_biasOmega()` 做同一阶近似。[OnlineGravityAlignment.cpp L185–190](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/OnlineGravityAlignment.cpp#L185-L190)

**工程推断（phad）：** 对窗内每对相邻 `WindowFrame`，视觉 \(\mathbf{R}_{ij}=\mathbf{R}_i^\top\mathbf{R}_j\)；PIM 来自已有 `preintegrate(interval, bias_hat)`。GTSAM `PreintegratedImuMeasurements` 在 `biasHat` 处保存对 gyro bias 的 Jacobian；`reintegrate` / 用新 `ConstantBias` 重建 PIM 对应 VINS 的 `repropagate`。acc bias 在此步保持 0。

残差（流形形式，Forster）：

\[
\mathbf{r}_R = \mathrm{Log}\big(\Delta\mathbf{R}(\mathbf{b}_g)^\top \mathbf{R}_i^\top \mathbf{R}_j\big)
\]

线性化后与 VINS 的 `2*(Δq^{-1}⊗q_ij).vec()` 同阶。

### C.3 gravity / velocity 估计

**论文明确声明（VINS-Mono eq. 16–20）：** 状态

\[
\mathcal{X}_I = \big[\mathbf{v}^{b_0}_{b_0},\ldots,\mathbf{v}^{b_n}_{b_n},\;\mathbf{g}^{c_0},\;s\big]
\]

**开源实现的实际行为：** Fusion / Mono 的 `LinearAlignment` **始终**含尺度列 \(s\)（状态 \(3N+3+1\)，尺度系数除以 100），**没有** `STEREO` 分支。立体 estimator **不调用**该函数。[initial_aligment.cpp L135–207](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/initial/initial_aligment.cpp#L135-L207)

立体已知尺度 → **删掉 \(s\)**。这是工程适配，对应 ORB-SLAM3「stereo-inertial by fixing the scale factor to one」，不是 Fusion stereo 的 live 路径。[arXiv:2007.11898](https://arxiv.org/abs/2007.11898)

相邻帧预积分模型（VINS eq. 17，\(s=1\)，\(p\) 已是米；phad body = IMU，外参平移为 0）：

\[
\begin{aligned}
\Delta\mathbf{p}_{k,k+1}
&= \mathbf{R}_k^\top\big((\mathbf{p}_{k+1}-\mathbf{p}_k)
+\tfrac12\mathbf{g}\,\Delta t^2
-\mathbf{v}_k\Delta t\big)\\
\Delta\mathbf{v}_{k,k+1}
&= \mathbf{R}_k^\top\big(\mathbf{v}_{k+1}+\mathbf{g}\,\Delta t-\mathbf{v}_k\big)
\end{aligned}
\]

线性系统（每对帧 6 个方程；视觉平移在 \(s=1\) 时移到右端，对应源码去掉 \(s\) 列）：

\[
\begin{bmatrix}
-\Delta t\,\mathbf{I} & \mathbf{0} & \tfrac12\mathbf{R}_k^\top\Delta t^2 \\
-\mathbf{I} & \mathbf{R}_k^\top\mathbf{R}_{k+1} & \mathbf{R}_k^\top\Delta t
\end{bmatrix}
\begin{bmatrix}\mathbf{v}_k\\\mathbf{v}_{k+1}\\\mathbf{g}\end{bmatrix}
=
\begin{bmatrix}
\Delta\mathbf{p}-\mathbf{R}_k^\top(\mathbf{p}_{k+1}-\mathbf{p}_k)\\
\Delta\mathbf{v}
\end{bmatrix}
\]

源码里 `cov_inv.setIdentity()`，法方程再乘 `1000`（数值缩放，不是物理协方差）。重力模长门：Fusion `>0.5`，Mono `>1.0`；另拒绝 \(s<0\)（立体适配不再验收 \(s\)）。

**工程推断：** phad 第一刀可以同样用单位权重；第二刀才把 Forster PIM 协方差放进加权 LS。不要在第一刀引入 `CombinedImuFactor`。

重力精化（VINS §V-B3；代码 `RefineGravity`，4 次切空间迭代）：

\[
\mathbf{g} = G\,\hat{\mathbf{g}} + w_1\mathbf{b}_1 + w_2\mathbf{b}_2,\quad
\mathbf{b}_1,\mathbf{b}_2\perp\hat{\mathbf{g}}
\]

Kimera 默认精化 4 次、切空间容差 `1e-1`。[OnlineGravityAlignment.cpp L58–63](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/OnlineGravityAlignment.cpp#L58-L63)

对齐完成后：把全部 `T_W_B`、`v_W` 旋转到世界 \(z\parallel\mathbf{g}\)；yaw 保持 gauge（VINS `g2R` 后去掉 yaw；phad 已有 `minimalRotationToWorldUp`）。平移原点保持第一帧位置，不重设 identity——否则会与已三角化的 `m_landmarks_w` 不一致。

**Gauge：**

| 量 | 处理 |
|---|---|
| 尺度 | 固定 1，不进状态 |
| yaw | 不可观；由重力对齐后的第一帧视觉姿态承担 |
| 世界原点 | 第一帧位置；root `PriorFactor<Pose3>` 已存在 |
| 重力模长 | 固定 `EstimatorOptions::m_gravity_mps2`（默认 9.81） |
| 速度 | 每帧 3 维，无零速度假设（运动路径） |

Campos 用 up-to-scale 速度 \(\bar{\mathbf{v}}_i\) 是为了单目 \(s\)；立体直接用米/秒速度。

### C.4 acc bias 处理

**论文明确声明：**

- VINS-Mono：初始化 **忽略** \(\mathbf{b}_a\)，因其与重力耦合，短窗、动力学远小于 \(G\) 时不可观。[arXiv:1708.03852 §V](https://arxiv.org/abs/1708.03852)
- Campos：把 \(\mathbf{b}_a\) 放进 MAP，但加 prior \(\\|\mathbf{b}_a\\|^2_{\Sigma_p}\)；激励不够时 prior 把它钉在 0。[arXiv:2003.05766 §II-B](https://arxiv.org/abs/2003.05766)
- Mur-Artal：第三步才解 \(\mathbf{b}_a\)，并靠 \(\\|\mathbf{g}\\|=G\) 与条件数；否则病态。[arXiv:1610.05949 §IV-C](https://arxiv.org/abs/1610.05949)

**开源实现的实际行为（运动/动态路径）：** OpenVINS 动态线性系统状态是 feature + velocity + gravity，注释写明 **不在此步恢复 bias**，保持 launch 猜测（默认零）。[DynamicInitializer.cpp L166–170](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_init/src/dynamic/DynamicInitializer.cpp#L166-L170) Kimera OGA 线性系同样只有 \(V_i,V_{i+1},g\)。VINS `solveGyroscopeBias` / 立体窗满一律 `repropagate(Vector3d::Zero(), Bgs)`；`InitialBiasFactor` 头文件存在但 estimator **从未** `new`。ORB-SLAM3 立体 inertial-only **会**估 \(\mathbf{b}_a\)（以及 \(\mathbf{b}_g\)），首次 `priorG=1e2`、`priorA=1e5`；ICRA 2020 写「gyro prior 不需要」，源码却同时加 `EdgePriorGyro` 与 `EdgePriorAcc`。[Optimizer.cc L3101–3114](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L3101-L3114) 静止路径（Kimera live、OpenVINS static）把 \(\mathrm{mean}(acc)-R^\top g\) 收进 \(\mathbf{b}_a\)，那是 **静止残差吸收**，与运动窗可观性不是同一问题。

**工程推断（phad 第一刀）：运动对齐钉 \(\mathbf{b}_a=\mathbf{0}\) + 现有 `m_acc_bias_prior_sigma_mps2=0.1`。** 不在对齐状态向量里解 \(\mathbf{b}_a\)。静止 bootstrap 维持现状（只估 \(\mathbf{b}_g=\mathrm{mean}(gyr)\)，不把重力残差写进 \(\mathbf{b}_a\)）。在线窗已经有 `ImuFactor` 与 bias RW。ORB-SLAM3 立体仍优化 \(\mathbf{b}_a\)（强 priorA，`InitializeIMU(1e2, 1e5, true)`），那是他们 1 s MAP 的一部分，推迟到 phad 证明短窗可观之后。

### C.5 nonlinear refinement

用现有 `buildGraph` + `solveGraph`（LM）：

- 值：对齐后的 `Pose3` / `Vector3` 速度 / `ConstantBias(0, b_g)`
- 因子：已有 `ImuFactor`、`BetweenFactor<ConstantBias>`、`GenericStereoFactor`、root prior
- **开源/官方 API：** GTSAM 4.3a0 `navigation.md` 写明短时实验可用单一常值 bias，**不推荐 Combined IMU factor**。[navigation.md L202–207](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/navigation.md#L202-L207) 重力只存在于 `PreintegrationParams::MakeSharedU` 的 `n_gravity`，`ImuFactor` 不把重力收进 Values。[PreintegrationParams.h L41–57](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegrationParams.h#L41-L57) LM 迭代内走 `biasCorrectedDelta` 一阶修正；换 `biasHat` 后从 raw 重积用 `resetIntegrationAndSetBias`（phad 每次 `buildGraph` 已从 raw steps 重建 PIM，等价于精确路径）。
- **工程推断：** 精化把 root **速度** prior 放到对齐结果上（而不是 `v=0`），pose prior 仍钉第一帧（yaw/原点 gauge）。同一 `LevenbergMarquardtOptimizer`，不引入 `CombinedImuFactor`、`ImuFactor2`、ISAM2。

这对应 Campos 的「inertial-only MAP → VI-BA」里的第二步，但视觉残差已经在 phad 图里，不必另写 g2o。

ORB-SLAM3 惯性-only：全部 KF 位姿 `setFixed(true)`，只优化速度、共享 bias、\(R_{wg}\)；立体再固定 `VertexScale`。[Optimizer.cc L3070–3082, L3121–3124](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L3070-L3124) 重力初值来自累积 \(-\mathbf{R}\Delta\mathbf{v}_{\mathrm{preint}}\)，速度初值是相邻 IMU 位置差 \(/dT\)，不是加速度均值。[LocalMapping.cc L1226–1252](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc#L1226-L1252) 随后 `FullInertialBA` 才放开位姿。

**工程推断：** 第一刀可以在精化时固定全部 `X(k)`，只释放 `V(k)` 与 `B(k)` 的 gyro 部分；若 LM 不定，退回线性解。第二刀再释放位姿。不要一次放开 \(\mathbf{b}_a\) 与全部位姿。

### C.6 acceptance

文献 **没有** 统一阈值。实现必须把下列检查做成 **显式拒绝 / 继续累积**，阈值先做成可测诊断，再在 MH_02/V2_02 上标定；**不要把下表数字写进默认产品门**。

| 检查 | 来源范围 | 失败动作 |
|---|---|---|
| 帧数 | ORB 立体 ≥10 KF；VINS `WINDOW_SIZE=10`（11 个 pose）；OpenVINS 动态 ≥4 pose | 继续累积 |
| 时长 | ORB 立体 ≥1 s；OpenVINS 默认 1 s；Campos 实验 ~1.3–2.2 s。VINS **无**时长下限 | 继续累积 |
| \(\lvert\\|\mathbf{g}\\|-G\rvert\) | Fusion `LinearAlignment`（仅 mono 路径）`>0.5`；Mono `>1.0`；Campos 转述 VINS 论文 10% | 拒绝本次解，继续累积 |
| 旋转激励 | OpenVINS 动态默认累计 45°；VINS SfM 视差 `*460>30`。VINS mono `std(Δv/dt)<0.25` **只打日志**；**立体路径无激励检查** | 继续累积。phad 应同时记录视觉位移模长与 IMU \(\Delta\mathbf{R}\) 角 |
| 近匀速 | ICRA 2020 **论文**：平均加速度 < 0.5% \(G\) 则丢弃。ORB-SLAM3 **源码无此门**；视觉 stereo 用的是帧间 \(\\|\Delta\mathrm{avgA}\\|<0.5\) m/s²（约 5% \(G\)） | 拒绝本次解，继续累积 |
| gyro 对齐残差 | Kimera gflag `5e-2` rad 量级 | 拒绝 |
| 线性系条件数 | Mur-Artal 2017 要求报告；**无公开截止值**。ORB-SLAM3 此 SHA **没有** SVD/Hessian 可观测性测试 | 记录 \(\kappa\)；phad 自测后再关门 |
| 速度有限、非 NaN | phad 现有 `allFinite` | rollback，`kFailed` 仅用于合同破坏 |
| LM 不定 / 非有限 | 现有 `GraphSolveError` | rollback，不提交 |

VINS-Fusion stereo **没有** 这些门，满窗即 `solver_flag = NON_LINEAR`。这是开源实现的实际行为，不是应复制的产品合同。

### C.7 原子提交

通过验收后，在同一 `VioUpdateTransaction` 内：

1. 把对齐+精化后的 `T_W_B`、`v_W_B`、`b_g` 写入每个 `WindowFrame`。
2. 用新 bias 重积每段 `m_imu`（已有 eviction 重积路径可复用）。
3. 旋转/保持 landmark 与重力对齐一致。
4. `m_bootstrap_nodes.clear()`，`m_initialized = true`，`m_continuity_anchor.reset()`。
5. `transaction.commit()`，返回 `kOk` + `VioEstimate`。

失败：`transaction.rollback()`，窗口回到累积前快照，返回 `kInitializing`。只有输入合同坏（非有限、interval 端点错）才 `kInvalidInput` / `kFailed`。

---

## D. phad-vio 实施映射

对照 commit `c999f58`。estimator README 仍有一段历史 `StereoVoEstimator` 叙述；**以 `vio_estimator.cpp` / `types.hpp` / `vio_update_transaction.hpp` 为准。**

### D.1 应修改的现有类 / 私有状态

| 位置 | 现状 | 初始化片需要的变化 |
|---|---|---|
| `VioEstimator::update()` | 唯一 public seam | 保持唯一入口。运动路径在 `m_initialized==false` 时改为「长窗 + 对齐」，而不是首次视觉就 `seedRoot` 成功 |
| `VioUpdateState` | `m_bootstrap_nodes`、`m_initialized`、`m_window`、`m_visual_coast_duration_ns`、`m_segment_id`、`m_continuity_anchor` | 增加：`m_init_frames` 计数或直接用 `m_window.size()`；optional `m_inherited_bias`；optional 对齐诊断缓存。不要平行第二套 window |
| `Impl::seedRoot` | 用 bootstrap 姿态、**零速度**、bias 种第一帧 | 静止路径保持。运动路径：临时姿态可种视觉地图，但 `m_initialized` 仍为 false，直到验收 |
| `Impl::completeActiveSegment` | 清空窗、路标、bootstrap、coast，`m_initialized=false`，`++m_segment_id` | visual-outage 调用前把 `window.back().m_bias` 的 **陀螺仪分量** 写入 `m_inherited_bias`；加速度计分量保持零。discontinuity 路径 **不** 继承 |
| `Impl::buildGraph` / `solveGraph` | 每帧 LM；root 同时 prior 住 pose、`v`、bias | 精化步可暂时固定 `X` 或换 root velocity prior。不要新 solver 类型 |
| `Impl::preintegrate` | `PIM(pim_params, bias)` + `integrateMeasurement` | 对齐后用新 `b_g` 重积；与 `enforceWindowCapacity` 的 eviction 重积同一函数 |
| `EstimatorOptions` | `m_enable_moving_bootstrap` 默认 false；bench 打开 | 产品化时用「多帧对齐」替换「20 ms mean(acc) 即提交」。具体开关命名留给 design，不要再加第二套 bootstrap 语义 |

当前运动提交（必须改掉的行为）：

```1821:1868:phad/estimator/vio_estimator.cpp
      bool moving_bootstrap = false;
      if ( !static_ready &&
           m_impl->options.m_enable_moving_bootstrap )
      {
        // ... recent suffix mean(acc) → roll/pitch ...
            moving_bootstrap = true;
      }
      // ...
      if ( !moving_bootstrap )
      {
        bootstrap_bias.m_gyr_radps = bootstrap_stats.m_gyr_mean;
      }
      bootstrap_T_W_B.linear() =
          minimalRotationToWorldUp( bootstrap_stats.m_acc_mean );
```

```2065:2078:phad/estimator/vio_estimator.cpp
      if ( !m_impl->seedRoot( bootstrap_T_W_B, Eigen::Vector3d::Zero(),
                              bootstrap_bias, *effective_measurement,
                              ... ) )
```

```2664:2666:phad/estimator/vio_estimator.cpp
    m_impl->m_state->m_initialized = true;
    result.status                     = UpdateStatus::kOk;
```

### D.2 可以直接复用的代码

- `normalizeRawImuInterval` / `NormalizedImuInterval` / `spliceNormalizedImuIntervals`（eviction 已经按 predecessor **当前** bias 重积）
- `preintegrate` + `NavState::predict`（coast 与普通传播）
- `VioUpdateTransaction` clone/swap rollback
- `minimalRotationToWorldUp`
- `bootstrapStats` / 静止门（acc/gyr std、\(\lvert\\|\bar{a}\\|-G\rvert\)）
- `GenericStereoFactor` 视觉结构
- `gyro_rotation_predictor` 的时间/梯形合同可作为 gyro LS 的单测 oracle（生产路径仍用 PIM Jacobian）
- GTSAM：`PreintegratedImuMeasurements`、`ImuFactor`、`imuBias::ConstantBias`、`PriorFactor`、`LevenbergMarquardtOptimizer`。项目已 `find_package(GTSAM 4.3 REQUIRED)`。

### D.3 新增的最小私有 helper

全部放在 `VioEstimator::Impl`（或 `phad/estimator/internal/` 的纯函数），**不**新建 public 类。

建议三个函数（名称可在 design 时缩短，语义固定）：

1. `solve_gyro_bias(window) → Vector3 | reject`  
   输入相邻视觉 `R` 与 PIM 旋转 Jacobian。
2. `align_gravity_velocity(window, bg, G) → {g_dir, v_k} | reject`  
   尺度固定；切空间精化。
3. `accept_stereo_vi_init(...) → reasons`  
   只读检查，不写状态。

禁止：`StereoViInitializer` 作为第二个拥有 PIM/图/optimizer 的对象；禁止 apps 侧再跑一遍对齐。

### D.4 transaction / rollback 语义

保持现有分层：

| 阶段 | 行为 |
|---|---|
| pre-staging 合同错 | 不 snapshot，`kInvalidInput` |
| 累积中（证据不足） | 短 transaction 写入 `m_bootstrap_nodes` / window，commit，`kInitializing` |
| 对齐尝试 | 在 staging transaction 内改 window 的 `T/V/B` 与 PIM；验收失败 **rollback**，外层仍 `kInitializing` |
| 验收通过 | commit，`m_initialized=true`，`kOk` |
| `kVisualOutage` / `kDiscontinuity` | 现有 `completeActiveSegment` + commit；无 estimate |

不要在 rollback 后留下半对齐的 `m_initialized=true`。

### D.5 public diagnostics

现有 `VioDiagnostics` 已有 nav/IMU/bias-RW/visual 计数、coast、eviction、PIM 协方差对角。[`types.hpp` L130–148](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/phad/estimator/types.hpp#L130-L148)

`diag.csv` 已有 `status`（含 `initializing` / `visual_outage` / `discontinuity`）。**最小增量（仍是 POD，不暴露 GTSAM/PIM）：**

| 字段 | 用途 |
|---|---|
| `init_frame_count` | 证据窗帧数 |
| `init_duration_ns` | 首末帧时间 |
| `init_gyro_bias_radps` | 提交的 \(\mathbf{b}_g\)（仅 kOk 首帧或单独列） |
| `init_gravity_err_mps2` | \(\lvert\\|\mathbf{g}\\|-G\rvert\) |
| `init_rot_exc_rad` | 窗内相对旋转累积 |
| `init_trans_exc_m` | 窗内位移模长 |
| `init_reject_reason` | 枚举/短字符串：need_frames / need_time / need_excitation / gravity_norm / gyro_residual / near_constant_vel / nonfinite |

这些足以判断 MH_02 分段是初始化拒绝还是后续 outage，而不泄漏 Jacobian。

**不要**在 public header 增加 `Values`、`NonlinearFactorGraph`、PIM、条件数矩阵。条件数可先打日志或仅测试 fixture 读取。

### D.6 明确禁止增加的接口或层

- 第二个 `updateImu` / `reset` / `initialize()` public API
- `SmartStereoProjectionFactor`（M7 议题；初始化不需要）
- `IncrementalFixedLagSmoother` / ISAM2（M6）
- 回环、Atlas、全局 BA
- 把 GTSAM 类型带出 PIMPL
- 为对齐单独复制一套 raw IMU 规范化

Session：`OfflineVoSession` 已把 `kInitializing` / `kDiscontinuity` / `kVisualOutage` 当非失败、不写 `est.tum`。[`offline_vo_session.cpp` L776–779](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/apps/offline_vo_session.cpp#L776-L779) 这与「失败不冒充成功」一致；对齐期间会拉长 `initializing` 帧数，completion 口径要在验证矩阵里一起看。

---

## E. 首轮验证矩阵

### E.1 public seam 单元测试

扩展 `tests/estimator/vio_full_state_test.cpp`、`vio_lifecycle_test.cpp`、`vio_eviction_test.cpp`，不改 production 默认直到定向序列过门。

| 用例 | 期望 |
|---|---|
| 静止 IMU + 视觉种子 | 与现在 static bootstrap 位相同：`b_g≈mean(gyr)`，`v=0`，单段 `kOk` |
| 匀加速 + 已知 \(\mathbf{b}_g\) + 度量立体位姿 | 对齐后 \(\mathbf{b}_g\) 进入容差；速度与真值同向；`kInitializing` 持续到验收 |
| 纯匀速、无加速度激励 | 保持 `kInitializing` 或显式拒绝，**永不** `kOk` 零速度 root（Campos 0.5% g 动机） |
| 证据不足（<N 帧） | `kInitializing`，无 estimate |
| 对齐中 LM 非有限 | rollback，状态回到累积快照 |
| visual-outage → 再初始化 | `segment_id` +1；继承的 \(\mathbf{b}_g\) 出现在新段 root；速度重新解 |
| measurement discontinuity | `kDiscontinuity`；**不**继承 bias；bootstrap 清空 |
| 500 ms coast 内视觉恢复 | 现有 coast 测试不回归：`m_visual_coast_duration_ns` 复位，同段 |

合成轨迹已有 `kGyroBiasRadps` fixture。[`vio_full_state_test.cpp` L39](https://github.com/Nothand0212/phad-vio/blob/c999f5891d8efbc353e2e4001df7656a9e19907a/tests/estimator/vio_full_state_test.cpp#L39)

### E.2 MH_02、V2_02、V2_03 定向真实序列

基线（`c999f58` / `default_0337287b`）：

| 序列 | ATE | segments | 旧 VO ATE | 角色 |
|---|---:|---:|---:|---|
| MH_02 | 0.29054 | 6 | 0.08937 | **初始化/重力误差主探针**：视觉相对稳，旧 VO 单段；M4 多段+ATE 变差 |
| V2_02 | 1.98198 | 9 | 0.85285 | **中等纹理 + 多次切段**：分离错误 root 与跟踪变差 |
| V2_03 | 1.70751 | 52 | — | **不要当纯 init 探针**：896 initializing、52 visual-outage、completion 50.65%；数据集 cam0=1922、cam1=2336、公共 1921、415 右目孤儿 |

V2_03 的右目不对称是数据集/manifest 事实，不是 sync bug。[`euroc-stereo-manifest-asymmetry-open-source-refs.md`](euroc-stereo-manifest-asymmetry-open-source-refs.md)

归因用法：

1. **MH_02：** 若多帧对齐后 `segments` 回到 1 且 ATE 接近 0.09 m，则当前回归主要是错误 moving root。若仍 6 段，查 `diag.status` 是 `visual_outage` 还是反复 `initializing`。
2. **V2_02：** 同时看 `num_shared` / `pnp_fallbacks`(196) 与 segment 边界。init 修好后 ATE 应下降，但不必一次打到旧 VO；PnP fallback 高说明还有前端债。
3. **V2_03：** 成功标准首先是 **不要用错误 init 把 completion 刷高**。看 initializing 是否从「每段都立刻 kOk 再崩」变成「拒绝直到激励足够」。52 outage 与 415 右目 drop 不是本片能消掉的。

### E.3 哪些指标改善才值得再跑 11 条全表

同时满足再跑全表：

- MH_02：`segments≤2` 且 ATE **低于** 0.29 m，并不得差于旧 VO 0.09 m 太多（具体容差留给 design；本调研不发明门）
- MH_01：ATE 不劣于 0.07054 m，`segments=1`，completion ≥ 0.999
- V2_02：ATE 低于 1.98 m **且** segments 下降，或 ATE 下降而 segments 持平但 `visual_outage` 未增加
- 合成单测全绿；`failed` 仍为 0

V2_03 改善是加分项，不是全表触发器。

### E.4 如何检查 MH_* 静止初始化没有回归

MH_01/03/04/05 在 `c999f58` 上已是单段、ATE 优于或接近旧 VO。验证：

- `diag.csv` 开头：`status=initializing` 只覆盖静止门闭合前；**不应**出现数秒运动对齐（这些序列有足够静止）。
- 首个 `kOk` 的 `init_gyro_bias`（若加列）应接近 `mean(gyr)`，而不是运动 LS 的另一个值。
- `segments=1`；`m_completed_segment_id` 空。
- 对比 `est.tum` 与 `c999f58` 锚点的 ATE，而不是逐字节（初始化改时间戳集合会变）。

若 MH_01 被运动对齐抢先提交，说明静止门被超时或 `m_enable_moving_bootstrap` 路径抢占——这是回归。

---

## F. 风险排序

| 秩 | 风险 | 为何排前 | 最小验证 |
|---|---|---|---|
| 1 正确性 | 第 1 帧零速度 `kOk` 把错误 `X/V/B` 送进 `ImuFactor` | 已在 MH_02/V2_02 上表现为多段+ATE 升 | 合成匀加速：提交前必须 `kInitializing`；验收后速度非零 |
| 2 可观测性 | 短窗解 \(\mathbf{b}_a\) 或无激励解 g/v | VINS/Campos/Mur-Artal 均警告重力–\(\mathbf{b}_a\) 耦合 | 匀速夹具必须拒绝；条件数/加速度激励写入诊断 |
| 3 数值 | 单位权重 LS、VINS `A*=1000`、重力切空间 | 开源用 ad-hoc 缩放；GTSAM PIM 协方差更干净但第一刀未用 | 夹具上 \(\mathbf{b}_g\) 误差 vs 真值；\(\lvert\\|\mathbf{g}\\|-G\rvert\) |
| 4 生命周期 | outage 清窗后再次零速度 bootstrap；discontinuity 误继承 bias | `completeActiveSegment` 已清一切 | lifecycle 测试：outage 继承 \(\mathbf{b}_g\)；discontinuity 不继承 |
| 5 性能 | 每帧对满窗 10 做 LS+LM | 窗已经每帧 LM；多一次 9×6 线性系可忽略 | 单测计时可选；不要为此上 ISAM2 |

---

## G. 来源清单

### 论文

| 文献 | 用途 |
|---|---|
| Qin, Li, Shen, *VINS-Mono*, [arXiv:1708.03852](https://arxiv.org/abs/1708.03852) / TRO 2018 | 顺序初始化；忽略 \(\mathbf{b}_a\)；同一模块做 failure recovery |
| Qin et al., *VINS-Fusion* 框架, [arXiv:1901.03642](https://arxiv.org/abs/1901.03642) | 官方实现入口；stereo 细节以仓库为准 |
| Campos, Montiel, Tardós, *Inertial-Only Optimization*, [arXiv:2003.05766](https://arxiv.org/abs/2003.05766) | MAP；\(\mathbf{b}_a\) prior；0.5% g 匀速拒绝；EuRoC 穷举时长 |
| Campos et al., *ORB-SLAM3*, [arXiv:2007.11898](https://arxiv.org/abs/2007.11898) / TRO | 立体固定尺度；1–2 s / 10 KF；15 s 内跟丢丢图 |
| Campos et al., *Fast and Robust Initialization*, [arXiv:1908.10653](https://arxiv.org/abs/1908.10653) | 可观测性/共识测试（Martinelli 路线的工程化） |
| Mur-Artal & Tardós, *Visual-Inertial Monocular SLAM with Map Reuse*, [arXiv:1610.05949](https://arxiv.org/abs/1610.05949) | gyro-first；条件数；重定位后 20 帧重估 bias |
| Forster, Carlone, Dellaert, Scaramuzza, *On-Manifold Preintegration*, [arXiv:1512.02363](https://arxiv.org/abs/1512.02363) | PIM、bias Jacobian、MAP IMU 残差 |
| Geneva, Eckenhoff, Lee, Yang, Huang, *OpenVINS*, ICRA 2020（[项目 PDF](https://pgeneva.com/downloads/papers/Geneva2020ICRA.pdf)） | 研究平台；init 数字以仓库为准 |
| Usenko et al., *Visual-Inertial Mapping with NFR*, [arXiv:1904.06504](https://arxiv.org/abs/1904.06504) | 仅说明长间隔 IMU 退化；不作为 init 算法 |

### 官方仓库源码（钉死 SHA）

| 仓库 | SHA | 关键文件 |
|---|---|---|
| [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) | `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d` | `initial_aligment.cpp`、`estimator.cpp` `failureDetection` / `initialStructure` |
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | `be55a937a57436548ddfb1bd324bc1e9a9e828e0` | `initial_aligment.cpp`（仓库文件名拼写）、`estimator.cpp` stereo 分支 L480–506、`parameters.h` `WINDOW_SIZE` |
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` | `LocalMapping.cc::InitializeIMU`、`Optimizer.cc::InertialOptimization` / `FullInertialBA`、`G2oTypes.cc::EdgeInertialGS` |
| [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO) | `ce8c59b7b273ab5ac29db7e5572e1623760e19c7` | live：`VioBackend.h` / `InitializationFromImu.cpp`；未接线：`OnlineGravityAlignment.cpp`、`InitializationBackend.cpp` |
| [OpenVINS](https://github.com/rpng/open_vins) | `69488123ed9362dd44b6f28e7f4680abbff1442b` | `ov_init/`：`InertialInitializer.cpp`、`StaticInitializer.cpp`、`DynamicInitializer.cpp` |
| [Basalt](https://gitlab.com/VladyslavUsenko/basalt) | `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` | `sqrt_keypoint_vio.cpp` 首帧 `FromTwoVectors` |
| [OKVIS](https://github.com/ethz-asl/okvis) | `f0c9ba472b89f85030cd58dfbae03ceec7817679` | `Estimator.cpp::initPoseFromImu` |
| [OKVIS2](https://github.com/ethz-mrl/okvis2) | `a2ea00688cd10988aae7bd52ab7935ce9a657ec0` | `ImuError.cpp` mean-acc 姿态 |
| [GTSAM](https://github.com/borglab/gtsam) | `3ad4b4c3cb28394c9597f48fa02dad361c8450e3`（tag **4.3a0**，与本机 `GTSAM_VERSION_STRING` 一致） | `ImuFactor`、`PreintegrationParams`、`ImuFactorsExample`、`navigation.md` |
| [phad-vio](https://github.com/Nothand0212/phad-vio) | `c999f5891d8efbc353e2e4001df7656a9e19907a` | `vio_estimator.cpp`、`types.hpp`、`vio_update_transaction.hpp`、`offline_vo_session.cpp` |

### 官方文档 / 项目文档

- [`docs/design/roadmap.md` M5/M6](../design/roadmap.md)
- [`docs/benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md`](../benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md)
- [`docs/specs/2026-08-25-m4-minimal-full-state-vio.md`](../specs/2026-08-25-m4-minimal-full-state-vio.md)（单一 `update()` seam、静止 bootstrap 合同）
- [`phad/estimator/types.hpp`](../../phad/estimator/types.hpp)、[`vio_estimator.hpp`](../../phad/estimator/vio_estimator.hpp)
- OpenVINS docs：<https://docs.openvins.com/>（init 默认值以头文件为准）

### 证据缺口（待确认）

1. **phad 对齐窗用全部 window 帧还是仅 keyframe。** 当前窗容量 10、非关键帧也可进图。ORB 用 KF；VINS 用 `WINDOW_SIZE` 图像帧。需在 design 钉死；验证：对同一 EuRoC 片段比较两种计数的 \(\kappa\) 与 \(\mathbf{b}_g\) 误差。
2. **VINS-Fusion stereo 不解线性 g/v 是否在他们的 Ceres 窗里够用。** 开源行为是满 11 pose 后 gyro LS + `optimization()`；`Vs` 初值来自 `processIMU` 中点积分。phad 不能依赖这一点，因为现在第 1 帧就 `kOk`。若第一刀只做 gyro LS + 固定位姿的 LM 已修复 MH_02，线性 g/v 可延后——用 E.3 决定。
3. **visual-outage 继承 \(\mathbf{b}_a\)。** 文献只明确 gyro 易观、acc 难观。建议第一刀只继承 \(\mathbf{b}_g\)。
4. **条件数截止值。** Mur-Artal 2017 只要求检查，未给数字；ORB-SLAM3 开源未实现该测试。必须在合成夹具与 MH_02 上测量后再写门。
5. **Kimera OGA 未接入 live。** 运动对齐的残差门（`5e-2`、重力模长 `0.1`）来自未接线代码与单测，不是 Euroc 默认运行路径。采用那些数字前仍须在 phad 夹具上重测。

---

## 附录：对 12 个问题的直接回答

1. **最小 vertical slice：** 立体顺序对齐（gyro LS → 固定尺度的 g/v 线性解 → 既有 GTSAM LM 精化）。原因见 §A：尺度已由双目给出；当前缺的是多帧可拒绝初值，不是另一套 MAP 库。
2. **最小求解集：** \(\mathbf{b}_g\)、重力方向（2-DOF）、每帧速度。尺度固定。yaw 与原点是 gauge。\(\mathbf{b}_a\) 不进第一刀状态。
3. **acc bias：** 运动对齐钉零 + 弱 prior，交给在线优化。短窗运动路径上 OpenVINS 动态线性、Kimera OGA、VINS 线性/立体 gyro 步都不把 \(\mathbf{b}_a\) 当自由未知数。ORB-SLAM3 立体 MAP 会估 \(\mathbf{b}_a\)（`priorA=1e5`），推迟到短窗可观之后。
4. **gyro bias：** §C.2，视觉 \(\mathbf{R}_i^\top\mathbf{R}_j\) vs PIM \(\Delta\mathbf{R}\) 与 \(\mathbf{J}_{b_g}\)，LDLT，然后重积。
5. **g/v 目标：** §C.3；状态 \(\{\mathbf{v}_k\}_{0:n},\mathbf{g}\)；残差 \(\Delta\mathbf{p},\Delta\mathbf{v}\)；prior 无（线性步）；gauge 见上。精化步加 root pose prior 与 \(\mathbf{b}_a=\mathbf{0}\) prior。
6. **证据窗：** 文献范围 1–2 s、约 10 帧/KF、OpenVINS 动态 45° 转角；**无统一门**。phad 必须测量。不要把 20 ms moving suffix 当激励窗。
7. **拒绝条件：** §C.6。错误 `X/V/B` 不得 `kOk`。
8. **恢复：** outage 继承 \(\mathbf{b}_g\)（第一刀不继承 \(\mathbf{b}_a\)），重解 g/v；discontinuity 全清；新段仅验收后原子提交。coast 未超时则同段传播，不切段。VINS estimator reboot 是 `clearState()` 冷启动；Fusion `failureDetection` 默认关闭。ORB-SLAM3 跟丢后开 Atlas 新图或 `ResetActiveMap`（M8）；`ResetFrameIMU` 为空。Kimera/OpenVINS/Basalt/OKVIS live 不重跑 stereo-VI init。
9. **单一 seam：** 只改 `Impl` 在 `m_initialized==false` 的控制流；transaction/PIMPL/raw interval 不动。
10. **诊断：** §D.5 的 init 计数、\(\mathbf{b}_g\)、重力模长误差、激励、拒绝原因。
11. **序列：** MH_02 主探针，V2_02 次探针，V2_03 生命周期/数据集对照，MH_01 防回归。见 §E。
12. **现在调什么：** 只做初始化合同。严格顺序：① 多帧对齐与验收 → ② 提交后的 root velocity prior 与对齐后的 bias prior → ③ 若在线仍发散再动 IMU noise → ④ M6 才 fixed-lag。不要为本片做参数大扫。
