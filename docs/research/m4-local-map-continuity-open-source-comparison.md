# M4 活动局部估计与跨段连续性：Kimera-VIO / OKVIS2(-X) 对照

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-27

相关：[世界系连续性 spec](../specs/2026-08-26-m4-world-frame-continuity.md)、
[1b 实施计划](../plans/2026-08-27_m4_observation_intake_continuity_slice1b_9e7a3c41.plan.md)、
[1a coast 传播探针](m4-coast-propagation-probe-design.md)。

## 1. 结论

**推断（基于下列一手实现与论文）：** 当前“段内精度高、段间绝对错位大”的
问题，和 *保持一个仍可被视觉约束的活动局部估计* 有关；因此它与 OKVIS2 的
bounded realtime VI graph 在抽象层面相似。但是，把它称为要新增的 `local map`
会混淆三件不同的事：

1. 活动的稀疏 VIO window / landmark 集合；
2. 固定滞后边缘化后保留的信息；
3. 跨很久以后重访的全局重定位、loop closure 或稠密 submap 对齐。

1b 只处理第 1 项的连续性：在既有 500 ms coast 内，不因支持不足而提前停止
将有效 observation、track lifetime 和既有 landmark 写入**同一个 active
segment**；恢复帧因而仍可能和旧 window/map 形成已有 `GenericStereoFactor`。
它不实现第 2、3 项的新机制，尤其不创建持久 local map、pose graph、全局图、
重激活旧因子或跨 segment bridge。

这不是词语上的保守：1a 的 13 个 inherited root 只有单帧 seed，root prior 固定
了传播误差；之后局部误差虽小，却不存在跨段视觉 factor 能回校该误差。1b 的
直接目标正是**在真正断裂前避免该图断开**。若视觉真的超过 horizon 或发生
discontinuity，1b 仍诚实结束 segment；它没有足够信息把两个已断开的世界系对齐。

## 2. 术语与 owner：三个“局部”不是同一对象

| 系统 / 名称 | 事实上的对象与 owner | 是否是 1b 的对象 |
|---|---|---|
| phad-vio 1b active segment | `VioEstimator::Impl` 的 transaction state：`WindowFrame::m_observations`、`m_track_times`、`m_landmarks_w` 与 window；`completeActiveSegment()` 才清除它们。 | **是，直接对应。** |
| Kimera-VIO fixed-lag VIO | `VioBackend` 持有 `FeatureTrack` / smart factors 和 GTSAM `FixedLagSmoother`；每 keyframe 写入 state、IMU 和视觉量测，再由 smoother 按 lag marginalize。 | **部分对应。** 是活动局部估计，不是“跨断裂地图”。 |
| OKVIS2 realtime graph | `ViSlamBackend::realtimeGraph_` 是有界实时图；另有 `fullGraph_` 服务全局优化。旧 keyframe 可从 realtime 图转为 pose-graph nodes/edges。 | **部分对应。** 它展示了 local/global 分层，但其 full graph / pose graph 超出 1b。 |
| OKVIS2-X `submap` | 稠密 volumetric occupancy submap，由 mapping / SuperEight 接口维护，可选 map-to-map 或 map-to-live alignment。 | **否。** 这是稠密建图与对齐，并非稀疏 coast 恢复。 |

### 2.1 Kimera-VIO

**事实：** 官方源码的 `VioBackend` 显式使用 GTSAM fixed-lag smoother；
`addVisualInertialStateAndOptimize()` 为每个 keyframe 添加 state、IMU factor，
将 stereo measurements 加入 feature tracks，随后对有效视觉路径加入 landmark
factors并优化。[`VioBackend.h`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/include/kimera-vio/backend/VioBackend.h#L36-L44)
与 [`VioBackend.cpp`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/backend/VioBackend.cpp#L288-L395)。

**事实：** 其 `addLandmarksToGraph()` 以 track observation 数决定是否首次入图，
此后把同一 landmark 的最新 observation 更新到 smart factor；这正是“接收
量测”和“有资格形成优化约束”可以相隔一个生命周期阶段的例子。
[`VioBackend.cpp`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/backend/VioBackend.cpp#L447-L487)

**事实：** frontend 在某帧 feature tracking 全失时，会检测新 feature、推进
reference frame，并把空 stereo measurement 返回；这段源码本身并没有提供
“失跟后保存一个 local map 并在新段重接”的状态机。
[`StereoVisionImuFrontend.cpp`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/frontend/StereoVisionImuFrontend.cpp#L308-L323)

**事实：** Kimera-VIO 的 loop-closure detector 和 RPGO 是可选模块，默认关闭；
因此局部 VIO 与全局回环校正也被显式分层。
[`README`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/README.md#L249-L256)。

### 2.2 OKVIS2：有界 realtime graph 加全局图，不等于一个单独缓存

**事实：** OKVIS2 论文将系统描述为带 observations、IMU preintegrals 与 pose-graph
edges 的有界图，并为大回环异步优化；论文具体说明只保留最新 (T) 个 frame，
旧 non-keyframe 经 IMU merge 消去，旧 keyframe 则可转换为 pose graph。
[OKVIS2 论文，§III-B/C](https://arxiv.org/pdf/2202.09199.pdf)。

**事实：** 对应的官方 OKVIS2-X 基础实现里，`ViSlamBackend` 同时维护
`realtimeGraph_` 与 `fullGraph_`；正常添加 state / landmark / observation 时会写入
两者，loop closure 期间再使用 backlog 同步。
[`ViSlamBackend.hpp`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/okvis_ceres/include/okvis/ViSlamBackend.hpp#L114-L153)
与 [`ViSlamBackend.cpp`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/okvis_ceres/src/ViSlamBackend.cpp#L175-L215)。

**事实：** 其 bounded realtime 策略按 current frame/keyframe 的 covisibility 挑选
要移出的 keyframe；被移出者可转换成 pose-graph edge，realtime 图再冻结更老的
pose、speed 和 bias。质量低时只有日志，源码明确写着 lost-component handling
当前禁用。
[`ViSlamBackend.cpp`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/okvis_ceres/src/ViSlamBackend.cpp#L555-L711)。

**事实：** frontend 对已有 landmark 的 observation 可以直接加入 estimator；
对全新 landmark，`matchStereo()` 仅在 `asKeyframe` 时创建。这是“保留已有观测”
和“播种新点”可有不同门的明确实现。
[`Frontend.cpp`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/okvis_frontend/src/Frontend.cpp#L2185-L2360)。

**事实：** place recognition 通过 DBoW 候选及几何验证后，触发 loop closure，
再把旧 loop-closure frame 的 landmark 带回、同当前帧 match；`convertToObservations()`
还可把 pose-graph 表达重新展开为 reprojection observations。这是全局重访
能力，而非单次短 coast 自动获得的能力。
[`Frontend.cpp`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/okvis_frontend/src/Frontend.cpp#L837-L930)
与 [`ViGraphEstimator.cpp`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/okvis_ceres/src/ViGraphEstimator.cpp#L804-L879)。

**事实（证据边界）：** 该快照有 `trackingLost_` 成员，初始化/`clear()` 时置为
false，也会在匹配路径读取；但未找到将其置为 true 的可执行赋值。因此不能据此
声称 OKVIS2-X 已提供一个“tracking lost → local map 保世界系重接”的路径。
[`Frontend.hpp`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/okvis_frontend/include/okvis/Frontend.hpp#L509-L521)
与 [`Frontend.cpp`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/okvis_frontend/src/Frontend.cpp#L1163-L1170)。

### 2.3 OKVIS2-X submap

**事实：** 官方 README 明定 OKVIS2-X 的 map representation 是为 exploration /
navigation 服务的 **submap-based volumetric occupancy**；可选项包含
`map-to-map` / `map-to-live` submap alignment factors，关闭 submapping 则退回
pure OKVIS mode。
[`README`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/README.md#L42-L44)
、[`README`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/README.md#L364-L365)
、[`README`](https://github.com/ethz-mrl/OKVIS2-X/blob/38043e4afe56d9b32a98434cc74e723737dd2bce/README.md#L491-L519)。

**推断：** 该 submap 与 1a 的“段 root 锚误差”表面上都涉及局部区域和对齐，
但它需要稠密地图、额外 alignment constraints 和全局更新面；正落在 1b 明确
排除的 global map / stitching / 新 factor 类型范围外，不能作为当前补丁的
实现模板。

## 3. fixed-lag / marginalization 能保留什么，不能保留什么

**事实：** fixed-lag 的作用是把离开活动窗口的变量消去或转换，保留它们对仍在
窗口变量的约束；OKVIS2 的非 keyframe 消除与 keyframe→pose graph 转换正是这
一层，而 Kimera-VIO 把 marginalization 委托给 `FixedLagSmoother`。

**推断：** 这能避免“窗口一滚动就完全忘掉过去”的信息损失，却**不能凭空连接
已经没有共同视觉量测的两个独立 segment**。跨 segment 对齐仍至少需要一条
跨边界约束：连续的同图 visual factor、外部绝对量测、成功 relocalization，或
loop closure。1a 在单帧 seed 后没有这类 factor，故 fixed-lag 思想本身不能
修复其被 root prior 锁住的绝对偏差。

## 4. 与 1b 冻结合同的逐项对应

| 问题 | 开源系统中的可比模式 | 1b 冻结选择 | 关键差异 |
|---|---|---|---|
| support 不足时是否接收观测 | Kimera 保留 feature tracks 后再按 track 长度入 smart factor；OKVIS 对已有 landmark 先加 observation，新点另受 keyframe 门。 | coast packet 写入既有 window/track/map；factor 仍受既有资格门。 | 不改变 `min_pnp_inliers`、keyframe/seed 门或 factor 类型。 |
| 何时形成视觉约束 | 两者均区分 raw/track observation 与图中的有效因子。 | 已摄入 coast observation 仅在既有 landmark、正 disparity、既有 observation-count 等条件齐备时进 `GenericStereoFactor`。 | 不引入 smart factor 或 pose-graph factor。 |
| 老信息如何生存 | Kimera fixed-lag marginalization；OKVIS realtime graph→pose graph/full graph。 | 只依赖现有 10-frame window、既有 landmark 及当前 transaction 生命周期。 | 没有新的 marginal prior、full graph 或持久 local map。 |
| 断后全局对齐 | OKVIS place recognition + loop closure / pose graph；Kimera 可选 LCD+RPGO。 | over-horizon / discontinuity 完成 segment 并诚实重开。 | 不会声称跨断裂世界系连续。 |
| 失跟 / reset | OKVIS2-X 当前快照没有可证实的 lost-component recovery 实现。 | `kVisualOutage` 保持已有语义；hard failure 仍 rollback。 | 不借“local map”掩盖真正 outage。 |

## 5. 对当前决策的回答

**回答：是相邻问题，但不是把 local map 整体搬进来的问题。** 更准确的命名是：

> 1b 是 *active local estimator continuity*：在仍属于同一段、仍在 horizon
> 内的 IMU 连续区间，保持 observation / landmark / window 生命周期，使视觉
> 恢复能够在原世界锚下重新形成现有局部约束。

它**能**解决：coast 中过早停止摄入导致旧段地图耗尽，恢复帧只能在新 segment
单帧 seed、没有跨段 factor 的问题。它**不能**解决：

- 500 ms 以后完全没有重叠观测的真实断裂；
- 1a 已发生的 root prior 绝对误差；
- 重访旧场景时的 global alignment / loop closure；
- 累积漂移的全局一致性或稠密 submap 对齐。

因此，本次对照为 1b 的方向提供了工程类比和边界校验，但不改变已冻结的范围：
先验证同段摄入是否让恢复帧形成已有视觉 factor 并改善产品门；若该机制仍无法
解决真正断裂后的全局错位，后续再单独立项讨论全局 relocalization / pose graph，
不能在 1b 中隐式引入。

## 6. 来源与可复查快照

- Kimera-VIO：官方仓库 `MIT-SPARK/Kimera-VIO`，检索时 `master` 为
  `ce8c59b7b273ab5ac29db7e5572e1623760e19c7`；本机只读副本
  `/home/lin/Projects/lin_ws/slam_ws/repos/Kimera-VIO` 无 `.git` 元数据，故以该
  官方固定 commit 链接为引用锚。
- OKVIS2-X：本机只读副本
  `/home/lin/Projects/lin_ws/slam_ws/repos/OKVIS2-X`，`origin` 为官方
  `ethz-mrl/OKVIS2-X`，HEAD
  `38043e4afe56d9b32a98434cc74e723737dd2bce`；本文全部源码链接固定此 SHA。
- 论文均为作者公开的原始论文：[Kimera](https://arxiv.org/abs/1910.02490)、
  [OKVIS2](https://arxiv.org/abs/2202.09199)。
