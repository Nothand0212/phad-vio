# M4 最小 gyro-aided VO 重启方案（历史）

日期：2026-08-12
状态：历史上位提案；2026-08-12 已由 evidence-gated 修订设计取代，禁止按旧详细计划直接实施。
issue [#36](https://github.com/Nothand0212/phad-vio/issues/36) 尚需在获得远程写入授权后同步为
Q1 Observe 权威链接；本地文档不冒充该远程前置已完成。
基线：`main@7026ebf`（M3 production VO + 已合入的 M4.1 IMU sync 数据通路）

## 目标判断

不继续修补 P2b fixed-lag candidate。先回答一个更小且可归因的问题：在不改变 M3
视觉估计器、关键帧策略、PnP、landmark lifecycle 和 outlier cull/reopt 的前提下，增加
经过对齐的 gyro rotation constraint，能否稳定改善或至少不损害 production VO。

本片准确命名为 **gyro-aided VO**，不称为 full VIO：暂不加入 velocity、gravity、
accelerometer bias、accelerometer factor，也不引入 `IncrementalFixedLagSmoother`。

## 大致做法

1. **先冻结控制组。** 在当前 main 上重跑 MH_01，确认 visual-only 仍与 M3
   `402d1925` 基线一致；以后每片都要求 IMU-off 三主产物保持 byte-identical。
2. **沿用唯一 estimator。** 保留 `StereoVoEstimator` 的 fixed-window batch BA，继续使用
   原 frontend、KF schedule、PnP、cull/reopt 和非 KF 更新路径。不创建
   `CandidatePipeline` 或第二套 estimator。现有 `update` seam 承载视觉测量及对应的
   `[t_prev,t_cur]` IMU segment；alignment、预积分和 factor 构造隐藏在 estimator PIMPL
   内，不把这些细节扩散到 caller。
3. **先 shadow，后影响 posterior。** 第一小步只消费 M4.1 的 IMU segment：核对
   `sum(dt)`、gap 和 frame 合同，用 production visual rotations 估 shared gyro bias，记录
   visual relative rotation、PIM prediction、bias 与 residual，但不改变输出。证据成立后，
   第二小步才在同一 visual batch graph 的相邻 pose 间加入 pose-only AHRS factor。
4. **不用零 bias 或错误 RW。** gyro factor 激活前必须完成 visual-gyro alignment；factor
   covariance 同时包含 sensor noise 与 alignment residual。最小片固定一个已对齐 shared
   bias，不创建 P2b 的 per-KF bias RW chain，因此也不存在把 `gyr_nd` 误作 `gyr_rw` 的
   路径。
5. **分开机制对拍与真实闭环。** bench/composition root 提供固定 observations + 固定 KF
   schedule 的 paired replay，用来隔离“只增加 gyro factor”的局部效应；随后再跑自然闭环
   on/off，单独报告 schedule 是否分叉。该 replay 只服务测试，不成为 estimator 的 public
   interface。

## 暂定验收与停止条件

- IMU-off 首先必须保持 M3 baseline 的产物与指标；不一致则停止，不进入 gyro 调参。
- shadow 必须证明 IMU 区间、bias 与 residual 合同成立；有 timestamp/frame/model 冲突则
  显式失败，不静默关闭 factor。
- MH_01 先报告 exact-common ATE、RPE、coverage，以及 visual-init / PIM-predict /
  posterior 三态。只有达到双方确认的精度门，gyro path 才有资格成为默认路径。
- 若最小 gyro factor 仍不能过门，记录负结果并保持 IMU 默认关闭；届时再决定是否进入
  full inertial `X/V/B + gravity`，而不是转回 P2b candidate 调阈值。
- fixed-lag 将来作为独立课题：必须先在 visual-only 下证明与 production VO 的精度和
  lifecycle 可接受，再单独接入 IMU。

## 明确不做

- 不复用 P2b 的 KF-only / non-KF PnP-only 架构；
- 不在本片修 candidate 的 frame 533 crash、PnP spikes 或 factor ledger；这些只转化为
  新路径的回归测试与反例；
- 不先扫 covariance、Huber 或 PnP 阈值寻找偶然甜区；
- MH_01 未过门前不扩 EuRoC 11/11；已授权的详细计划只覆盖到 MH_01 gate。

当前合同与实施入口：

- [M4 gyro measurement / factor 资格实验设计（修订版）](2026-08-12-note-m4-minimal-gyro-slice-design.md)；
- [Q1 Observe 实施计划](../plans/2026-08-12_m4_gyro_q1_observe_7d3a91e6.plan.md)。

原 [M4 最小 gyro-aided VO 详细计划](../plans/2026-08-12_m4_minimal_gyro_slice_c4e62b35.plan.md)
仅保留为历史，不得执行，其中的旧实现状态不构成新版 Q1–Q5 资格证据。
