# M5 世界系恢复优化实施计划

> 执行方式：本会话按 `executing-plans` inline 实施。复用 #58，Git / 远程写入按独立授权执行。

**目标**：在同一 V2_03 输入、局部 root 与状态节奏下，降低恢复边界误差和全局 ATE。

**架构**：段内 PnP、IMU factor、LM、landmark/cache 保持局部坐标；原子安装的
`T_W_local` 只用于返回世界系 pose/velocity 与下一次 recovery anchor。
长间隔位移的候选只消费已验证的新 root，不回写局部图、不更改 admission。

**技术栈**：既有 C++20 / Eigen / GTSAM / OpenCV；EuRoC V2_03。

**依据**：`docs/specs/2026-09-07-m5-post-outage-world-frame-recovery.md` 的外部目标、
typed lifecycle、gravity-preserving yaw、全六组边界和共同 timestamp 评估。历史
fixed-input STOP 保留；本轮独立验证新的内部坐标表达和传播候选。

## 约束与验收

- 单一 `VioEstimator::update()`，500 ms active coast，bounded window；bias 保留新局部解。
- 世界系表达不得改变 frontend 输入、outlier feedback、status/segment/root/attempt cadence。
- static/discontinuity 消费或清除 anchor / transform；fatal 完整 rollback。
- 六组边界全部进入 GT-corrected translation RMS；ATE 共同支持集仅一次 SE3 对齐。
- V2_03 产品门：有限、failed/rejected=0，边界 RMS 和共同 ATE 下降，可用性不降。
- V2_03 通过后验证 core-4；其他数据集依据实际触发范围追加。

## 1. 段内求解与世界输出解耦

文件：`phad/estimator/vio_estimator.cpp`、`internal/vio_update_transaction.hpp`、
`tests/estimator/vio_full_state_test.cpp`。

- [x] 保存现有 dirty source 与 before binary；构建独立 control / subject。
- [x] 复现 i460 的 31 点 gauge 等变、相同像素、PnP masks 28/26。
- [x] `RecoveryKeepsPnpInputsInSegmentFrame` RED：世界重表达改变 PnP 输入。
- [x] 最小修改：私有 optional `T_W_local`，世界输出 / anchor 转换，清除与 rollback。
- [x] GREEN：223/223 estimator tests；V2_03 输入与 diag 1921/1921 对拍一致。
- [x] 保留实际产品失败：control ATE 1.960225 m；局部图 subject ATE 32.757155 m。

## 2. 长间隔传播的有界候选

文件以实际证实的传播原因决定；候选优先复用 GTSAM 预积分和新根导航状态。

- [x] 同一 control 记录全六组旧 state、新 root 与实际 raw IMU；最大间隔 14.2 s。
- [x] 直接旧 bias 的传播边界 RMS 35.813320 m；fresh-bias 单独替换仍为 12.214511 m。
- [x] 用独立端点姿态 / 速度约束检查传播位移；处理长旋转的 tangent 分支后裁决。
- [x] 保留失败结果：端点闭合降低边界 RMS，但整体 ATE 4.726010 / 4.993420 m，未过产品门。
- [x] 离线有界预演：仅 0.4 s 的 root i937 桥接；共同 ATE 1.80973 m，六组边界 RMS 4.070271 m。
- [x] Expired root 与 writer/parser RED；实现有界分支与诊断，GREEN。
- [x] 同一 V2_03 全量闭环 PASS：共同 ATE 1.809729554 m，六组 RMS 4.070272737 m，可用性不降。

**已确认设计（2026-10-14）**：复用既有 500 ms horizon，超时 root 保持新段并报告
`AnchorExpired`。用户选择「采用有界恢复」；#58 合同与公开诊断枚举按此修订。
六组边界全部计入，预演最大 translation error 由 5.368271 增至 6.235210 m；
长中断的世界系连续仍是后续工作。

## 3. 验证和交付

- [x] 完成阶段一构建、223/223 estimator tests 及 static/discontinuity/repeated-outage/fatal rollback。
- [x] 全 unit 667 项通过、4 项缺数据集环境跳过；设置 MH_01 路径后 4/4 补跑通过，671 项均已执行。
- [x] V2_03 通过后完成 core-4，8/8 生产进程、全程输入及 diag 相同，ATE/RPE 与可用性逐序列不回退。
- [x] 阶段一独立代码审查，相对 `before/` 未发现可验证缺陷；原有 dirty 工作保留。
- [x] 阶段一与有界预演记入 research / machine-readable evidence，区分实际运行与离线表达。
- [x] 设计确认后完成候选；675/675 unit（无 skip）、31/31 recovery script tests。
- [x] 最终只读审查：C++ 未发现可验证缺陷；全过期额外轨迹尾行 P2 经 RED/GREEN 修复。
- [x] 补充已安装非 identity 世界 transform 后 active fatal → rollback → retry clean-equivalence。
- [x] core-4 结果回填 research 和 evidence：mean ATE 0.780840 → 0.576463 m，mean RPE 0.163085 → 0.051917 m。

**编译身份裁决**：测试宏 control 的 `diag.csv` 一致，但 TUM 浮点末位变化，
保留为诊断、不进入产品统计。重新以生产编译选项构建 control 后，
`diag.csv/est.tum/kf.tum` 均与历史 control byte-identical；S1 前缀、全程有序 input、
完整 initialization JSON 与 cadence 均一致。两臂使用共同生产编译选项。

## 本地证据

已入库的[验收证据](../benchmark/m5/2026-10-14-bounded-world-frame-recovery.evidence.json)
保存本轮 core-4 与固定六组边界、完整参数快照。2026-10-15 的
[TUM-VI room1–6 guardrail](../benchmark/m5/2026-10-15-bounded-world-frame-recovery-tumvi-guardrail.evidence.json) 已通过。

`build/world-frame-recovery-20261013/`：`source-before.tar`、`existing.diff`、
`before/`、before/control binaries、三组 full runs、RED/GREEN tests、真实 PnP
capture、gap probes。目录名标记首次创建日，后续产物沿用同一任务目录。

## 2026-10-15 后续验收

- [x] 用户选择第 1 项后，完成 room1–6 的 12/12 个生产回放。
- [x] 全程 NotApplicable；完整有序输入、initialization 与三类算法 payload 对拍一致。
- [x] 逐序列 native 指标、配置和 source/binary identity 入库。
- [x] 复用构建；临时数据按序列回收，观测峰值 1.688 GiB，留存约 120.7 MiB。
