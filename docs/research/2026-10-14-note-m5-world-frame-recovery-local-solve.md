# M5 世界系恢复与局部求解诊断

有界恢复已实现：复用 500 ms horizon，窗口内继承世界系，超时提交局部新段并
报告 `AnchorExpired`。本地 core-4 的 8 个生产回放通过本轮门控；V2_03 ATE
1.960225 → 1.809730 m，V1_03 ATE 0.836282 → 0.169270 m。
675 项 unit 与 31 项 recovery script tests 全部执行通过。

阶段一先解耦段内求解与世界系输出，恢复 V2_03 的 1921 帧输入、状态节奏及
`diag.csv` 对拍。该阶段全时长 IMU 桥接的 ATE 为 32.757155 m，暴露了长间隔
传播漂移；以下保留该诊断及实施前的离线预演，最终回放结果见末节。

本轮复用现有 M5 worktree 的 dirty source，HEAD 为 `b3402b5`，配置
`default_4e2b30f2`，启用已有 moving bootstrap，500 ms active coast。
已入库的[验收证据与完整参数快照](../benchmark/m5/2026-10-14-bounded-world-frame-recovery.evidence.json)
保存逐序列质量向量、六组边界、配置和两臂身份。原始回放和诊断文件保存在本地
`build/world-frame-recovery-20261013/`；下文指向该目录的链接仅在本机可用。
2026-10-15 已补齐 TUM-VI room1–6 guardrail，12/12 回放通过，#58 的本地验收
已完成；M4 与 #51 checkpoint 仍按各自合同验收。

## 局部求解与世界输出

首个恢复 root i459 后的 i460 PnP 使用相同的 31 组对应点、像素、顺序、
内参和外参。世界系表达前后，3D 点的等变残差不超过 1.594e-13 m，camera
geometry 残差不超过 1.985e-13 m，实际 PnP 内点数仍为 28/26。
这证明几何等变不足以保证该求解器在两种坐标表达下产生相同的内点集合；
本轮保留其局部输入，消除世界系表达对后续求解的干扰。

`VioUpdateState` 新增 private optional `T_W_local`。Solver、landmark 与 pose
cache 留在段内坐标；公开 pose 左乘 transform，velocity 只旋转，bias 保持
新局部解。下一次 outage 捕获公开世界系 navigation，完成段和 discontinuity
清除 transform，fatal 保持完整 rollback。

新增测试 `RecoveryKeepsPnpInputsInSegmentFrame` 经 RED/GREEN 验证；已有非零
yaw/translation、重复 outage、static/discontinuity 与 fatal 测试继续通过。
全部 estimator tests 为 223/223。全 unit 首次 667 项通过、4 项因数据集环境
跳过；设置 `PHAD_EUROC_MH01_PATH` 后补跑 4/4 通过，671 项均已执行。
阶段一独立审查未发现可验证缺陷。

两臂的输入、status/segment、initialization path/outcome/attempt/current-graph/
transaction 与 root/outage cadence 一致；全部 initializer JSON 有两处
recoverable rejection 的 phase/reason 差异，位于 i444 与 i1086。
完整差异保存在 [`input-cadence-analysis.json`](../../build/world-frame-recovery-20261013/input-cadence-analysis.json)。

## 全量回放与传播误差

以下三组是独立的完整本地 V2_03 回放，均为同一配置：

| 求解与表达 | ATE RMS m | RPE RMS m |
|---|---:|---:|
| Control 新段局部表达 | 1.960225 | 0.416013 |
| 原整窗世界系转换 | 35.978999 | 0.067615 |
| 段内局部求解与世界输出解耦 | 32.757155 | 0.069868 |

Control 与局部求解方案均发布 942 poses，completion 为 0.490369599，coverage
为 0.724935733，7 个 segment，failed/rejected 均为 0。
局部求解修复恢复了可比输入；它单独不足以改善全局精度。

六组真实恢复间隔为 0.4–14.2 s，不能按 frame index 差乘固定周期估算。
同一旧 state、新 root 与 raw IMU 下，旧 bias 直接传播的边界 translation
RMS 为 35.813320 m；仅换新 root bias 仍为 12.214511 m。
端点姿态与速度闭合候选在修正长旋转 tangent 分支后，边界 RMS 可降至
3.455533/3.587609 m，但全局 ATE 为 4.726010/4.993420 m，未达到总体精度门。
这些 probe 保留研究身份，见 [`gap-closure-analysis.json`](../../build/world-frame-recovery-20261013/gap-closure-analysis.json)。

## 实施前的 500 ms 有界恢复预演

有界候选复用现有 500 ms horizon：仅 i937 的 0.4 s 恢复继承世界系，另外
五次超时保留新段。用户于 2026-10-14 确认采用此方案，
[#58 合同](../specs/2026-09-07-m5-post-outage-world-frame-recovery.md)及
诊断 `AnchorExpired` 已按此实现。

下表保留冻结 control 轨迹的离线重表达，实际产品回放见下一节。
固定六组事件全部计入，使用相同 GT 端点 association；每组按
`E = inverse(inverse(T_gt(i))*T_gt(j)) * (inverse(T_est(i))*T_est(j))`
计算 translation error，扣除 GT 的真实运动。

| 边界 i → j | 间隔 s | Control error m | 有界预演 error m |
|---|---:|---:|---:|
| 433 → 459 | 1.300000000 | 3.656201 | 3.656201 |
| 701 → 809 | 9.250000128 | 4.875073 | 4.875073 |
| 929 → 937 | 0.400000000 | 3.382952 | 0.055674 |
| 1010 → 1097 | 6.700000000 | 5.368271 | 6.235210 |
| 1157 → 1343 | 14.200000000 | 3.559579 | 3.559579 |
| 1521 → 1563 | 3.350000128 | 3.273677 | 3.273677 |

六组 translation RMS 为 4.098463 → 4.070271 m，median 为 3.607890 m；
max 为 5.368271 → 6.235210 m。Rotation RMS 为 1.714967 → 1.751430 rad。
因此该方案只解决短中断，跨长中断的世界系连续仍需视觉重定位等后续证据。

同一 942 timestamp 支持集、各臂仅一次 fixed-scale SE3 对齐，native evaluator
报告 ATE 1.960225 → 1.80973 m，1 s RPE 0.416013 → 0.0698681 m；
RPE 为相同的 808 pairs。预演的完整旋转指标、GT association、逐事件结果见
[`boundary-analysis.json`](../../build/world-frame-recovery-20261013/boundary-analysis.json) 与
[`preview-bounded-eval.log`](../../build/world-frame-recovery-20261013/preview-bounded-eval.log)。

## 实际回放与 core-4 验证

两臂共用生产编译选项和 canonical config `4e2b30f2`。测试宏 control 曾产生
TUM 浮点末位差异，保留为诊断。生产 control 的 `diag.csv/est.tum/kf.tum`
与历史 control 全部 byte-identical；V2_03 两臂首次 Applied 前全部算法 payload、
全程有序 frontend 输入、完整 initialization JSON 和状态节奏均一致。

V2_03 固定六组边界全部保留。i937 Applied，另五次为 AnchorExpired；942/942
估计 poses 均被 native GT association 匹配，两臂使用相同的 808 RPE pairs。
实际 translation RMS 为 4.098463 → 4.070273 m，max 为
5.368271 → 6.235216 m；rotation RMS 为 1.714967 → 1.751430 rad。
ATE 1.960225 → 1.809730 m，RPE 0.416013 → 0.069868 m。
两项主精度门下降，completion/coverage 不降，本轮 V2_03 产品门 PASS。

| 序列 | Control ATE m | Subject ATE m | Control RPE m | Subject RPE m | GT matched / output poses | Applied / Expired |
|---|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 0.120218 | 0.120218 | 0.031596 | 0.031596 | 3638 / 3676 | 0 / 0 |
| V1_03_difficult | 0.836282 | 0.169270 | 0.159416 | 0.060888 | 2052 / 2088 | 1 / 0 |
| V2_02_medium | 0.206634 | 0.206634 | 0.045316 | 0.045316 | 2265 / 2284 | 0 / 0 |
| V2_03_difficult | 1.960225 | 1.809730 | 0.416013 | 0.069868 | 942 / 942 | 1 / 5 |

四序列 arithmetic mean ATE 为 0.780840 → 0.576463 m，下降 26.17%；
mean RPE 为 0.163085 → 0.051917 m，下降 68.17%。全部 8 个进程
failed/rejected 为 0。四序列输入、完整 initialization JSON 和全部 diag payload
均一致；native GT 支持集及 RPE pairs 一致，逐序列 ATE/RPE、completion/coverage
均不回退。GT 匹配数小于输出数的部分来自同一 native association，保留完整分母。

V2_03 completion 仍为 0.490369599、coverage 0.724935733，7 个 segment；
V1_03 为 2 个 segment，MH_01/V2_02 各 1 个，两臂相同。
本轮收益集中于短中断的世界系继承；长中断和最大跨段误差仍需后续工作。
这是本地 core-4 的候选结果，#51 formal exact RED 和 EuRoC-11 checkpoint
资格沿独立合同处理。

完整工作快照的最终检查为 675/675 unit（无 skip）与 31/31 recovery script tests。独立审查
未发现 C++ correctness 缺陷；全过期时多余轨迹尾行漏检经失败测试复现后修复，
并补齐已安装非 identity transform 后 active fatal → rollback → retry
clean-equivalence 的测试。

发布时从既有 dirty 工作中分离 #58 必要依赖，导出暂存区独立构建；该快照的
642/642 unit 与 31/31 recovery script tests 全部执行通过。两臂固定相同的
生产编译选项及 estimator object 链接位置（紧邻且前置于 `libphad_estimator.a`），
该身份下 core-4 的 8 个进程全部复现冻结的各臂 trajectory/diag payload 与完整
有序输入。初次不对称链接的复核在 V2_03 TUM 出现最大约 1.03e-11 的
末位差异，按身份门排除；同一链接身份的回放结果随 evidence 记录。

完整逐序列质量向量、GT 支持、时间统计、输入身份和固定六组边界见
[入库 evidence](../benchmark/m5/2026-10-14-bounded-world-frame-recovery.evidence.json)。
重建 control 的单变量源码差异见
[control patch](../benchmark/m5/2026-10-14-bounded-world-frame-recovery.control.patch)。
实施计划见 [`58c9f001`](../plans/2026-10-14_m5_world_frame_recovery_58c9f001.plan.md)。

## 2026-10-15 TUM-VI room1–6 验收

提交 `3ea548b` 的 TUM-VI guardrail **PASS**：6 条序列、12/12 个完整生产回放，
bridge 全程 `NotApplicable`。每条序列的完整有序输入、initialization、bridge sidecar
及 `diag.csv/est.tum/kf.tum` 全量对拍一致；trajectory 支持数量和可用性一致。
使用 native `dso/gt_imu.csv`（body=IMU），association tolerance 为 4.166667 ms，
min match rate 为 0.80，RPE interval 为 1 s。完整参数快照随 evidence 保存。

| 序列 | processed frames | output poses | ATE RMS m（两臂相同） | RPE RMS m（两臂相同） | 分段 |
|---|---:|---:|---:|---:|---:|
| room1 | 2821 | 2816 | 0.789953 | 0.116896 | 1 |
| room2 | 2882 | 2876 | 0.560701 | 0.056133 | 1 |
| room3 | 2821 | 2816 | 0.436790 | 0.052093 | 1 |
| room4 | 2227 | 2222 | 0.367008 | 0.077148 | 1 |
| room5 | 2846 | 2841 | 1.345398 | 0.338915 | 1 |
| room6 | 2636 | 2631 | 0.289299 | 0.053960 | 1 |

表中 processed frames 为实际处理的 packet 数，原始输入与同步数量见 `summary.sync`。
12 个进程合计 failed=0、rejected=0；常规 warnings 与完整质量向量按原样保留。
复用此前验收的生产二进制，并逐文件核对 213 份源码与提交内容一致；两臂共用
编译选项、estimator object 链接位置和依赖。运行 meta 的 dirty 标记来自 `--repo`
引用的工作树中保留的其他未提交工作，被测代码身份由独立 source/binary manifest 固定。

本轮新增占用观测峰值约 1.688 GiB，低于 3 GiB 上限；
临时输入逐序列回收，留存压缩日志、轨迹和 input manifests 约 120.7 MiB。
原始 archive 保持原位，复用已有构建。

[完整机器证据](../benchmark/m5/2026-10-15-bounded-world-frame-recovery-tumvi-guardrail.evidence.json)。#58 的本地产品验收已完成；
#51 formal exact RED 和 M5 default-on 资格仍沿各自合同处理。
