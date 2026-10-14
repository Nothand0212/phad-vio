# M5 recovery pair 首个输入分叉记录

- 日期：2026-09-07；Issue：[#58](https://github.com/Nothand0212/phad-vio/issues/58)
- 合同：[recovery spec](../specs/2026-09-07-m5-post-outage-world-frame-recovery.md)；计划：[58a7c201](../plans/2026-09-07_m5_post_outage_world_frame_recovery_58a7c201.plan.md)
- 状态：**fixed-input/cadence 协议 STOP；main 与 reviewer 已双重确认，review 结束。**

C1 qualification PASS，`diag.csv/est.tum/kf.tum` 与冻结 baseline 全字节一致。
六个恢复 root `459/809/937/1097/1343/1563` 全部 Moving Eligible。
S1 build、unit、V2_03 均完成且 rc=0；V2_03 terminal 为 warning，paired gate STOP。

## 证据位置

Base：`/tmp/phad-m5-recovery-pair-launcherfix-20260907`，以下路径相对 base。

| 产物 | 内容 |
|---|---|
| `plan.json`、`control-start.json`、`subject-start.json` | source/input/config 与 run identity |
| `control-exec/check-control.stdout` | 六个 Eligible index |
| `first-failure.json`、`subject-exec/check-pair.stderr` | outage.indices gate STOP |
| `paired-first-protocol-differences.json` | 各类首差；两臂均 1921 rows |
| `post-first-apply-diag-through-input-fork.json` | i460 至 i484 的 diag 差异 |

Raw C1：`raw/runs/20260907T083418Z-ci-euroc11-f4e649aaea52-1b6137/remote/`。
Raw S1：`raw/runs/20260907T083920Z-ci-euroc11-3946461dee84-56b48f/remote/`。
各自 `run-status.json` 保存 terminal；`tasks/V2_03_difficult/artifacts/bench/`
保存 sidecar、diag、est/kf TUM、meta 与 summary。

## 已确认事实

Gate `outage.indices` expected `[434,702,930,1011,1158,1522,1631]`，
actual `[434,702,930,1010,1159,1523,1631]`；最早真实输入分叉在 i484。

| processed i | 观察字段 | C1 | S1 |
|---:|---|---|---|
| 460 | frontend 相同；diag pnp_inliers | 28 | 26 |
| 483 | frontend 相同（73 obs）；diag outliers_culled | 0 | 2 |
| 484 | 首次 frontend.observations.length 差异 | 34 | 32 |
| 716 | initialization.reason 首差 | 6 | 5 |
| 808 | status 首差 | initializing | ok |
| 1010 | segment_id 首差 | 3 | 4 |

I484 timestamp 为 **1413394906105760512 ns**；第一个不同 obs 在 zero-based 位置 **5**：
`C=[27047,239.86117553710938,316.2589111328125,0.0]`，
`S=[27154,419.6762390136719,262.02667236328125,0.0]`。
共同 ID 值相同，集合与有序数组不同；该行两臂均 status=ok、KF=true、segment=1。
这是有序 ingestion 硬门失败，不是 typed bridge 误读；i460/i483 只列相关证据，不归因。
`rows 0..458` 的 diag/est/kf 原 bytes 独立校验通过；i459 的
`Eligible/world_origin_segment_id=1 → Applied/world_origin_segment_id=0` 是预期变化。

## 停止边界

未做全六组 J/共同支持 ATE 产品裁决，未扩 core/TUM，未调整阈值或 PnP。
Evaluator 准备产物保留在 `evaluator-prep/`；当前不作机制证伪或数值收益失败结论。
下一候选只读因果线为 i460 同输入下 gauge 等变性检查，尚未执行；当前停写、停止调查。
[MH_01 历史 formal RED](2026-09-03-note-m5-mh01-formal-rpe-first-pair.md) 继续独立记账。
