# M4 多序列产品 gate 设计笔记

## 1. 问题

`MH_01_easy` 在 clean `42f99e9/default_0337287b` 上得到 ATE
`0.0725592618 m`，相对 M4 checkpoint 的 `0.0705388305 m` 增加
`0.0020204313 m`（`+2.86%`）；同一次 run 的 RPE 从
`0.0331756107 m` 降到 `0.0326412725 m`（`-1.61%`）。该结果说明两件不同的事：

1. [Q0 首分叉诊断](2026-08-27-note-m4-mapped-bearing-q0-control.md) 已经定位到
   哪个既有 factor admission 行为产生数值差异；
2. 单条 easy sequence 的绝对 ATE 上界，不能单独回答该行为对整个 EuRoC
   产品面的净收益。

第一项是机制归因，第二项是产品判定。两者不能互相替代。

## 2. 当前四序列证据

下表比较 M4 checkpoint `c999f58/default_0337287b` 与 `42f99e9` 的既有
record-only 四序列结果。后者 source manifest 为 dirty，因此这些数字只用于设计
判定方法，不能作为 clean Q0 或 candidate PASS。

| sequence | ATE ratio | ATE 变化 | RPE ratio | RPE 变化 |
|---|---:|---:|---:|---:|
| `MH_01_easy` | `1.0286` | `+2.86%` | `0.9839` | `-1.61%` |
| `V1_03_difficult` | `0.6123` | `-38.77%` | `0.2450` | `-75.50%` |
| `V2_02_medium` | `0.0866` | `-91.34%` | `0.1204` | `-87.96%` |
| `V2_03_difficult` | `1.2189` | `+21.89%` | `0.8201` | `-17.99%` |

四序列 ATE 算术均值从 `1.291756 m` 降到 `0.796747 m`；RPE 算术均值从
`0.470125 m` 降到 `0.227234 m`。按每条 sequence 等权、先对各自 checkpoint
归一化再取几何平均，ATE ratio 为 `0.5077`，RPE ratio 为 `0.3928`。

因此，`MH_01_easy` 的单项上界会在其余序列证据尚未收集时提前停止一个可能在
整体上显著改善的候选。反过来，只看算术均值也会让数值较大的 difficult sequence
主导结论，不能表达每条 sequence 的相对变化。

## 3. 一手实践对照

- [KITTI odometry benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
  在多个 sequence、多个子轨迹长度上计算 translation / rotation error，再对全部
  子轨迹求平均；同一方法与同一参数集用于全部 sequence。
- [ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644) 在 EuRoC 的 11 条
  sequence 上逐条报告 RMS ATE，并用重复运行的中位数处理系统的运行间波动，而
  不是由首条 sequence 决定整个方法是否继续评估。
- [EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/) 本身包含
  machine hall 与 Vicon room 的多种难度条件；只用 easy sequence 不能覆盖当前
  continuity slice 面向的退化场景。

由这些实践得到的设计推断是：产品 gate 应先完成预注册 suite，再综合 suite-level
outcome、每序列 tail、liveness 与机制证据；不应把任一单序列的小幅 accuracy
变化同时当作全套实验的停止条件与产品否决条件。

## 4. 判定模型

### 4.1 三类指标

| 角色 | 指标 | 判定用途 |
|---|---|---|
| primary outcome | 对 clean Q0 归一化后的多序列 ATE 几何平均 | 判断整个 suite 的全局精度净变化 |
| driver | segments、段内 RMS、绝对段间分量、同段 recovery | 判断 continuity 机制是否改善目标失败模式 |
| guardrail | RPE、completion、coverage、failed/rejected/reanchors、有限性 | 防止用短轨迹、失败轨迹或局部改善换取表面 ATE |
| mechanism evidence | eligible/attached mono factor、flip-capable episode、factor count invariants | 证明变化确由本片机制产生 |

对 suite 中每条有效 sequence `s` 定义：

```text
r_ATE(s) = ATE_candidate(s) / ATE_Q0(s)
G_ATE    = exp(mean_s(log(r_ATE(s))))

r_RPE(s) = RPE_candidate(s) / RPE_Q0(s)
G_RPE    = exp(mean_s(log(r_RPE(s))))
```

归一化避免 difficult sequence 仅因绝对误差较大而支配结果；几何平均让等比例改善
与退化对称，并让每条 sequence 等权。原始 ATE/RPE、算术均值和逐序列 ratio 仍
全部落盘，不能只保留 aggregate。

### 4.2 两层 suite

1. `core-4`：`MH_01_easy`、`V1_03_difficult`、`V2_02_medium`、
   `V2_03_difficult`。用于 Q0、开发期短反馈与失败模式分解。
2. `EuRoC-11`：最终产品判定。candidate 与同 identity/config 条件下的 clean Q0
   全量对拍，避免四条人工选择的 sequence 隐藏其他场景回归。

accuracy 数值变化不触发 suite early-stop。单条运行失败也应记录并尽量继续后续
sequence；只有 source/config/input/evaluator 身份失效、使后续结果同样不可解释
时，才停止整个 suite。

### 4.3 三态结论

- `PASS`：identity 与 hard validity 成立；机制门成立；`core-4` 与
  `EuRoC-11` 的 aggregate accuracy 满足预注册 envelope；continuity target 改善；
  guardrail 没有不可接受的整体退化。
- `REVIEW`：aggregate 与机制门成立，但存在单序列 ATE/RPE、coverage 或
  completion tail。必须结合该序列的 segments、段内/段间分解与绝对误差量级解释；
  单项 tail 本身不自动否决 candidate。
- `FAIL`：身份或有效性失真、机制未实际触发、出现 hard runtime failure，或
  suite-level outcome/guardrail 越过预注册 envelope。

## 5. 冻结时点

先完成 clean Q0 的 `core-4`，再补齐 clean `EuRoC-11`。在看到 Q1/Q4/Q5
candidate 结果前冻结：

1. aggregate ATE/RPE envelope；
2. completion/coverage 的有效性界限；
3. 进入 `REVIEW` 的 per-sequence tail 规则；
4. continuity 改善量与 V2_03 机制非零要求。

“多序列完成后判定、单序列 accuracy 不 early-stop”的结构已经冻结。当前证据
不足以凭一条 clean 结果为以上四项指定可信数值；数值应由完整 clean Q0、既有
M4 checkpoint 和绝对工程意义共同确定，之后不得根据 candidate 放宽。
