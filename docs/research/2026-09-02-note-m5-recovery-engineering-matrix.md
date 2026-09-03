# M5 recovery engineering matrix

- 日期：2026-09-02（Asia/Shanghai）
- Issue：[#51](https://github.com/Nothand0212/phad-vio/issues/51)
- 状态：**已按预注册协议执行完成；结果见
  [M5 recovery engineering matrix results](2026-09-02-note-m5-recovery-engineering-matrix-results.md)**

## 1. 目的与边界

本协议为 M5 recovery 建立独立的工程评估矩阵。它回答的是：在满足结构与
安全合同的前提下，候选变更在不同真实序列、不同精度指标、连续性和运行代价上的
工程影响是什么。

它不替代或修改 #51 formal matrix：`mh01-static` 的 exact control identity、
candidate cadence、typed lifecycle 与 `1e-6 m` 比较仍按
[M5 plan](../plans/2026-09-01_m5_initial_moving_root_51a1b7e2.plan.md)执行。
其中 `V1_02-118` 是 ADR-0004 availability handoff verifier，并不属于 #51 formal
matrix。formal red 依根
[AGENTS](../../AGENTS.md) 的 failure protocol 停止其当前矩阵并保留首个分叉；
它不自动给出工程回滚结论。

本协议把根 AGENTS 的三层职责落实为独立报告面：hard correctness 继续直接阻止
对应行为或交付；exact regression 记录可复现差异；engineering significance 才用于
产品接受、继续实验或回退。任何工程矩阵结果均不反向放宽 hard correctness。

## 2. 已有隔离重复性事实

当前已有一次隔离的 MH_01 recovery candidate run-1/run-2 repeatability check：算法产物 hash
一致，评测 metrics delta 为零。该事实只证明该固定输入、固定实现与该执行条件下的
算法/评测链可重复；它不构成多序列工程矩阵结果，也不构成 runtime baseline。两个
run 的 runtime 没有在预注册的同负载、同资源条件下比较，故 runtime 结论为
**不可比**。

NIST 将同一对象在同一 procedure、instrument、location 和短时间条件下的重复测量
一致性定义为 repeatability，并以结果离散度描述；改变条件后的 reproducibility 必须
明确列出哪些条件变化。[NIST TN 1551](https://www.nist.gov/document/nist-tn-1551pdf)
同样要求把 resolution 与 uncertainty 同测量过程关联，而不是把微小浮点差直接解释为
算法事实。[NIST Measurement Process Characterization](https://www.itl.nist.gov/div898/handbook/mpc/section5/mpc52.htm)

因此本协议的三次独立进程是报告 run-to-run repeatability envelope 的操作最小值，
不是统计显著性或实践重要性的阈值；host、build、evaluator 或输入身份变化时，结果
按显式条件变化报告。

## 3. 比较角色与输入层级

每个候选固定两个不同职责的参考角色：

| role | 身份与用途 | 是否作为工程效应比较对象 |
|---|---|---|
| `C_formal` | `ba607c5` clean exact control / `0337287b`；只承担 #51 identity、historical numeric baseline 与 deterministic repeatability | 否 |
| `C_engineering` | 候选变更前、最近的可复现 M5 artifact 或 commit；与 candidate 使用同一 manifest、CLI、build、host 与 evaluator | 是 |

`C_engineering` 的 full commit/tree、dirty state、artifact hashes 与运行条件必须在
candidate 之前冻结。它不能因结果较好而事后替换。若当前工作树无法提供可复现的
工程对照，则本矩阵只报告候选绝对结果与 repeatability，不报告 candidate-control
工程效应。

| 层级 | 本地输入 | 作用 | 精度聚合 |
|---|---|---|---|
| structural | synthetic/unit、EuRoC `V1_02_medium` fixed 118-packet prefix、TUM-VI `dataset-corridor1_512_16` corridor1-100 | typed lifecycle、输入/loader、finite trajectory、连续性 sentinel | 不进入工程精度总览 |
| 主工程精度集 | EuRoC native 11条：MH_01–05、V1_01–03、V2_01–03 | 按 easy/medium/difficult 覆盖真实精度、健康与runtime | 仅在此层内聚合 |
| 独立来源 sentinel | TUM-VI corridor1 | 检查跨数据源结构行为与资源方向 | 单列，不并入 EuRoC |

当前 TUM-VI corridor1 路径没有 GT adapter；因此它只报告 structural、trajectory
health、typed diagnostics 与 runtime，**不**计算或比较 ATE/RPE。V1_02 fixed prefix
同样保留其 lifecycle/availability 职责，而不是伪装成独立全序列精度样本。

## 4. 运行设计与身份记录

每个 `candidate × C_engineering × EuRoC sequence` 各运行 **3 个独立进程**。
一个完整候选/工程对照对需要：

```text
EuRoC: 11 sequences × 2 roles × 3 repeats = 66 full processes
```

即66个预注册full processes。该数字是工作量估计，不是性能或质量阈值；实际 wall-time
在首次受控 calibration run 前未知，不能预先虚构。`C_formal` 不进入此成本或工程
对照分母，除非 #51 formal 自身要求重新生成它。V1_02与TUM-VI是两个source
identity的单次structural前置门；它们不做3-repeat、不进入本矩阵process数或精度聚合。

### 4.1 远程并行执行

EuRoC主工程精度集固定使用
[`ci-euroc11`](../agents/remote-ci.md) profile在远程服务器执行。每个profile run先做
一次Release共享build与unit，再将11条sequence最多8路并行；profile之间串行，避免
两个8路矩阵互相争用资源。两个角色各启动3个独立profile runs，共6个remote run、
66个full sequence processes。固定顺序为：

```text
C_engineering-1 → candidate-1 → candidate-2 → C_engineering-2 →
C_engineering-3 → candidate-3
```

该顺序在查看任何EuRoC-11结果前冻结。`start`只启动后台worker；因此每个箭头都是
同步边界：run N必须先由`wait <run-id>`到达终态并完成fetch，才允许执行run N+1的
`start`。每个run使用全新run-id与输出目录；相同角色的3个runs必须具有相同
source-id、image identity与profile。

两个角色启动前分别冻结canonical source manifest，并记录`C_engineering` manifest
SHA-256、recovery patch SHA-256与candidate manifest SHA-256。manifest diff必须只含
recovery patch声明的路径和mode/content变化，symlink与其余tracked/untracked条目必须
一致；随后remote `source_id`还必须对应各自已冻结snapshot。`source_id`不同本身不能
证明“只差recovery patch”。任一identity不一致时，该run不进入矩阵并按原角色重跑。

远端当前已验证32 CPU、98.7 GB memory及EuRoC-11完整数据；profile内8路并发满足本轮
“远程并行”的执行约束。由于profile没有TUM-VI GT adapter、V1_02 opt-in product task
或受控CPU affinity，本轮远程矩阵只执行66个EuRoC full processes。V1_02与TUM-VI继续
使用已冻结structural gates单列报告，不计入远程precision process数；不得把它们写成
已完成的远程精度证据。Runtime记录wall/RTF与并发条件，但当前仍标记为不可作产品裁决。

每个进程必须保存：

- `meta.json`、`summary.json`、`diag.csv`、`est.tum`、stdout/stderr 和所有
  typed sidecar；
- full commit、branch、dirty state、源码/构建目录 identity、compiler、GTSAM、OpenCV、
  evaluator 与 canonical config text/hash；
- dataset、GT、calibration manifest/digest，CLI，CPU/OS、频率/affinity 与并发负载；
- artifact SHA-256，运行起止时间和环境中影响 runtime 可比性的条件。

任何缺失 identity 的进程不进入 paired 或 runtime 聚合。SLAMBench2 将 accuracy、
execution time、memory 和 energy 作为共同量化的 trade-off，并要求实验参数和统计量
被清楚记录；本项目沿用其“可比较、可复现、完整身份”的原则，但只报告当前 bench
已经稳定提供的 timing 指标。[SLAMBench2 paper](https://www.pure.ed.ac.uk/ws/files/54741939/main.pdf)
与 [SLAMBench repository](https://github.com/pamela-project/slambench)

## 5. 预注册指标

### 5.1 精度与配对分布

EuRoC 的每个 repeat 报告 ATE 与 RPE 的 translation（m 和 mm）及 rotation（deg）：
baseline、actual、absolute delta 后才报告百分比。ATE 使用现有固定尺度 SE3 对齐；
RPE 使用 1 s fixed-time delta 且不对齐，均沿用 `phad::eval` 的关联合同。
[phad/eval README](../../phad/eval/README.md) 定义了 ATE association/SE3 alignment
与 RPE 的 fixed-time 语义；[rpe.cpp](../../phad/eval/rpe.cpp) 明确从 matched estimate
timestamps 选择只向后的 partner。

candidate/control 的每条 EuRoC sequence 另报告：

- common timestamp intersection 上的 ATE sample distribution；
- common `(from_timestamp_ns, to_timestamp_ns)` RPE-pair intersection 上的
  translation/rotation error delta distribution；
- 每个分布的 count、RMSE、mean、median、p10、p90、max，以及每个重复的相同摘要；
- 各自 association/pair count、dropped-no-partner 与任何缺轨迹原因。

缺少的 timestamp/pair 不得通过只报较小的 RMSE 隐去；它与 completion/coverage
一起作为独立健康结果记录。

### 5.2 连续性、机制与运行代价

每条 run 同时报告：

- `ok/image_frames`、completion、coverage、failed、rejected、`segment_id`跳变、
  `trajectory.segments`；当前 `reanchors` 仅作历史记录，连续性以实际
  `segment_id` 和 segments 判定；
- PnP primary/retry/fallback 分类、current-graph/IMU typed字段、keyframe率、
  cheirality、cull/drop/zombie 和其他与候选机制直接相连的诊断；
- wall time、RTF，以及 rectify/frontend/estimator/total 的 mean、p95、max。

Runtime 只在相同 host、build、affinity、并发负载与输入身份的 repeat 内比较；条件
不同则逐项列出并标为 runtime 不可比。不能把 runtime 不可比扩展为精度不可比。

## 6. 聚合与工程判断

工程报告先保留 EuRoC 11条的完整向量，再按 easy/medium/difficult 分层展示每序列
median 与 range。总览使用“每序列 median 的等权分布”，避免长序列按帧数主导结果；
不生成跨数据源的单一总分，TUM-VI/V1_02 structural 结果独立列出。

每个指标都同时保留绝对量、paired distribution、repeatability envelope 和 trade-off
向量。若某项的变化小于 evaluator/measurement resolution 或 run-to-run spread，报告为
数值差异已观测、实践解释未确认；不得把它升级为算法优劣事实。

本轮尚未有经产品风险、传感器/GT/evaluator precision 和资源预算共同预先批准的
practical margin。因此本矩阵的工程结论只允许使用以下状态：

| 状态 | 含义 |
|---|---|
| `directional and repeatable` | 同方向变化在预注册序列与重复中可见，但未声明产品重要性 |
| `mixed across sequences/metrics` | 各序列、指标或运行条件间存在trade-off |
| `engineering significance unresolved` | 缺少 practical margin、可比 runtime 条件或足够的 paired/repeatability 证据 |

这些状态均不等价于产品接受或算法否决。任何后续 practical margin 必须在下一轮矩阵
开始前单独对齐和预注册，不能由本轮观测结果反推。

## 7. 停止与分流

1. 输入/control identity 不一致、hard correctness/lifecycle失败、non-finite state、
   错误 config/manifest：停止当前候选矩阵，保存首个 typed expected/actual 分叉。
2. exact/hash 差异：记录为 exact-regression 或 repeatability 事实；它单独不触发
   rollback。只有同时出现 hard correctness failure 或已预注册 practical margin
   跨越时，才进入工程回退判断。
3. 单序列或单指标数值变化：完成既定矩阵并报告完整 distribution；不为追逐该结果
   调整 threshold、support、population、数据集或 evaluator。
4. runtime 条件失去可比性：停止 runtime aggregation，保留精度与结构结果的独立报告。
5. 停止后的修复作为新 candidate 与新矩阵执行；本矩阵保留为诊断证据，不自动代表
   算法回滚。

## 8. 执行前检查单

- [x] 冻结 `C_engineering`、recovery patch、两个canonical source manifests及
  3-run artifact roots；见[preflight §1](2026-09-02-note-m5-recovery-engineering-matrix-preflight.md)。
- [x] 记录远端EuRoC-11完整内容manifest、V1_02 prefix与TUM-VI calibration identity；
  TUM-VI明确为无GT adapter的structural sentinel；见preflight §2。
- [x] 固定remote-ci build/profile、host记录、profile间串行边界与隔离输出布局；
  见preflight §2–§3。
- [x] Paired timestamp/RPE-pair postprocessor已对11条self-test，且summary复现误差
  小于预注册检查值；见preflight §3。
- [x] Practical margin缺失已显式冻结为`engineering significance unresolved`；
  见preflight §4。
