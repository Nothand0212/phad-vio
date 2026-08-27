# M4 Q3 outcome-independent qualification budget 调研

日期：2026-08-18

> **历史 / superseded / non-normative。** 旧 external product-requirement、owner budget、locator、
> HAC/TOST/power 与 IUT decision DAG 的 qualification authority/weight 均为 `0`。本文只保留
> outcome-independent 推导与审计来历；[Q3 design](2026-08-13-note-m4-minimal-gyro-q3-offline-bias-alignment-design.md)
> 是唯一 executable authority。

## 1. Supersession ledger

2026-08-18 用户明确：不存在任何外部需求记录，唯一目标是最终做出 VIO，工程实现选择授权团队。
这项事实一次性取代旧路线，不改写当时的 commit/decision history：

| 历史记录 | 当前处置 |
|---|---|
| O1E-PATH / ANCHOR / BASIS / REFERENCE | historical decision record；无 active authority |
| O1E-ANCHOR-CONTRACT-PATH / LOCATOR / canonical locator | superseded；不再 pending，不再询问，不是 blocker |
| 外部 identifier/version/owner/baseline provenance | 不存在且不再要求；qualification weight `0` |
| O1E-MME / ADJ / POWER、O2/O3/O4、U/S/V/A DAG | 从 active Q3 V1 删除 |
| fixed-point alpha tuple、Holm/TOST/HAC candidates | historical、non-normative、weight `0` |

新 authority 是 repo-owned preregistration `PHAD-M4-Q3-GYRO-ALIGN-V1`。七份 docs 的 exact
commit 已成为 `HEAD` 且 worktree clean 后、任何实现前，由独立 preflight 创建唯一
write-once protocol identity receipt，绑定 docs commit/tree、七份 docs 各自的
path/git blob/SHA-256，以及 `design_commit/design_tree/design_blob/design_sha256`；Git objects
始终是权威。真实 qualification 后新增的 result ledger 只能原样引用该已存在的
receipt/identity，不得届时首次声明、重新计算后选择或改绑；因此无需将 identity
回填 design 自身，也避免自哈希。

## 2. Outcome-independent 边界来源

V1 的全部 verdict-changing 常数在任何真实 qualification 之前由团队冻结：

| 常数 | 工程推导 | 限定 |
|---|---|---|
| aggregate loss `<=0.8*zero` | 要求 squared rotation inconsistency 至少下降 20%，排除仅数值噪声级收益 | 不声称产品 MME 或显著性 |
| `3*n_improved>=2*n_blocks` | 至少 2/3 calendar blocks 不变差，避免少数大收益支配总和 | 等号通过；block 是唯一 unit |
| 每轴 loss 不增加 | 防止三轴 aggregate 掩盖单轴退化 | 不把 axes 当独立 samples |
| half bias max diff `<=1e-3 rad/s` | 1 s 积分约 `1e-3 rad` 的简单稳定性量级 | 不是 physical bias accuracy |
| SVD condition `<=1e6` | 限制线性 solve 的数值放大 | 不是 Eigen 或 sensor guarantee |
| full/half duration `>=90/45 s` | 各自至少覆盖目标 100/50 s window 的 90% | exact eligible duration |
| validation blocks `>=60` | 至少一分钟 non-overlap calendar evidence | 不作 power claim |

所有常数是 team-owned engineering boundaries。真实 run 后不得在同轮放宽、重解释或选择性删除。

## 3. 一手资料的合法用途

Eigen 3.4
[`SVDBase`](https://gitlab.com/libeigen/eigen/-/blob/3.4.0/Eigen/src/SVD/SVDBase.h#L142-205)
支持 `diagSize*epsilon*sigma_max` 的 rank threshold；对于三列 Jacobian 即
`3*epsilon*sigma_max`。它不提供 `1e6` condition 产品门，所以该常数明确归团队工程选择。

Analog Devices 的
[ADIS16448 Rev. H datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16448.pdf)
给出多种 typical/1-sigma sensor statistics；它们的测试条件和 estimand 与 visual-proxy alignment
不同。只能用于量纲 sanity，不可自动变成 Q3 hard gate。

Newey-West
[*A Simple, Positive Semi-definite, Heteroskedasticity and Autocorrelation Consistent Covariance Matrix*](https://doi.org/10.2307/1913610)
提供 HAC covariance；Schuirmann 1987
[*A Comparison of the Two One-Sided Tests Procedure and the Power Approach for Assessing the Equivalence of Average Bioavailability*](https://doi.org/10.1007/BF01068419)
提供 TOST；Berger & Hsu 1996
[*Bioequivalence Trials, Intersection-Union Tests and Equivalence Confidence Sets*](https://doi.org/10.1214/ss/1032280304)
说明 IUT 的 level 结构。这些来源支持相应统计工具的事实，不要求本项目使用它们。

## 4. 为什么 V1 删除 HAC、TOST 与 power

V1 的 claim 是固定 frozen input 上的确定性 qualification，不是总体均值显著性、equivalence、
跨序列推广或产品风险声明。non-overlap calendar block 是唯一 validation unit；decision 直接用
loss sum、block count 和 axis sum，不估 standard error。因此：

- HAC 会引入 bandwidth、gap covariance、normalization 与 asymptotic validity，但不会改变当前
  Q4 plan-only 决策需要的 effect/support contract；
- TOST 需要 externally meaningful equivalence margins 与 alpha，而 V1 不提出 equivalence claim；
- power 需要 probabilistic alternative/variance model，而 `60` 是透明的 deterministic support floor。

删除这些层级使协议更小且可执行，也避免把固定单序列 qualification 包装成过度的统计推断。
未来若目标改为跨序列总体推断，必须新建 protocol/version，不得事后修改 V1 的解释。

## 5. Leakage prohibition

冻结常数、schema 与 verdict 前后均禁止使用 Q3 suffix、future DUT、GT、ATE、RPE 或 desired PASS
反推门。Q1 四输入只承担预注册 qualification input；Q1/Q2 result 只提供 provenance 与 helper
资格。真实 qualification 固定 clean commit/tree 后只允许一次，任何结果都不得触发同轮调门。
