# M4 Q3 O1E-ANCHOR-CONTRACT existing-record 审计

- 原审计日期：2026-08-17
- Fixed point：`31061bf0351647a346bff003063c80e004243c39`
- 状态：**historical / superseded / non-normative / qualification weight `0`**

> 2026-08-18 用户明确不存在任何外部需求记录，唯一目标是最终做出 VIO，并将工程实现选择授权
> 团队。因此旧 external locator/owner/requirement closure 路线已终止：不再 pending、不再是
> blocker、不再提问 locator。本文只保留既有审计事实，不持有 Q3 authority。

新 repo-owned authority 是 `PHAD-M4-Q3-GYRO-ALIGN-V1`，唯一可执行文本见
[Q3 design](m4-minimal-gyro-q3-offline-bias-alignment-design.md)。七份 docs 的 exact commit 已成为
`HEAD` 且 worktree clean 后、任何实现前，由独立 preflight 创建唯一 write-once protocol
identity receipt，绑定 docs commit/tree、七份 docs 各自的 path/git blob/SHA-256，以及
`design_commit/design_tree/design_blob/design_sha256`；Git objects 始终是权威。真实
qualification 后新增的 result ledger 只能原样引用该已存在的 receipt/identity，不得届时
首次声明、重新计算后选择或改绑；identity 不写入 design 自身。

## 2026-08-18 权限后记

同一后置用户指令——唯一目标是做出 VIO、实现路径交由团队决定——也构成 Issue #39 要求的
separate explicit Q3 implementation go。权限状态为 `implementation_go=satisfied`、
`docs_gate=pending`：只有完成 fresh docs review/返修、独立 verifier、七文件精确 docs commit 与
clean worktree 后，才可使用该既有 go 进入 Q3 hardening、RED、GREEN 与恰好一次 qualification；
docs gate 本身不自动产生授权。

该后置 go 仅为 Q3 supersede Q2 result ledger 当时的 plan/design-only 权限上限，不修改、否认或
追溯改写 Q2 technical verdict、waiver 与其他历史结论，也不授权 Q4 implementation、factor 或
posterior 修改。具体执行边界仍只由 Q3 normative design 持有；本文只记录 authority chronology。

## 原审计结论

**NOT FOUND（限原覆盖范围）。** 三条独立审计——可检索 repo 文档、本地 Git
history/refs/tags/notes、Issue #39 及其可定位的一方 issue provenance——均为
`QUALIFIED existing record=0`。原结论不曾证明外部全局 absence；2026-08-18 用户随后直接提供了
更强的项目事实“外部需求记录不存在”，因此无需继续 locator 路线。

原 qualified record 判定要求同时具备：exact `identifier/version/owner/baseline provenance`、
pre-outcome/arm-independent、明确属于 downstream 1 s orientation-quality requirement，且不是
论文、实现常数、candidate threshold、runtime result 或 textual decision label。

## 原覆盖与事实

- 检查当前 tree 中 docs/README/plans/roadmap/architecture，以及 `git log --all`、`-S/-G/-L`、
  refs、tags 与 notes；未发现 requirement tuple。
- 检查 Issue #39 body/comments/timeline、其链接的 #38/#36，以及 #30/#18；#39 comments 为 0，
  body 不含 tuple。
- Git author、issue association、commit SHA 或 runtime control provenance 都不能自动补成
  requirement owner/version/baseline。
- 仓库存在 1 s RPE measurement contract；它定义 measurement/pairing/unit，不提供产品 threshold。
- 历史 Q5 `0.001 rad` rotation guardrail 晚于已观测 control，且缺 identifier/version/owner/
  requirement provenance；不能回推为 outcome-independent Q3 budget。
- Q1 `est.tum` exact bytes 是 visual-posterior proxy 的 input identity，不是 GT、truth 或独立产品
  requirement。
- ADIS16448 datasheet 是 sensor specification，不是本项目 product orientation requirement。

## 候选 disposition

| 原候选 | 原审计结果 | 当前资格用途 |
|---|---|---|
| Q5 1 s rotation guardrail | 不满足 record tuple 与 outcome-independent 条件 | `0`；不得回推 Q3 门 |
| MH_01 control / Q1 result | runtime baseline，不是 requirement | `0` |
| M2.1 1 s RPE | metric contract，无 product threshold | `0` |
| Issue #38 Q3 block structure | planning structure，不是 requirement | `0` |
| O1E textual labels | decision-record IDs，不是 requirement IDs | superseded |
| Q1 visual-posterior proxy | input/reference provenance，不是 requirement | 只按 V1 design 用作冻结 proxy |
| Issue #30 bias/ATE hints | endpoint/量纲不匹配 | `0` |
| ADIS16448 official specs | sensor facts，不是 product gate | sanity-only |

## Supersession ledger

原历史保留为：closure path A、external-system location/class A 与 canonical-locator waiting 曾经是
项目决策序列。当前统一处置为 `SUPERSEDED_2026_08_18`：actual locator、tuple verification、owner
authorization、endpoint mapping 与 external adoption 都不再要求；不得把它们恢复为 active
question、blocker 或 qualification authority。

Q1/Q2 result ledger 的原结论不变。Q3 V1 的常数由团队在不观察 qualification 结果的前提下作
工程推导，并明确不冒充外部产品需求。
