# M4 Q3 O1E-ANCHOR-CONTRACT existing-record 审计

- 日期：2026-08-17
- Fixed point：`31061bf0351647a346bff003063c80e004243c39`
- 跟踪：[Issue #39](https://github.com/Nothand0212/phad-vio/issues/39)

## 结论

**NOT FOUND（限本次覆盖范围）。** 三条独立审计——可检索 repo 文档、全部本地 Git
history/refs/tags/notes、Issue #39 及其可定位的一方 issue provenance——均为
`QUALIFIED existing record=0`。Issue #39 没有 tuple，comments 为 0；在这些范围内未发现一份
既有 normative record 能同时满足以下全部条件：

1. exact `identifier`、`version`、`owner`、`baseline provenance`；
2. outcome-independent / pre-outcome；arm-independent 另有独立证据；
3. 明确属于 O1E-ANCHOR=A 已选中的 downstream **1 s relative-orientation consistency /
   product orientation-quality requirement**；
4. 不是论文/成熟实现经验、代码注释、候选 threshold、Q3 outcome 或 stable textual decision label。

该 `NOT FOUND` 不是“任何外部 record 都不存在”的全局 absence proof；未接入本次审计范围的
受控 product-requirement system 仍是证据缺口。

用户在上一提交后的下一问明确选择 budget §7.5 的 **closure path A**。稳定 textual decision
label 为 `O1E-ANCHOR-CONTRACT-PATH=A`（user/owner selected，非 schema field）；它只冻结
“采用 existing qualified record 路径”，**没有提供、暗示或批准实际 tuple**。用户随后对 stable
decision `O1E-ANCHOR-CONTRACT-LOCATOR` 选择 A；必须原样记录为
`O1E-ANCHOR-CONTRACT-LOCATOR=A selected`，不得事后改名为 `AUTHORITY=A`。它只选择
canonical authority 的 location/class 为非 repo/GitHub Issue 的受控 product-requirement
system，不是 actual locator payload。严格下游 textual node
`O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR` 当前 pending。因此当前状态为
`PATH_A_SELECTED / LOCATOR_A_SELECTED(EXTERNAL_AUTHORITY) / AWAITING_CANONICAL_LOCATOR`：

- `O1E-ANCHOR-CONTRACT` 仍为 pending / `UNFROZEN`，本审计不把任何候选升级为合同；
- actual canonical locator、identifier、version、owner、baseline provenance、category locator、
  authorization 与 adoption 均未提供、未验证，不能从 external system class 推定；
- `O1E-ENDPOINT` waiting，Q3 implementation 保持 **STOP**，Q3 plan 5/5 todos 保持 pending。

## 审计边界与判定方法

### 事实

- fixed point 的 budget 已明确把 record contract 定义为 exact
  `identifier/version/owner/baseline provenance`，并明确写出实际 tuple 尚未提供、不得臆造：
  [budget §7.5「O1E-ANCHOR-CONTRACT record contract」](m4-minimal-gyro-q3-outcome-independent-budget-research.md#75-o1e-anchor-contractselected-requirement-的-record-contract)。
- Q3 normative design 同样记录 actual tuple 未提供和验证，且要求
  `O1E-ENDPOINT` 等待：
  [Q3 design §6.1「O1E-ANCHOR-CONTRACT tuple pending」](m4-minimal-gyro-q3-offline-bias-alignment-design.md#61-primary-diagnostic-componenthac)、
  [Q3 design §6.1「O1E-ENDPOINT waiting」](m4-minimal-gyro-q3-offline-bias-alignment-design.md#61-primary-diagnostic-componenthac)。
- budget 的防泄漏边界禁止从 Q3 suffix outcome、GT/ATE/RPE、Q5、future DUT 或 desired PASS
  反推 threshold；既有 Q1 runtime artifacts 只能承担 identity/schema/provenance 角色：
  [budget §1「leakage prohibition」](m4-minimal-gyro-q3-outcome-independent-budget-research.md#1-fixed-point权限与-leakage-prohibition)。

### 本轮覆盖

- 当前 tree 的 `docs/`、`README`/module contract、计划、roadmap、architecture 与 issue-tracker
  约定；只读取仓库内已入 Git 的记录，未读取外部 Q3 outcome artifacts。
- `git log --all`、`git log -S/-G/-L`、全部 refs、tag 与 Git notes；同时检查历史上可能出现的
  requirement/spec/PRD/hazard/mission/rotation/orientation 记录路径。
- GitHub Issue #39 的完整 body、comments、timeline、sub-issue/dependency/closed-PR 状态；其 body
  直接链接的 [Issue #38](https://github.com/Nothand0212/phad-vio/issues/38) 及 #38 唯一 comment，
  并沿 parent provenance 核对 [Issue #36](https://github.com/Nothand0212/phad-vio/issues/36)。
- 仓库其他相关 first-party issue，包括
  [#30 M4 总图](https://github.com/Nothand0212/phad-vio/issues/30)、
  [#18 M2.1 RPE](https://github.com/Nothand0212/phad-vio/issues/18)。

以上三类是本审计的可检索边界；结论仅为该 repo/Git/Issue scope 内 `NOT FOUND`，不外推到未提供
locator、因而无法检索的外部 requirement systems 或 records。

### 判定原则

- Git commit SHA 可以定位某段文本的 source revision，但在 record 没有如此声明时，不能替代
  requirement `version`。
- commit/issue author 或 repository owner 不能被推定为 requirement `owner/approver`。
- runtime control 的 source commit、config、artifact hash 可以形成运行基线 provenance，但不能
  替代 **requirement baseline provenance**。
- 描述性标题、plan ID、milestone label 与本轮 decision-record alias 不能补造成 exact
  requirement identifier。

## 候选逐项审计

| 候选 | Primary source（URL / issue / commit / file-line） | identifier | version | owner | baseline provenance | outcome-independent / arm-independent | 所选 product category | 处置 |
|---|---|---|---|---|---|---|---|---|
| Q5 rotation guardrail：`1 s rotation RPE RMSE <= control + 0.0572958 deg`（`0.001 rad`） | 当前文本：[M4 slice design §10.2「Q5 rotation guardrail」](m4-minimal-gyro-slice-design.md#102-数据实验门)，fixed point `31061bf`；首次引入：commit `8506378639a6ef8709b615cab9855cbdd0d54f88`，Issue [#36](https://github.com/Nothand0212/phad-vio/issues/36) | **否**：`Q5 rotation guardrail` 只是 milestone-local 行标签，不是可审计 requirement record ID | **否**：无 requirement version；commit SHA 只是文档 revision | **否**：无 owner/approver 字段；不得从 Git author 推定 | **否**：有 runtime control 的部分 provenance，但没有 requirement baseline provenance | **outcome-independent：否**。现有证据只证明 guardrail 晚于 control record，且表达式依赖已观测 control outcome。**arm-independent：未确认**；现有证据不足以作否定结论 | **是（表面语义）**：这是唯一明确写成 1 s product rotation gate 的候选 | **排除**。outcome-independent 不满足；arm-independent 不作否定结论。无论该属性后续如何确认，identifier/version/owner/baseline/category/adoption tuple 均不完整，仍不 qualified；不得用 control/Q1/Q5 runtime outcome 补造 requirement。Q3 design 也明确禁止从该 `0.001 rad` guardrail 跨维度推导 Q3 budgets：[Q3 design §6.5「Outcome 外 owner budgets」](m4-minimal-gyro-q3-offline-bias-alignment-design.md#65-outcome-外-owner-budgets)。 |
| MH_01 clean control / Q1 final visual result | [MH_01 control「运行身份与输入」](m4-minimal-gyro-mh01-control.md#运行身份与输入)、[MH_01 control「指标与行为快照」](m4-minimal-gyro-mh01-control.md#指标与行为快照)，commit `a8e892fcccbe061c5b69d7dabb150026ff19b341`；[Q1 result §4「standalone 输出与 control byte gate」](m4-minimal-gyro-q1-observe-result.md#4-standalone-输出与-control-byte-gate) | 否 | 否（不是 requirement version） | 否 | **仅 runtime baseline 完整**，不是 requirement baseline | **否**：本身就是已观测 runtime outcome | **否**：performance snapshot / byte baseline，不是产品需求 | **排除**。不得从 `RPE(1 s) rotation RMSE=0.151519... deg` 或任何 ATE/RPE/result 反推产品 requirement。 |
| M2.1 fixed-interval RPE metric（默认 1 s） | [roadmap「M2.1 评估与可视化底座」](../roadmap.md#m21-评估与可视化底座已完成)；[M2.1 plan「已对齐的决策」](../plans/2026-07-30_m2.1_eval_visualization_baseline_d818d653.plan.md#已对齐的决策)、[M2.1 plan「测试」](../plans/2026-07-30_m2.1_eval_visualization_baseline_d818d653.plan.md#测试)；Issue [#18](https://github.com/Nothand0212/phad-vio/issues/18) | 否（plan ID 是实施计划 identity，不是产品 requirement ID） | 否 | 否 | 否 | **是**：早于 M4/Q3 outcomes，算法定义也不依赖 Q3 arms | **否**：只定义 measurement/pairing/unit/tool contract，没有 orientation-quality threshold 或产品风险语义 | **排除**。它证明仓库会计算 1 s rotation RPE，不证明存在 1 s product orientation-quality requirement。 |
| Issue #38 的 adjacent / non-overlap 1 s Q3 test structure | [Issue #38 body](https://github.com/Nothand0212/phad-vio/issues/38) 与[唯一 owner comment](https://github.com/Nothand0212/phad-vio/issues/38#issuecomment-5272321233)，structural fixed point `dbec0be89512f54c386319762838fe16296e7b1f` | 否 | 否 | 否（comment author 不等于 requirement owner） | 否 | **是**：planning record 明确禁止 Q3/Q5 outcome 与实现 | **否**：这是 statistical/protocol structure；comment 还明确 owner margins/effect-size/power budgets 未冻结 | **排除**。protocol test horizon 不能替代 downstream product requirement record。 |
| `O1E-ANCHOR=A` / `O1E-ANCHOR-CONTRACT` | [budget §7.4「O1E-ANCHOR decision record」](m4-minimal-gyro-q3-outcome-independent-budget-research.md#74-o1e-anchorprimary-positive-minimum-meaningful-effect-endpoint-anchor)，commits `0d96fd7a481f60a4e8eea203b609fbd1e31c8c28`、`31061bf0351647a346bff003063c80e004243c39` | **否**：它们是 stable textual decision-record IDs，不是 actual requirement identifier | 否 | 否 | 否 | **是（决策过程）**，但不是 pre-existing requirement record | **是（category only）** | **排除**。原文明确 actual tuple 未提供；禁止把 stable textual decision label 升级成 actual requirement contract。`O1E-ANCHOR-CONTRACT-PATH=A` 只改变 closure path；`O1E-ANCHOR-CONTRACT-LOCATOR=A selected` 只选择 external authority location/class，不是 actual canonical locator。 |
| `q1-final-visual-posterior-proxy-v1` / O1E-REFERENCE=A | [budget §1「O1E-REFERENCE identity / role boundary」](m4-minimal-gyro-q3-outcome-independent-budget-research.md#1-fixed-point权限与-leakage-prohibition)；[Q3 design §6.1「O1E-REFERENCE contract」](m4-minimal-gyro-q3-offline-bias-alignment-design.md#61-primary-diagnostic-componenthac)，commit `31061bf0351647a346bff003063c80e004243c39` | **有 reference alias/bytes identity，但不是 requirement identifier** | reference source revision/digests 有 | requirement owner 无 | **reference provenance 完整，但不是 requirement baseline provenance** | **是**：两臂共享、arm-independent、pre-outcome | **否**：原文明确它不是 product endpoint metric 或 independent product-qualification oracle | **排除**。reference contract 与 requirement contract 必须分离，不得因 provenance 完整而合并。 |
| Issue #30 M4 三重门中的 gyro bias `< 1e-3 rad/s` 量级与 ATE gate | [Issue #30 body](https://github.com/Nothand0212/phad-vio/issues/30) | 否 | 否 | 否 | M3 runtime 基线仅部分有 | 是（早于当前 Q3） | **否**：bias mechanism / translation gate，不是 1 s relative-orientation product quality | **排除**。量纲、endpoint 与 requirement category 均不匹配。 |
| ADIS16448 官方器件规格 | 来源边界与排除规则：[budget §1「leakage prohibition」](m4-minimal-gyro-q3-outcome-independent-budget-research.md#1-fixed-point权限与-leakage-prohibition) | 器件文档有自己的文档 identity | 器件 revision 有 | 厂商有 | 器件规格 provenance 有 | 通常 pre-outcome | **否**：sensor characteristic，不是本项目产品 orientation-quality requirement | **排除**。官方规格最多提供量纲、候选模型或 sanity floor，不能自动成为产品门。研究论文与成熟实现经验按资格条件直接排除，未被用来补齐合同。 |

## Git 与 GitHub 账本事实

- `git log --all -S'0.0572958'` 只命中 commit
  `8506378639a6ef8709b615cab9855cbdd0d54f88`；`git log -L` 表明该行从此首次引入且到 fixed
  point 未被另一份 versioned requirement record 接管。
- runtime control record 先由 `a8e892fcccbe061c5b69d7dabb150026ff19b341` 于
  2026-08-12 09:50:50 +08:00 固定，guardrail 后由 `8506378...` 于 12:58:07 +08:00 写入。
  这是时间顺序事实；它不能证明 `0.001 rad` 的风险来源或 owner approval，反而不能满足严格
  pre-outcome requirement-record 资格。
- 全部 tag 中只有一个 `m3.3-pnp-arbitration`，与 M4/Q3/product orientation requirement
  无关；无 Git notes。GitHub 无 release，远端同样只有该 M3.3 tag。
- 未发现 `CODEOWNERS`、`OWNERS`、PRD/spec/requirements/hazard/mission record 文件能补齐 tuple。
- Issue #39 是 issue，不是 PR；其 body 只直接链接 #38，comments 为 0、
  `closedByPullRequestsReferences` 为空、sub-issue/dependency 计数均为 0。timeline 只有 label 事件。
  #38 body 上溯 #36；#38 唯一 comment 明确 protocol amendment 仍 STOP、owner budgets 仍待冻结。

## 事实、推断与证据缺口

### 已证实事实

1. 仓库确有 1 s RPE measurement contract，也确有一个 Q5 rotation guardrail 文本。
2. Q5 guardrail 的 source revision、写入时间和其所引用 runtime control 的 provenance 可定位。
3. fixed point 的 budget/design 多处明确声明 requirement tuple 尚未提供和验证。

### 受证据约束的推断

- `Q5 rotation guardrail` 在语义上最接近所选 product category，但它不是合格 existing
  requirement record：outcome-independent 不满足；arm-independent 依据不足，不作否定结论；
  且 identifier/version/owner/baseline/category/adoption tuple 不完整。
- Git author、issue OWNER association、document commit 和 control artifact digest 即使可定位，也只能
  分别证明谁写了文本、文本 revision 与 runtime identity，不能合成为一份未经 owner 明确批准的
  requirement tuple。

### 仍缺的证据

- external authority 中的 actual immutable canonical permalink；若系统不支持 permalink，则缺其
  明确 instance/name 与 exact record ID；
- 真实 requirement system/ledger 中的 exact record identity 与 immutable revision；
- 被授权的 requirement owner/approver identity；
- requirement baseline 的权威位置、版本/签名/hash 与 approval trace；
- 证明该 exact record 在相关 outcome/arms 之前独立存在、且其可定位条款明确属于所选 1 s
  product orientation-quality category 的证据。

## Path A / locator A 之后：canonical locator

原 locator location/class A/B/C 已成为历史决策记录：

- **A（selected）**：canonical authority 位于非 repo/GitHub Issue 的受控
  product-requirement system。
- **B（not selected for this decision）**：canonical authority 本身位于 repo 或 GitHub Issue。
- **C（not selected for this decision）**：确认没有 qualified existing record 并切换到新 baseline
  路线。

B/C 只是在本次 decision 中未选择，不永久否决未来路径变更。stable label 必须保留为
`O1E-ANCHOR-CONTRACT-LOCATOR=A selected`；A 只说明 external authority location/class，不能
推出 permalink、record ID、system instance/name 或任何 tuple 字段。

下一项唯一 owner decision 是严格下游 textual node
`O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR`；无需且不得在本节点提前提供 endpoint semantics、
threshold、metric、mapping 或任何 Q3 numeric gate：

- **A（推荐，pending）**：现在提供 immutable canonical permalink；若知道 exact record ID，可
  一并提供。
- **B（pending）**：仅当该受控系统没有 immutable permalink 时，提供明确 system instance/name
  与 exact record ID。
- **C（pending）**：当前两者都不能提供，继续 pending / STOP。C 不等于 record 不存在，不自动
  撤回 path A/locator A，也不授权新 baseline。

三项按当前可提供性互斥。actual canonical locator 未取得前不得进入 `TUPLE_VERIFICATION`；取得后
才可按权威记录验证 actual identifier/version/owner/baseline provenance、outcome-independent /
arm-independent 属性与 category 归属。只有 verification 通过后才允许 owner authorization，
authorization 不得先行或与 verification 合并。只有随后合同 closed，O1E-ENDPOINT 才可继续；
唯一 canonical 有向顺序见
[budget §6](m4-minimal-gyro-q3-outcome-independent-budget-research.md#6-无循环-decision-dag)。在此之前维持：
**`PATH_A_SELECTED / LOCATOR_A_SELECTED(EXTERNAL_AUTHORITY) / AWAITING_CANONICAL_LOCATOR`；
O1E-ANCHOR-CONTRACT pending / `UNFROZEN`；actual canonical locator、tuple、category locator、
authorization 与 adoption 未提供、未验证；O1E-ENDPOINT waiting；Q3 implementation STOP；Q3
plan 5/5 todos pending**。没有 actual canonical locator 时，不得进入 tuple verification 或询问
O1E-ENDPOINT。
