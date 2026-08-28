# M5 accumulated cold-root seed qualification spec

本文档描述当前约定，不是绝对约束，会随项目开发修订。

- 日期：2026-08-28
- 状态：**已定稿**
- issue：[#50](https://github.com/Nothand0212/phad-vio/issues/50)
  （decision map：[#42](https://github.com/Nothand0212/phad-vio/issues/42)）
- predecessor：[#48](https://github.com/Nothand0212/phad-vio/issues/48) /
  [PR #49](https://github.com/Nothand0212/phad-vio/pull/49)
- candidate base：`main@a007cc8748214daca6d2b84acef0e419e61bcbb2`
  / tree `5a3073e714144c5d4ba90ed76976d8c74316750a`
- frozen control：
  [current-path Observe checkpoint](../benchmark/m5/cold-root-current-path-observe_6a193a8_0337287b.md)
  与
  [evidence JSON](../benchmark/m5/cold-root-current-path-observe_6a193a8_0337287b.evidence.json)
- 前序合同：
  [M5 cold-root current-path Observe spec](2026-08-28-m5-cold-root-seed-observe.md)
- 历史风险证据：
  [pre-M4 accumulated-seed result](../benchmark/m3.3/prem4_round2_8906684_402d1925.md)

## 1. 目标与可证伪问题

本片只回答：

> 在当前 M4 full-state VIO 基线上，现有、默认关闭的 accumulated-seed 路径以
> global opt-in 方式运行完整 `V2_03_difficult` 时，是否有 accumulated cold-root
> population 跨过现有 population gate 并真实进入唯一 `seedRoot()` call-site；若进入，
> attempt 最远到达 geometry、current graph 或 commit 的哪一层，root commit 后是否恢复
> 真实 estimate/output？

本片取得的是 **capability qualification** 权限。结论可以证实或否证现有路径的
population、attempt 与 root-availability 层级，但不授予默认启用、门限调整、跨-root
world-frame alignment 或 formal gate 改判。

qualification 不预设成功。合法结果包括未达到 population、进入 attempt 后在下游停止、
成功提交 root，以及 evidence validity 不足导致的 inconclusive。

## 2. 冻结身份与唯一 treatment delta

| 项 | 冻结合同 |
|---|---|
| candidate base | `main@a007cc8`；若 `main` 变化，先停止并重新确认 identity |
| production source | `#48` candidate 之后只增加 benchmark/plan 文档；当前 production/test source 与已验证实现一致 |
| control | 直接复用 `#48` frozen V2_03 artifact，不重跑 |
| dataset | 与 `#48` 完全相同的 EuRoC `V2_03_difficult` |
| common runtime option | 现有 moving-bootstrap opt-in 保持与 control 相同 |
| 唯一 treatment delta | 额外开启现有 `--estimator-enable-accumulated-seed` |
| seed threshold | 保持 control 的实际 option 值；不通过 CLI 改写 |
| diagnostics | 复用 `#48` frozen typed taxonomy、presence invariant 与固定 64 列 schema |
| canonical treatment budget | 一次完整 sequence replay |

`enable_accumulated_seed` 是既有 CLI-only option，当前不进入 canonical config text/hash。
因此相同 `config_hash` 只证明其余配置相同，不能单独证明 treatment identity。candidate
必须同时绑定 exact command 与 diagnostics 中的实际 option snapshot。

merge commit `a007cc8` 与 PR head `59e813e` 的 tree 相同；`6a193a8..59e813e`
只包含 benchmark/plan 文档变化。由此，复用 `#48` control 不需要重新执行 Observe，
但新的 global treatment 必须从当前 candidate base clean build。

## 3. 当前基线为何需要重新资格验证

历史 M3.3 文档将该 option 描述为“首段专用”，当时 active re-anchor 与首次
uninitialized seed 是两条不同路径。当前 M4 lifecycle 在 active segment 完成时会清空
active state 并令 estimator 回到 uninitialized；现有 accumulated branch 的实际 guard 是
`!m_initialized`。因此同一既有路径现在也可能作用于后续 cold root。

本片以当前源码控制流和 `#48` typed diagnostics 为 runtime authority。历史结果继续约束
风险解释，但不替代当前 full-state VIO、visual-coast、segment-completion 与 current-graph
transaction 下的自然 replay。

这项差异只构成重新 qualification 的理由，不构成默认启用证据。

## 4. 术语与 verdict 层级

| 术语 | 本片含义 |
|---|---|
| treatment cold-root episode | treatment replay 中一个或多个连续 uninitialized cold-root rows，直到 root commit、其它 lifecycle 事件或 sequence 结束 |
| accumulated population pass | 实际 seed input origin 为 accumulated，population gate 为 passed，effective population 达到实际 threshold |
| real seed attempt | `attempt_id` 存在；按前序 frozen spec，它只在唯一 `seedRoot()` 调用紧邻调用前分配 |
| downstream stop | real attempt 在 root geometry 或 current graph 的现有 typed gate/reason 上停止 |
| root commit | accumulated-origin attempt 到达 `commit/committed`，transaction 提交，row 为 `kOk`，且对应 estimate 被写入输出 artifact |
| prefix divergence | treatment 与 control 在 control terminal epoch 之前的 legacy projection 或 trajectory output 首次出现差异 |
| system-level qualification | 评价 global option 从 sequence 起点生效后的完整真实 replay，不声称构造了与 control 完全相同的隐藏 estimator state |

最终 verdict 使用单一最强层级：

1. `inconclusive`；
2. `population_not_reached`；
3. `seed_attempt_reached`；
4. `root_committed`。

顺序 2–4 表示 capability 逐层增强；`inconclusive` 具有优先级，只要 evidence validity
未通过，就不发布 capability verdict。

## 5. Module authority、state flow 与 seam

`VioEstimator` 继续是该 capability 的 deep Module 与唯一 mutable authority：

```text
completeActiveSegment
  └─ initialized=false，清空上一 active segment state

VioEstimator::update                         唯一 public measurement seam
  ├─ existing bootstrap/keyframe gates
  ├─ current-packet population
  ├─ existing pending_seed_obs               LandmarkId 去重、latest wins
  ├─ existing population gate
  └─ VioUpdateTransaction
       ├─ attempt_id → seedRoot              唯一真实 attempt call-site
       ├─ current graph build/solve/validation
       └─ commit / rollback

UpdateDiagnostics::m_cold_root
  └─ OfflineVoSession 逐字段复制 → diag.csv
```

本片的 design boundary：

1. `VioEstimator::update()` 保持唯一 public measurement interface；
2. pending population、effective input、gate trace、attempt identity 与 transaction outcome
   继续由 estimator 生成；
3. session/CSV 不从 message、status 或其它 diagnostics 反推 cold-root 语义；
4. treatment 通过现有 composition-root flag 打开，不增加 timed activation、callback、
   second policy、shadow calculation 或 production injection seam；
5. 当前 `ColdRootObserveDiagnostics`、CSV schema、default 与 threshold 均保持冻结合同；
6. 预计 production、test 与 app source diff 为空。

若实际执行前发现现有 observability 无法独立判断 population、attempt 或下游 gate，本片
立即停在 design gate，另行形成 TDD 修订计划，不用不完整 artifact 代替证据。

## 6. Candidate 与 artifact identity

canonical treatment 必须记录：

| Identity | 要求 |
|---|---|
| issue / branch | `#50` 与本片独立 branch |
| candidate commit / tree / parent | full SHA；canonical replay candidate 只能含已审查的本 spec/plan，evidence 在 replay 后另行形成 |
| worktree | canonical run 开始时 tracked clean |
| source delta | candidate 相对 `a007cc8` 的 production/app/test/build source diff 为空 |
| build | Release；configure/build 命令与成功状态记录 |
| dataset | dataset root、sequence name 与可用的 manifest identity |
| command | 完整 argv，包括两个现有 estimator flags 与 output root |
| config | canonical text、label 与 hash；说明 accumulated flag 不在 hash 中 |
| runtime proof | 每个可归因 treatment cold-root row 的 accumulated option snapshot 为 true |
| artifacts | `diag.csv`、`est.tum`、`kf.tum`、`meta.json`、`summary.json` 的路径与 SHA-256 |
| run count | canonical V2_03 treatment invocation 数量 |

candidate identity 不允许用 dirty tree、未记录 CLI 或聊天摘要补足。

## 7. Canonical experiment contract

### 7.1 Preflight

canonical replay 前依次完成：

1. 核对 branch 从 `a007cc8` 建立，`main` 未漂移；
2. 核对本片 tracked diff 仅为已确认 spec/plan；
3. 建立 Release build，编译 `phad_vo_bench` 与相关 tests；
4. 运行现有 accumulated-seed lifecycle、typed diagnostics、rollback 与 session/CSV
   定向测试，再扩大到 estimator/apps suites；
5. 核对 `git diff --check`、64 列 header 与 `#48` evidence 可读；
6. 汇报 exact command、一次 replay 成本、output root 与停止条件，取得单独运行授权。

本片没有 production/test 行为变更，因此不制造新的 RED。任何既有测试失败都先诊断；
不得以修改测试、threshold 或 option 规避 preflight。

### 7.2 One canonical treatment

唯一正式数据实验从 sequence 起点运行完整 V2_03：

- 使用与 `#48` 相同的 dataset、sync、frontend、estimator、keyframe、evaluation 与
  moving-bootstrap 配置；
- 额外开启现有 accumulated-seed flag；
- 使用新的、唯一 output directory；
- 不用 `--max-frames`、tail dataset 或中途 option toggle；
- 不重跑 control，不在本片运行其它 EuRoC sequence；
- canonical invocation 结束后，无论 capability verdict 为何都停止数据实验。

若 invocation、artifact 或 provenance 无效，本次结果为 `inconclusive`。保留首个失败
证据，不自动修正后重跑。

## 8. Evidence validity gate

machine-readable evidence 必须逐项给出 pass/fail 与原始来源：

| Gate | 通过条件 |
|---|---|
| E0 candidate identity | commit/tree/parent、clean 状态、source delta 与 issue/branch 可对账 |
| E1 treatment identity | exact command 含现有 accumulated flag；diagnostics option snapshot 证明 runtime 实际开启 |
| E2 control comparability | dataset 与除 accumulated flag 外的 canonical config/runtime option 与 `#48` control 一致 |
| E3 artifact integrity | 必需 artifacts 存在、可解析、hash 已记录；diag row cadence 与输入 replay 完整对应 |
| E4 typed invariant | 每行 phase/reason/gate/presence/origin/attempt 组合满足 frozen `#48` invariant |
| E5 attempt reconciliation | 每个 non-empty attempt ID 只对应合法 attempt phase；ID 严格单调；population/origin/threshold 与 attempt 一致 |
| E6 output reconciliation | 若存在 commit，commit row、`kOk` status、estimate timestamp 与 `est.tum`/session output 能交叉核对；没有 commit 时明确为 `not_applicable` |

任一适用 gate 未通过，最终 verdict 固定为 `inconclusive`。conditional gate 可以标记
`not_applicable`，但必须记录原因。trajectory 指标、运行时间或“看起来恢复”不能覆盖
identity、typed invariant 或 output reconciliation 失败。

## 9. Capability verdict 判定

### 9.1 `population_not_reached`

在 E0–E6 可适用部分均有效的前提下：

1. 没有 accumulated-origin population pass；
2. 没有 accumulated-origin attempt ID。

该 verdict 进一步记录一个 subreason：

- `branch_not_exercised`：global treatment 的实际 lifecycle 没有产生
  accumulated-origin cold-root row；
- `accumulating_below_threshold`：存在 accumulated-origin rows，但都停在
  `stereo_population/population_accumulating`。

存在 accumulated-origin rows 时，同时报告 pending/effective population 的最大值、时间
分布与 sequence 结束时状态，不能只报一个 summary boolean。

### 9.2 `seed_attempt_reached`

至少一个 event 同时满足：

1. seed input origin 为 accumulated；
2. pending unique 与 effective population 存在；
3. effective population 达到实际 threshold；
4. stereo population gate 为 passed；
5. `attempt_id` 存在。

若没有 accumulated-origin root commit，最终 verdict 为 `seed_attempt_reached`。evidence
按 attempt ID 输出 first/farthest phase、primary reason、geometry result、current graph 与
commit gate；首个下游失败必须保留原 token，不归并成笼统的 seed failure。

### 9.3 `root_committed`

至少一个 accumulated-origin attempt 同时满足：

1. `commit/committed`；
2. geometry、current graph 与 commit gate 均为 passed；
3. update status 为 `ok`；
4. 对应 timestamp 存在真实 estimate/session output；
5. transaction 后续状态没有违反 frozen lifecycle invariant。

root commit 后继续记录 output 持续区间、后续 cold-root episode 与 sequence tail，但这些记录
不升级为 world-frame alignment 结论。

## 10. Global treatment 的因果与输出边界

现有 flag 对所有 uninitialized cold roots 生效。treatment 可能在 control terminal epoch
之前改变首次 root 时间、segment 生命周期、map state 与后续输入消费历史。本片因此使用：

1. **全程 capability 账本**：列出 treatment 自身的全部 cold-root episodes、population
   passes、attempts、commits 与 stops；
2. **first-divergence 账本**：以 control 的旧列 projection、status/segment cadence、
   `est.tum` 与 `kf.tum` 找到最早 artifact-level divergence；
3. **control-terminal observation**：在 `#48` frozen terminal timestamp interval 内，记录
   treatment 的 status、attempt、commit 与 estimate/output availability；
4. **claim qualifier**：若 prefix 已分叉，明确写为 system-level qualification；若 artifact
   prefix 相同，也只声明 artifact-level equivalence，不声称隐藏 estimator state 相同。

ATE/RPE、completion/coverage、local/intersegment components 与 timing 只作 record。无论
这些数字改善或退化，`#47` formal gate 与 root availability 仍是两个独立结论。

## 11. Transaction 与 failure 真实性

本片不改变现有 pending/transaction 语义。evidence 必须区分：

- population accumulating 的已提交 pending state；
- `seedRoot()` geometry rejection 后的 rollback；
- current graph build/solve/validation failure 后的 rollback；
- commit 后清空或保留的实际 pending state；
- attempt counter 不随 rollback 回收的现有语义。

正常返回、exception、invalid input 与 artifact 缺失保持原 status/message/error contract。
不得把 exception 吞掉后写成 capability verdict，也不得用缺失 optional 当数值零。

## 12. 停止条件

以下任一事实出现即停止并请求重新确认：

1. `main` 不再是 frozen candidate base；
2. canonical delta 除 existing accumulated flag 外还需要 threshold、default、frontend、
   bootstrap、keyframe、solver 或 dataset 变化；
3. 需要新增 production/test seam、CLI、schema、diagnostics 或 policy；
4. current artifact 无法证明 runtime option、population、attempt 或 commit；
5. preflight build/tests 或 frozen typed invariant 失败；
6. canonical treatment invocation 或必需 artifact 无效；
7. 需要第二次 V2_03、其它 sequence 或 tail replay 才能解释结果；
8. spec、Issue #50、源码或历史证据出现实质冲突。

停止后保留现场与首个差异，先形成修订计划。生产实现需要 TDD 与 cpp-pro 的独立授权。

## 13. 验收与交付

本片完成需同时具备：

1. 本 frozen spec 与 confirmed plan；
2. clean、可复现的 canonical candidate identity；
3. 现有 build/tests 与 Standards/Spec review 通过；
4. 一次且仅一次 canonical V2_03 treatment 的完整 artifacts；
5. E0–E6 的 `pass` / conditional `not_applicable` 机器对账与单一 capability verdict；
6. benchmark Markdown checkpoint 与 machine-readable evidence JSON；
7. 历史 pre-M4 结论、当前 system-level result 与 `#47` alignment boundary 分开叙述；
8. 经单独授权后的 Issue #50 / #42 回填。

spec/plan 的 local commit、canonical replay、evidence commit、push、PR、merge 与 Issue 状态
变更分别受用户授权约束；本 spec 本身不授予这些操作。
