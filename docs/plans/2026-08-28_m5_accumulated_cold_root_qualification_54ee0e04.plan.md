---
name: M5 accumulated cold-root qualification
overview: 在 main@a007cc8 上复用现有 default-off accumulated-seed 路径与 #48 typed diagnostics，以一次完整 V2_03 global treatment 形成 population、真实 seed attempt 与 root commit 的分层资格证据；本计划预计不修改 production/test source。
todos:
  - id: align-qualification-contract
    content: 确认 capability-only scope、分层 verdict 与 full-sequence system-level experiment boundary
    status: completed
  - id: establish-issue-worktree
    content: 创建 Issue #50，并从 main@a007cc8 建立独立 branch/worktree
    status: completed
  - id: freeze-spec-plan
    content: 落盘 frozen qualification spec 与本执行/evidence plan，核对 authority、identity、run budget 与停止条件
    status: completed
  - id: authorize-docs-candidate
    content: 文档审查通过后单独取得 spec/plan local commit 授权，形成 clean canonical candidate
    status: completed
  - id: preflight-existing-path
    content: 从 clean candidate 建立 Release build，运行现有 accumulated lifecycle、typed diagnostics、rollback、session schema 定向测试及 estimator/apps suites
    status: pending
  - id: authorize-v2-03-treatment
    content: 汇报 exact command、成本、output root 与停止条件，单独取得一次 V2_03 global treatment 授权
    status: pending
  - id: run-v2-03-once
    content: 复用 #48 control，仅运行一次完整 V2_03 candidate，唯一 runtime delta 为现有 accumulated-seed flag
    status: pending
  - id: reconcile-capability-evidence
    content: 对账 treatment identity、typed rows、first divergence、population/attempt/commit、control-terminal output 与 machine-readable verdict
    status: pending
  - id: review-and-deliver
    content: 完成 Standards/Spec 与 evidence red-team，形成 checkpoint/evidence，并逐项请求 commit、push、PR 与 Issue 回填授权
    status: pending
isProject: false
---

# M5 accumulated cold-root qualification

## 0. 状态与 authority

本计划的 design、verdict 与 experiment boundary 已于 2026-08-28 逐项确认；Issue、独立
worktree、frozen spec/plan 与 local docs candidate commit 已取得授权。build/test、canonical
replay 与其它远端写入尚未执行。

| 对齐段 | 已确认结论 |
|---|---|
| scope | A：qualification-only；复用现有路径，default 与 threshold 保持当前合同 |
| verdict | A：`population_not_reached` → `seed_attempt_reached` → `root_committed` 分层；validity 不足为 `inconclusive` |
| experiment | A：一次完整 V2_03 global treatment；若 prefix 分叉，结论限定为 system-level qualification |
| workflow | Issue #50 已建立；branch/worktree 与 spec/plan 起草已获授权 |

按具体性从高到低使用以下 authority：

1. [frozen qualification spec](../specs/2026-08-28-m5-accumulated-cold-root-qualification.md)；
2. [Issue #50](https://github.com/Nothand0212/phad-vio/issues/50) 当前正文；
3. [M5 cold-root Observe spec](../specs/2026-08-28-m5-cold-root-seed-observe.md)
   与 [checkpoint](../benchmark/m5/cold-root-current-path-observe_6a193a8_0337287b.md)；
4. [`evidence-gated-integration.md`](../agents/evidence-gated-integration.md)、
   [`incremental-development.md`](../agents/incremental-development.md) 与 Git/文档约定；
5. estimator、apps、tests 当前源码与既有 tests。

| 项 | 当前事实 |
|---|---|
| worktree | `/home/lin/Projects/lin_ws/slam_ws/phad-vio/.worktree/codex-m5-cold-root-accumulated-qualify` |
| branch | `codex/m5-cold-root-accumulated-qualify`；尚无 upstream |
| base HEAD | `a007cc8748214daca6d2b84acef0e419e61bcbb2` |
| base tree | `5a3073e714144c5d4ba90ed76976d8c74316750a` |
| Issue | `#50` Open / `wayfinder:task` |
| control | `#48` frozen V2_03 artifact 与 machine-readable evidence；不重跑 |
| intended tracked diff | 本 spec 与本 plan；production/app/test/build source 为空 |

## 1. 本片问题、产物与边界

唯一问题是：

> current-baseline global treatment 中，现有 accumulated population 是否真实跨过
> population gate 并触发 `seedRoot()`；若触发，attempt 最远到达哪里，commit 是否恢复
> estimate/output？

计划产物：

1. clean candidate 与完整 preflight 记录；
2. 一次 canonical V2_03 treatment artifact；
3. 分层 capability verdict 与 first-divergence/system-level claim qualifier；
4. benchmark checkpoint 和 machine-readable evidence。

本片使用现有 option、state、transaction 与 diagnostics。预计没有 production、test、app
或 CMake 修改；因此执行阶段不是功能实现，不人为创建 RED。若 observability 或现有行为
必须修改，立即退出本计划的 evidence-only 路径，回到 TDD/cpp-pro 修订计划确认。

## 2. graphify 与源码核对的 authority flow

既有 graphify DFS 与当前源码交叉核对得到：

```text
runOfflineVoSession
  └─ VioEstimator::update
       ├─ bootstrap / keyframe
       ├─ current population
       ├─ pending_seed_obs             existing accumulated state
       ├─ population gate
       └─ VioUpdateTransaction
            ├─ attempt_id → seedRoot
            ├─ buildGraph / solveGraph / validation
            └─ commit / rollback

UpdateDiagnostics::m_cold_root
  └─ VoDiagRow → writeDiagCsv → fixed 64-column diag.csv
```

关键源码事实：

1. `completeActiveSegment()` 令 state 回到 uninitialized，并清空上一 segment 的 pending；
2. accumulated branch 的 runtime guard 是 `!m_initialized` 与 existing option；
3. pending 使用 `LandmarkId` 去重且 latest observation wins；
4. `attempt_id` 紧邻唯一 `seedRoot()` call-site 分配；
5. pending staging、root graph 与 output mutation 受现有 transaction 管理；
6. `#48` diagnostics 已区分 current/pending/effective population、origin、attempt 与下游 gate。

由此，本片没有增加 second policy 或观测 seam 的设计理由。

## 3. Candidate 形成与权限 gate

spec/plan 审查通过后已向用户汇报：

- 两个新增文档的路径与 diff；
- production/app/test/build source diff 为空；
- frozen base、Issue 与 worktree identity；
- 后续 preflight 与 canonical replay 尚未运行。

用户已单独授权 local docs candidate commit，commit subject：

```text
docs(m5): qualify accumulated cold-root seed (#50)
```

该授权不包含 build/test、push、PR、Issue comment、数据 replay 或 evidence commit。

commit 后必须核对：

```bash
git status --short --branch
git merge-base HEAD a007cc8748214daca6d2b84acef0e419e61bcbb2
git diff --check a007cc8748214daca6d2b84acef0e419e61bcbb2...HEAD
git diff --name-status a007cc8748214daca6d2b84acef0e419e61bcbb2...HEAD
git diff --quiet a007cc8748214daca6d2b84acef0e419e61bcbb2...HEAD -- \
  ':!docs/specs/2026-08-28-m5-accumulated-cold-root-qualification.md' \
  ':!docs/plans/2026-08-28_m5_accumulated_cold_root_qualification_54ee0e04.plan.md'
```

## 4. Preflight：clean build 与现有 tests

只有 clean docs candidate 获授权后才执行 preflight。

### 4.1 Release configure/build

```bash
cmake -S . -B build -G Ninja \
  -DPHAD_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build build --target phad_estimator_tests phad_apps_tests phad_vo_bench \
  --parallel 8
```

### 4.2 Existing-path directed tests

先运行真实覆盖 existing accumulated path 与 transaction 的定向 tests：

```bash
./build/phad_estimator_tests \
  --gtest_filter='VioInitialization.AccumulatedSeedingSeedsAfterSparseFrames:ColdRootObservePopulation.AccumulatedSeedKeepsCurrentPendingAndEffectiveCountsDistinct:ColdRootObserveAttempt.GeometryRollbackConsumesIdAndNextCommitUsesTheNextId:VioFullState.AccumulatedRootFailureDoesNotLeakPendingObservations'

./build/phad_apps_tests \
  --gtest_filter='OfflineVoSessionTest.*Diag*:OfflineVoSessionTest.*Stream*'
```

随后扩大到：

```bash
./build/phad_estimator_tests
./build/phad_apps_tests
```

记录每条 command、return code、passed/failed/skipped 与实际 GTest filter expansion。当前
没有源码修改，任何失败都是 STOP，不通过顺手修改生产或测试恢复 Green。

### 4.3 Static evidence preflight

1. `git status` tracked clean；
2. `git diff --check` 通过；
3. candidate 相对 `a007cc8` 的 non-doc diff 为空；
4. `#48` checkpoint、evidence JSON 与 candidate control artifact 可读；
5. current `diag.csv` header 精确为 frozen 64 列；
6. dataset root 与 canonical output parent 可读/可写，目标 output directory 尚不存在。

## 5. Canonical replay 授权 gate

preflight 全部通过后先汇报：

- capability 问题与四级 verdict；
- candidate commit/tree/config 与 exact command；
- 本机一次完整 replay 的预计成本；
- output directory；
- canonical run count 为 1；
- invalid artifact、identity mismatch、typed invariant failure 或需要第二次运行时立即停止。

只有用户单独授权后才执行数据 replay。授权不包含重跑 control、其它 sequence、commit、
push、PR 或 Issue comment。

## 6. One canonical V2_03 treatment

计划命令：

```bash
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/V2_03_difficult \
  --out /home/lin/Projects/data/phad-bench/m5-cold-root-accumulated-<timestamp>/V2_03_difficult \
  --sequence-name V2_03_difficult \
  --repo /home/lin/Projects/lin_ws/slam_ws/phad-vio/.worktree/codex-m5-cold-root-accumulated-qualify \
  --estimator-enable-moving-bootstrap \
  --estimator-enable-accumulated-seed
```

执行前将 `<timestamp>` 展开为唯一 UTC identity，并再次检查目标不存在。canonical
invocation 从 sequence 起点运行到自然结束；不用 `--max-frames`，不做 tail replay 或
runtime toggle。

进程结束后立即冻结：

- argv、return code、external wall time；
- git commit/tree/dirty、config text/hash；
- dataset/output absolute path；
- 必需 artifacts 的 size 与 SHA-256；
- candidate run count。

canonical invocation 成功或失败后都不发起第二次数据运行。

## 7. Artifact reconciliation

### 7.1 Identity 与 schema

1. `meta.json` / `summary.json` 可解析；
2. `diag.csv` header 与 frozen 64 列逐字一致；
3. row timestamps/cadence 与完整 input replay 对应；
4. accumulated option snapshot 在全部可归因 cold-root rows 为 true；
5. actual threshold 与 control 相同；
6. canonical config text/hash 与 control 对账，同时显式记录 flag 不在 hash 中；
7. all artifact hashes 进入 evidence JSON。

### 7.2 Typed cold-root ledger

按连续 cold-root rows 分 episode，并对每个 episode输出：

- timestamp range、segment/status cadence；
- bootstrap/keyframe/population trace；
- current、pending unique 与 effective population；
- seed input origin；
- attempt IDs；
- first/farthest phase、primary reason、geometry result；
- current graph 与 commit gate；
- commit timestamp 与 estimate/output presence。

机器检查：

1. phase/reason/gate/presence 组合满足 frozen invariant；
2. accumulated population pass 必须有 pending/effective/threshold 且数值关系合法；
3. attempt IDs 唯一且严格单调，只出现在合法 attempt phases；
4. accumulated attempt 必须与 population pass 同 row 对账；
5. commit 必须与 `ok` status、`est.tum` timestamp 和 session output 对账；
6. rollback attempt 不得冒充 commit。

### 7.3 First divergence 与 control-terminal observation

相对 `#48` control：

1. 比较旧列 projection，定位 first differing row/field；
2. 比较 `est.tum` / `kf.tum` prefix，定位 first differing timestamp；
3. 判断 first divergence 是否早于 control frozen terminal interval；
4. 在该 terminal interval 内统计 treatment status、attempt、commit 与 estimate/output；
5. prefix 分叉后只形成 system-level claim；artifact prefix 相同也不宣称隐藏 state 相同。

### 7.4 Record-only metrics

记录完整 summary、completion/coverage、ATE/RPE、local/intersegment components 与 timing，
但不把它们纳入 capability verdict。`#47` formal gate 保持独立。

## 8. Verdict algorithm

按以下顺序计算一个最终 verdict：

1. E0–E6 任一适用 gate 失败 → `inconclusive`；conditional gate 必须显式记录
   `not_applicable` 原因；
2. 没有 accumulated population pass / attempt → `population_not_reached`，并区分
   `branch_not_exercised` 与 `accumulating_below_threshold` subreason；
3. 有 accumulated attempt、没有 accumulated commit → `seed_attempt_reached`；
4. 有 reconciled accumulated commit 与 estimate/output → `root_committed`。

对 `seed_attempt_reached` 额外发布每个 attempt 的 downstream stop；对
`root_committed` 额外发布 commit 后 output 持续性。下游 failure 不覆盖已经成立的
population/attempt 事实。

## 9. Evidence files 与 review

计划新增：

```text
docs/benchmark/m5/cold-root-accumulated-qualification_<candidate>_0337287b.md
docs/benchmark/m5/cold-root-accumulated-qualification_<candidate>_0337287b.evidence.json
```

并按需更新 `docs/benchmark/m5/README.md` 与 benchmark index。checkpoint 不复述
`#48` 全部合同；以链接引用 control，只记录 identity delta、treatment ledger、verdict、
first divergence、output observation 与 claim boundary。

review 顺序：

1. evidence JSON schema/算术/row reconciliation 自审；
2. code-review Standards 轴检查仓库文档、命名、Issue 与交付流程；
3. code-review Spec 轴逐条核对 frozen spec 的 E0–E6 与 verdict；
4. evidence red-team 检查 config-hash blind spot、global-history confound、attempt/commit
   误报与未运行实验的虚假声明；
5. `git diff --check` 与最终 tracked-path 审查。

## 10. 停止、权限与交付

以下事实触发 STOP：

- base/source/treatment identity 超出 frozen spec；
- 现有 schema 或 diagnostics 不足；
- build/tests、typed invariant、artifact integrity 或 output reconciliation 失败；
- 需要 threshold/default/source/schema/seam 变化；
- 需要第二次 V2_03、tail replay 或其它 sequence；
- Issue、spec、源码与历史证据出现实质冲突。

STOP 后保留首个差异与 artifact，回到用户确认；不在同一轮扩大 scope。

后续每项均需单独授权：

1. spec/plan local commit；
2. canonical V2_03 replay；
3. evidence local commit；
4. push / PR；
5. Issue #50 / #42 comment 或状态变化；
6. merge 与 branch 清理。

本片完成只发布 capability verdict 与可复现 evidence，不改变 default 或 `#47` formal
gate。
