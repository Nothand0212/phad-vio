---
name: M5 cold-root current-path Observe
overview: 在唯一的 VioEstimator::update() current path 内发布 frozen typed cold-root diagnostics，并由 OfflineVoSession 逐字段投影到固定 64 列 diag.csv；本片只取得 Q1 Observe 证据，不改变任何初始化、求解、地图或 lifecycle 决策。
todos:
  - id: align-plan
    content: '分段确认 module authority、typed/presence 合同、session schema 与一次性 V2_03 证据门'
    status: completed
  - id: estimator-contract-red-green
    content: '经 public update seam 逐例完成 phase/reason/gate/bootstrap/population presence 的 Red→Green，并对拍 legacy result'
    status: completed
  - id: attempt-rollback-red-green
    content: '经合法 root rejection/commit 输入锁定 geometry outcome、attempt identity、rollback 后不回收与 fresh-control 等价'
    status: completed
  - id: session-csv-red-green
    content: '逐字段扩展 VoDiagRow 与 diag.csv 至 frozen 64 列，验证 optional 空值、真实零、旧 26 列 projection 与 row cadence'
    status: completed
  - id: targeted-verification
    content: '按 estimator 定向门、apps/schema 门、相关 build/tests 的顺序完成验证与独立审查'
    status: completed
  - id: authorize-clean-candidate
    content: '定向门全绿后汇报 diff、实验问题、约 38 秒成本与停止条件，并单独取得 local candidate commit 授权'
    status: completed
  - id: run-v2-03-once
    content: '从授权后的本机 Release clean candidate 仅回放一次 V2_03_difficult，复用 frozen db22656 control'
    status: completed
  - id: reconcile-evidence
    content: '对账 289 个 terminal rows、旧行为投影与全部新诊断 invariant，记录可复现 Q1 Observe 证据并完成 red-team 审查'
    status: completed
isProject: false
---

# M5 cold-root current-path Observe

## 0. 状态与 authority

本计划已于 2026-08-28 完成分段确认、TDD 实施、local candidate、单次 replay 与
证据对账；checkpoint 见
[`cold-root-current-path-observe_6a193a8_0337287b.md`](../benchmark/m5/cold-root-current-path-observe_6a193a8_0337287b.md)。

| 对齐段 | 状态 |
|---|---|
| Module authority、数据流与 typed design（§2–§3） | 2026-08-28 已确认：A |
| Estimator taxonomy/presence/attempt TDD（§5.1–§5.3） | 2026-08-28 已确认：A |
| Session/64 列 projection（§5.4） | 2026-08-28 已确认：A |
| 验证、clean candidate 与一次性 replay（§6–§9） | 2026-08-28 已确认：A |

按具体性从高到低使用以下 authority：

1. [`M5 cold-root seed Observe frozen spec`](../specs/2026-08-28-m5-cold-root-seed-observe.md)；
2. [Issue #48](https://github.com/Nothand0212/phad-vio/issues/48) 当前正文；
3. [`evidence-gated-integration.md`](../agents/evidence-gated-integration.md)、
   [`incremental-development.md`](../agents/incremental-development.md) 与 Git/TDD/C++ 约定；
4. estimator、apps、tests 当前 public interface、源码与既有测试。

| 项 | 已核对事实 |
|---|---|
| worktree | `/home/lin/Projects/lin_ws/slam_ws/phad-vio/.worktree/codex-m5-cold-root-observe` |
| branch / upstream | `codex/m5-cold-root-observe` / `origin/codex/m5-cold-root-observe` |
| HEAD | `e6705d57d3959abcb6b273db6a32c0d63298e0ef`；tracked worktree clean |
| base | `main@34c309196440710be86d0a05e32901d58bfdd9aa`；merge-base 相同；ahead 1 / behind 0 |
| HEAD 相对 base | 只新增 frozen spec；production/test baseline 与 `main` 一致 |
| Issue | `#48` 为 OPEN；当前正文与 frozen spec 的目标、范围、schema 和停止条件一致 |
| mechanism control | `db226563c9a386bc70e4f19665ec909bd80905ad` / tree `a011a891...` / `default_0337287b` |
| control artifact | `/home/lin/Projects/data/phad-bench/m4-mapped-bearing-final-db22656-20260828T022252Z/V2_03_difficult` |

当前未发现 frozen spec 与 production behavior 的实质冲突。`apps/AGENTS.md` 对既有
initialization summary warning 的描述与当前 session 实现存在历史文档漂移；本片以
冻结的 runtime warning/FrameCounts 合同保持现状，只同步新增 diagnostics/schema，
不把该文档漂移并入实现范围。

## 1. 本片目标、问题与边界

唯一可证伪问题是：

> `V2_03_difficult` 在 `96.6–116.7 s` 的 289 个 terminal
> `initializing` frames，最早被哪个现有 cold-root phase/reason/gate 持续阻塞，
> 当前实现是否发生真实 `seedRoot()` attempt？

本片交付三项稳定能力：

1. estimator-owned `ColdRootObserveDiagnostics` typed value；
2. `OfflineVoSession` 对该 value 的逐字段镜像；
3. 旧 26 列尾部追加 frozen 38 列的固定 64 列 `diag.csv`。

本片的输出权限止于 Q1 Observe。既有 bootstrap、seed、solver、posterior、segment、
active-map、PnP、support、transaction 与 world-frame 行为保持基线；formal IMU
excitation、conditioning、initialization solve 三项始终为 `not_evaluated`。
`#47` formal gate 继续为 `FAIL`，Observe 结论只回答 current-path 可观测性。

## 2. graphify 核对的数据流与 module authority

在指定 worktree 生成 ignored `graphify-out/graph.json` 后，使用 vocabulary-first DFS
并回到源码逐点核对，得到以下调用关系：

```text
OfflineVoSession::runOfflineVoSession
  └─ VioEstimator::update                         唯一 measurement seam
       ├─ bootstrapStats / recentBootstrapBegin   static 与 moving readiness
       ├─ keyframe / current-or-pending population gates
       ├─ VioUpdateTransaction                    staged state owner
       │    ├─ seedRoot                           唯一真实 root attempt call-site
       │    ├─ buildGraph → solveGraph → validation
       │    └─ commit / rollback
       └─ VioUpdateResult::UpdateDiagnostics      typed authority

runOfflineVoSession result.diag
  └─ phad_vo_bench / phad_stereo_vo_probe         composition roots
       └─ writeDiagCsv → diag.csv                 fixed field serialization
```

`runOfflineVoSession()` 调用 `update()`，但不直接调用 `writeDiagCsv()`；writer 由 bench
和 probe 在 session 返回后统一调用。这一边界意味着：

- estimator 直接在真实 branch 上写 phase、reason、gate、presence、attempt；
- session 不解析 `message`、不复算 predicate、不持有第二套 cold-root policy；
- composition root 与 writer 只消费 `VoDiagRow`，不新增 CLI、callback 或 sidecar。

## 3. 已确认的 typed design

### 3.1 Public value 与稳定 token

在 `phad/estimator/types.hpp` 增加 frozen taxonomy 对应的 scoped enums：

- `ColdRootPhase`；
- `ColdRootReason`；
- `ColdRootGateState`；
- `ColdRootBootstrapPath`；
- `ColdRootSeedInputOrigin`；
- `ColdRootGeometryResult`。

`ColdRootObserveDiagnostics` 聚合 frozen gates 与 optional values，并作为一个成员追加到
`UpdateDiagnostics`。默认构造只产生稳定的 `not_evaluated` enum；partial-phase 数值用
`std::optional`，不以 sentinel/NaN/default-zero 表示缺失。

enum→ASCII token 的无状态 `constexpr` helper 与 enum 放在 estimator types 层，确保
taxonomy 的 token authority 不落入 apps。session row 保留 typed enum/optional，
`writeDiagCsv()` 只调用这些 helper 并按列写值。

### 3.2 Presence 的写入位置

`VioEstimator::update()` 在现有 branch 上直接发布 diagnostics：

1. active segment、discontinuity、raw/observation/provenance invalid 保持
   `not_evaluated/not_evaluated`；cold-root-only optional 依 frozen presence 为空；
2. 当前 packet positive-disparity count 在现有 observation validation 中只累积到
   local，待 uninitialized cold-root provenance validation 全部通过后才发布；
3. 同一位置发布 `enable_accumulated_seed` 的真实 option snapshot；这两项位于
   bootstrap early returns 之前，但不移动或提前填写旧 `num_disparity`；
4. static bootstrap 的 full-window sample/duration/stats 在 moving fallback 覆盖现有
   local stats 之前单独快照；moving option 及 suffix values 只在现有控制流真实读取/
   计算时出现；
5. empty observations 记录 effective population `0`、current origin、实际 minimum，
   并保持 keyframe gate `not_evaluated`；
6. current、pending unique、effective population 分别在各自真实访问点赋值；
7. geometry/current-graph/commit 信息在 transaction rollback 前留在 attempt-local
   result，不从 rollback 后的 estimator state 反推。

### 3.3 Attempt 与 geometry 的事务边界

`attempt_id` counter 放在 `VioEstimator::Impl` 的非事务成员中，从 `1` 起按 estimator
instance 单调递增。它不进入 `VioUpdateState`，因此 transaction clone/rollback 不会
回收 ID。

在唯一 `seedRoot()` call-site 紧邻前完成 counter increment 与 result assignment；
两者和真实调用之间没有 branch、early return 或第二个 policy owner。`seedRoot()` 的
private 返回值扩为 attempt-local outcome，携带：

- 现有成功/拒绝结果；
- `accepted / nonfinite_backprojection / behind_camera / empty_after_filter` token；
- 本 attempt 已接受 landmark count。

该 private outcome 只拆分现有 normal-return branch；exception 传播、map/window
mutation、probe counters、调用次数、caller status/message 与 rollback 次序保持原样。

### 3.4 Session 与 CSV

`VoDiagRow` 逐字段追加 estimator enum/optional 镜像；唯一 row aggregate 在现有位置
复制每一个字段。`kInvalidInput/kFailed` 仍在 row push 前 hard-stop，session 不为它们
增加行。

writer 固定写出原 26 列，再写 frozen 38 列。旧列继续使用现有 fixed/precision 格式；
新增 optional floating values 使用 `max_digits10` round-trip precision，使 moving
epsilon 等非零 threshold 可机读，且不改变旧 26 列 bytes。空 optional 输出空 field，
存在的 boolean 输出 `0/1`，真实数值零仍写 `0`。

## 4. 预计 tracked 修改面

| 文件 | 聚焦修改 |
|---|---|
| `phad/estimator/types.hpp`、`cold_root_observe.cpp`、`internal/cold_root_observe.hpp` | typed taxonomy、token helper、`ColdRootObserveDiagnostics`、`UpdateDiagnostics` member 与 private result invariant |
| `phad/estimator/vio_estimator.hpp/.cpp` | branch-local phase/gate/presence；bootstrap snapshot；private root outcome；non-transaction attempt counter；public return 前 invariant check |
| `tests/estimator/cold_root_observe_test.cpp`（新） | 只经 public `update()` 的 taxonomy/presence/attempt/rollback vertical tests |
| `CMakeLists.txt` | 将新测试源接入既有 `phad_estimator_tests` target |
| `apps/offline_vo_session.hpp/.cpp` | `VoDiagRow` 逐字段镜像与固定 64 列 writer |
| `tests/apps/offline_vo_session_test.cpp` | 64 列 schema、legacy projection、optional empty/zero 与 cadence |
| `phad/estimator/README.md` | 当前 typed Observe contract 与 authority |
| `apps/AGENTS.md` | 当前 64 列 projection 合同 |
| `docs/benchmark/m5/` | replay 后的命令、identity、Q1 histogram/invariant 与行为对账证据 |

实现中若某文件无需语义更新，不为满足清单而修改。frozen spec、CLI/config、summary
schema 与既有 benchmark control 不在修改面。

## 5. TDD 实施顺序

每一步执行一个 public-behavior Red→最小 Green→局部整理循环；不新增 private/injection
test seam，不一次铺开全部测试。

### 5.1 Typed default 与 entry classification

先在新 estimator test 中锁定 enum token、default `not_evaluated` 和 optional 缺失，
再加入最小 public types。随后逐例覆盖：

1. active `kOk`；
2. measurement discontinuity；
3. raw/observation invalid input。

每例同时保存并比较 status、message、estimate 与全部 legacy diagnostics。新增 value
不能影响后续同输入 update 的结果。

### 5.2 Bootstrap predicate 与 early returns

按现有合法 IMU packet 逐个构造并锁定：

1. static evidence collecting；
2. bootstrap timeout；
3. static-ready 的 full-window value/threshold；
4. static-not-ready、moving option 实际读取但关闭；
5. moving-ready 的 full-window static snapshot 与 recent suffix snapshot；
6. ready + empty observations 的 population `0` 与 keyframe-not-evaluated；
7. ready + non-empty + non-keyframe 的 `keyframe_required`。

每个 predicate 的 value/threshold presence 成对断言；formal excitation、conditioning、
initialization solve 在所有结果中保持 `not_evaluated`。

### 5.3 Population、attempt、rollback 与 commit

按以下纵向路径逐个 Red→Green：

1. current population `9` / minimum `10`；
2. accumulated enabled 后本帧纳入 pending unique、仍不足；
3. accumulated population 达门后 effective count/origin 与真实 seed input 一致；
4. 复用现有合法 huge-finite-pixel fixture 触发 root geometry rejection，断言 attempt
   outcome/count 在 rollback 后仍保留；
5. 同一 estimator 的下一次合法 root commit 取得下一 ID，不复用 rejection ID；
6. fresh-control estimator 对拍 commit estimate、transaction state、track timestamps
   与全部 legacy diagnostics。

同一 instance 的综合序列还会证明 collecting、non-keyframe、current不足、pending不足
均没有 ID，真实 rejection/commit 才依次取得 ID。

root graph build/solve/validation failure 当前没有找到可由合法 public input 稳定触达的
cold-root branch；其 production mapping 按真实 failure return 点接入，覆盖报告明确记为
`not_covered`。existing active-segment graph-failure test 不冒充 root 证据，也不增加
production injection seam。geometry secondary token 同样只覆盖合法 public input 能
真实触达的 normal-return branch，其余明确记为 `not_covered`。

### 5.4 Session 与 artifact projection

在现有 `OfflineVoSessionTest` 上逐步完成：

1. header 精确等于 frozen 64 列、顺序固定；
2. synthetic row 的前 26 field vector 精确等于既有 projection；
3. 同列 `nullopt`、数值 `0`、boolean `false` 三种语义可区分；
4. stable enum tokens 与 moving epsilon 可往返解析；
5. TinyEurocFixture 的 row count/timestamp/status/segment cadence 不变；
6. 既有 stream error/hard-stop 仍不增加 failed/invalid row；
7. FrameCounts、warning、summary 与 config snapshot/hash 的既有测试不改语义。

## 6. 定向验证与审查顺序

在计划确认后先建立本 worktree 的 Release build；每个 Red 只运行对应 filter，Green
后逐步扩大。最终门严格按以下顺序：

```bash
cmake -S . -B build -G Ninja \
  -DPHAD_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build build --target phad_estimator_tests --parallel 8
./build/phad_estimator_tests \
  --gtest_filter='ColdRootObserve*'

cmake --build build --target phad_apps_tests --parallel 8
./build/phad_apps_tests \
  --gtest_filter='OfflineVoSessionTest.*Diag*:OfflineVoSessionTest.*Stream*'

cmake --build build --target phad_vo_bench --parallel 8
./build/phad_estimator_tests
./build/phad_apps_tests
```

实际 GTest names 以 RED 测试落地后的 discover 结果为准，并在执行记录中列出精确
filters。全部定向门通过后再运行完整 `phad_estimator_tests` 与 `phad_apps_tests`。

随后进行三层只读审查：

1. `git diff --check`、逐文件 diff 与 frozen 38-column order 对账；
2. 独立 reviewer 检查 authority、transaction、presence、legacy projection 和测试盲区；
3. red-team 从合法 public input、rollback、optional 及 CSV parser 角度复核。

任何 status/message/estimate/legacy diagnostic、algorithm call order 或 session cadence
差异都先定位根因；在等价门恢复前不进入自然 replay。

## 7. Clean candidate 与一次性 V2_03 gate

定向门、完整相关 suites 与审查全部通过后，先向用户汇报：

- 实验问题：terminal 289 行的最早阻塞 gate 与真实 attempt 数；
- frozen control 与 exact candidate command；
- 预算：本机 Release `wall_s≈37.993`，约 38 秒；
- STOP：行为 projection、289 行、gate/value/attempt invariant 任一不一致即停止。

为让 replay 具有 clean source identity，在该节点单独请求 local candidate commit 授权；
授权只覆盖已审查的 #48 tracked paths，不包含 push/merge/PR/Issue 操作。未获授权则停在
已验证的 working-tree diff，不运行正式 replay。

授权后只运行：

```bash
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/V2_03_difficult \
  --out /home/lin/Projects/data/phad-bench/m5-cold-root-observe-<timestamp>/V2_03_difficult \
  --sequence-name V2_03_difficult \
  --repo /home/lin/Projects/lin_ws/slam_ws/phad-vio/.worktree/codex-m5-cold-root-observe \
  --estimator-enable-moving-bootstrap
```

不重跑 db22656 control，不运行 MH_01、其它 EuRoC、EuRoC-11 或 TUM VI。若进程失败或
STOP 命中，保留 artifact 和首个差异，当前轮不修改 threshold/seed/solver 后重跑。

## 8. Frozen control 与 candidate 对账

frozen db22656 V2_03 control：

| 项 | 值 |
|---|---:|
| diag rows / columns | `1921 / 26` |
| status counts | `1141 ok / 771 initializing / 9 visual_outage` |
| segments / last ok | `9 / 96.5 s` |
| terminal interval | `96.6–116.7 s`，`289 initializing` rows |
| terminal max old values | `num_observations=200`，`num_disparity=9` |
| completion / coverage | `0.5939614784 / 0.7613539000` |
| ATE / RPE | `2.0418160161 / 0.6342349346 m` |
| weighted local RMS / intersegment | `0.0602358861 / 1.9815801300 m` |
| config | `default_0337287b`；moving bootstrap enabled；accumulated seed disabled |

candidate 必须完成以下机器对账：

1. `diag.csv` 为 1921 rows / 64 columns；每行 `[0:26]` field projection 与 frozen
   control 相同，timestamp/status/segment cadence 相同；
2. `est.tum`、`kf.tum` byte-identical；
3. status counts、segments、last-ok、completion/coverage、ATE/RPE、段内/段间指标、
   canonical config text/hash 与 frozen 值一致；
4. terminal 289 行完整落在 `96.6–116.7 s`，phase/reason buckets 精确合计 289；
5. 每行验证 phase/reason、gate trace、value/threshold、path/origin/geometry presence；
6. attempt IDs 唯一、严格递增，只出现在 root_geometry/current_graph/commit；无真实
   `seedRoot()` 时全部为空；
7. 输出 phase/reason、bootstrap path、每个 gate 三态分布、attempt count 和最早持续
   阻塞 gate；分散证据报告 `mixed`，不只报最大 bucket；
8. formal excitation/conditioning/initialization solve 三 gate 全部
   `not_evaluated`。

比较忽略 source git identity、wall/timing 和新增 38 列；这些忽略项不参与行为等价
判定。`summary.json` 不假定存在 status-counts key，status counts 从 `diag.csv` 重算并
与 frozen evidence JSON 交叉核对。

## 9. 停止与交付

以下任一事实出现即停止当前轮并回到 frozen spec/用户确认：

- 实现需要改变已有 branch、threshold、algorithm call order 或 transaction semantics；
- 需要新增 CLI、sidecar、callback、second policy 或 production injection seam；
- legacy behavior projection、289-row reconciliation 或 typed invariant 失败；
- 未执行的 formal gate 被写为 passed/failed，或 attempt identity 与 `seedRoot()` 不一致；
- 为解释 artifact 必须开启 accumulated seed、shadow calculation 或扩大数据集。

成功或失败的 Q1 结果都记录 exact source/config/data/command/artifact、coverage boundary、
首个差异或最早阻塞 gate。完成本片只证明 Observe 合同与证据成立，不授予 root 恢复、
dynamic initialization、world-frame alignment 或默认路径变更。
