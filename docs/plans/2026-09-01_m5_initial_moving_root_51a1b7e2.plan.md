---
name: M5 Slice B estimator-owned initial moving root
overview: 以单一 VioEstimator::update() seam 按 RED→GREEN 交付 verified connected stereo + raw IMU 的 initial moving root，并以 deterministic oracle、V1_02-118、fresh MH_01 与 corridor1-100 形成 bounded checkpoint。
todos:
  - id: freeze-controls
    content: 冻结 GitHub issue 51 authority、ba607c5 clean controls、dataset/config/evaluator identities，并生成可重复的 MH_01 fresh baseline
    status: pending
  - id: typed-lifecycle-red-green
    content: 先写 InitializationDiagnostics/UpdateStatus 合法组合 RED，再加入 compact in-memory types 与 default/presence contracts
    status: pending
  - id: verified-evidence-red-green
    content: 逐个写 visual seed/verified pose、latest component、bounded retention、evidence revision RED，再实现单一 state/window evidence admission
    status: pending
  - id: moving-math-red-green
    content: 以 independent oracles 驱动 gyro、fresh PIM、gravity/velocity、joint GLS 与 chi-square pure math helpers
    status: pending
  - id: public-moving-root-red-green
    content: 通过 public update sequence逐步实现6/250 eligibility、deterministic attempt、current graph authority与latest X/V/B success
    status: pending
  - id: transaction-failures-red-green
    content: 逐个实现 recoverable evidence commit、fatal packet rollback、attempt audit和最早 typed reason negative arms
    status: pending
  - id: app-ordering-legacy-retirement
    content: 修正 accepted camera rotation snapshot ordering，原位接管 moving flag，并删除旧 moving/accumulated-seed initial-root path
    status: pending
  - id: bounded-real-gates-checkpoint
    content: 依次运行完整unit、synthetic、V1_02-118、corridor1-100与MH_01 controls，审查diff并生成本地Slice B checkpoint
    status: pending
isProject: false
---

# M5 Slice B：estimator-owned initial moving root implementation plan

- Plan ID：`51a1b7e2`
- Issue：[#51](https://github.com/Nothand0212/phad-vio/issues/51)
- Wayfinding map：[#42](https://github.com/Nothand0212/phad-vio/issues/42)
- ADR：[ADR-0003](../adr/0003-estimator-owned-initial-moving-root.md)
- Design：[M5 initial moving root](../design/m5-initial-moving-root.md)
- Spec：[2026-09-01-m5-initial-moving-root.md](../specs/2026-09-01-m5-initial-moving-root.md)
- Product code baseline：`ba607c5738b3c5d9bc9cb0423199eee67648b0b3`
- Slice A checkpoint：[tum-vi-product-input-seam_ba607c5_noconfig.md](../benchmark/m5/tum-vi-product-input-seam_ba607c5_noconfig.md)

状态：**已定稿**（用户于 2026-09-01 逐项确认；尚未授权或执行实现、commit、push或远程实验）。

## 1. 唯一产品 RED

Current moving bootstrap 可以在未完成多帧 gyro bias、gravity 与 velocity 闭合时返回 `kOk`。本片必须同时消除两种假成功：

1. 解析 moving input 在6/250与后续 gates之前产生 estimate；
2. 形式上提交 root，但 `V1_02_medium` 首118帧立即产生不健康trajectory。

实现只服务以下产品链：

```text
OfflineVoSession / direct test caller
  → VioEstimator::update()
  → verified connected visual evidence
  → isolated initial-moving candidate
  → existing current graph
  → VioUpdateResult latest X/V/B or typed kInitializing/failure
```

## 2. 已确认 seams 与测试面

### 2.1 Public behavior seam

长期行为只通过：

- `VioEstimator::update()`；
- `VioUpdateResult::{status, estimate, diagnostics, message}`；
- existing `observationTimestamps()` rollback control；
- `runOfflineVoSession()` 与现有 trajectory/diag/counts；
- existing bench/eval artifacts。

Public tests 不读取 `VioUpdateState`、GTSAM graph、PIM、candidate objects 或调用次数。

### 2.2 Estimator-private math seam

在 `phad/estimator/internal/` 增加无 I/O、无 state ownership 的 pure math helper。建议文件布局：

```text
phad/estimator/internal/
  initial_moving_root.hpp/.cpp       # candidate input/result 与 sequential/joint solve
  initial_moving_root_math.hpp/.cpp  # canonical residual、covariance、SVD/GLS/DoF
```

若实现时更小的文件布局能保持同一职责，可在不改变 interface/ownership 的前提下合并；不得拆出 stateful initializer。

Private tests 可 include internal headers，但不得因此把 internal types装入 `phad::estimator` public headers。

### 2.3 Primary touched files

预期修改：

- `phad/estimator/types.hpp`；
- `phad/estimator/internal/vio_update_transaction.hpp`；
- `phad/estimator/internal/imu_interval.{hpp,cpp}`（只在现有 exact splice seam不足时最小扩展）；
- `phad/estimator/vio_estimator.cpp`；
- `apps/offline_vo_session.cpp`；
- `apps/phad_vo_bench.cpp`；
- `CMakeLists.txt`；
- `tests/estimator/vio_full_state_test.cpp`；
- `tests/estimator/vio_lifecycle_test.cpp`；
- new private math tests；
- `tests/apps/offline_vo_session_test.cpp`；
- `tests/apps/tum_vi_corridor1_product_test.cpp`；
- `scripts/validate_m5_initial_root.py`；
- `tests/scripts/test_validate_m5_initial_root.py`；
- related CLI tests与module README/AGENTS；
- final M5 checkpoint。

## 3. Step 0：冻结 control 与实验身份

### 3.1 Worktree authority

Implementation开始时重新记录：

```bash
git status --short --branch
git rev-parse HEAD
git diff --check
```

从包含已定稿 ADR/design/spec/plan 的明确 commit创建短分支。保留当前 worktree中的用户既有文档修改，不 reset、clean、checkout或批量格式化。

### 3.2 Fresh MH_01 baseline prerequisite

在查看candidate output前，用exact `ba607c5` 创建隔离clean source/build。以下
命令是 baseline authority；实现者只替换由 `mktemp` 生成的本地artifact root，
不得改变 build/bench options：

```bash
CONTROL_ROOT="$(mktemp -d /tmp/phad-m5-mh01-control.XXXXXX)"
CONTROL_SRC="$CONTROL_ROOT/src"
CONTROL_BUILD="$CONTROL_ROOT/build"
CONTROL_ARTIFACTS="$CONTROL_ROOT/artifacts"
MH01_ROOT=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy
printf 'export CONTROL_ROOT=%q\nexport CONTROL_ARTIFACTS=%q\n' \
  "$CONTROL_ROOT" "$CONTROL_ARTIFACTS" \
  > /tmp/phad-m5-mh01-control.env

git worktree add --detach "$CONTROL_SRC" \
  ba607c5738b3c5d9bc9cb0423199eee67648b0b3
cmake -S "$CONTROL_SRC" -B "$CONTROL_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DPHAD_BUILD_TESTS=ON
cmake --build "$CONTROL_BUILD" --target phad_vo_bench -j2
mkdir -p "$CONTROL_ARTIFACTS"
(
  cd "$MH01_ROOT"
  LC_ALL=C find . -type f -print0 | sort -z | xargs -0 sha256sum
) > "$CONTROL_ROOT/MH_01_easy.input.sha256"

for run in 1 2; do
  "$CONTROL_BUILD/phad_vo_bench" "$MH01_ROOT" \
    --out "$CONTROL_ARTIFACTS/run-$run" \
    --sequence-name MH_01_easy --config-label default \
    --gt-euroc "$MH01_ROOT" --repo "$CONTROL_SRC" \
    --max-dt-ms 2.5 --min-match-rate 0.5 --rpe-delta-s 1.0 \
    --estimator-enable-moving-bootstrap --force
done
```

冻结：

```text
dataset root / ordered manifest / calibration identity
default_0337287b canonical config
compiler、GTSAM、OpenCV、build type
evaluator version、SE3 alignment、association tolerance、RPE delta=1 s
```

两个 run 的 `meta.json` 都必须给出 exact `ba607c5`、`git_dirty=false` 与
`config_hash=0337287b`；否则 baseline无效。记录：

- `meta.json`/config identity；
- summary ATE/RPE/completion/coverage/segments/reanchors；
- status cadence与trajectory timestamps；
- relevant artifact hashes；
- run-to-run envelope。

默认candidate comparison tolerance为`1e-6 m`。若control自身超过该值，必须在candidate结果可见前按实测上界冻结新tolerance并说明数值来源；无法建立稳定control则停止实现。

### 3.3 Current controls

配置compile database并先运行当前unit controls：

```bash
cmake -S . -B build-m5 -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPHAD_BUILD_TESTS=ON
cmake --build build-m5 --target phad_estimator_tests phad_apps_tests -j2
./build-m5/phad_estimator_tests
./build-m5/phad_apps_tests --gtest_filter='OfflineVoSessionTest.*'
```

Baseline failure先定位，不开始candidate implementation。旧`phad_apps_mh01_test`不进入formal denominator。

## 4. Cycle 1：typed lifecycle tracer bullet

### 4.1 RED

在`tests/estimator/types_test.cpp`与最小public update test逐个增加：

1. `InitializationDiagnostics` inactive defaults；
2. path/phase/outcome/reason合法组合；
3. optional group presence；
4. `attempt_id`只在真实attempt；
5. pending/recoverable→`kInitializing/nullopt`；
6. committed→`kOk/estimate`；
7. fatal→`kFailed/nullopt`；
8. initial moving path不产生`kRejected`；
9. default false的static-only timeout为
   `kFailed/kStaticTimeout/kRolledBack`，post-call committed state保持packet前
   identity。

每加一条先运行并确认RED来自缺失合同，再写最少GREEN；不预写后续math或state machine。

### 4.2 GREEN

在`types.hpp`增加spec冻结的 enums、summary value aggregates与always-present `UpdateDiagnostics::m_initialization`。用单一private/local builder或validation helper生成合法组合；不要让各return path自由拼装不一致字段。

此cycle不扩`VoDiagRow`、CSV/JSON writer或summary schema。

### 4.3 验证

```bash
cmake --build build-m5 --target phad_estimator_tests -j2
./build-m5/phad_estimator_tests --gtest_filter='*Initialization*:*Types*'
```

## 5. Cycle 2：verified visual evidence 与 bounded state

### 5.1 RED sequence

通过public `update()`逐条增加：

1. keyframe与non-keyframe verified poses都计入evidence；
2. PnP proposal/constant visual guess不直接计数；
3. current latest未verified时不attempt；
4. disconnected components不pooling；
5. interleaved disconnected frame不阻止后续reconnect；
6. component必须覆盖latest；
7. window eviction后interval exact closure；
8. evidence revision只在verified latest admission结算后递增；
9. 同一revision最多一次attempt。

另以 internal provenance tests固定：live window root的 predecessor/IMU均
absent；每个non-root frame两者均present且 endpoints/duration精确；selected
candidate gauge root在isolated view中重新成为无 predecessor/interval 的root。

Tests只断言public status、typed summaries、attempt ID与existing observation history，不读取window。

### 5.2 GREEN state shape

- 在existing `WindowFrame`加入typed visual seed/evidence与factor-mask provenance；
- 保持单一`VioUpdateState::m_window`；
- 把visual admission、component selection与exact interval splice藏在estimator implementation；
- candidate input以值复制，不持有live references；
- existing `enforceWindowCapacity()`与landmark pruning继续作为唯一bounded lifecycle。

### 5.3 Visual graph gate

逐测试实现：

- connected metric-stereo graph；
- final factor residual mask；
- underlying-Gaussian per-factor chi-square 0.99 mask、一次masked re-solve与
  final no-remask fit verdict；
- unique connected landmark population；
- weakest non-gauge pose support；
- fit与cheirality；
- QR non-root pose joint marginal covariance。

先使每条visual RED独立GREEN，再进入inertial math。

## 6. Cycle 3：private initial-moving mathematics

### 6.1 Independent fixture

建立无随机、分段解析fixture：

- 6 image frames / exactly250 ms；
- noncommuting三轴rotation；
- time-varying 3D acceleration；
- `b_a=0`、known `b_g=[0.01,-0.02,0.03]`；
- integer-ns timestamps与明确interval endpoint ownership；
- deterministic multi-depth stereo landmarks。

Expected pose/velocity/specific force/gyro、residual/Jacobian与dense GLS不得调用production helper生成。

### 6.2 RED→GREEN order

每项一个test/最小实现：

1. canonical residual与GTSAM factor error；
2. analytic Jacobian与independent central difference；
3. visual covariance mapping、PIM block diagonal innovation及
   covariance/factorization non-finite/invalid-contract fatal分类；
4. supported-subspace whitening与Q；
5. normalized column/SVD rank exact boundary；
6. non-finite whitened residual/Jacobian/column norm fatal分类；
7. condition equality/nextafter boundary；
8. gyro-only solve from zero；
9. final-bias raw reintegration；
10. fixed-scale gravity/velocity closure；
11. bounded joint refinement；
12. no-convergence；
13. joint chi-square/DoF equality与nextafter。

Private tests必须报告independent expected/actual与upstream margins；不得用productionoutput作为oracle。

## 7. Cycle 4：public exact moving root

### 7.1 Eligibility tracer bullets

从同一解析truth派生并依次GREEN：

- 5 poses/250 ms：pending，无attempt；
- 6 poses/<250 ms：pending，无attempt；
- 6 poses/exact250 ms：发生attempt；
- count/duration同时不足按固定gate order报告primary reason。

同一public seam另行钉住static arbitration：default false stationary success、default false non-stationary timeout，以及flag true时static-ready在moving floor前优先提交。

### 7.2 Current-graph authority vertical slice

在 exact success test 转绿前，先把 joint-consistent candidate 以值复制到
isolated `VioUpdateState`，复用 existing graph build/solve/quality/cull/reopt：

- root prior、`X/V/B`、IMU、bias RW与visual factor ownership全部验证；
- graph-optimized result是唯一可published state；
- initialization candidate只作seed；
- graph result不重复执行shared-bias initialization chi-square；
- validated graph state以noexcept swap或等价原子替换提交。

该最小 graph path通过定向 internal/public test后，才进入下一小节。

### 7.3 Exact success

Public sequence必须在资格前保持`kInitializing/nullopt`，成功时满足：

```text
latest translation error <= 1e-3 m
latest rotation error    <= 1e-4 rad
latest velocity error    <= 1e-3 m/s
gyro bias error          <= 1e-4 rad/s
gravity direction error  <= 1e-4 rad
```

Commit必须是graph-optimized latest state；随后一个active update继续成功并使用同一segment/state lifecycle。

## 8. Cycle 5：orthogonal negative arms 与 transaction

### 8.1 Recoverable arms

从success fixture一次改变一个因果边，按spec顺序逐个RED→GREEN：

- visual connectivity/population/support/fit/cheirality；
- gyro与gravity/velocity rank；
- full-rank ill-condition；
- no-convergence；
- joint supported-subspace DoF不足；
- full-rank healthy-condition joint inconsistency；
- current graph underconstraint；
- current graph finite quality/admission rejection。

每个test必须断言：更早gates有明确PASS margin、最早reason、`attempt_id` presence、`kEvidenceCommitted`、无estimate，以及下一verified revision可恢复。

### 8.2 Fatal arms

使用真实可达invalid input与现有/最窄test-only fault seam覆盖：

- missing root prior/key ownership；
- visual factor evaluation/whitening exception或non-finite `q_f`；
- non-finite whitened residual/Jacobian/column norm、QR/PIM/S covariance或
  supported-subspace factorization；
- non-finite candidate/current graph；
- generic optimizer exception；
- interval/provenance contract。

断言`kFailed`或pre-staging`kInvalidInput`、`kRolledBack`/`kNotEvaluated`、attempt audit按值保留、topology diagnostics来自restored state，并与fresh control后续update exact一致。

不得为fault injection增加production public backdoor。

## 9. Cycle 6：app ordering、activation 与 artifact verifier

### 9.1 Accepted camera rotation ordering

先在`tests/apps/offline_vo_session_test.cpp`建立non-identity `T_B_left_rectified` RED：

1. estimator返回`kOk`后先计算current accepted camera rotation；
2. keyframe snapshot使用同帧accepted camera rotation；
3. 再更新last-accepted state；
4. 下一帧parallax compensation不使用stale identity/body-only rotation。

GREEN只修改session ordering/extrinsic composition，不引入initializer callback。

### 9.2 Activation与legacy removal

- 让existing moving flag原位调用formal path；
- default保持false；
- 删除旧short-suffixmovingroot；
- 删除`enable_accumulated_seed` option、CLI、wiring与legacytests；
- 更新CLI tests与config snapshot expectations；
- 确认同一config hash必须与code commit共同解释。

### 9.3 Artifact verifier RED→GREEN

新增 stdlib-only `scripts/validate_m5_initial_root.py`，不得修改 regular
CSV/JSON schema。先用 `tests/scripts/test_validate_m5_initial_root.py` 的临时
fixtures覆盖：

- summary execution/health fields按key读取；
- diag按header name读取，status cadence不依赖column position；
- 每个`ok` row与`est.tum` pose按timestamp一一对应，TUM serialization容差
  `<=500 ns`；
- V1 absolute predicates；
- MH control/candidate relative predicates；
- missing/non-finite/duplicate timestamp、错误cadence与任何predicate failure均
  nonzero exit并输出首个expected/actual差异。
- V1 manifest text digest与预注册digest不一致时，在读取candidate metrics前
  拒绝。

固定 CLI：

```text
validate_m5_initial_root.py v102-118 RUN_DIR --manifest-sha256 FILE \
  --expected-manifest-digest SHA256
validate_m5_initial_root.py mh01-static CANDIDATE_DIR \
  --control-dir CONTROL_DIR --comparison-tolerance-m VALUE \
  --manifest-sha256 FILE
```

成功 exit `0` 并输出machine-readable JSON verdict；输入/合同错误 exit `2`；
predicate不通过 exit `1`。Validator tests必须在任何candidate real output可见前
通过。

## 10. Cycle 7：bounded formal gates

### 10.1 Compile、完整 unit 与 synthetic gate

这是 candidate 的第一个 formal gate：

```bash
cmake -S . -B build-m5 -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPHAD_BUILD_TESTS=ON
cmake --build build-m5 --parallel 2
ctest --test-dir build-m5 --output-on-failure -L unit
./build-m5/phad_estimator_tests \
  --gtest_filter='*InitialMoving*:*Initialization*'
python3 -m unittest discover -s tests/scripts \
  -p 'test_validate_m5_initial_root.py'
```

全量 build保证所有带`unit` label的 targets均已生成，包括
`phad_eval_tests`、`phad_viz_tests`与`phad_gyro_alignment_tests`。任何failure
在进入real data前原位闭合。

### 10.2 V1_02-118

运行固定prefix：

```bash
V102_ROOT=/home/lin/Projects/data/thidparty/euroc/native/V1_02_medium
V102_OUT=/tmp/phad-m5-v102-118
V102_MANIFEST=/tmp/phad-m5-v102-118.input.sha256
V102_MANIFEST_DIGEST=a80c02d6c9bd9e5cd76412f30b6859fd8acdabf0885de1cba494725a183ced8f
(
  cd "$V102_ROOT"
  LC_ALL=C find . -type f -print0 | sort -z | xargs -0 sha256sum
) > "$V102_MANIFEST"
test "$(sha256sum "$V102_MANIFEST" | awk '{print $1}')" = \
  "$V102_MANIFEST_DIGEST"
./build-m5/phad_vo_bench \
  "$V102_ROOT" --out "$V102_OUT" \
  --sequence-name V1_02_medium --config-label m5_slice_b_v102_118 \
  --gt-euroc "$V102_ROOT" --repo "$PWD" --max-frames 118 \
  --max-dt-ms 2.5 --min-match-rate 0.5 --rpe-delta-s 1.0 \
  --estimator-enable-moving-bootstrap --force
python3 scripts/validate_m5_initial_root.py v102-118 "$V102_OUT" \
  --manifest-sha256 "$V102_MANIFEST" \
  --expected-manifest-digest "$V102_MANIFEST_DIGEST"
```

Validator检查spec §15.1全部identity/execution/cadence/health predicates。首次失败立即停止；不得扩大frames或调threshold。

### 10.3 TUM VI corridor1-100

```bash
cmake -S . -B build-tumvi-m5 -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPHAD_BUILD_TESTS=ON -DPHAD_ENABLE_TUMVI_CORRIDOR1_TESTS=ON
cmake --build build-tumvi-m5 --target \
  phad_tumvi_corridor1_test phad_tumvi_corridor1_product_test -j2
PHAD_TUMVI_CORRIDOR1_PATH=/home/lin/Projects/data/thidparty/tum/dataset-corridor1_512_16 \
  ctest --test-dir build-tumvi-m5 --output-on-failure -L tumvi-corridor1
```

保留#53structural predicate并增加spec §15.3 root/cadence/trajectory assertions。

### 10.4 MH_01 controls 与 fresh static regression

```bash
cmake -S . -B build-mh01-m5 -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DPHAD_BUILD_TESTS=ON -DPHAD_ENABLE_MH01_TESTS=ON
cmake --build build-mh01-m5 --target \
  phad_camera_mh01_test phad_frontend_mh01_test phad_apps_tests phad_vo_bench -j2
PHAD_EUROC_MH01_PATH=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  ctest --test-dir build-mh01-m5 --output-on-failure \
  -R 'Mh01RectifyTest|Mh01FrontendTest'
PHAD_EUROC_MH01_PATH=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  ./build-mh01-m5/phad_apps_tests --gtest_filter='OfflineVoSessionTest.*'
```

然后以与§3.2 fresh control完全相同的bench options运行candidate：

```bash
MH01_ROOT=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy
MH01_CANDIDATE=/tmp/phad-m5-mh01-candidate
source /tmp/phad-m5-mh01-control.env
./build-mh01-m5/phad_vo_bench "$MH01_ROOT" \
  --out "$MH01_CANDIDATE" \
  --sequence-name MH_01_easy --config-label default \
  --gt-euroc "$MH01_ROOT" --repo "$PWD" \
  --max-dt-ms 2.5 --min-match-rate 0.5 --rpe-delta-s 1.0 \
  --estimator-enable-moving-bootstrap --force
python3 scripts/validate_m5_initial_root.py mh01-static \
  "$MH01_CANDIDATE" \
  --control-dir "$CONTROL_ARTIFACTS/run-1" \
  --comparison-tolerance-m 1e-6 \
  --manifest-sha256 "$CONTROL_ROOT/MH_01_easy.input.sha256"
```

若§3.2在candidate可见前冻结了更大的repeatability tolerance，则命令使用该已
记录值。旧`phad_apps_mh01_test`保持已知陈旧non-gating事实，不纳入denominator。

## 11. Diff review

全部formal gates通过后：

```bash
git diff --check
git status --short --branch
git diff --stat
git diff -- phad/estimator apps tests CMakeLists.txt docs
```

确认：

- public header无GTSAM类型；
- regular CSV/JSON schema无变化；
- deletedlegacy option无parser/config/test残留；
- user既有文档修改未被整理、覆盖或混入实现commit；
- no placeholder、silent fallback或partially initialized success。

## 12. 建议 commit slices

只有对应vertical slice GREEN且diff独立可审时才commit；未经用户授权不执行commit。

| slice | subject |
|---|---|
| typed lifecycle + verified evidence | `estimator: stage initial moving evidence (#51)` |
| private sequential/joint solve | `estimator: solve initial moving root (#51)` |
| graph authority + transaction | `estimator: commit initial moving root atomically (#51)` |
| app ordering + legacy retirement | `apps: preserve initial root camera cadence (#51)` |
| real gates + checkpoint docs | `tests: gate M5 initial moving root (#51)` |

不得把所有RED、implementation、real gates与checkpoint压成一个commit。

## 13. Checkpoint

全部formal matrix通过后，在`docs/benchmark/m5/`新增Slice B checkpoint，记录：

- code commit/tree与clean/dirty identity；
- config canonical text/hash或明确的no-config test identity；
- dataset manifests/calibration identities；
- compiler/GTSAM/OpenCV/evaluator；
- 每个formal command的actual result；
- synthetic tolerances与mutant outcomes；
- V1_02-118、TUM100、fresh MH_01 control/candidate结果；
- 未运行EuRoC-11及其原因；
- next Slice边界。

Checkpoint完成后立即停止，不开始cold-root recovery、world-frame continuity、posterior precision或default-on。

GitHub issue评论、关闭或其他remote write不属于本计划的自动步骤；只有获得用户
单独授权后，才可引用已完成的local checkpoint执行。

## 14. 失败协议

任一gate失败：

1. 停止后续matrix；
2. 保存input/config/evaluator与actual typed outcome；
3. 定位相对control的最早分叉；
4. 一次只提出一个可证伪根因；
5. 只修改该因果边；
6. 原位重跑同一gate；
7. 不改变冻结thresholds/predicates，不扩序列，不实现后续Slice。

若独立oracle证明已定稿合同内部矛盾或不可实现，停止implementation并回到design对齐；不得静默改spec继续。
