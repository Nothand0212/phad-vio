---
name: M4 online gyro bias synthetic
overview: 以冻结的 PHAD-M4-ONLINE-GYRO-BIAS-SYNTHETIC-V1 为唯一协议，先锁定 exact-six authority 与双轴审查身份，再用唯一 public RED、独立 Eigen oracle 和 default-off estimator-private 实现资格化 14-state no-eviction online gyro bias；不接真实数据、apps、默认开启或完整 X/V/B。
todos:
  - id: authority-lock
    content: 将 exact-six authority 文件锁在同一 commit/tree，记录逐文件 blob/SHA-256，并由身份分离的 Standards 与 Spec reviewer 对 exact bytes 各自 fresh 零 finding 审查
    status: completed
  - id: red
    content: 在任何 product declaration 前先冻结旧 public API visual control canonical digest/marker identity，再加入唯一 public NoEvictionFourteenPoseRecovery marker region，并保留缺失 online-bias capability 的有效 RED receipt
    status: completed
  - id: oracle
    content: 新建只链接 Eigen3 与 GTest 的独立 oracle target，复算 G0..G13、whitening/cost、rank、奇异值与 condition，并冻结实际输出
    status: completed
  - id: reducer-factor
    content: 先以测试闭合唯一 interval reducer、Q2 regression、private AHRS-composed NoiseModelFactorN、fixed-PIM correction、Jacobian、noise 与 mutant gates
    status: completed
  - id: estimator-state
    content: 以四个 direct field/address gates 实现 private State/RAII，再在 StereoVoEstimator::Impl 内接入 default-off G/RW/root forest、事务式 writeback 与最多三轮 fixed-PIM solve，使 14-state no-eviction recovery、m_window_biases 和 kOk-only extra-round 主门转 GREEN
    status: completed
  - id: lifecycle-gates
    content: 闭合 validation/precedence/provenance、missing/gap/rejected/segment/evicted 断链、unknown-predecessor rollback-before-materialize、reanchor/reopt、private transaction 四门、zero-drift、determinism 与 off byte-identity gates
    status: completed
  - id: activation-deletion
    content: 用 RED 前冻结的 repo-external verifier，从只读 clean-GREEN archive 独立执行 positive、两条 deletion、graph activation、3 条 transaction source arms、static-cap 与 12 条 production-source numerical-semantic mutant arms，并恢复同一 source/marker/command/outcome identity
    status: pending
  - id: verify-result
    content: 独立 verifier 在 clean build 只运行 oracle、受影响 estimator 与 Q2 regression，逐门记录 actual/verdict；随后仅按 bookkeeping allowlist 写 result、plan 状态和 roadmap 权限
    status: pending
isProject: false
---

# M4 online gyro bias synthetic 实施计划

日期：2026-08-18

状态：**Replay2 已形成 `HARD_ERROR` 并强制 STOP；不是 `PASS` 或 scientific/product `FAIL`。
前 6 个 todo 已完成；`activation-deletion` 仅完成 5/19 条有效 negative arms，`verify-result` 也未
完成，二者保持 pending。`permission_granted=false`。**

计划 ID：`1e3569b4`；protocol ID：`PHAD-M4-ONLINE-GYRO-BIAS-SYNTHETIC-V1`；
跟踪 issue：[#40](https://github.com/Nothand0212/phad-vio/issues/40)。

Post-verdict evidence：
[result ledger](../research/m4-online-gyro-bias-synthetic-result.md)；冻结 receipt：
`/home/lin/orca/evidence/phad-m4-online-bias-v1-replay2-634c4ec/replay2-hard-error-receipt.json`
（mode `0444`，SHA-256
`bf4dc238eef9751042c7077a2713c9d0fed64fa3c189bf00b48e86179b89716a`）。

## 0. 唯一 authority、目标与权限边界

[online gyro bias synthetic design](../research/m4-online-gyro-bias-synthetic-design.md)
是本计划唯一 normative executable authority。开工前冻结身份为：

```text
SHA-256  e7db8acdf7d87d78f74d8006039dca1e9134ff85e4940b6cbaadc1698c00d37d
git blob 07489430cde613f9e0fc431885c7bbd3835a9ff7
```

本计划只规定执行顺序、文件边界、证据交接与 commit 切片，不另行解释或替代 design 中的
truth、fixture、公式、阈值、错误语义和 verdict。开工 preflight 若 design 任一 byte 与上述身份
不同，立即 `STOP`；先以新 protocol version 闭合 authority transaction，不能把漂移后的 bytes
继续叫作 V1。看过 qualification actual 后不得原地修改 V1。

相关 authority 与 provenance：

- [ADR-0002](../adr/0002-stage-gated-gyro-only-bias-state.md) 只授权 synthetic、default-off、
  no-real-caller 的 estimator-private `G(k)` 窄例外；
- [conventions](../design/conventions.md) 保持完整 VIO 的 canonical bias key 为
  `B(k): imuBias::ConstantBias`；同一 graph 禁止 `G/B` 双写或 alias；
- [roadmap](../design/roadmap.md) 记录 Q3 结束与本片入口；
- [Q3 result](../research/m4-minimal-gyro-q3-offline-bias-alignment-result.md) 的唯一
  qualification 结论继续是 `HYPOTHESIS_FAIL / HALF_STABILITY / STOP`；本计划不重跑、补考、
  调门或推翻 Q3；
- [Q1 result](../research/m4-minimal-gyro-q1-observe-result.md) 与
  [Q2 result](../research/m4-minimal-gyro-q2-known-bias-predict-result.md) 只提供既有 provenance；
  Q2 known-bias public contract 必须保持；
- 资格阶梯服从 [evidence-gated integration](../agents/evidence-gated-integration.md)。

本片只回答：冻结的 14-pose no-eviction synthetic graph 中，online three-axis gyro bias、
rotation-only factor、RW、root prior、noise、transaction 与 lifecycle 是否按 V1 工作。`PASS`
只授权：

1. 保留 default-off、无 real caller 的 estimator-private mechanism；
2. 另写一份独立的 eviction-information-handoff design/plan。

即使 `PASS`，也**不授权** real dataset、apps wiring、GT/ATE/RPE、natural-feedback、artifact/config
schema、default-on、bounded online 长窗口、完整 `X/V/B`、旧 Q4 或“VIO 已完成”的结论。

## 1. 事务与文件边界

### 1.1 Pre-implementation exact-six authority transaction

production RED 前只允许以下六个文件处于同一个 authority commit/tree：

```text
docs/research/m4-online-gyro-bias-synthetic-design.md
docs/adr/0002-stage-gated-gyro-only-bias-state.md
docs/design/conventions.md
docs/design/roadmap.md
docs/plans/2026-08-18_m4_online_gyro_bias_synthetic_1e3569b4.plan.md
phad/estimator/README.md
```

该 commit 不得含 product code、test 或 CMake。冻结 commit/tree 后，先锁定两个不同的 canonical
reviewer identity：一个只做 fresh Standards review，一个只做 fresh Spec review；两者不得是
authority writer，也不得复用同一 review 输出。repo 外 write-once receipt 至少记录 reviewer
task/profile identity、authority commit/tree、六文件 blob/SHA-256、命令、时间和完整 findings。
两轴必须对**同一 exact bytes**均为 zero findings。任一 finding 的修补会产生新 commit/tree，并
同时作废两份旧 review；必须重新锁定 identity、双轴 fresh review。只有该 lock 完整存在，§1.2
才生效；authority review 与 production RED 不能合并成同一证据回合。

最低机械检查先覆盖 staged/untracked transition；在 `git add` exact six 后、commit 前执行：

```bash
set -euo pipefail
mode="${1:?usage: authority-check.sh staged|committed}"
authority_paths=(
  docs/research/m4-online-gyro-bias-synthetic-design.md
  docs/adr/0002-stage-gated-gyro-only-bias-state.md
  docs/design/conventions.md
  docs/design/roadmap.md
  docs/plans/2026-08-18_m4_online_gyro_bias_synthetic_1e3569b4.plan.md
  phad/estimator/README.md
)

if check_dir="$(mktemp -d)"; then
  :
else
  rc=$?
  exit "$rc"
fi
trap 'rm -rf -- "$check_dir"' EXIT
printf '%s\n' "${authority_paths[@]}" >"$check_dir/expected.unsorted"
if LC_ALL=C sort -u "$check_dir/expected.unsorted" >"$check_dir/expected"; then
  :
else
  rc=$?
  exit "$rc"
fi

check_exact_name_status() {
  local status_file="$1"
  if awk -F '\t' '
    function emit(path) { if (path == "") exit 20; print path }
    {
      kind = substr($1, 1, 1)
      if (kind !~ /^[ACMRTD]$/) exit 21
      if (kind == "C" || kind == "R") {
        if (NF != 3) exit 22
        emit($2); emit($3)
      } else {
        if (NF != 2) exit 23
        emit($2)
      }
      if (kind == "D" || kind == "T") exit 24
    }
  ' "$status_file" >"$check_dir/paths.unsorted"; then
    :
  else
    rc=$?
    exit "$rc"
  fi
  if LC_ALL=C sort -u "$check_dir/paths.unsorted" >"$check_dir/paths"; then
    :
  else
    rc=$?
    exit "$rc"
  fi
  if cmp -s "$check_dir/expected" "$check_dir/paths"; then
    :
  else
    exit 25
  fi
}

case "$mode" in
  staged)
    if git diff --cached --no-renames --name-status \
        --diff-filter=ACMRTD >"$check_dir/name-status"; then
      :
    else
      rc=$?
      exit "$rc"
    fi
    check_exact_name_status "$check_dir/name-status"
    if git diff --no-renames --quiet --; then
      :
    else
      rc=$?
      exit "$rc"
    fi
    if git ls-files --others --exclude-standard >"$check_dir/untracked"; then
      :
    else
      rc=$?
      exit "$rc"
    fi
    if [[ -s "$check_dir/untracked" ]]; then exit 26; fi
    git diff --cached --no-renames --check -- "${authority_paths[@]}"
    ;;
  committed)
    if git status --porcelain=v1 --untracked-files=all \
        >"$check_dir/status"; then
      :
    else
      rc=$?
      exit "$rc"
    fi
    if [[ -s "$check_dir/status" ]]; then exit 27; fi
    if git diff-tree --no-commit-id -r --no-renames --name-status \
        --diff-filter=ACMRTD HEAD >"$check_dir/name-status"; then
      :
    else
      rc=$?
      exit "$rc"
    fi
    check_exact_name_status "$check_dir/name-status"
    git diff-tree --no-renames --check HEAD^ HEAD -- "${authority_paths[@]}"
    git rev-parse HEAD
    git rev-parse 'HEAD^{tree}'
    ;;
  *)
    exit 64
    ;;
esac

sha256sum "${authority_paths[@]}"
git hash-object "${authority_paths[@]}"
```

脚本的 staged mode 必须 exit `0` 后才能 commit；authority commit 形成后再在 clean checkout 执行
committed mode。两种 mode 都以 `--no-renames` 检查 `A/C/M/R/T/D`，`C/R` 会验证 old/new 两个
path，`T/D` 直接失败；committed mode 的 clean status 与 exact commit diff 都是 lock 的组成部分。

```bash
set -euo pipefail
: "${PHAD_M4_AUTHORITY_CHECK:?set the frozen repo-external authority checker}"
if bash "$PHAD_M4_AUTHORITY_CHECK" committed; then
  :
else
  rc=$?
  exit "$rc"
fi
```

建议 authority commit：

```text
docs(m4): lock online gyro bias synthetic authority (#40)
```

### 1.2 Production RED→GREEN exact allowlist

authority lock 后，production/test/CMake transaction 只允许：

```text
phad/estimator/types.hpp
phad/estimator/stereo_vo_estimator.cpp
phad/estimator/gyro_rotation_predictor.cpp
phad/estimator/internal/gyro_bias_initial_value.hpp
phad/estimator/internal/stereo_vo_update_transaction.hpp
phad/estimator/internal/gyro_interval_reducer.hpp
phad/estimator/internal/gyro_interval_reducer.cpp
phad/estimator/internal/gyro_rotation_factor.hpp
phad/estimator/internal/gyro_rotation_factor.cpp
tests/estimator/stereo_vo_online_gyro_bias_test.cpp
tests/estimator/online_gyro_bias_oracle_test.cpp
CMakeLists.txt
```

production implementation 与 qualification 期间，§1.1 exact-six、Q3 全部文件及以下路径只读：

```text
apps/**
phad/sync/**
phad/sensor/**
phad/frontend/**
phad/eval/**
phad/bench/**
docs/**
```

不得新建 product library、runner、app、sidecar、长期实验 target 或 feature switch；不得修改
`diag.csv`、`summary.json`、`meta.json`、`est.tum`、`kf.tum` schema 或 config hash 输入；不得运行
real、GT、ATE、RPE、Q3 或 full-unit qualification。

### 1.3 Post-verdict bookkeeping allowlist

只有 independent verifier 已形成 §9 verdict，才另开不含 product code 的 transaction：

```text
docs/research/m4-online-gyro-bias-synthetic-result.md
docs/plans/2026-08-18_m4_online_gyro_bias_synthetic_1e3569b4.plan.md
docs/design/roadmap.md
```

result 新建并完整保留 receipt；本计划只更新 todo 状态与 evidence link；roadmap 只写 verdict 与下一
权限。design、ADR、conventions、`phad/estimator/README.md` 与 Q3 继续只读。所有 verdict 都要写
结果，但只有 `PASS` 可另开
eviction-information-handoff authority slice；它不能混进本片 result commit。

## 2. 冻结 interface 与 ownership skeleton

### 2.1 Public value types；唯一 update seam

只在 [`types.hpp`](../../phad/estimator/types.hpp) 增加 design 冻结的项目自有 POD；
[`StereoVoEstimator::update`](../../phad/estimator/stereo_vo_estimator.hpp) 保持唯一更新入口，header
不修改，也不暴露 GTSAM type：

```cpp
struct GyroInterval
{
  common::Timestamp                    m_t_prev;
  std::vector<sensor::ImuMeasurement> m_samples;
  bool                                 m_imu_gap = false;
};

struct GyroBiasOptions
{
  double          m_gyr_nd            = 0.0;
  double          m_gyr_rw            = 0.0;
  Eigen::Vector3d m_prior_mean_radps  = Eigen::Vector3d::Zero();
  double          m_prior_sigma_radps = 0.0;
};

enum class GyroBreakReason : std::uint8_t
{
  kNone = 0,
  kMissingInterval,
  kDeclaredGap,
  kRejectedEndpoint,
  kEvictedEndpoint,
  kSegmentChange
};

struct GyroWindowBias
{
  std::uint64_t                  m_frame_index = 0;
  common::Timestamp              m_timestamp;
  std::optional<Eigen::Vector3d> m_bias_radps;
};

struct GyroDiagnostics
{
  std::optional<Eigen::Vector3d> m_bias_radps;
  std::vector<GyroWindowBias>    m_window_biases;
  std::uint32_t m_rotation_factors       = 0;
  std::uint32_t m_rw_factors             = 0;
  std::uint32_t m_root_priors            = 0;
  std::uint32_t m_relinearization_rounds = 0;
  GyroBreakReason m_break_reason = GyroBreakReason::kNone;
};
```

并按 design 在现有 aggregates 末尾增加（下列省略已有字段）：

```cpp
struct KeyframeMeasurement
{
  // existing fields unchanged
  std::optional<GyroInterval> m_gyro_interval;
};

struct EstimatorOptions
{
  // existing fields unchanged
  std::optional<GyroBiasOptions> m_gyro_bias;
};

struct UpdateDiagnostics
{
  // existing fields unchanged
  std::optional<GyroDiagnostics> m_gyro;
};
```

`EstimatorOptions::m_gyro_bias == nullopt` 是唯一 off 语义。off 不得读取/验证 interval presence 或
bytes，旧视觉 status/message/estimate/diagnostics 的 double bit pattern 必须不变，`m_gyro` 必须
null。on 时 `m_gyro` present；`m_window_biases` 每个实际 `G(k)` 恰一项，按 frame index 严增并绑定
同一 timestamp。prior-only component 的 optional bias 为 null；含 rotation observation 的
component、其 root 与 RW-only terminal 才 present。成功 scalar 等于 vector back。
`m_relinearization_rounds` 只对最终 `kOk` 表示实际 extra count（包括内部成功/fallback 后最终
`kOk`）；它不是 solver trace。任何最终 `kRejected` / `kFailed` 都报 `0`，即使 cap path
已执行 extra round。hard reject/fail scalar null、rounds `0`、reason none，而 vector/counts 必须在
rollback 后从 committed topology materialize；diagnostics 不进入 transaction state。

该 optional POD 只允许 test code 在内存直接激活本 synthetic 协议；不得新增 parser、
`flattenConfig()`、`config_hash`、persistent artifact、apps/session 或 real caller。所有 GTSAM
key/type、PIM、factor 与 lifecycle 继续只在 `StereoVoEstimator::Impl`/private files 内存在。

### 2.2 Internal seams；不新增第二套 estimator 或 reducer

private headers 均位于 `phad::estimator::internal`：

```cpp
enum class GyroIntervalErrorCode
{
  kTooFewSamples,
  kEndpointMismatch,
  kNonIncreasingTimestamp,
  kNonFiniteGyro,
  kTimestampOverflow,
  kInvalidNoiseScale
};

struct ValidatedGyroInterval;  // immutable raw samples、exact duration、endpoint provenance

class GyroRotationFactor final
  : public gtsam::NoiseModelFactorN<gtsam::Pose3,
                                    gtsam::Pose3,
                                    gtsam::Vector3>
{
  // compose official AHRSFactor; implement clone/evaluateError
};
```

另新增 header-only `gyro_bias_initial_value.hpp`，exact symbols/behavior 服从 design §3.1.1；它不进入
CMake source list，不依赖 GTSAM，也不暴露到 public header。

reducer 的具体 result carrier 沿用项目 `std::variant<value,error>` 风格，但职责必须只有一份：checked
integer-ns timestamp/duration、finite validation、endpoint trapezoid、seconds conversion、expected
endpoint/noise derivation。Q2 public `integrateGyroRotation()` 与 online private PIM 都必须调用这个
reducer；不得复制 endpoint average 或 `dt` 算法。Q2 public header/error contract 不改。

`GyroRotationFactor` 必须继承 canonical `NoiseModelFactorN<Pose3,Pose3,Vector3>`、内部组合官方
`AHRSFactor`、实现 `clone()`，并把 `Pose3::rotation()` derivative 链式提升为 `[H_R,0]`；translation
残差/Jacobian exact zero。每轮用 raw samples 和固定 `biasHat` 新建
`PreintegratedAhrsMeasurements`，通过 `integrateMeasurement()` 形成
`Sigma_R=m_gyr_nd^2*dt`；`evaluateError()` 只使用 cached PIM 的一阶 bias correction，严禁重积分。

[`StereoVoEstimator::Impl`](../../phad/estimator/stereo_vo_estimator.cpp) 独占：

- `G(k)=gtsam::Symbol('g',frame_index)`，每个当前 `X(k)` 恰一个 Vector3；
- 每个 `WindowFrame` 的 committed/provisional bias、segment、显式 predecessor frame index、
  validated raw interval 与 component/root metadata；
- rotation factor、`BetweenFactor<Vector3>` RW、component root prior、graph rebuild、LM、writeback
  与 rollback；
- `constexpr ... kMaxFixedPimRounds = 3` 及真正控制 outer loop 的有界 condition。

staging 新 `G_j` 时，exact link 必须继承 predecessor 最近的 update-local provisional bias（若本
transaction 尚无 provisional 则用 committed bias）；任何新 component/root 则精确使用 protocol
`m_prior_mean_radps`。Q3 nuisance、evicted point estimate 与旧 posterior 禁止作为初值。每个 fixed-PIM
round 的 `Values[G_i]` 与对应 PIM `biasHat_i` 必须来自同一 round-start snapshot 并逐轴一致。

exact-link inherit / new-component prior 选择只实现在 header-only private seam
`internal/gyro_bias_initial_value.hpp`。它冻结 design §3.1.1 的
`GyroBiasInitialKind{kExactLink,kComponentRoot}`、
`GyroBiasInitialErrorCode{kMissingPredecessor,kNonFiniteSelectedValue}`、
`variant<Vector3d,error>` result 与 `selectGyroBiasInitialValue(kind,provisional,committed,prior)`。
exact link 按 provisional→committed 选择、缺失/selected non-finite hard error；component root 只返回
finite prior 并忽略 predecessor。signature 不接受 truth/Q3/evicted posterior。bit-exact distinct
nonzero fixtures 精确使用 design 的 provisional `[0.03125,-0.0625,0.125]`、committed
`[-0.25,0.5,-1.0]` 与 prior `[1.5,-2.0,0.75]`，覆盖全部分支；graph test 另验
`Values`/`biasHat` snapshot 一致，PIMPL 禁止复制逻辑。

本片新增或迁出的所有 C++ `struct` / `class` data member 统一按
[`cpp-naming.md`](../agents/cpp-naming.md) 命名为 `m_` + snake_case，transform 使用
`m_T_target_source`。该规则包括 public 新 POD、现有 aggregate 中新增的
`m_gyro_interval` / `m_gyro_bias` / `m_gyro`、reducer/result carrier、initializer/factor private state、
`ValidatedGyroInterval`、`GyroFrameState`、迁出的 `WindowFrame`、`StereoVoUpdateState` 与
transaction `m_owner` / `m_before`。方法与参数保持现有规则；既有
`KeyframeMeasurement::timestamp/observations`、`EstimatorOptions` 旧字段与 `UpdateDiagnostics` 旧字段
不在本片重命名。

另新增 header-only `internal/stereo_vo_update_transaction.hpp`，exact types/interface 服从 design
§3.1.2。它迁出 `WindowFrame`，新增 `GyroFrameState`，并把 `m_window`、`m_landmarks_w`、
`m_track_times`、`m_T_W_B_last_stereo`、`m_T_W_B_last_accepted`、`m_T_W_B_prev_accepted`、
`m_next_frame_index`、`m_initialized`、`m_segment_id`、`m_culled_ids`、`m_pending_seed_obs`、
`m_eligible_visual_rejected_timestamp` 与 `m_next_gyro_component_id` 全部聚合到
`StereoVoUpdateState`。`GyroFrameState` 使用 `m_bias_radps`、`m_segment_id`、`m_component_id`、
`m_predecessor_frame_index`、`m_interval`；`WindowFrame` 使用 `m_frame_index`、`m_timestamp`、
`m_T_W_B`、`m_observations`、`m_is_keyframe`、`m_gyro`。`Impl` 只以
`unique_ptr<StereoVoUpdateState> m_state` 持有这份 mutable ownership；
diagnostics 不进 State，而在 rollback 后从恢复 topology materialize。

`StereoVoUpdateTransaction(unique_ptr<StereoVoUpdateState>&)` 只 deep-copy staging-entry committed
state 到 backup，production 继续原地修改原 owner；constructor 不得 swap copy 成 live owner，以免成功/off path 因
`unordered_map` copy 改变 iteration/bucket 表示。rollback/destructor 以 `unique_ptr::swap` + reset
恢复，commit 只 reset backup；copy/move 删除，rollback/commit/destructor 均 `noexcept` 且幂等。
任何 state iterator/reference/pointer 不得跨 rollback。正常 gyro on/off 成功 update 均保持
`m_state.get()` 的进入地址 identity。

constructor 位于 basic/gyro validation、visual support 与 intentional pre-staging pending/provenance
之后、seed/reanchor/normal 三个 staging branch 之前，且不在 gyro/keyframe guard 内。post-staging hard
return 禁止分散构造 result，全部调用唯一 private/local
`finalizePostStagingHardResult(...)`。该 finalizer 内必须严格“explicit `transaction.rollback()` → 从
恢复后 `*m_state` materialize result 与 `m_gyro` topology diagnostics → return”，且不得跨 rollback
复用 state 内部 reference/pointer/iterator。exception unwind 由 destructor rollback；唯一成功 return 在
result 完整形成后调用一次 commit。production 放置 design 冻结的 constructor、
rollback-before-diagnostics 和 commit 三个 exact source marker。

同一 graph 禁止并行 `Rot3` state；factor 只约束 Pose3 rotation，不增加 translation constraint。

### 2.3 CMake 变化

[`CMakeLists.txt`](../../CMakeLists.txt) 只允许：

1. 将 `gyro_interval_reducer.cpp` 与 `gyro_rotation_factor.cpp` 加入既有 `phad_estimator`；
2. 将 `stereo_vo_online_gyro_bias_test.cpp` 加入既有 `phad_estimator_tests`；
3. 新建 test-only `phad_online_gyro_bias_oracle_tests`，source 只有
   `online_gyro_bias_oracle_test.cpp`，只链接 `Eigen3::Eigen` 与 `GTest::gtest_main`，注册 unit；
4. GTSAM 对 `phad_estimator` 继续 `PRIVATE`，不手写裸 `-lgtsam`。

两个 header-only private seams 不增加 CMake source/target；transaction direct tests 复用同一 product
test source 与既有 `phad_estimator_tests`。

oracle source 不 include `phad::estimator`、GTSAM、private factor 或 reducer，也不与 product test
共享 literals/helper。product test 只消费 design 冻结 literals。

## 3. Todo `authority-lock`：docs commit 与 fresh 双审

1. 在不含 product/test/CMake diff 的 branch 上检查 §1.1 exact-six list；确认 design SHA/blob
   等于 §0。
2. 将 exact six 作为一个 authority commit；记录 commit/tree 和六文件逐一 blob/SHA-256 到 repo 外
   write-once receipt。
3. 在 review 开始前冻结不同的 Standards/Spec reviewer identities。两位 reviewer 均从该 commit
   fresh checkout、完整读取适用 AGENTS 与各自 review skill，只读审同一 tree。
4. 任一轴有 finding：保持 implementation STOP，writer 只修 exact-six allowlist，形成新 identity，
   两轴全部重来。只有双轴 zero findings 才写 `authority_lock=PASS` receipt。
5. 从 production branch 的 first parent 明确指向 authority commit；后续生产阶段禁止改 exact-six
   authority files。

验收：authority commit/tree、六文件 identity、两位 reviewer identity、两份 zero-finding output
可互相核对；否则后续任何 RED/GREEN 都无资格权重。

## 4. Todo `red`：唯一 public capability RED

authority lock PASS 后、任何 product declaration 前，先在 repo 与全部 worktree 之外创建 §9 使用的
external verifier script。冻结 script version/source SHA-256、exact invocation、interpreter/tool
versions 和 read-only mode；脚本必须先通过 §9 的 canonical-realpath/worktree exclusion。脚本后续
负责 test marker region byte extraction、allowlist、production transaction
constructor/failure-finalizer/commit/order/guard 与
`Impl` ownership static checks、archive 与 mutation arms，任何 byte 漂移都使证据失效。preflight 前
先创建并保留 evidence、空 positive-archive 与 mutation-scratch-root 三个 canonical
目录，使 §9 的 `realpath -e` 检查在 RED 前可真实执行；archive 只在 clean GREEN 复制完成后转只读。

第一刀只新建 `tests/estimator/stereo_vo_online_gyro_bias_test.cpp` 及其 existing-target CMake entry，
且只加入以下 marker-delimited helper/control region：

```text
// PHAD_M4_ONLINE_BIAS_AUTHORITY_CONTROL_BEGIN
StereoVoOnlineGyroBias.AuthorityBaselineVisualControl
// PHAD_M4_ONLINE_BIAS_AUTHORITY_CONTROL_END
```

该 region 只使用 authority commit 上已经存在的 old public API，复用 design §5 的 calibration、
20-point visual geometry、14-state visual schedule 与旧 options；不得引用任何 online-bias declaration。
冻结 helper 生成 frame 0..13 status/message/pose/全部旧 diagnostics 的 canonical bytes，并发布
SHA-256。先使这个 control GREEN，再记录 control region SHA-256、baseline commit/tree/test blob、
exact commands/output/digest。

随后才追加第二个 marker region；唯一初始 capability RED 为：

```text
// PHAD_M4_ONLINE_BIAS_NO_EVICTION_BEGIN
StereoVoOnlineGyroBias.NoEvictionFourteenPoseRecovery
// PHAD_M4_ONLINE_BIAS_NO_EVICTION_END
```

测试必须经 public `StereoVoEstimator::update()`，使用 design §5 的完整 calibration、20-point
landmarks/projections/options、14 poses、13 intervals、`K,K,N×12`、`window_size=14` 和
`stereo_sigma_px=0.003`；不得直接注入 pose、`NonlinearEquality`、private state、prior fake 或
identity fallback。它一次性断言 public recovery 的 observable outputs：14 个 ordered
`m_window_biases`、truth error `<=5e-4 rad/s`、frame1→frame13 signed-axis/L2 drift ratio
`[0.85,1.05]`、counts `14/13/13/1`、全程无 eviction、round vector
`[0,1,0,0,0,0,0,0,0,0,0,0,0,0]`，且 current scalar 等于 vector back。

测试必须保存并断言全部 14 个 prefix actual，而不是只验 final：prefix `i=0..13` 分别满足
`N_G=i+1`、`N_rotation=i`、`N_rw=i`、`N_root=1`、旧 diagnostics `window_size=i+1`、
`m_window_biases.size=i+1`，其中每项 `j=0..i` 的 `m_frame_index/m_timestamp` 精确为
`j/t_j`，当前 `m_break_reason=kNone`。prefix 0 的唯一 bias entry 与 scalar 为 null；所有 `i>=1` 的 entries 全部
present/finite，scalar 等于 back。每个 prefix 无 eviction；per-update extra-round actual 逐项形成上述
vector，final frame 13 必须为 `0`，不能把累计值写回 final diagnostics。

首次运行前冻结 NoEviction region SHA-256；RED receipt 记录 authority identity、external verifier
identity、authority-baseline bytes/digest/region identity、NoEviction region identity、RED
commit/tree/test blob、cwd/compiler/configure/build/test exact commands、完整 stdout/stderr、exit、first
diagnostic 和 staged/committed diff/allowlist。有效 RED 必须明确来自 online-bias capability 尚不存在；
路径、依赖、语法或 harness 错误是 `HARD_ERROR`。为制造 RED 不得先加空实现或 prior-mean 假成功。

后续 lifecycle tests 只能加在两个 marker regions 之外。最终 verifier 逐字抽取两段并与 historical
SHA/blob 对拍；整文件会按计划增长，所以 RED 时的整文件 SHA/blob 只作 historical provenance，
不得要求最终整文件 SHA 与它相等。

建议两个分离的 commit：

```text
test(m4): freeze online gyro bias visual control (#40)
test(m4): preregister online gyro bias recovery RED (#40)
```

## 5. Todo `oracle`：Eigen-only independent oracle

在 public RED 已留证但 product GREEN 尚未开始时，新建 oracle source/target。它独立构造 dense
linear system并复算 design §6 中的：

- 完整 `G0..G13`，不是只算端点；
- rotation/RW whitening vector、matrix、individual/total cost；
- Between-only、prior-only、observed systems 的 rank/nullity、singular values、
  `condition(H)` 与 `condition(H^T H)`；
- design frozen exact literals、tolerances 与 rank tolerance。

comparison tolerance 同 design 冻结：`G0..G13` 每个 coefficient absolute
`<=1e-12 rad/s`；列出的 singular values 与两个 condition 同时满足 absolute `<=1e-9` 且 relative
`<=1e-10`；rank 只用 design 的 `sigma_max*max(m,n)*epsilon_double`。这些是跨 Eigen/libm double
复算预算，不是 public recovery science gate。

oracle 不调用未来 DUT、不读取 product actual、不读取 real/GT/ATE/RPE。若它不能重现 frozen
literals，判 protocol/harness failure 并 `STOP`，不能把 DUT 输出回填 expected 或放宽门。

定向命令：

```bash
cmake -S . -B build-m4-online-bias \
  -DPHAD_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build-m4-online-bias \
  --target phad_online_gyro_bias_oracle_tests -j2 --verbose
build-m4-online-bias/phad_online_gyro_bias_oracle_tests
if rg -n 'phad::estimator|phad/estimator|gtsam' \
    tests/estimator/online_gyro_bias_oracle_test.cpp; then
  exit 1
else
  rg_exit=$?
  test "$rg_exit" -eq 1
fi
```

最后一个 `if` 只在 `rg` 明确以 `1` 表示无匹配时整体 exit `0`，此时才是 PASS；`rg` error `2+`
不得伪装为 independence。另审 `CMakeLists.txt` 的 oracle link stanza 只有 Eigen/GTest。记录 binary、
stdout/stderr 与 digest。建议 commit：

```text
test(m4): add independent online gyro bias oracle (#40)
```

## 6. Todo `reducer-factor`：唯一 reducer 与 private math

每个子门先在同一个 product test source 中加入失败断言，再做最小 GREEN；这些是 private numerical
seam 的局部 TDD，不新增第二个 public capability RED receipt。

### 6.1 API declarations 与唯一 reducer

public RED 留证后，先加入 §2.1 的 pure declarations，使 public test 从 missing-type compile RED
前进为 behavior RED；不得让 estimator 返回 prior mean。随后实现唯一 internal reducer，并让
`gyro_rotation_predictor.cpp` 改用它。

必须闭合：少于两点、expected endpoints、duplicate/reverse、non-finite、int64 subtraction/total
duration overflow、derived noise scale、endpoint trapezoid、exact integer-ns duration，以及组合错误
优先级。Q2 `GyroRotationPredictor.*` 全部保持原 contract 与结果；不修改 predictor public header。

### 6.2 Private factor、PIM domain、noise 与 mutants

实现 §2.2 factor，并依 design frozen fixtures 闭合：

- official AHRS residual direction；correct residual `<=1e-12 rad`；
- Pose3 analytic Jacobian central-difference `<=1e-6`，translation blocks exact zero；
- wrong bias sign、omit bias、rad/deg 双向、visual inverse、left-compose、wrong frame 七个 mutant
  均达到 design 的 `>=0.02 rad` hard separation；
- 41-sample noncommuting fixture 在 `||delta_bias||_inf<=1e-3 rad/s` 的 9261 lattice 上，cached
  first-order 对 fresh full reintegration geodesic error `<=3e-8 rad`；27 points bias Jacobian
  `<=1e-7`；
- `Sigma_R=m_gyr_nd^2*dt`、`Q_b=m_gyr_rw^2*dt`，whitened/cost total exact gate `2.03`；杀死 density
  当 covariance、漏平方、dt 除法、variance 当 sigma 四个 mutant；
- factor evaluation 内 PIM 固定；candidate 超域只能由 outer relinearization 处理。

定向命令：

```bash
cmake --build build-m4-online-bias --target phad_estimator_tests -j2
build-m4-online-bias/phad_estimator_tests \
  --gtest_filter='GyroBiasInitialValue.*:GyroIntervalReducer.*:GyroRotationFactor.*'
build-m4-online-bias/phad_estimator_tests \
  --gtest_filter='GyroRotationPredictor.*'
```

建议 commit：

```text
feat(estimator): add gyro interval reducer and rotation factor (#40)
```

## 7. Todo `estimator-state`：G/RW/root、14-state GREEN 与 writeback

在 `StereoVoEstimator::Impl` 中按 design 实现 transient rebuilt forest：每个现存 `X(k)` 恰一个
`G(k)`；每条有效 exact interval 同时加入一条 rotation factor 和一条 RW factor；每个 component
恰一个 fixed protocol root prior。每次成功 rebuild 必须满足：

```text
N_G == N_X
N_rotation == N_rw
N_G == N_rotation + N_root_components
```

counts 是 current rebuilt graph，不是累计数。主门是严格 no-eviction：14-state
`K,K,N×12`、window 14，final `14 G / 13 rotation / 13 RW / 1 root`。private fixed-pose graph 对
independent `G0..G13` 的 max coefficient error `<=1e-8 rad/s`；public joint visual estimator 不得
套用这个 exact-pose 门，而按 §4 的 `<=5e-4` 与 drift ratio 门。

fixed-PIM solve lifecycle：

1. staging 新 `G_j` 时，exact link 继承 predecessor 最近 update-local provisional/committed bias；新
   component/root 精确使用 protocol prior mean；禁止 Q3 nuisance、evicted point 或 gyro truth 初值；
2. 本 update first solve initial round 从 visual-staging-entry committed `G/X/L` 与 committed `G_i`
   `biasHat` 开始；每个 round 的 `Values[G_i]` 与对应 PIM `biasHat_i` 必须来自同一 round-start
   snapshot；
3. round 后若任一 `||G_opt-biasHat||_inf>1e-3`，不 commit，从 raw samples 以 provisional candidate
   fresh rebuild 全部相关 PIM，再从上一 provisional `G/X/L` solve；
4. 总 fixed-PIM rounds 最多 3（initial + 最多 2 extra）；仅对最终 `kOk`，
   `m_relinearization_rounds` 计实际额外 rebuild+solve 次数并对本 update 所有 graph solves
   求和（包含内部成功/fallback 后最终 `kOk`）；最终 `kRejected` / `kFailed` 总是 `0`，
   即使 cap 前执行过 extra round，也不将此 result field 冒充 actual trace；
5. preflight 的 frame 1 initial delta 与 fresh-PIM 后打印 exact zero 只作 rationale，不是 public
   bit-exact gate，也不新增 per-round trace；hard gate 是所有 final factor max-axis delta
   `<=1e-3`，且 per-update vector 精确为
   `[0,1,0,0,0,0,0,0,0,0,0,0,0,0]`，所以 frame 1 diagnostics `==1`、final frame 13
   `==0`，不得按进程累计；
6. final `kOk` 才一次性写回。cap exhaustion 映射 private
   `GyroSolveErrorCode::kRelinearizationLimit`、public `kRejected`、stable message
   `gyro bias relinearization did not converge`，并完整 rollback；V1 不伪造动态 cap fixture。

同时闭合 `m_window_biases` 的 per-component nullability、ordered frame/timestamp、scalar-back equality，
以及 `G13` 只有 RW prediction 仍可 reportable 的合同。

在实现 graph build 前，先让 `GyroBiasInitialValue.*` 的 distinct-nonzero/缺失/non-finite fixtures 与
“交换 exact-link/root selected source” mutant 形成 RED，再以最小 header-only 实现 GREEN；不得另造
第二套 topology state machine。随后 graph tests 才接入 selector 并检查 Values/PIM snapshot 一致。

替换现有手工 backup/restore 前，同样先在 frozen marker regions 外加入 design §3.1.2 的四个
`StereoVoUpdateTransaction.*` direct tests，记录 transaction header/type 尚不存在或行为未实现的局部
RED；再迁出 production `WindowFrame`/State、实现 RAII 并转 GREEN，最后才让 `update()` 使用该 seam。
不得先写 generic guard 再用 surrogate state 补测试。

定向命令：

```bash
cmake --build build-m4-online-bias --target phad_estimator_tests -j2
build-m4-online-bias/phad_estimator_tests \
  --gtest_filter='StereoVoUpdateTransaction.*:OnlineGyroBiasFixedPose.*:StereoVoOnlineGyroBias.NoEvictionFourteenPoseRecovery'
```

建议 commit：

```text
feat(estimator): add default-off online gyro bias state (#40)
```

## 8. Todo `lifecycle-gates`：error precedence、断链与回归

### 8.1 Validation 与 public error precedence

执行顺序不得交换：

1. off 完全忽略 gyro；
2. on 先跑既有 visual basic syntax/timestamp validation；
3. non-gap provided interval 按下列 private precedence structural validate；
4. 再跑 visual support/backprojection/gating；
5. visual accepted 才 stage topology、joint LM，并仅在 final `kOk` commit。

```text
kTooFewSamples
  > kEndpointMismatch
  > kNonIncreasingTimestamp
  > kNonFiniteGyro
  > kTimestampOverflow
  > kInvalidNoiseScale
```

stable messages、constructor options validation 与 `kFailed` 原因传播完全服从 design §4.4。
`m_imu_gap=true` 不得读取 payload。hard reject/fail 的 `m_break_reason=kNone`；break enum 只描述 valid
accepted topology。

只保留一个 `m_eligible_visual_rejected_timestamp`：basic visual 与 gyro structure 均通过、随后明确在
visual support/backprojection stage `kRejected` 才登记；malformed gyro、optimizer failure、bias cap
不得登记；accepted topology classification 后清除。

### 8.2 Topology precedence 与 eviction 安全边界

accepted staged state 按固定顺序分类：

```text
segment change
  > missing interval
  > declared gap
  > exact predecessor (survives: attach; evicted: break/reset)
  > latest eligible visual rejected endpoint
  > unknown endpoint mismatch (hard reject)
```

首个 accepted absent/gap 是 `kNone` root；首帧 non-gap 是 endpoint hard reject。非首帧
missing/gap/rejected endpoint/segment change/evicted endpoint 均只断链并建独立 synthetic root，
绝不跨过 endpoint 猜连。interior non-KF eviction 只资格化 `K0,K1,N2,N3`、window 3 的负向安全：
N2 被逐出后保留 K0→K1、删除 N2→N3、禁止 K1→N3，surviving census 为
`3 G / 1 rotation / 1 RW / 2 roots`、reason `kEvictedEndpoint`。

这不授权 eviction 后 mean/information continuity；禁止 point-only prior 冒充 handoff。真正的
eviction information/correlation/marginal factor 是 `PASS` 后下一片。

### 8.3 必过 lifecycle matrix

在同一 product test source 中覆盖：

- continuous 3-state exact census；
- rejected endpoint 后 two roots，下一 exact 只从当前 state 起链；
- missing、declared gap、segment change、interior eviction 的 reason/forest；
- malformed/combined precedence、gap payload ignore、下一 exact update 证明零污染；
- `StereoVoOnlineGyroBias.UnknownPredecessorRollsBackBeforeDiagnostics`：给 visually supported `t1`
  传 structurally valid non-gap interval，其 `m_t_prev` 既不是 committed `t0` predecessor 也不是
  eligible rejected provenance；topology classification 后必须 `kRejected` / stable message
  `gyro interval endpoints do not match pose timestamps`，scalar null、rounds `0`、
  reason none，`m_window_biases.size()==1` 且唯一 entry 是 frame 0 / `t0` / null，
  `m_rotation_factors=0`、`m_rw_factors=0`、`m_root_priors=1`，exact 等于 rollback 后只含
  `t0` root 的 committed topology。随后 `t0→t2` exact update 与 fresh control 的
  status/message/estimate/旧 diagnostics/`m_gyro`
  全字段 bit-exact，证明零污染。该门必须走唯一 failure-finalizer，动态证明 explicit
  rollback 先于 diagnostics materialization；
- reanchor 清旧 bias/link/root，新 segment 独立 root；
- initial LM 与 successful outlier reopt 从正确 provisional rebuild，不重复计 factors；
- failed reopt 回上一成功 provisional；
- production `StereoVoUpdateState` / RAII seam 的 scope-exit、exception-unwind、explicit rollback、
  commit 四个 direct gates，逐字段恢复或发布 visual-staging-entry state，并按下文检查
  owner address；
- 正常 KF/non-KF 均经过同一 transaction constructor/commit 与 14-prefix 状态推进；production
  activation/order 另由 §9 source/static arm 证明；
- constant-bias zero-drift、exact-zero；non-identity extrinsic 独立满足同一 body-truth/drift/count/
  lifecycle 门，不与 identity posterior 作逐值或 `1e-8` 比较；test-local `R_B_left`/inverse frame
  mutants 必须跨过 body-truth 门；
- 16 个 fresh instances 的全最少输出 bit determinism；
- off 的 absent/valid/malformed/NaN/Inf/overflow 五组，都用冻结 encoder 对拍 RED 前
  authority-baseline canonical bytes/digest；只在同一新 build 内互比不够。gyro diagnostics 必须 null，
  两个 marker regions 必须与 historical SHA/blob 逐字相同。

不再使用“无 stereo factor、translation 欠约束”触发 public LM exception：GTSAM LM damping 与
`tryLambda()` 使该路径不可稳定触达。不得增加 mock、public failure hook、test-only solver switch，
也不得声称取得 public outer optimizer failure 动态覆盖。

四个 direct tests 精确命名为
`StereoVoUpdateTransaction.ScopeExitRestoresEveryField`、
`StereoVoUpdateTransaction.ExceptionUnwindRestoresEveryField`、
`StereoVoUpdateTransaction.ExplicitRollbackRestoresEveryField` 与
`StereoVoUpdateTransaction.CommitPublishesEveryField`。每门的 before/after 都是完整、nonempty、
非平凡 state，并对 `m_window`、`m_landmarks_w`、`m_track_times`、
`m_T_W_B_last_stereo`、`m_T_W_B_last_accepted`、`m_T_W_B_prev_accepted`、`m_next_frame_index`、
`m_initialized`、`m_segment_id`、`m_culled_ids`、`m_pending_seed_obs`、
`m_eligible_visual_rejected_timestamp` 与 `m_next_gyro_component_id` 逐项使用不同值；bool
翻转，optional 覆盖 semantic presence/null。至少两个 window frame 逐项覆盖
`m_frame_index`、`m_timestamp`、`m_T_W_B`、`m_observations`、`m_is_keyframe`、`m_gyro`；nested
gyro 逐项覆盖 `m_bias_radps`、`m_segment_id`、`m_component_id`、
`m_predecessor_frame_index`、root/exact-link 与 `m_interval` 的 raw samples/duration/provenance。

每门 constructor 前保存 `owner_before = owner.get()`，constructor 后断言
`owner.get() == owner_before`。前 3 门逐字段 exact 恢复，并在 rollback/destructor 后断言
`owner.get() != owner_before`：此时 owner 必须指向 snapshot-owned 对象，不错误要求回到已销毁
原地址。commit 门逐字段保留 mutation，并在 commit 后断言
`owner.get() == owner_before`。diagnostics 不进入 state；hard result 只能由
`finalizePostStagingHardResult(...)` 在 explicit rollback 后从恢复 topology 重新 materialize，且不得
跨 rollback 复用 iterator/reference/pointer。

visual lifecycle 复用既有触发机制，不复制一套假 gate；receipt 锁定 design 指定的三份 source blob。
transaction tests 直接 include production private header，不另造 generic surrogate：

- [`keyframe_update_test.cpp`](../../tests/estimator/keyframe_update_test.cpp) low-shared fixture，
  blob `73274733a90448956090350cf0a4a086b0c29c29`；
- [`stereo_vo_reanchor_test.cpp`](../../tests/estimator/stereo_vo_reanchor_test.cpp) reanchor/reject
  rollback fixture，blob `f6ef740f62e7dec2bc6da62d82b25bd3bce1ec3f`；
- [`stereo_vo_outlier_reopt_test.cpp`](../../tests/estimator/stereo_vo_outlier_reopt_test.cpp)
  successful reopt/LM2 fallback fixture，blob
  `4bd1e1c7f36425817f73d3923c7829a1544f8c7e`。

定向命令：

```bash
cmake --build build-m4-online-bias --target phad_estimator_tests -j2
build-m4-online-bias/phad_estimator_tests \
  --gtest_filter='StereoVoOnlineGyroBias.*:StereoVoUpdateTransaction.*'
build-m4-online-bias/phad_estimator_tests \
  --gtest_filter='StereoVoOnlineGyroBias.UnknownPredecessorRollsBackBeforeDiagnostics'
build-m4-online-bias/phad_estimator_tests \
  --gtest_filter='KeyframeUpdateTest.NonKeyframeRejectedOnLowShared:StereoVoReanchor.RecoversAfterLandmarkIdTurnover:StereoVoReanchor.SeedGateRejectsWithoutPoisoningState:StereoVoOutlierReoptTest.ReoptsWhenAtLeastFourCulled:StereoVoOutlierReoptTest.Lm2FailureFallsBackToLm1Cull'
```

建议 commit：

```text
test(estimator): qualify online gyro bias lifecycle (#40)
```

## 9. Todo `activation-deletion`：repo-external mechanism/transaction/numerical-mutation proof

该门只在原始 public RED 与 clean GREEN 已留证后运行，不能替代 public capability RED。先把 clean
GREEN exact source tree、commands、logs、test binaries/digests 放入 repo/worktree 外只读 archive，
记录 archive absolute path、source commit/tree 与逐文件 SHA-256。每个 arm 都从 archive fresh
restore 到新的 repo-external writable scratch，不在工作分支制造长期开关。执行者只能使用 §4 在
RED 前已冻结为 read-only 的 external verifier script；receipt 必须核对其 version/source SHA/exact
invocation 未漂移。

| arm | 唯一 mutation | 预期 |
|---|---|---|
| positive | 无 | fresh configure/build、oracle、new product gates、Q2 regression 均通过 |
| deletion A | 只删除 `gyro_interval_reducer.cpp`，保留 CMake source entry | build 因 missing production source 非零 |
| deletion B | 只删除 `gyro_rotation_factor.cpp`，保留 CMake source entry | build 因 missing production source 非零 |
| graph activation | 只移除 `StereoVoEstimator::Impl` 注册 gyro state/factor 的调用 | build 成功；14-pose public recovery 非零 |
| transaction activation | 只在 transaction ctor 后插入一次提前 `commit()`，保留末尾 commit | build 成功；source/static verifier 因 commit count/order 非零 |
| transaction constructor wrong-swap | 只在 production constructor deep-copy 后增加 `m_owner.swap(m_before)` | build 成功；四个 direct gate 的 constructor-address assertion 非零 |
| transaction rollback order | 只交换唯一 failure-finalizer 中 explicit rollback 与 gyro diagnostics materialization 的顺序 | build 成功；unknown-predecessor targeted test 与 source/static verifier 均非零 |
| static cap | 只把 frozen `kMaxFixedPimRounds=3` 或其 loop-bound 改错/移除 | repo-external source/static verifier 非零 |
| factor 1 | production source 反转 `candidate-biasHat` increment 符号 | build 成功；对应 factor test 非零 |
| factor 2 | production source 忽略 candidate、固定使用 `biasHat` | build 成功；对应 factor test 非零 |
| factor 3 | production source 把 rad 当 degree | build 成功；对应 factor test 非零 |
| factor 4 | production source 把 degree 当 rad | build 成功；对应 factor test 非零 |
| factor 5 | production source 使用 visual relative rotation inverse | build 成功；对应 factor test 非零 |
| factor 6 | production source 将 right-compose 改为 left-compose | build 成功；对应 factor test 非零 |
| frame | gyro staging 错误施加一次 `R_B_left` | build 成功；non-identity body-truth test 非零 |
| noise 1 | production source 把 gyro density 直接当 covariance | build 成功；对应 noise/whitening test 非零 |
| noise 2 | production source 漏掉 density 平方 | build 成功；对应 noise/whitening test 非零 |
| noise 3 | production source 对 `Delta t` 使用除法 | build 成功；对应 noise/whitening test 非零 |
| noise 4 | production source 把 RW variance 传给 `Diagonal::Sigmas` | build 成功；对应 noise/whitening test 非零 |
| initializer | production source 交换 exact-link 与 component-root selected value source | build 成功；`GyroBiasInitialValue.*` 非零 |

static verifier 必须同时锁定常量值 `3` 和真实 outer-loop 对该常量的有界使用；它不声称动态触达
nonconvergent optimizer cap。transaction source 另锁定 design 的 constructor/
rollback-before-diagnostics/commit 三个 markers，验证唯一 constructor 在 gyro/keyframe guard 外并支配
三个 staging branches，唯一 commit 在完整 result 后、唯一成功 `kOk` return 前；且验证
transaction region 内所有 post-staging hard branch 都调用唯一
`finalizePostStagingHardResult(...)`，禁止直接 `kRejected` / `kFailed` return，finalizer 中 explicit
rollback 必须位于从新 `*m_state` materialize diagnostics 之前。还验证 `Impl` 只有一个 mutable
`unique_ptr<StereoVoUpdateState> m_state` ownership，不残留散落 legacy fields。graph activation、3 个
transaction source arms 与 static-cap arm 的 exact one-hunk patch、source before/after SHA 写入 receipt。
这 3 个 transaction arms 不计入 numerical semantics；下表仍是 exact **12** 个 numerical-semantic
arms，每臂只允许一个 production-source hunk，并记录 exact
patch 与受影响文件 before/after SHA-256；test-local mutant separation 不能替代这些 production-source
arms。每个 mutation 验证后丢弃整个 scratch，下一 arm 从 positive archive 重建；最后一次 fresh
restore 只要求 source tree/逐文件 SHA、两个 marker region、exact command/toolchain identity 与
targeted semantic test outcome 同 positive arm；positive arm 自身 binary digest 可记录为 provenance，
但未另行冻结可复现编译映射时不得跨不同 scratch 比较 binary bytes/SHA。删 assertion、改
test/expected/threshold、留下 mutant switch 或从 mutated arm 继续叠改均为 `HARD_ERROR`。source/static
transaction evidence、四个 direct RAII field/address tests 与 unknown-predecessor public hard gate 共同闭合
production use 及 rollback-before-materialize，但不得宣称 public outer optimizer failure 已动态触达。

repo-external evidence、positive archive 与 mutation scratch root 由执行者分别提供；冻结脚本必须
以等价于以下 bash 的逻辑解析 symlink 后验证所有路径，并遍历**全部** worktree roots：

```bash
set -euo pipefail
: "${PHAD_M4_ONLINE_BIAS_VERIFIER:?set the frozen repo-external verifier script}"
: "${PHAD_M4_ONLINE_BIAS_EVIDENCE:?set an absolute repo-external evidence directory}"
: "${PHAD_M4_ONLINE_BIAS_ARCHIVE:?set an absolute positive archive directory}"
: "${PHAD_M4_ONLINE_BIAS_SCRATCH_ROOT:?set an absolute mutation scratch root}"
resolve_existing() {
  local raw="$1"
  local out_name="$2"
  local resolved
  if resolved="$(realpath -e -- "$raw")"; then
    :
  else
    rc=$?
    exit "$rc"
  fi
  printf -v "$out_name" '%s' "$resolved"
}

resolve_existing "$PHAD_M4_ONLINE_BIAS_VERIFIER" verifier_real
resolve_existing "$PHAD_M4_ONLINE_BIAS_EVIDENCE" evidence_real
resolve_existing "$PHAD_M4_ONLINE_BIAS_ARCHIVE" archive_real
resolve_existing "$PHAD_M4_ONLINE_BIAS_SCRATCH_ROOT" scratch_real

roots=("$evidence_real" "$archive_real" "$scratch_real")
for left_index in "${!roots[@]}"; do
  for right_index in "${!roots[@]}"; do
    if [[ "$left_index" == "$right_index" ]]; then continue; fi
    case "${roots[$left_index]}/" in
      "${roots[$right_index]}/"*) exit 30 ;;
    esac
  done
done

if verifier_check_dir="$(mktemp -d)"; then
  :
else
  rc=$?
  exit "$rc"
fi
trap 'rm -rf -- "$verifier_check_dir"' EXIT

check_no_writable() {
  local target="$1"
  local output="$2"
  local rc
  if find -L "$target" -perm /222 -print >"$output"; then
    :
  else
    rc=$?
    return "$rc"
  fi
  if [[ -s "$output" ]]; then return 90; fi
}

owner_write_file="$verifier_check_dir/owner-write-file"
owner_write_dir="$verifier_check_dir/owner-write-dir"
if : >"$owner_write_file"; then :; else rc=$?; exit "$rc"; fi
if mkdir "$owner_write_dir"; then :; else rc=$?; exit "$rc"; fi
if chmod 0644 "$owner_write_file"; then :; else rc=$?; exit "$rc"; fi
if chmod 0755 "$owner_write_dir"; then :; else rc=$?; exit "$rc"; fi
if check_no_writable "$owner_write_file" \
    "$verifier_check_dir/owner-write-file-result"; then
  exit 32
else
  rc=$?
  if [[ "$rc" -ne 90 ]]; then exit "$rc"; fi
fi
if check_no_writable "$owner_write_dir" \
    "$verifier_check_dir/owner-write-dir-result"; then
  exit 33
else
  rc=$?
  if [[ "$rc" -ne 90 ]]; then exit "$rc"; fi
fi

if git worktree list --porcelain >"$verifier_check_dir/worktrees"; then
  :
else
  rc=$?
  exit "$rc"
fi

worktree_count=0
while IFS= read -r worktree_line || [[ -n "$worktree_line" ]]; do
  case "$worktree_line" in
    worktree\ *)
      worktree_root="${worktree_line#worktree }"
      resolve_existing "$worktree_root" worktree_real
      worktree_count=$((worktree_count + 1))
      ;;
    *)
      continue
      ;;
  esac
  for candidate in "$verifier_real" "$evidence_real" "$archive_real" "$scratch_real"; do
    case "$candidate/" in "$worktree_real/"*) exit 1 ;; esac
  done
done <"$verifier_check_dir/worktrees"
if [[ "$worktree_count" -eq 0 ]]; then exit 31; fi

if sha256sum "$verifier_real"; then :; else rc=$?; exit "$rc"; fi
if chmod a-w "$verifier_real"; then :; else rc=$?; exit "$rc"; fi
if check_no_writable "$verifier_real" \
    "$verifier_check_dir/verifier-writable"; then
  :
else
  rc=$?
  exit "$rc"
fi
```

negative self-check 的两次期望结果都是 checker exit `90`，且 output 分别列出 mode
`0644` 文件和 mode `0755` 目录；任一 fixture 被误判为“无 writable entry”、`find` 本身失败，
或实际 verifier 仍有任一 owner/group/other write bit，均必须 fail closed 并写入 receipt。

positive archive 写完后必须执行并通过 mode-bit 检查；mutation 复制到另建 scratch 后才恢复 owner
write bit：

```bash
set -euo pipefail
if chmod -R a-w "$archive_real"; then :; else rc=$?; exit "$rc"; fi
if check_no_writable "$archive_real" \
    "$verifier_check_dir/archive-writable"; then
  :
else
  rc=$?
  exit "$rc"
fi
if arm_scratch="$(mktemp -d "$scratch_real/arm.XXXXXX")"; then
  :
else
  rc=$?
  exit "$rc"
fi
resolve_existing "$arm_scratch" arm_scratch_real
case "$arm_scratch_real/" in
  "$scratch_real/"*) ;;
  *) exit 34 ;;
esac
if cp -a "$archive_real/." "$arm_scratch_real/"; then
  :
else
  rc=$?
  exit "$rc"
fi
if chmod -R u+w "$arm_scratch_real"; then :; else rc=$?; exit "$rc"; fi
```

以上是同一冻结 script 的连续阶段，第二段复用第一段已验证的 canonical variables/functions/tmpdir；
任一 external command 的非零 status 都必须保留到 receipt 并使该 arm 非零退出。禁止 process
substitution，也禁止用空 stdout 推断命令成功。

每个 arm 前 external verifier 还必须抽取两个 frozen marker regions 并核对 historical region
SHA/commit/test blob。mutation arms 不提交到 repo；只提交 signed/digested receipt 到最终 result 文档。

## 10. Todo `verify-result`：clean independent qualification

### 10.1 独立 verifier preflight

独立 verifier 不能是 feature implementer，并必须从 clean production commit/tree 创建新 build
directory。先复核 authority、production allowlist、RED marker-region identities、fixture source
blobs、compiler/CMake/GTSAM/Eigen identity；这里的 RED identity 是两个 frozen marker-region SHA
及各自 historical
commit/test blob，不要求已增长 test file 的整文件 SHA 不变。任何漂移先判 `HARD_ERROR`，不能
“尽量跑完”。

建议 clean 命令：

```bash
cmake -S . -B build-m4-online-bias-verify \
  -DPHAD_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build-m4-online-bias-verify \
  --target phad_estimator_tests phad_online_gyro_bias_oracle_tests -j2
build-m4-online-bias-verify/phad_online_gyro_bias_oracle_tests
build-m4-online-bias-verify/phad_estimator_tests \
  --gtest_filter='StereoVoOnlineGyroBias.*:StereoVoUpdateTransaction.*:GyroBiasInitialValue.*:GyroIntervalReducer.*:GyroRotationFactor.*:OnlineGyroBiasFixedPose.*'
build-m4-online-bias-verify/phad_estimator_tests \
  --gtest_filter='GyroRotationPredictor.*'
build-m4-online-bias-verify/phad_estimator_tests
```

最后一条是受影响 estimator module regression；本协议不运行或声称 `ctest -L unit`、apps、real、
Q3、GT/ATE/RPE 或全仓测试。格式与 diff 门只覆盖实际 allowlist files：

```bash
clang-format --dry-run --Werror \
  phad/estimator/types.hpp \
  phad/estimator/gyro_rotation_predictor.cpp \
  phad/estimator/internal/gyro_bias_initial_value.hpp \
  phad/estimator/internal/stereo_vo_update_transaction.hpp \
  phad/estimator/internal/gyro_interval_reducer.hpp \
  phad/estimator/internal/gyro_interval_reducer.cpp \
  phad/estimator/internal/gyro_rotation_factor.hpp \
  phad/estimator/internal/gyro_rotation_factor.cpp \
  phad/estimator/stereo_vo_estimator.cpp \
  tests/estimator/stereo_vo_online_gyro_bias_test.cpp \
  tests/estimator/online_gyro_bias_oracle_test.cpp
set -euo pipefail
: "${PHAD_M4_AUTHORITY_COMMIT:?set the locked exact-six authority commit}"
production_paths=(
  phad/estimator/types.hpp
  phad/estimator/stereo_vo_estimator.cpp
  phad/estimator/gyro_rotation_predictor.cpp
  phad/estimator/internal/gyro_bias_initial_value.hpp
  phad/estimator/internal/stereo_vo_update_transaction.hpp
  phad/estimator/internal/gyro_interval_reducer.hpp
  phad/estimator/internal/gyro_interval_reducer.cpp
  phad/estimator/internal/gyro_rotation_factor.hpp
  phad/estimator/internal/gyro_rotation_factor.cpp
  tests/estimator/stereo_vo_online_gyro_bias_test.cpp
  tests/estimator/online_gyro_bias_oracle_test.cpp
  CMakeLists.txt
)
if final_check_dir="$(mktemp -d)"; then
  :
else
  rc=$?
  exit "$rc"
fi
trap 'rm -rf -- "$final_check_dir"' EXIT
if git status --porcelain=v1 --untracked-files=all \
    >"$final_check_dir/status"; then
  :
else
  rc=$?
  exit "$rc"
fi
if [[ -s "$final_check_dir/status" ]]; then exit 40; fi
git diff --no-renames --check "$PHAD_M4_AUTHORITY_COMMIT"...HEAD -- \
  "${production_paths[@]}"
if git diff --no-renames --name-status --diff-filter=ACMRTD \
    "$PHAD_M4_AUTHORITY_COMMIT"...HEAD \
    >"$final_check_dir/name-status"; then
  :
else
  rc=$?
  exit "$rc"
fi

check_production_path() {
  case "$1" in
    phad/estimator/types.hpp|\
    phad/estimator/stereo_vo_estimator.cpp|\
    phad/estimator/gyro_rotation_predictor.cpp|\
    phad/estimator/internal/gyro_bias_initial_value.hpp|\
    phad/estimator/internal/stereo_vo_update_transaction.hpp|\
    phad/estimator/internal/gyro_interval_reducer.hpp|\
    phad/estimator/internal/gyro_interval_reducer.cpp|\
    phad/estimator/internal/gyro_rotation_factor.hpp|\
    phad/estimator/internal/gyro_rotation_factor.cpp|\
    tests/estimator/stereo_vo_online_gyro_bias_test.cpp|\
    tests/estimator/online_gyro_bias_oracle_test.cpp|\
    CMakeLists.txt) ;;
    *) exit 41 ;;
  esac
}

while IFS=$'\t' read -r status path_a path_b extra; do
  kind="${status:0:1}"
  case "$kind" in
    A|M)
      if [[ -z "$path_a" || -n "$path_b" || -n "$extra" ]]; then exit 42; fi
      check_production_path "$path_a"
      ;;
    C|R)
      if [[ -z "$path_a" || -z "$path_b" || -n "$extra" ]]; then exit 43; fi
      check_production_path "$path_a"
      check_production_path "$path_b"
      ;;
    T|D)
      if [[ -z "$path_a" || -n "$path_b" || -n "$extra" ]]; then exit 44; fi
      check_production_path "$path_a"
      exit 45
      ;;
    *)
      exit 46
      ;;
  esac
done <"$final_check_dir/name-status"
```

clean-status、committed-range diff-check 与 exact allowlist subset check 必须全部 exit `0`；不得用裸
`git diff --check` 冒充 committed bytes 检查。所有 range checks 都用 `--no-renames`，覆盖
`A/C/M/R/T/D` 并检查 `C/R` old/new paths；本协议不允许 type-change/deletion。verifier 还须以 RED
前冻结的 external script 抽取
authority-control/NoEviction 两个 marker regions，对拍 historical SHA/commit/test blob，并让新 build
的 off canonical bytes/digest 对拍 authority baseline。随后逐门记录 threshold、actual、
PASS/FAIL，并至少发布 design §9.2 要求的完整 `G0..G13`、14-frame public vector、truth/drift/
per-update round-count vector、rank/SV/condition、whitening/cost、Jacobian、first-order/full
reintegration、test-local 与 exact 12 个 production-source numerical-semantic mutants、
zero-drift/exact-zero、16-run digest、off identity、typed precedence/provenance/counts、
unknown-predecessor rollback-before-materialize result/control digest、四个 direct transaction field/address actual、
production constructor/failure-finalizer/commit/order/guard static identity、3 个 transaction source arm exits、
owner-only-writable `0644`/`0755` negative self-check exits、fixture blobs、test/options
identity 及 §9 activation/deletion evidence。

### 10.2 Verdict 与 STOP

verdict 只能是 design §9.4 的 `PASS`、`HYPOTHESIS_FAIL`、`INCONCLUSIVE` 或 `HARD_ERROR`。任一
frozen gate fail/inconclusive、identity/allowlist/harness 错误、或为了通过需要改 truth/noise/prior/
schedule/threshold/oracle 时立即 STOP；不得继续调参、加补丁或扩大实验。以下也立即 STOP：

- Q2/private PIM 出现两套 reducer；
- factor 约束 translation、复制 Rot3 state、LM round 内换 PIM 或 evaluate 时重积分；
- 总三轮后仍超一阶 domain，或 forest invariant 不闭合；
- malformed/off path 污染 state 或读取不该读取的 gyro bytes；
- 跨接 missing/rejected/gap/segment/evicted endpoint；
- authority-baseline digest、任一 marker region 或 external verifier identity 漂移，或者 external
  path/worktree exclusion、archive read-only mode 失败；
- 任一 direct transaction field/address gate、production constructor/failure-finalizer/commit/order/guard
  static gate、unknown-predecessor rollback-before-materialize public gate，或 3 个 transaction source arms 任一未闭合；
- 任一 production-source numerical-semantic mutant 未被 targeted test 杀死；
- 需要 point-only eviction prior、apps、real config/schema、GT/ATE/RPE 或 default-on；
- 把 synthetic PASS 表述为 Q3 通过、旧 Q4 获授权或完整 VIO 完成。

### 10.3 Commit slices 与结果交接

建议保持以下可审计 slices；subject 可随实际 issue 规则微调，但不得合并 authority/RED/GREEN/
result 的证据边界：

| slice | 建议 subject | 内容边界 |
|---|---|---|
| authority | `docs(m4): lock online gyro bias synthetic authority (#40)` | 仅 §1.1 exact-six files |
| visual control | `test(m4): freeze online gyro bias visual control (#40)` | 仅 old-API control/helper marker region + existing-target CMake entry；先冻结 authority-baseline digest |
| public RED | `test(m4): preregister online gyro bias recovery RED (#40)` | 只追加 NoEviction marker region；保留有效 RED |
| oracle | `test(m4): add independent online gyro bias oracle (#40)` | oracle source/target 与 oracle evidence |
| reducer/factor | `feat(estimator): add gyro interval reducer and rotation factor (#40)` | declarations、unique reducer、Q2 reuse、private factor/PIM math |
| estimator state | `feat(estimator): add default-off online gyro bias state (#40)` | private State/RAII direct gates、Impl G/RW/root、rounds、writeback、main GREEN；authority README 保持只读 |
| lifecycle | `test(estimator): qualify online gyro bias lifecycle (#40)` | marker 外 lifecycle/private-transaction/off/determinism regressions；不扩 product scope |
| external verifier | 不提交 mutation | repo-external positive/deletion/graph-activation/3 transaction-source/static-cap/12 numerical-semantic mutant receipts |
| result | `docs(m4): record online gyro bias synthetic verdict (#40)` | 仅 result + 本计划状态/evidence link + roadmap verdict/permission |

只有所有 gate `PASS` 才可保留机制并进入下一 eviction-handoff **plan/design**；其余 verdict 只保留
negative/inconclusive/error evidence，不产生新增 production 权限。无论结果如何，都不重跑 Q3。
