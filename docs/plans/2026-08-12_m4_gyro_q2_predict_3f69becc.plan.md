---
name: M4 gyro Q2 Predict
overview: 仅实施 Q2 known-bias deterministic predict：以 GTSAM base PreintegratedRotation 验证 exact-ns、endpoint-average、bias-once、right-compose 的纯 rotation helper；不接入 production caller、visual posterior、配置或 Q3+。
todos:
  - id: freeze-fixed-point-and-contract
    content: 以 c0e214a 为 Q2 base、7427057 为 Q1 executable evidence，冻结 public API、typed errors、数学合同、allowlist、forbidden scope 与独立 oracle identity
    status: pending
  - id: write-red-oracles-and-mutants
    content: 先写正常 fixture、hard-error、off-grid/shared-endpoint 与六类 mutant rejection tests，固定 80-dps oracle 全量常量并取得 DUT 未实现时的 RED 证据
    status: pending
  - id: implement-pure-predictor
    content: 在 phad::estimator 实现无 production caller 的纯 predictor，以 checked integer ns 切段、endpoint average、GTSAM bias-once/right-compose 返回 exact duration 与有效 SO(3)
    status: pending
  - id: run-independent-unit-qualification
    content: 由独立 clean verifier 重建并逐门核验正常 upper bounds、二阶 ratio、六类 mutant lower bounds、typed errors、duration、SO(3)、deletion test 与范围审计
    status: pending
  - id: record-q2-stop-go
    content: 将命令、身份、oracle hash、全部 actual 与 PASS/FAIL/INCONCLUSIVE 记录到预定 result doc；只有 Q2 全门 PASS 才允许 Q3 plan-only
    status: pending
isProject: false
---

# M4 gyro Q2 Predict 实施计划

状态：**计划已冻结；五个 todo 全部 pending；DUT 尚未实现，Q2 尚无机制证据**

计划 ID：`3f69becc`

跟踪 issue：[#37](https://github.com/Nothand0212/phad-vio/issues/37)

权威输入：

- [Q2 known-bias deterministic predict 设计](../research/m4-minimal-gyro-q2-known-bias-predict-design.md)
- [Q1 Observe 实施计划](2026-08-12_m4_gyro_q1_observe_7d3a91e6.plan.md)
- [证据门控的信息接入](../agents/evidence-gated-integration.md)
- [M4 路线](../roadmap.md)

本计划的代码 base 是 `c0e214a04f8521dcf7f1c2769ebf10bf7ca06051`（短 hash
`c0e214a`）；Q1 executable evidence 是
`74270572cc1fcc2eac82559117efd0951c800ab9`（短 hash `7427057`）。Q1 final
independent PASS 只给 Q2 发出 plan-only go；issue #37 与本计划均不把当前状态描述为
implementation started 或 Q2 qualified。

## 1. 本片问题、权限与完成定义

Q2 只回答：给定同一 body frame 中、单位均为 rad/s 的 timestamped gyro point samples 与
已知常值 bias，纯 helper 能否确定性返回

\[
\Delta R_{ij}=R_{WB}(t_i)^\top R_{WB}(t_j)
\]

以及 exact `int64` nanosecond duration。

Q2 完成必须同时满足：

1. fixed point、Q1 evidence、API、错误与范围合同一致；
2. helper 不存在时，最终测试因 API/符号缺失而 RED；
3. 正常 fixture、全部 hard-error、boundary、unused-accel invariance、off-grid/shared endpoint 门通过；
4. 六类 mutant 均被各自预注册 lower bound 拒绝；
5. 独立 clean verifier 重建、逐项记录 actual，并通过 deletion/range audit。

测试进程 exit 0 不能替代逐门证据。Q2 不读取 Q1 CSV、MH_01、ATE 或 visual pose，不产生
bias estimate、covariance、factor、optimizer、posterior、初始化或 feedback。

## 2. 实施 allowlist 与禁止范围

Q2 implementation allowlist **恰为**：

```text
CMakeLists.txt
phad/estimator/gyro_rotation_predictor.hpp
phad/estimator/gyro_rotation_predictor.cpp
tests/estimator/gyro_rotation_predictor_test.cpp
phad/estimator/README.md
docs/plans/2026-08-12_m4_gyro_q2_predict_3f69becc.plan.md
```

除上述路径外不得修改 implementation 文件；不需要某个文件时不改。资格结果文档只预定为
`docs/research/m4-minimal-gyro-q2-known-bias-predict-result.md`，用于完成后记录证据，**不进入
implementation allowlist**。本次 plan preregistration 对 `docs/roadmap.md` 的链接更新也不是
Q2 implementation 差异。

CMake 只把新 `.cpp` 加入既有 `phad_estimator`，把新 test 加入既有
`phad_estimator_tests`；不新增 target。`phad_estimator` 链接关系明确为 PUBLIC
`phad::sensor`、PRIVATE `gtsam`，不得借 `phad_sync` 或其他 target 的转递依赖取得
`sensor::ImuMeasurement`；GTSAM 类型只出现在 `.cpp`。

以下路径或效果全部禁止：

```text
apps/**
phad/sync/**
phad/sensor/**
phad/estimator/stereo_vo_estimator.*
phad/frontend/**
phad/bench/**
phad/io/**
phad/eval/**
scripts/**
任何 runtime config / config_hash / CLI / artifact schema
任何 MH_01 run
任何 Q3+ source、test、统计实现或 production caller
```

不得新增 `PreintegratedAhrsMeasurements`、AHRS/rotation factor、noise model、optimizer、
graph state 或第二套 integration。若完成 Q2 必须越出 allowlist，判为合同冲突并停止，不扩大范围。

## 3. 冻结 public API 与 typed errors

`phad/estimator/gyro_rotation_predictor.hpp` 冻结以下语义；实现时只可按项目 naming/style 做
不改变合同的排版：

```cpp
namespace phad::estimator {

struct GyroRotationPrediction {
  Eigen::Matrix3d delta_R_i_j;
  std::int64_t duration_ns;
};

enum class GyroRotationErrorCode {
  kInsufficientSamples,
  kNonFiniteGyro,
  kNonFiniteBias,
  kDuplicateTimestamp,
  kOutOfOrderTimestamp,
  kTimestampDeltaOverflow,
  kDurationOverflow,
  kNonFiniteComputation,
  kNonFinitePrediction,
  kInvalidRotation,
};

struct GyroRotationError {
  GyroRotationErrorCode code;
  std::optional<std::size_t> sample_index;
  std::optional<std::size_t> interval_index;
  std::optional<std::int64_t> timestamp_ns;
  std::string detail;
};

using GyroRotationResult =
    std::variant<GyroRotationPrediction, GyroRotationError>;

GyroRotationResult integrateGyroRotation(
    std::span<const sensor::ImuMeasurement> samples,
    const Eigen::Vector3d& known_bias_radps);

}  // namespace phad::estimator
```

public header 只暴露 Eigen、`sensor::ImuMeasurement` 与标准库类型，不暴露
`gtsam::Rot3`、PIM、Params、factor 或 optimizer。`sample_index` / `timestamp_ns` 在错误能
归因到样本时必须填首个坏点；bias 或整体 output 错误没有诚实的样本归因时保持 `nullopt`，
不得伪造索引。`interval_index` 只在错误能诚实归因到相邻 endpoint interval 时填写；
`kNonFiniteComputation` fixture 必须为 `0`。`detail` 补充诊断但不替代稳定 enum code。

所有错误返回 typed failure：不得 throw 裸字符串、skip interval、排序、去重、clamp dt、
normalize invalid rotation、返回 identity 或部分成功。

## 4. 冻结数学与数据流合同

输入 `samples` 是 chronological timestamped point samples，至少两个；sample gyro 与
`known_bias_radps` 同为 body/IMU-aligned frame、单位 rad/s 且 finite。sample accel 不属于
该 helper 的输入语义：实现不得读取、检查或分支于 accel，即使 accel 含 NaN/±Inf 也不得影响
结果。helper 不接收
`StereoImuPacket`、image、gap、segment 或 visual topology。

对相邻 endpoints `k,k+1` 严格执行：

\[
d_k^{ns}=t_{k+1}^{ns}-t_k^{ns},\quad
dt_k=\operatorname{double}(d_k^{ns})\times10^{-9},\quad
\bar\omega_k=\tfrac12\omega_k+\tfrac12\omega_{k+1}.
\]

先在 integer domain checked-subtract，要求 `d_k_ns > 0`；checked-add 累计 exact
`duration_ns`，之后才将严格递增、checked `int64` 得到的正 `d_k_ns` 转为 double seconds；该转换
不另设不可达的 runtime error。不得创建 midpoint timestamp，也不得对 fixed-point upstream sync
chain 已生成的 exact endpoints 再插值。Q2 不新增、修改、链接或调用任何 sync 实现；existing
sync chain behavior 只是冻结的 upstream premise。

endpoint average 必须逐轴按 `0.5 * omega_k + 0.5 * omega_k_plus_1` 计算，禁止先相加再乘
`0.5`。调用 GTSAM 前，必须依次检查 endpoint mean、`omega_bar_k - known_bias_radps` corrected
rate，以及 corrected rate 乘 `dt_k` 得到的 rotation vector 均 finite；任一 computation 为
non-finite 都返回 `kNonFiniteComputation` 并标注对应 `interval_index`，不得把 non-finite 值交给
GTSAM。这些检查只验证 computation，不得把 corrected rate 或 rotation vector 传给 GTSAM，
也不得造成 bias double subtraction。

每段只调用：

```cpp
pim.integrateGyroMeasurement(omega_bar_k, known_bias_radps, dt_k);
```

项目代码不得预减 bias；GTSAM base `PreintegratedRotation` 恰减一次 bias。按输入顺序迭代，
保持 chronological right-compose：

\[
\Delta R_{k+1}=\Delta R_k
\operatorname{Exp}((\bar\omega_k-b_g)dt_k),\qquad \Delta R_0=I.
\]

成功只返回 `delta_R_i_j` 与 exact `duration_ns = t_last - t_first`。GTSAM 的
double `deltaTij()` 只作内部一致性诊断；不能替代时间权威值。返回前要求 matrix finite，且

```text
||R^T R - I||_F <= 1e-12
abs(det(R) - 1) <= 1e-12
```

最终 GTSAM output 非 finite 返回 `kNonFinitePrediction`；只有 output finite 但 SO(3) invariant
超限时才返回 `kInvalidRotation`。两者均不得修正后报告成功。

## 5. `freeze-fixed-point-and-contract`

任何 source/test 写入前记录：

```bash
git rev-parse HEAD
git merge-base --is-ancestor c0e214a HEAD
git merge-base --is-ancestor 7427057 HEAD
git status --short
git diff --name-status c0e214a -- CMakeLists.txt phad tests docs
```

实施 base 必须能证明包含 Q1 executable evidence，受托路径不得已有未解释的并发修改。冻结
compiler/build identity、GTSAM `4.3a0` identity，以及 CPython `3.12.3`、`mpmath 1.3.0`、
`mp.dps=80` oracle identity。任何前提无法复现都先记证据缺口，不用旧 build 冒充。

## 6. `write-red-oracles-and-mutants`

### 6.1 独立 oracle

oracle 不 import/call GTSAM、Eigen rotation utility 或 DUT，不复制 DUT C++；只用 mpmath
scalar/matrix 运算与 80 dps Rodrigues：

\[
\operatorname{Exp}(\phi)=I+\frac{\sin\theta}{\theta}[\phi]_\times
+\frac{1-\cos\theta}{\theta^2}[\phi]^2_\times
\]

并为 `theta=0` 使用解析极限。冻结 script bytes/SHA256、版本、precision、inputs 与
full-precision outputs。C++ test 只读取冻结常量，不在 test 内借 DUT/GTSAM 生成 expected。

SO(3) geodesic 使用：

\[
A=R_{ref}^\top R_{dut},\quad
s=\tfrac12\sqrt{(A_{32}-A_{23})^2+(A_{13}-A_{31})^2+(A_{21}-A_{12})^2},
\]

\[
c=\tfrac12(\operatorname{tr}A-1),\qquad e_R=\operatorname{atan2}(s,c).
\]

不得用小角 conditioning 较差的 trace-only `acos`。

### 6.2 主非交换 fixture、upper bounds 与二阶 ratio

冻结解析轨迹与 bias：

\[
R(t)=R_x(1.1t)R_y(0.7t),\quad
\omega_B(t)=[1.1\cos(0.7t),0.7,1.1\sin(0.7t)]^\top,
\]

\[
T=0.4\ \mathrm{s},\qquad b_g=[0.12,-0.08,0.05]^\top\ \mathrm{rad/s}.
\]

measurement 为 `omega_B(t)+b_g`，timestamp 由 exact integer-ns grid 生成，参考值为解析
`R(0)^T R(T)`：

| h (s) | step (ns) | oracle actual geodesic error (rad) | upper `U=0.06h^2` |
|---:|---:|---:|---:|
| 0.04 | 40,000,000 | `7.288791970389558e-5` | `9.6e-5` |
| 0.02 | 20,000,000 | `1.822294989221110e-5` | `2.4e-5` |
| 0.01 | 10,000,000 | `4.555798094738877e-6` | `6.0e-6` |

正常 DUT 每格 `error <= U`，相邻 ratio `e(h)/e(h/2)` 各在 `[3.8,4.2]`；冻结 ratios
`3.9997870890842715` 与 `3.9999467740362142`。三格都要求
`duration_ns == 400000000`。absolute 和 ratio 必须同时通过。

### 6.3 Irregular exact-ns numeric fixtures

以下三例共用 timestamps：

```text
[0, 7000000, 31000000, 60000000,
 113000000, 191000000, 260000000, 400000000]
```

每例必须 `duration_ns == 400000000`：

| case | measured gyro | exact expected | geodesic upper |
|---|---|---|---:|
| stationary + nonzero bias | `b_g` | `I` | `2e-12 rad` |
| constant 3-axis rate | `[0.35,-0.42,0.27]+b_g` | `Exp([0.35,-0.42,0.27]*0.4)` | `2e-12 rad` |
| linear commuting | `u(0.3+1.4t)+b_g`, `u=[2,-1,2]/3` | `Exp(u*(0.3T+0.7T^2))` | `2e-12 rad` |

linear commuting 例必须证明 irregular intervals 上 endpoint trapezoid 的解析 exactness，不能
用非交换误差容差替代 quadrature rule 门。

### 6.4 六类 mutant rejection

mutant 只可存在于 test-only 独立实现或冻结常量，不得在 production helper 留 branch。每门先
要求正常 DUT 通过对应 upper bound，再要求 mutant error 达到 lower `L`：

| mutant | fixture | reject lower `L` | 预注册 actual |
|---|---|---:|---:|
| reverse compose/order | 主非交换三 grid | 每格 `>=8e-3 rad` | `0.010562749192115032` / `0.010591148277551452` / `0.010598253194581134 rad` |
| left-ZOH | irregular linear commuting | `>=2e-2 rad` | `0.024304 rad` |
| double bias subtraction | stationary + nonzero bias | `>=5e-2 rad` | `0.061057350089895 rad` |
| rad/s 当 deg/s（乘 `pi/180`） | constant rate | `>=0.20 rad` | `0.239644733199408 rad` |
| deg/s 当 rad/s（乘 `180/pi`） | constant rate | `>=1.0 rad` | `1.164261180505555 rad` |
| angular-rate sign flip | constant rate | `>=0.40 rad` | `0.4878032390 rad` |

“mutant gate PASS”仅表示 test 有区分力，不表示 mutant 正确。

### 6.5 Hard-error 与 integer boundary matrix

| 输入/边界 | 精确期望 |
|---|---|
| 0 或 1 sample | `kInsufficientSamples`，不得 identity success |
| 任一 gyro axis NaN/±Inf | `kNonFiniteGyro`，首个坏 sample index/timestamp |
| bias 任一 axis NaN/±Inf | 在 GTSAM 前返回 `kNonFiniteBias` |
| duplicate timestamp | `kDuplicateTimestamp` |
| decreasing timestamp | `kOutOfOrderTimestamp`，不得排序 |
| 相邻 subtraction 超 int64 | `kTimestampDeltaOverflow`，不得触发 signed UB |
| 正 interval 累加 duration 超 int64 | `kDurationOverflow` |
| 合法 timestamp、所有输入 finite，但 endpoint mean、corrected rate 或 rotation vector 为 non-finite | GTSAM 前返回 `kNonFiniteComputation`，对应 `interval_index`，不得部分成功 |
| 最终 GTSAM output 非 finite | `kNonFinitePrediction` |
| GTSAM output finite，但正交性或 determinant 超门 | `kInvalidRotation` |
| negative epoch 且严格递增 | success，duration 只由差值决定 |
| 单个 1 ns interval | success，`duration_ns==1` |
| `[INT64_MIN, INT64_MIN+1]` | success，`duration_ns==1` |
| `[INT64_MAX-1, INT64_MAX]` | success，`duration_ns==1` |
| `[INT64_MIN, INT64_MAX]` | `kTimestampDeltaOverflow` |

所有正常 success 例同时过 SO(3) 两项门。

`kNonFiniteComputation` 的自然可达 fixture 冻结为 timestamps `[0,1]` ns，两个 endpoint gyro
均为 `[DBL_MAX,0,0]` rad/s，known bias 为 `[-DBL_MAX,0,0]` rad/s，accel 为 finite zero。
timestamp 合法且所有输入 finite；safe mean `0.5*a + 0.5*b` 仍为 `DBL_MAX`，但 corrected
x-axis rate 自然溢出。精确期望为 `kNonFiniteComputation`、`interval_index == 0`、无 prediction
或其他 partial result。不得为此新增 injection seam。若独立 verifier 所在平台不能由这组冻结
常量自然触发该结果，则 Q2 结论必须为 `INCONCLUSIVE` 并停止；不得临时更换常量、构造方式或
注入点取得 PASS。

### 6.6 Unused-accel invariance hard gate

冻结一组相同 timestamps、gyro 与 known bias 的合法多 interval fixture，只改变每个
`sensor::ImuMeasurement` 的 accel。变体必须至少覆盖：正常 accel、任意不同的 finite accel、
含 NaN、含 `+Inf`、含 `-Inf`。全部变体都必须 success，且 `duration_ns` 与基准 exact 相同，
每个返回 rotation matrix 的每个 `double` 元素均与基准 element-exact equal；不是 tolerance
comparison。该门直接限定 helper 不得读取 accel，包括不得对 accel 做 finite validation。

### 6.7 Off-grid 与 shared endpoint fixture

existing sync chain behavior 在 Q2 中只作为 fixed-point upstream premise；Q2 不新增、修改、
链接或调用 sync，也不在 estimator test 中复刻或测试 sync policy。estimator test 只构造带明确
provenance 注释的 endpoint-aligned `sensor::ImuMeasurement` samples：这些冻结值被声明为来自
上游 chain 的既有输出。用于说明 provenance 的 raw conceptual motion 为
`omega(t)=u(0.3+1.4t)+b_g`，`u=[2,-1,2]/3`，原始 stamps
`[-10,7,31,70,120] ms`，边界为 `0,60,113 ms`；测试本身不得由 raw stamps 调用、链接或重算
sync。fixture 必须显式断言：

1. 0、60、113 ms endpoints timestamp/value 与 sync-owned 插值期望 exact；
2. `[0,60]` 与 `[60,113] ms` 无 midpoint timestamp；
3. 两段共享的 60 ms endpoint sample value bit/exact-equal；
4. durations 为 `60000000` 与 `53000000` ns，总和 `113000000` ns；
5. 各段相邻 integer dt sum 等于 endpoints difference；
6. 总计三次 helper 调用：分别计算 `[0,60]` 与 `[60,113] ms` 两段（shared endpoint 因而被
   两段调用消费），再计算 shared endpoint 仅保留一次的 `[0,113] ms` 整段；exact-add 两段返回的
   duration，结果必须满足 `R_02 = R_01 * R_12`，且 `R_01 * R_12` 与整段 helper result 的
   geodesic error `<=2e-12 rad`；
7. segmented 与 whole-span 均相对 linear commuting analytic integral `<=2e-12 rad`。

这只验证 helper 无歧义消费已有 endpoint-aligned samples；不授权 packet merge，也不授权新增、
修改、链接、调用 sync 或再次插值。

### 6.8 RED 证据

先只加入最终 Q2 test 与既有 target 接线；helper header/source 尚不存在时构建
`phad_estimator_tests`，预期 compile 或 link failure，记录 exact command、exit code 与首个
缺失 API/符号。随后才新增 helper。不得用人为 `EXPECT_TRUE(false)` 或 disabled test 伪造 RED。

## 7. `implement-pure-predictor`

按以下最短顺序 GREEN：

1. 在 public header 落第 3 节值类型、typed errors、variant result 与函数；
2. `.cpp` 先验证 bias 与全部 sample gyro finite（不得读取或验证 accel），再逐段 checked
   timestamp arithmetic；
3. 只用 `0.5*a + 0.5*b` safe endpoint average 形成 interval representative；在 GTSAM 前验证
   mean、corrected rate 与 rotation vector finite，任一 computation non-finite 时返回
   `kNonFiniteComputation` 与对应 `interval_index`；验证通过后仍将原始 representative 与 known
   bias 分别交给 GTSAM base `PreintegratedRotation`，不得预减 bias；
4. 按 chronological right-compose 累积，返回 exact integer duration；
5. 对 output finite、SO(3) 与 GTSAM double duration 只做 hard validation/诊断，不做修复；
6. README 只补当前 helper 的职责、依赖方向、输入/输出/错误与“尚无 production caller”边界；
7. CMake 只扩既有 targets 与既有依赖，不新增 target 或测试入口。

实现不得加入 future options、config、generic integration framework、failure injection seam 或
Q3 seam。若标准库 checked
arithmetic 需要小型私有 helper，只放 `.cpp` unnamed namespace，不新增文件或 public abstraction。

## 8. `run-independent-unit-qualification`

implementer 先做定向 GREEN；随后由未复用 implementer build/output 的独立 verifier 在 clean
checkout/build 执行同一资格矩阵。命令以仓库实际 generator 为准，至少包括：

```bash
cmake -S . -B build-q2-verifier \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DPHAD_BUILD_TESTS=ON
cmake --build build-q2-verifier --target phad_estimator_tests -j2
build-q2-verifier/phad_estimator_tests \
  --gtest_filter='*GyroRotationPredictor*'
ctest --test-dir build-q2-verifier -L unit --output-on-failure -j2
```

资格账本逐例记录 measured actual，而非只抄预注册值：三组 normal errors/U/ratio、三组
irregular errors、每个 duration、六个 mutant errors/L、每个保留的 typed error
code/sample_index/interval_index/timestamp、SO(3) actual、unused-accel 各变体的 success、exact
duration 与 matrix element-exact equality、off-grid/shared endpoint 两段调用与整段调用的 exact
duration add、`R_01 * R_12` 与 whole-span result 的 composition error，以及 segmented 与
whole-span 各自相对 analytic result 的 error；并记录 frozen `DBL_MAX` fixture 是否在 GTSAM 前
自然返回 `kNonFiniteComputation`。后者若在 verifier 平台不能自然触发，立即记 `INCONCLUSIVE`
并停止，不得换常量或增加 injection seam。

独立 verifier 还必须完成：

```bash
rg -n 'integrateGyroRotation|GyroRotationPrediction' . \
  -g '!docs/research/m4-minimal-gyro-q2-known-bias-predict-design.md' \
  -g '!docs/plans/2026-08-12_m4_gyro_q2_predict_3f69becc.plan.md'
git diff --name-only c0e214a -- CMakeLists.txt phad tests docs
git diff --check
git status --short
```

deletion test 在可恢复的临时副本/patch 上验证：删除 helper `.hpp/.cpp` 后 production
libraries/apps 无 source 修改需求；保留 tests 删除 helper 时，Q2 target 必须 compile/link fail；
恢复后重新构建定向 target。最终 `rg` 只允许 helper test 引用 API，不得出现
`StereoVoEstimator`、apps/session/bench caller。

不运行 MH_01、EuRoC 11/11、ATE、Sanitizer、性能或 stochastic qualification；这些不是 Q2
deterministic unit gate，结果文档必须明确列为未执行。

## 9. `record-q2-stop-go`

结果只记录到预定的
`docs/research/m4-minimal-gyro-q2-known-bias-predict-result.md`，必须包含 source/build/compiler/
GTSAM/oracle identity、RED 与 GREEN exact commands/exit codes、oracle SHA、逐门 actual、范围审计、
deletion test、未执行项与剩余风险。

| 结论 | 判定 | stop/go |
|---|---|---|
| `PASS` | 正常 DUT 全部 upper/duration/SO(3)/hard-error 门通过；六 mutant gates 全通过；RED、allowlist、deletion 与独立 clean verifier 成立 | 只允许另建 **Q3 plan**；Q3 implementation 仍 stop |
| `FAIL` | 任一 deterministic 正常门、typed error、duration、SO(3) 或 mutant rejection 失败；或已有有效 observation 超门 | 保留负结果，修 Q2 最早失败；不得进入 Q3 |
| `INCONCLUSIVE` | 仅 fixed point/Q1 前提、冻结 CPython/mpmath/oracle、compiler、必需 dependency 无法复现，或 verifier 平台不能用冻结 `DBL_MAX` fixture 自然触发 `kNonFiniteComputation`，因而资格合同无法完整观测 | 记录缺口和复现方式，Q2 保持 pending；不得换常量、加 injection seam 或以 unit exit 0 放行 |

观测到 deterministic failure 后不能降格为 `INCONCLUSIVE`。fixed-seed noise、covariance、
whitening 与 stochastic coverage 已从 Q2 删除，未来必须另行预注册。

Q2 PASS 的最大权限是 **Q3 plan-only go**；不得在本片写 Q3 统计实现、读 MH_01/Q1 CSV、
fit bias、建 factor 或改变 visual posterior。

## 10. 收尾与执行状态

计划落库阶段只验证 YAML、链接/code fence 与 diff scope；不会把任何 todo 标为 completed。
实施结束时再执行：

```bash
git diff --check
git status --short
git diff -- CMakeLists.txt phad/estimator tests/estimator docs/plans
```

当前事实：DUT、tests 与 CMake 均未按本计划实现，independent clean verifier 未运行，result doc
尚未创建，Q2 结论尚未产生。
