# M4 minimal gyro Q2：known-bias deterministic predict 资格结果

日期：2026-08-13

## Verdict 与权限边界

**Q2 deterministic known-bias rotation-only technical PASS under one-time post-hoc
evidence-retention waiver。** original frozen-plan RED exact-record conformance: **NOT MET**。唯一
technical verdict 绑定 clean fixed point `d1c4385809a6bf461f1c6b8acd81870f98634aa0` / tree
`9c9c991b21c4d40e8c5f2ce974334b76e1a42b64`。数值、SO(3)、exact duration、typed/natural
defensive、六类 mutant、unused-accel、shared endpoint、deletion、可达性、独立 oracle、范围与格式门
均通过；原始 RED 不参与资格判定，qualifying weight = `0`。范围门按已记录的
edit 前 narrow waiver 判定，RED 留存缺口按 coordinator 的一次性 post-hoc waiver 判定，
见第 8–9 节。

本 PASS **只授权 Q3 plan/design**。它不授权 Q3 implementation，不接入 production caller，
不改变 visual posterior、配置或产品输出，也不证明 stochastic IMU、bias estimation、factor、
covariance 或完整 VIO 已取得资格。Q1 前置 PASS 见
[Q1 Observe 资格结果](m4-minimal-gyro-q1-observe-result.md)，冻结合同见
[Q2 Predict 设计](m4-minimal-gyro-q2-known-bias-predict-design.md)和
[Q2 实施计划](../plans/2026-08-12_m4_gyro_q2_predict_3f69becc.plan.md)。

## 1. 身份账本

| 对象 | 身份 |
|---|---|
| Q2 base / Q1 result ledger | `c0e214a04f8521dcf7f1c2769ebf10bf7ca06051` |
| Q1 executable evidence | `74270572cc1fcc2eac82559117efd0951c800ab9` |
| Q2 implementation | `0f51eaa37a3828d2ffea7ec07c75c0e56c10451d` |
| Q2 ledger correction | `222c5a31a964f66bab1fff30cf328d9a00bce575` |
| Q2 final code / tree | `d1c4385809a6bf461f1c6b8acd81870f98634aa0` / `9c9c991b21c4d40e8c5f2ce974334b76e1a42b64` |
| verifier config | `/home/lin/Projects/lin_ws/slam_ws/phad-vio/.worktree/m4-minimal-gyro/.codex/agents/vio_verifier.toml` |
| verifier config SHA-256 | `8be3b22014708ed38f1032d81d8e1d591f3ff5da749417ab8021b1a94edfea46` |
| final verifier report | `/tmp/q2-final-requal-d1c4385/Q2-final-independent-requalification.md` |
| final verifier report SHA-256 | `574a946fd08a6006de9b5477766443fed9fe5fd92b6079b93c772630a30b9ed1` |
| finding / SHA-256 | `/tmp/q2-final-docs-spec-review.md` / `d2368fa122f69b1e35eb51109f9ae2ec667cadda90a6b49e3e932c34f94eaa69` |
| RED forensics / SHA-256 | `/tmp/q2-missing-evidence-forensics.md` / `482571e2355d65ec257d6405332377ccc5587216282026f3778094af18915aa5` |
| independent adjudication / SHA-256 | `/tmp/q2-red-gap-adjudication.md` / `bb3e1a5c6f668c13a9ad8a520e1b35e56ac87fcb7e724fa8845179e4ab31ad1c` |
| coordinator resolution | `task_4d57d3d8c484` / `gate_f800287d8728` / `2026-08-12T17:26:28Z` |
| post-final replay v2 report / SHA-256 | `/tmp/q2-missing-evidence-replay-v2.md` / `06951458173c910dc09e36624a782b233d85ef777703ccf8d155a9ba39ea8809` |
| v2 authoritative command ledger / SHA-256 | `/tmp/q2-missing-evidence-replay-v2/commands-v2.txt` / `272f434aa0154e914454a3e41f5141f678784e3510d7fdfadf5e083e6745a63e` |
| v2 executable child / SHA-256 | `/tmp/q2-missing-evidence-replay-v2/run.sh` / `d6e22e390b780a802d5998d03950351222cdac306bd97dbf1a0f0317771060b9` |
| v2 parent archive/invoker / SHA-256 | `/tmp/q2-missing-evidence-replay-v2/invoke.sh` / `ed95091ae60a2f1b5c6aafbc21aaf2a4eb0d2be7ddf0a001244498a2435e9bce` |
| v2 probe source / SHA-256 | `/tmp/q2-missing-evidence-replay-v2/dut_probe.cpp` / `c313e9a8bdd89afcef31c0c71271d8986a829a0dde848078a53966b7ede2f0aa` |
| v2 all-log/input manifest / SHA-256 | `/tmp/q2-missing-evidence-replay-v2/evidence/log-sha256.txt` / `9670629a24205c6da28340092ef29a21d0f2ec322ede15d102f936b39c4f11bd` |

final verifier 开始和结束时 `git status --short` 均为空，`git diff --check` 均 exit 0；
`c0e214a` 与 `7427057` 的 ancestor checks 均 exit 0。环境为 x86_64、glibc 2.39、GCC
13.3.0、CMake 3.31.11、Eigen 3.4.0、GTSAM 4.3（configured library `4.3a0`）、
CPython 3.12.3、mpmath 1.3.0、Release；exact-decimal oracle 使用 `mp.dps=100`，高于冻结的
80-dps 下限。

## 2. Fresh Release 命令、exit 与 counts

除 deletion 临时 archive 外，cwd 为
`/home/lin/orca/workspaces/m4-minimal-gyro/tigerfish`，无额外显式 env。

| 命令 / 动作 | exit | 实测结果 |
|---|---:|---|
| `cmake -S . -B build-q2-final-requal-d1c4385 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPHAD_BUILD_TESTS=ON` | `0` | fresh configure |
| `cmake --build build-q2-final-requal-d1c4385 --target phad_estimator_tests -j2` | `0` | target built；只有既有非本片 Eigen warning |
| `build-q2-final-requal-d1c4385/phad_estimator_tests --gtest_filter='*GyroRotationPredictor*'` | `0` | **10/10 PASS** |
| `ctest --test-dir build-q2-final-requal-d1c4385 -L unit --output-on-failure -j2` | `0` | **77/77 PASS** |
| `python3 /tmp/q2-final-requal-d1c4385/q2_exact_decimal_oracle.py` | `0` | exact-decimal oracle；C++ source/expected/actual bits **9/9** |
| standalone Release DUT probe | `0` | 第 4–6 节 actual |
| standalone direct-GTSAM defensive probe | `0` | 第 7 节 natural-invalid actual |
| repo 外 positive / negative deletion configure+build | `0` / expected `2` | 两臂均取得预注册结果 |
| `clang-format --dry-run --Werror`（Q2 hpp/cpp/test） | `0` | clean |
| scope / API references / public GTSAM leak / oracle independence audit | `0` | clean |

关键证据 SHA-256：DUT probe log
`a76c7f4301d78505a816663f09c6e4ab543980083f17910703ea1b9dd749649e`；CTest log
`d3d9ec26ae5f211231e304afe2f22d8a0ba3d1886d25f3b1977800bedc5d82ad`。

## 3. Independent exact-decimal oracle 与 1-ULP 闭环

oracle 输入为 decimal strings `0.4 * [0.35,-0.42,0.27]`，以 100-dps scalar/matrix
Rodrigues 计算；不 import/call DUT、Eigen 或 GTSAM。脚本 SHA-256 为
`e513d13f97f9abd6a5f79a154bf484b5dc9f6f6e658ef87e5831c11029a4d5b8`，输出 SHA-256 为
`bae58704d85e828b709ecb1d9be0120dcb91ce27a7dcd1b8699771fa652dad7d`。

row-major nearest-binary64 与 C++ hex literal 逐项为：

| index | bits | C++ hex literal |
|---:|---|---|
| 0 | `3fef5d6d554f2303` | `0x1.f5d6d554f2303p-1` |
| 1 | `bfbe5ecfd3e7bb9f` | `-0x1.e5ecfd3e7bb9fp-4` |
| 2 | `bfc4541aed6189d5` | `-0x1.4541aed6189d5p-3` |
| 3 | `3fb86107ff3762f5` | `0x1.86107ff3762f5p-4` |
| 4 | `3fef8093833ffa60` | `0x1.f8093833ffa60p-1` |
| 5 | `bfc2e5f8ced3715c` | `-0x1.2e5f8ced3715cp-3` |
| 6 | `3fc6411b289a389e` | `0x1.6411b289a389ep-3` |
| 7 | `3fc0965eee290604` | `0x1.0965eee290604p-3` |
| 8 | `3fef3d15013cca11` | `0x1.f3d15013cca11p-1` |

旧 `222c5a3` oracle 把 binary64 `0.4` 提升到 mpmath，令 index 2、3、7 分别偏离正确值
`-1/+1/+1 ULP`。`d1c4385` 改为上述 canonical hex；final run 中 source/expected/actual
9/9 相等。旧行为门没有因 3 ULP 修正翻转，但旧 oracle identity finding 已由新 fixed point
与从头重验闭合，不能继承 `222c5a3` 的 PASS 作为 final verdict。

## 4. 正常 DUT、ratio、SO(3) 与 duration actual

| fixture | geodesic error (rad) | upper | duration (ns) | `(||R^T R-I||_F, |det(R)-1|)` |
|---|---:|---:|---:|---|
| noncommuting `h=.04` | `7.2887919703836087e-05` | `9.6e-5` | `400000000` | `(1.0290171443781209e-15,1.1102230246251565e-16)` |
| noncommuting `h=.02` | `1.8222949892207639e-05` | `2.4e-5` | `400000000` | `(1.2708598865977067e-15,8.8817841970012523e-16)` |
| noncommuting `h=.01` | `4.5557980946926499e-06` | `6e-6` | `400000000` | `(2.3489155167184182e-15,1.5543122344752192e-15)` |
| commuting stationary | `0` | `2e-12` | `400000000` | `(0,0)` |
| commuting constant | `1.3877787807814457e-17` | `2e-12` | `400000000` | `(6.4981546634941018e-16,0)` |
| commuting linear | `3.2186016552109489e-17` | `2e-12` | `400000000` | `(5.8470217996261925e-16,2.2204460492503131e-16)` |

noncommuting convergence ratios 为 `3.9997870890817668` / `3.999946774076041`，均在
`[3.8,4.2]`。irregular linear commuting actual 同时闭合 endpoint trapezoid 的解析 exactness；
不是用非交换误差容差替代 quadrature rule 门。

## 5. 六类 mutant actual 与 lower bounds

| mutant class | actual error (rad) | frozen lower `L` |
|---|---:|---:|
| reverse compose，grid 1 | `0.010562749192114983` | `8e-3` |
| reverse compose，grid 2 | `0.010591148277551326` | `8e-3` |
| reverse compose，grid 3 | `0.010598253194581133` | `8e-3` |
| left-ZOH | `0.024303999999999989` | `.02` |
| double-bias subtraction | `0.061057350089894998` | `.05` |
| rad/s as deg/s | `0.23964473319940813` | `.20` |
| deg/s as rad/s | `1.1642611805055534` | `1` |
| angular-rate sign flip | `0.48780323902163669` | `.40` |

这是六类 mutant；reverse-compose 类在三组 grid 上各有一项 actual，因此表中共八行。
所有 normal upper gate 先通过，随后对应 mutant 才按 lower bound 被拒绝。

## 6. Shared endpoint 与 unused accel

shared endpoint 三次 helper 调用的 exact durations 为：first `60000000` ns、second
`53000000` ns、whole `113000000` ns；前两项 exact-add 等于 whole。final requalification
当时只保留了 composition、segmented analytic、whole analytic 三项 `<=2e-12` PASS
bound，没有打印逐值十进制 actual。后续 post-final replay 在同一 fixed code/tree 上补齐：

| item | geodesic actual (rad) | exact duration |
|---|---:|---:|
| first analytic vs first prediction | `5.1165412024903705e-18` | `60000000 ns` |
| second analytic vs second prediction | `2.6942274008182644e-18` | `53000000 ns` |
| whole analytic vs whole prediction | `3.962996241602435e-18` | `113000000 ns` |
| composition closure: `(R01 * R12)` vs `R02` | `0` | `60000000 + 53000000 = 113000000 ns` |
| segmented vs frozen whole analytic | `3.962996241602435e-18` | — |

三项逐项 actual、composition closure 与 duration exact-add 均通过原冻结门；没有改动 fixture
或 `2e-12` 阈值。replay 证据身份与命令见 §8.3。

unused-accel hard gate 覆盖 normal、different finite、NaN、`+Inf`、`-Inf` 五类 accel；全部
success，duration exact 相同，返回 rotation matrix 每个 `double` 元素 bit/exact equal。这限定
helper 不读取或验证 accel。

## 7. Typed errors 与 natural defensive actual

- 0/1 sample → `kInsufficientSamples`；gyro 每轴 NaN/`+Inf`/`-Inf` →
  `kNonFiniteGyro`，携带 first bad sample/timestamp；bias 同类输入 → `kNonFiniteBias`，三个
  optional 均为 `nullopt`。
- duplicate/decreasing timestamp 分别返回 `kDuplicateTimestamp` / `kOutOfOrderTimestamp`，
  actual 为 `(sample=1, interval=0, timestamp=10/9)`；negative epoch、1 ns 与两端 INT64
  safe edges 均 success。
- full-range subtraction → `kTimestampDeltaOverflow (1,0,INT64_MAX)`；累计溢出 →
  `kDurationOverflow (null,1,null)`。
- natural nonfinite computation → enum code `7` / `kNonFiniteComputation (null,0,null)`；
  natural nonfinite prediction → code `8` / `kNonFinitePrediction (null,null,null)`；natural
  invalid rotation → code `9` / `kInvalidRotation (null,null,null)`。
- invalid-rotation 的 direct-GTSAM 470-interval matrix 在 DUT final validation 前 finite，
  `||R^T R-I||_F=1.0012190690733831e-12`、`|det(R)-1|=7.5839334812144443e-13`；故该门
  自然触发，不是 injection。
- production `.cpp` 静态审计确认 final `allFinite/isfinite`、GTSAM duration consistency、
  orthogonality 与 determinant checks 存在且未绕过。

## 8. TDD RED、deletion、API 与 oracle independence

### 8.1 原始 TDD RED 的已知事实与证据缺口

Orca task/dispatch `task_a3ea5796bd7b` / `ctx_5f26c67ef846` 的 lifecycle summary 只证明：
helper hpp/source 尚不存在时，`phad_estimator_tests` build exit `2`，首个 fatal 是缺失
`phad/estimator/gyro_rotation_predictor.hpp`。bounded archived transcript 没有保留原始 RED 的
exact command、build directory、stdout/stderr artifact 或 log SHA；原 final requalification evidence 也
无法恢复当时的 exact standalone probe compile command。上述原历史项目是明确证据缺口，
本文不伪造；§8.3 记录的是后来 replay 新执行的可复制命令，不是恢复原命令。

final verifier 的 negative deletion arm 独立复现了相同依赖失败，但**不冒充原始 RED**：保留 Q2
test，删除 predictor hpp/cpp 与 production source CMake wiring，fresh configure exit `0`，构建
`phad_estimator_tests` expected exit `2`，首个 fatal 同为缺失 header。

### 8.2 Deletion、可达性与边界

positive arm 将 `git archive HEAD` 放到 repo 外，删除 predictor hpp/cpp、Q2 test 与两处 CMake
wiring，以 `PHAD_BUILD_TESTS=OFF` fresh Release configure 后构建 `phad_vo_bench -j2` exit `0`。
因此 production apps 不依赖该 helper，当前**没有 production caller**。

排除 plan/research 后，API reference 只命中 helper hpp/cpp、README 与 Q2 test；apps/session/
bench/`StereoVoEstimator` 没有 caller。public estimator header 不泄漏 GTSAM、`Rot3`、PIM、
factor 或 optimizer；GTSAM 保持 `.cpp` PRIVATE。Q2 test 不 include/use GTSAM；exact oracle 不依赖
DUT、Eigen 或 GTSAM，故没有 test-side GTSAM oracle injection。

### 8.3 Post-final replay：可重跑账本闭合

这是 finding 之后针对可重跑项的 supplementary replay，**不是原始 TDD RED**，
也不替代原 RED exact record。P3 审计发现旧
`/tmp/q2-missing-evidence-replay/commands.txt` v1 header 声称 inherited environment，
却与 v1 的实际 `env -i` 记录矛盾；v1 已被 supersede，**不得用作最终命令证据**，但无需删除。

最终命令证据以 `/tmp/q2-missing-evidence-replay-v2.md`（SHA-256
`06951458173c910dc09e36624a782b233d85ef777703ccf8d155a9ba39ea8809`）及 §1 所列 v2
artifact 为权威。`invoke.sh` 从
`git archive d1c4385809a6bf461f1c6b8acd81870f98634aa0` 解出固定 tree
`9c9c991b21c4d40e8c5f2ce974334b76e1a42b64`，不从当前工作树复制 source；随后父进程实际执行：

```text
/usr/bin/env -i PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin HOME=/home/lin LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Asia/Shanghai /bin/bash --noprofile --norc /tmp/q2-missing-evidence-replay-v2/run.sh
```

`run.sh` 使用 `set -euxo pipefail`，child source cwd 为
`/tmp/q2-missing-evidence-replay-v2/source`；effective environment、工具版本、每条展开命令、cwd
与 exit 均落入 manifest 覆盖的日志。它使用 `vio_verifier.toml`（SHA-256
`8be3b22014708ed38f1032d81d8e1d591f3ff5da749417ab8021b1a94edfea46`）从固定 archive source
fresh Release configure/build；定向 `GyroRotationPredictor.*` **10/10 PASS**，standalone probe
链接该 fresh build 的 `libphad_estimator.a`，configure、build、gtest、compile 与 run 均 exit `0`。

权威日志中记录的 expanded commands 为：

```bash
/usr/local/bin/cmake -S /tmp/q2-missing-evidence-replay-v2/source -B /tmp/q2-missing-evidence-replay-v2/build -DCMAKE_BUILD_TYPE=Release -DPHAD_BUILD_TESTS=ON -DPHAD_ENABLE_MH01_TESTS=OFF -DPHAD_ENABLE_TUMVI_CORRIDOR1_TESTS=OFF
/usr/local/bin/cmake --build /tmp/q2-missing-evidence-replay-v2/build --target phad_estimator_tests --parallel 2
/tmp/q2-missing-evidence-replay-v2/build/phad_estimator_tests --gtest_filter='GyroRotationPredictor.*'
/usr/bin/c++ -std=c++20 -O3 -DNDEBUG -I/tmp/q2-missing-evidence-replay-v2/source -isystem /usr/include/eigen3 -isystem /usr/local/include/gtsam/3rdparty/SuiteSparse_config -isystem /usr/local/include/gtsam/3rdparty/CCOLAMD -isystem /usr/local/include/gtsam/3rdparty/metis -isystem /usr/local/include/gtsam/3rdparty/cephes /tmp/q2-missing-evidence-replay-v2/dut_probe.cpp /tmp/q2-missing-evidence-replay-v2/build/libphad_estimator.a -Wl,-rpath,/usr/local/lib /usr/local/lib/libgtsam.so.4.3a0 -o /tmp/q2-missing-evidence-replay-v2/dut_probe
/tmp/q2-missing-evidence-replay-v2/dut_probe
```

| 命令 | exit | v2 log / SHA-256 |
|---|---:|---|
| configure | `0` | `evidence/configure.log` / `c8224820227abba8434f361e1a60968b768a009dcbe84b51f463ec4a9bd964ae` |
| build `phad_estimator_tests` | `0` | `evidence/build-estimator-tests.log` / `db9d45b6335f713792368498db605e4b0989c87dfa04d3072a7fde9140e4a7c8` |
| directed gtest | `0` | `evidence/gyro-tests.log` / `9212b194800fb2d2c40ca1e46a7267e7ccda870c6dd50b72fac751aefac3c5c5` |
| standalone compile | `0` | `evidence/dut-probe-build.log` / `51f4c0a14a12c42e61e39614dc9fb93c8ef13e32854ffdb5f11fa9e0a51f5968` |
| standalone run | `0` | `evidence/dut-probe.log` / `38389d1abfdd355ed6ad5cf128a888216067cae90012b9150283aa275fbeb052` |

### 8.4 Evidence disposition

| disposition | evidence | 资格用途 |
|---|---|---|
| qualifying | `d1c4385` clean requalification、independent exact-decimal oracle、normal/boundary/error/SO(3)/duration、mutants、natural defensive、scope 与 positive/negative deletion characterization | 计入当前 Q2 technical verdict |
| qualifying supplementary | post-final replay 的 exact probe command/source/log 与 shared-endpoint numeric actual | 补齐可重跑 ledger 项；计入 technical verdict，不计作 RED |
| historical non-qualifying | 原始 implementer RED lifecycle summary：`phad_estimator_tests` exit `2`、缺失 header | qualifying weight = `0` |
| permanently unavailable | 原始 RED exact command、build directory、full stdout/stderr artifact 及 log SHA | original frozen-plan exact-record conformance **NOT MET** |
| explicitly not a substitute | later final deletion negative arm | 仅表征当前 test→header 依赖/deletion sensitivity；不替代原 RED |

### 8.5 一次性 waiver 与非泛化边界

coordinator task `task_4d57d3d8c484` / decision gate `gate_f800287d8728` 于
`2026-08-12T17:26:28Z` resolved，授予一次性 post-hoc RED exact-record retention
waiver。它只豁免上表中不可恢复的原 RED 留存义务，不豁免其他任何机制门、数值、
fixture、阈值、scope、clean-verifier 要求或权限上限。这是 Q2 历史 evidence-retention
事故的一次性处置，不改写
[`evidence-gated-integration`](../agents/evidence-gated-integration.md) 的一般规则，不降低
Q3+ 证据留存标准，也不授权 Q3 implementation。

## 9. Scope truth 与 authority chronology

| scope class | 冻结 / authority | actual | 判定 |
|---|---|---|---|
| original prereg production implementation | `a5e1a04` §2 六路径 | 六路径，无 forbidden production path | PASS |
| pre-edit evidence correction waiver | `run_6cae5c72bf1b/task_76375610b1d9`；created `2026-08-12T15:18:26Z`，dispatched `15:18:40Z` | 只改 Q2 research design 的 natural-defensive evidence correction 与 provisional/status | PASS under documented narrow waiver |
| `0f51eaa` packaging | 上述两类 authority 合计 | six + one research，共七路径 | MATCH effective authorized scope |

事实：`a5e1a04` 原始 prereg 的 production implementation allowlist 恰为六路径，**不包含**
Q2 research design。reachability review 后，coordinator 在 research edit 与 `0f51eaa` commit 前创建并
dispatch `task_76375610b1d9`，对该 research 单一路径授予上述窄幅 waiver。`0f51eaa` 随后将六路径
production implementation 与这一项 evidence correction 打包为七路径。该 authority 是原 prereg
的 pre-edit narrow waiver，不是原 §2 已有的第七路径，也不是 `222c5a3` 的事后追授；
`222c5a3` 只是后续 ledger commit。

`c0e214a..d1c4385` 总计八条路径：上述七条，加既有 `docs/roadmap.md` 状态链接。没有
apps、sync、sensor、config/hash、factor、posterior、Q3 source 或其他 forbidden production path。

## 10. Verifier chronology correction

1. `0f51eaa` first verifier attempt 没有读取 dispatch 指定、checkout 外且实际存在的 mandatory
   `vio_verifier.toml`，故是 **process-invalid / unqualified attempt；无 Q2 PASS/FAIL/
   INCONCLUSIVE verdict**。它报告的 10/10、77/77、probe、deletion、scope 与格式结果仅作
   non-qualifying historical observations，不产生 stop/go。
2. `222c5a3` 后续 clean requalification 独立给出 PASS，但其 exact-constant oracle identity 随后被
   发现有三处 1-ULP 偏差，故该 PASS 被 supersede，不作为 final verdict。
3. `d1c4385` 修正 oracle bits 后，verifier 先读取 mandatory config，再从头重跑全部资格门；
   后续 replay 补齐可重跑 ledger 项。在原 RED 资格权重为零的前提下，本页唯一有效结论为
   **technical PASS under one-time post-hoc evidence-retention waiver**；原 frozen-plan RED
   exact-record conformance 仍为 **NOT MET**。

## 11. 事实、推断与未运行项

### 已验证事实

- 本文列出的 commit/tree、命令 exit/count、oracle SHA/bits、DUT/mutant/defensive actual、
  deletion 与 scope provenance 均由指定 final report、post-final replay、裁定报告、Git/Orca
  记录交叉核验。
- Q2 helper 只覆盖 deterministic known-bias rotation prediction，且无 production caller。

### 受证据支持的推断

- positive deletion 与 reference audit 支持“删除 helper 不影响当前 production app 构建”；这不等于
  已证明所有未来 caller 的集成行为。
- 六类 mutant 被拒绝说明当前 tests 对冻结错误模型有区分力，不证明未预注册错误类已被穷尽。

### 刻意未运行与剩余边界

本次未运行 MH_01、任何 EuRoC 序列、EuRoC 11/11、ATE、RPE、Sanitizer、performance、
noise、covariance、whitening 或 stochastic qualification。没有 MH_01/11/11/product metric
结论；duration 与数值资格不应外推为 bias fit、factor 或 posterior 已正确。下一步仅可写 Q3
plan/design，Q3 implementation 仍未授权。
