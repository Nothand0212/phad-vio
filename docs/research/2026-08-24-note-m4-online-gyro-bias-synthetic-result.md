# M4 online gyro bias synthetic 资格结果

日期：2026-08-19（receipt sealed）；2026-08-24 完成 post-verdict bookkeeping

Protocol ID：`PHAD-M4-ONLINE-GYRO-BIAS-SYNTHETIC-V1`

最终 verdict：`HARD_ERROR`

## 1. 结论与权限

Replay2 在 `negative-arm setup` 阶段因
`harness/operator wrong-argument invocation` 停止。该 verdict：

```text
product_attribution=false
scientific_attribution=false
permission_granted=false
stop_applied=true
```

这不是 `PASS`，也不是 product/scientific `FAIL` 或 `HYPOTHESIS_FAIL`。此前已经取得的 positive
actual 与 5 条有效 negative-arm `PASS` 继续作为 provenance 保留，但不能把整体 replay2 verdict
升级为 `PASS`，也不能产生科学结论或新增权限。

本结果不改写 [normative design](2026-08-19-note-m4-online-gyro-bias-synthetic-design.md)、
[ADR-0002](../adr/0002-stage-gated-gyro-only-bias-state.md)、
[conventions](../design/conventions.md)、[estimator contract](../../phad/estimator/README.md) 或任何冻结阈值、
fixture、协议与历史结论。Q3 的唯一结论继续是
`HYPOTHESIS_FAIL / HALF_STABILITY / STOP`，见
[Q3 result ledger](2026-08-18-note-m4-minimal-gyro-q3-offline-bias-alignment-result.md)。

## 2. Authority、candidate 与 evidence 身份

### 2.1 Receipt 与 roots

| 项 | 冻结身份 |
|---|---|
| receipt | `/home/lin/orca/evidence/phad-m4-online-bias-v1-replay2-634c4ec/replay2-hard-error-receipt.json` |
| receipt mode / SHA-256 | `0444` / `bf4dc238eef9751042c7077a2713c9d0fed64fa3c189bf00b48e86179b89716a` |
| sealed at | `2026-08-19T07:25:57+08:00` |
| candidate repo | `/home/lin/orca/workspaces/m4-minimal-gyro/tigerfish` |
| evidence root | `/home/lin/orca/evidence/phad-m4-online-bias-v1-replay2-634c4ec` |
| positive archive | `/home/lin/orca/archives/phad-m4-online-bias-v1-positive-replay2-634c4ec` |
| mutation scratch root | `/home/lin/orca/scratch/phad-m4-online-bias-v1-replay2-634c4ec` |

root realpath/worktree exclusion preflight exit 为 `0`；owner-writable `0644` 文件与 `0755` 目录的
negative self-check 均按预期 exit `90`。

### 2.2 Authority 与 candidate

| 项 | commit / tree / content identity |
|---|---|
| exact-six authority | commit `634c4ecfbc5cbd7b0d8da1a295e48c12f7f81194`；tree `a7fd588983b1f8597e98517a3d0d4cbcf6609100` |
| normative design | blob `07489430cde613f9e0fc431885c7bbd3835a9ff7`；SHA-256 `e7db8acdf7d87d78f74d8006039dca1e9134ff85e4940b6cbaadc1698c00d37d` |
| implementation plan at authority | blob `9bfdef4b59bdb747df55294c975eaf1a5f343a37`；SHA-256 `fe5aab349164e24c7f499cae65da964276e3d207a2c14d656bf0db1f793d7d8d` |
| candidate branch | `Nothand0212/m4-online-gyro-bias-synthetic-replay2` |
| candidate source | commit `9bdd32df4089774b667db721815bb89a3007a47b`；tree `732c8ea3e8a280693b58dac4211646f5a6a0e44a` |

receipt 形成与 STOP 时 candidate `status_bytes=0`、工作树 clean，且 authority/verifier 未被修改。
这里记录的是 qualification 时的 source identity；本 result、plan 与 roadmap 的 bookkeeping diff
不改变该 candidate source commit/tree。

Replay2 的 historical commit → replay commit 映射逐项保持 tree 相等：

| stage | historical commit | replay commit | tree |
|---|---|---|---|
| control | `178cf6c78a34e19d6081089e550333e49953a001` | `0da3896cd9a240e5e5837650a169e8974481ebbe` | `9ea94a6021e484afc2ad29cc4fc408b3c8a20182` |
| RED | `2f6227a2648945fee57fb36f33bde094a37da526` | `97cd330fb33c2658c0263b82976903ed279acc62` | `10afbbdf2973e199b663fa42ffbc6ad5e8721445` |
| oracle | `a7e04cc45881715c4426125f74e7e4b8bc178361` | `fdac8fea0298093c199531b4d1ed0419a2839b3d` | `0a389628e7a0b09cbb2e7f9a008e2e2f6edef767` |
| reducer/factor | `b794195ec579a025217f0778d029d79a993e9b74` | `31e3437b7aca12dd23cdf983997b59deef8b937a` | `003dd3d67286e2cffbae271e06b401684ed2cb1a` |
| estimator state | `313ab1752aaa14704877e8b7255c6b06af3654f2` | `0372b61edc8896fd95414e61556f5b1586bca8c6` | `024db63c0ef21cc9573ea561b16394490908e520` |
| lifecycle | `9d860e7739b5191aa054d9b172cb168f7f9ec6b6` | `9bdd32df4089774b667db721815bb89a3007a47b` | `732c8ea3e8a280693b58dac4211646f5a6a0e44a` |

### 2.3 Verifier 与 archive

| 项 | 冻结身份 |
|---|---|
| base verifier | `/home/lin/orca/verifiers/phad-m4-online-bias-synthetic-v1-634c4ec.sh`；mode `0444`；SHA-256 `f14f9c4993847414508d0a82edbb386bdbe5e878ced06144c0bf3e27797d367f` |
| verifier 2 | `/home/lin/orca/verifiers/phad-m4-online-bias-synthetic-v1-634c4ec-verifier-2.sh`；mode `0444`；SHA-256 `c7fdd4863a442df8bb1527b9d2dfc17ad0f0dc8c126f7964ffa9d53b61c03502` |
| verifier-2 freeze receipt | `/home/lin/orca/evidence/phad-m4-online-bias-v1-replay2-634c4ec/verifier-2-freeze-receipt.json`；mode `0444`；SHA-256 `a46a9bb366fb1a55d068fb3ef7e3f7e7b31d23c5c4159560ac921895512e764b`；independent review findings `0` |
| locked product source | SHA-256 `4552e5b8a4036a0dd43afb7b1f28538fc706e0813eed7c0a9724f8c16b57047a` |
| locked fixed-PIM region | SHA-256 `216e6fddad7473f371f09b3760a1f13c73666a6b6c1d38d9d601f63d2e02acdd` |

positive archive 的 HEAD/tree 等于 candidate；tracked files=`339`，identity manifest rows=`342`。
manifest 为 `replay2-archive-identities-v2.tsv`，mode `0444`，SHA-256
`135878379da910586193f57edc5b7517af7452c7b09e573971769e45ec3e2c0f`。archive root mode=`0555`、
writable entry count=`0`、seal exit=`0`。

两个冻结 test marker region 为：

| region | bytes | SHA-256 |
|---|---:|---|
| authority control | `10510` | `70e2c2fe15096c4db749160a698b91bc599a1e0455f3f404d71bcb4cc8c3975d` |
| no eviction | `5865` | `456df739dd87b433391829fa5d8bd793509e5a6e1a7d543a5bbfcb733bdd7cce` |

## 3. 已完成的 positive evidence

### 3.1 RED 与 stage boundary

| boundary | receipt actual |
|---|---|
| authority visual control | configure/build exit `0/0`；control `1/1 PASS`；estimator module `78/78 PASS`；canonical bytes `4138`，SHA-256 `d418cbcf3c8cc0f55a8bd12e69fc8ea72bea9cf9a37416f049d7a8e4d3ebe8de` |
| capability RED | configure exit `0`；build exit `1`；expected capability failure=`true`；首诊断为 `GyroBiasOptions has not been declared in phad::estimator` |
| oracle stage | configure/build exit `0/0`；oracle `3/3 PASS`；independence/link audit exit `0/0` |
| reducer/factor stage | configure/build exit `0/0`；targeted `10/10 PASS`；Q2 `10/10 PASS`；oracle `3/3 PASS` |
| estimator-state stage | build exit `0`；state targeted `5/5 PASS`；reducer/factor `10/10 PASS`；Q2 `10/10 PASS`；oracle `3/3 PASS` |
| lifecycle stage | build exit `0`；online+transaction `25/25 PASS`；unknown predecessor `1/1 PASS`；visual regressions `5/5 PASS`；off canonical bytes/hash 与 authority control 相同 |

### 3.2 Clean positive arm

positive arm scratch 为
`/home/lin/orca/scratch/phad-m4-online-bias-v1-replay2-634c4ec/arm.neskIu`。fresh restore、
configure、build、format、allowlist、static source、两个 marker、oracle independence 与 link audit
全部 exit `0`。测试 actual 为：

```text
independent oracle    4/4 PASS
online targeted      37/37 PASS
Q2 regression        10/10 PASS
estimator module    114/114 PASS
```

binary SHA-256 仅作 provenance，不作跨 scratch reproducible-build gate：estimator
`d2c0282ed140fd2d3eaf717015cba81ef7d9b06ffa68e6904dc34fced0ee9ab2`；oracle
`1fa07f73147aa1db41146067127ace2ba938d40edec6110466e24eac05e46019`。

这些 positive actual 证明对应命令在该 arm 通过；由于完整 negative qualification 与 final restore/
retest 未完成，它们不等于 overall `PASS`。

## 4. Negative arms：已完成与未运行

冻结协议共要求 `19` 条有效 negative arms。Replay2 有效完成并通过 `5` 条，未运行 `14` 条。

### 4.1 已有效完成：5/19

| arm | mutation / identity | configure/build/static/targeted actual | arm verdict |
|---|---|---|---|
| `deletion-reducer` | 删除 `gyro_interval_reducer.cpp`；before SHA-256 `a7d4546f4068edf7b306cbd8bf328c7fe279ba215eed7f8b6a37c0c34ef9f8f8` | configure `0`；build `1`，missing source / no known Ninja rule | `PASS` |
| `deletion-factor` | 删除 `gyro_rotation_factor.cpp`；before SHA-256 `98ab4f88a9c552dd0500e38a6d4cdef3862770c318e26beded0cf4c990971b8c` | configure `0`；build `1`，missing source / no known Ninja rule | `PASS` |
| `graph-activation` | `stereo_vo_estimator.cpp` before/after `4552e5b8a4036a0dd43afb7b1f28538fc706e0813eed7c0a9724f8c16b57047a` / `3d6c6309de976949009d213b9ccce54fb13f0cb5701dc737b20db9e633960a93`；patch SHA-256 `e45858ed1b12ee9498fac671601837bdb4cbed1234c0fd9766a5e45a89b088a6` | single hunk/configure/build `0/0/0`；NoEviction targeted exit `1`，`0/1 PASS` | `PASS` |
| `transaction-early-commit` | `stereo_vo_estimator.cpp` before/after `4552e5b8a4036a0dd43afb7b1f28538fc706e0813eed7c0a9724f8c16b57047a` / `8b1eaf48e974aeb8382ea26f8688ccca105fa62f0874d0e8739d3e65da36c051`；patch SHA-256 `05b6d92a8048056ae2842fdb32a2d0bc78ca5a997629e4ef7c956ffbae4ba876` | single hunk/configure/build `0/0/0`；static exit `128`，首诊断 `product source SHA-256 mismatch`；unknown-predecessor targeted exit `1`，`0/1 PASS` | `PASS` |
| `transaction-wrong-swap` | transaction header before/after `36fae6496561489ce490d049df669fdf2bd5ce221deb922a0e417127089e5f4a` / `46c3ec3383bf4bddfddd37d3119e6544057f9f06b30196806d1f8fac03169a9f`；patch SHA-256 `8b58600f03b659b2e9b706a0828cbc1c56750ebd65c6923902b96807dc57a2e0` | single hunk/configure/build/static `0/0/0/0`；transaction targeted exit `1`，`0/4 PASS` | `PASS` |

### 4.2 STOP 后未运行：14/19

- `transaction-rollback-order`；
- `static-cap`；
- `numerical-factor-1-wrong-bias-sign`；
- `numerical-factor-2-ignore-candidate-bias`；
- `numerical-factor-3-rad-as-degree`；
- `numerical-factor-4-degree-as-rad`；
- `numerical-factor-5-visual-relative-inverse`；
- `numerical-factor-6-left-compose`；
- `numerical-frame-r-b-left`；
- `numerical-noise-1-density-as-covariance`；
- `numerical-noise-2-missing-square`；
- `numerical-noise-3-dt-division`；
- `numerical-noise-4-rw-variance-as-sigma`；
- `numerical-initializer-swapped-source`。

因此 final fresh restore、final rebuild 与 final positive retest 均未运行。`activation-deletion` 与
`verify-result` 两个 plan todo 都仍未完成。

## 5. 精确 STOP event

停止 arm label 为 `transaction-rollback-order-v2`，scratch 为
`/home/lin/orca/scratch/phad-m4-online-bias-v1-replay2-634c4ec/arm.c2K3OX`。fresh scratch exit=`0`，
fresh identity exit=`0`；identity 为 candidate commit/tree，status bytes=`0`。

实际错误命令为：

```text
PHAD_M4_ONLINE_BIAS_EVIDENCE=/home/lin/orca/evidence/phad-m4-online-bias-v1-replay2-634c4ec PHAD_M4_ONLINE_BIAS_ARCHIVE=/home/lin/orca/archives/phad-m4-online-bias-v1-positive-replay2-634c4ec PHAD_M4_ONLINE_BIAS_SCRATCH_ROOT=/home/lin/orca/scratch/phad-m4-online-bias-v1-replay2-634c4ec bash /home/lin/orca/verifiers/phad-m4-online-bias-synthetic-v1-634c4ec-verifier-2.sh marker-sha /home/lin/orca/scratch/phad-m4-online-bias-v1-replay2-634c4ec/arm.c2K3OX tests/estimator/stereo_vo_online_gyro_bias_test.cpp PHAD_M4_ONLINE_BIAS_CONTROL_BEGIN PHAD_M4_ONLINE_BIAS_CONTROL_END
```

错误 begin/end 参数是：

```text
PHAD_M4_ONLINE_BIAS_CONTROL_BEGIN
PHAD_M4_ONLINE_BIAS_CONTROL_END
```

它们同时漏掉冻结 token 的 `// ` 前缀与 `AUTHORITY_` 部分。正确冻结 marker 是：

```text
// PHAD_M4_ONLINE_BIAS_AUTHORITY_CONTROL_BEGIN
// PHAD_M4_ONLINE_BIAS_AUTHORITY_CONTROL_END
```

实际 marker gate exit=`51`，stdout/stderr 均为 `0` bytes。该错误发生时
`valid_arm_started=false`、`mutation_applied=false`、`configure_invoked=false`、
`build_invoked=false`、`targeted_test_invoked=false`。receipt 随即应用强制 STOP；未用重试覆盖该
真实 gate failure。

另有两项不计 gate 的 incident：archive manifest v1 在 process 启动前被 execution layer 拒绝，已由
完成并封存的 v2 替代；较早一次 rollback-order `fresh-scratch` 调用漏传三个冻结 root 环境变量，
在 scratch/gate/state change 之前 exit `1`，不计有效 arm。它们都不是本次最终 STOP event。

## 6. 未获得的权限与非结论

`permission_granted=false` 精确表示本次结果不授权：

- replay3 或任何自动重跑；
- 真实数据、GT、ATE、RPE 或 natural feedback；
- apps/session 接线、CLI/config parser、persistent artifact/config-hash 变化；
- default-on、bounded-online 或 production capability 声称；
- eviction information handoff 的 design/implementation；
- accelerometer、gravity、velocity、完整 `X/V/B` 或完整 VIO；
- M5 或历史 M4.2–M4.4 路线。

receipt 记录 Q3、real dataset、GT、ATE、RPE、full-repo unit 与 push 均未运行。当前只保留
default-off、无 real caller 的 candidate 与已封存 provenance；M4 仍在进行中且未完成。

## 7. Evidence index 与保全状态

evidence log index 为
`/home/lin/orca/evidence/phad-m4-online-bias-v1-replay2-634c4ec/replay2-hard-error-log-index.tsv`，
mode `0444`，indexed files=`383`，含 header rows=`384`，SHA-256
`da2d44a2276a4efa58dec24a0e9977b531b3df37e86afce494927c54e53b3070`。

receipt 写入前共有 `129` 个 exit files：`120` 个 zero、`9` 个 nonzero；其中 `7` 个为预期 RED/
有效 negative kill，`2` 个为上述 incident/HARD_ERROR。STOP scratch 未删除且保持 clean；attempt 1
输出未被复用为 actual，历史 cross-binary receipt 也未被拿来替代 replay2 证据。

实施步骤与未完成 todo 见
[implementation plan](../plans/2026-08-18_m4_online_gyro_bias_synthetic_1e3569b4.plan.md)；M4 总状态见
[roadmap](../design/roadmap.md)。
