# M4 gyro Q3 offline constant-bias alignment 资格结果

日期：2026-08-18

状态：**final `HYPOTHESIS_FAIL`；权限 `STOP`**

## 结论与权限

Q3 V1 在冻结 implementation、协议、输入和命令上完成了**唯一一次**真实资格运行。该运行不是
`HARD_ERROR`，而是完整发布 artifact 后得到的科学非通过结果：support 充分，`full/early/late`
三个 fit 都为 `solved`，rank、condition、prefix nonlinear loss、validation aggregate reduction、
improved-block fraction 与 per-axis non-worsening 均通过；唯一失败项是
`HALF_STABILITY`：

```text
half_bias_max_abs_diff_radps = 0.001227119335357879
frozen upper threshold       = 0.001 rad/s
verdict                      = HYPOTHESIS_FAIL
exit_code                    = 2
reason_codes                 = ["HALF_STABILITY"]
```

事实层面的含义是：冻结协议用前、后各约 50 s 的 eligible prefix 独立拟合后，两个 nuisance bias
estimate 的最大单轴差超过预注册上限。它不证明 physical gyro bias，不证明或否定完整 VIO，且不提供
online initializer、factor、posterior 或产品收益结论。

权限结论为 **STOP**：不授权 Q4 plan 或 implementation，不授权 online initialization、factor、
posterior 修改，不授权调阈值后重跑，也不授权第二次 Q3 qualification。真实运行未读取或评估
GT、ATE、RPE；本文不声称 VIO 指标改善。

关联权威文档：

- [M4 最小 gyro 总体资格设计](m4-minimal-gyro-slice-design.md)
- [Q3 可执行设计](m4-minimal-gyro-q3-offline-bias-alignment-design.md)
- [Q3 immutable 实施计划](../plans/2026-08-13_m4_gyro_q3_align_0e897ba8.plan.md)
- [Q1 Observe 资格结果](m4-minimal-gyro-q1-observe-result.md)
- [Q2 known-bias predict 资格结果](m4-minimal-gyro-q2-known-bias-predict-result.md)
- [证据门控规则](../agents/evidence-gated-integration.md)

## 1. Protocol、docs 与 implementation 身份

### 1.1 Protocol 与冻结七文档

| 项 | 值 |
|---|---|
| protocol ID | `PHAD-M4-Q3-GYRO-ALIGN-V1` |
| docs / design commit | `16affcd2fc271f1db4d6a060b3d2864004d8a40b` |
| docs / design tree | `634b048fd9501a9c807ebfcc645f5ce9fe4d8bb3` |
| design blob | `a6eaa88b4553cb03f012bd090d502bc3ff3576e3` |
| design SHA-256 | `1e2f20d181af5453b383ceb09d53850045e7b68ec17b13f8d40c883d417c081b` |
| seven-doc aggregate SHA-256 | `5066eda2df21a9e911758be18472f6efb4235f28763f148d9103726b89fcba9f` |
| write-once protocol receipt | [`protocol_identity.json`](../../build/q3-preflight-16affcd2fc271f1db4d6a060b3d2864004d8a40b/protocol_identity.json) |
| receipt mode / SHA-256 | `0444` / `2214de455e56f18df07bb884cef5bbc8929ca2baba21ca8bb2f558727c3578ef` |

七文档 identity 如下；implementation `HEAD` 中的对应 entries 与 receipt 逐项一致：

| path | git blob | SHA-256 |
|---|---|---|
| `docs/plans/2026-08-13_m4_gyro_q3_align_0e897ba8.plan.md` | `92db32acad1a090054317a5daecadc54094119da` | `34a69ebda34cb5bb1121f81a68e77bef12de393171fe7ef16115fff12a4ccc9b` |
| `docs/research/m4-minimal-gyro-q3-anchor-contract-record-audit.md` | `cf082aff142a9653db61753277a19c4a382f7b37` | `d2bd3e898d70720ff2aa14fa4b3ab6ecb60584520a04fc8e37bff0e81d7ec004` |
| `docs/research/m4-minimal-gyro-q3-offline-bias-alignment-design.md` | `a6eaa88b4553cb03f012bd090d502bc3ff3576e3` | `1e2f20d181af5453b383ceb09d53850045e7b68ec17b13f8d40c883d417c081b` |
| `docs/research/m4-minimal-gyro-q3-offline-bias-alignment-research.md` | `aaccc69a91ad6c0bbcb98c990cf7f5d8b5139934` | `71b422b589d718111341b6f3817954b2bafc61635d50ecdce74fef053565358c` |
| `docs/research/m4-minimal-gyro-q3-outcome-independent-budget-research.md` | `2e3470b582073918f471cac56111cd7ec788f688` | `9c9448960886dced5904579021be7c6fadcc527f483cd688c88bc29e02c8af6a` |
| `docs/research/m4-minimal-gyro-slice-design.md` | `3e1d0d479880765fb1bd93bffdaf43d9f1e60c56` | `7d0a03207d38c356a28987a51587acdd96abdc97ba2c4e845a73d1aa7d3a0084` |
| `docs/design/roadmap.md` | `e40f8392a90c24491977114c1d9a0b12f917be36` | `54e967f5695c85f2c97697a91faeff999ed73d8691f626f6cb1a2681575780b8` |

当前 protocol receipt supersede 旧 `d59a73d...` receipt（SHA-256
`b9d9bb7787e866029306b3b7c807a991cc6ca84db54564c76e43b0b9356fd6f1`）；原因是 runner protocol
amendment 与 roadmap link correction。旧 receipt 原样保留，但对本次资格没有 authority。

### 1.2 Implementation、binary 与 GREEN

| 项 | 值 |
|---|---|
| implementation commit | `b040676e924b9b8805809f37328c040025aec0cd` |
| implementation tree | `8ec81584233fb1d66ad45a9b5f7fe66028695f93` |
| commit subject | `feat(m4): add Q3 gyro alignment runner (#39)` |
| qualification 前 worktree | clean |
| analyzer build identity | `Release`; `GNU 13.3.0`; source commit/tree 同上 |
| binary | `build/q3-green-final-v3-b040676e924b9b8805809f37328c040025aec0cd/release/phad_gyro_align` |
| binary SHA-256 | `19c5058de35f62f6f29fbaf8cd0283d4eee7a372fbe063f5c8244d3716778d38` |
| ELF build ID | `121316fd32e8d26b3fc8800bb5d7c54d86cbe050` |

五个 production 文件的 GREEN identity：

| path | SHA-256 |
|---|---|
| `apps/gyro_alignment.hpp` | `092cc83b3817e992e756bb6400b32ec0c84c678bcc7037a2811cf787d61c4704` |
| `apps/gyro_alignment.cpp` | `68fe6462105e00e79766422d3a1bd484e0a5b2df280db67488f527c4fd06bfbd` |
| `apps/gyro_alignment_runner.hpp` | `8be9315a04343e2198c5932634b360a0b6bbeaa2f76b7f572cc5c8a4956ca3fe` |
| `apps/gyro_alignment_runner.cpp` | `ada6dc0ced26a7f5be87ea0a79a4ef263b75106ad0f09a9aace3925c091beb9e` |
| `apps/phad_gyro_align.cpp` | `3b572039908600201826e9207faf0e82f8cabcb35c6ee15598c88aafb423678b` |

权威 GREEN 是 [`v3 verification_receipt.json`](../../build/q3-green-final-v3-b040676e924b9b8805809f37328c040025aec0cd/verification_receipt.json)，
mode `0444`，SHA-256
`42730ae00113c2c1a4dec00d2968c71e8ef8a71083b5605dcccf3487d7f0913d`，状态 `PASS`。其中记录：
Q3 binary `61/61`、定向 CTest `61/61`、unit `432/432`、format/diff/link isolation 与 repo 外
deletion semantic pair 全部 PASS。

两个更早的 GREEN verifier harness failure 原样保留：v1 receipt SHA-256
`f0240b9796009e0d3a0f8b165efca36d0acb3a51e5b423e525bd94d12091f093`，原因为
`probe_harness_fail_wrong_test_path`；v2 receipt SHA-256
`8f2ecfe5237a294a08e92be1fcdc235bd3c28a8c156d1e3c06688ffc37279589`，原因为
`archive_missing_identity_cmake_input`。两者 `product_attribution=none`，不替代 v3 PASS。

### 1.3 Upstream provenance

artifact 原样记录的 Q1/Q2 provenance 为：

```text
q1_source_run=q1_observe_final_q1_final_verifier
q1_source_commit=74270572cc1fcc2eac82559117efd0951c800ab9
q1_git_tree_object=e4379bf44db8d1127450d674b60b2e451fbe0aef
q1_git_ls_tree_sha256=bb5a7854e4a3a89a52c8c0332e965474b8b9b2f2d4cbe0b83bcd7224e35f94ec
q1_meta_sha256=bbeaa21edaee34733f61b8ef093d45d5b8f4268fc4c0a36da261ee6664e9fab1
q1_config_hash=402d1925
q1_input_manifest_v2_sha256=aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5
q2_commit=d1c4385809a6bf461f1c6b8acd81870f98634aa0
q2_tree=9c9c991b21c4d40e8c5f2ce974334b76e1a42b64
```

## 2. Preflight 与唯一 one-shot 命令账本

### 2.1 Preflight correction chain

第一次 one-shot preflight STOP 是 harness 使用了不存在的非 `release/` binary path；没有执行 Q3。
该 [`v1 preflight receipt`](../../build/q3-one-shot-preflight-b040676e924b9b8805809f37328c040025aec0cd/preflight_receipt.json)
mode `0444`，SHA-256
`41da8f644803dbb45464b25d083d4e6abc49dda710c7a199bb14e81b5d9dbc14`，被归类为
`harness_wrong_binary_path`、`product_attribution=none`，并原样保留。

权威 [`preflight-v2 receipt`](../../build/q3-one-shot-preflight-v2-b040676e924b9b8805809f37328c040025aec0cd/preflight_receipt.json)
mode `0444`，SHA-256
`31bb9d7d6a5f1ced53d85072bad240f5373db0c770170290a8f6ea4b12d7c7a2`，状态 `GO`，明确记录
`q3_executed=false`。它验证了 implementation/docs/protocol/GREEN/binary identities、四输入 hash 和
output 尚不存在，并冻结了下节的 exact command。

### 2.2 run-v1：错误 command directive，STOP 且 count 0

run-v1 收到的 directive 漏掉 required `--out`：

```text
build/q3-green-final-v3-b040676e924b9b8805809f37328c040025aec0cd/release/phad_gyro_align /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q3_align_v1_b040676e924b9b8805809f37328c040025aec0cd
```

它与冻结 CLI grammar 和 preflight command 冲突，因此在执行前 STOP。事实记录为
`invocation_count=0`、`q3_executed=false`、output 未创建、artifact validation 未运行；不是一次 Q3
尝试，也没有消费 one-shot。权威
[`run-v1 qualification_receipt.json`](../../build/q3-one-shot-run-b040676e924b9b8805809f37328c040025aec0cd/qualification_receipt.json)
mode `0444`，SHA-256
`769f000e82779c0e5c79d4d86d9504c1e307c619c54c3d8170be5bf22759b835`，状态 `STOP`，分类
`harness_wrong_command_directive`，`product_attribution=none`，且已原样保留。

### 2.3 run-v2：唯一真实 invocation

exact command：

```text
build/q3-green-final-v3-b040676e924b9b8805809f37328c040025aec0cd/release/phad_gyro_align /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier --out /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q3_align_v1_b040676e924b9b8805809f37328c040025aec0cd
```

exact cwd：

```text
/home/lin/orca/workspaces/m4-minimal-gyro/tigerfish
```

receipt 与 `environment.txt` 记录的冻结 environment 字段：

```text
LANG=en_US.UTF-8
LC_ALL=C.UTF-8
TZ=
SOURCE_DATE_EPOCH=
CC=
CXX=
CMAKE_GENERATOR=
CMAKE_BUILD_TYPE=
```

| 项 | actual |
|---|---|
| `invocation_count` | `1` |
| `q3_executed` | `true` |
| start | `2026-08-18T20:21:04+08:00` |
| end | `2026-08-18T20:21:04+08:00` |
| process exit | `2` |
| stdout | empty，`0 bytes` |
| stderr | `gyro alignment scientific gates did not all pass\n` |
| receipt status | `COMPLETE` |

权威 [`run-v2 qualification_receipt.json`](../../build/q3-one-shot-run-v2-b040676e924b9b8805809f37328c040025aec0cd/qualification_receipt.json)
mode `0444`，SHA-256
`2f21903af2851b2516efc3dc1a4bb6bb9ccb8a3314d38379867c1659ff902b43`。run-v2 evidence files 的
SHA-256 为：

| evidence | SHA-256 |
|---|---|
| `command.txt` | `8c60adec773e956534c9b72fb19470cb96c819c08098d46edf2b386d3d9fe365` |
| `cwd.txt` | `febe409448b689265fe917df1a93ee3f7182b8b449e7aac563d1f2b15792f9bc` |
| `environment.txt` | `97243429c7f50c5b48eebf5ab40f7c3197d9a89f8434ccd84936b2d1d764a44a` |
| `start.txt` | `20b596734e7e59677e7f484d9d6fe8a047b53fe80414a65648f651aa14046bd1` |
| `end.txt` | `20b596734e7e59677e7f484d9d6fe8a047b53fe80414a65648f651aa14046bd1` |
| `exit.txt` | `53c234e5e8472b6ac51c1ae1cab3fe06fad053beb8ebfd8977b010655bfdd3c3` |
| `stdout.txt` | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| `stderr.txt` | `69fc1eba34300abe020137ffabcb3a499783b698f99a2865b2bfbdf85c3d73e9` |

## 3. 冻结输入与发布 artifact

输入目录：

```text
/home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier
```

四个输入的 size/hash 同时与 design、preflight、summary 和 manifest 对拍相等：

| order | input | size bytes | SHA-256 |
|---:|---|---:|---|
| 0 | `est.tum` | `602094` | `18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321` |
| 1 | `diag.csv` | `366627` | `1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb` |
| 2 | `gyro_packets.csv` | `278808` | `fbf574545e418fc19d100b7b05f79546a5336420bca60852770605b42702a3fe` |
| 3 | `gyro_samples.csv` | `3618112` | `da35227b40a1ab47c94217b4210b5445d11927a864d6634f8b824812ccec0344` |

输出目录：

```text
/home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q3_align_v1_b040676e924b9b8805809f37328c040025aec0cd
```

目录中只有三个 final 文件，无 temp 文件：

| output | size bytes | SHA-256 | schema / role |
|---|---:|---|---|
| `gyro_alignment.json` | `5548` | `063f23a4f8d0a619534056bd072d6cb1db9ebf2dd6859a9aa034a467ac2c21bd` | `phad.gyro_alignment.v1` |
| `gyro_alignment_blocks.csv` | `19590` | `5ad3bf097138e1d8fa3a6276a07c3bb5b0ae3d1674c574540df45a1fb3ea8bfe` | exact blocks CSV；84 data rows |
| `gyro_alignment.manifest.json` | `1470` | `bb9b89d81b9347bd84d98745e73b6e573870cec05b625c1c33bb90bc04fccff4` | `phad.gyro_alignment.manifest.v1` |

manifest 按协议列出两个 science outputs 与四个 inputs，所有 size/hash 与 final bytes 相等；manifest
自身不自哈希，因此由本 ledger 单独记录其 size/hash。manifest、summary 与 process exit 的 verdict
一致；protocol identity 与 preflight 相等，analyzer identity 与 binary build evidence 相等。

## 4. Ranges、eligibility 与 support actual

### 4.1 Sentinel-anchored ranges

```text
t0_ns                     = 1403636579763555584
t_mid_ns                  = 1403636629763555584
t_split_ns                = 1403636679763555584
validation_last_end_ns    = 1403636764763555584
```

### 4.2 Eligibility census

| field | actual |
|---|---:|
| `packet_total_nonfirst` | `3681` |
| `packet_structurally_eligible` | `3679` |
| `packet_fit_eligible` | `3679` |
| interval `gap` | `0` |
| interval `empty_nonfirst` | `0` |
| interval `diag_rejected_no_pose` | `2` |
| interval `segment_boundary` | `0` |
| interval `near_pi` | `0` |
| `validation_slots_total` | `85` |
| `validation_blocks_eligible` | `84` |
| block `incomplete_calendar_block` | `1` |
| block `near_pi` | `0` |

三条 protocol census 恒等式实际闭合：

```text
3681 = 0 + 0 + 2 + 0 + 3679
3679 = 3679 + 0
85   = 1 + 0 + 84
```

### 4.3 Support

| support | actual | frozen minimum | result |
|---|---:|---:|---|
| full eligible duration | `99900000000 ns` (`99.9 s`) | `90000000000 ns` (`90 s`) | PASS |
| early eligible duration | `49900000000 ns` (`49.9 s`) | `45000000000 ns` (`45 s`) | PASS |
| late eligible duration | `50000000000 ns` (`50.0 s`) | `45000000000 ns` (`45 s`) | PASS |
| eligible validation blocks | `84` | `60` | PASS |

`support.sufficient=true`。

## 5. Full / early / late fit actual

三个 sibling 均 `status=solved`、`rank=3` 且 `nonlinear_nonincrease=true`。rank 要求为 `3`；
singular-value nonzero 判定为 `sigma_i > 3*epsilon_double*sigma_max`；condition 要求 finite 且
`<=1e6`。三项实际 condition 都约为 `1.000002`，远低于冻结上限。

| fit | bias `[x,y,z]` rad/s | singular values descending | rank | condition | eligible duration / packets | prefix loss `zero -> fit` rad² | nonlinear |
|---|---|---|---:|---:|---|---|---|
| full | `[-0.002979854772897611, 0.021309696001635334, 0.07865762210282835]` | `[2.234942510096555, 2.23493883131049, 2.234937309110766]` | `3` | `1.0000023271282679` | `99900000000 ns` / `1998` | `0.03460000164746047 -> 0.001383564965607747` | PASS |
| early | `[-0.003250253372505292, 0.0210359161408832, 0.07927179464619838]` | `[1.579553373092583, 1.5795518068163619, 1.5795493852557478]` | `3` | `1.0000025246673971` | `49900000000 ns` / `998` | `0.017929947744318376 -> 0.0011210399502038036` | PASS |
| late | `[-0.0027099956441850467, 0.0215829302357044, 0.0780446753108405]` | `[1.5811322933176406, 1.581128970349336, 1.5811284576576352]` | `3` | `1.0000024259003035` | `50000000000 ns` / `1000` | `0.01667005390314205 -> 0.0002599064212817735` | PASS |

这些 bias 是 protocol 定义的 `visual_posterior_aligned_nuisance`，不是经过 physical-bias 资格化的
传感器参数。

## 6. Scientific gates 与最终 verdict

`gates.evaluated=true`。四个冻结 scientific gates 的完整 actual、阈值与判定如下：

| gate | actual | frozen comparator / threshold | result |
|---|---|---|---|
| aggregate reduction | `loss_zero=0.5465985048616602`; `loss_fit=0.000765362807896201` | `loss_zero>0` 且 `loss_fit <= 0.8*loss_zero = 0.43727880388932816` | PASS (`true`) |
| improved fraction | `blocks_improved=84`; `blocks_total=84`; `84/84=1` | `3*n_improved >= 2*n_blocks`，actual `252 >= 168`，即至少 `2/3` | PASS (`true`) |
| axis non-worsening x | `0.00021191414838153883 <= 0.0011387742007851686` rad² | `axis_loss_fit <= axis_loss_zero` | PASS |
| axis non-worsening y | `0.0002636513614285943 <= 0.04060497578728528` rad² | `axis_loss_fit <= axis_loss_zero` | PASS |
| axis non-worsening z | `0.0002897972980860679 <= 0.5048547548735897` rad² | `axis_loss_fit <= axis_loss_zero` | PASS |
| half stability | `max_abs_diff=0.001227119335357879 rad/s` | `<=0.001 rad/s` | **FAIL (`false`)** |

early/late bias 的逐轴绝对差为：

```text
x = 0.0005402577283202454 rad/s
y = 0.0005470140948212014 rad/s
z = 0.001227119335357879  rad/s
```

最大值来自 z 轴，超过上限 `0.00022711933535787907 rad/s`。因此：

```text
aggregate_reduction = true
improved_fraction   = true
axis_nonworsening   = true
half_stability      = false
all_passed          = false
verdict.status      = HYPOTHESIS_FAIL
verdict.exit_code   = 2
reason_codes        = ["HALF_STABILITY"]
```

`HALF_STABILITY` 是唯一 reason；不存在 support、rank、condition、observability、prefix nonlinear
loss、aggregate、fraction 或 axis reason。

## 7. 事实、解释与刻意未运行项

### 已验证事实

- 唯一真实 Q3 invocation 的 count 为 `1`；run-v1 在执行前 STOP，count 为 `0`。
- run-v2 在冻结 commit/tree、binary、四输入和全新 output path 上 exit `2`，完整发布了三个 final
  artifact；其 manifest、summary、receipt 与 final bytes 的 identity 和 verdict 一致。
- 完整 support 和三个 fit 都成立；四个 scientific gate 中只有 half stability 未达。

### 协议内解释

- `HYPOTHESIS_FAIL` 表示“这份冻结输入上的 frozen visual-proxy nuisance estimate 未同时满足所有预注册
  effect/stability gates”。其直接原因仅为两个 half-fit 的最大单轴差超过 `1e-3 rad/s`。
- aggregate、block fraction 与三轴 validation loss 的通过，不能覆盖 half stability failure；冻结协议
  要求四门全部通过。

### 刻意未运行与不可外推范围

- 未运行 GT、ATE、RPE，也未运行任何第二次 Q3 invocation。
- 未以结果调阈值、改输入、重选 run、重建 binary 或覆盖 output。
- 没有 Q4、online initialization、factor 或 posterior 证据；没有 VIO accuracy、robustness 或产品收益
  结论。
- 本 ledger 只记录既成 one-shot 结果，不改写冻结七文档，也不提供任何继续扩权。
