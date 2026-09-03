# M5 recovery engineering matrix preflight

- 日期：2026-09-02（Asia/Shanghai）
- Issue：[#51](https://github.com/Nothand0212/phad-vio/issues/51)
- 状态：**执行前检查完成；正式矩阵已按冻结协议执行完成**
- 协议：[M5 recovery engineering matrix](2026-09-02-note-m5-recovery-engineering-matrix.md)
- 结果：[M5 recovery engineering matrix results](2026-09-02-note-m5-recovery-engineering-matrix-results.md)

## 1. 角色源码身份

首次preflight快照证明两个角色各有488个canonical source entries，HEAD均为
`643fd9095e6bc753868814e3d9393d9e549a5066`、tree均为
`43a0f149bc66320230b849c5ec26e76a81140930`：

| role | source ID |
|---|---|
| `C_engineering` | `sha256:2b8330ad8848671a5049d35d9e4902a3c990a508d3528e05e527315f3cee811a` |
| recovery candidate | `sha256:dc4ac846d4b023878dff3edb018d130ebbebfe07a981f4a1504c716ba4767aa8` |

Recovery-only patch SHA-256为
`fb3e19cd244fc167f308ad69650e99bf10358be752a8fdd3480917dbe86997d4`。
Canonical manifest diff证据位于
`/tmp/phad-m5-remote-matrix.8m7SWE/source-manifest-diff.json`，SHA-256为
`cc93722e5e76c2344f7f92afd348755078fc84234e24ebda34250d2c73555ec7`。
差异精确限定为以下6个regular files，mode均保持`0664`，其余tracked/untracked、
mode与symlink entries一致：

- `apps/probe_b_writer.cpp`
- `phad/estimator/internal/vio_estimator_test_access.hpp`
- `phad/estimator/types.hpp`
- `phad/estimator/vio_estimator.cpp`
- `tests/apps/offline_vo_session_test.cpp`
- `tests/estimator/stereo_vo_pnp_test.cpp`

正式run会在包含本preflight记录的最终快照上重新计算source IDs；仍须满足同一六路径
manifest-diff谓词，不能直接复用上表ID。

## 2. 输入与远端身份

`scripts/remote_ci.py doctor`已通过固定ED25519 fingerprint核验，并报告远端
x86_64、32 CPU、98,741,403,648 bytes memory、Docker 29.1.3及完整EuRoC-11。
远端`/home/lin/data/euroc/native`共20,307,274,497 bytes的全部regular files已以
8路只读SHA-256扫描。Canonical per-sequence manifests保存在
`/tmp/phad-m5-remote-matrix.8m7SWE/remote-dataset-manifests.json`，其SHA-256为
`ade6e76e36731366b29eaa3c7cb99febdc29f02311b1bab0cbca2b7d6db452d3`。
每条sequence均记录relative path/type/mode/size/content SHA-256，并单独记录
cam0/cam1/imu0/ground-truth的`sensor.yaml`、`data.csv` header与row count。

Structural identities单列：

- V1_02-118 manifest SHA-256：
  `a80c02d6c9bd9e5cd76412f30b6859fd8acdabf0885de1cba494725a183ced8f`。
- TUM-VI corridor1：`camchain.yaml=3a0c0285f4d0a3a1678053b2e5912e3e250e78ac09bd009f6d7d8bba669d5703`，
  `imu_config.yaml=e309d3f4b5e6e812580fcb9c9f402ab79f4a09931e41f433e7038301118683a1`。
  当前无TUM-VI GT adapter，故只作structural sentinel，不进入ATE/RPE。

## 3. Postprocessor与输出布局

预注册时的paired postprocessor位于
`/tmp/phad-m5-remote-matrix.8m7SWE/compare_matrix.py`，冻结SHA-256为
`40b5d30928d678551c6be3d90c5d84d32ced05f0e6f8c3312175c52bccac9746`。
它复现project的nearest association、fixed-scale SE3 ATE alignment与1 s / 25 ms
RPE partner。对calibration run的11/11 self-test中，所有ATE/RPE translation和
rotation paired delta严格为0；summary最大复现误差为
`1.0130979388733863e-11`，低于预注册`1e-10`检查值。Self-test JSON SHA-256为
`2dbeb73bca5ece1dc2227d6ef7a4ab5ba9c9bc76141f13f6710aa42cbbbe9744`。

矩阵执行后，同一路径上的postprocessor按原协议补齐baseline/candidate distribution
的`count/p10/p90`，当前schema v2 SHA-256为
`267f55e9789a024ae67bf17aa93e715790db00921c629ef312e1e50fe0edf8cf`；
算法、association、alignment、RPE partner与判断边界均未变化。三份v2结果移除新增
字段和provenance后与冻结v1逐字等价，additive equivalence检查通过；v2 self-test
SHA-256为`9dab80dc614aed6416ddcf19c0f36ae997890c7b797bd13696fd332ffc22f4a9`。

正式输出分别置于隔离的engineering/candidate source root下：
`artifacts/remote-ci/runs/<run-id>`。每个run必须`wait`到终态并完成fetch，
再启动下一个；固定次序是
`engineering-1 → candidate-1 → candidate-2 → engineering-2 → engineering-3 → candidate-3`。

## 4. Calibration与判断边界

Remote run
`20260902T083946Z-ci-euroc11-2b8330ad8848-fa1fe3`仅用于验证build、unit、
EuRoC-11 profile与postprocessor，不计入3-repeat工程矩阵。随后发起的candidate
本地start在创建run-id/remote worker前中断，只留下ignored local cache partial，
不构成remote matrix run。

当前没有经产品风险与measurement/evaluator resolution共同批准的practical margin。
矩阵结论因此只允许是`directional and repeatable`、
`mixed across sequences/metrics`或`engineering significance unresolved`。
Exact/formal状态单独记录，不作为工程候选的单点否决。
