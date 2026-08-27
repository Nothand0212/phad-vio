# M4 gyro Q1 Observe 资格结果

日期：2026-08-12

状态：**final independent PASS**

## 结论

**Q1 Observe PASS。** 在 clean commit
`74270572cc1fcc2eac82559117efd0951c800ab9` 上，fresh Release build、portable tests、
MH_01 qualification、standalone Observe run、完整输入 manifest、离线 replay 与 control
byte gate 均通过。gyro packet summary 和逐样本数据已确定性落盘；gyro 仍未进入 estimator、
state、initialization、factor、posterior 或 feedback，M3 visual output 与权威 control 的
`est.tum`、`kf.tum`、`diag.csv` 逐字节相同。

因此 Q2 的状态是 **go to plan only**：可以另建并评审 known-bias synthetic integration 的
独立 Q2 计划，但 Q2 尚未实施，本任务也没有授权开始 Q2 编码、测试或运行。该结论只放行
[证据门控](../agents/evidence-gated-integration.md)的下一次规划，不放行真实序列 bias fit、
factor 或 posterior。

上位与关联文档：

- [资格实验设计](m4-minimal-gyro-slice-design.md)
- [Q1 实施计划](../plans/2026-08-12_m4_gyro_q1_observe_7d3a91e6.plan.md)
- [MH_01 权威控制组](m4-minimal-gyro-mh01-control.md)
- [证据门控的信息接入](../agents/evidence-gated-integration.md)

## 1. 证据身份与路径

### 1.1 Source 与 fixed point

以下是独立 verifier 开始和结束时均成立的事实：

```text
source_commit=74270572cc1fcc2eac82559117efd0951c800ab9
git_tree_object=e4379bf44db8d1127450d674b60b2e451fbe0aef
git_ls_tree_sha256=bb5a7854e4a3a89a52c8c0332e965474b8b9b2f2d4cbe0b83bcd7224e35f94ec
git_dirty=false
design_review_fixed_point=a8e892fcccbe
m3_runtime_anchor=7026ebf
q1_implementation_base=8506378639a6ef8709b615cab9855cbdd0d54f88
```

`git_tree_object` 是 Git 在该 commit 记录的 tree object id；`git_ls_tree_sha256` 是
`git ls-tree -r HEAD` 文本输出的 SHA-256，二者语义不同。final verifier 的
`git status --porcelain=v1` 为空。`8506378..HEAD` 的 source/test 差异严格落在 Q1 plan
allowlist 内；唯一同时出现的文档差异是实施期间更新的 Q1 plan：

```text
M CMakeLists.txt
A apps/gyro_observe_writer.cpp
A apps/gyro_observe_writer.hpp
M apps/offline_vo_session.cpp
M apps/offline_vo_session.hpp
M apps/phad_vo_bench.cpp
M docs/plans/2026-08-12_m4_gyro_q1_observe_7d3a91e6.plan.md
A tests/apps/gyro_observe_artifact_test.cpp
A tests/apps/gyro_observe_mh01_test.cpp
M tests/apps/offline_vo_session_test.cpp
M tests/apps/phad_vo_bench_cli_test.cpp
```

### 1.2 Fresh paths 与冻结输入

```text
repository_cwd=/home/lin/orca/workspaces/m4-minimal-gyro/tigerfish
fresh_build=/home/lin/orca/workspaces/m4-minimal-gyro/tigerfish/build-q1-final-verifier
dataset=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy
control=/home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control
output=/home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier
input_manifest_v2=/home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier_inputs_v2.sha256
input_manifest_v2_sha256=aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5
input_manifest_v2_lines=7372
```

目录本身没有伪造一个“目录 SHA”。fresh build 以实际执行的四个 ELF bytes 固定，control
以其 `meta.json` 与三主产物固定，standalone output 以 `meta.json`、`summary.json` 与第 4.1
节五个主/Observe artifact 固定：

```text
ec45469a213bf882ab962e8a8e769fdf0354851dae5d0412c395403ae7f888ee  /home/lin/orca/workspaces/m4-minimal-gyro/tigerfish/build-q1-final-verifier/phad_vo_bench
1a72448c01375d726843a7d75f960da01a1afe56ce42a86d8724fa2a0ef6dbc4  /home/lin/orca/workspaces/m4-minimal-gyro/tigerfish/build-q1-final-verifier/phad_apps_tests
145d28236796c902f6f88dcd037ebb136356762ecfa2d04ba94ec55d5520d94e  /home/lin/orca/workspaces/m4-minimal-gyro/tigerfish/build-q1-final-verifier/phad_apps_mh01_test
eecf16f73f41ce00d0dd72f4721c9658839e3e88bb762287da5907ea9001177e  /home/lin/orca/workspaces/m4-minimal-gyro/tigerfish/build-q1-final-verifier/phad_sync_tests
115f6c615164178a5c10770becf261e4b78695f0bc95a2e96eee729512395e43  /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control/meta.json
bbeaa21edaee34733f61b8ef093d45d5b8f4268fc4c0a36da261ee6664e9fab1  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/meta.json
88f2cd76ac944aadc58bb6167736fc5884b0e84a43ae3c1d9914f47ffc9abd09  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/summary.json
```

manifest v2 包含实际消费的 8 个 calibration/index/GT 文件，以及 cam0/cam1 index
列出的全部左右 PNG；有意排除未消费的 Leica/body 文件和 `.DS_Store`。本次独立复核执行
`sha256sum -c`，7,372 项全部通过，exit `0`。dataset 是目录，没有把目录名冒充文件 hash；
其冻结身份由上述 manifest 文件 SHA 和 manifest 内逐文件 SHA 共同定义。

废弃的 v1 位于
`/home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier_inputs.sha256`。
EuRoC camera index 使用 CRLF；v1 未删除文件名末尾的 `\r`，导致 PNG 路径不存在，只留下
8 条非图像文件 hash，因此 v1 **不是**有效输入证据，也不得供后续阶段引用。

## 2. 独立 verifier 命令账本

除明确说明外，以下命令的 cwd 均为
`/home/lin/orca/workspaces/m4-minimal-gyro/tigerfish`，environment 为继承环境、无额外变量。
duration 是 verifier transcript 记录的 shell wall time；CTest/GTest 自报时间另列。只读
复核在 verifier 完成后再次命中相同结果，不改外部产物。

### 2.1 身份、fresh configure/build 与 tests

| 命令原文 | 显式 env | exit | count / 结果 | duration |
|---|---|---:|---|---:|
| `git rev-parse HEAD` | 无 | `0` | 精确 source commit | `<1 s` |
| `git status --porcelain=v1` | 无 | `0` | 0 行，clean | `<1 s` |
| `git diff 8506378..HEAD --name-status` | 无 | `0` | 上述 11 个路径 | `<1 s` |
| `git rev-parse 'HEAD^{tree}'` | 无 | `0` | `e4379bf...` | `<1 s` |
| `git ls-tree -r HEAD \| sha256sum` | 无 | `0` | `bb5a785...` | `<1 s` |
| `cmake -S . -B build-q1-final-verifier -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPHAD_BUILD_TESTS=ON -DPHAD_ENABLE_MH01_TESTS=ON` | 无 | `0` | fresh Release configure | 包含于 `0.159 s` 的首次 shell invocation |
| `cmake --build build-q1-final-verifier --target phad_sync_tests phad_apps_tests phad_apps_mh01_test phad_vo_bench -j2` | 无 | `0` | 4 个目标构建完成 | `42.368 s` |
| `build-q1-final-verifier/phad_apps_tests --gtest_filter='*GyroObserve*:*OfflineVoSession*:*VoBenchCli*'` | 无 | `0` | 31 passed、3 conditional skipped | 与下一命令合计 `2.163 s`；GTest `0.886 s` |
| `ctest --test-dir build-q1-final-verifier -L unit --output-on-failure -j2` | 无 | `0` | 66 passed、3 conditional skipped、0 failed | 同上；CTest real `1.38 s` |

fresh configure 所在的首次 shell invocation 也尝试生成 input manifest v1。该 invocation
最终因 configure 成功而 exit `0`，但其中 v1 的 PNG hashing 已报错；因此该 overall exit
不能把 v1 变成有效 manifest。v2 使用下面的修正版命令生成，shell exit `0`、duration
`3.566 s`：

```bash
{
  printf '%s\n' \
    /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/cam0/sensor.yaml \
    /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/cam1/sensor.yaml \
    /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/imu0/sensor.yaml \
    /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/cam0/data.csv \
    /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/cam1/data.csv \
    /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/imu0/data.csv \
    /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/state_groundtruth_estimate0/sensor.yaml \
    /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv
  tail -n +2 /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/cam0/data.csv | cut -d, -f2 | sed 's/\r$//' | sed 's#^#/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/cam0/data/#'
  tail -n +2 /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/cam1/data.csv | cut -d, -f2 | sed 's/\r$//' | sed 's#^#/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy/mav0/cam1/data/#'
} | LC_ALL=C sort -u | xargs -r -d '\n' sha256sum > /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier_inputs_v2.sha256
```

### 2.2 MH_01 qualification 与勘误

首次运行的是未带 dataset environment 的命令：

```bash
ctest --test-dir build-q1-final-verifier -L mh01 --output-on-failure -j2
```

它 exit `0`，但 3 个 test 全部 skipped，CTest real `0.08 s`（所在诊断 shell
`0.624 s`）；**不计为 qualification pass**。

之后两次带 env 的运行均实际执行并 3/3 PASS：

| 命令原文 | 显式 env | exit | count / 结果 | duration |
|---|---|---:|---|---:|
| `PHAD_EUROC_MH01_PATH=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy ctest --test-dir build-q1-final-verifier -L mh01 --output-on-failure -j2` | prefix assignment 如命令 | `0` | 3/3 passed、0 skipped；full CLI `93.29 s` | shell `93.146 s` |
| `env PHAD_EUROC_MH01_PATH=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy ctest --test-dir build-q1-final-verifier -L mh01 --output-on-failure` | `PHAD_EUROC_MH01_PATH` 如命令 | `0` | **正式门**：3/3 passed；full CLI `95.867 s`；label `97.82 s` | shell `97.683 s`，CTest real `97.83 s` |

证据质量勘误：SHA-256 已核验为
`25fdd6aefc340a3bf1f1513612e0abb6acf2015fdb69eb93e74b942570bad21a` 的 verifier
handoff 把第一次带 prefix assignment 的命令描述成“全 skip”。原始 verifier transcript
表明，全 skip 的其实是更早的**无 env**命令；两次带 env 的命令均 3/3 PASS。本结果使用
原始 transcript 和 `LastTest.log` 修正该措辞，不复制 handoff 中的错误归因。

### 2.3 Standalone 原命令

```bash
build-q1-final-verifier/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --gt-euroc /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --out /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier \
  --errors-csv \
  --gyro-observe
```

cwd 与 inherited env 如本节开头；exit `0`；shell duration `93.421 s`；
`summary.json.timing.wall_s=93.406634082`。该命令完整跑至 EOS。

### 2.4 Output、config、replay 与格式复核

以下有效复核命令均 exit `0`、各自 `<1 s`，除 manifest 全量复核约 `1.1 s`：

```bash
sha256sum -c /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier_inputs_v2.sha256

cmp /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control/est.tum /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/est.tum
cmp /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control/kf.tum /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/kf.tum
cmp /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control/diag.csv /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/diag.csv

jq -S '.config' /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control/meta.json > /tmp/q1-control-config.json
jq -S '.config' /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/meta.json > /tmp/q1-run-config.json
cmp /tmp/q1-control-config.json /tmp/q1-run-config.json
jq -r '.config_canonical_text' /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control/meta.json > /tmp/q1-control-canonical.txt
jq -r '.config_canonical_text' /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/meta.json > /tmp/q1-run-canonical.txt
cmp /tmp/q1-control-canonical.txt /tmp/q1-run-canonical.txt
jq -r '.config_hash' /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/meta.json | grep -Fx 402d1925

sha256sum \
  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/est.tum \
  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/kf.tum \
  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/diag.csv \
  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/gyro_packets.csv \
  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/gyro_samples.csv

git diff --check 8506378..HEAD
git diff --name-only 8506378..HEAD -- '*.cpp' '*.hpp' | xargs -r clang-format --dry-run --Werror
```

原始 transcript 中这些命令按 shell invocation 分组的 duration 为：SHA/三项 `cmp`
`0.537 s`；完整 config/canonical replacement gate `0.605 s`；packet/sample schema、无 temp、
`git diff --check` 与 clang-format gate `0.436 s`；独立 replay `0.531 s`；修正列号后的 segment
join 加最终 identity 复核 `0.612 s`；summary/bytes/首末行复核 `0.596 s`。进程观察、`sleep`
和 `tail` 等诊断命令不是资格门，不列作通过证据。

installed `jq` 不支持 `--argfile`，一次 helper 尝试 exit `2`；上面的 `jq -S` 加 `cmp`
完整 object 比较取代了它，不把失败 helper 计入 gate。

独立 replay 使用下面的原命令，只读两个 CSV，不读取 dataset，也不调用 product sync。
exit `0`，输出 `independent_replay packets=3682 samples=40491 last_packet=3681`：

```bash
awk -F, '
NR==FNR { if (FNR==1) {if ($0!="packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,sample_count,sum_dt_ns,interval_ns,status") exit 2; next}; n[$1]=$6; pre[$1]=$2; cur[$1]=$3; seg[$1]=$4; stat[$1]=$9; next }
FNR==1 { if ($0!="packet_index,sample_index,timestamp_ns,gyr_x_radps,gyr_y_radps,gyr_z_radps") exit 3; next }
{ if ($1<lastp || ($1==lastp && $2!=lasti+1) || ($1!=lastp && seen[$1])) exit 4; seen[$1]=1; cnt[$1]++; if (!(($3 ~ /^-?[0-9]+$/) && ($4 !~ /[Nn][Aa][Nn]|[Ii][Nn][Ff]/) && ($5 !~ /[Nn][Aa][Nn]|[Ii][Nn][Ff]/) && ($6 !~ /[Nn][Aa][Nn]|[Ii][Nn][Ff]/))) exit 5; if (cnt[$1]==1) fst[$1]=$3; lst[$1]=$3; if (cnt[$1]>1) sum[$1]+=$3-lastts[$1]; lastts[$1]=$3; lastp=$1; lasti=$2; rows++ }
END { for (i=0;i<=lastp;i++) { if (!(i in n) || cnt[i]!=n[i]) exit 6; if (stat[i]=="valid" && !(fst[i]==pre[i] && lst[i]==cur[i] && sum[i]==cur[i]-pre[i])) exit 7; } printf "independent_replay packets=%d samples=%d last_packet=%d\n",lastp+1,rows,lastp }' \
  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/gyro_packets.csv \
  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/gyro_samples.csv
```

segment join 使用 `diag.csv` 第 14 列 `segment_id`，exit `0`，输出
`diag_join_rows=3682`：

```bash
awk -F, 'NR==FNR { if (FNR>1) {ts[$1]=1; seg[$1]=$14}; next } FNR>1 { if (!($3 in ts) || seg[$3]!=$4) exit 2; rows++ } END {printf "diag_join_rows=%d\n",rows}' \
  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/diag.csv \
  /home/lin/Projects/data/phad-bench/MH_01_easy/7427057/q1_observe_final_q1_final_verifier/gyro_packets.csv
```

## 3. Runtime config 身份

`meta.json.config_hash` 为 `402d1925`。以下 `config` JSON 从 standalone
`meta.json` 原样提取，并已作为完整 JSON object 与 control 比较相等：

```json
{
  "estimator.block_culled_rebirth": true,
  "estimator.enable_outlier_cull": true,
  "estimator.enable_outlier_reopt": true,
  "estimator.enable_pnp_init": true,
  "estimator.enable_reanchor": true,
  "estimator.far_return_refresh_px": 6.0,
  "estimator.hanging_landmark_gate_m": 1.0,
  "estimator.huber_k_px": 3.0,
  "estimator.max_outlier_reopts": 3,
  "estimator.min_landmark_observations": 2,
  "estimator.min_pnp_inliers": 10,
  "estimator.min_seed_observations": 10,
  "estimator.min_shared_landmarks": 10,
  "estimator.min_track_observations_for_seed": 1,
  "estimator.outlier_avg_reproj_px": 4.0,
  "estimator.pnp_confidence": 0.99,
  "estimator.pnp_reproj_px": 2.0,
  "estimator.prior_rotation_sigma_rad": 0.0001,
  "estimator.prior_translation_sigma_m": 0.0001,
  "estimator.stereo_sigma_px": 1.0,
  "estimator.use_constant_velocity_init": true,
  "estimator.window_size": 10,
  "eval.max_dt_ms": 2.5,
  "eval.min_match_rate": 0.5,
  "eval.rpe_delta_s": 1.0,
  "session.dataset_format": "euroc",
  "session.drop_culled_tracks": true,
  "session.skip_drop_min_culled": 4,
  "session.zombie_drop_age": 5,
  "tracker.forward_backward_px": 0.5,
  "tracker.lk_pyramid_levels": 4,
  "tracker.lk_window_px": 21,
  "tracker.mask_radius_px": 20,
  "tracker.max_depth_m": 25.0,
  "tracker.max_epipolar_px": 1.5,
  "tracker.max_tracks": 200,
  "tracker.min_depth_m": 0.3,
  "tracker.min_disparity_px": 2.0,
  "tracker.min_distance_px": 20.0,
  "tracker.quality_level": 0.003,
  "tracker.stereo_bidir_px": 0.5,
  "tracker.stereo_check_bidir": true,
  "tracker.stereo_row_tol_px": 0,
  "tracker.stereo_sad_half_win_px": 7,
  "tracker.stereo_uniq_ratio": 0.5
}
```

以下 `config_canonical_text` 同样从 standalone `meta.json` 原样提取，并与 control
完整 string 相同：

```text
estimator.block_culled_rebirth=true
estimator.enable_outlier_cull=true
estimator.enable_outlier_reopt=true
estimator.enable_pnp_init=true
estimator.enable_reanchor=true
estimator.far_return_refresh_px=6
estimator.hanging_landmark_gate_m=1
estimator.huber_k_px=3
estimator.max_outlier_reopts=3
estimator.min_landmark_observations=2
estimator.min_pnp_inliers=10
estimator.min_seed_observations=10
estimator.min_shared_landmarks=10
estimator.min_track_observations_for_seed=1
estimator.outlier_avg_reproj_px=4
estimator.pnp_confidence=0.98999999999999999
estimator.pnp_reproj_px=2
estimator.prior_rotation_sigma_rad=0.0001
estimator.prior_translation_sigma_m=0.0001
estimator.stereo_sigma_px=1
estimator.use_constant_velocity_init=true
estimator.window_size=10
eval.max_dt_ms=2.5
eval.min_match_rate=0.5
eval.rpe_delta_s=1
session.dataset_format=euroc
session.drop_culled_tracks=true
session.skip_drop_min_culled=4
session.zombie_drop_age=5
tracker.forward_backward_px=0.5
tracker.lk_pyramid_levels=4
tracker.lk_window_px=21
tracker.mask_radius_px=20
tracker.max_depth_m=25
tracker.max_epipolar_px=1.5
tracker.max_tracks=200
tracker.min_depth_m=0.29999999999999999
tracker.min_disparity_px=2
tracker.min_distance_px=20
tracker.quality_level=0.0030000000000000001
tracker.stereo_bidir_px=0.5
tracker.stereo_check_bidir=true
tracker.stereo_row_tol_px=0
tracker.stereo_sad_half_win_px=7
tracker.stereo_uniq_ratio=0.5
```

## 4. Standalone 输出与 control byte gate

### 4.1 五个完整文件 SHA256 与三个 `cmp`

```text
18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321  est.tum
4a4a1ba8f7fb2729e482d5aca644c9195ad605ccc0e94ffb9e1e670d1f898bb9  kf.tum
1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb  diag.csv
fbf574545e418fc19d100b7b05f79546a5336420bca60852770605b42702a3fe  gyro_packets.csv
da35227b40a1ab47c94217b4210b5445d11927a864d6634f8b824812ccec0344  gyro_samples.csv
```

control → standalone 的 `est.tum`、`kf.tum`、`diag.csv` 三个独立 `cmp` 均 exit `0`。
前三个 SHA 也精确命中权威 control。完整 `config` object、canonical string 和 hash gate
均通过，因此 `--gyro-observe` 没有改变 M3 config identity 或三个主输出。

### 4.2 Summary、warning 与关键指标

```text
status=completed_with_warnings
warning_count=1
warning=vo pnp summary: pnp_successes=3670 pnp_fallbacks=10
image_frames=3682
ok=3681
poses_written=3681
rejected=1
failed=0
segments=1
keyframes=662
track_only_frames=3020
completion=0.9997284084736556
coverage=1.0
ATE translation RMSE=0.08096405792439258 m
ATE rotation RMSE=2.0140355174028675 deg
RPE(1 s) translation RMSE=0.017781218595500668 m
RPE(1 s) rotation RMSE=0.15151944370189444 deg
low_connectivity=2
pnp_successes=3670
pnp_fallbacks=10
outliers_culled/unique=5/5
reanchors=0
wall_s=93.406634082
rtf=1.970416787638722
```

唯一 warning 与权威 control 已记录的健康运行 warning 相同，不是 Observe contract
failure。

## 5. Packet/sample artifact 合同与 replay

### 5.1 Header、行数、bytes、状态与范围

`gyro_packets.csv` header：

```text
packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,sample_count,sum_dt_ns,interval_ns,status
```

`gyro_samples.csv` header：

```text
packet_index,sample_index,timestamp_ns,gyr_x_radps,gyr_y_radps,gyr_z_radps
```

| 项 | `gyro_packets.csv` | `gyro_samples.csv` |
|---|---:|---:|
| data rows | `3682` | `40491` |
| total lines（含 header） | `3683` | `40492` |
| bytes | `278808` | `3618112` |
| first timestamp | `t_cur_ns=1403636579763555584` | `1403636579763555584` |
| last timestamp | `t_cur_ns=1403636763813555456` | `1403636763813555456` |

packet status 全量计数：`first_zero=1`、`valid=3681`、`gap=0`、
`empty_nonfirst=0`。首 packet 为
`0,1403636579763555584,1403636579763555584,0,0,0,0,0,first_zero`；末 packet
为 `3681,1403636763763555584,1403636763813555456,0,0,11,49999872,49999872,valid`。

### 5.2 脱离 dataset 的 replay 规则与结果

replay 只消费两个 CSV 与 `diag.csv`，规则为：

1. packet index 必须从 0 连续；sample 按 packet 分组且 sample index 从 0 连续；
2. sample group 数必须等于 packet `sample_count`，gyro 三轴必须 finite；
3. `valid` packet 的首末 sample 必须精确等于 `t_prev_ns/t_cur_ns`；按顺序重算的
   `sum_dt_ns` 必须等于 `interval_ns == t_cur_ns - t_prev_ns > 0`；
4. `first_zero` 必须是 index 0、零区间、零 samples、无 gap；
5. 以 packet `t_cur_ns` join `diag.csv.timestamp_ns`，并比较
   `vo_segment_id == diag.csv.segment_id`。

结果：3,682 个 packet index、40,491 个 sample row、所有 finite/count/endpoint/Σdt/
interval 检查通过；3,682 个 timestamp + segment join 全部通过，segment id 均为 0。
output 目录没有任何 `*.tmp`。这些事实证明成功 run 的两个同级 artifact 都已 publish，
但不把两个独立 rename 误称为跨文件事务。

## 6. 判定矩阵

| Q1 硬条件 | 独立证据 | 判定 |
|---|---|---|
| fixed point、source 身份与 clean | exact commit/tree/fingerprint；开始与结束均 clean；allowlist diff | PASS |
| 正例、边界、local-invalid、hard-fail | 定向 31 passed / 3 conditional skipped；unit 66 passed / 3 conditional skipped；writer failure MH_01 test PASS | PASS |
| MH_01 到 EOS、双 CSV 可重放 | 正式 MH_01 3/3；standalone exit 0；3,682 packets / 40,491 samples；replay/join exit 0 | PASS |
| config identity 不变 | `402d1925`；完整 JSON object 与完整 canonical string 相同 | PASS |
| M3 三主产物 byte-identical | 三个 `cmp` exit 0；三个 SHA 精确命中 control | PASS |
| hard failure / local-invalid | hard failure 0；gap 0；empty_nonfirst 0；无 temp 残留 | PASS |
| 总结论 | 上述条件全部满足 | **Q1 final independent PASS** |

## 7. 事实、推断与剩余风险

### 已验证事实

- 当前 commit clean；fresh build 与正式 MH_01 qualification 通过。
- Observe 开启时发布两个完整 artifact；其 bytes、schema、行数、时间范围和 replay 合同成立。
- M3 config identity 和三主产物相对权威 control 不变。
- Q1 没有启用 bias、PIM、factor、state、posterior、initialization 或 feedback。

### 受证据支持的推断

- 对 MH_01 和该 source/input/config 固定点，Q1 新增的采集/落盘边不会改变 visual output；
  该推断由 byte gate 支持，但不外推到未跑序列或不同平台。
- 冻结 CSV 足以供后续独立计划在不重读 EuRoC IMU、不重做 sync 的前提下重放 Q1 packet；
  它不证明 gyro integration 数学正确，那正是 Q2 的待证问题。

### 未跑项与剩余风险

- 未跑 EuRoC 11/11、Sanitizer、性能专项；`summary.json` 的 wall time/RTF 是记录值，
  不是性能资格门。
- 未运行 Q2 或任何更高资格层；没有 synthetic integration、真实 bias fit、factor 或
  posterior 证据。
- fixed point 的 `phad/sync/stereo_pair_synchronizer.cpp` 在 Observe collector 之前仍有
  extreme-int64 timestamp subtraction 未检查风险。MH_01 未触发，本次也未修改该路径；
  因而 Q1 PASS 不能证明全 int64 域安全。
- 结论固定于本页记录的 source commit、manifest v2、control、config 与 artifact SHA；
  任一字节变化都需要新的资格 run。

本任务没有修改 source/tests/CMake，没有回写外部 output/control/manifest，没有执行
commit、push、issue 写、发布或部署。
