# M5 recovery engineering matrix 最终结果

- 日期：2026-09-02（Asia/Shanghai）
- Issue：[#51](https://github.com/Nothand0212/phad-vio/issues/51)
- 协议：[M5 recovery engineering matrix](2026-09-02-note-m5-recovery-engineering-matrix.md)
- Preflight：[M5 recovery engineering matrix preflight](2026-09-02-note-m5-recovery-engineering-matrix-preflight.md)

## 1. 结果摘要

本轮已完成预注册的 6 个 remote profile runs、66 个 EuRoC full sequence
processes（11 sequences × 2 roles × 3 independent processes）。6/6 run 的
hard gate 与 data identity 均通过；C_engineering 和 recovery candidate 各自
3 次运行均达到 exact repeatability。

三层结论保持独立：

| 层级 | 本轮结论 | 解释边界 |
|---|---|---|
| hard correctness | **PASS** | 66/66 full sequence processes 纳入矩阵，6/6 run hard/data identity 通过 |
| formal / exact | 两个 role 各自 3/3 exact repeatability；formal 状态仍由独立 formal matrix 决定 | role 内重复性不等于 candidate 与 C_formal 或 C_engineering 的 exact identity |
| engineering significance | **mixed across sequences/metrics** | 完整向量同时包含改善、退化和不变项 |

本轮协议未预注册 practical margin。上述 engineering state 只描述已观测的多序列、
多指标 trade-off，产品接受或否决状态保持待定。

## 2. 执行与身份

最终角色快照共享 base HEAD
643fd9095e6bc753868814e3d9393d9e549a5066 和 tree
43a0f149bc66320230b849c5ec26e76a81140930；remote artifact记录为detached
`HEAD`、`git_dirty=true`，canonical config hash均为`0337287b`。两个role的
dirty源码由下列完整source ID而不是短commit字符串区分。C_engineering source ID 为
sha256:84a5712939d2ad799768a866fe66102866d905537057d90fe0455f7d1529354c，
candidate source ID 为
sha256:f1d64eea76f8066cf6846f8888d05bb437f93461aa4a9477f6044f5141cf0b60。
两个 canonical manifest 各含 489 entries；identity verifier 确认 recovery patch
fb3e19cd244fc167f308ad69650e99bf10358be752a8fdd3480917dbe86997d4
构成两个 role 的全部源码差异。

差异路径精确为 apps/probe_b_writer.cpp、
phad/estimator/internal/vio_estimator_test_access.hpp、
phad/estimator/types.hpp、phad/estimator/vio_estimator.cpp、
tests/apps/offline_vo_session_test.cpp 和
tests/estimator/stereo_vo_pnp_test.cpp。

6 个 run 使用相同 OCI manifest
sha256:3cad515a1087a8b2e9ddb4f066780a9a8d881795c1b90e4dd1c55106feb8c370、
OCI config
sha256:2d780f2477a216467acb171ba28d3f84b0f861b97c57e82fb4a7940bb3a303ee
及 toolchain
a1418e0e9061e3dcc4714c246a65dac49ec105d7d1f5d7114aa71a0a544abd28。
远端数据 content tree 为
5fa70bb4262ff0f1f139294f07f36635b11c1f2dec1924345697e9210bc29d6a，
mode tree 为
4f93237c778d8e5b57e93b10b3e8cd360bf416f8e40fb2dbdad1eb988370b7d5，
local/remote content equality 为 true；每个 run 的 dataset verify artifact SHA
均为 65e03e6608a190087b8eb9dee9f20363a19e557f6e7890ef055b812742dba1ea。

| 执行序号 | role | run ID | terminal state | results archive SHA-256 | run verify SHA-256 | hard / data |
|---:|---|---|---|---|---|---|
| 1 | C_engineering-1 | 20260902T103541Z-ci-euroc11-84a5712939d2-fb5782 | warning | 5f2aec873a6ac4eb20dd90d0b2f2491c59adaeff5df8900b90c172a03da8e889 | fe717a706687247fe6dfd3c0abbcb015330135f6a78964e5f21cdde42f214075 | PASS / PASS |
| 2 | candidate-1 | 20260902T104948Z-ci-euroc11-f1d64eea76f8-c13422 | warning | caf82ccad851e2ec87fca33fffeba283cdfd75f7ec39cca13e5bcc2f5c684373 | 522da4baae41467a6df1196b38502ec00cd58f005477d75fcdb5677eca3a6345 | PASS / PASS |
| 3 | candidate-2 | 20260902T105410Z-ci-euroc11-f1d64eea76f8-8449ab | warning | 778e407a4edf5b4d3f185ca143426ee642867d5891248b80dc36b94b3246ca8d | c520c5a451dc33885e07e0d1177b8639e09d9f3f1f140d0cbb64261d496b6adc | PASS / PASS |
| 4 | C_engineering-2 | 20260902T105756Z-ci-euroc11-84a5712939d2-0a0a85 | warning | 1fa038a895b76d6316b259b5056e967a282037fa890f49e5ed0c81192dcb8241 | f3fc253b1869ad9a22a83dbadd35493ee67eca22264741d94d93debe9ca3ef3e | PASS / PASS |
| 5 | C_engineering-3 | 20260902T110142Z-ci-euroc11-84a5712939d2-f966c2 | warning | 79b43903d0953b047ab7ebdf6ef42506790e90c08407c76a03b2cdf0b8b43758 | fc8526e62f34f36da20bed5908b6e676626697b58051f5750006e1899e0784a4 | PASS / PASS |
| 6 | candidate-3 | 20260902T110809Z-ci-euroc11-f1d64eea76f8-b55b03 | warning | f1ffda95efe56f0f5562ac1cba9de468d32590d5446f4bc53635162ef2e6c1f3 | 627f28be12b5342975d6be316c5e36cda4f5f60cb98e1e61c56b139722b47860 | PASS / PASS |

表中的 warning 是 profile terminal state；hard gate、dataset identity 与后续
repeatability verdict 均为 PASS。Calibration run
20260902T083946Z-ci-euroc11-2b8330ad8848-fa1fe3 未计入 66 个正式 processes。

## 3. Role 内 exact repeatability

| role | independent runs | sequences / run | structural verdict | differences | errors | exact repeatability |
|---|---:|---:|---|---:|---:|---|
| C_engineering | 3 | 11 | pass | 0 | 0 | true |
| candidate | 3 | 11 | pass | 0 | 0 | true |

Exact payload 对每条序列比较 diag.csv、est.tum、kf.tum 的 SHA-256、canonical
meta SHA-256，以及移除 timing 后的 summary SHA-256；两个 role 的各项均在三次
独立进程间一致。Timing 不属于 exact payload，按第 7 节单独记录。

## 4. EuRoC-11 完整 RMSE 与健康向量

以下均为 candidate − C_engineering。每格先给 C_engineering → candidate，再在括号
中给绝对 delta；translation 从原始 m 换算为 mm，rotation 使用 deg，
completion/coverage 的 delta 使用 percentage points（pp）。数值是每序列三次
独立进程的 median；由于 role 内 exact repeatability，每序列对应的三次数值相同。
负的 RMSE delta 表示 candidate 的该误差较低；其余计数只表示方向，不单独承载产品判断。

| sequence | tier | ATE trans RMSE mm | ATE rot RMSE deg | RPE trans RMSE mm | RPE rot RMSE deg | completion | coverage | segments | PnP fallbacks |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | easy | 79.224→89.149 (+9.925) | 1.941744→1.734242 (-0.207502) | 32.721→33.317 (+0.596) | 0.255947→0.255776 (-0.000171) | 99.8370%→99.8370% (+0.0000 pp) | 99.8370%→99.8370% (+0.0000 pp) | 1→1 (+0) | 12→3 (-9) |
| MH_02_easy | easy | 115.341→123.733 (+8.391) | 3.022801→2.455625 (-0.567177) | 28.884→28.460 (-0.424) | 0.224730→0.221418 (-0.003312) | 99.8355%→99.8355% (+0.0000 pp) | 99.8355%→99.8355% (+0.0000 pp) | 1→1 (+0) | 26→0 (-26) |
| MH_03_medium | medium | 113.403→120.817 (+7.414) | 1.402944→1.304466 (-0.098478) | 56.250→54.084 (-2.166) | 0.345316→0.340653 (-0.004663) | 99.7778%→99.7778% (+0.0000 pp) | 99.7777%→99.7777% (+0.0000 pp) | 1→1 (+0) | 7→1 (-6) |
| MH_04_difficult | difficult | 136.313→134.364 (-1.949) | 1.573512→1.572319 (-0.001193) | 64.920→66.653 (+1.733) | 0.403948→0.428102 (+0.024154) | 99.6555%→99.6555% (+0.0000 pp) | 99.6553%→99.6553% (+0.0000 pp) | 1→1 (+0) | 4→0 (-4) |
| MH_05_difficult | difficult | 167.884→152.558 (-15.326) | 1.235297→1.172219 (-0.063078) | 54.819→58.068 (+3.249) | 0.326714→0.327098 (+0.000384) | 100.0000%→100.0000% (+0.0000 pp) | 100.0000%→100.0000% (+0.0000 pp) | 1→1 (+0) | 5→0 (-5) |
| V1_01_easy | easy | 96.170→74.773 (-21.398) | 5.480208→5.674060 (+0.193851) | 46.242→46.676 (+0.434) | 0.590047→0.604357 (+0.014310) | 100.0000%→100.0000% (+0.0000 pp) | 100.0000%→100.0000% (+0.0000 pp) | 1→1 (+0) | 4→0 (-4) |
| V1_02_medium | medium | 76.232→71.815 (-4.418) | 2.098819→2.069868 (-0.028952) | 45.910→45.729 (-0.182) | 0.540057→0.536231 (-0.003826) | 93.9731%→93.9731% (+0.0000 pp) | 93.9696%→93.9696% (+0.0000 pp) | 1→1 (+0) | 2→0 (-2) |
| V1_03_difficult | difficult | 836.282→173.959 (-662.324) | 12.261011→4.689890 (-7.571121) | 159.416→60.709 (-98.707) | 1.123494→0.921591 (-0.201903) | 97.1615%→97.4872% (+0.3257 pp) | 97.4860%→97.4860% (+0.0000 pp) | 2→1 (-1) | 55→45 (-10) |
| V2_01_easy | easy | 147.986→153.937 (+5.952) | 2.325613→2.378586 (+0.052972) | 43.393→43.631 (+0.238) | 1.062411→1.067593 (+0.005181) | 95.3509%→95.3509% (+0.0000 pp) | 95.3488%→95.3488% (+0.0000 pp) | 1→1 (+0) | 16→13 (-3) |
| V2_02_medium | medium | 206.634→212.521 (+5.887) | 2.356606→2.249935 (-0.106671) | 45.316→52.283 (+6.967) | 0.746435→0.798755 (+0.052319) | 97.2743%→97.2317% (-0.0426 pp) | 97.2731%→97.2305% (-0.0426 pp) | 1→1 (+0) | 15→8 (-7) |
| V2_03_difficult | difficult | 1960.225→1960.225 (+0.000) | 115.034700→115.034700 (+0.000000) | 416.013→416.013 (+0.000) | 8.157255→8.157255 (+0.000000) | 49.0370%→49.0370% (+0.0000 pp) | 72.4936%→72.4936% (+0.0000 pp) | 7→7 (+0) | 119→119 (+0) |

本结果面以绝对单位为主；completion/coverage 已直接用 rate 与 pp 表示。原始 JSON
保留未舍入值，百分比形式不作为本轮额外判断面。

## 5. 等权分层 delta 分布

聚合单位是“每序列三次运行 median”，随后在 tier 内对序列等权取
median [min, max]。Overall 同样对 11 条序列等权，不按 frame、timestamp 或 pair
数量加权。

### 5.1 Accuracy

| tier | ΔATE trans mm | ΔATE rot deg | ΔRPE trans mm | ΔRPE rot deg |
|---|---:|---:|---:|---:|
| easy | +7.171 [-21.398, +9.925] | -0.077265 [-0.567177, +0.193851] | +0.336 [-0.424, +0.596] | +0.002505 [-0.003312, +0.014310] |
| medium | +5.887 [-4.418, +7.414] | -0.098478 [-0.106671, -0.028952] | -0.182 [-2.166, +6.967] | -0.003826 [-0.004663, +0.052319] |
| difficult | -8.637 [-662.324, +0.000] | -0.032136 [-7.571121, +0.000000] | +0.867 [-98.707, +3.249] | +0.000192 [-0.201903, +0.024154] |
| overall | +0.000 [-662.324, +9.925] | -0.063078 [-7.571121, +0.193851] | +0.238 [-98.707, +6.967] | +0.000000 [-0.201903, +0.052319] |

### 5.2 Continuity 与机制

| tier | Δcompletion pp | Δcoverage pp | Δsegments | ΔPnP fallbacks |
|---|---:|---:|---:|---:|
| easy | +0.0000 [+0.0000, +0.0000] | +0.0000 [+0.0000, +0.0000] | +0.0 [+0, +0] | -6.5 [-26, -3] |
| medium | +0.0000 [-0.0426, +0.0000] | +0.0000 [-0.0426, +0.0000] | +0.0 [+0, +0] | -6.0 [-7, -2] |
| difficult | +0.0000 [+0.0000, +0.3257] | +0.0000 [+0.0000, +0.0000] | +0.0 [-1, +0] | -4.5 [-10, +0] |
| overall | +0.0000 [-0.0426, +0.3257] | +0.0000 [-0.0426, +0.0000] | +0.0 [-1, +0] | -5.0 [-26, +0] |

Accuracy 的四个维度在 tier 内和 tier 间均出现方向分叉；PnP fallbacks 在 10 条
序列减少、V2_03 不变，但该机制计数不覆盖 RMSE 与 health 的完整 trade-off。

## 6. Common timestamp / pair 支持集

下表的 E / C / ∩ 分别表示 C_engineering、candidate 和两者 intersection；
only 列显式保留未进入 common-support 分布的样本。三个 repeat 的计数完全一致。

| sequence | ATE timestamps E / C / ∩ | ATE only E / C | RPE pairs E / C / ∩ | RPE only E / C | dropped-no-partner E / C |
|---|---:|---:|---:|---:|---:|
| MH_01_easy | 3638 / 3638 / 3638 | 0 / 0 | 3618 / 3618 / 3618 | 0 / 0 | 20 / 20 |
| MH_02_easy | 2999 / 2999 / 2999 | 0 / 0 | 2979 / 2979 / 2979 | 0 / 0 | 20 / 20 |
| MH_03_medium | 2631 / 2631 / 2631 | 0 / 0 | 2611 / 2611 / 2611 | 0 / 0 | 20 / 20 |
| MH_04_difficult | 1976 / 1976 / 1976 | 0 / 0 | 1956 / 1956 / 1956 | 0 / 0 | 20 / 20 |
| MH_05_difficult | 2221 / 2221 / 2221 | 0 / 0 | 2201 / 2201 / 2201 | 0 / 0 | 20 / 20 |
| V1_01_easy | 2871 / 2871 / 2871 | 0 / 0 | 2851 / 2851 / 2851 | 0 / 0 | 20 / 20 |
| V1_02_medium | 1587 / 1587 / 1587 | 0 / 0 | 1567 / 1567 / 1567 | 0 / 0 | 20 / 20 |
| **V1_03_difficult** | **2052 / 2059 / 2052** | **0 / 7** | **2025 / 2039 / 2025** | **0 / 14** | **27 / 20** |
| V2_01_easy | 2154 / 2154 / 2154 | 0 / 0 | 2134 / 2134 / 2134 | 0 / 0 | 20 / 20 |
| **V2_02_medium** | **2265 / 2264 / 2264** | **1 / 0** | **2245 / 2244 / 2244** | **1 / 0** | **20 / 20** |
| V2_03_difficult | 942 / 942 / 942 | 0 / 0 | 808 / 808 / 808 | 0 / 0 | 134 / 134 |

V1_03 的 candidate 比 C_engineering 多 7 个 associated timestamps 和 14 个 RPE
pairs；common support 仍固定为 2052 timestamps / 2025 pairs。V2_02 的 candidate
少 1 个 associated timestamp 和 1 个 RPE pair；common support 为
2264 timestamps / 2244 pairs。其余 9 条序列两侧支持集计数一致。

下面只展开支持集发生变化的两条序列。这里的 RMSE(Δe) 是 common support 上逐样本
signed error delta 的 RMS，和第 4 节的 ΔRMSE（两个 RMSE 相减）不是同一统计量；
mean、median、p10、p90 和 max 保留方向。三个 paired repeats 的分布值一致。

| sequence | common-support metric | count | RMSE(Δe) | mean | median | p10 | p90 | max |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| V1_03_difficult | ATE Δtranslation (mm) | 2052 | 677.956029 | -653.228969 | -632.658174 | -914.173308 | -428.751419 | -287.091938 |
| V1_03_difficult | ATE Δrotation (deg) | 2052 | 7.789384 | -7.598050 | -7.778060 | -9.603461 | -5.520461 | -4.105750 |
| V1_03_difficult | RPE Δtranslation (mm) | 2025 | 143.929934 | -11.187796 | +0.000000 | -10.858059 | +10.173661 | +41.891700 |
| V1_03_difficult | RPE Δrotation (deg) | 2025 | 0.583842 | -0.054221 | +0.000000 | -0.213464 | +0.161554 | +0.723028 |
| V2_02_medium | ATE Δtranslation (mm) | 2264 | 16.149999 | +3.872750 | +1.257217 | -8.123867 | +15.952279 | +150.686432 |
| V2_02_medium | ATE Δrotation (deg) | 2264 | 0.303702 | -0.108289 | -0.217844 | -0.319009 | +0.286926 | +1.736897 |
| V2_02_medium | RPE Δtranslation (mm) | 2244 | 20.222543 | +3.046816 | +0.000000 | -0.613447 | +1.098400 | +193.452636 |
| V2_02_medium | RPE Δrotation (deg) | 2244 | 0.215556 | +0.024496 | +0.000000 | -0.031436 | +0.000000 | +2.192220 |

其余 9 条序列的完整 common-support distributions，以及以上两条序列的未舍入值，
保存在第 8 节的 paired-v2 JSON。

## 7. Runtime：record-only

运行使用相同 remote image/toolchain 和 ci-euroc11 profile，但没有受控 CPU affinity，
且 profile 内最多 8 路并发。因此 runtime 合同为 **record-only**。下表仅记录每个
profile 内 11 条 sequence process 的 median [min, max]；不计算 role 间 runtime
delta，也不据此作产品判断。

| 执行 run | role | wall_s median [min, max] | RTF median [min, max] |
|---|---|---:|---:|
| 20260902T103541Z-ci-euroc11-84a5712939d2-fb5782 | C_engineering-1 | 69.119 [40.393, 128.353] | 1.451571 [1.351648, 2.889136] |
| 20260902T104948Z-ci-euroc11-f1d64eea76f8-c13422 | candidate-1 | 69.700 [40.367, 127.451] | 1.460061 [1.347429, 2.891001] |
| 20260902T105410Z-ci-euroc11-f1d64eea76f8-8449ab | candidate-2 | 69.543 [40.064, 127.484] | 1.454510 [1.348908, 2.912861] |
| 20260902T105756Z-ci-euroc11-84a5712939d2-0a0a85 | C_engineering-2 | 69.268 [40.080, 129.163] | 1.447807 [1.345192, 2.911664] |
| 20260902T110142Z-ci-euroc11-84a5712939d2-f966c2 | C_engineering-3 | 69.231 [40.086, 128.759] | 1.445494 [1.350985, 2.911247] |
| 20260902T110809Z-ci-euroc11-f1d64eea76f8-b55b03 | candidate-3 | 69.285 [40.033, 127.135] | 1.460032 [1.349791, 2.915092] |

Rectify、frontend、estimator 与 total 的 mean/p95/max，以及每条序列的 wall/RTF
原值，均保留在两个 repeatability JSON 与三个 paired-v2 JSON 中。

## 8. 可复现输入

| artifact | raw path | SHA-256 |
|---|---|---|
| frozen protocol receipt | identity.md 的 `protocol_sha256` | eb732f643d7a13cb1c5088b2a4ee230f98901d00ae2d17d0d0968d044a9a5afa |
| frozen preflight receipt | identity.md 的 `preflight_sha256` | 227239aa339acdd718ee8f6e3ae1b0b82469c9d30b33de28191a8afe6c18e303 |
| current protocol | docs/research/2026-09-02-note-m5-recovery-engineering-matrix.md | cbd938f8652b012a36f3e997a15dd25774f84962f90450bcc277460ddc4aa156 |
| current preflight | docs/research/2026-09-02-note-m5-recovery-engineering-matrix-preflight.md | 921d026e5ebd24a7cb452360b40f0c06d44cc0780db93f75cbb96975e6028062 |
| final identity | /tmp/phad-m5-remote-matrix-final.PJm1ZB/identity.md | c1f1dc3f16d465141b56a02ceecbcbedb854075fb8d379d53c53d3c73cf53d78 |
| aggregate script | /tmp/phad-m5-remote-matrix-final.PJm1ZB/aggregate_matrix.py | 642befa4abb7b03a3ba5e1979182f350dea7bea9b8c05c04f2d032c870c043f7 |
| aggregate | /tmp/phad-m5-remote-matrix-final.PJm1ZB/aggregate.json | dccf844b68f544e67d8021d8f96a0dc413976a74878529a8d8a7f41723b0a57c |
| paired-v2 self-test | /tmp/phad-m5-remote-matrix-final.PJm1ZB/paired-v2/selftest.json | 9dab80dc614aed6416ddcf19c0f36ae997890c7b797bd13696fd332ffc22f4a9 |
| C_engineering repeatability | /tmp/phad-m5-remote-matrix-final.PJm1ZB/repeatability-engineering.json | 016fb2fee14b69956147988f3ccdf6666474919be84f5c7b5b487423ebdc2b48 |
| candidate repeatability | /tmp/phad-m5-remote-matrix-final.PJm1ZB/repeatability-candidate.json | 5b3deebf2ffb26847aee8d39db2ac384108fb3233e53a2eea1bdba7fb689792d |
| paired repeat 1 | /tmp/phad-m5-remote-matrix-final.PJm1ZB/paired-v2/repeat-1.json | cdd5d134698bb9d52844a956140396ccd096bcd6606f83a06443cda6753d9784 |
| paired repeat 2 | /tmp/phad-m5-remote-matrix-final.PJm1ZB/paired-v2/repeat-2.json | 7c5259299ecff144a1a7b0e486e4a38a362ec9774ccfdec6f56805c53638ca97 |
| paired repeat 3 | /tmp/phad-m5-remote-matrix-final.PJm1ZB/paired-v2/repeat-3.json | 60ebd47cc7bf7979ad1d035de9c138466da84fecf25e9c34451e90d064805354 |

Executed paired schema v2 postprocessor SHA-256 为
267f55e9789a024ae67bf17aa93e715790db00921c629ef312e1e50fe0edf8cf；
其 predecessor SHA-256 为
40b5d30928d678551c6be3d90c5d84d32ced05f0e6f8c3312175c52bccac9746。
Schema v2 增加 count、p10 与 p90，summary reproduction tolerance 为 1e-10。

本结果不生成跨指标单一总分。后续工程决策应继续保留这 11 条序列的完整向量，
并在新一轮矩阵启动前单独预注册 practical margin。

## 9. 2026-09-03 产品候选后续决定

本轮结果审读后，recovery进入隔离、default-off的产品候选验证，状态为
`REVIEW`。该决定不改变本轮协议在执行前未预注册practical margin的历史事实；后续
裁决沿用早于本轮结果的[通用多序列产品envelope](../specs/2026-08-27-m4-mapped-landmark-bearing-continuity.md#133-三态产品判定)，
并为本候选确认11序列mean completion/coverage的candidate − control absolute
rate delta均须`>= -0.010`（即不低于`-1.0 percentage point`）。Core-4固定由
`MH_01_easy`、`V1_03_difficult`、`V2_02_medium`、`V2_03_difficult`组成。

独立复算得到：EuRoC-11 `G_ATE=0.8589466745`、`G_RPE=0.9329340350`，
core-4 `G_ATE=0.7004679146`、`G_RPE=0.8178403367`；mean completion delta为
`+0.0002574031` absolute rate（`+0.02574031 percentage point`），mean coverage
delta为`-0.0000387341` absolute rate（`-0.00387341 percentage point`）。两层
accuracy与suite availability均通过`1.05/1.05/-0.010 rate` envelope，且zero
failed/rejected、segments未增加。`V2_02_medium`的RPE增加`6.967 mm`，超过逐序列
`max(5 mm, 10%)` review线，并少1个associated timestamp与1个RPE pair，因此产品
状态保持`REVIEW`。

候选行为、activation与confirmatory合同以
[M5 Slice B spec §15.5](../specs/2026-09-01-m5-initial-moving-root.md#155-default-off-pnp-cheirality-retry-产品候选)
为准：只允许typed cheirality proposal复用精确inlier set进行一次
guess-consistent retry，失败原子不生效，primary-success/non-trigger路径不变，且
activation进入run identity。下一轮在结果可见前冻结未参与本轮裁决的holdout、
source/input/evaluator identity、independent process repeat count、paired repeat mapping、
支持集处置、holdout自身aggregation、stopping rules与受控runtime。Holdout作为独立
stratum报告，不重新产出named EuRoC-11/core-4
aggregate；若加入已观测序列的controlled repeat，则单列repeatability stratum。
Rotation/runtime继续record-only。该产品候选不改变formal exact RED，不生成Slice B
checkpoint。固定执行合同见
[TUM-VI room confirmatory protocol](2026-09-03-note-m5-pnp-retry-tumvi-room-confirmatory-protocol.md)。
该独立holdout已于2026-09-03完成，产品裁决与完整向量见
[TUM-VI room confirmatory results](2026-09-03-note-m5-pnp-retry-tumvi-room-confirmatory-results.md)。
