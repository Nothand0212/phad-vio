# Handoff：EuRoC 双目 manifest 不等长（loader 拒开）

日期：2026-07-31  
状态：诊断已完成；**设计已确认并入库** → [`stereo-pair-synchronizer-design.md`](stereo-pair-synchronizer-design.md)（左右配对进 Synchronizer / B 族 / 默认 tol=0）；已排为 roadmap **M3.2**，计划 [`2026-07-31_m3.2_stereo_pair_synchronizer_5b7d1c93.plan.md`](../plans/2026-07-31_m3.2_stereo_pair_synchronizer_5b7d1c93.plan.md)；实施尚未动手。注意 §5 的待决选项已由设计稿取代（选的不是 A 的「改 `joinStereo` 交集」，而是删掉 `joinStereo`、配对移入 `phad::sync`）  
相关：M3.1 已完成（issue [#21](https://github.com/Nothand0212/phad-vio/issues/21) 已关）；本问题阻塞全序列 bench 中 3 条序列。开源对照：[`euroc-stereo-manifest-asymmetry-open-source-refs.md`](euroc-stereo-manifest-asymmetry-open-source-refs.md)。

## 1. 给新会话的一句话

全量 EuRoC ASL bench 有 3 条在 `open()` 阶段失败，报 `camera manifests have different record counts`。已核实：**不是本地下载损坏**，是官方 ASL/bag 左右清单本身不对称；我们的 `joinStereo` 比开源更严（等长 + 按下标 exact match）。下一步要决定是否改为 **timestamp 交集 join**（或保持严合同、仅用 bag/软同步路径）。

## 2. 背景（不必重做）

- 分支已本地 merge 进 `main`：`2b28616`（`Merge branch 'm3.1-vo-regression-benchmark'`）；相对 `origin/main` 曾 ahead（是否已 push 以当前 `git status` 为准）。
- 全序列 bench（tmux `agent-euroc-bench`，日志 `/tmp/phad-bench-logs/euroc_all_20260731_165019.log`）结果：**8 OK / 3 FAIL**，产物在 `PHAD_BENCH_ROOT=/home/lin/Projects/data/phad-bench`，commit `2b28616`，config `default_0885385a`。
- 失败三条：`MH_04_difficult`、`V1_02_medium`、`V2_03_difficult`；错误码路径经 `phad::io::dataset` → `joinStereo`。

成功序列里也有「算法差」问题（低 completion / 高 ATE），与本次 loader 拒开无关，可另开议题。

## 3. 诊断结论（已核实）

### 3.1 本地 ASL 自洽（非解压丢文件）

| sequence | cam0 CSV/PNG | cam1 CSV/PNG | timestamp 交集 | 交集 PNG |
|---|---:|---:|---:|---|
| MH_04_difficult | 2033 / 2033 | 2032 / 2032 | 2032 | 齐全 |
| V1_02_medium | 1710 / 1710 | 1711 / 1711 | 1710 | 齐全 |
| V2_03_difficult | 1922 / 1922 | 2336 / 2336 | 1921 | 齐全 |

无 missing PNG、无 orphan PNG；时间戳各自严格递增。

### 3.2 ASL 与同机 rosbag 计数一致

用 `rosbags` 统计 `/home/lin/Projects/data/thidparty/euroc/ros1/<seq>.bag` 的 `/cam0/image_raw` 与 `/cam1/image_raw`：**与 ASL `data.csv` 行数完全一致**（含上述不对称）。若是 ASL zip 下坏，通常会与 bag 不一致。

### 3.3 社区已承认官方数据不对称

Kimera-VIO README 原文：MH_04 与 V2_03 left/right 帧数不同，建议用他们整理版。

- https://github.com/MIT-SPARK/Kimera-VIO/blob/master/README.md  
  （章节 *Euroc Dataset → Download*）  
- raw：https://raw.githubusercontent.com/MIT-SPARK/Kimera-VIO/master/README.md  
- 他们整理的 Drive：https://drive.google.com/open?id=1_kwqHojvBusHxilcclqXh6haxelhJW0O  
- 官方数据集入口：https://ethz-asl.github.io/datasets/（旧页会重定向）

Kimera 文档/issue 语境中 MH_04 的典型报错即 **Left: 2033 / Right: 2032**，与本地一致。

### 3.4 我们失败点在合同，不在 VO

实现：`phad/io/dataset/internal/dataset_adapter_utils.cpp` → `joinStereo`：

1. `left.size() != right.size()` → 立即 `kStereoTimestampMismatch`（本次三条都死在这）；
2. 否则按下标要求 `left[i].timestamp == right[i].timestamp`。

设计意图见 `docs/research/euroc-dataset-loader-design.md` §3.3 / §7：M1 对原生 EuRoC 做 **exact join**，不一致则失败；明确不照搬 VINS 的 3 ms 最近邻丢帧。  
但设计文字也写「以 timestamp 做 exact join / 左右集合完全匹配」——**当前实现是「等长 + 下标对齐」**，还没走到「集合交集」；不等长时直接拒，即使交集很大（如 MH_04 可配对 2032 帧）。

开源常见更松：ORB-SLAM3/Basalt 偏 cam0 时间表 + 拼路径；VINS 多用 rosbag + 容差队列。故「别的算法能跑」与「我们 open 失败」不矛盾。

## 4. 若放宽为交集 join 的预期

在只做 **exact timestamp 集合交集**（仍不做最近邻）时：

| sequence | 可用 stereo 帧（约） |
|---|---:|
| MH_04_difficult | 2032 |
| V1_02_medium | 1710 |
| V2_03_difficult | 1921 |

应 warning 记录 `only_cam0` / `only_cam1` 丢弃数；V2_03 的 cam1 多出约 415 帧多数落在 cam0 时间跨度内（左目缺帧），不是简单「尾部多一行」。

## 5. 待决决策（新会话从这里对齐）

推荐先问用户（A/B/C）：

- **A（推荐）**：`joinStereo` 改为 timestamp **exact 交集**；集合非空即可 open；丢弃帧进 warning / 诊断计数；单测覆盖 MH_04 式 off-by-one 与 V2_03 式大量不对称。保持「不做时间容差最近邻」。
- **B**：保持严合同（全集必须相等）；bench/文档标明 MH_04、V1_02、V2_03 原生 ASL 不可用；改走 bag 或 Kimera 整理包。
- **C**：另加 soft-sync（容差配对）——偏离 M1 设计，需单独 design 审阅。

用户偏好：milestone/slice 开工前逐项对齐；改 loader 合同前应有短 design/决策记录（可补一小节进 `euroc-dataset-loader-design.md` 或独立短文）。

## 6. 建议实施切片（仅在选 A 后）

1. 改 `joinStereo`：按 timestamp 建 map/set 交集，排序后产出 manifest；记录 dropped_left / dropped_right。  
2. 单测：合成不等长 + 部分共同 timestamp；回归 MH_01 等长路径不变。  
3. 更新 `docs/research/euroc-dataset-loader-design.md` 与 `phad/io` README（若有）合同说明。  
4. 重跑三条失败序列的 `phad_vo_bench`（clean 树；路径已有 `summary.json` 时需 `--force` 或换 commit）。  
5. 用 `scripts/bench_table.py $PHAD_BENCH_ROOT` 刷新对照表。

**不要**在未对齐前大改 frontend/estimator；三条失败与 VO 算法无关。

## 7. 路径速查

```text
数据 ASL:  /home/lin/Projects/data/thidparty/euroc/native/<seq>
数据 bag:  /home/lin/Projects/data/thidparty/euroc/ros1/<seq>.bag
bench:     /home/lin/Projects/data/phad-bench/
日志:      /tmp/phad-bench-logs/euroc_all_20260731_165019.log
代码:      phad/io/dataset/internal/dataset_adapter_utils.cpp  (joinStereo)
设计:      docs/research/euroc-dataset-loader-design.md
拼表:      python3 scripts/bench_table.py /home/lin/Projects/data/phad-bench
```

本地快速复现拒开：

```bash
./build/phad_euroc_inspect /home/lin/Projects/data/thidparty/euroc/native/MH_04_difficult
# 或
./build/phad_vo_bench /home/lin/Projects/data/thidparty/euroc/native/MH_04_difficult \
  --bench-root /home/lin/Projects/data/phad-bench --force
```

## 8. 杂项

- tmux 会话 `agent-euroc-bench` 可能仍在 `sleep infinity`；无用可 `tmux kill-session -t agent-euroc-bench`。
- 工作区规则：主 checkout 短分支、不默认 push；EuRoC 路径拼写是 `thidparty`（历史目录名）。
- `AGENTS.md` 已记：先区分 loader 合同 vs 数据损坏；`joinStereo` 等长行为。
