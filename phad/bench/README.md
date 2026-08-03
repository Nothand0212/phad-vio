# `phad::bench` 回归落盘合同库

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录实现 VO 回归 benchmark 的**纯逻辑**合同：run 身份、config 快照与
hash、路径模板、`meta` / `summary` schema 与 JSON 序列化。库本身不跑
pipeline、不算 ATE/RPE、不落盘；组装与写文件由 `apps/phad_vo_bench` 完成。

CMake target：`phad_bench`（alias `phad::bench`）。**零** `phad::*` 依赖；
`nlohmann_json` 仅 PRIVATE（public header 不出现 json 类型）。

## 职责边界

| 做 | 不做 |
|---|---|
| git 身份查询（tracked dirty） | 认识 frontend / estimator / eval 类型 |
| `ConfigSnapshot` 规范化文本与 `config_hash` | 展平 options（归 apps） |
| `composeRunDir` / `decideOverwrite` | 创建目录、写文件 |
| `RunMeta` / `RunSummary` → JSON 文本 | 跑 VO、算指标、拼对比表 |

## 文件布局

| 文件 | 作用 |
|---|---|
| `config_snapshot.hpp/.cpp` | 有序 key→scalar、`canonicalText` / `hash8` / `toJson` |
| `code_identity.hpp/.cpp` | `CodeIdentity`、`queryGitIdentity`（唯一外部进程 TU） |
| `run_paths.hpp/.cpp` | 路径模板与覆盖策略判定 |
| `run_summary.hpp/.cpp` | `RunMeta` / `RunSummary` schema 与 JSON |

## 数据流

```text
apps 展平 options ──set──► ConfigSnapshot
                              │
                    canonicalText() ──► hash8() / toJson()
queryGitIdentity(repo) ──► CodeIdentity
composeRunDir(...)     ──► run_dir
decideOverwrite(...)   ──► kWrite | kOverwriteWithWarning | kRefuse
RunMeta / RunSummary   ──toJson──► meta.json / summary.json 文本
                                      （写盘由 app 完成）
```

## 格式合同

### 路径模板

```text
<bench_root>/<sequence>/<commit_short>[_dirty]/<config_label>_<hash8>/
```

- `summary.json` 是占用哨兵：clean 树已存在则 `kRefuse`（app → exit 3）；
  dirty 或 `--force` → `kOverwriteWithWarning`。
- dirty 只看 tracked（`git diff --quiet HEAD`）；untracked 不计。

### `config_hash`

1. 将 key→scalar 写成规范化文本：key 字典序，每行 `key=value`，
   double 用 `%.17g`，bool 用 `true`/`false`；
2. 对全文做 FNV-1a 64，取十六进制表示的前 8 位。

新增 options 字段时，apps 侧展平必须同步，否则 hash 对改动不敏感。

### `meta.json` / `summary.json`

- `schema_version = 1`
- 完整 `config` 只在 `meta.json`；`summary.json` 保持小以便拼表
- `ate` / `rpe` 缺失时序列化为 JSON `null`（不写假 0）
- `status`：`completed` / `completed_with_failures` /
  `completed_with_warnings` / `eval_failed` / `failed`

M3.3 在 `summary.json` 追加段 / PnP / 剔点可观测性字段（`schema_version`
仍为 1）：

| 路径 | 含义 |
|---|---|
| `trajectory.segments` | 轨迹段数；有序列接受位姿时为 `reanchors + 1`，否则 `0` |
| `robustness.reanchors` | `segment_id` 跳变次数（每次成功 re-anchor 计 1） |
| `robustness.pnp_successes` | 正常路径上采用 PnP 初值的 `kOk` 帧数 |
| `robustness.pnp_fallbacks` | 正常路径上本可尝试 PnP 但未采用的帧（对应点不足 / RANSAC 失败 / inliers 不足）；首段 seed 与 re-anchor 不计 |
| `robustness.outliers_culled` | 全 run 累计 mean-reproj 删点**次数**（按帧累加本帧删点数） |
| `robustness.outliers_culled_unique` | 全 run 累计去重 id 数；`outliers_culled / outliers_culled_unique` 为重复删除率（>1 表示同 id「删→再喂→重建→再删」空转） |
| `robustness.outlier_reopts` | 全 run 累计成功 reopt **次数**（Σ `UpdateDiagnostics.outlier_reopt_rounds`；失败回退的轮次不计；不是「触发过 reopt 的帧数」） |
| `robustness.drops_skipped` | 全 run 累计因 `outliers_culled ≥ session.skip_drop_min_culled` 跳过 `dropTracks` 的**帧数**（Slice ④f；`drop_culled_tracks=false` 时不计） |
| `robustness.deferred_drops` | 全 run 累计 skip 后延后冲刷 `dropTracks` 的**次数**（Slice ④g） |
| `robustness.deferred_drop_ids` | 全 run 累计延后 drop 的 id **个数**（Slice ④g） |
| `robustness.evictable_marked` | 全 run 累计 skip 帧调用 `markEvictable` 的**次数**（`--evict-skip-culled`） |
| `robustness.tracks_evicted` | 全 run 累计 frontend 懒腾槽移除的 track **个数** |

由 `OfflineVoSession` 统计后经 `phad_vo_bench` 写入；`bench_table.py`
可据此区分「算法变好」与「re-anchor / PnP fallback / 剔点空转 /
重优触发变多」。`est.tum` 仍为单条连续轨迹，不含段号列。
`schema_version` 仍为 1。

`config_hash` 在 M3.3 起纳入 `estimator.min_seed_observations` 与
`estimator.enable_reanchor`；Slice ③ 再纳入 `estimator.enable_pnp_init` /
`estimator.pnp_reproj_px` / `estimator.pnp_confidence` /
`estimator.min_pnp_inliers`；Slice ④ 再纳入
`estimator.enable_outlier_cull` / `estimator.outlier_avg_reproj_px`；
Slice ④b 再纳入 `estimator.enable_outlier_reopt`（apps 侧
`flattenConfig` 展平）；Slice ④e 再纳入 `estimator.max_outlier_reopts`
（默认 `3`；bench `--max-outlier-reopts` 可覆盖）；Slice ④c 再纳入
`session.drop_culled_tracks`（bench `--no-drop-culled-tracks` 可设为
`false`）；Slice ④f 再纳入 `session.skip_drop_min_culled`（默认 `4`；bench
`--skip-drop-min-culled` 可覆盖；`0` = 关闭阈值跳过）；新配置落新目录。
