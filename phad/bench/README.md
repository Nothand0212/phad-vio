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

M3.3 在 `summary.json` 追加段 / PnP 可观测性字段（`schema_version` 仍为 1）：

| 路径 | 含义 |
|---|---|
| `trajectory.segments` | 轨迹段数；有序列接受位姿时为 `reanchors + 1`，否则 `0` |
| `robustness.reanchors` | `segment_id` 跳变次数（每次成功 re-anchor 计 1） |
| `robustness.pnp_successes` | 正常路径上采用 PnP 初值的 `kOk` 帧数 |
| `robustness.pnp_fallbacks` | 正常路径上本可尝试 PnP 但未采用的帧（对应点不足 / RANSAC 失败 / inliers 不足）；首段 seed 与 re-anchor 不计 |

由 `OfflineVoSession` 统计后经 `phad_vo_bench` 写入；`bench_table.py`
可据此区分「算法变好」与「re-anchor / PnP fallback 变多」。`est.tum` 仍为
单条连续轨迹，不含段号列。

`config_hash` 在 M3.3 起纳入 `estimator.min_seed_observations` 与
`estimator.enable_reanchor`；Slice ③ 再纳入 `estimator.enable_pnp_init` /
`estimator.pnp_reproj_px` / `estimator.pnp_confidence` /
`estimator.min_pnp_inliers`（apps 侧 `flattenConfig` 展平）；新配置落新目录。
