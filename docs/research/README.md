# research —— 查到了什么

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录只追加：调研、开源对照、诊断、历史切片设计笔记。活架构在
[`../design/`](../design/)；验收合同在 [`../specs/`](../specs/)；行为 checkpoint
数字权威在 [`../benchmark/`](../benchmark/)。文档地图见 [`../README.md`](../README.md)。

## 命名

`YYYY-MM-DD-<kind>-<topic>.md`，日期为首次落库日。

| 前缀 | 适用 |
|---|---|
| `opensource-` | 开源对照、`*-comparison` |
| `note-` | 设计笔记、诊断、probe、叙事 baseline、result、postmortem、handoff、attribution |

## 索引（按种类）

### opensource

| 文件 |
|---|
| [2026-07-31-opensource-euroc-stereo-manifest-asymmetry.md](2026-07-31-opensource-euroc-stereo-manifest-asymmetry.md) |
| [2026-07-31-opensource-m2-3-vo-backend.md](2026-07-31-opensource-m2-3-vo-backend.md) |
| [2026-07-31-opensource-m3-3-vo-hardening.md](2026-07-31-opensource-m3-3-vo-hardening.md) |
| [2026-08-01-opensource-m3-3-slice2-right-match.md](2026-08-01-opensource-m3-3-slice2-right-match.md) |
| [2026-08-01-opensource-m3-3-slice3-pnp.md](2026-08-01-opensource-m3-3-slice3-pnp.md) |
| [2026-08-01-opensource-m3-3-slice4-outlier-cull.md](2026-08-01-opensource-m3-3-slice4-outlier-cull.md) |
| [2026-08-02-opensource-m3-3-cull-id-rebirth.md](2026-08-02-opensource-m3-3-cull-id-rebirth.md) |
| [2026-08-02-opensource-m3-3-slice4e-multiround.md](2026-08-02-opensource-m3-3-slice4e-multiround.md) |
| [2026-08-04-opensource-m3-3-pnp-stereo-consistency.md](2026-08-04-opensource-m3-3-pnp-stereo-consistency.md) |
| [2026-08-05-opensource-m3-3-keyframe.md](2026-08-05-opensource-m3-3-keyframe.md) |
| [2026-08-06-opensource-m3-3-slice5-open-benchmark.md](2026-08-06-opensource-m3-3-slice5-open-benchmark.md) |
| [2026-08-27-opensource-m4-local-map-continuity.md](2026-08-27-opensource-m4-local-map-continuity.md) |

### note

其余本目录 `YYYY-MM-DD-note-*.md` 文件（切片设计、诊断、probe、叙事 baseline、
result、postmortem、handoff、attribution）。历史切片 `*-design` 笔记在此，不是
`docs/design/` 活架构。

叙事 baseline / results 指回对应 [`../benchmark/`](../benchmark/) checkpoint，例如：

- [2026-08-03-note-m3-3-full-suite-baseline-773ea011.md](2026-08-03-note-m3-3-full-suite-baseline-773ea011.md) → [`../benchmark/m3.3/`](../benchmark/m3.3/)
- M4 锚点叙事见 [`../benchmark/m4/`](../benchmark/m4/)
