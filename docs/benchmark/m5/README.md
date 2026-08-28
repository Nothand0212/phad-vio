# M5 Benchmark Checkpoints

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录保存 M5 初始化演进的 benchmark 与 Observe 证据。Issue：
[#48](https://github.com/Nothand0212/phad-vio/issues/48)。

## Checkpoint 索引

| checkpoint | commit | config | control | 范围 | 判定 | 文档 |
|---|---|---|---|---|---|---|
| Cold-root current-path Observe | `6a193a8` | `default_0337287b` | `db22656` V2_03 frozen control | 仅 `V2_03_difficult` Q1 Observe | Observe PASS；#47 formal gate 仍为 FAIL | [checkpoint](cold-root-current-path-observe_6a193a8_0337287b.md) |

原始产物位于：

```text
/home/lin/Projects/data/phad-bench/m5-cold-root-observe-20260828T070950Z/V2_03_difficult/
```

本 checkpoint 只回答 terminal cold-root 最早阻塞 gate 与真实 root attempt 数；不改变
bootstrap、seed、solver、posterior、segment 或 active-map 语义，也不授权 root 恢复、
dynamic initialization 或跨-root world-frame alignment。
