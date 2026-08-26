# M4 Benchmark Checkpoints

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录保存 M4 full-state VIO 关键行为 checkpoint 的 EuRoC 11 条全量结果。

Issue：[#41](https://github.com/Nothand0212/phad-vio/issues/41)。

## Checkpoint 索引

| checkpoint | commit | config | predecessor | 全量状态 | 用途 | 文档 |
|---|---|---|---|---|---|---|
| Minimal full-state VIO | `c999f58` | `default_0337287b` | M3.3 Slice ⑦ | clean 11/11 ✓ | M4 后续精度与 lifecycle 优化的首个全量锚点 | [checkpoint](minimal-full-state-vio_c999f58_0337287b.md) |

原始产物位于：

```text
/home/lin/Projects/data/phad-bench/<sequence>/c999f58/default_0337287b/
```

每条序列保存 `meta.json`、`summary.json`、`diag.csv`、`est.tum` 和
`kf.tum`。
