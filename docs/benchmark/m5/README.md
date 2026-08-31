# M5 Benchmark Checkpoints

本目录保存 M5 的产品行为 checkpoint。当前只有 #53 的 bounded TUM VI product-input
checkpoint；它证明 raw input seam 可达，不代表 M5 initial moving root、trajectory
质量或全序列资格化已完成。

Issues：[#53](https://github.com/Nothand0212/phad-vio/issues/53)、
[#51](https://github.com/Nothand0212/phad-vio/issues/51)。

## Checkpoint 索引

| checkpoint | commit | config | 范围 | 状态 | 文档 |
|---|---|---|---|---|---|
| TUM VI product input seam | `ba607c5` | `noconfig`（无 emitted meta/hash） | camera/frontend EuRoC controls + `corridor1_512_16` fixed 100-frame product gate | formal gates passed；非全序列 benchmark | [checkpoint](tum-vi-product-input-seam_ba607c5_noconfig.md) |

`noconfig` 是明确的无-hash 标记：本次 test composition root 不生成 `meta.json`，
也没有与运行同源的 `ConfigSnapshot`。Checkpoint 以 clean code commit、测试源码、
显式 options 与 dataset identity 共同复现；后续有真实 bench artifact 时另行记录其
emitted `meta.json`、config hash、raw artifact root 与完整结果。
