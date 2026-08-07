# M3.3 Benchmark Checkpoints

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录保存 M3.3 关键行为 checkpoint 的 EuRoC 11 条全量结果。补录主线为：

```text
Slice ④e → Slice ④f → Slice ④g → zombie-age=5 → PnP stereo 一致性仲裁
→ ⑤c → Slice ⑥ → Slice ⑦
```

其中 ④g 已被证伪并回退，但仍保留为独立历史 checkpoint；④f 与 ④g 的 config
hash 相同、zombie-age 与 PnP 的 config hash 相同，正好证明 config identity 不能
替代 code identity。

Issue：[#26](https://github.com/Nothand0212/phad-vio/issues/26)。

## Checkpoint 索引

| checkpoint | commit | config | predecessor | 全量状态 | 产品状态 | 文档 |
|---|---|---|---|---|---|---|
| Slice ④e | `0ced28b` | `default_3a21162e` | Slice ④d | clean 11/11 ✓ | 部分完成 | [slice-4e](slice-4e_0ced28b_3a21162e.md) |
| Slice ④f | `c446ac5` | `default_a5e90dc7` | Slice ④e | clean 11/11 ✓ | 已完成、后被 zombie-age 扩展 | [slice-4f](slice-4f_c446ac5_a5e90dc7.md) |
| Slice ④g | `3ee5dea` | `default_a5e90dc7` | Slice ④f | clean 11/11 ✓（≡ ④e） | 已证伪并回退 | [slice-4g](slice-4g_3ee5dea_a5e90dc7.md) |
| zombie-age=5 | `4cf55ca` | `default_773ea011` | Slice ④f | clean 11/11 ✓（复用历史 raw） | 当前生命周期默认 | [zombie-age](zombie-age_4cf55ca_773ea011.md) |
| PnP stereo 仲裁 | `afe3829` | `default_773ea011` | zombie-age=5 | clean 11/11 ✓ | 当前 estimator 默认 | [pnp-stereo](pnp-stereo_afe3829_773ea011.md) |
| Slice ⑥ | `167478e` | `default_773ea011` | ⑤c（`86212e0`） | clean 11/11 ✓ | 当前前端默认 | [slice-6](slice-6_167478e_773ea011.md) |
| Slice ⑦ | `e77ee5d` | `default_402d1925` | Slice ⑥ | clean 11/11 ✓ | 当前生命周期默认（E13 composed g1） | [slice-7](slice-7_e77ee5d_402d1925.md) |
| pre-M4 小片 | `cbb4505`(dirty) | `sad_baseline_402d1925` | Slice ⑦ | 诊断收尾 + Census 实测否决（SAD-only ≡ Slice ⑦，无回归） | 非产品改动（B1/B2 诊断列 + 脚本） | [prem4-diag-census](prem4_diag_census_cbb4505_402d1925.md) |
| pre-M4 round 2 | `8906684`(dirty) | `default_402d1925` | pre-M4 小片 | ②③/零均值/no-CV/seed-20/首段累积 7 变体全否决（默认 ≡ Slice ⑦ 逐项一致） | 非产品改动（变体全在 CLI flag 后，默认回退） | [prem4-round2](prem4_round2_8906684_402d1925.md) |

表格中的 predecessor 指算法血缘，不强制等于 Git 一阶父提交。④g 的产品 successor
是回退后的 ④f，再由 zombie-age=5 扩展；不能把 ④g 当成 zombie-age 的直接基线。

## 逐前驱对比

以下只摘录每条对比的判定级结论；完整 11 条质量/robustness 表与逐列 delta 见各
checkpoint 文档。所有数字均来自对应文档，跨 coverage 的 ATE 差异不直接归因。

### ④e → ④f（`skip_drop_min_culled=4`）

| 关键序列 | ④e ATE | ④f ATE | 结论 |
|---|---:|---:|---|
| MH_01 | 0.098784 | 0.098784 | 逐字节一致 |
| MH_05 | 4.565065 | 3.056878 | 软门 PASS（-1.508） |
| V1_02 | 7.042673 | 0.454234 | 灾难发散消除 |
| V1_03 | 16659720.362561 | 8.452270 | 灾难发散消除 |
| MH_02 | 276879.639366 | 162764.565042 | 仍灾难级，但幅度与失败帧下降 |
| MH_03 | 0.712381 | 0.958684 | 轻微回退（无失败帧） |

### ④f → ④g（同 config hash，延后一帧 drop）

④g 全序列 `est.tum` 与 **④e** 逐字节一致（11/11），即“skip 当帧 + 下帧 flush”与
“立即 drop”相对下一帧 `process` 观测等价。④f 的所有收益（MH_05 3.057、V1_02
0.454、V1_03 8.45）在 ④g 全部消失，回到 ④e 平台——证伪，编排已回退。

### ④f → zombie-age=5（`zombie_drop_age=5`）

| 关键序列 | ④f ATE | zombie-age ATE | 结论 |
|---|---:|---:|---|
| MH_05 | 3.056878 | 2.455726 | 软门改善（-0.601） |
| MH_02 | 162764.565042 | 530013.866966 | 恶化；completion 0.812 → 0.522 |
| V1_02 | 0.454234 | 8.407697 | 恶化 |
| V1_03 | 8.452270 | 9.791492 | 恶化 |
| 其余 | — | — | MH_01/MH_04/V1_01/V2_03 逐字节一致 |

record-only baseline：MH_05 是 n=5 甜区收益，但 MH_02/V1_02/V1_03 暴露风险。

### zombie-age → PnP stereo（同 config hash，proposal 采用仲裁）

| 关键序列 | zombie-age ATE | PnP ATE | 结论 |
|---|---:|---:|---|
| MH_02 | 530013.866966 | 0.092417 | 灾难消除；completion 0.522 → 1.0 |
| MH_05 | 2.455726 | 0.472225 | 大幅改善 |
| V1_02 | 8.407697 | 0.768334 | 改善 |
| V1_03 | 9.791492 | 3.353343 | 改善，仍 failures |
| V2_01 | 5.179970 | 1.259471 | 改善，failed 清零 |
| MH_03 | 1.120766 | 1.336250 | **已知回归**（提交前 dirty 验证亦一致） |
| 其余 | — | — | MH_01/MH_04/V1_01 逐字节一致 |

## 历史资料

- [Slice ④～④g 原始门控史](../../research/m3.3-slice4-baseline.md)
- [zombie-age=5 修复前正式基线](../../research/m3.3-full-suite-baseline-773ea011.md)
- [PnP stereo 仲裁 dirty 验证](../../research/m3.3-pnp-stereo-arbitration-results.md)

这些文档在迁移完成前仍保留原始叙事与诊断上下文；本目录生成的 clean checkpoint
将作为后续数值比较的权威入口。
