# pre-M4 小片: 诊断收尾 + Census 实测否决（`sad_baseline_402d1925`）

日期：2026-08-07
状态：**Census 候选修复被实测否决，回退 SAD-only；B1/B2 诊断收尾完成**。
Census 兜底（SAD 主路径 + Census fallback）在 V2_03 上 ATE 7937m（225 次
>5m 锚跳）vs SAD-only 3.628m（0 锚跳）；MH_01 上 +13%（0.0915 vs 0.0810）。
SAD-only（`enable_census=false`）与 slice-7 官方 run 逐项一致（MH_01
0.0810/0.9997、V2_03 3.6278/0.5039/0.7579），无回归。

相关：

- issue [#27](https://github.com/Nothand0212/phad-vio/issues/27)
- 诊断：[m3.3-remaining-failure-debt.md](../research/m3.3-remaining-failure-debt.md)（§0.1 实测否决 + §4.2 更新）
- 计划：[pre-M4 小片计划](../../plans/synchronous-wiggling-star.md)
- predecessor：Slice ⑦（`e77ee5d` / `402d1925`）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

## 1. 身份与执行

| 项 | 值 |
|---|---|
| commit | `cbb4505`（工作区 dirty——B1/B2/C 改动未提交） |
| config | `sad_baseline` / hash `402d1925`（= slice-7 default；census 是 CLI-only 不进 hash） |
| 代码改动 | B1：diag `num_disparity` 列（20 列）；B2：`scripts/segment_ate_decomp.py`；C：matchRight 双路径（SAD 主路径 + Census 兜底，实测后默认关闭） |
| 对比锚 | slice-7 `e77ee5d` / `402d1925`（SAD-only ≡ slice-7 行为，双重确认） |
| bench root | `/home/lin/Projects/data/phad-bench` |
| 数据集 | EuRoC ASL native |

## 2. B1：`num_disparity` 列（完成）

diag.csv 20 列：`is_keyframe` 后追加 `num_disparity`（本帧 `disparity_px>0`
的观测数，不查 landmark 表）。与 `num_shared` 联立区分「前端立体匹配死」
vs「地图 overlap 死」：

- **V2_03 启动段 0-152 实测**（SAD-only）：每帧 200 条 track 中仅 1-7 条
  产出视差（≈1-3%）→ `min_seed_observations=10` 饿死 → shared=0。
  旧文档「disparity=0」精确化为「≈1-3%」（见 research §1.2）。
- **V2_03 暗段 700-710 实测**：Census on 时 122 视差 vs SAD 0 —— 提升
  存在，但提升部分是错误匹配（§4）。

## 3. B2：V1_03 逐段 ATE 分解（完成）

`scripts/segment_ate_decomp.py`（run_dir + euroc_root 两参）：按
`segment_id` 分割轨迹段，每段独立对齐 GT 后算 ATE，与全局对齐对比量化
锚偏移贡献。结论（已并入 research §5.3）：

| 序列 | 全局 ATE | 段内独立加权 RMS | 段间错位贡献 |
|---|---:|---:|---:|
| V1_03 | 4.960 | 0.768 | **+4.19 m（84.5%）** |
| V2_03 | 3.628 | 0.889 | **+2.74 m（75%）** |

误差主体是 re-anchor 段间锚偏移，不是段内跟踪质量 → 修复优先级 =
锚质量门（M4 IMU 预积分先验结构上消除 CV 外推锚）。

## 4. C：Census 实测否决（核心结论）

### 4.1 实现

matchRight 双路径：SAD 主路径（原行为）+ Census 兜底（5×5 窗 24bit
Hamming，仅 SAD 被拒时触发：hamming 选择 + SAD tiebreak + 唯一性绝对
阈值 1 位 + SAD 抛物线子像素）。`enable_census` 默认 **false**，
CLI `--tracker-enable-census` 显式打开（旧 `--tracker-disable-census`
已移除）。

### 4.2 实测（cbb4505 双路径代码）

| 配置 | V2_03 ATE | V2_03 覆盖 | V2_03 锚跳(>5m) | MH_01 ATE |
|---|---:|---:|---:|---:|
| SAD-only | **3.6278** | 0.758 | 0 | **0.0810** |
| Census 兜底 on | 7937 | 0.99 | 225（均 236m） | 0.0915（+13%） |

机制：暗帧 Hamming 平顶（±2-3px 噪声驱动）+ 唯一性 1 位阈值过松 →
Census 兜底产出的暗帧匹配（启动段累计 1633 vs SAD 296）中错误匹配进
PnP → pose 污染 → 反复丢跟踪（~100 segments vs SAD 10）→ 重建锚错
（225 次 >5m 跳变，对应 GT 真实运动 0.2m）→ 全局 ATE 7937m。SAD 在
暗帧「少产视差」反而是保护：只消费健康匹配，重建少且锚定正确。

### 4.3 对诊断文档的修正

「cam1 欠曝光 → 非归一化 SAD 全败 → disparity=0」在双路径代码下实测为
「≈1-3% track 产视差（非 0）」；Census 作为链 A/B1 候选修复被否决。
剩余候选：帧级曝光归一化（不换匹配核）或接受数据侧限制（research
§0.1）。

## 5. 门控判定

| 门控 | 标准 | 结果 |
|---|---|---|
| MH_01 硬门 | ATE ≤ 0.098784 | ✓ 0.0810（SAD-only；Census on 0.0915 亦过） |
| 编译 | 零警告 | ✓ |
| 双路径重构 ≡ 原 SAD | MH_01 diag/结果一致 | ✓ 0.0810 vs slice-7 0.080964；V2_03 3.6278/0.5039/0.7579 ≡ slice-7 逐项一致 |
| V2_03 `num_disparity` | 启动段 > 0 | ✓ SAD-only 296（1-7/帧）；Census on 1633 —— 但提升为负收益（§4） |
| Census off ≡ SAD baseline | 行为一致 | ✓（同代码同路径） |

**全表 11 序列 record-only**：以 `sad_baseline_402d1925`（SAD-only）执行
（Census 否决后不再跑 census on 全表）。结果见 §6（跑完后回填）。

## 6. 全量质量表（11/11，SAD-only）

| sequence | ATE (m) | RPE (m) | completion | coverage | segments | reanchors |
|---|---:|---:|---:|---:|---:|---:|---:|
| MH_01_easy | 0.0810 | 0.0178 | 1.000 | 1.000 | 1 | 0 |
| MH_02_easy | 0.0894 | 0.0156 | 0.996 | 1.000 | 1 | 0 |
| MH_03_medium | 0.1286 | 0.0356 | 1.000 | 1.000 | 1 | 0 |
| MH_04_difficult | 0.2882 | 0.0541 | 0.999 | 1.000 | 1 | 0 |
| MH_05_difficult | 0.3241 | 0.0440 | 1.000 | 1.000 | 1 | 0 |
| V1_01_easy | 0.1163 | 0.0458 | 1.000 | 1.000 | 1 | 0 |
| V1_02_medium | 0.5215 | 0.1196 | 0.937 | 0.943 | 1 | 0 |
| V1_03_difficult | 4.9595 | 0.9364 | 0.960 | 0.995 | 6 | 5 |
| V2_01_easy | 0.3094 | 0.0410 | 0.949 | 0.951 | 1 | 0 |
| V2_02_medium | 0.8528 | 0.2678 | 0.966 | 0.971 | 1 | 0 |
| V2_03_difficult | 3.6278 | 0.8869 | 0.504 | 0.758 | 10 | 9 |

与 slice-7 全量表（`e77ee5d` / `402d1925`）**11/11 逐项一致**（ATE/RPE/
completion/coverage/segments 全同）——双路径重构的 SAD 主路径与原代码
行为完全一致，零回归确认。Census 兜底全表未跑（实测否决，§4）。

## 7. 未决

- ~~**帧级曝光归一化**（cam1 整帧亮度补偿后 SAD）~~ **已立项并实测否决**
  （2026-08-07，[prem4-round2](prem4_round2_8906684_402d1925.md)）：
  V2_03 覆盖 0.758 → 0.933 但 ATE 3.628 → 5.213；阈值 1.0 时 MH_01
  +41%（`cv::add` 饱和裁剪），阈值 20 才零回归。与之同批的 ② 累积播种 /
  零均值 SAD / no-CV 锚 / seed 门 20 / 首段累积共 7 变体全灭 ——
  匹配/播种层不是瓶颈，ATE 损失 100% 来自 re-anchor 对齐税。代码保留
  在 CLI flag 后供 M4 复测。
- V2_03 覆盖 0.758 缺口（913 rejected 帧）是数据侧（cam1 暗 + cam0
  缺帧）已知限制；round 2 证明解锁覆盖（0.975）的代价是对齐税，首段
  饿死不是绑定约束（首段累积 0.766 覆盖仍微劣于门）→ 结构性修复在
  M4 的 IMU 预积分 + 锚质量门。
