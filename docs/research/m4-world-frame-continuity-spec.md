# 视觉中断期世界系连续性 spec（片 1）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-26

issue：[#43](https://github.com/Nothand0212/phad-vio/issues/43)（map：[#42](https://github.com/Nothand0212/phad-vio/issues/42)）

control：[M4 checkpoint `c999f58` / `default_0337287b`](../benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md)

证据：[方向分析](m4-next-direction-analysis.md)（§B 分解、§C 机制）、
[coast 传播漂移探针](m4-coast-propagation-probe-design.md)（预注册 + PASS）

## 1. 目标与证据边界

视觉中断后不再把新段的 root 打回世界原点。IMU 连续且**累计无支撑跨度在
已实测范围内**时，新 root 继承传播后的 `NavState`，世界系不变。

已实测的支撑（不是推断）：

| 事实 | 值 | 来源 |
|---|---|---|
| 段间错位占全局 ATE 平方的份额（均值口径） | `0.4454 m` / **76.9%** | 方向分析 §B |
| 93 个 segment 首帧位置 | 全部精确 `[0,0,0]` | 方向分析 §C.1 |
| 500 ms 桥接位置漂移 | median `0.0708 m`、p90 `0.1624 m` | 探针 §8.1 |
| 500 ms 桥接姿态漂移 | median `0.603°` | 探针 §8.1 |
| 已测量的桥接跨度 | `450–500 ms` | 探针 §8.1 |
| V2_03 最长无支撑跨度 | `13 s`（未测） | 探针 §8.3 |

**证据边界**：探针的 PASS 只覆盖 `450–500 ms` 的单次桥接。本 spec 因此
不授权任何超出该范围的继承；秒级桥接没有证据，必须落回诚实重开。

## 2. 与 M4 spec 的 authority 关系

本文**关闭** [M4 minimal full-state VIO spec](m4-minimal-full-state-vio-spec.md)
的两处已声明开口，不新开合同：

- §3.5 Visual outage lifecycle 把「`re-anchor` 与 recovery」列为
  implementation-plan blocking；§7.1 同样列出「最小 recovery/re-anchor 行为」。
  本文给出该行为的具体合同。
- §7.2 把「跨 segment global trajectory stitching」延后到后续 milestone。
  现在已有 §1 的实测证据，本文将其收窄为「有界的世界系继承」——**不是**
  全局 stitching，没有回环、没有全局地图、没有跨段 posterior 关联。

保持不变：§2 external seam 不扩大；§3.1–3.4、§3.6、§4 的合同不动；
`kDiscontinuity`（IMU 真断裂）语义不动。

## 3. 分片与资格（两条因果边分开取权）

方向分析 §E 把片 1 描述为「两半」。按
[证据门控 §2](../agents/evidence-gated-integration.md)（「不要在同一片中
同时让数据通路、初始化、状态、factor 取得新权限」），两半是两条不同的
因果边，拆成顺序 sub-slice：

| sub-slice | 唯一新增因果边 | 冻结的变量 |
|---|---|---|
| **1a 世界系继承** | 传播后的 `NavState` 有资格设定新段 root | 观测摄入策略、播种策略、门限数值全部不动 |
| **1b 观测摄入解耦** | 低 overlap 帧仍有资格摄入观测并播种 landmark | 1a 已交付的继承行为不动 |

**1a 先行**的理由：它直接攻击已测量的 76.9%，且**不改变 overlap 与
segment 的统计口径**，因此 1b（以及后续片 3）的归因不被污染。反之若先做
1b，outage 次数与 coast 分布同时变化，1a 的收益无法单独归因。

本 spec 的 §4–§6 覆盖 1a 的完整合同；1b 的合同见 §7，在 1a 过门后细化。

## 4. 运行时合同（1a）

### 4.1 无支撑跨度预算

引入单一新概念：**累计无支撑跨度** `unsupported_span_ns`。

- 定义：当前图像 timestamp 减去**最后一个带视觉支撑的 committed state**
  的 timestamp。
- 「带视觉支撑」沿用既有 predicate，不新造：`m_initialized` 且
  `num_shared >= min_pnp_inliers`。
- 每次带视觉支撑的 state 提交时归零；`imu_only_coast` 期间随 IMU interval
  累加。
- 与既有 `m_visual_coast_duration_ns` 的区别：后者在每次
  `completeActiveSegment()` 时被清零，只度量**单次** coast；
  `unsupported_span_ns` **跨段累加**，直到真正恢复视觉支撑。

必须用累计量而不是单次 coast 时长，理由是探针 §8.3 的实测：V2_03 存在
最长 8 次的连锁 outage。若按单次 coast 判定，每一环都各自「≤ 500 ms」而
放行，8 环累计将在毫无证据的 `4 s+` 跨度上继承世界系。

### 4.2 继承与诚实重开

`completeActiveSegment()` 之后的 seed 分两条真实分支（不是兼容层——
origin 分支是超出证据范围时唯一诚实的行为）：

| 条件 | 行为 |
|---|---|
| IMU 连续（raw arm，非 `MeasurementDiscontinuity`）**且** `unsupported_span_ns <= world_inherit_horizon_ns` | **继承**：root pose/velocity 取传播后的 `NavState`，bias 继承上一段末值 |
| 否则 | **诚实重开**：现有行为不变（root 回原点、`v = 0`、bias 归零、重走 bootstrap） |

`kDiscontinuity`（IMU 真断裂）永远走重开分支——IMU 不连续时没有可传播的
状态，这与 §3.4 的既有语义一致。

### 4.3 horizon 的取值与纪律

`world_inherit_horizon_ns` 默认 **`500'000'000`（500 ms）**，等于探针实测
范围的上界。

按 M4 spec §3.3 对 `limit_ns` 的同一套纪律：

- 显式提供的正整数纳秒 fixed duration，进入 canonical config snapshot 与
  `config_hash`；
- **不从** `m_visual_coast_horizon_ns` 或任何其他 duration 派生（两者语义
  不同：一个是输出 coast 合同，一个是传播证据范围），不在线自适应；
- `unsupported_span_ns == world_inherit_horizon_ns` 仍可继承；
- 没有 disabled 或 unbounded sentinel；扩大该值需要新的探针证据。

### 4.4 segment identity 与输出

- `segment_id` **继续自增**，作为 visual-outage 事件的簿记。这保留与
  baseline 的可比性，也保留逐段诊断能力。
- `est.tum` / `kf.tum` 格式不变，`summary.json` schema 不变。世界系连续
  之后全局 ATE 自动改善，不需要改评估口径。
- `robustness.reanchors` 当前是恒 `0` 的死字段（全仓无自增点，三处测试把它
  断言在 0；`phad/bench/README.md:76` 的 `segments = reanchors + 1` 已不
  成立）。本片是修正它的自然时机：重新定义为**继承世界系的段切换次数**，
  与 `segments`（总段数）配合即可读出「继承 vs 重开」的比例。

## 5. 错误与诊断

新增诊断字段限于回答「机制是否真的执行」（证据门控 §5 完成条件 1），共
三项，不做投机性字段扩张：

| 字段 | 语义 |
|---|---|
| `unsupported_span_ns` | seed 时刻的累计无支撑跨度 |
| `world_inherited` | 该次 seed 是否走了继承分支 |
| `inherit_declined_reason` | 未继承的原因：`imu_discontinuity` / `over_horizon` / `not_applicable` |

失败语义（按证据门控 §8）：

- IMU 传播抛异常或产生非有限值：**hard failure**，返回既有失败 result，
  不静默落回 origin 分支假装成功；
- 超出 horizon：正常落 origin 分支，并在 `inherit_declined_reason` 记明
  `over_horizon`，计数可见——这是 `pending` 类语义，不是错误；
- 继承分支与重开分支都必须在**同一原子 transaction** 内完成，复用既有
  `VioUpdateTransaction` 回滚。

## 6. 测试与验收（1a）

### 6.1 定向测试（机制层，独立于 ATE）

`phad_estimator_tests` 新增用例，全部使用 synthetic 输入：

1. IMU 连续 + 跨度在 horizon 内 ⇒ 新段 root 等于传播后的 `NavState`
   （位姿 + 速度），bias 等于上一段末值；世界系不变。
2. IMU 连续 + 跨度超 horizon ⇒ 落 origin 分支，`inherit_declined_reason
   == over_horizon`。
3. `MeasurementDiscontinuity` ⇒ 无论跨度多小都落 origin 分支。
4. `unsupported_span_ns` 跨**连锁** outage 正确累加，且在恢复视觉支撑时
   归零（对应探针 §8.3 的连锁人群）。
5. 边界：`unsupported_span_ns == world_inherit_horizon_ns` 仍继承。
6. 继承路径上的 transaction 失败 ⇒ observable state 与后续合法调用结果
   不变（沿用 §3.4/§5 既有回滚约定）。

### 6.2 产品门（EuRoC）

按门控序列推进，**不直接跳全序列**：

| 阶段 | 序列 | 门 |
|---|---|---|
| ① 硬门 | MH_01 | 不劣于 `0.070539`（单段序列，本片不应触及它） |
| ② 机制靶 | V1_03、V2_02、V2_03 | 段间错位占比须显著下降（当前 `99.9% / 99.4% / 99.9%`） |
| ③ 回归 | 其余 7 条 | 不劣化；MH_02 段内 `0.2497` 预期不变（属 #44） |

量化预期（上界是段内水平，来自方向分析 §B）：

| sequence | 当前全局 ATE | 预期 |
|---|---:|---:|
| V1_03 | 1.407 | ~0.05 |
| V2_03 | 1.708 | ~0.06 |
| V2_02 | 1.982 | ~0.15 |
| 11 条均值 | 0.579 | ~0.134 |

### 6.3 归因要求

- 段间错位占比用 `scripts/segment_ate_decomp.py` 的口径复核，不只看 ATE
  单列；同时比较 completion、coverage、segments、`world_inherited` 计数。
- 保留**逐事件**明细：探针最差单次桥接（V1_03 `0.4754 m` / `2.93°`）出现在
  `0.37 m/s` 低速段，聚合中位数会掩盖它。
- 机制门与产品门分开判读（证据门控 §6）：若 `world_inherited` 计数为 0 而
  ATE 改善，结论是「机制未生效」，不得记为通过。

## 7. 1b 的范围（在 1a 过门后细化）

目标：解耦 `min_pnp_inliers` 的双重语义——它现在同时表示「PnP 是否可解」
与「是否继续摄入观测」。低 overlap 帧应继续摄入观测并播种 landmark，
不再清空 `candidate.m_observations`（`vio_estimator.cpp:2085-2087`）。

直接抓手（探针 §8.4 实测）：coast 期间窗口地图在第 8 帧**归零**
（`num_landmarks` 25→23→21→19→17→14→11→0），`num_shared` 全程锁在 `2–5`。
coast 期间继续播种即可阻止归零。

预期收益人群：探针 §8.3 的连锁 outage（V2_03 约一半 outage、最长链 8 次），
传播桥接对它们无效，只能靠减少中断本身。

外部对照：slambook2 ch13 `InsertKeyframe()`（`frontend.cpp:71-95`）的触发
条件正是 `tracking_inliers_` 低于阈值，随后立刻
`DetectFeatures` + `FindFeaturesInRight` + `TriangulateNewPoints`——
**overlap 变薄即补点**，与当前「变薄即停止播种」相反。

1b 的门与 factor 级细节不在本 spec 冻结：它会改变 overlap 统计口径，须在
1a 的 baseline 之上重新预注册。

## 8. OPEN

- `world_inherit_horizon_ns` 的 concrete config key 名、parser/serialization
  与 construction validation 归实施计划（沿用 M4 spec §7.1 的收口方式）。
- 继承时 root 是否需要 velocity prior、以及其 sigma 是否沿用
  `velocity_prior_sigma_mps=0.1`，属实施计划的数值项；本 spec 不冻结。
- 秒级桥接（V2_03 的 `13 s` 人群）**不在本片范围**。若将来要扩大 horizon，
  须先跑一次覆盖该量级的新探针，不得靠调大默认值达成。
- `phad/estimator/README.md` 与 `phad/bench/README.md` 的 `reanchor` 描述
  需随 §4.4 的语义变更同步更新。

## 9. 不做

不改 [architecture](../architecture.md)、[conventions](../conventions.md)、
[roadmap](../roadmap.md)；不引入新 solver、新线性代数、可观测性理论；
不做回环、全局地图、跨段 posterior 关联、正式 marginalization；
不在本片内实现 staged dynamic initialization（属片 4）。
