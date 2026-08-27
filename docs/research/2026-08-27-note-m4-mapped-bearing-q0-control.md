# M4 mapped-landmark bearing Q0 control diagnosis

## 1. 问题与证据边界

[Q0 control](../benchmark/m4/mapped-landmark-bearing-q0_42f99e9_0337287b.md)
在 clean `42f99e9/default_0337287b` 的首条 `MH_01_easy` 上得到 ATE
`0.0725592618 m`，超过冻结上界 `0.070539 m`。该值与 spec 记录的同 source
record-only run 一致，因此 dirty manifest 不是该数值差异的解释。

本诊断只定位 Q0 的既有行为差异，不修改 #43 或 #47 合同，也不把临时消融
当作候选实现。

## 2. 可重复的短回路

在两个 detached clean worktree 中分别构建 `c999f58` 与 `42f99e9`，对同一
`MH_01_easy` 输入运行：

```bash
./build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --out <baseline-or-candidate> \
  --sequence-name MH_01_easy_prefix_379 \
  --repo <matching-clean-worktree> \
  --estimator-enable-moving-bootstrap \
  --max-frames 379
```

两次运行各约 10 秒；`est.tum` 前 377 行逐字节相同，唯一进入该前缀的分叉为：

```text
line 378 / timestamp 1403636598.663555584
c999f58 : 0.02442980932050947 -0.0075171635448574893 -0.19176539154199376 ...
42f99e9 : 0.025041498631116734 -0.0079990170819646995 -0.19369089945278556 ...
```

首个 translation 差为 `0.0020769996 m`。同一候选的 full run 与 prefix run
在该行逐字节相同，排除了该边界上的运行间漂移。

## 3. 首分叉诊断

以 shared CSV 列按 timestamp 对齐，首个共同字段差异出现在对应 packet：

| 字段 | `42f99e9` 前一 packet | `c999f58` | `42f99e9` |
|---|---:|---:|---:|
| `num_shared` | `17` | `8` | `8` |
| `unsupported_span_ns` | `0` | 不适用 | `50000128` |
| `num_current_visual_factors` | `15` | 不适用 | `8` |
| `num_landmarks` | `80` | `78` | `79` |
| reprojection RMS before (px) | `0.559955` | `0.564185` | `0.577818` |
| reprojection RMS after (px) | `0.566685` | `0.539598` | `0.572089` |

`42f99e9` 在 `num_shared < min_pnp_inliers` 时按 #43 合同保留 observations，
并让 8 个当前 stereo factors 进入 graph；`c999f58` 的 coast frame 没有这些
当前 observations/factors。下一 packet 上，候选 PnP 成功且有 10 个 inliers，
control 则 fallback，行为开始进入离散分支。

全序列只有连续 2 帧同时满足 `unsupported_span_ns > 0` 与
`num_current_visual_factors > 0`。二者足以改变后续 PnP、outlier、keyframe 与
window 路径：两条 raw trajectory 的 translation 差在第 1225 行首次超过
`1 cm`，末行达到 `0.108139 m`；评估各自全轨迹对齐后表现为 ATE
`0.0705388305 → 0.0725592618 m`。

## 4. 临时消融

在临时 `42f99e9` worktree 中只让 low-support 当前 frame 跳过 visual graph
admission，同时保留它的 window/map 生命周期，然后重建并重跑 379 帧前缀：

| variant | prefix ATE (m) | prefix RPE (m) | line 378 translation 差 |
|---|---:|---:|---:|
| `c999f58` | `0.0161634` | `0.0125777` | `0` |
| `42f99e9` | `0.0161619` | `0.0125779` | `0.0020769996 m` |
| temporary no-current-factor | `0.0161634` | `0.0125777` | 约 `1e-15 m` |

临时源码随后恢复到 clean `42f99e9`。该消融证明首个有意义的位姿分叉由
low-support 当前 visual factors 直接触发；单独保留 map/window 生命周期在该
前缀只留下浮点舍入量级差异。它没有执行 full-run counterfactual，也不证明
“删除因子”满足 #43 的产品目标。

## 5. 结论与下一决策面

事实：Q0 的 clean identity、config 与输入均有效；首个 material 分叉来自
`bf8c321` 引入且由 #43 冻结合同明确要求的 low-support factor admission。
Q0 不是 #47 新增代码造成的回归，因为 #47 尚未进入 Q1。

推断：该分叉触发下一帧 PnP 的离散分支，并沿后续生命周期放大到 full-run ATE；
尚未执行 full-run counterfactual 来量化 retention 与 factor admission 各自的最终
ATE 贡献。继续 Q1/Q4/Q5 会把新 bearing 机制叠加到一个未通过自身 MH_01 产品门
的 control 上，最终无法把产品指标变化单独归因给 #47。

下一步应先为 #43 建立独立 control-repair slice，首个 Observe 问题是：这两帧
当前 factors 的 predicted/optimized residual、PnP 可验证性与后续离散分支中，
哪一项能提供不改变 support predicate 的质量许可。该 slice 需要重新对齐 factor
admission/quality 合同；通过 `MH_01 <= 0.070539` 后，#47 才能回到 Q1。

## 6. Full-run delayed-current-factor counterfactual

为量化 §4 留下的 full-run 证据缺口，在临时 `42f99e9` worktree 中采用同一
消融：low-support packet 仍立即提交 observations、track history、window 与 map
lifecycle，但 `buildGraph()` 在该 packet 上跳过当前 back frame；下一 packet
重建 graph 时，前一 low-support frame 已不再是 back frame，其已保留 observations
仍按既有 count/map 资格建立 factors。

完整 `MH_01_easy` 结果：

| 指标 | `c999f58` | delayed-current-factor | 差值 |
|---|---:|---:|---:|
| ATE trans RMSE (m) | `0.07053883048680992` | `0.07053883048875172` | `+1.94e-12` |
| RPE trans RMSE (m) | `0.03317561070387584` | `0.03317561070377601` | `-9.98e-14` |
| keyframe / track-only | `660 / 3022` | `660 / 3022` | `0 / 0` |
| PnP success / fallback | `3667 / 13` | `3667 / 13` | `0 / 0` |
| outliers culled / reopt | `17 / 1` | `17 / 1` | `0 / 0` |
| completion / coverage / segments | `0.9997284085 / 0.9997283354 / 1` | 相同 | `0 / 0 / 0` |

两条 raw trajectory 的最大 translation 差为 `9.73e-11 m`，末行差
`3.48e-11 m`。所有比较的离散 diagnostics 均相同；唯一生命周期差异是两个
low-support row 继续保留额外一个 landmark (`79` 对 `78`)，且其
`num_current_visual_factors == 0`。因此，在 `MH_01` 上 retention 对最终行为可视为
零影响，当前 endpoint factors 是 Q0 数值分叉的实际载体。

该 run 的 config 仍为 `default_0337287b`，但源码含临时消融并如实标记 dirty；
它只提供因果证据，不构成 clean candidate 或产品 PASS。

## 7. Factor residual 与现有质量门

恢复原 #43 factor admission 后，在两个 low-support packet 的 primary graph solve
前逐 factor 读取 `GenericStereoFactor::unwhitenedError()`：

| timestamp (ns) | factors | norm min / median / mean / max (px) | `> 2 px` | `> 3 px` |
|---|---:|---:|---:|---:|
| `1403636598663555584` | `8` | `0.102 / 1.124 / 0.910 / 1.343` | `0` | `0` |
| `1403636598713555456` | `8` | `0.338 / 1.270 / 1.197 / 2.199` | `1` | `0` |

两帧均无 cheirality；所有 residual norm 都低于现有 Huber `k=3`，landmark mean
也没有触发 `outlier_avg_reproj_px=4` 的 cull。现有 robust noise 与 mean-cull
按当前合同会接受这批 factors，因此不能用“复用既有 residual 门”解释或修复
Q0。临时 instrumentation 随后与 worktree 一同清除。

当前证据支持把 control-repair 的首选合同问题收窄为：是否允许
**one-packet delayed current-factor admission**——立即保留 low-support observation
与 map 生命周期，但直到下一次 graph rebuild 才让该 observation 约束 posterior。
这保留了后续视觉因果边，也在完整 `MH_01` 恢复冻结门；同时它会把 #43 的
“factor-bearing low-support 当前帧”改为“retained low-support 历史帧可在后续图中
成 factor”，需要先修订 #43 的机制诊断与对应 public-seam tests，不能作为 #47
内部实现细节直接落地。

## 8. Public-seam contract delta audit

把 delayed-current-factor 消融套到 clean `42f99e9` 的完整
`phad_estimator_tests` 后，102 tests 中 95 个通过、7 个失败：

| contract surface | 失败测试数 | 实际差异 |
|---|---:|---|
| current/total factor diagnostics | `6` | low-support 当前 frame count 从 observation 数变为 `0`；其余 recovery、segment、horizon、rollback 断言继续通过 |
| same-packet low-support cull | `1` | poison landmark 不在第 4 个 low-support packet cull |

把 poison fixture 延长一个 packet 后，既有 cull-ID 与
`outliers_culled >= 1` 断言通过；剩余失败只来自 current factor count 仍为 `0`
以及 coast/span 随新增 packet 从 `200 ms` 增为 `250 ms`。这证明 quality path
是随 observation factor **延迟一次 rebuild**，而不是丢失。

因此，若用户批准 control repair，需在 #43 amendment 中明确且只改以下时序面：

1. low-support 当前 stereo observation 立即进入 transaction state，但不在同一
   endpoint 的 graph 中建立 stereo factor；
2. 后续 graph rebuild 按原 count/map 门处理该历史 observation，既有
   cheirality、mean-cull 与 reopt 在该时点运行；
3. support predicate、PnP threshold、coast budget、outage/recovery cadence、seed、
   rollback 与 public `update()` seam 不变；
4. `num_current_visual_factors` 继续表示最终成功 graph 中连接当前 frame 的 factors，
   因而 low-support control row 为 `0`；总 visual factor count 反映已进入 graph 的
   历史 factors；
5. #47 后续对 current mono bearing factor 与 mapped-support 的权限仍由 Q4/Q5
   单独授予，不由该 stereo control repair 预先决定。

这组差异是现有 tests 与完整 MH_01 counterfactual 共同证明的最小合同面；不需要
新增配置、阈值、factor 类型或 public interface。
