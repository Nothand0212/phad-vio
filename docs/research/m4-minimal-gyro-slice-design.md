# M4 gyro measurement / factor 资格实验设计（修订版）

日期：2026-08-12

状态：设计修订完成；已纳入闭合审查勘误；Q1 Observe 实施计划已按本设计重写，
Q2–Q5 仍须在上一层通过后逐层新建独立计划，当前均未授权实施。

控制基线：`main@7026ebf` 的 M3 production VO + M4.1 IMU sync 数据通路。MH_01
权威控制组见 [M4 最小 gyro-aided VO：MH_01 控制组](m4-minimal-gyro-mh01-control.md)。

关联：issue [#36](https://github.com/Nothand0212/phad-vio/issues/36)

上位方法：[证据门控的信息接入](../agents/evidence-gated-integration.md)

既有提案：[M4 最小 gyro-aided VO 重启方案](m4-minimal-gyro-slice-proposal.md)

旧实施计划：[M4 最小 gyro-aided VO](../plans/2026-08-12_m4_minimal_gyro_slice_c4e62b35.plan.md)
（保留为历史，不得直接执行）

当前 Q1 计划：[M4 gyro Q1 Observe](../plans/2026-08-12_m4_gyro_q1_observe_7d3a91e6.plan.md)

本文档描述当前约定，不是绝对约束，会随项目开发修订。

## 闭合审查勘误（2026-08-12）

以下勘误优先于本文后续旧措辞：

1. Q1 必须在同一次 Observe run 同时冻结 packet summary 与 packet 中的逐样本
   gyro 数据。只有 `gyro_packets.csv` 不能脱离 dataset 重放；因此
   `gyro_samples.csv` 是 Q1 的同级必需产物，不是 Q2 的可选扩展。
2. Q4 active/attach 门的分母只包括 **graph-state-compatible packets**：packet
   两端 timestamp 对应本次受控图中真实存在的两个 Pose3 state、同 segment、exact
   interval 且合同有效。另行报告 `graph_state_compatible / all_valid_packets` 及 duration
   比例，作为描述性覆盖，不能用它把 Q4 判成 pass/fail。
3. 禁止为了抬高上述描述性覆盖而 merge packets、跨被拒帧拼区间，或把当前产品图改成
   全帧 state。packet merge 与 state topology 都是新的因果边，必须另立资格片。
4. Q2 的硬门只使用 known-bias synthetic integration；真实 MH_01 的 bias estimation、
   fit/held-out 与统计门全部属于 Q3，不得提前塞进 Q2。

## 0. 先说结论

本文不是“修好 VIO”的一片式方案，也不是“证明 IMU 能提高 ATE”。它定义一组依次放行的
**offline qualification experiments**：

> 在 gyro 数据合同、常值 bias 对齐和 rotation-only factor 机制分别取得证据后，允许
> gyro 在 MH_01 的一个受控实验中影响 M3 VO posterior，并观察自然闭环如何响应。

Q1 至 Q5 必须拆成独立、可停止的实施小片；表中的上一层未通过，不实现下一层。本文把它们
放在同一设计中，是为了固定整条证据链，而不是授权一次性把所有机制写进代码。

即使所有门都通过，本片仍不能把 gyro 默认打开，更不能宣称完成 VIO。原因是
\(\hat b_g\) 和 empirical covariance 使用了同一序列的未来数据；它们是实验 oracle，
不是在线初始化能力。

本片要获得的真正资产是：

1. 可复现的 IMU-off control；
2. 被 held-out 数据验证的 gyro rotation measurement；
3. 数学上严格 rotation-only、实际生效的 factor；
4. factor 进入受控图和自然闭环后的可归因结果；
5. 一个明确的下一层失败点，而不是一组“ATE 最好”的 covariance。

## 1. 本片位于资格阶梯的哪里

下表每一行都是独立实施 checkpoint，不是一个大提交中的阶段标签。

| 资格层 | 本阶段问题 | 新信息权限 | 是否在本设计覆盖 |
|---|---|---|---|
| Q0 Control | M3 VO 是否可复现 | gyro 无权限 | 已由控制组完成 |
| Q1 Observe | packet 的时间、单位、frame、区间和 gap 是否正确 | 只落盘和诊断 | 是 |
| Q2 Predict | 固定 bias 后的 gyro 积分能否解释 visual relative rotation | 只开环预测 | 是 |
| Q3 Align | 常值 gyro bias 在 MH_01 是否可辨识并能通过 held-out | 只离线估计 | 是，offline oracle |
| Q4 Constrain | rotation-only factor 是否按预期改变受控 posterior | 只进 synthetic / frozen-input 实验 | 是 |
| Q5 Feedback | 放进自然 VO 闭环后发生什么 | 可影响实验 posterior，默认关 | 是 |
| Q6 Default | 在线、跨序列、退化场景是否可用 | production 默认 | **否** |

因此，本片的成功状态是“gyro measurement / factor 获得局部资格”。后续至少还要单独证明
online bias initialization，再逐层证明 gravity、velocity、accelerometer bias 和完整
`X/V/B` posterior。

## 2. 单一因果问题与冻结项

整个资格系列最终只允许下面这一条新因果边改变 posterior；Q1 至 Q3 只能提供观测、预测和
离线参数，不能提前影响输出：

```text
同一对 Pose3 state
    ← 固定 bias 下、与端点严格同区间的 gyro ΔR
```

以下全部冻结：

- `StereoVoEstimator` 的视觉 residual、prior、fixed-window batch BA；
- tracker、KF schedule 规则、PnP、re-anchor、landmark lifecycle、cull/reopt；
- 所有 M3 参数与控制组数据范围；
- 不增加 `V`、`B`、gravity 或 accelerometer factor；
- 不引入 fixed-lag smoother；
- 不用 IMU prediction 改优化初值；
- 不扫 `gyr_nd`、factor covariance、Huber、PnP 或 frontend 阈值。

若自然闭环使 KF 或 lifecycle 分叉，那是 Q5 的**观测结果**，不是允许同步修改这些模块的
理由。

## 3. 关键合同

### 3.1 Pose 与坐标系

`est.tum` 发布的是 body pose \(\mathbf T_{WB}\)，因此 visual relative rotation 定义为：

\[
\Delta \mathbf R^{vis}_{ij}
=
\mathbf R_{WB}(t_i)^\top \mathbf R_{WB}(t_j).
\]

gyro 也在 body / IMU frame 中积分。当前 EuRoC normalized measurement 的 body 即 IMU；
本片不使用 `T_B_left` 转换 visual rotation。若未来允许 body 与 IMU 不重合，必须先在
sensor / calibration 合同中增加显式的 \(\mathbf T_{BI}\)，不能在实验工具中猜测。

角速度单位固定为 rad/s，时间单位在边界转换为 s；报告角度必须显式写 rad 或 deg。

### 3.2 时间区间与拓扑

`StereoImuPacket` 的 IMU segment 只证明原始相邻图像时间区间
\([t_{prev},t_{cur}]\)。它不能自动证明“当前 window 中相邻的两个 state”具有同一区间。

一条 gyro factor 只有同时满足以下条件才 eligible：

1. 两个端点 state 的时间戳分别**精确等于** packet 的 \(t_{prev}\) 和 \(t_{cur}\)；
2. 两端点属于同一 segment，且该 packet 没有 `imu_gap`；
3. IMU samples、\(dt\)、积分结果与 covariance 全部 finite；
4. \(\sum dt\) 满足 M4.1 sync 已定义的区间容差；
5. 两个 Pose3 key 都存在于本次被优化的图中。

若中间图像被拒绝、state 已不存在，或 window adjacency 跨过多个 packet，本片标记
`non_contiguous_state` 并跳过 factor。**不 merge segment，不把一个 packet 错接到更长
的 state interval。** packet merge 是需要独立验证的下一片。

### 3.3 Gap 与失败

- 单 packet gap：只跳过对应 factor，视觉路径继续；必须记录 count、duration 和原因。
- 单位、frame、时间反向、区间不闭合或非有限：hard failure，不得静默退回 gyro-off 后
  报告实验成功。
- eligible support 不足：结果为 `inconclusive`，不是 pass。

## 4. 两趟协议

### 4.1 第 1 趟：纯 VO control + 观测

`enable_gyro_factor` 缺省 / 关闭，估计器只走 M3 视觉路径。

必须满足：

- `est.tum`、`kf.tum`、`diag.csv` 与权威 control 逐字节一致；
- control 的 canonical config 和 `config_hash=402d1925` 不变；
- composition root 额外写 packet dump，但 dump 不反向改变 estimator；
- 每个 packet 记录端点 timestamp、segment、gap、sample count、\(\sum dt\) 与合同状态。
- 同时逐行写出 packet `samples` 中的 timestamp 与三轴 `gyro_radps`，保留 packet/index
  顺序与共享端点，使后续步骤只凭 `est.tum`、`diag.csv`、`gyro_packets.csv` 和
  `gyro_samples.csv` 即可重放，不再读取原 dataset 或重新实现 sync。

如果三主产物不一致，在 Q1 停止，不能继续解释任何 gyro 指标。

### 4.2 离线 alignment：fit 与 held-out 分开

离线工具只使用第 1 趟已经发布的 \(\mathbf T_{WB}\) 和 packet dump，不重新实现 sync。

对 MH_01 使用固定的时间协议：

- fit：从首个 eligible interval 起的前 100 s；
- validation：100 s 之后的剩余 suffix；
- exact 起止 timestamp、eligible count 和 duration 写入产物；
- 100 s 是本实验 protocol，不是 production initializer 参数。

在 fit prefix 上求一个全局常值 bias：

\[
\hat{\mathbf b}_g
=
\arg\min_{\mathbf b}
\sum_{(i,j)\in \mathcal F}
\left\|
\operatorname{Log}
\left(
\Delta \mathbf R^{imu}_{ij}(\mathbf b)^\top
\Delta \mathbf R^{vis}_{ij}
\right)
\right\|^2 .
\]

不使用 GT 求 \(\hat{\mathbf b}_g\)、选择 covariance 或决定 pass。EuRoC GT bias 只允许在
实验定案后做 post-hoc sanity check，并须单独标注。

alignment 证据包括：

- solve finite、Jacobian rank 与 condition；
- fit prefix 两个时间半段各自的 bias 及不确定性；
- fit 与 validation 上，零 bias / \(\hat b_g\) 的 adjacent residual；
- validation 上只由连续 eligible packet 组成的非重叠 1 s residual；
- 每轴 mean、RMSE、分位数、时间序列、lag 与 angular-speed 分桶；
- corrected 与 zero-bias residual 的 paired comparison。

放行第二趟前必须记录 go/no-go；前四项是 Q3 硬门，第五项只作拓扑覆盖描述：

1. rank、condition、finite 等数值门通过；
2. 两个 fit half 的 bias 在估计不确定性内相容；
3. validation 的 adjacent 和 1 s rotation residual 都优于 zero-bias；
4. validation residual 没有未解释的显著均值、轴向、时间或速度结构；
5. 报告能形成 exact contiguous visual interval 的 packet count/duration 占 all-valid packet
   的比例；该比例不设 95% 硬门，也不得通过 merge packet 或新增全帧 state 提高。

统计实现、condition 上限和“显著结构”的具体检验必须在 Q3 的独立
实施计划中预注册，并在 factor-enabled 结果产生前冻结。不得看到 ATE 后回改。

不再设置 \(\lVert\hat b_g\rVert<0.05\) rad/s 之类绝对大小门。bias 大小本身不证明
alignment 错误；应由可观性、稳定性和 held-out residual 决定。

### 4.3 第 2 趟：固定 offline bias 的 factor-enabled 实验

只有第 1 趟和 alignment 门通过才运行。第 2 趟仍使用同一数据和 M3 视觉配置，唯一新增
的 posterior 信息是第 5 节定义的 gyro rotation factor。

\(\hat b_g\)、bias covariance、empirical covariance、fit / validation 时间范围及各自
产物 hash 必须进入第 2 趟可复现元数据。由于这些量来自当前序列的未来数据，本趟结果只能
评价 factor mechanism 与 interaction，不能评价 online VIO。

## 5. 严格 rotation-only factor

不使用带“极大 translation sigma”的 `BetweenFactor<Pose3>`。那仍然表达了一个很弱的
zero-translation measurement，并非严格 rotation-only。

估计器 PIMPL 内实现局部 3D factor，概念 residual 为：

\[
\mathbf r_{ij}
=
\operatorname{Log}
\left(
\Delta \mathbf R^{imu}_{ij}(\hat{\mathbf b}_g)^{-1}
\left(\mathbf R_{WB,i}^{-1}\mathbf R_{WB,j}\right)
\right).
\]

约定：

- factor 连接现有两个 Pose3 key，但 residual dimension 固定为 3；
- 对两端 translation 的直接 Jacobian 严格为零；
- gyro preintegration、factor class 与缓存只存在于 estimator PIMPL；
- 名称可使用内部 `GyroRotationFactor`，不进入 public estimator API；
- GTSAM AHRS factor 若要求独立 Rot3 / bias variable，不能为了“复用库存类”改变本片
  已冻结的 state topology。

rotation-only 只表示 factor 没有直接 translation residual。完整 BA 中 rotation 与
translation 通过 visual factors 相关，posterior translation 间接变化是可能的，必须
报告而不能误判为 factor 泄漏。

## 6. 不确定性模型

factor covariance 必须说明来源，而不是从 ATE 反调：

\[
\Sigma_{ij}
=
\Sigma^{pim}_{ij}
+ \mathbf J_b\Sigma_b\mathbf J_b^\top
+ \Sigma^{model}_{edge}.
\]

- \(\Sigma^{pim}_{ij}\)：由已标定的 gyro noise density 按实际 samples / \(dt\) 传播；
- \(\Sigma_b\)：fit prefix 对 \(\hat b_g\) 的估计不确定性；
- \(\Sigma^{model}_{edge}\)：从 validation 的 exact adjacent residual 得到的保守 3×3
  PSD floor；先扣除可解释的平均 PIM / bias 项，再将负 eigenvalue 截到 0。

\(\Sigma^{model}_{edge}\) 含有 visual rotation error，因此只能作为保守上界。它不证明
gyro stochastic model 已校准，也不能迁移成 production 默认值。

每次运行报告三项 covariance 的 eigenvalues、总 covariance 的 condition，以及
unwhitened / whitened residual。若 residual 有系统结构，停止；不增加 isotropic RMS，
不扫统一 sigma 把 factor 隐藏起来。

## 7. 三种实验不能混称“同图对拍”

### 7.1 Synthetic fixed-graph oracle

这是 Q4 的机制硬门。构造已知 Pose3、已知 gyro motion 和可控冲突的最小图，证明：

- exact rotation 的 residual 为零；
- analytic Jacobian 与 numerical derivative 相符；
- 任意 translation perturbation 不改变 factor residual，translation Jacobian 为零；
- 有冲突时 posterior rotation 沿预期方向移动；
- 在刻意解耦 translation 的图中，factor 不直接移动 translation；
- rad/s 当 deg/s、反向 rotation、错 timestamp 能被测试检出。

### 7.2 Frozen-input replay

可以冻结第 1 趟的 observations、IMU packets 和外生 KF schedule，隔离 frontend 输入。
但 optimizer 结果不同仍可能改变 cull、reopt 和 graph membership，因此只能称
“frozen-input replay”，不能声称 graph 完全相同。

必须同时报告两臂的 state / factor / landmark counts 与第一次 graph divergence。

### 7.3 Natural closed loop

使用正常 composition root 和现有 feedback，比较 gyro-off / on。除指标外，报告首次
不同的 KF decision、PnP result、cull、reopt、re-anchor、rejected frame 与 segment。

如果指标变差，先定位最早分叉，不立刻扫 covariance。

## 8. 模块边界

| 组件 | 本片职责 | 明确不职责 |
|---|---|---|
| `phad::sync` | 既有 `StereoImuPacket` 与 gap / interpolation 合同 | 不算 bias，不预积分，不造 factor |
| `apps` composition root / 实验工具 | packet dump、offline alignment、两趟编排、独立实验产物 | 不把实验协议塞回 estimator |
| `phad::estimator` PIMPL | exact interval 校验、gyro PIM、3D rotation factor、active diagnostics | 不做全序列 LS，不加 `V/B`，不改 KF / lifecycle |
| `phad::bench` | 保持既有 run identity、config、path、summary 纯逻辑合同 | **不**读 pose / IMU，不承担 LS；保持零 `phad::*` 依赖 |
| tests | synthetic fixed graph、导数、单位、gap、frozen-input harness | 不形成第二套 production estimator |

旧设计把 offline LS 放入 `phad::bench`，违反该模块“零 `phad::*` 依赖”的现有合同；
本版将它放回 composition root / 独立实验 executable。

## 9. Artifact 与配置

既有 `diag.csv` 在两趟都保持 schema 不变。实验字段写入独立产物，避免研究协议污染长期
VO 合同：

| 产物 | 内容 |
|---|---|
| `gyro_packets.csv` | packet 端点、segment、gap、sample count、sum_dt、合同状态；Q1 必需 |
| `gyro_samples.csv` | 每个 packet 的逐样本 timestamp 与三轴 gyro（rad/s），保留 sample 顺序与共享端点；Q1 必需 |
| `gyro_alignment.json` | fit / validation 范围、bias、rank、condition、covariance、门结果、输入 hash |
| `gyro_alignment.csv` | 每 interval 的 visual / IMU rotation、zero / corrected residual、split |
| `gyro_factor.csv` | eligible、attached、skip reason、covariance eigenvalues、factor residual |
| 第 2 趟 `meta.json` | frozen bias / covariance 来源、实验开关、canonical config、artifact hash |

在 Q1 中，两个 CSV 分别先写同目录 temp file，成功 flush/close 后再 rename；
这只保证每个文件的独立原子 publish，不构成跨双文件事务。任一写入或 publish
失败都必须使整次 run 非零退出；只有成功 run 中两文件都存在且验证通过，
才能记为 Observe 成功。

默认关闭时不向 M3 canonical config 注入 gyro experiment keys，保持 control hash。
factor-enabled run 才记录 `enable_gyro_factor=true` 及全部冻结参数。GT 不进入任何配置。

## 10. 测试门

### 10.1 合同与单元测试

- packet \(\sum dt\) 与图像 interval；
- exact endpoint match 与 `non_contiguous_state`；
- gap 跳 factor 但不跳视觉 frame；
- gyro-only PIM 对解析 constant-rate rotation；
- rad/s / deg/s、rotation 方向、body / camera frame 错用；
- fixed bias correction 与 covariance propagation；
- factor residual、numerical Jacobian、translation invariance；
- eligible factor 全部 attached，所有 skip 都有唯一 reason。

### 10.2 数据实验门

| 层 | MH_01 门 |
|---|---|
| Q0 / Q1 | IMU-off 三主产物与 control byte-identical；canonical hash `402d1925` |
| Q2 | known-bias synthetic integration 解析门通过；不使用 MH_01 bias fit 或 ATE |
| Q3 | 第 4.2 节 alignment 前四项硬门通过并报告描述性拓扑覆盖；未通过不开 factor |
| Q4 active | 以 graph-state-compatible packets 为唯一 eligible 分母，`attached / eligible = 1.0` 且 attached 非零；另报 compatible / all-valid packet count 与 duration，仅作描述性覆盖，不设 0.95 硬门 |
| Q4 mechanism | synthetic fixed graph 全过；frozen-input 首次变化符合 factor 作用方向 |
| Q5 translation guardrail | exact-common ATE RMSE ≤ `0.100 m` |
| Q5 rotation guardrail | 1 s rotation RPE RMSE ≤ control + `0.0572958 deg`（即 `0.001 rad`） |
| Q5 availability | completion ≥ `0.9997284084736556`，coverage ≥ `1.0` |
| Q5 diagnostics | 报告 on/off 的严格 delta 与最早 schedule / lifecycle 分叉 |

控制组 rotation RPE RMSE 是 `0.15151944370189444 deg`，因此上述上限是
`0.20881524370189444 deg`。所有报告沿用 eval 当前的 degree 单位，不再写“或沿用现有
单位”。

`ATE ≤ 0.100 m` 是产品预算，不等于“相对 control 不回归”。若 ATE 从
`0.0809640579 m` 变为 `0.09 m`，应如实写“变差但仍在预算内”，不能写持平或改善。

## 11. 结果解释与停止条件

| measurement / factor 机制 | Q5 产品门 | 本片结论 | 默认状态 |
|---|---|---|---|
| 通过 | 通过 | qualification experiment 完成；可设计 online-init 下一片 | 仍关 |
| 通过 | 失败 | 局部机制成立，closed-loop interaction 未取得资格 | 关 |
| 失败 | 通过 | 指标可能由误差抵消得到，拒绝该实现 | 关 |
| 失败 | 失败 | 在最早失败层停止并记录负结果 | 关 |
| support 不足 | 任意 | `inconclusive`，不得宣称 pass | 关 |

任一前层失败就停止后层。尤其禁止：

- alignment 失败后用 GT bias 继续；
- factor 没有实际 attached 却用 gyro-on 标签报指标；
- ATE 变差后扫描 covariance；
- 为了过 MH_01 同时修改 KF、PnP、cull 或 tracker；
- 把 offline future-data 结果设为 production default；
- MH_01 未完成归因就扩 EuRoC 11/11。

fixed-lag 仍是独立课题：先在 visual-only 下证明它与 production VO 的精度、coverage 和
lifecycle 可接受，再让 IMU 取得进入该 posterior 的资格。

## 12. 本片明确不做

- 不做 IMU prediction / re-anchor 初值；
- 不加入 `V`、`B`、gravity、accelerometer residual 或
  `CombinedImuFactor`；
- 不做 per-KF bias random walk；
- 不实现 packet merge；
- 不修 P2b candidate 的 frame 533、PnP spikes 或 factor ledger；
- 不新建 `CandidatePipeline` 或第二套 estimator；
- 不把 frozen-input harness 变成 public API；
- 不把 experimental artifact 字段追加到长期 `diag.csv`；
- 不要求 ATE 必须优于 VO，也不把“预算内变差”写成不回归。

## 13. 对旧设计的关键修正

| 旧设计 | 问题 | 本版 |
|---|---|---|
| 全序列 LS 后在同序列宣布 alignment 成立 | 未来信息 + fit/eval 同源 | 前 100 s fit、suffix held-out；明确 offline oracle |
| \(\|\hat b\|<0.05\) rad/s | 物理大小不是正确性门，会拒绝合法 bias | rank、condition、split stability、held-out residual |
| window 相邻 state 直接接 packet | state adjacency 不保证 packet interval 相同 | 端点 timestamp exact match；否则 skip，不 merge |
| 必要时用 `T_B_left` 转 visual rotation | `est.tum` 已是 \(\mathbf T_{WB}\)；该转换会制造 frame 错误 | 直接 \(\mathbf R_{WB,i}^\top\mathbf R_{WB,j}\) |
| `BetweenFactor<Pose3>` + translation σ=1e3 m | 仍是弱 zero-translation measurement | residual dimension 3 的 rotation-only factor |
| fit residual RMS 各向同性膨胀 | 混淆系统误差与噪声，且丢失轴相关 | PIM + bias uncertainty + held-out 3×3 PSD floor |
| LS 放 `phad::bench` | 破坏 bench 零 `phad::*` 依赖合同 | apps / composition-root 实验工具 |
| frozen observations + KF schedule 称“同图” | optimizer/lifecycle 仍会改变 graph | 只称 frozen-input；另用 synthetic fixed graph 作 oracle |
| ATE / RPE 门代表 factor 正确 | 终局指标不能证明机制 | mechanism gate 与 product guardrail 分开 |
| 第 2 趟扩 `diag.csv` | 一次性实验污染长期 schema | 独立 gyro artifacts |

## 14. 下一步

1. 先停止执行旧计划中与本设计冲突的步骤。
2. Q1 Observe 计划已重写；Q1 未通过前不新建 Q2 实施片，后续 Q2–Q5 也必须
   在各自上一层通过后逐层新建独立计划；Q3 计划需预注册第 4.2 节
   尚需冻结的统计门。
3. 从 Q1 packet summary + raw gyro sample artifacts 开始；上一层没有证据，不实现下一层。
4. 本片结束后，根据最早失败层选择下一片，不直接跳到 full VIO。
