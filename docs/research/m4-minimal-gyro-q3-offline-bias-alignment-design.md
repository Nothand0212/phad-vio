# M4 Q3 offline constant-bias alignment 设计草案

日期：2026-08-13

状态：module/data/structural contract draft 已完成；**O0=A、O1-DELTA=A
（`primary.delta_loss_rad2=0`）、O1E-ANCHOR=A 与 O1E-BASIS=A 已冻结，
O1E-PATH=A 已选择；下一项仅为 exact product reference identity，其余 amendment pending，
protocol amendment STOP**。这不是 DUT fail：完整统计、solver、schema 与 owner budgets 尚未
冻结，Q3 plan 5/5 todos 仍 pending，Q3 implementation 与 Q4+ 未授权。

关联：issue [#38](https://github.com/Nothand0212/phad-vio/issues/38)

前置：

- [Q3 一手资料调研](m4-minimal-gyro-q3-offline-bias-alignment-research.md)
- [Q3 outcome-independent budget 调研](m4-minimal-gyro-q3-outcome-independent-budget-research.md)
- [Q2 资格结果](m4-minimal-gyro-q2-known-bias-predict-result.md)
- [证据门控的信息接入](../agents/evidence-gated-integration.md)
- [Q3 实施计划](../plans/2026-08-13_m4_gyro_q3_align_0e897ba8.plan.md)

本文是 Q3 O0/统计合同的**唯一 normative executable protocol authority**。其中明确标为已冻结的
O0/O1-DELTA/O1E-ANCHOR/O1E-BASIS 条款可规范性引用；标为 pending 的条目仍不可执行。所选
product endpoint 持有 meaningful-effect evidence authority；它不持有 executable protocol
authority。budget 文档只记录 evidence
与 owner decision history，structural research 只记录来源/历史，implementation plan 只安排步骤，
roadmap 只给出状态和 wayfinding；它们均不构成平行的 executable protocol authority。

## 1. 本片问题、权限与 fixed point

Q3 只回答：在不读取 GT、不改变 visual posterior 的前提下，能否从 Q1 的前 100 s eligible
prefix 拟合一个全局常值 gyro bias，并在严格 held-out suffix 上按 amendment 最终冻结的
practical gates 取得资格。

计划 fixed point 是 commit `3785acfc31225e270eff40b597300451a9fcae48` / tree
`2830cae722eb7c99b80b216c6cd77ecb874b128f`。Q3 只拥有 offline nuisance-parameter
qualification 权限；不得接入 `StereoVoEstimator`、`OfflineVoSession` 或 `phad_vo_bench`，不得
构造 factor、修改 posterior、读取 held-out suffix outcome 或 GT/ATE/RPE、运行 Q3 DUT、runtime、
qualification 或 Q5、产生 Q4+ 结果，或把 offline bias 称为 online initializer。Q3 source、tests
与 CMake 也仍为 STOP。

当前文档冻结不依赖 owner threshold 的结构、O0=A 以及 O1-DELTA=A 的零损失容忍，并完整列出
amendment blockers。不得把 candidate formula、future allowlist 或接口草图描述为已完成
protocol。

本次 amendment 在 O0=A 之上已写入 O1-DELTA=A 的
`primary.delta_loss_rad2=0`，并写入 O1E-ANCHOR=A 与 O1E-BASIS=A。O1-DELTA 以外的
`O1E-MME`（primary positive `δ_MME` evidence package + owner adoption decision）、
`O1E-ADJ`（adjacent-equivalence effect/margin）与 `O1E-POWER`（target power）
仍 pending / `UNFROZEN`。这三个名称只是稳定的 textual decision-record IDs，不是
JSON/schema/protocol fields；
`O2/O3/O4`、`U1/U2/U3`、`S1/S2`、`V1/V2`、`A1/R1` 也继续 pending。尤其不设置
positive effect、alpha，不冻结 mixed verdict precedence，也不改 schema、solver、Jacobian、
support 的实质合同。pending decisions 的依赖与提问顺序只引用
[budget §6 decision DAG](m4-minimal-gyro-q3-outcome-independent-budget-research.md#6-无循环-decision-dag)
和 [budget §8 owner choices](m4-minimal-gyro-q3-outcome-independent-budget-research.md#8-owner-choices-顺序与最小下一步)；
本文不复制该 DAG 或 V2 alternatives，owner 选择后只在本文单向写入 adopted normative contract。

## 2. Deep module 与依赖方向

### 2.1 唯一 public seam

未来获 implementation go 后，只暴露一个 deep module interface：

```cpp
Q3ExecutionResult runGyroAlignmentQualification(
    const GyroAlignmentRequest& request);
```

`GyroAlignmentRequest` 只携带 frozen protocol identity、Q1 run directory 与 output destination；
不得注入 threshold、margin、alpha、bandwidth、solver strategy、weights、fallback 或 artifact
policy。返回为 `Q3Report | Q3HardError`；report 内含一个科学 verdict，hard error 不伪造
report/completion。caller 不参与 parse、eligibility、fit、statistics 或 publish 编排。

所有纯计算 helper 留在 `apps/gyro_alignment.cpp` 的 anonymous namespace。当前只有一个真实
caller，不新增 internal header/interface；出现第二个真实 caller 时再另案判断 seam。deletion
test 应证明删除该 module 后复杂度会重新散落到 CLI/测试，而不是只删掉 pass-through。

### 2.2 Future dependency graph

```text
phad_gyro_align
  -> phad_gyro_alignment
       -> phad::eval
       -> phad::estimator
       -> Eigen3::Eigen                 (PRIVATE)
       -> nlohmann_json::nlohmann_json  (PRIVATE)
```

`phad::estimator` 只提供已取得 Q2 资格的 `integrateGyroRotation`；不扩 estimator public
interface，不把 alignment、统计或 artifact writer 下沉进 estimator。`phad::bench` 保持零
`phad::*` 数据处理职责。不投机修改 README。

## 3. 输入 identity、provenance 与 schema blocker

### 3.1 Runtime 唯一可打开的输入

| artifact | parse 前 expected SHA-256 |
|---|---|
| `est.tum` | `18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321` |
| `diag.csv` | `1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb` |
| `gyro_packets.csv` | `fbf574545e418fc19d100b7b05f79546a5336420bca60852770605b42702a3fe` |
| `gyro_samples.csv` | `da35227b40a1ab47c94217b4210b5445d11927a864d6634f8b824812ccec0344` |

runner 的 filesystem contract 只允许打开这四个 artifact。执行顺序固定为：resolve exact
paths → size/SHA-256 → parse/schema → cross-file order/join → eligibility。任一 hash mismatch
不得进入 parse 或产生候选科学 verdict。

Q1 final ledger 的 `config_hash=402d1925` 与 input manifest v2 SHA-256
`aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5` 是 protocol
provenance，必须写入 report；runner 不能声称从四文件重新验证 config 或原 manifest。

### 3.2 Protocol amendment 必须冻结的 input contract

下列内容尚未冻结，缺任一项都不能实现 parser/eligibility：

1. `gyro_packets.csv` eligible `status` 的 exact enum value；
2. `diag.csv` 的 exact ok column/value、两端 join key 与 accepted 语义；
3. packet/diag/pose 的 exact segment join，segment 缺失或冲突的处理；
4. `est.tum` decimal timestamp 无损转换为 `int64_t` nanoseconds 的 grammar、舍入禁止规则、
   overflow 与 canonical representation；
5. duplicate/missing/out-of-order/nonfinite pose、quaternion normalization/frame mismatch 的错误；
6. packet/sample header、字段、类型、单位、row order、shared endpoint 与 count/duration 校验；
7. 每个 exclusion/error 的唯一 stable enum，以及 local exclusion、`HARD_ERROR`、
   `INCONCLUSIVE` 的 exact mapping 和优先级。

不得用“accepted/ok”“valid”自然语言替代这些 predicates，也不得在实现中临时选择 CSV 值。

## 4. 已选定的时间与 held-out observation/block 结构

### 4.1 Eligibility、split 与 fit halves

首个满足 amendment 最终 exact packet/pose/diag/segment predicates 的 interval 定义 (t_0)：

\[
t_{mid}=t_0+50000000000\ \mathrm{ns},\qquad
t_{split}=t_0+100000000000\ \mathrm{ns}.
\]

结构固定为：

```text
full fit  : t_prev >= t0    && t_cur <= t_split
early half: t_prev >= t0    && t_cur <= t_mid
late half : t_prev >= t_mid && t_cur <= t_split
validation: t_prev >= t_split
```

- split-straddling interval 不进 fit/validation；
- `t_mid` straddling interval 不进 early/late halves，但可留在 full fit；
- early/late 从**相同权威初值**独立 refit，使用相同 objective、weights、solver、Jacobian 与
  termination；
- validation 只使用 full-fit bias，不使用 half-fit bias；
- 权威初值是 zero 还是其他值尚未决定，必须随完整 solver contract 在 amendment 中冻结。

### 4.2 Held-out adjacent paired observation

对 validation 中每个满足 amendment 最终 eligibility predicates 的 held-out packet interval
\(I_p=[t_{prev,p},t_{cur,p}]\)，adjacent observation 必须使用完全相同的 visual endpoints：

\[
\Delta R^{vis}_{p}=R_{WB}(t_{prev,p})^\top R_{WB}(t_{cur,p}).
\]

zero/full-fit bias 两臂分别对该 packet 恰调用一次 Q2 `integrateGyroRotation`，得到

\[
r_{0,p}=Log(\Delta R^{imu}_{p}(0)^\top\Delta R^{vis}_{p}),\qquad
r_{b,p}=Log(\Delta R^{imu}_{p}(\hat b)^\top\Delta R^{vis}_{p}),
\]

并记录逐 packet paired improvement

\[
d^{adj}_p=\lVert r_{0,p}\rVert^2-\lVert r_{b,p}\rVert^2
\quad[\mathrm{rad}^2].
\]

不得为 adjacent 路径重采样、merge、partial integrate、跨 gap 拼接、重做 sync 或改换 visual
endpoints。adjacent observations 与下节 1 s blocks 共用底层 packets，因此不得把两者混作同一
analysis table 的独立 samples、相加扩大样本量，或以 pseudo-replication 计算 support/power。

该路径是 overall IUT 下的 mandatory diagnostic component，但当前只冻结 observation
construction；estimand、aggregation/inference 与 go/no-go 边界仍属 §6 protocol amendment。
它不成为第二 primary；完整 1 s block 仍保留 primary 的结构性名称，但两者都不构成独立
confirmatory claim。

### 4.3 Exact non-overlap 1 s block

对非负整数 (k)：

\[
B_k=[a_k,b_k]=
[t_{split}+k\cdot10^9,\ t_{split}+(k+1)\cdot10^9]\ \mathrm{ns}.
\]

一个 block 只有同时满足以下条件才存在：

1. 首 interval `t_prev == a_k`，末 interval `t_cur == b_k`；
2. block 内相邻 intervals 的前一 `t_cur` 与后一 `t_prev` exact equal；
3. 全部 intervals 都满足 eligibility 且属于同一 segment；
4. packet 按 `(k, t_prev, t_cur)` canonical order 唯一排列，无 duplicate/overlap；
5. 不 merge、不 interpolation、不 partial、不重采样、不重做 sync。

对每个 `(bias arm, packet)` 恰调用一次 Q2 `integrateGyroRotation`，按时间 chronological
right-compose packet rotations 得到 block IMU rotation。shared endpoint 的 timestamp/value 必须
一致，时间只覆盖一次；禁止把相邻 packet 的共享 point 当作额外 interval 重复积分。输出 rows
按 (k) 排序。缺失 (B_k) 后不得把 (B_{k+1}) 改号或压紧成相邻 statistical lag。

visual rotation 只使用 block 的直接 endpoints：

\[
\Delta R^{vis}_{B_k}=R_{WB}(a_k)^\top R_{WB}(b_k).
\]

zero/full-fit bias arms 定义为：

\[
r_{0,k}=Log(\Delta R^{imu}_{B_k}(0)^\top\Delta R^{vis}_{B_k}),
\]

\[
r_{b,k}=Log(\Delta R^{imu}_{B_k}(\hat b)^\top\Delta R^{vis}_{B_k}),
\]

\[
d_k=\lVert r_{0,k}\rVert^2-\lVert r_{b,k}\rVert^2
\quad[\mathrm{rad}^2].
\]

near-\(\pi\) `Log` 的 valid/exclusion/hard-error 状态与容差尚未冻结。缺块后的 HAC 使用
calendar lag、分 run 还是拒绝整段也尚未冻结；implementation 不得自行压紧 gaps。

## 5. Solver、Jacobian、rank 与 uncertainty amendment

full/early/late fit 的概念 residual 均为

\[
r_{ij}(b)=Log(\Delta R^{imu}_{ij}(b)^\top
              (R_{WB}(t_i)^\top R_{WB}(t_j))).
\]

bias 单位 rad/s；IMU arm 只能调用 Q2 helper，保留 endpoint average、bias-once、right-compose
与 exact duration。不得交换 transpose/order、再减一次 bias或写第二套积分器。

以下 solver contract **全部 pending protocol amendment**：

1. residual stack canonical order、每 interval/block 的 weights 与 rank matrix (W/J)；
2. 是否使用 unweighted LS；禁止 robust loss 已选定，但 exact objective 仍须写出；
3. optimizer 名称、library/version 与全部参数；
4. authoritative initialization 与 additive bias update convention；
5. iteration limit、step/cost/gradient convergence、termination priority；
6. no fallback/restart/clamp，nonconvergence 属于 `HARD_ERROR` 还是 `INCONCLUSIVE`；
7. rank evaluation point、用于 rank 的 exact matrix 与 whitening/weighting；
8. covariance estimator、degrees of freedom、full-fit 3-axis uncertainty 与 finite checks；
9. deterministic ordering、threading、locale、floating environment 与 repeatability tolerance；
10. analytic Jacobian definition；独立 central finite difference 的 step、norm、absolute/relative
    tolerance、evaluation points 与 rejection rule。

对 amendment 最终 rank matrix 的 SVD threshold 结构固定为

\[
\tau=\epsilon\,diagSize\,\sigma_{max}=3\epsilon\sigma_{max},
\qquad diagSize=\min(rows,cols)=3,\qquad rank=3.
\]

`diagSize` 遵循 Eigen 3.4 documented default；singular values/condition 均报告，
condition 只作 descriptive，不设 `1e6` 门。rank evaluation
point/matrix 尚未冻结，所以不能声称 rank protocol 已完整。即使 rank=3，full-fit bias 三轴
uncertainty 仍必须分别落入 outcome 外 owner budgets；三个 budgets 当前缺失。

## 6. O0 normative statistical protocol

O0=A 已冻结唯一 sole overall conjunctive confirmatory qualification claim：overall null 是所有
mandatory preregistered component null 的 union，overall alternative 是这些 component
alternatives 的 intersection；overall PASS 当且仅当每个 mandatory component 都 reject。primary、
12 个 equivalence TOST 与 adjacent 等名称只标识 mandatory diagnostic components，不形成独立
confirmatory claim；component actual 可进入 artifact/result 供诊断，但不得形成独立可发布 claim。

O0 的规范性 claim 与决策函数为

\[
H_0=\bigcup_j H_{0j},\qquad H_1=\bigcap_j H_{1j},
\]

\[
\operatorname{PASS}\iff
\bigcap_j\{\text{mandatory component }j\text{ rejects its complete }H_{0j}\}.
\]

任一 mandatory component 未 reject（包括未冻结、未实现、未执行或其前置不足）均不得
产生 `PASS`；具体非 PASS 状态的聚合优先级由 V2 冻结，当前仍 pending。

每个 component test 对其完整自身 null 都须 level 不大于同一个 overall alpha，即可控制 sole
overall Type-I 不大于 alpha。不得跨 component 做 Bonferroni、Holm、FWER allocation 或 alpha
分账。overall alpha 数值、critical law、df/SE/HAC validity 仍由 S1 冻结，本节不填数值。
所有 fixed-point 旧 alpha 数值与分配已在
[budget 调研的单一退役账本](m4-minimal-gyro-q3-outcome-independent-budget-research.md#41-fixed-point-旧-alpha-tuple-退役账本)
中标为 superseded / non-normative / qualifying weight `0`，不得当作 S1 候选证据。

### 6.1 Primary diagnostic component：HAC

保留 primary 的结构性名称；其 sample unit 是完整 1 s block 的
\(d_k\,[\mathrm{rad}^2]\)，但它不是独立 confirmatory claim。O1-DELTA=A 已冻结唯一规范定义：

\[
d_k=loss_{zero,k}-loss_{corrected,k}
=\lVert r_{0,k}\rVert^2-\lVert r_{b,k}\rVert^2
\quad[\mathrm{rad}^2],
\]

正值表示 corrected arm 的 held-out squared loss 低于 zero arm。字段、单位与 owner tolerance 为

```text
primary.delta_loss_rad2 = 0  # rad^2
```

因此 owner 不允许 corrected arm 的 held-out mean squared loss 变差。该决策的精确标签是
O1-DELTA=A，不得使用未限定的简写。这个零损失容忍不是
positive primary minimum meaningful effect，也不替 owner 完成 `O1E-MME`、选择 design
alternative 或 `O1E-POWER`；这些仍 pending / `UNFROZEN`。

O1E-ANCHOR=A 冻结且仅冻结以下规范条款：downstream 1 s
relative-orientation consistency / product orientation-quality requirement 是 positive `δ_MME`
evidence chain 的唯一上位 normative anchor，并持有 meaningful-effect evidence authority。
Q3 design 仍是唯一 executable protocol authority。`O1E-ANCHOR=A` 与 budget 中只选定
workflow 的 `O1E-PATH=A` 是不同 decision labels；两者都不是 protocol schema
field。exact product requirement identifier/version/owner/baseline provenance、exact product
reference identity/bytes/identifier/version/owner/baseline provenance 与 exact endpoint semantics
仍待提供和冻结；下一项只允许冻结 exact product reference identity。`1 s` 只是该 product
endpoint 的 identity/horizon，不自动
冻结 Q3 analytical aggregation/window/block/axis/frame/reference semantics。

O1E-BASIS=A 冻结且仅冻结以下规范条款：downstream product endpoint 的 primitive comparison
basis 是 **arm-wise common-reference quality**。zero-bias arm 与 fitted-bias arm 必须先各自相对
同一个 downstream product-reference role 评价 endpoint-native quality；paired benefit 只能随后由
这两项 arm quality 派生。该 common-reference 关系必须两臂共享、arm-independent 且
pre-outcome；reference 不得由任一 arm output、fitted bias、future DUT/outcome 或 desired PASS
选择。`O1E-BASIS=A` 是稳定 textual decision-record ID，不是 JSON/schema/protocol field。

上述条款只定义 product endpoint evidence semantics，不改写或升级当前 Q3 analytic
\(\Delta R^{vis}\)、\(r_{0,k}\)、\(r_{b,k}\)、\(d_k\)、primary 或 adjacent estimand，也不把它们冒充已选
product reference。只有 exact reference、metric 与 mapping 后续分别冻结后，两项 product
endpoint arm quality 才可单向映射到 Q3 paired estimand；绝不能由 `d_k`、`μ_d` 或 outcome
反向定义 reference 或 threshold。hazard-analysis 与 mission-success endpoint 仍只是未选作
co-equal anchor；该状态不是对它们的全局否决。

O1E-ANCHOR=A 与 O1E-BASIS=A 均不冻结 `δ_MME`；`δ_MME` 仍为 `UNFROZEN`，
`μ_d [rad²]` 仍是目标 population 上 true mean paired squared-SO(3)-loss improvement 的未知
estimand。仍未冻结：exact product reference identity/bytes/identifier/version/owner/baseline
provenance、任何具体 GT/reference、metric/loss/formula、1 s analytical window/block、time pairing、
axis/frame/direction/handedness、target population/domain、risk/severity/threshold、mapping
assumptions/formula、`μ_d`、positive `δ_MME`、rounding/strict boundary、`O1E-MME`/
`O1E-ADJ`/`O1E-POWER`、`U/S/V/A` decisions、schema/protocol numeric field、implementation go
与 Q3 outcome。它们均保持 pending / `UNFROZEN`；下一项只能是 exact product reference
identity，不得提前代选。

当且仅当该 component 的全部前置与 support 充分，且按 S1 最终冻结的同一 overall
alpha/critical law 计算的 held-out mean improvement 单侧下置置信界满足

\[
LCB(d)>0\ \mathrm{rad}^2
\]

时，primary component 才 reject 其完整 null。不等号是严格的：`LCB(d)=0`、零收益、
负收益，或仅有正 point estimate 但 `LCB(d)<=0`，均不 reject。primary component reject 只是
O0 sole overall conjunctive IUT 的一个必要 component，绝不单独产生 overall `PASS`。
`O1E-MME` adoption 本身不设置 alpha 或 critical law；`O1E-MME` 仍 pending /
`UNFROZEN`，两者仍由 S1 冻结，当前 pending。

候选 bandwidth 为

\[
L=\lfloor4(n/100)^{2/9}\rfloor,
\]

但 amendment 仍须精确定义：

- HAC autocovariance formula、centering、分母与 normalization；
- Bartlett weights 与是否 small-sample corrected；
- S1 冻结的同一个 overall alpha、critical distribution（Normal/t）、df/SE 与 HAC validity；
- negative/zero estimated variance、nonfinite、`n <= L`/small-n 的处理；
- missing calendar block 的 lag、contiguous-run aggregation/combination 或 rejection rule；
- sensitivity `L=0/2L` 是否只 descriptive 及其不影响 verdict 的机器可检验合同。

在这些量冻结前，“使用 Bartlett/Newey-West”不是完整可执行统计门。

### 6.2 十二项 equivalence diagnostic components

sample unit 仍是完整 1 s block。结构候选恰为：

1. early/late independently refit bias difference 的 3 轴；
2. validation residual mean 的 3 轴；
3. validation normalized-time slope 的 3 轴；
4. validation centered-speed slope 的 3 轴。

half-bias difference 的 joint/separate covariance、cross-half independence 与 df 尚未冻结。
validation 建议一次联合回归：

\[
r_{axis,k}=\beta_{0,axis}+\beta_{t,axis}x_{t,k}
           +\beta_{s,axis}x_{s,k}+\varepsilon_{axis,k}.
\]

但 `x_t` center/scale、speed 的来源/frame/formula、`x_s` center/scale、OLS/weights、design-rank、
covariance/HAC、Normal/t 与 df 均 pending owner decision；不得把“mean/time/speed”标签冒充 exact
hypothesis。

每项 equivalence component 内部的 TOST 必须让两个 one-sided tests 都在 S1 冻结的同一个
overall alpha 下 reject；若 matching CI 为 equal-tail interval，则 coverage 为 (1-2alpha)，且 CI
必须严格位于对应 outcome 外 margin 内。每个 component test 必须对其完整自身 null 均为 level
不大于该 overall alpha；不在 12 项之间分账。alpha 数值、critical distribution、df/SE、HAC
validity 与 covariance 未冻结，故 12 项门仍未完成。

### 6.3 Mandatory adjacent diagnostic component

§4.2 的 held-out adjacent paired observations 必须形成身份明确的 diagnostic component；它不改变
§6.1 的 primary 结构性名称，也不是 §6.2 的 equivalence component。由于 adjacent 与 1 s block
重用 packet evidence，amendment 必须显式建模这种依赖，不能假定两路 samples 独立。它与其他
component 一样只能共同支持 sole overall claim，不得单独发布 confirmatory claim。

Slice 2 implementation go 前必须冻结并独立评审：

1. paired improvement 序列的 exact estimand、方向和 aggregation/statistic；
2. packet-level autocorrelation estimator/lag/normalization/small-sample rule，以及 segment、local
   exclusion 与 calendar gap 是保留 calendar lag、分 contiguous runs、组合还是拒绝；
3. 对完整 adjacent component null 的 level 控制、critical law 与 df/SE/HAC validity，使用 S1
   冻结的同一个 overall alpha；不得另配 alpha，也不得与 primary/12 项 components 做跨 component
   multiplicity allocation；
4. outcome 外 practical margin/effect 与 design alternative，含单位和独立来源；旧 adjacent
   residual/RMSE hints 与探索性摘要不能自动成为 threshold；
5. 仅凭 prefix、不得查看 held-out outcome 的 prospective variance/support/power 方法、required
   packet/run/segment support 与 infeasible rule；不得把已知 suffix capacity 当作 requirement cap；
6. point estimate、confidence bound 与 practical boundary 的严格 `>`/`<`/equality/nonfinite
   语义，以及 support 充分但科学边界未达时 `HYPOTHESIS_FAIL`、support/power/科学前置不足时
   `INCONCLUSIVE` 的唯一映射。

未冻结、未实现或未实际执行该 diagnostic component 时，Q3 在任何情况下都不得 `PASS`。protocol
未闭合时仍是当前的 amendment `STOP`，不产生科学 verdict，也不冒充 adjacent hypothesis 已
失败。只有冻结 protocol 实际执行后，prospective prerequisite/support 不足才是
`INCONCLUSIVE`；前置与 support 充分但未达已冻结科学边界才是 `HYPOTHESIS_FAIL`。

### 6.4 Prospective support 与 power

minimum support 不得是固定 60，也不得被已知 suffix capacity cap/下调。amendment 必须在查看
suffix outcome 前冻结：

1. 只由 prefix 构造的 primary `d`-like series 的 exact block/pseudo-block definition；
2. prefix long-run variance estimator，与 primary 分析是否同 normalization/gap rule；
3. outcome 外 primary design alternative（rad²）、`O1E-POWER` target power，以及 S1 冻结的
   overall alpha；
4. sample-size/power equation、critical distribution、ceil、integer overflow 与 infeasible 结果；
5. 12 项 equivalence 的 precision/power rule、每项 design alternative 与 family requirement；
6. adjacent component 的 prefix-only prospective variance、support/power、design alternative 与
   requirement，且不得把同一 packets 在 adjacent/1 s 两路重复计为独立 support；
7. observed/known support 不得参与 effect/margin 选择，也不得 cap calculated requirement。

任一 prospective prerequisite 或 required support 不足是 `INCONCLUSIVE`；支持充分后任一科学门
不达是 `HYPOTHESIS_FAIL`。不得把探索性 census 变成门；没有 exact command/log/hash 的 count
qualifying weight 为 `0`。

### 6.5 Outcome 外 owner budgets

owner 必须独立提供且说明量纲/来源：

- primary design effect（与已冻结的 `delta_loss=0` 不是同一个量；
  O1E-ANCHOR=A 只冻结上位 evidence authority，不冻结该 effect）；
- `O1E-POWER` target power；
- 3 个 full-fit bias uncertainty budgets；
- 3 个 half-bias difference margins；
- 3 个 residual-mean margins；
- 3 个 normalized-time-slope margins；
- 3 个 centered-speed-slope margins；
- 12 项 equivalence design alternatives；
- adjacent diagnostic component 的 practical margin/effect 与 design alternative。

Q5 `0.001 rad` 1 s product RPE guardrail 不得跨维度推导 rad/s、slope 或 rad² budgets。

## 7. Verdict 与错误优先级

逐 component 先形成 machine-checkable diagnostic result：prerequisite、required support 或 prospective
power 不足映射为 `INCONCLUSIVE`；这些前置充分而科学边界未达映射为 `HYPOTHESIS_FAIL`；完整
component null 被拒绝才记为 component reject。component reject/actual 只作诊断，不构成独立
confirmatory claim。

以下是 unordered status/exit-code candidate mapping；为便于核查仅按 exit 数值排序。row order
不具语义，也不定义 mixed aggregate precedence：

| 状态 | exit | 无重叠判定 |
|---|---:|---|
| `PASS` | 0 | 所有 mandatory preregistered components 都按冻结 protocol reject，因而 sole overall union-null 被合取拒绝；不发布 component claim |
| `HARD_ERROR` | 1 | input identity/schema/integrity、计算 finite/合同、I/O/publish 失败；不产生科学 completion |
| `HYPOTHESIS_FAIL` | 2 | 至少一个前置充分的 mandatory component 未达科学边界；mixed aggregate 如何折叠由 V2 冻结 |
| `INCONCLUSIVE` | 3 | integrity/computation 均有效，但至少一个 mandatory component 的 prerequisite/support/power 不足；mixed aggregate 如何折叠由 V2 冻结 |

V2 仍 pending；O0 不冻结 mixed aggregate precedence。待决 alternatives 只见
[budget §8 的 A/B/C decision record](m4-minimal-gyro-q3-outcome-independent-budget-research.md#8-owner-choices-顺序与最小下一步)。

protocol amendment 还必须把每个 error/exclusion/nonconvergence/near-\(\pi\) 情形映射到唯一状态，
消除 component-level 状态重叠；这项唯一映射要求不定义 aggregate precedence。

## 8. Artifact schema 与 manifest-last publication

### 8.1 Pending exact schemas

amendment 必须冻结：

- `gyro_alignment.json` schema version、所有 field/type/unit、object/array order 或 canonical JSON
  serialization；
- `gyro_alignment.csv` schema version/header、column type/unit/order、canonical float/integer text、
  row key/order（至少能分别唯一追踪 adjacent packet (p)、block (k) 与 exclusions，且不把两者
  表示成独立同质 samples）；
- completion manifest 的 schema version、field/type/order、canonical serialization 与 protocol
  hash algorithm/input；
- report 中 commit/tree、Q1/Q2/protocol/analyzer identity、exact command/environment、四输入
  hash/size/schema/rows、join/exclusion/support、fit/validation ranges、solver/stat actual 与 output
  hashes 的 exact names。

O4/A1 尚未冻结最终 schema 字段名，但 future schema 必须以机器可核查方式表达以下语义（这些是
semantic requirements，不是字段命名决定）：`claim_scope` 为 sole overall conjunctive claim；每个
component 及其 actual 都是 `diagnostic-only`；artifact/result 满足 `no-component-claim`，不能编码或
导出独立 component confirmatory claim。consumer/verifier 必须拒绝缺失、矛盾或把 component
reject 提升为可发布 claim 的 artifact。

元数据必须含：

```text
offline_future_data=true
bias_fit_available_after=t_split
qualification_available_after=run_end
q1_config_hash_provenance=402d1925
q1_input_manifest_v2_sha256_provenance=aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5
```

后两项是 ledger provenance，不冒充 runtime 从四文件重新验证的事实。

### 8.2 唯一 publish protocol：manifest-last completion marker

不再保留 atomic-directory-rename 备选。未来实现严格执行：

1. 在 staged output directory 将 CSV/JSON 写到各自 temp path；
2. 每个 data/report temp file 全部写完并 flush/close；
3. rename 为最终 data/report filename；
4. 对最终文件重新计算 size/SHA-256 并复核；
5. 最后写 completion manifest temp，flush/close 后 rename 为最终 manifest；
6. consumer 只有在 manifest 存在、schema/identity 有效且全部 size/hash 匹配时才认定科学结果
   完成。

任何失败都不得留下科学 completion manifest。可另写明确标为 non-scientific、且不被 consumer
当作 completion 的 failure receipt；它不能包含候选 verdict 冒充结果。

三种科学 verdict 都必须走完整 manifest-last transaction；`HARD_ERROR` 不发布科学 report
completion。结论措辞仅允许“相对于 frozen VO 的 temporal held-out consistency”，不得声明
physical true bias 或 online initialization。

## 9. GT physical isolation

qualification runner 及 module 只能打开四个 Q1 artifacts；不得接受 GT path，不链接 truth/ATE
parser，也不得通过 cwd/environment 猜测 GT。未来 post-hoc sanity 必须另立未授权工具，读取
immutable completion manifest/verdict hash，且不能修改 Q3 verdict/artifacts。

## 10. Future TDD 两趟与 allowlist

### 10.1 RED retention

Slice 2 amendment 独立评审通过且另获 implementation go 后，先冻结并保存：exact command、
build directory、environment、stdout、stderr、exit、failure class 与 log SHA-256。RED 必须由
缺失 Q3 interface/symbol 或预注册行为失败导致；归档后才允许 GREEN。删除/恢复证据必须证明
tests 约束新 module。Q2 的 retention waiver 不得泛化。

### 10.2 GREEN / independent requalification

tests 必须覆盖 identity-before-parse、schema/join、split/halves/block、adjacent 同端点 zero/bias
pair、shared endpoint、禁止 resample/merge/pseudo-replication、SO(3)、solver/Jacobian/rank、
primary/equivalence/adjacent components 的完整自身 null、共同 overall alpha 与 support/power、
strict component boundary、V2 aggregate precedence、manifest-last、GT isolation，以及 §8.1
`claim_scope` / `diagnostic-only` / `no-component-claim` 语义。mutants 至少覆盖 transpose、compose
order、rad/deg、ns/s、bias sign、double subtraction、eligibility、adjacent 端点错配、adjacent/1 s
独立性误设、把 alpha 按 `alpha/12` 或其他方式跨 component 分账、component claim 泄漏、
hash/schema bypass、gap compression 与 early manifest；每类 mutant 都必须被预注册 oracle 检出。

primary strict-boundary 的 exact future oracle 只在本文定义：在其余前置、support 与 S1
inference 合同已冻结且有效的 fixture 中，分别注入 `LCB(d)<0`、`LCB(d)=0`、
`LCB(d)>0` 三个边界值；前两者必须使 primary component 不 reject，第三者只能使
primary component reject。第三者在其他任一 mandatory component 未 reject 时仍必须禁止 overall
`PASS`。budget research 不持有该可执行 oracle，只保留 owner decision history 并链接本节。

独立 verifier 从固定 commit/tree fresh build，逐门记录 actual、command/log 与 hashes；test
process exit 0 不能替代逐门证据。

### 10.3 Future allowlist

**当前不生效。**只有完整 protocol amendment 独立评审通过并另获 implementation go 后才生效：

```text
CMakeLists.txt
apps/gyro_alignment.hpp
apps/gyro_alignment.cpp
apps/phad_gyro_align.cpp
tests/apps/gyro_alignment_test.cpp
tests/apps/gyro_alignment_cli_test.cpp
docs/research/m4-minimal-gyro-q3-offline-bias-alignment-result.md
Q3 research / design / plan / roadmap 的状态或结果更新
```

禁止新增 `apps/internal/**`，禁止扩 estimator public interface、修改 `phad/bench`、接入
`OfflineVoSession`/`StereoVoEstimator`、引入 GT/ATE/factor/posterior/Q4+，或新增 runtime tuning
knobs、fallback、第二套积分器及无实际必要的 README 修改。

## 11. Implementation go 条件

只有以下各项全部闭合，才可请求另行授权 Q3 implementation：

1. §3.2 exact schema/eligibility/error predicates；
2. §5 complete solver/Jacobian/rank/uncertainty/determinism contract；
3. §6 primary、12 项 equivalence、mandatory adjacent diagnostic components 的完整自身 null、
   S1 单一 overall alpha/critical law/df/SE/HAC validity、各自 support/power、严格 component
   boundary 与全部剩余 owner budgets；O1-DELTA=A 的 `primary.delta_loss_rad2=0`
   以及 O1E-ANCHOR=A、O1E-BASIS=A 已冻结，但不代表本项其他 blockers 已闭合；
4. §8.1 exact versioned artifact/manifest schemas，以及 machine-checkable `claim_scope`、
   `diagnostic-only`、`no-component-claim` 语义；
5. near-\(\pi\)、gap-run、nonconvergence、V1 component truth table 与 V2 aggregate precedence 的
   唯一映射；
6. canonical protocol serialization/hash；
7. 独立 Standards/Spec review 无 blocker；
8. issue/comment 明确授予 §10.3 allowlist。

在此之前，正确动作是保留 Q2 已验证能力、记录 **protocol amendment STOP**，不实现“先跑再
补门”。
