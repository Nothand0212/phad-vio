# M4 Q3 outcome-independent qualification budget 调研

日期：2026-08-13

状态：**O0=A、O1-DELTA=A（`primary.delta_loss_rad2=0`）、O1E-PATH=A、
O1E-ANCHOR=A、O1E-BASIS=A、O1E-REFERENCE=A、O1E-ANCHOR-CONTRACT-PATH=A 与
O1E-ANCHOR-CONTRACT-LOCATOR=A 已选择，
但 primary positive minimum meaningful effect `δ_MME` 仍为 `UNFROZEN`；当前状态为
`PATH_A_SELECTED / LOCATOR_A_SELECTED(EXTERNAL_AUTHORITY) / AWAITING_CANONICAL_LOCATOR`，
O1E-ANCHOR-CONTRACT pending / `UNFROZEN`，下一项仅为
O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR，O1E-ENDPOINT 必须等待 / 其余 owner 与 protocol
decisions pending / Q3 implementation STOP / Q3 plan 5/5 todos pending**。
本文没有打开或统计 Q1 runtime CSV/TUM/diag 内容，没有读取 Q3 held-out suffix、GT/ATE/RPE、
Q5 结果或 future DUT，也没有生成、执行或判定任何 Q3 outcome。

关联：issue [#39](https://github.com/Nothand0212/phad-vio/issues/39)、
[Q3 结构调研](m4-minimal-gyro-q3-offline-bias-alignment-research.md)、
[Q3 normative 设计](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)、
[O1E-ANCHOR-CONTRACT existing-record 审计](m4-minimal-gyro-q3-anchor-contract-record-audit.md)、
[Q3 计划](../plans/2026-08-13_m4_gyro_q3_align_0e897ba8.plan.md)。

文档角色：本文是 **evidence + owner decision record**，保留来源、数学推导、候选评估与
A/B/C 决策历史；不是 executable protocol authority。O0/O1-DELTA/O1E-ANCHOR/O1E-BASIS/
O1E-REFERENCE 与统计合同的 adopted normative 定义只在上述 Q3 design 规范节中维护。

## 1. Fixed point、权限与 leakage prohibition

issue #39 固定 structural Q3 plan 于 commit
`dbec0be89512f54c386319762838fe16296e7b1f`。本片只允许 protocol-amendment 的
research/design/plan；Q3 source/tests/CMake、执行、Q4+、factor-enabled experiment 与 posterior
变化均未授权。amendment 即使完成，也仍须另获明确 implementation go。

本次记录 O0=A、O1-DELTA=A（零损失容忍）、O1E-PATH=A、
O1E-ANCHOR=A、O1E-BASIS=A、O1E-REFERENCE=A、O1E-ANCHOR-CONTRACT-PATH=A 与
O1E-ANCHOR-CONTRACT-LOCATOR=A selected；
这些是互不可代替的稳定 textual decision labels，不是 JSON/schema/protocol fields。
O1E-ANCHOR-CONTRACT-PATH=A 只冻结 existing qualified record closure path；O0、O1-DELTA、O1E-ANCHOR 与
O1E-BASIS、O1E-REFERENCE 的
adopted normative contract 均只见
[Q3 design §6](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)。
后续三个稳定的 textual decision-record IDs 定义为：`O1E-MME`=primary positive `δ_MME`
evidence package + owner adoption decision，`O1E-ADJ`=adjacent-equivalence effect/margin，
`O1E-POWER`=target power。它们不是 JSON/schema/protocol fields，且全都保持 pending /
`UNFROZEN`。
O1E-PATH=A 只确定从 product/hazard/mission requirement 经完整可审计映射派生 positive
`δ_MME [rad²]` candidate 的工作流，不采用任何数值 protocol field。目标 population 上 true mean
paired squared-SO(3)-loss improvement `μ_d [rad²]` 是未知 estimand，只说明映射目标语义，不能由
owner 采用为设计阈值或替代 `δ_MME`；`O1E-MME`、`O1E-ADJ`、`O1E-POWER`、alpha、
mixed verdict precedence、schema、solver、Jacobian 和 support 也均未冻结。

O1E-BASIS=A 只冻结 downstream product endpoint 的 primitive comparison basis：zero-bias arm
与 fitted-bias arm 必须先各自相对**同一个** downstream product-reference role 评价
endpoint-native quality，paired benefit 只能随后由这两项 arm quality 派生。该 common-reference
关系必须两臂共享、arm-independent 且 pre-outcome；不得由任一 arm output、fitted bias、future
DUT/outcome 或 desired PASS 选择。它没有冻结任何具体 reference、metric、formula、threshold
或 Q3 mapping，也不改变 Q3 当前的 `d_k`、primary 或 adjacent estimand。

O1E-REFERENCE=A 只冻结该 common reference 的 bytes/source identity：本文为该已选 identity 定义的
stable normalization alias
`q1-final-visual-posterior-proxy-v1` 指向 Q1 final independent PASS run
`q1_observe_final_q1_final_verifier` 发布的 `est.tum` exact bytes，SHA-256 为
`18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321`。这是
immutable Q1 final-qualified visual-posterior reference proxy；它不冻结 endpoint semantic
object/record selection 或任何 metric/mapping，也不把既有 visual residual 代理升级为 product
endpoint metric。

`O1E-ANCHOR-CONTRACT` 是另一个 stable textual pending decision-record node，不是
JSON/schema/protocol field。它只负责 O1E-ANCHOR=A 已选中的 downstream 1 s
relative-orientation consistency / product orientation-quality requirement 的 exact record
identifier、version、owner 与 baseline provenance；实际可审计 tuple 尚未由 owner 提供并验证，
因此保持 pending / `UNFROZEN`。它不冻结 endpoint semantics、threshold、risk、metric、reference、
mapping 或任何 numeric gate，也不得与 O1E-REFERENCE=A 的 reference identity 合并。

用户于 2026-08-17 在上一提交后的下一问明确选择 budget §7.5 closure path A。稳定 textual
decision label `O1E-ANCHOR-CONTRACT-PATH=A` 记录该 user/owner 选择；它不是 schema field，
且只冻结“采用 existing qualified record 路径”。它不填写或批准 tuple，也不闭合
O1E-ANCHOR-CONTRACT。用户随后对 stable decision `O1E-ANCHOR-CONTRACT-LOCATOR` 选择 A；
必须原样记录为 `O1E-ANCHOR-CONTRACT-LOCATOR=A selected`，不得事后改名为
`AUTHORITY=A`。该选择只确定 canonical authority 的 location/class：非 repo/GitHub Issue 的
受控 product-requirement system；它不是 actual locator payload，也没有提供 permalink、record
ID 或 system instance/name。严格下游 textual node
`O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR` 因此仍 pending。当前状态是
`PATH_A_SELECTED / LOCATOR_A_SELECTED(EXTERNAL_AUTHORITY) / AWAITING_CANONICAL_LOCATOR`：
actual canonical locator、identifier、version、owner、baseline provenance、category locator、
authorization 与 adoption 均未提供、未验证。
[existing-record 审计](m4-minimal-gyro-q3-anchor-contract-record-audit.md)在可检索 repo/Git/Issue
范围内的三条独立审计均得到 `QUALIFIED existing record=0`；`NOT FOUND` 只描述该覆盖范围，
不构成“任何外部 record 都不存在”的全局证明。

本研究采用以下防泄漏边界：

- 禁止从 Q3 suffix outcome、GT/ATE/RPE、Q5、future DUT 或 desired PASS 反推 threshold；
- 既有 Q1 runtime artifacts 只能用于 identity/schema/provenance 合同；只有 owner 后续明确授权，
  才可用 prospective prefix-only 数据估 nuisance variance，而且 effect、margin 与 risk 仍须先冻结；
- 官方规格、成熟实现常数和历史表现只可提供量纲、候选模型或 sanity floor，不能自动成为产品门；
- 本文所有数值均来自公开一手规格的单位换算或显式标注的候选假设，不是 Q3 outcome。

### 1.1 O1E-REFERENCE=A 的 stable identity 与角色边界

用户于 2026-08-17 明确选择 A；`O1E-REFERENCE=A` 是本轮 user/owner
adopted textual decision record，其 adoption 对象是已选的 exact `est.tum` bytes/source
identity。`q1-final-visual-posterior-proxy-v1` 是本文为该已选 identity 定义的
stable normalization alias；它不是用户在原始 A/B/C 中显式采用的命名，也不是
[Q1 final ledger](m4-minimal-gyro-q1-observe-result.md) 的原生字段。`O1E-REFERENCE=A`
同样不是 Q1 ledger 原生字段；`source_run` 仍是从 ledger canonical output locator
规范化出的 run leaf。其余列出的 artifact、digests、source commit、
`git_tree_object`、`git_ls_tree_sha256`、meta、config 与 manifest provenance 才逐项摘录/引用
ledger；本节不重写 ledger，也不读取外部 runtime artifact：

```text
decision_record=O1E-REFERENCE=A
reference_alias=q1-final-visual-posterior-proxy-v1
artifact=est.tum
artifact_sha256=18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321
source_run=q1_observe_final_q1_final_verifier
source_commit=74270572cc1fcc2eac82559117efd0951c800ab9
git_tree_object=e4379bf44db8d1127450d674b60b2e451fbe0aef
git_ls_tree_sha256=bb5a7854e4a3a89a52c8c0332e965474b8b9b2f2d4cbe0b83bcd7224e35f94ec
meta_sha256=bbeaa21edaee34733f61b8ef093d45d5b8f4268fc4c0a36da261ee6664e9fab1
config_hash=402d1925
input_manifest_v2_sha256=aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5
```

stable identity 由 alias、exact bytes hash 与上述 source identity/provenance 组成。Q1 ledger
§1.2 的绝对 runtime path 只可作为 ledger locator，不属于 stable identity；ledger 明确判废的
input manifest v1 不是有效来源，不得引用。ledger 另证实 control 与该 run 的 `est.tum`、
`kf.tum`、`diag.csv` 三项 `cmp` 均 exit `0`；此事实不创建第二份 reference，也不推出任何
未由 ledger 记录的 size 或 row schema。

该 proxy 的规范角色是 two arms shared、arm-independent、pre-outcome、source-identity-only。
zero-bias arm 与 fitted-bias arm 只能读取同一份 hash-verified bytes；任何 arm、fitted bias、
future DUT/outcome、desired PASS 或 post-outcome analysis 均不得生成、替换、筛选、修改或重新
解释 reference。它不是 GT、truth、physical-orientation oracle、reference gyro bias、arm output，
也不是 independent product-qualification oracle。

Q3 prefix bias fit 使用由该冻结 visual posterior 派生的 visual residual，不构成 feedback loop；
但 `b_hat` 只能称为 **visual-posterior-aligned constant gyro bias nuisance estimate**。prefix fit、
half-fit 与 arm-quality comparison 都不得冒充 confirmatory held-out/product evidence；suffix 最多
支持相对该 proxy 的 temporal held-out consistency，shared-source error 与 visual systematic error
仍然存在。

O1E-REFERENCE=A 只冻结 reference bytes/source identity；O1E-ANCHOR-CONTRACT-PATH=A 只冻结
existing qualified record closure path。O1E-ANCHOR-CONTRACT、exact endpoint semantic
object/record selection、
timestamp parser/pairing、1 s product analytical window/block、frame/axis/direction/handedness、
metric/loss/formula/aggregation、population/domain、severity/risk、requirement threshold、mapping、
`μ_d`、positive `δ_MME`、rounding/strict boundary、O1E-MME adoption、O1E-ADJ、
O1E-POWER、全部 U/S/V/A、schema/solver/Jacobian/support、implementation go 与 outcome 全部
保持 `UNFROZEN` / pending。

## 2. EuRoC 与 ADIS16448 的一手事实

### 2.1 数据集传感器与采样

ETH ASL 的 [EuRoC 官方数据集页](https://projects.asl.ethz.ch/datasets/euroc-mav/)及 Burri 等人的
[数据集论文（DOI 10.1177/0278364915620033）](https://doi.org/10.1177/0278364915620033)
记录：visual-inertial unit 输出两路 global-shutter monochrome stereo images，`2 x 20 FPS`；IMU
angular rate 与 specific force 为 `200 Hz`。MH_01 随数据发布的官方 `imu0/sensor.yaml` 将其标为
`VI-Sensor IMU (ADIS16448)`、`rate_hz: 200`。因此本项目可把“型号 ADIS16448、dataset IMU
200 Hz、相机 20 Hz”当 source facts；不能据此假定厂商规格的测试 filter、量程、温度与 EuRoC
录制配置完全相同。

### 2.2 ADIS16448 Rev. H gyro 规格及换算

权威器件文档是 Analog Devices 的
[ADIS16448 Data Sheet Rev. H](https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16448.pdf)
（Rev. H，2019）。Table 1 的总条件是 `TA=25 C`、`VDD=3.3 V`、
angular rate `0 deg/s`、dynamic range `+/-1000 deg/s`、`+/-1 g`，除非该行另注。以下数值均是
datasheet 的 **typical**；带 `1 sigma` 的行是统计描述，不是 min/max guarantee。

| quantity | datasheet condition / statistic | official value | SI conversion | 合法含义 |
|---|---|---:|---:|---|
| Bias repeatability | `-40 C <= TA <= +85 C`，`1 sigma`；脚注列项为 temperature hysteresis、electronics drift、temperature cycling、10-year rate random walk、broadband noise | `0.5 deg/s` | `8.72665e-3 rad/s` | 长期/跨条件 offset repeatability 的 typical population scale，不是单次 100 s run 的 bias CI |
| In-run bias stability | `1 sigma`，`SMPL_PRD=0x0001` | `14.5 deg/h` | `7.02980e-5 rad/s` | Allan-variance 最低区附近的短期稳定性尺度；不是 constant-bias 真值界，也不是 1 s angle noise |
| Angular random walk (ARW) | `1 sigma`，`SMPL_PRD=0x0001` | `0.66 deg/sqrt(h)` | `1.91986e-4 rad/sqrt(s)` | white-angle-noise coefficient；在额外白噪声/积分模型下，1 s 单轴 angle sigma 候选为 `1.91986e-4 rad` |
| Rate noise density | `f=25 Hz`、`+/-1000 deg/s`、no filtering、rms | `0.0135 deg/s/sqrt(Hz)` | `2.35619e-4 rad/s/sqrt(Hz)` | 指定频点/量程/无滤波下的 rate spectral density；不能不经 bandwidth/filter 模型直接变成 residual variance |

换算只使用

\[
1\ \mathrm{deg}=\pi/180\ \mathrm{rad},\qquad
1\ \mathrm{h}=3600\ \mathrm{s},\qquad
\sqrt{1\ \mathrm{h}}=60\sqrt{\mathrm{s}}.
\]

例如 `14.5*(pi/180)/3600 = 7.029798376e-5 rad/s`（显示为 `7.02980e-5 rad/s`），
`0.66*(pi/180)/60 = 1.91986e-4 rad/sqrt(s)`。Bias repeatability、IRBS、ARW 与 rate
noise density 是四种不同统计对象，不能互换。ADI 的
[ADIS16448 evaluation guide](https://wiki.analog.com/resources/eval/user-guides/inertial-mems/imu/adis16448)
还说明 IRBS 是对连续时域样本平均后可达的最佳分辨率、对应 Allan-variance minimum；其示例
配置与 EuRoC 的 200 Hz 发布采样并不相同。

### 2.3 为什么规格不能直接成为 hard margin

1. `typical`、`1 sigma` 与 rms 都不是 guaranteed upper bound；乘 `2`/`3`、跨三轴合成或取最大轴
   都会新增 distribution、independence 与 risk assumptions。
2. Bias repeatability 描述跨环境/寿命条件，IRBS 描述 Allan-variance minimum，ARW/ND 描述随机
   噪声；Q3 则检验 visual-vs-gyro paired loss、bias fit uncertainty、half stability 与 residual
   structure，estimand 不同。
3. 从 rate density 到 angle/residual 必须先冻结 bandwidth、filter、sampling、integration、轴相关
   与 visual reference error；从单轴到三轴 squared norm 还必须冻结 covariance。
4. datasheet 不定义产品愿意接受的 false-go 风险、corrected arm 可损失多少、最小有意义收益或
   time/speed slope hazard。产品 hard margin 因而必须由 owner 选择并记录理由。

所以这些规格最多是 budget candidate、prior sanity floor 或 sensitivity-analysis input；未经完整
`source -> units -> assumption -> owner approval -> protocol field` 链，不得进入 verdict。

## 3. Equivalence、HAC 与 prospective sizing 的一手边界

### 3.1 TOST / equivalence

- Schuirmann 1987，*A Comparison of the Two One-Sided Tests Procedure and the Power Approach for
  Assessing the Equivalence of Average Bioavailability*，
  [DOI 10.1007/BF01068419](https://doi.org/10.1007/BF01068419)，提出 two one-sided tests；
- [ICH E9 Step 4（1998-02-05）](https://database.ich.org/sites/default/files/E9_Guideline.pdf)
  要求 equivalence margin 在 protocol 中预先规定并由实际可接受差异论证；未显著拒绝零差异
  不等于等价；EMA 的 [ICH E9 版本页](https://www.ema.europa.eu/en/ich-e9-statistical-principles-clinical-trials-scientific-guideline)
  记录欧盟采纳状态；
- [ICH M13A Step 4（2024-07-23）](https://database.ich.org/sites/default/files/ICH_M13A_Step4_Final_Guideline_2024_0723.pdf)、
  EMA [Bioequivalence Rev.1 版本页](https://www.ema.europa.eu/en/investigation-bioequivalence-scientific-guideline)
  与 FDA [Statistical Approaches to Establishing Bioequivalence, Final, May 2026](https://www.fda.gov/regulatory-information/search-fda-guidance-documents/statistical-approaches-establishing-bioequivalence)
  是权威监管用例。其 `80.00%-125.00%` 等药代领域 margin 不能移植为 VIO margin。

对事先冻结的 `(L,U)`，TOST 为

\[
H_0:\theta\le L\ \text{or}\ \theta\ge U,\qquad H_1:L<\theta<U.
\]

两个单侧检验都在 `alpha` 拒绝才通过。若 CI 与检验严格匹配且为 equal-tail interval，则等价于
`CI_(1-2alpha)` 严格包含于 `(L,U)`。经典 paired t-TOST 的
finite-sample 结果依赖 normal model、独立实验单位、正确 SE/df。把 HAC CI 装进 TOST 只是
决策壳，其有效性来自 HAC CI 的 coverage，而不是 Schuirmann 的小样本 t 推导。

[Berger 1982，*Multiparameter Hypothesis Testing and Acceptance Sampling*](https://doi.org/10.1080/00401706.1982.10487790)
与 [Berger & Hsu 1996，Theorem 1](https://doi.org/10.1214/ss/1032280304) 给出 intersection-union
test（IUT）的一手依据。精确地，仅当 sole overall confirmatory claim 定义为

\[
H_0=\bigcup_j H_{0j},\qquad H_1=\bigcap_j H_{1j},
\]

且 overall reject 当且仅当每个预注册 component 都 reject，同时每个 component test
\(\varphi_j\) 对完整 \(H_{0j}\) 都是 level \(\le\alpha\) 时，才有：对任意
\(\theta\in H_0\)，至少存在 \(j^*\) 使 \(\theta\in H_{0j^*}\)，因而

\[
P_\theta(\text{false overall reject})
=P_\theta\!\left(\bigcap_j\{\varphi_j=1\}\right)
\le P_\theta(\varphi_{j^*}=1)\le\alpha.
\]

因此在上述 IUT 前提下，overall reject 的 level 上界可由某个成立的 component null 直接
给出，不需要 component independence。这是 O0=A owner 选择的数学依据，不在本 research
中定义项目的可执行 claim 或 alpha 操作；采用后的完整规范语义只见
[Q3 design §6](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)。

### 3.2 Newey-West / Bartlett

Newey 与 West 1987 的
[*A Simple, Positive Semi-definite, Heteroskedasticity and Autocorrelation Consistent Covariance Matrix*](https://doi.org/10.2307/1913610)
给出 positive semi-definite HAC covariance；Bartlett 1950 的
[*Periodogram Analysis and Continuous Spectra*](https://doi.org/10.1093/biomet/37.1-2.1)
是三角/Bartlett window 的原始一手来源。固定 lag `L` 时，候选权重为

\[
w_h=1-\frac{h}{L+1},\quad h=0,\ldots,L.
\]

这不替项目决定 `L`、normalization、small-sample correction、Normal/t critical law、回归 sandwich、
negative variance、短序列或 gaps。若 block index 是 calendar `k`，缺失 `k` 不能压紧成相邻；
只对两端实际存在的 `(k,k+h)` 累加，lag 仍是 `h`。Newey-West 是大样本近似，少量 blocks 下
不能声称 finite-sample calibrated。

### 3.3 Prospective precision / power

对称 TOST margin `[-Delta,Delta]` 的必要 precision 条件为

\[
|\hat\theta|+q_{1-\alpha}SE<\Delta.
\]

在 known-variance Normal、工作真值为 0、`SE=sigma/sqrt(n)` 的简化条件下，一个尚未由
owner 采用的首轮 sizing 候选近似为

\[
Power_{candidate}\approx\max\!\left\{0,
2\Phi\!\left(\frac{\Delta\sqrt n}{\sigma}-z_{1-\alpha}\right)-1\right\}.
\]

当 \(\Delta\sqrt n/\sigma\le z_{1-\alpha}\) 时该候选概率为 0，而不是负值。它只可作
outcome-independent 首轮 sizing，不能冒充已冻结协议。HAC、unknown variance、gaps、slope design
或 small sample 必须用与最终 estimator/covariance/calendar rule 相同的 prospective
calculation/simulation。不得用 suffix variance 或已知 suffix capacity 下调 `n_required`；若最终
prospective calculation 判定预注册的 `O1E-POWER` target power 在预算内不可达，或候选设计
power 低于该目标，
应按预注册规则映射为 `INCONCLUSIVE`，不能通过改 margin 取得通过。

## 4. Budget traceability chain

每个 verdict-changing 数值都必须有一行完整链；任一环缺失，protocol field 保持 `UNFROZEN`。

| source | source units | protocol units | explicit assumption | owner approval required | candidate protocol field / current state |
|---|---|---|---|---|---|
| 产品风险：corrected arm 不得更差 | product policy | `rad^2` | 零损失容忍；exact sign/gate/equality 语义只见 [Q3 design §6.1](m4-minimal-gyro-q3-offline-bias-alignment-design.md#61-primary-diagnostic-componenthac) | owner 已选 A | `primary.delta_loss_rad2=0` / **selected；adopted into design** |
| ADIS16448 ARW `0.66 deg/sqrt(h)` | angle random walk | `rad/sqrt(s)`，再到 1 s `rad` | 若用于 stochastic model，须先冻结 white/stationary、dataset 配置适用性、轴相关与传播模型；目前 pending | 是否只作 sanity floor，或进入 stochastic model | `sensor.arw_rad_sqrt_s=1.91986e-4` / source fact；用途 pending |
| ADIS16448 IRBS `14.5 deg/h` | bias stability | `rad/s` | 若用于 budget，须先冻结 Allan-minimum typical 1 sigma 对目标 run 的适用性、轴相关与倍数；目前 pending | 接受 typical/model risk | `bias.full_uncertainty_budget_radps[3]`、`half.margin_radps[3]` / pending |
| ADIS16448 bias repeatability `0.5 deg/s` | long-term offset | `rad/s` | 若用于 sensitivity，须先论证跨环境 population scale 与本次 100 s estimand 的关系；目前 pending | 通常应拒绝作 Q3 tight hard gate | sensitivity-only / pending |
| ADIS16448 rate ND `0.0135 deg/s/sqrt(Hz)` | rate spectral density | residual `rad` 或 loss `rad^2` | 若用于 residual/primary model，须先冻结 bandwidth、filter、sampling、integration、axes covariance 与 visual error；目前 pending | 批准完整 noise propagation model | residual/primary design input / pending |
| 产品容许 constant-bias mismatch | hazard/product units | `rad/s` | 若用于 budget，须先冻结 full/early/late estimator 的共同 frame 与 axis 定义；目前 pending | 逐轴预算及风险等级 | full-fit uncertainty、half-difference margins / pending |
| 产品容许 held-out orientation structure | hazard/product units | mean `rad`、time slope `rad/s`、speed slope `s` | 若采用 joint regression，须先冻结 time/speed predictor、centering、scaling 及共同 frame/axis；目前 pending | 12 margins 与 design alternatives | equivalence margins / pending |
| downstream 1 s relative-orientation consistency / product orientation-quality requirement；common reference alias `q1-final-visual-posterior-proxy-v1` | endpoint-native product units；requirement record identity 待 O1E-ANCHOR-CONTRACT 闭合；reference identity 为 Q1 final-qualified `est.tum` exact bytes，SHA-256 `18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321` | 两臂先各自相对同一 immutable visual-posterior proxy 得到 endpoint-native arm quality，paired benefit 只能随后派生；再经尚未冻结的 mapping 派生 candidate `δ_MME [rad²]`；`μ_d [rad²]` 仍是未知 estimand | evidence chain 分为两条严格前置分支：O1E-PATH=A → O1E-ANCHOR=A → O1E-ANCHOR-CONTRACT-PATH=A → O1E-ANCHOR-CONTRACT-LOCATOR=A → O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR (pending) → TUPLE_VERIFICATION → OWNER_AUTHORIZATION → O1E-ANCHOR-CONTRACT closed → O1E-ENDPOINT；O1E-PATH=A → O1E-BASIS=A → O1E-REFERENCE=A。actual canonical locator 未取得前不得进入 TUPLE_VERIFICATION。O1E-ENDPOINT 还必须等待 O1E-BASIS=A、O1E-REFERENCE=A 汇合；随后才可进入 metric/loss/formula → 1 s product analytical window/block + timestamp pairing → frame/axis/direction/handedness → population/domain → severity/risk → requirement threshold → mapping → derived candidate positive `δ_MME` → rounding → strict boundary → O1E-MME evidence package → independent owner adoption。上述新增 process nodes 都是 textual labels，不是 schema fields，也不冻结 endpoint semantics；reference 只冻结 bytes/source identity，且必须 shared、arm-independent、pre-outcome、source-identity-only；两条合同不得合并 | 用户于 2026-08-17 已选择 O1E-PATH=A、O1E-ANCHOR=A、O1E-BASIS=A、O1E-REFERENCE=A、O1E-ANCHOR-CONTRACT-PATH=A 与 O1E-ANCHOR-CONTRACT-LOCATOR=A；后者只选择 external authority 的 location/class，不是 actual locator payload。当前为 `PATH_A_SELECTED / LOCATOR_A_SELECTED(EXTERNAL_AUTHORITY) / AWAITING_CANONICAL_LOCATOR`，actual canonical locator、identifier、version、owner、baseline provenance、category locator、authorization 与 adoption 均未提供、未验证，因此下一项仅为 O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR；其后必须先完成 tuple verification，验证通过后才允许 owner authorization，O1E-ENDPOINT 必须等待。可检索 repo/Git/Issue 范围的三条独立审计均为 `QUALIFIED existing record=0`；reference 不是 GT/truth/physical-orientation oracle/reference gyro bias/arm output/independent product-qualification oracle；其余链条仍待逐项批准 | primary positive minimum meaningful effect `δ_MME` / **path A selected；locator A selected (external authority class only)；anchor contract pending / `UNFROZEN`；awaiting canonical locator；reference identity selected；tuple unverified；endpoint waiting；`δ_MME` 仍 `UNFROZEN`；无候选 protocol 数值；`O1E-MME` 未进入 adoption decision** |
| 产品最小有意义改善 | product value | primary `rad^2`；adjacent rate-loss `rad^2/s^2` | 只有 `O1E-MME` evidence package 完成且 owner 随后明确 adopt positive `δ_MME`，`O1E-MME` 才 closed/resolved；此后才可进入 `O1E-ADJ`，完成 `O1E-ADJ` 后才可进入 `O1E-POWER`。若用于 prospective sizing，还须冻结 block/adjacent estimand、duration normalization 与实验单位 | `O1E-MME`、`O1E-ADJ` 与 `O1E-POWER` 分阶段批准 | prospective support/power / pending |
| Schuirmann/ICH/FDA | dimensionless alpha/CI | matching estimator units | equivalence margin 须事先给定，且 inference coverage 依赖所选 SE/df/HAC model | owner 仍须批准 margins 与 S1 risk choices | O0 已决；adopted executable 语义见 [Q3 design §6](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)；其余 pending |
| Newey-West/Bartlett | indexed covariance | estimator-specific squared units | 若采用 HAC，须先冻结 calendar lag、kernel、bandwidth、normalization、correction 与 critical law；目前 pending | 接受 asymptotic risk与 sensitivity rule | `hac.*` / pending |

除 O0 与 O1-DELTA=A 的既有 selected/adopted decision history 外，O1E-PATH=A 只具有
evidence-routing status；O1E-ANCHOR=A 只赋予所选 product endpoint meaningful-effect
evidence authority；O1E-BASIS=A 只选定 endpoint evidence 的 primitive comparison basis；
O1E-REFERENCE=A 只冻结 common reference bytes/source identity；O1E-ANCHOR-CONTRACT 仍只是一项
pending / `UNFROZEN` 的 requirement-record identity 合同。
O1E-ANCHOR-CONTRACT-PATH=A 只冻结 existing qualified record 的 closure path；它不改变该 pending
状态。O1E-ANCHOR-CONTRACT-LOCATOR=A 只冻结 external authority 的 location/class；它不是 actual
locator payload，也不能由本次自动补造 actual canonical locator、tuple、category locator、owner
authorization 或 adoption。
Q3 design 仍是唯一 executable protocol authority。表中的 effect formula、model 与 estimand
均须等待 owner 逐项选择并补齐前提。strict boundary 前的 evidence package 即使完成，也不
等于 `O1E-MME` adoption；只有 owner 随后明确 adopt positive `δ_MME`，`O1E-MME` 才
closed/resolved。在此之前它们都不是 protocol fact，不得写入 executable verdict 合同。O0 与
O1-DELTA=A 的可执行语义仍只见 Q3 design。

禁止来源：`condition<=1e6`、固定 60 blocks、未枚举 22-Holm、Mahalanobis non-rejection、旧
adjacent/RMSE hints、无 command/log/hash 的 census、suffix/GT/ATE/Q5、成熟项目 online constants。
尤其 Q5 `0.001 rad` guardrail 不能跨量纲变成 `rad/s`、slope 或 `rad^2`。

### 4.1 Fixed-point 旧 alpha tuple 退役账本

本表是唯一退役 ledger，只用于证明 fixed point 中的旧值已不生效；不得将任一行恢复为
procedure、候选 alpha 或资格证据。

| fixed-point 旧项 | 退役状态 | qualification 语义 |
|---|---|---|
| overall `0.05` | superseded / non-normative | qualifying weight `0` |
| primary `0.025` | superseded / non-normative | qualifying weight `0` |
| `alpha_E=0.025` | superseded / non-normative | qualifying weight `0` |
| `alpha_j=alpha_E/12` | superseded / non-normative | qualifying weight `0` |
| adjacent separate allocation | superseded / non-normative | qualifying weight `0` |

新的 overall alpha 数值仍由 S1 pending decision 冻结；本 ledger 不提供任何数值建议。

## 5. 对抗审查 F1-F9 结论

| ID | severity | 分类 | review conclusion / recommendation | current status |
|---|---|---|---|---|
| F1 | BLOCKER | 事实 + owner 待决 | 所有 budget 必须走上述 traceability chain；官方 typical 规格不能替 owner 定产品风险。 | traceability 与禁止替 owner 决策的原则已记录；具体 budget 仍 pending。 |
| F2 | BLOCKER | 事实 + 已决 owner scope + 协议待决 | Berger 1982 与 Berger & Hsu 1996 Theorem 1 支持了 O0=A 的 IUT 选择，但不替代项目冻结 alpha 数值与 estimator validity。 | O0 owner decision 已记录；可执行 claim/level/PASS 语义只见 [Q3 design §6](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)，S1 仍 pending。 |
| F3 | BLOCKER | 事实 + 审查建议 + owner/协议待决 | 逐 component 先判 prerequisite/support：不足是 `INCONCLUSIVE`，充分但科学边界未达是 `HYPOTHESIS_FAIL`；mixed aggregate precedence 存在待决 alternatives，不能从 component 状态语义推断。 | V2 pending，具体 A/B/C 只见本文 §8；normative design 只维护 component-level 状态语义与 V2 pending/link，O0 不冻结 aggregate precedence。 |
| F4 | BLOCKER | 审查建议 + owner/协议待决 | Recommendation：四数据输入与可信 provenance 分两层；trusted outer launcher/verifier 固定 analyzer/build attestation、embedded protocol bytes 与 Q1/Q2 ledger；manifest 由 out-of-band expected digest 验证，不能只自报 hash。 | O4/A1 pending；provenance architecture 与 artifact trust anchor 尚未冻结。 |
| F5 | HIGH | 审查建议 + 协议待决 | Recommendation：schema/integrity 先 hard validate；eligible packet 必须 exact valid、无 gap、样本/端点/segment/diag joins 全成立；拒帧无 pose 是 local exclusion，缺重 diag、unknown enum、pose-with-non-ok 是 hard error；`t0` 为 canonical order 首个 eligible interval。 | U1 pending；schema、eligibility、error table 与 calendar identity 尚未冻结。 |
| F6 | HIGH | 事实 + 审查建议 + 协议待决 | adjacent raw `d_p` 受 duration square 影响；recommendation 为块内 `q_p=d_p/dt_p^2`，再以 `A_k=sum(dt_p*q_p)/sum(dt_p)` 得完整 1 s block 的 time-average rate-loss；与 primary 共用 `k`，不加 support、不声称独立；missing `k` 不压紧。 | U2 pending；adjacent estimand、duration normalization 与 calendar index 尚未冻结。 |
| F7 | HIGH | 审查建议 + 协议待决 | Recommendation：objective 为 unweighted SO(3) residual sum、zero init、无 robust/fallback/clamp；authoritative Jacobian 用同一 Q2 helper 的 per-axis central FD + step-halving；covariance 用 calendar-block score HAC sandwich；half difference 用 stacked influence covariance，不能假定 halves independent。 | U3 pending；上述 objective、Jacobian、estimator 与 covariance 选择均尚未冻结。 |
| F8 | HIGH | 审查建议 + owner/协议待决 | Recommendation：三轴 validation 用同一 joint regression `r=beta0+beta_t*x_t+beta_s*x_s+eps`；time midpoint 以秒中心化，speed 用 visual block angular speed 后中心化；half difference 不冒充 block observation。 | O3 pending，12 margins、design alternatives、joint regression 与 predictor 定义尚未冻结；prefix-only nuisance 仍须 owner 另行授权。 |
| F9 | HIGH | 审查建议 + 协议待决 | Recommendation：JSON/CSV/manifest 各有 schema id/version、exact types/units/enums/canonical bytes；CSV 以 `record_type={adjacent,block,exclusion}` 构成 discriminated union 与稳定 key；manifest-last 且不自哈希；unknown/nonfinite/duplicate/early manifest 必须拒绝。 | A1 pending；canonical schemas、artifact encoding 与 external trust anchor 尚未冻结。 |

## 6. 无循环 decision DAG

本 DAG 只定义待决问题的依赖与提问顺序，是 evidence + owner decision record 的工作流，
**不是 executable protocol authority**。任何 adopted normative contract 仍只写入
[Q3 design](m4-minimal-gyro-q3-offline-bias-alignment-design.md)；下列 candidate 职责在 owner
选择并单向写入 design 前均不具规范性。

1. `U0` 固定上游事实：Q1 writer/validator、timestamp parser、Q2 helper、EuRoC/ADI source facts。
2. `U0 -> U1/U2/U3`：分别冻结 exact input/eligibility；block+adjacent estimands/calendar index；
   objective/FD Jacobian/rank/HAC sandwich estimator。
3. `O0=A` 与 O1-DELTA=A（`primary.delta_loss_rad2=0`）是 resolved inputs；
   当前 evidence chain 分为两条严格前置分支：`O1E-PATH=A -> O1E-ANCHOR=A ->
   O1E-ANCHOR-CONTRACT-PATH=A -> O1E-ANCHOR-CONTRACT-LOCATOR=A ->
   O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR (pending) -> TUPLE_VERIFICATION ->
   OWNER_AUTHORIZATION -> O1E-ANCHOR-CONTRACT closed ->
   O1E-ENDPOINT`；
   `O1E-PATH=A -> O1E-BASIS=A -> O1E-REFERENCE=A`。只有前一分支达到
   `O1E-ANCHOR-CONTRACT closed`，并与 `O1E-BASIS=A`、`O1E-REFERENCE=A` 汇合后，
   `O1E-ENDPOINT` 才可继续；此后严格为
   `metric/loss/formula -> 1 s product analytical window/block + timestamp pairing ->
   frame/axis/direction/handedness -> population/domain -> severity/risk -> requirement threshold ->
   mapping -> positive `δ_MME` candidate -> rounding -> strict boundary -> O1E-MME evidence package ->
   independent owner adoption`。其中 O1E-PATH=A 是已选择的 evidence-source/workflow path；
   O1E-ANCHOR=A 将 downstream 1 s relative-orientation consistency / product orientation-quality
   requirement 选为唯一上位 normative anchor；O1E-ANCHOR-CONTRACT-PATH=A 只冻结采用 existing
   qualified record 路径；O1E-ANCHOR-CONTRACT-LOCATOR=A 只选择 canonical authority 的
   external-system location/class，不是 actual locator payload；O1E-ANCHOR-CONTRACT 只负责该已选
   requirement 的 exact record identifier/version/owner/baseline provenance，当前
   `PATH_A_SELECTED / LOCATOR_A_SELECTED(EXTERNAL_AUTHORITY) / AWAITING_CANONICAL_LOCATOR`、
   pending / `UNFROZEN`，actual canonical locator、tuple、category locator、authorization 与
   adoption 未提供、未验证；
   O1E-BASIS=A 将 arm-wise common-reference quality 选为 primitive comparison basis；
   O1E-REFERENCE=A 将 owner 已选的 exact `est.tum` bytes/source identity 选为唯一 common
   reference；`q1-final-visual-posterior-proxy-v1` 是本文对该 identity 的 stable normalization
   alias。上述 decision/process nodes 都是 textual labels，不是 JSON/schema/protocol
   fields，也不冻结 endpoint semantics。actual canonical locator 未取得前不得进入
   `TUPLE_VERIFICATION`；`OWNER_AUTHORIZATION` 必须以通过 `TUPLE_VERIFICATION` 为前置，
   不得先行或与 verification 合并。两臂必须先各自相对同一
   arm-independent、pre-outcome product-reference role 评价 endpoint-native quality，paired
   benefit 只能随后派生；reference 不得由任一 arm output、fitted bias、future DUT/outcome、
   desired PASS 或 post-outcome analysis 生成、替换、筛选、修改或重新解释。reference identity
   已冻结；但 O1E-ANCHOR-CONTRACT 未闭合前不得进入 O1E-ENDPOINT。只有 actual canonical
   locator 已提供，actual tuple 与 category 归属先通过 verification，此后 owner authorization
   也完成，合同才具备 future closed 条件；
   此后才依次冻结 exact
   endpoint semantics、metric/loss/formula、
   1 s analytical window/block、time pairing、axis/frame/direction/handedness、target population/
   operational domain、severity/risk class、product requirement threshold、mapping assumptions +
   formula → derived
   candidate positive `δ_MME [rad²]`（同时明确 `μ_d` 是未知 estimand，只表达映射目标语义）→
   rounding rule → strict boundary。O1E-REFERENCE=A 是 source-identity-only，不是 methodology
   PASS。Q3 既有 `ΔRvis`、`r0`、`rb`、`d_k` 解析代理、primary 与 adjacent estimand 不变；只有
   endpoint semantics、metric 与 mapping 后续另行冻结后，两项 product endpoint arm quality 才可单向映射
   到 Q3 paired estimand，绝不能由 `d_k`、`μ_d` 或 outcome 反向定义 reference 或 threshold。
   到 strict boundary 为止只形成 `O1E-MME` adoption-ready evidence
   package；candidate 出现前的前置未完成时不得派生 candidate，strict boundary 未完成时不得
   请求 `O1E-MME` adoption，且不得把 `μ_d` 当成 owner 可采用的设计阈值。随后 owner 才作独立
   `O1E-MME` adoption decision；只有 owner 明确 adopt 该 positive `δ_MME`，`O1E-MME` 才
   closed/resolved，`δ_MME` 才从 `UNFROZEN` 变为 frozen。未明确 adopt 时，`O1E-MME` 保持
   open / unresolved / pending / `UNFROZEN`。
4. 只有 owner 明确 adopt positive `δ_MME`、使 `O1E-MME` resolved 后，才进入
   `O1E-ADJ`；只有 `O1E-ADJ` 完成后，才进入 `O1E-POWER`。三者当前均保持 pending /
   `UNFROZEN`。
   再随后依次冻结 `O2` 三轴
   uncertainty/stability budgets、`O3` 与 O0=A
   一致的 12 margins 与 design alternatives、`O4` provenance architecture。
5. `(U1,U2,U3,O1E-MME,O1E-ADJ,O1E-POWER,O2,O3) -> S1`：冻结
   alpha/critical law/TOST/HAC/gap rules。
6. 只有 owner 明确授权 prefix-only nuisance 后，`S1 -> S2` prospective
   support/power/infeasible rule。
7. `S2 -> V1 -> V2`：先冻结逐 gate boundary truth table，再由独立 owner 决策冻结 overall
   mixed-verdict precedence。
8. `(U1,V2,O4) -> A1`：canonical JSON/CSV/manifest 与 external trust anchor；随后
   `A1 -> R1` 独立 Standards/Spec review。
9. `R1` 通过后 amendment 才可称 `COMPLETE`；amendment `COMPLETE` 仍是 implementation
   `STOP`，必须另获 separate implementation go。

## 7. Owner decision record

### 7.1 O0：product claim 与 risk semantics

决策来源：用户于 2026-08-14 在 O0 对齐后回复“继续”；按项目 Learned User Preferences 的
“默认先落地推荐项”规则，冻结推荐 A。该回复只冻结 O0 claim scope，不冻结 alpha、margin、
solver、schema、support、verdict precedence 或任何后续 owner/protocol decision。

O0 的 A/B/C 处置如下：

- **A（selected / 已冻结）**：接受 sole overall conjunctive claim scope。这是 owner decision
  record；完整 PASS、component 和 level 语义只见
  [Q3 design §6](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)。
- **B（not selected / non-normative / qualifying weight 0）**：带可单独发布的 component
  confirmatory claims。
- **C（not selected / non-normative / qualifying weight 0）**：sensitivity/protocol-development
  only。

### 7.2 O1-DELTA：`delta_loss`

决策来源：用户于 2026-08-14 选择 Q3 amendment 中 O1-DELTA=A，即
`primary.delta_loss_rad2=0`。该回复只冻结零损失容忍，
不冻结 `O1E-MME`、`O1E-ADJ`、`O1E-POWER`、alpha、
solver、schema、support 或 outcome。exact field/sign/unit/strict-boundary 与 component/overall 语义只见
[Q3 design §6.1](m4-minimal-gyro-q3-offline-bias-alignment-design.md#61-primary-diagnostic-componenthac)；
本 research 不持有平行的 normative gate 或可执行 test oracle。

O1-DELTA 的 A/B/C 处置如下：

- **A（selected / adopted into design）**：`primary.delta_loss_rad2=0`。owner 选择的产品含义是
  corrected arm 的 held-out mean squared loss 不得变差。ADI 规格只作 uncertainty/sanity
  floor，不冒充 guarantee。
- **B（not selected / non-normative / no qualification authority）**：曾候选允许 corrected 最多坏
  `delta_allow=1.25e-7 rad^2`，其历史对应候选字段为 `delta_loss=-1.25e-7 rad^2`。这个数目前仍不可用于
  executable field：其原候选算法把 IRBS `7.02980e-5 rad/s` 静默当作 1 s angle stochastic term，
  但缺少随机过程与传播合同；还额外假设 ARW 与 IRBS 可独立合成、三轴同分布且 dataset 配置
  适用。在完整 noise process、integration/filter、axis covariance 与 visual error 模型冻结前，
  不得将该候选写入 protocol。
- **C（not selected / non-normative / no qualification authority）**：owner 另行提供非零
  application loss budget `[rad^2]` 及 hazard/risk 来源；不得引用 Q5、suffix、GT/ATE 或
  desired PASS。

`primary.delta_loss_rad2` 已选定不代表 protocol complete。O1E-ANCHOR-CONTRACT-PATH=A 已选择；
O1E-ANCHOR-CONTRACT-LOCATOR=A 也已选择且只确定 external authority location/class。下一 owner
decision 只能是 `O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR`，O1E-ENDPOINT 必须等待该合同闭合。
`O1E-MME` 必须在
anchor contract → endpoint → metric/pairing/frame/population/risk/threshold/mapping → positive
`δ_MME` 的完整 evidence chain 闭合并由 owner 独立 adopt 之前保持 pending /
`UNFROZEN`。`O1E-ADJ` 与 `O1E-POWER` 继续作为随后 pending / `UNFROZEN` 事项，本文不代选
任何数值。

### 7.3 O1E-PATH：primary positive minimum meaningful effect evidence path

决策来源：用户于 2026-08-17 选择 **O1E-PATH=A**。该 decision label 与既有、
且唯一指向 `primary.delta_loss_rad2=0` 的 O1-DELTA=A 分开；本次选择只确定 evidence
source 与决策工作流，
不冻结 `δ_MME`，不采用任何数值 protocol field，也不推进 `O1E-MME`、`O1E-ADJ`、
`O1E-POWER`、alpha 或 implementation 权限。三者与 `δ_MME` 继续 pending / `UNFROZEN`。

O1E-PATH 的 A/B/C 处置如下：

- **A（selected evidence-source/workflow path）**：按以下完整 DAG 派生并审查：
  `O1E-PATH=A -> O1E-ANCHOR=A -> O1E-ANCHOR-CONTRACT-PATH=A ->
  O1E-ANCHOR-CONTRACT-LOCATOR=A -> O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR (pending) ->
  TUPLE_VERIFICATION -> OWNER_AUTHORIZATION ->
  O1E-ANCHOR-CONTRACT closed -> O1E-ENDPOINT`；
  `O1E-PATH=A -> O1E-BASIS=A -> O1E-REFERENCE=A`；
  只有 `O1E-ANCHOR-CONTRACT closed` 才能与 `O1E-BASIS=A`、`O1E-REFERENCE=A` 汇合进入
  `O1E-ENDPOINT ->
  metric/loss/formula -> 1 s product analytical window/block + timestamp pairing ->
  frame/axis/direction/handedness -> population/domain -> severity/risk -> requirement threshold -> mapping ->
  derived candidate positive δ_MME -> rounding -> strict boundary -> O1E-MME evidence package -> independent
  owner adoption`。目标 population 上 true mean paired squared-SO(3)-loss
  improvement `μ_d [rad²]` 是未知 estimand，只用于明确映射目标语义；它不是 owner 可采用的
  设计阈值，也不能替代 `δ_MME`。完成 strict boundary 只得到 `O1E-MME` adoption-ready evidence
  package，不能预先闭合 `O1E-MME`；此时 owner 才能另作独立 `O1E-MME` adoption
  decision。只有 owner 明确 adopt 该 positive `δ_MME`，`O1E-MME` 才 closed/resolved，
  `δ_MME` 才冻结；未明确 adopt 时，`O1E-MME` 与 `δ_MME` 仍为 pending / `UNFROZEN`，
  且本条本身没有 qualification authority。
  actual canonical locator 未取得前不得进入 `TUPLE_VERIFICATION`；`OWNER_AUTHORIZATION` 只有在
  `TUPLE_VERIFICATION` 通过后才能进入，不得先行或合并。其中新增的 contract
  path/locator/canonical-locator/verification/authorization process nodes 都是 textual labels，
  不是 schema fields，也不冻结 endpoint semantics。
- **B（not selected / non-normative / no qualification authority）**：owner 直接给出 exact
  `δ_MME [rad²]` policy，不补 product/hazard/mission endpoint 到目标 estimand 的映射链。
- **C（not selected / non-normative / no qualification authority）**：以 sensor-floor proxy
  代替 product/hazard/mission requirement。§2 的 illustrative sensor source facts 仍只保留其
  原有 evidence/sanity 含义，不成为 effect candidate 或 protocol 值。

### 7.4 O1E-ANCHOR：primary positive minimum meaningful effect endpoint anchor

决策来源：用户于 2026-08-17 选择 **O1E-ANCHOR=A**，即将 downstream
1 s relative-orientation consistency / product orientation-quality requirement 作为 positive
`δ_MME` evidence chain 的唯一上位 normative anchor。该 decision 与 O1E-PATH=A
互不可代替，两者均不是 protocol schema field。exact product requirement
identifier/version/owner/baseline provenance 尚未提供，现由独立 pending node
O1E-ANCHOR-CONTRACT 管理；common reference identity 已由 O1E-REFERENCE=A 独立冻结，不得用
requirement contract 替代或扩张其角色。

O1E-ANCHOR 的 A/B/C 处置如下：

- **A（selected）**：downstream 1 s relative-orientation consistency / product
  orientation-quality requirement。它持有 meaningful-effect evidence authority；Q3 design 仍持有
  唯一 executable protocol authority。`1 s` 只是所选 product endpoint 的 identity/horizon，
  不自动冻结 Q3 analytical aggregation/window/block/axis/frame/reference semantics。
- **B（not selected as co-equal anchor）**：existing hazard-analysis orientation endpoint。本轮仅
  未将它选为 co-equal normative anchor，不作 hazard 有效性或风险结论，也不构成全局否决。
- **C（not selected as co-equal anchor）**：existing mission-success orientation endpoint。本轮仅
  未将它选为 co-equal normative anchor，不作 mission 成功语义或风险结论，也不构成全局否决。

该选择未冻结 exact endpoint semantics，也未采用 threshold、population/domain、
aggregation/window/axis、severity/risk、mapping/formula、candidate、rounding、strict boundary
或 `O1E-MME` adoption。`O1E-MME` 与 `δ_MME` 仍为 pending / `UNFROZEN`。

### 7.5 O1E-ANCHOR-CONTRACT：selected requirement 的 record contract

`O1E-ANCHOR-CONTRACT` 是 stable textual pending decision-record node，不是 schema field。它只负责
O1E-ANCHOR=A 已选中的 downstream 1 s relative-orientation consistency / product
orientation-quality requirement 的 exact record identifier、version、owner 与 baseline provenance；
不负责且不冻结 endpoint semantics、threshold、risk、metric、reference、mapping 或任何 numeric
gate。用户在上一提交后的下一问明确选择 A；稳定 textual decision label
`O1E-ANCHOR-CONTRACT-PATH=A` 是 user/owner selected，且不是 schema field。它只冻结“采用
existing qualified record 路径”，不填写、暗示或批准任何实际 tuple，也不冻结 endpoint
semantics。

原 A/B/C closure-path 历史选择表保留如下：

- **A（selected）**：owner 提供并授权采用该已选 1 s requirement 的真实、可审计 existing
  record，以及其 exact identifier/version/owner/baseline provenance。该选择不可由本次自动补造
  tuple；actual canonical locator、tuple、category locator、authorization 与 adoption 提供并验证
  前，O1E-ANCHOR-CONTRACT 仍为 pending / `UNFROZEN`。
- **B（not selected for current path / non-normative）**：owner 明确没有可提供的 qualified
  existing record，先为同一已选 requirement category 建立并批准新的 versioned requirement
  baseline。只有撤回当前 A path 后才可返回此路径；新 baseline 获批准前保持 STOP。
- **C（not selected for current path / non-normative）**：owner 不提供也不建立 requirement
  record；O1E-ANCHOR-CONTRACT 与 O1E-ENDPOINT 保持 pending / `UNFROZEN`，Q3 protocol
  amendment 保持 STOP。

用户随后对 stable decision `O1E-ANCHOR-CONTRACT-LOCATOR` 选择 A。该 label 必须原样记录为
`O1E-ANCHOR-CONTRACT-LOCATOR=A selected`，不得事后重命名为 `AUTHORITY=A`。它只选择
canonical authority 的 location/class：非 repo/GitHub Issue 的受控 product-requirement
system；不是 actual locator payload，不能推出实际 permalink、record ID、system instance/name
或任何 tuple 字段。

原 locator A/B/C 现作为历史决策记录：

- **A（selected）**：canonical authority 位于非 repo/GitHub Issue 的受控
  product-requirement system。
- **B（not selected for this decision）**：canonical authority 本身位于 repo 或 GitHub Issue。
- **C（not selected for this decision）**：确认没有 qualified existing record 并切换到新 baseline
  路线。

B/C 只是在本次 locator location/class decision 中未选择，不永久否决未来在有明确 owner 决策时
变更路径。当前状态明确为
`PATH_A_SELECTED / LOCATOR_A_SELECTED(EXTERNAL_AUTHORITY) / AWAITING_CANONICAL_LOCATOR`。
actual canonical locator、identifier、version、owner、baseline provenance、category locator、
authorization 与 adoption 均未提供、未验证；不能从 external system class 推定其中任何一项。
O1E-ANCHOR-CONTRACT 继续 pending / `UNFROZEN`，O1E-ENDPOINT waiting，Q3 implementation
STOP，Q3 plan 5/5 todos pending。
[existing-record 审计](m4-minimal-gyro-q3-anchor-contract-record-audit.md)覆盖可检索 repo 文档、
Git history/refs/tags/notes 与 Issue #39 及其可定位的一方 issue provenance；三条独立审计均为
`QUALIFIED existing record=0`，Issue #39 没有 tuple/comments。`NOT FOUND` 不证明不存在任何
外部 record，只说明上述覆盖范围没有发现 qualified existing record。

下一项唯一 owner decision 是严格下游 textual node
`O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR`：

- **A（推荐，pending）**：现在提供 immutable canonical permalink；若知道 exact record ID，可
  一并提供。
- **B（pending）**：仅当该受控系统没有 immutable permalink 时，提供该系统明确的
  instance/name 与 exact record ID。
- **C（pending）**：当前两者都不能提供，继续 pending / STOP。C 不等于 record 不存在，不自动
  撤回 `O1E-ANCHOR-CONTRACT-PATH=A` 或 `O1E-ANCHOR-CONTRACT-LOCATOR=A`，也不授权建立或
  采用新 baseline。

三项按当前可提供性互斥。取得 actual canonical locator 后才可进入 `TUPLE_VERIFICATION`；在此
之前不得询问或进入 O1E-ENDPOINT。

### 7.6 O1E-BASIS：product endpoint primitive comparison basis

决策来源：用户于 2026-08-17 在 endpoint-basis 对齐中选择推荐 **O1E-BASIS=A**。这是稳定的
textual decision-record ID，不是 JSON/schema/protocol field；其 adopted normative clause 只见
[Q3 design §6.1](m4-minimal-gyro-q3-offline-bias-alignment-design.md#61-primary-diagnostic-componenthac)。

O1E-BASIS 的 A/B/C 处置如下：

- **A（selected）**：arm-wise common-reference quality。zero-bias arm 与 fitted-bias arm 先各自
  相对同一个 downstream product-reference role 评价 endpoint-native quality；paired benefit 只能
  随后由两项 arm quality 派生。common reference 必须两臂共享、arm-independent、pre-outcome，
  不得由任一 arm output、fitted bias、future DUT/outcome 或 desired PASS 选择。
- **B（not selected for this evidence chain / non-normative）**：reference-free self-consistency。
  本轮未选择不等于其在其他问题中全局无效，但它不持有本 evidence chain 的 primitive basis
  authority。
- **C（not selected for this evidence chain / non-normative）**：native paired-benefit endpoint。
  本轮未选择不等于其在其他问题中全局无效，但它不持有本 evidence chain 的 primitive basis
  authority。

该选择本身没有冻结 reference；随后独立选择的 O1E-REFERENCE=A 仅冻结 exact bytes/source
identity。两项选择合并后仍未冻结 endpoint semantics、metric/loss/formula、1 s analytical
window/block、time pairing、axis/frame/
direction/handedness、target population/domain、risk/severity/threshold、mapping assumptions/formula、
`μ_d`、positive `δ_MME`、rounding/strict boundary、`O1E-MME`/`O1E-ADJ`/`O1E-POWER`、
`U/S/V/A` decisions、schema/protocol numeric field、implementation go 或 Q3 outcome。Q3 既有
`ΔRvis/r0/rb/d_k` 解析代理、primary 与 adjacent estimand 不变；只有 endpoint semantics、metric
与 mapping 后续另行冻结后，
product endpoint 的两项 arm quality 才可单向映射到 Q3 paired estimand，绝不能由 `d_k`、
`μ_d` 或 outcome 反向定义 reference 或 threshold。

### 7.7 O1E-REFERENCE：common reference source identity

决策来源：用户于 2026-08-17 明确选择 **O1E-REFERENCE=A**。这是 user/owner
adopted stable textual decision record，不是 schema field，也不是 methodology PASS；用户明确采用的
是其指向的 exact `est.tum` bytes/source identity，而非本文后续为其定义的 stable
normalization alias `q1-final-visual-posterior-proxy-v1`；该 alias 也不是 Q1 ledger 原生字段。其
adopted normative clause 只见
[Q3 design §6.1](m4-minimal-gyro-q3-offline-bias-alignment-design.md#61-primary-diagnostic-componenthac)。

O1E-REFERENCE 的原始 A/B/C 处置如下。A 已由 owner 选择并冻结，本次 amendment 不重开、撤销
或降级该选择；B/C 只是在本 evidence chain 未选，不构成对其他用途的全局否决：

- **A（selected / 已冻结）**：冻结 Q1 final independent PASS run
  `q1_observe_final_q1_final_verifier` 发布的 `est.tum` exact bytes/source identity
  （SHA-256 `18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321`），作为唯一
  common reference source；只提供 identity/provenance authority。本文以
  `q1-final-visual-posterior-proxy-v1` 作为该已选 identity 的 stable normalization alias。完整
  provenance 见 §1.1。
- **B（not selected for this evidence chain / non-normative / no qualification authority）**：采用
  另一份具有明确 identifier、version、owner 与 baseline provenance 的 downstream product
  reference。本轮未选不等于该 reference 在其他问题中全局无效。
- **C（not selected for this evidence chain / non-normative / no qualification authority）**：在
  owner 提供可审计 reference contract 前保持 O1E-REFERENCE `UNFROZEN` / STOP。本轮未选不等于
  该 closure path 在其他问题中全局无效。

该选择只闭合 source identity：two arms shared、arm-independent、pre-outcome、
source-identity-only。它不选择 endpoint semantic object/record、parser/pairing/window/frame、
metric/aggregation/population/risk/threshold/mapping，也不升级 `ΔRvis/r0/rb/d_k`，更不闭合
O1E-ANCHOR-CONTRACT。O1E-ANCHOR-CONTRACT-PATH=A 已选择；当前下一道单一 owner decision 是
O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR；O1E-ANCHOR-CONTRACT-LOCATOR=A 已选择且只表示
external authority location/class；
O1E-ENDPOINT 必须等待。

## 8. Owner choices 顺序与最小下一步

`O0=A` 与 O1-DELTA=A（`primary.delta_loss_rad2=0`）已 resolved 并退出待决队列。以下只记录依赖顺序、
候选职责与待 owner 选择的
alternatives，不构成 adopted protocol；按顺序一次只问一项：

1. `O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR`（pending）：O1E-ANCHOR-CONTRACT-PATH=A 与
   O1E-ANCHOR-CONTRACT-LOCATOR=A 已选择，当前
   `PATH_A_SELECTED / LOCATOR_A_SELECTED(EXTERNAL_AUTHORITY) / AWAITING_CANONICAL_LOCATOR`；
   只选择 §7.5 所列 canonical-locator A/B/C。三项都不得自动填写 actual
   identifier/version/owner/baseline provenance、category locator、authorization 或 adoption；
   actual canonical locator 确定后必须先完成 tuple verification，验证通过后才允许 owner
   authorization；在两者依序完成前，
   O1E-ANCHOR-CONTRACT 保持 pending /
   `UNFROZEN`，O1E-ENDPOINT 必须等待。
2. `O1E-MME`（pending / `UNFROZEN`）：O1E-PATH=A、O1E-ANCHOR=A、
   O1E-BASIS=A 与 O1E-REFERENCE=A 已选择，但 O1E-ANCHOR-CONTRACT 尚未闭合，primary positive
   `δ_MME` evidence package + owner adoption decision 本身仍未决。合同闭合后才可进入
   O1E-ENDPOINT；随后严格按 metric/loss/formula → 1 s product analytical window/block + timestamp
   pairing → frame/axis/direction/handedness → population/domain → severity/risk → requirement threshold →
   mapping → derived candidate positive `δ_MME [rad²]`（同时明确 `μ_d` 是未知 estimand，只表达
   映射目标语义）→ rounding → strict boundary 的顺序逐项冻结，不得合并提问或越级采用结果。
   到此只完成 `O1E-MME` adoption-ready evidence package；
   随后 owner 才独立作 `O1E-MME` adoption decision。只有 owner 明确 adopt 后，`O1E-MME`
   才 closed/resolved，`δ_MME` 才冻结；在此之前（包括 candidate、rounding 与 strict
   boundary 已完成但尚未 adopt）均保持 `UNFROZEN`，`μ_d` 也不得被采用为设计阈值。
3. `O1E-ADJ`（pending / `UNFROZEN`）：只有 owner 明确 adopt positive `δ_MME`、使
   `O1E-MME` resolved 后，才进入 adjacent-equivalence effect/margin。
4. `O1E-POWER`（pending / `UNFROZEN`）：只有 `O1E-ADJ` 完成后才进入 target power。
5. `O2`：ADI typical specs 只作 sanity floor，还是在何种明确模型下进入三轴
   full-fit uncertainty 与 half-stability budgets；
6. `O3`：在已冻结 O0=A 下选择 12 项 equivalence margins 与 design alternatives，并保持 sole
   overall IUT、component diagnostic-only 与 no-component-claim 语义；
7. `O4`：trusted outer launcher/verifier、build attestation、embedded protocol bytes、Q1/Q2 ledger
   与 out-of-band expected manifest digest 的 provenance architecture；
8. `U1`：独立冻结 exact input/eligibility/near-pi/error table 与 calendar identity；
9. `U2`：独立冻结 block/adjacent estimands、duration normalization 与 calendar index；
10. `U3`：独立冻结 zero-init objective、Q2-helper central-FD Jacobian、rank rule 与 HAC
   sandwich estimator；
11. `S1`：在 `U1/U2/U3/O1E-MME/O1E-ADJ/O1E-POWER/O2/O3` 都冻结后，冻结与
    O0=A 一致的 alpha/critical law、TOST、
    calendar-gap、Bartlett bandwidth/normalization/correction；IUT 代数不会替 owner 冻结 alpha；
12. `S2`：只有 owner 另行授权 prefix-only nuisance 后，才冻结 prospective sizing、support 与
    precision-infeasible semantics；
13. `V1`：先冻结逐 gate boundary truth table；
14. `V2`：在 V1 之后，由 owner 在以下互斥 alternatives 中选择 overall mixed-verdict
    precedence：
    - **A（pending）**：保持 provisional candidate
      `HARD_ERROR -> INCONCLUSIVE -> HYPOTHESIS_FAIL -> PASS`；
    - **B（pending）**：采用 reviewer candidate
      `HARD_ERROR -> HYPOTHESIS_FAIL -> INCONCLUSIVE -> PASS`；
    - **C（pending）**：不折叠 mixed state，另定义并论证 explicit mixed verdict。

    三者均 pending；“现行”或“推荐”只描述候选来源，不等于 owner selected，`O0=A` 也不冻结
    V2。
15. `A1`：最后冻结 canonical JSON/CSV/manifest schemas 与 external trust anchor。

下一次只问 `O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR`：**A（推荐，pending）** 现在提供
immutable canonical permalink（若知道 exact record ID 可一并提供）；**B（pending）** 仅当该
受控系统无 immutable permalink 时，提供明确 system instance/name 与 exact record ID；
**C（pending）** 当前两者都不能提供，继续 pending / STOP。C 不等于 record 不存在，不自动撤回
path A/locator A，也不授权新 baseline。三项按当前可提供性互斥；actual canonical locator 未取得
前不得进入 `TUPLE_VERIFICATION` 或询问 O1E-ENDPOINT。O1E-ANCHOR-CONTRACT-LOCATOR=A
selected 只选择 external authority location/class，不是 actual locator payload。
O1E-REFERENCE=A 保持冻结且不由本次重开或撤销。

最小实验也必须在上述数值与 protocol hash 冻结后进行：先只用 synthetic fixtures 和 boundary
mutants 验证单位、timestamp、eligibility、duration split invariance、calendar gap、FD step-halving、
HAC/TOST truth table 与 manifest attacks。primary strict-boundary 的 executable oracle 仅见
[Q3 design §10.2](m4-minimal-gyro-q3-offline-bias-alignment-design.md#102-green--independent-requalification)；
本 budget 不复制或扩展该 oracle。再由独立 reviewer 检查 protocol bytes。不得为这一步读取 Q3
suffix 或运行 DUT。

当前结论保持：**O0=A、O1-DELTA=A（`primary.delta_loss_rad2=0`）、O1E-PATH=A、
O1E-ANCHOR=A、O1E-BASIS=A、O1E-REFERENCE=A、O1E-ANCHOR-CONTRACT-PATH=A 与
O1E-ANCHOR-CONTRACT-LOCATOR=A 已决，但 `δ_MME` 仍为 `UNFROZEN`；locator A 只选择
external authority location/class，不是 actual locator payload；当前为
`PATH_A_SELECTED / LOCATOR_A_SELECTED(EXTERNAL_AUTHORITY) / AWAITING_CANONICAL_LOCATOR`，
actual canonical locator、identifier、version、owner、baseline provenance、category locator、
authorization 与 adoption 均未提供、未验证，O1E-ANCHOR-CONTRACT pending / `UNFROZEN`，
下一次只问 O1E-ANCHOR-CONTRACT-CANONICAL-LOCATOR，actual canonical locator 未取得前不得进入
TUPLE_VERIFICATION，O1E-ENDPOINT 必须等待；
`O1E-MME`、`O1E-ADJ`、`O1E-POWER` 仍依序 pending / `UNFROZEN`：
`O1E-ADJ` 仅在 owner 明确 adopt positive `δ_MME`、`O1E-MME` resolved 后才进入，
`O1E-POWER` 仍在 `O1E-ADJ` 完成后 pending；其余 owner/protocol decisions pending；Q3
implementation STOP；Q3 plan 5/5 todos 仍 pending；没有任何 Q3 outcome。**
