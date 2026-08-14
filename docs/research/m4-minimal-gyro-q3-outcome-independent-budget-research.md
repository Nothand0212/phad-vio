# M4 Q3 outcome-independent qualification budget 调研

日期：2026-08-13

状态：**O0=A 已冻结 / 其余 owner 与 protocol decisions pending / Q3 implementation STOP**。
本文没有打开或统计 Q1 runtime CSV/TUM/diag 内容，没有读取 Q3 held-out suffix、GT/ATE/RPE、
Q5 结果或 future DUT，也没有生成、执行或判定任何 Q3 outcome。

关联：issue [#39](https://github.com/Nothand0212/phad-vio/issues/39)、
[Q3 结构调研](m4-minimal-gyro-q3-offline-bias-alignment-research.md)、
[Q3 normative 设计](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)、
[Q3 计划](../plans/2026-08-13_m4_gyro_q3_align_0e897ba8.plan.md)。

文档角色：本文是 **evidence + owner decision record**，保留来源、数学推导、候选评估与
A/B/C 决策历史；不是 executable protocol authority。O0 与统计合同的 adopted normative
定义只在上述 Q3 design 规范节中维护。

## 1. Fixed point、权限与 leakage prohibition

issue #39 固定 structural Q3 plan 于 commit
`dbec0be89512f54c386319762838fe16296e7b1f`。本片只允许 protocol-amendment 的
research/design/plan；Q3 source/tests/CMake、执行、Q4+、factor-enabled experiment 与 posterior
变化均未授权。amendment 即使完成，也仍须另获明确 implementation go。

本次只记录 O0=A resolved；所有后续 owner/protocol decisions 仍 pending。尤其
`delta_loss`、alpha、mixed verdict precedence、schema、solver 和 support 均未冻结。

本研究采用以下防泄漏边界：

- 禁止从 Q3 suffix outcome、GT/ATE/RPE、Q5、future DUT 或 desired PASS 反推 threshold；
- 既有 Q1 runtime artifacts 只能用于 identity/schema/provenance 合同；只有 owner 后续明确授权，
  才可用 prospective prefix-only 数据估 nuisance variance，而且 effect、margin 与 risk 仍须先冻结；
- 官方规格、成熟实现常数和历史表现只可提供量纲、候选模型或 sanity floor，不能自动成为产品门；
- 本文所有数值均来自公开一手规格的单位换算或显式标注的候选假设，不是 Q3 outcome。

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
prospective calculation 判定预注册的 target power 在预算内不可达，或候选设计 power 低于该目标，
应按预注册规则映射为 `INCONCLUSIVE`，不能通过改 margin 取得通过。

## 4. Budget traceability chain

每个 verdict-changing 数值都必须有一行完整链；任一环缺失，protocol field 保持 `UNFROZEN`。

| source | source units | protocol units | explicit assumption | owner approval required | candidate protocol field / current state |
|---|---|---|---|---|---|
| 产品风险：corrected arm 不得更差 | product policy | `rad^2` | `d_k=loss_zero-loss_corrected`，candidate one-sided gate 为 `LCB(d)>delta_loss` | owner 尚未选择损失容忍 | `primary.delta_loss_rad2=0` / **推荐 A 候选，pending** |
| ADIS16448 ARW `0.66 deg/sqrt(h)` | angle random walk | `rad/sqrt(s)`，再到 1 s `rad` | 若用于 stochastic model，须先冻结 white/stationary、dataset 配置适用性、轴相关与传播模型；目前 pending | 是否只作 sanity floor，或进入 stochastic model | `sensor.arw_rad_sqrt_s=1.91986e-4` / source fact；用途 pending |
| ADIS16448 IRBS `14.5 deg/h` | bias stability | `rad/s` | 若用于 budget，须先冻结 Allan-minimum typical 1 sigma 对目标 run 的适用性、轴相关与倍数；目前 pending | 接受 typical/model risk | `bias.full_uncertainty_budget_radps[3]`、`half.margin_radps[3]` / pending |
| ADIS16448 bias repeatability `0.5 deg/s` | long-term offset | `rad/s` | 若用于 sensitivity，须先论证跨环境 population scale 与本次 100 s estimand 的关系；目前 pending | 通常应拒绝作 Q3 tight hard gate | sensitivity-only / pending |
| ADIS16448 rate ND `0.0135 deg/s/sqrt(Hz)` | rate spectral density | residual `rad` 或 loss `rad^2` | 若用于 residual/primary model，须先冻结 bandwidth、filter、sampling、integration、axes covariance 与 visual error；目前 pending | 批准完整 noise propagation model | residual/primary design input / pending |
| 产品容许 constant-bias mismatch | hazard/product units | `rad/s` | 若用于 budget，须先冻结 full/early/late estimator 的共同 frame 与 axis 定义；目前 pending | 逐轴预算及风险等级 | full-fit uncertainty、half-difference margins / pending |
| 产品容许 held-out orientation structure | hazard/product units | mean `rad`、time slope `rad/s`、speed slope `s` | 若采用 joint regression，须先冻结 time/speed predictor、centering、scaling 及共同 frame/axis；目前 pending | 12 margins 与 design alternatives | equivalence margins / pending |
| 产品最小有意义改善 | product value | primary `rad^2`；adjacent rate-loss `rad^2/s^2` | 若用于 prospective sizing，须先冻结 block/adjacent estimand、duration normalization 与实验单位；目前 pending | effect 与 target power | prospective support/power / pending |
| Schuirmann/ICH/FDA | dimensionless alpha/CI | matching estimator units | equivalence margin 须事先给定，且 inference coverage 依赖所选 SE/df/HAC model | owner 仍须批准 margins 与 S1 risk choices | O0 已决；adopted executable 语义见 [Q3 design §6](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)；其余 pending |
| Newey-West/Bartlett | indexed covariance | estimator-specific squared units | 若采用 HAC，须先冻结 calendar lag、kernel、bandwidth、normalization、correction 与 critical law；目前 pending | 接受 asymptotic risk与 sensitivity rule | `hac.*` / pending |

表中的 candidate formula、model 与 estimand 均只是待 owner 选择且补齐前提的候选；在对应链条
完整冻结前，它们都不是 protocol fact，不得写入 executable verdict 合同。

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
3. `O0=A` 是 resolved input；下一项为 `O1` delta loss/effects/power，随后依次冻结 `O2` 三轴
   uncertainty/stability budgets、`O3` 与 O0=A 一致的 12 margins 与 design alternatives、`O4`
   provenance architecture。
4. `(U1,U2,U3,O1,O2,O3) -> S1`：冻结 alpha/critical law/TOST/HAC/gap rules。
5. 只有 owner 明确授权 prefix-only nuisance 后，`S1 -> S2` prospective
   support/power/infeasible rule。
6. `S2 -> V1 -> V2`：先冻结逐 gate boundary truth table，再由独立 owner 决策冻结 overall
   mixed-verdict precedence。
7. `(U1,V2,O4) -> A1`：canonical JSON/CSV/manifest 与 external trust anchor；随后
   `A1 -> R1` 独立 Standards/Spec review。
8. `R1` 通过后 amendment 才可称 `COMPLETE`；amendment `COMPLETE` 仍是 implementation
   `STOP`，必须另获 separate implementation go。

## 7. Owner decisions：O0 已决，下一项为 O1 `delta_loss`

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

### 7.2 O1：`delta_loss`

先固定符号与 candidate gate：

\[
d_k=loss_{zero,k}-loss_{corrected,k},\qquad LCB(d)>delta_{loss}.
\]

若产品允许 corrected arm 比 zero arm 最多坏 `delta_allow>=0`，则阈值必须取
`delta_loss=-delta_allow`，不能把正的允许损失直接填入 `delta_loss`。

owner 尚未回答本题，推荐 A 也尚未获 owner 批准；当前选项为：

- **A（推荐）**：`primary.delta_loss_rad2=0`。qualification 不允许 corrected arm 的 held-out 平均
  squared loss 变差；最小有意义正向 design effect 另由 owner 产品目标给出；ADI 规格只作
  uncertainty/sanity floor。优点是语义直接、无需把 typical sensor stats 冒充 guarantee。
- **B（候选但当前不可执行）**：若产品允许 corrected 最多坏
  `delta_allow=1.25e-7 rad^2`，则对应的是 `delta_loss=-1.25e-7 rad^2`。这个数目前仍不可用于
  executable field：其原候选算法把 IRBS `7.02980e-5 rad/s` 静默当作 1 s angle stochastic term，
  但缺少随机过程与传播合同；还额外假设 ARW 与 IRBS 可独立合成、三轴同分布且 dataset 配置
  适用。在完整 noise process、integration/filter、axis covariance 与 visual error 模型冻结前，
  不得将该候选写入 protocol。
- **C**：owner 提供非零 application loss budget `[rad^2]` 及 hazard/risk 来源；不得引用 Q5、
  suffix、GT/ATE 或 desired PASS。

`primary.delta_loss_rad2` 仍为 pending；effect、power 和其余数值门也待后续 owner input，不在本文
填具体数值。owner 回答本题只会冻结这一项，不代表 protocol complete。

## 8. Owner choices 顺序与最小下一步

`O0=A` 已 resolved 并退出待决队列。以下只记录依赖顺序、候选职责与待 owner 选择的
alternatives，不构成 adopted protocol；按顺序一次只问一项：

1. `O1` 的 `delta_loss`：先选择 Section 7.2 的损失容忍；
2. `O1` 其余项：随后选择 primary/adjacent minimum meaningful effects 与 target power（具体数值由
   owner product/risk input）；
3. `O2`：ADI typical specs 只作 sanity floor，还是在何种明确模型下进入三轴
   full-fit uncertainty 与 half-stability budgets；
4. `O3`：在已冻结 O0=A 下选择 12 项 equivalence margins 与 design alternatives，并保持 sole
   overall IUT、component diagnostic-only 与 no-component-claim 语义；
5. `O4`：trusted outer launcher/verifier、build attestation、embedded protocol bytes、Q1/Q2 ledger
   与 out-of-band expected manifest digest 的 provenance architecture；
6. `U1`：独立冻结 exact input/eligibility/near-pi/error table 与 calendar identity；
7. `U2`：独立冻结 block/adjacent estimands、duration normalization 与 calendar index；
8. `U3`：独立冻结 zero-init objective、Q2-helper central-FD Jacobian、rank rule 与 HAC
   sandwich estimator；
9. `S1`：在 `U1/U2/U3/O1/O2/O3` 都冻结后，冻结与 O0=A 一致的 alpha/critical law、TOST、
   calendar-gap、Bartlett bandwidth/normalization/correction；IUT 代数不会替 owner 冻结 alpha；
10. `S2`：只有 owner 另行授权 prefix-only nuisance 后，才冻结 prospective sizing、support 与
    precision-infeasible semantics；
11. `V1`：先冻结逐 gate boundary truth table；
12. `V2`：在 V1 之后，由 owner 在以下互斥 alternatives 中选择 overall mixed-verdict
    precedence：
    - **A（pending）**：保持 provisional candidate
      `HARD_ERROR -> INCONCLUSIVE -> HYPOTHESIS_FAIL -> PASS`；
    - **B（pending）**：采用 reviewer candidate
      `HARD_ERROR -> HYPOTHESIS_FAIL -> INCONCLUSIVE -> PASS`；
    - **C（pending）**：不折叠 mixed state，另定义并论证 explicit mixed verdict。

    三者均 pending；“现行”或“推荐”只描述候选来源，不等于 owner selected，`O0=A` 也不冻结
    V2。
13. `A1`：最后冻结 canonical JSON/CSV/manifest schemas 与 external trust anchor。

最小实验也必须在上述数值与 protocol hash 冻结后进行：先只用 synthetic fixtures 和 boundary
mutants 验证单位、timestamp、eligibility、duration split invariance、calendar gap、FD step-halving、
HAC/TOST truth table、manifest attacks；primary future tests 还必须覆盖 `delta_allow=+delta` 与
`delta_loss=-delta` 的符号映射、`LCB(d)=delta_loss±δ` 两侧 fixture，以及 equality boundary
`LCB(d)=delta_loss` 不得通过严格 `>`。再由独立 reviewer 检查 protocol bytes。不得为这一步读取
Q3 suffix 或运行 DUT。

当前结论保持：**O0=A 已决；其余 owner/protocol decisions pending；Q3 implementation
STOP；没有任何 Q3 PASS/FAIL/INCONCLUSIVE outcome。**
