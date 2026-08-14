# M4 Q3 offline constant-bias alignment 调研

日期：2026-08-13

状态：一手资料对照、无依据旧 hints 的废止、deep one-call seam 选择以及 module/data/structural
contract draft 已完成；**O0=A 与 O1 primary delta-loss 零容忍已冻结，
effects/power 与其余 amendment 仍 pending，protocol amendment STOP**。
这不是 DUT fail：Q3 implementation 尚未授权，也没有生成 Q3 科学结果。

关联：issue [#38](https://github.com/Nothand0212/phad-vio/issues/38)、
[outcome-independent budget 调研](m4-minimal-gyro-q3-outcome-independent-budget-research.md)、
[Q3 normative 设计](m4-minimal-gyro-q3-offline-bias-alignment-design.md)

上位路线：[M4 gyro measurement / factor 资格实验设计](m4-minimal-gyro-slice-design.md)

本文只核对固定版本的一手资料与 Q1/Q2 资格账本。
文档角色是 **structural/historical research**：保留证据、来源对照、历史问题与结构性
选择，不定义可执行统计 protocol。O0 与统计合同只以 Q3 design 为 normative
authority。
以下标签含义为：

- **来源事实**：由固定版本的一手源码、论文或 Q1/Q2 账本直接支持；
- **受来源支持的推断**：由来源事实导出的工程判断，不能冒充外部项目合同；
- **结构性项目决定**：Q3 draft 中已经选定、但不足以单独构成完整 protocol 的结构；
- **protocol blocker**：implementation go 前仍须在 amendment 中精确定义并独立评审的合同。

## 1. 冻结来源与当前权限

### 1.1 Q1 artifact identity

Q3 runtime runner 只允许打开以下四个 Q1 artifact，并必须在 parse 前核对 SHA-256：

| artifact | SHA-256 |
|---|---|
| `est.tum` | `18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321` |
| `diag.csv` | `1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb` |
| `gyro_packets.csv` | `fbf574545e418fc19d100b7b05f79546a5336420bca60852770605b42702a3fe` |
| `gyro_samples.csv` | `da35227b40a1ab47c94217b4210b5445d11927a864d6634f8b824812ccec0344` |

Q1 final ledger 另记录 `config_hash=402d1925` 与 input manifest v2 SHA-256
`aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5`。两者只是 protocol
provenance，必须写入 Q3 report；runner 不能声称仅凭上述四文件重新验证了 config 或原
manifest。Q3 不得重读 dataset、重放 sync、使用 GT/ATE，或以新生成文件替换四个 identity。

### 1.2 Q2 已取得的最小能力

Q2 fixed code `d1c4385809a6bf461f1c6b8acd81870f98634aa0` 已对无 production caller 的
`integrateGyroRotation` 取得 technical PASS under one-time post-hoc evidence-retention
waiver；original frozen-plan RED exact-record conformance 仍为 NOT MET。已验证行为是：

- timestamped point samples，单位 rad/s；
- 相邻 endpoint average；
- bias 由 GTSAM 恰减一次；
- chronological right-compose；
- checked exact nanosecond duration。

Q3 可以逐 packet 复用该 helper；Q2 没有授权 bias solver、统计门、artifact publisher、产品
caller 或 posterior。

## 2. 一手成熟实现对照

### 2.1 GTSAM 4.3a0 的直接语义

当前依赖对应 GTSAM tag `4.3a0`、commit
[`3ad4b4c3cb28394c9597f48fa02dad361c8450e3`](https://github.com/borglab/gtsam/tree/3ad4b4c3cb28394c9597f48fa02dad361c8450e3)。
其
[`IncrementalRotation::operator()` 与 `integrateGyroMeasurement`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegratedRotation.cpp#L68-L111)
先执行 `measuredOmega - bias` 与 `Expmap`，再以
`deltaRij_ = deltaRij_.compose(incrR)` 累积。这直接支持 Q2 的 bias-once 与 chronological
right-compose，不替 Q3 定义 fit/held-out 或科学门。

### 2.2 Residual、Jacobian、bias initialization 与失败边界

| 实现与固定版本 | residual / Jacobian | bias initialization | 失败语义 | 对 Q3 的适用边界 |
|---|---|---|---|---|
| GTSAM 4.3a0 `3ad4b4c...` | [`AHRSFactor::evaluateError`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.cpp#L122-L168)以 `between` + `Logmap` 构造 SO(3) residual，并显式传播 rotation/bias Jacobian | [`ImuFactorsExample`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/examples/ImuFactorsExample.cpp#L172-L196)显式建立初值/先验，并在[后续 graph optimization](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/examples/ImuFactorsExample.cpp#L244-L288)联合估计；[`CombinedImuFactorsExample`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/examples/CombinedImuFactorsExample.cpp#L161-L189)是完整惯导状态图 | library/example 只提供 numerical/optimizer 行为，不定义 held-out qualification verdict | 复用 SO(3) residual、bias correction/Jacobian 语义；不引入 AHRS/Combined factor、`V/B` graph 或示例 noise/initial values |
| Kimera-VIO `ce8c59b7...` | [`constructVisualInertialFrames`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/OnlineGravityAlignment.cpp#L167-L205)取出 preintegration 的 gyro-bias rotation Jacobian；[`estimateGyroscopeBias`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/OnlineGravityAlignment.cpp#L246-L280)以 `Logmap` rotation residual/Jacobian 组装并求解线性 Gaussian graph | [`estimateBiasAndUpdateStates`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/OnlineGravityAlignment.cpp#L214-L236)要求零初值，选择 bias estimator 后更新在线 window 的 delta states | [同一入口](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/OnlineGravityAlignment.cpp#L214-L242)对非零初值或更新后高 residual 返回 `false`；它不定义 Q3 四态统计 verdict | 借鉴显式未初始化/失败状态；不移植窗口常数、gravity/velocity 状态或把 online bootstrap 当 held-out 证据 |
| ORB-SLAM3 v1.0-release-2 `4452a3c4...` | [`Optimizer::InertialOptimization`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L3042-L3145)组装 pose/velocity/bias/gravity/scale 惯性优化；[`EdgeInertial`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/G2oTypes.cc#L514-L556)以 bias-corrected delta rotation 构造 `LogSO3` rotation residual 并开始显式 Jacobian 计算；[`EdgeGyroRW`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/G2oTypes.h#L635-L668)定义两个 gyro-bias vertex 间的 random-walk residual/Jacobian | [`LocalMapping::InitializeIMU`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc#L1173-L1275)使用在线 map/keyframes 并调用 inertial optimization | `InitializeIMU` 在 support/time/scale 不足时 early return；[已初始化门与 reset 路径](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc#L129-L145)属于产品状态机，不提供 Q3 四态统计 verdict | 借鉴 bias 明确未资格化时不进入已初始化状态；不移植时间阈值、priors、map-scale/gravity 优化或在线 constants |
| VINS-Fusion `be55a937...` | [`solveGyroscopeBias`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/initial/initial_aligment.cpp#L14-L46)用相邻帧 rotation residual/Jacobian 解 gyro bias，并[继续视觉惯性 alignment](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/initial/initial_aligment.cpp#L209-L216) | estimator 在[初始化流程](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L585-L609)调用并更新 preintegration | [初始化条件与状态转移](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/estimator/estimator.cpp#L726-L735)服务在线 pipeline，不是独立统计报告 | 借鉴相邻区间、SO(3) residual、bias correction/Jacobian；不照搬一次 LS、在线阈值或把 fit residual 当 held-out threshold |

**受来源支持的推断：**这些成熟实现共同证明：相邻 IMU/visual 区间、SO(3) residual、bias
correction/Jacobian，以及“未取得初始化资格就不进入已初始化状态”是可复用模式。没有一个来源
提供本项目 Q3 所需的 held-out practical margins、primary effect、power、adjacent supporting
gate 的 estimand/inference 或完整 artifact 事务；不能把其 online constants、adjacent residual
RMSE 或实现内阈值移植为本项目资格门。

## 3. 数值与相关序列资料

### 3.1 Eigen 3.4 SVD

Eigen 3.4
[`SVDBase`](https://gitlab.com/libeigen/eigen/-/blob/3.4.0/Eigen/src/SVD/SVDBase.h#L142-205)
以相对最大 singular value 的 threshold 判定非零；默认 threshold 为
`diagSize * epsilon`。它不支持凭空设一个固定 condition number 产品门。

结构性候选为：对 amendment 最终选定的 rank matrix
\(J\in\mathbb R^{m\times3}\)，rank 门要求 3，故 \(m\ge3\)，并使用 Eigen 3.4 documented default：

\[
diagSize=\min(m,3)=3,\qquad
\tau=\epsilon\,diagSize\,\sigma_{max}=3\epsilon\sigma_{max},\qquad
rank(J)=\#\{\sigma_i>\tau\}=3.
\]

condition 只作 descriptive。`condition <= 1e6` 没有来自 Eigen、GTSAM 或既有产品预算的
独立依据。rank matrix、evaluation point 与 uncertainty/covariance 仍属于 protocol blocker；
数值满秩不等于 practical observability。

### 3.2 Newey–West / Bartlett

Newey 与 West 原论文给出 heteroskedasticity/autocorrelation consistent、positive
semi-definite covariance estimator：
[A Simple, Positive Semi-definite, Heteroskedasticity and Autocorrelation Consistent Covariance Matrix](https://doi.org/10.2307/1913610)。
statsmodels `v0.14.5` 官方
[`sandwich_covariance.py`](https://github.com/statsmodels/statsmodels/blob/v0.14.5/statsmodels/stats/sandwich_covariance.py#L316-L376)
实现 Bartlett weights，并以 `floor(4 * (T / 100) ** (2 / 9))` 作为默认 `nlags`。

这支持把 Bartlett HAC 与该 bandwidth 作为可复现候选，不支持声称 Q3 已冻结 exact HAC：
normalization、small-sample correction、critical distribution、negative variance、small-n 与缺块
run/calendar-lag 语义仍须 amendment 决定。

## 4. 已选结构与明确未冻结项

### 4.1 Time split、fit halves 与 validation observations/blocks

首个满足 amendment 最终 exact schema/eligibility predicates 的 interval 定义 (t_0)：

\[
t_{mid}=t_0+50000000000\ \mathrm{ns},\qquad
t_{split}=t_0+100000000000\ \mathrm{ns}.
\]

- full fit：`t_prev >= t0 && t_cur <= t_split`；
- early half：`t_prev >= t0 && t_cur <= t_mid`；
- late half：`t_prev >= t_mid && t_cur <= t_split`；
- half-straddling interval 不进 halves，但可留在 full fit；split-straddling interval 不进
  validation；
- early/late 从相同权威初值独立 refit，使用相同 objective、weights、solver、Jacobian 与
  termination；validation 只使用 full-fit bias。

权威初值究竟是 zero 或其他值，连同完整 solver 合同，仍是 protocol blocker。

validation block 精确定义为

\[
B_k=[t_{split}+k\cdot10^9,\ t_{split}+(k+1)\cdot10^9]\ \mathrm{ns}.
\]

一个 block 只有在首 interval `t_prev == a`、末 interval `t_cur == b`、相邻 interval exact
首尾相接且全部 eligible/同 segment 时成立。packet 按 (k) 与时间排序；每个 packet 只调用
一次 Q2 helper，再 chronological right-compose 得 block IMU rotation。禁止重采样、重做 sync、
重复积分或跨缺块压紧序列。visual rotation 直接取 block endpoints：

\[
\Delta R^{vis}_{B_k}=R_{WB}(a)^\top R_{WB}(b).
\]

对 zero/full-fit bias 分别定义

\[
r_{0,k}=Log(\Delta R^{imu}_{B_k}(0)^\top\Delta R^{vis}_{B_k}),\quad
r_{b,k}=Log(\Delta R^{imu}_{B_k}(\hat b)^\top\Delta R^{vis}_{B_k}),
\]

\[
d_k=\lVert r_{0,k}\rVert^2-\lVert r_{b,k}\rVert^2\quad[\mathrm{rad}^2].
\]

shared endpoint 必须一致且只积分一次。near-\(\pi\) `Log` 的状态/错误语义，以及缺块后的 HAC
calendar-lag/run 规则仍是 protocol blocker。

对 validation 中**每个**满足最终 eligibility predicates 的 held-out packet interval
\(I_p=[t_{prev,p},t_{cur,p}]\)，structural observation 固定使用该 packet 的同一对 visual
endpoints：

\[
\Delta R^{vis}_{p}=R_{WB}(t_{prev,p})^\top R_{WB}(t_{cur,p}).
\]

zero/full-fit bias 两臂分别对该 packet 恰调用一次 Q2 helper，并记录
\(r_{0,p}=Log(\Delta R^{imu}_{p}(0)^\top\Delta R^{vis}_{p})\)、
\(r_{b,p}=Log(\Delta R^{imu}_{p}(\hat b)^\top\Delta R^{vis}_{p})\) 与逐 interval paired
improvement \(d^{adj}_p=\lVert r_{0,p}\rVert^2-\lVert r_{b,p}\rVert^2\ [rad^2]\)。禁止为
adjacent observation 重采样、merge、跨 gap 拼接或改换 visual endpoints。

这些 adjacent observations 与 1 s blocks 共用底层 packets，不能和 block sample 混合、相加或
当作独立重复样本。这是本 research 保留的 structural dependency；其统计角色、inference、
support 和 verdict 语义不在此定义，只见
[Q3 design §6](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)。

### 4.2 统计结构

O0=A 的 owner 决策已被 design 采用；本文只保留这一 historical trace，不重述 claim、
level、PASS 或 component 的可执行定义。完整规范只见
[Q3 design §6](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol)。

fixed point 的全部旧 alpha 数值/分配已退役，不得当作候选资格证据；唯一逐项账本见
[budget research §4.1](m4-minimal-gyro-q3-outcome-independent-budget-research.md#41-fixed-point-旧-alpha-tuple-退役账本)。
本 research 也没有采用任何无 exact command/log/hash 的 support census；此类计数的
qualifying weight 为 `0`。

## 5. 被否决的旧 future hints

| 旧 hint | 否决原因 | draft 替代方向 |
|---|---|---|
| `condition <= 1e6` | 无独立数值/工程依据；condition 不代表 practical observability | machine-epsilon rank 候选 + 外部 full-fit uncertainty budgets |
| 固定至少 60 blocks | 未绑定 alpha/effect/power | prefix-only prospective support/power contract |
| 未枚举 22 项 Holm family | 历史 procedure 不可执行；关联旧 alpha tuple 的退役状态见 [budget ledger](m4-minimal-gyro-q3-outcome-independent-budget-research.md#41-fixed-point-旧-alpha-tuple-退役账本) | adopted 合同只见 [Q3 design §6](m4-minimal-gyro-q3-offline-bias-alignment-design.md#6-o0-normative-statistical-protocol) |
| fit-half Mahalanobis “相容” | non-rejection 不证明 practical equivalence | 两 half 独立 refit + 每轴 TOST/equivalence margin |
| 旧 adjacent/RMSE hints | 只有描述性 residual/RMSE，没有 estimand、相关性、multiplicity、margin 或 power 合同；已 superseded、non-normative、qualifying weight `0` | mandatory adjacent diagnostic component；完整 inference 在 amendment 冻结 |

历史 Q1 plan 不回写；其中 future hints 仅保留审计价值，从 Q3 起不具规范性。

## 6. Protocol amendment blocker

amendment 必须在查看 Q3 suffix outcome 前闭合 input/schema/error、solver/Jacobian/rank/
uncertainty、statistics/support/budgets/verdict 以及 artifact/publication 五类 blocker，并通过
独立评审。本 research 不再维护一份平行 checklist；完整待决合同、O0 已冻结条款与
测试要求只见 [Q3 normative design](m4-minimal-gyro-q3-offline-bias-alignment-design.md)。

Q5 的 `0.001 rad` 1 s product RPE guardrail 不能在缺少物理/统计推导时跨维度变成 rad/s bias
budget、slope margin 或 rad² effect。

因此当前结论是 **protocol amendment STOP**：没有 DUT 被运行或判失败；Q3 implementation、
qualification、result 与 Q4 均不得开始。完整 blocker 与 manifest-last 发布决定见
[Q3 设计](m4-minimal-gyro-q3-offline-bias-alignment-design.md)。
