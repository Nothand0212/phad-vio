# M4 Q3 offline constant-bias alignment 调研

日期：2026-08-18

> **历史 / non-normative。** 本文只保留一手资料、Q1/Q2 provenance 与工程取舍依据；所有旧
> external locator、owner budget、HAC/TOST/power decision DAG 已被 2026-08-18 用户事实取代，
> qualification authority/weight=`0`。唯一可执行合同是
> [Q3 design](2026-08-13-note-m4-minimal-gyro-q3-offline-bias-alignment-design.md)。

用户已明确不存在任何外部需求记录，唯一目标是最终做出 VIO，工程实现选择授权团队。因此本文
不再维护 pending locator、next question 或外部 requirement blocker，也不依据未来 Q3 结果选门。

## 1. 冻结项目事实

Q1 final independent PASS 固定了四项 Q3 输入：

| artifact | SHA-256 |
|---|---|
| `est.tum` | `18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321` |
| `diag.csv` | `1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb` |
| `gyro_packets.csv` | `fbf574545e418fc19d100b7b05f79546a5336420bca60852770605b42702a3fe` |
| `gyro_samples.csv` | `da35227b40a1ab47c94217b4210b5445d11927a864d6634f8b824812ccec0344` |

来源身份是 source commit `74270572cc1fcc2eac82559117efd0951c800ab9`、tree
`e4379bf44db8d1127450d674b60b2e451fbe0aef`、config hash `402d1925` 与 input manifest v2
SHA-256 `aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5`。
Q1 证明 Observe 落盘与 visual output 隔离，不证明 gyro bias fit。

Q2 fixed code `d1c4385809a6bf461f1c6b8acd81870f98634aa0` / tree
`9c9c991b21c4d40e8c5f2ce974334b76e1a42b64` 的 `integrateGyroRotation` 已获 technical PASS
under one-time post-hoc evidence-retention waiver；原 RED exact-record conformance 仍 NOT MET。
已验证合同是 timestamped rad/s samples、endpoint trapezoid、bias once、chronological
right-compose 与 checked exact duration。Q2 没有 product caller，也未证明 estimation 或 VIO。

## 2. 成熟方案的一手资料

下表严格区分 source fact 与本项目 engineering choice。

| 一手来源 | source fact | 本项目选择（非来源事实） |
|---|---|---|
| GTSAM 4.3a0 commit [`3ad4b4c`](https://github.com/borglab/gtsam/tree/3ad4b4c3cb28394c9597f48fa02dad361c8450e3) | [`PreintegratedRotation`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/PreintegratedRotation.cpp#L68-L111)执行 `measuredOmega-bias`、`Expmap`、right-compose；[`AHRSFactor`](https://github.com/borglab/gtsam/blob/3ad4b4c3cb28394c9597f48fa02dad361c8450e3/gtsam/navigation/AHRSFactor.cpp#L122-L168)以 between/Logmap 构造 rotation residual/Jacobian。 | Q3 复用 SO(3) residual 和 Q2 helper，不引入 AHRS/Combined factor、noise prior 或 graph optimization。 |
| Kimera-VIO commit [`ce8c59b`](https://github.com/MIT-SPARK/Kimera-VIO/tree/ce8c59b7b273ab5ac29db7e5572e1623760e19c7) | [`estimateGyroscopeBias`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/OnlineGravityAlignment.cpp#L246-L280)用 Logmap residual 与 preintegration bias Jacobian 解线性 Gaussian graph；[`estimateBiasAndUpdateStates`](https://github.com/MIT-SPARK/Kimera-VIO/blob/ce8c59b7b273ab5ac29db7e5572e1623760e19c7/src/initial/OnlineGravityAlignment.cpp#L214-L242)服务 online window。 | 只借鉴显式 bias alignment 结构；不复制 window、gravity/velocity 状态、threshold 或 online state update。 |
| VINS-Fusion commit [`be55a93`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/tree/be55a937a57436548ddfb1bd324bc1e9a9e828e0) | [`solveGyroscopeBias`](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/initial/initial_aligment.cpp#L14-L46)以相邻帧 rotation residual/Jacobian 解 bias，随后进入视觉惯性 alignment。 | Q3 采用相邻 interval 与一次线性 solve，但额外要求 held-out calendar blocks；不把 online initializer 当 qualification oracle。 |
| ORB-SLAM3 v1.0-release-2 commit [`4452a3c`](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4) | [`InertialOptimization`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L3042-L3145)联合 pose/velocity/bias/gravity/scale；[`InitializeIMU`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc#L1173-L1275)依赖在线 map/keyframes 和 support/time state。 | 借鉴“不满足资格就不进入 initialized state”；不复制其时间门、prior 或地图状态机。 |

共同事实是成熟 VIO 会显式处理 bias、SO(3) residual 与初始化状态；没有一手实现替本项目定义
held-out pass/fail、artifact transaction 或 product threshold。Q3 的 100 s split、one-shot FD LS 与
四个 engineering gates 均是团队选择，不冒充这些项目的标准答案。

## 3. 数值与统计依据

Eigen 3.4 [`SVDBase`](https://gitlab.com/libeigen/eigen/-/blob/3.4.0/Eigen/src/SVD/SVDBase.h#L142-205)
以相对最大 singular value 的 threshold 判定非零；三列矩阵的 documented default 对应
`3*epsilon*sigma_max`。Q3 直接冻结该 rank 规则，并另设 `condition<=1e6` 作为团队的数值放大
边界；后者不是 Eigen 产品保证。

Newey 与 West 的[原论文](https://doi.org/10.2307/1913610)给出 HAC covariance；Schuirmann
1987 的[原论文](https://doi.org/10.1007/BF01068419)给出 TOST。它们分别回答相关序列的渐近
standard error 与 equivalence inference。Q3 V1 不发布总体显著性或 equivalence claim，只对固定
input 做确定性 effect/support gate，所以不需要估 SE、alpha、power 或 margin。删除 HAC/TOST/power
避免引入无助于当前 Q4 stop/go 的 covariance、small-sample 与 risk 假设；若未来要跨序列做总体
推断，必须另立协议，不能事后把 Q3 V1 改称统计推断。

Analog Devices [ADIS16448 Rev. H datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16448.pdf)
列出 typical bias repeatability `0.5 deg/s`、in-run bias stability `14.5 deg/h`、ARW
`0.66 deg/sqrt(h)` 和 rate noise density `0.0135 deg/s/sqrt(Hz)`，且各自测试条件/统计对象不同。
这些是 sensor facts，不是本项目 visual-proxy residual 的保证。Q3 half-fit `1e-3 rad/s` 是团队工程
边界，不从上述 typical 数值伪推产品要求。

## 4. Interface alternatives

选定接口是 pathless、typed 的完整 analyzer：

```cpp
GyroAlignmentResult analyzeGyroAlignment(const GyroAlignmentInput& input);
```

其优点是 eligibility、fit、validation 与 verdict 聚合在一个 deep module 内，CLI 只承担 I/O。

statistics-centered alternative 是把数值汇总单独暴露：

```cpp
GyroAlignmentStatistics summarizeGyroAlignment(
    std::span<const GyroAlignmentBlock> blocks,
    const BiasTriplet& fitted_biases);
```

调用示例：

```cpp
const auto stats = summarizeGyroAlignment(result.blocks, result.biases);
const auto verdict = evaluateGyroAlignmentGates(stats);
```

它只需 Eigen 与 STL，便于独立替换 gate 或跨工具复用 summary；代价是向 caller 暴露 block、bias
和 verdict 两段协议，使顺序、support 与 error precedence 更容易分叉。当前只有一个 caller，也不
需要可替换 statistics engine，因此不采用该 seam；统计仍是 analyzer 内部纯 helper，等出现第二个
真实 caller 再评估提取。

## 5. Supersession ledger

| 历史事项 | 2026-08-18 disposition |
|---|---|
| O1E external locator/owner/requirement record 路线 | superseded；non-normative；qualification weight `0` |
| 旧 HAC/TOST/power、多层 component/IUT DAG | 从 active protocol 删除；仅保留来源背景 |
| 旧固定值曾被称为“无依据 hints” | 不再作为外部 requirement；由团队以 outcome-independent engineering boundary 重新冻结于 V1 design |
| 旧 Q1/Q2 结果 | 原结论与证据身份不变，不因 Q3 supersession 改写 |

新 authority 是 repo-owned `PHAD-M4-Q3-GYRO-ALIGN-V1`，唯一 normative 文本为 Q3 design。
七份 docs 的 exact commit 已成为 `HEAD` 且 worktree clean 后、任何实现前，由独立
preflight 创建唯一 write-once protocol identity receipt，绑定 docs commit/tree、七份 docs
各自的 path/git blob/SHA-256，以及
`design_commit/design_tree/design_blob/design_sha256`；Git objects 始终是权威。真实
qualification 后新增的 result ledger 只能原样引用该已存在的 receipt/identity，不得届时
首次声明、重新计算后选择或改绑。
