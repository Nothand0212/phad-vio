# 证据门控的信息接入

本文档描述当前约定，不是绝对约束，会随项目开发修订。

## 1. 适用范围

当系统已有可工作的 baseline，又要接入一种会改变既有输出的新信息时，使用本文方法。
“新信息”包括但不限于：

- 新传感器、先验、模型或 learned output；
- 新 residual、factor、loss、optimizer 或初始化结果；
- 会参与状态更新、决策或反馈闭环的外部服务结果。

本文补充[增量开发](incremental-development.md)：vertical slice 解决“本次贯通哪一个行为”，
证据门控解决“新信息何时有资格影响既有行为”。

## 2. 核心原则

接入不是“数据已经传到代码里”，而是**把改变 posterior 或决策的权力交给新信息**。
这个权力必须逐级取得：先证明合同正确，再证明它能独立解释现实，然后证明它在受控条件下
能正确改变结果，最后才允许它进入自然闭环和默认路径。

每个 slice 只让一条尚未证明的因果边取得新权限；为它服务但不改变输出的 plumbing 可以同片
交付。例如：

```text
数据可达 → 独立预测成立 → nuisance parameter 可辨识 →
受控 posterior 变化符合预期 → 自然闭环不回归 → production 默认
```

不要在同一片中同时让数据通路、初始化、状态、factor、噪声调参和闭环反馈取得新权限。
否则终局指标变化时，无法判断是哪一条假设成立或失败。这里追求的“最小”首先是**不可区分
的假设最少**，不只是代码行数最少。

## 3. 三类问题必须分开

| 层面 | 要回答的问题 | 典型证据 |
|---|---|---|
| 合同正确性 | 时间、单位、坐标系、方向、区间、缺测语义是否正确 | 不变量、对拍、边界样例、失败注入 |
| 机制正确性 | 在不依赖终局指标时，新信息能否预测或约束目标量 | synthetic oracle、开环残差、held-out residual、Jacobian |
| 产品收益 | 进入真实反馈闭环后，质量、鲁棒性、覆盖率和成本是否可接受 | 自然 replay、跨场景 benchmark、退化段诊断 |

ATE、成功率或业务 KPI 通常只回答第三类问题。它们不能证明前两类正确，也不应在前两类
失败时被用来搜索 covariance、权重或阈值。

## 4. 资格阶梯

| 阶段 | 新信息拥有的权限 | 本阶段唯一问题 | 通过证据 | 失败时 |
|---|---|---|---|---|
| Q0 Control | 无 | 既有 baseline 是否冻结且可复现 | 代码、配置、数据、产物与指标指纹 | 先修复基线，不开始接入 |
| Q1 Observe | 可采集、落盘、诊断；不可改输出 | 数据合同是否真实成立 | 时间/单位/坐标系/区间检查；off 路径等价 | 显式失败或局部标记无效 |
| Q2 Predict | 可独立计算预测；不可进 posterior | 新信息本身是否解释目标量 | synthetic 对拍、开环 residual、退化段 | 修合同或模型，不调融合权重 |
| Q3 Align | 可估计必要的 nuisance parameter；仍不可进 posterior | 参数是否可观、稳定且能泛化 | rank/condition、fit/held-out、分段一致性 | `pending` / `inconclusive`，不得伪初始化 |
| Q4 Constrain | 只在受控图或 frozen input 中影响 posterior | residual、Jacobian、噪声与作用方向是否正确 | 固定图 oracle、单因子冲突实验、active count | 修 factor 或不确定性模型 |
| Q5 Feedback | 可进入自然闭环；默认仍关 | 新 posterior 引起的下游反馈是否可接受 | 与 Q0 对照的轨迹、调度、lifecycle、失败段 | 保持默认关，定位最早分叉 |
| Q6 Default | 可成为默认路径 | 是否在承诺的场景内稳定优于或不劣于 baseline | 多场景门、成本门、回滚开关、错误语义 | 不推广；保留已证明的较低层能力 |

阶段编号不是产品架构。实验脚手架可以删除，已证明的合同和测试留下；不要把整套阶梯做成
一套长期 parallel pipeline。

## 5. 每一阶段的工作协议

进入一阶段前：

1. 写出一个可证伪的问题，而不是“把功能接上”。
2. 冻结 control 的 commit、canonical config、数据范围、随机性和产物指纹。
3. 列出本阶段唯一新增的因果边，以及刻意冻结的其他变量。
4. 在查看下一阶段结果前，预先写明机制 oracle、通过门、失败语义和最小支持量。
5. 标明 GT、未来数据、人工 schedule、fixed graph 等实验特权。

完成一阶段时：

1. 证明新增代码实际执行，报告 eligible、attached、skipped 及 skip reason；“运行成功”不等于机制生效。
2. 同时保留正例、边界例、负例和退化例，不能只看聚合均值。
3. 记录配置快照、输入范围、产物和命令，使结论可复现。
4. 只得出本阶段证据允许的结论，不把 offline oracle 宣称成 online capability。
5. 若失败，保存负结果并停止扩权；不要在同一轮顺手放宽门或新增补偿机制。

## 6. Oracle、指标与四种结论

机制 oracle 必须尽量独立于被测实现。例如用解析运动验证预积分，用数值微分验证
Jacobian，用 held-out 区间验证对齐，而不是用同一段数据既拟合又宣布正确。

终局指标是 guardrail，不是机制 oracle。结果按两条轴解释：

| 机制证据 | 产品指标 | 结论 |
|---|---|---|
| 通过 | 通过 | 可进入下一资格阶段；尚不能越级宣称 production 成立 |
| 通过 | 失败 | 机制可能正确，但 interaction、权重、拓扑或反馈失败；默认保持关 |
| 失败 | 通过 | 可能是偶然甜点或误差抵消；拒绝推广 |
| 失败 | 失败 | 当前假设被否决，记录最早失败层 |

证据覆盖不足、激励不足或参数不可观时，结论是 `inconclusive`，不是 pass，也不是用默认值
继续运行。

## 7. 参数、噪声与实验特权

- 物理标定、随机噪声、模型误差和产品决策阈值是四种不同对象，不混成一个“可调参数”。
- covariance 表达可信的不确定性，不负责掩盖时间、坐标系、bias 或模型错误。
- residual 存在均值、轴向结构、速度相关或时间相关时，先修系统误差；不得靠增大 covariance
  把新信息变成名义开启、实际无效。
- 参数估计必须先检查 observability、rank、condition 和分段稳定性，再讨论数值大小。
- 使用 GT、整段未来数据或同一序列的 held-out 段可以构造 qualification oracle，但必须写入
  产物元数据；由此得到的参数不得冒充在线初始化，也不足以支持默认开启。
- 调参必须基于已通过的机制层，并预注册搜索空间和选择指标；不得在终局指标下降后无边界扫参。

## 8. 失败语义

不同失败需要不同处理：

- 合同破坏、非有限值、坐标系或单位不明：hard failure，不能静默关闭新路径后报告成功。
- 局部缺测或 gap：只跳过受影响的约束，保留原因、数量和时长。
- 激励或覆盖不足：`pending` / `inconclusive`，保持旧路径。
- 初始化未取得资格：继续运行已验证的 baseline，不制造假的 initialized 状态。
- 机制通过但闭环回归：保持新路径默认关，比较最早发生变化的 state、schedule 和 lifecycle。

失败记录也是完整交付：它应说明被否决的假设、证据范围、最早分叉点，以及下一次实验只需改变
哪一条因果边。

## 9. 模块边界

- 数据合同归拥有数据的模块；核心机制藏在已有 deep module / PIMPL 后，不为实验泄漏 public API。
- dump、两趟运行、frozen input、报告与 benchmark 编排放 composition root 或一次性实验工具。
- synthetic / fixed-graph harness 属于测试资产，不演变成第二套产品 estimator。
- 只有稳定、跨调用方的语义才进入公共配置和长期 artifact schema；实验字段优先放独立产物。
- 每进入一层，只实现该层需要的最小 seam；下一层的真实需求出现后再扩展。

## 10. VIO 映射示例

从 VO 接入 IMU 时，建议按以下问题推进，而不是以“实现 VIO”为一个任务：

1. IMU packet 的时间区间、单位、frame 和 gap 合同是否成立，IMU-off 是否等价于 VO？
2. 已知 motion / bias 的 synthetic 输入上，gyro 开环积分是否正确？
3. offline 是否存在一个可观、稳定，并能通过 held-out 的 gyro bias alignment？
4. 不用未来数据或 GT 时，同一 bias 能否在线稳定初始化？offline pass 不授予这项资格。
5. 固定 bias 的 rotation factor 是否只沿预期方向改变一个受控图？
6. 进入自然 VO 闭环后，最早在哪里改变 KF、PnP、cull 或 re-anchor？
7. velocity、gravity、accelerometer bias 各自满足什么初始化与 observability 条件？
8. 完整 `X/V/B` posterior 在多序列上过门后，才讨论默认 VIO。

某一步失败就停在上一层仍正确的产品上。这样“IMU 接入后 ATE 变差”会被拆成一个可定位的
层级失败，而不是再次变成 covariance 扫参问题。

## 11. Slice 设计检查表

- [ ] control 的版本、配置、数据与产物已冻结。
- [ ] 本片只有一个新的可证伪因果问题。
- [ ] 时间、单位、frame、方向、区间和 gap 语义已写明。
- [ ] 有独立于终局指标的 mechanism oracle。
- [ ] fit 与 held-out 分开；实验特权已标注。
- [ ] residual、Jacobian、covariance 和 active coverage 可观测。
- [ ] fixed-input 与 natural feedback 的结论分开。
- [ ] pass、fail、inconclusive 及停止条件在运行前写明。
- [ ] 机制门与产品门分开，默认开启条件单独定义。
- [ ] 负结果也有可复现产物和下一步边界。
