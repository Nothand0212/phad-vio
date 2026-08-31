# 薄而诚实的 Vertical Slice

本文档描述当前约定，不是绝对约束，会随项目开发修订。

始终推进**薄而诚实**的 vertical slice：足够小，可以快速进入真实反馈；足够诚实，
保留已知数学不变量、typed failure、rollback 和可审计证据。

最小不等于伪初始化、虚假成功或忽略已知错误。严谨也不等于在接触产品行为前穷尽
所有可能风险。正确性保护放进最小**产品闭环**；准备工作、诊断、protocol 或
uncertainty study 服务该闭环，不取代它。

本文只约束切片选择、验收与报告格式。commit / push / 远程实验 / 阶段授权仍以
当前用户指令为准。

相邻约定：

| 文档 | 回答的问题 |
|---|---|
| [`incremental-development.md`](incremental-development.md) | 怎么从无到有、从有到好 |
| [`evidence-gated-integration.md`](evidence-gated-integration.md) | 新信息何时有资格改变既有输出 |
| 本文 | 下一刀是否仍对准当前产品 RED，以及何时把工作移出关键路径 |

## 何时读

- 规划 milestone / vertical slice 的下一步
- 准备把 component、统计模型、covariance、diagnostics 或 qualification 放进关键路径
- 真实闭环失败后选下一刀

## 步骤

1. 填写[核心矛盾卡](#核心矛盾卡)。完成条件：能写出一条有证据支持的因果链
   `产品 RED → 最早失败边界 → 本轮唯一干预 → 预期 product endpoint 变化`。
   写不出时，只用现有日志、代码和 artifact 做一次最小诊断，或增加一个直接区分
   候选根因的局部 probe。
2. 若下一任务是 component、统计模型、covariance、diagnostics 或 qualification，
   先做[代理问题检查](#代理问题检查)。过不了则移入独立 research backlog，
   不作为当前 milestone 的 predecessor。
3. 选择[最小产品闭环](#最小产品闭环)，并按[风险分层](#风险分层)安置 hard gate
   与观察项。
4. 执行该闭环。每一步结束时按[步末报告](#步末报告)交代可观察变化。
5. 真实闭环失败时，按[失败后沿主导因果链迭代](#失败后沿主导因果链迭代)。
6. 外部行为达到本片验收条件后立即停止。后续 hardening、precision、统计
   calibration 和更宽泛场景，等新的真实失败证据再启动。

## 核心矛盾卡

采取任何设计、实现、诊断或实验动作前，先写清：

1. 当前 roadmap 出口是什么？
2. 用户能观察到的具体 RED 是什么？
3. 已有证据确认的最早失败边界在哪里？
4. 哪个机制对该 RED 的贡献最大？
5. 下一项改动通过什么因果链影响该 RED？
6. 本轮将触达哪个真实 product endpoint？
7. 哪些已知不变量必须作为 hard gate 保留？
8. 哪些不确定性可以先作为 diagnostics、rollback 或 failure bundle 观察？

写不出因果链时，只允许最小诊断或局部 probe。通用 runner、长期 schema、formal
protocol 或大规模 synthetic study 不在这一步的允许集里。

## 代理问题检查

下一任务若是某个 component、统计模型、covariance、diagnostics 或 qualification，
先回答：

- 有什么证据表明它是当前产品 RED 的**主导**原因，而不只是一个可能原因？
- 它的结果将在两个什么具体产品动作之间作出选择？
- 如果它 PASS，下一步是否立即进入 product seam？
- 如果它 FAIL，是否能直接排除一个产品方向？

「更容易隔离、形式上更严谨、未来可能有风险」不构成关键路径依据。给不出主导性
证据或产品决策分支时，把该任务移入独立 research backlog。

## 最小产品闭环

优先实现从真实 caller 到真实 output 的最短路径，并遵守现有 deep-module ownership。

同一个 slice 内同时交付：

- 必要的 interface / dataflow
- 当前行为所需的最小算法
- 已知时间、单位、坐标系和生命周期不变量
- typed failure 与原子 rollback
- 一个 deterministic oracle
- 一个 tight 真实失败样例
- 一个健康路径回归
- 一个目标 product run 或 prefix

component helper 在本 slice 结束时必须拥有真实 product caller。仅有 unit PASS、
artifact valid 或 protocol complete，不算 milestone 前进。

## 风险分层

| 类别 | 处理 |
|---|---|
| 已知数学 / 接口不变量 | 立即成为 hard gate |
| 已复现且影响当前产品出口的失败 | 成为本 slice 的测试和验收条件 |
| 合理但尚未观察到的风险 | 先用 diagnostics、rollback、opt-in failure bundle 观察 |
| 纯推测风险 | 记录到 backlog，等待真实证据 |

每个新增 gate 都必须注明：对应哪条现有失败证据；移除它会让当前验收如何失败。

没有稳定产品 consumer 的矩阵、posterior、fingerprint、研究字段和完整 replay
evidence，留在 opt-in artifact，不扩张 regular API / CSV schema。

数据流、state ownership、frame / sign、初始化闭合或 transaction 尚未正确时，
先修这些系统误差。threshold、covariance 和统计 budget 只有在基础模型正确、且
真实失败确实落在 uncertainty surface 后，才进入关键路径。

## 熔断连续准备工作

docs、research、diagnostics、fixture、runner、protocol 和 static review 都属于
**support work**。

一个 support-only step 完成后，下一步必须触达 product seam。不得连续安排两个
support-only step，除非同时满足：

- 明确把它们重新归类为独立 research milestone
- 说明其科学问题和停止条件
- 获得用户对该研究里程碑的单独授权

若 roadmap 出口没有任何变化，下一步只能选择：直接集成、删除或推迟当前机制，
或请求把工作转为独立研究。不得再增加准备层。

## 失败后沿主导因果链迭代

真实闭环失败时：

1. 找到相对 control 的最早分叉。
2. 按贡献量级识别主导失败。
3. 一次只提出一个可证伪假设。
4. 一次只改变一个因果边。
5. 重跑同一 tight loop。
6. 结果符合预测则继续；不符合则否决该假设。

## 完成定义

milestone 前进必须表现为外部行为变化，例如：

- 原来不能建立的 state / root 现在可以诚实建立
- 原来错误提交的 candidate 现在被正确拒绝
- 原来断裂的 trajectory 现在保持连续
- completion、coverage、ATE / RPE 或明确 lifecycle 指标得到可复现变化

测试数量、代码量、文档量、artifact 完整性、formal identity 或 validator PASS
都是支持证据，不单独构成产品完成。

## 步末报告

每一步结束时必须报告：

- roadmap 出口发生了什么可观察变化
- product caller 是否实际执行了新增机制
- tight RED 是改善、保持还是被反证
- 下一步唯一动作是什么

## 最终报告

**事实**

- 实际执行了什么
- product seam 是否被触达
- tight loop 与产品指标结果

**推断**

- 哪条因果假设得到支持或被否决
- 证据边界是什么

**产品进度**

- 哪个 roadmap 出口前进了
- 哪些限制仍被诚实保留

**下一步**

- 只提出一个最小、直接、可验证的动作
- 说明它如何作用于当前核心矛盾

## 历史校准

M4 的成功路径是：把正确性保护放进最小产品闭环，直接贯通单一 `VioEstimator` 的
`X/V/B`、raw IMU、PIM、graph、transaction、outage 与 eviction；以约 `+4.6k/-5.9k`
取得 clean EuRoC 11/11 checkpoint。

对照：M4.3 已通过真实闭环定位到 accel scale / tilt 混淆与 bias walk，但后续把
关键路径转到更容易隔离的 gyro-only Q1 / Q2 / Q3 与 formal qualification。该链约
`+25.8k` 行，长期没有 product caller，也没有推进完整 `X/V/B` 产品出口。

本校准只约束切片选择：资格阶梯仍按
[`evidence-gated-integration.md`](evidence-gated-integration.md) 执行；Q 阶段
本身不是 milestone 前进。
