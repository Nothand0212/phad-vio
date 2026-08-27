# Agent 工作偏好

本文档描述当前约定，不是绝对约束，会随项目开发修订。

权威正文；根 [`AGENTS.md`](../../AGENTS.md) 只作摘要索引。Git / C++ 细则见
[`git-workflow.md`](git-workflow.md)、[`cpp-naming.md`](cpp-naming.md)、
[`cpp-style.md`](cpp-style.md)。设计对齐见 [`design-alignment.md`](design-alignment.md)。

## 语言与文档

- 讨论、评审、面向 agent 的文档以**中文**为主；专有名词、技术术语与 code
  identifiers 保留英文。
- 跨库规则持久化到 `docs/agents/`，并在根 `AGENTS.md` 用指针链接。
- **模块作用域**约定写在对应目录 `AGENTS.md`，由根索引链接；勿把库细节堆进根文件。
- `phad/` 下每个库目录都要有 `README.md`（职责边界、文件布局、数据流、格式合同），
  顶部声明「本文档描述当前约定，不是绝对约束，会随项目开发修订」；`apps/`、
  `tests/` 与 adapter 子目录不单独写。

## 确认节奏

- 每个 milestone 与 vertical slice 开工前先逐项对齐：一次只问一个问题、给 A/B/C
  选项并标明推荐；默认先落地推荐项，验收不够再切备选。
- 设计稿分段确认后再动手：模块边界 → 数据流 → 错误与诊断 → 测试与验收。
- 底层代码已演进时，勿直接执行基于旧代码的设计/计划，应重新对齐或重写。
- 写正式 spec 或实施计划前先建 GitHub issues。

## 调研与定案

- 重大设计决策前先检索开源与一手来源（GTSAM examples、Kimera-VIO、ORB-SLAM3、
  VINS-Fusion，以及教学参考如 slambook2），把对照笔记写进 `docs/research/` 再定方案。
- 开源多为研究型代码，对照后以工程判断定案，勿盲抄其权宜实现。
- 对比既有 agent/他人调研稿时，先立足当前进度与一手来源独立分析，再对照，勿把
  调研稿当绝对准则直接综写。
- 审阅设计文档时直接指出问题并与用户讨论改法，不擅自定稿。
- 排障时先对照开源实现与本地清单，区分数据损坏与 loader 合同过严，勿直接断定
  数据源损坏。

## 工程取向

- 模块边界优先低耦合、高内聚、深模块与单向依赖；benchmark/编排放 composition
  root（见 [`apps/AGENTS.md`](../../apps/AGENTS.md)），不反向注入 frontend/estimator。
- C++ 的 go-to-definition 与 references 优先使用 clangd；禁用 Microsoft C/C++
  IntelliSense 以避免冲突。
- C++ identifiers 宜短，使用常见领域缩写（`imu`、`acc`、`gyr`、`nd`、`rw`、`fx`）；
  细则见 [`cpp-naming.md`](cpp-naming.md)。

## Git 与里程碑

- 短生命周期分支、`--no-ff` 合入、commit subject 带 issue 号、默认不 push：见
  [`git-workflow.md`](git-workflow.md)。
- EuRoC / milestone baseline 文档须含可复现的运行时参数快照（取自对应 run 的
  `meta.json`：`config` / `config_canonical_text`，与 `config_hash` 同源），不能
  只写 hash 名；跨切片时标出相对上一基线的增量键。
- 验收不够或续片选刀前先短诊断失败模式再按诊断优化；消化已知发散债时，编码后
  先跑诊断序列（如 MH_05），不够则停、不扩全序列。

## 文档分层

- 文档地图与命名见 [`../README.md`](../README.md)。
- 调研纪律见 [`research-and-planning.md`](research-and-planning.md)。
