# `docs/` — agent 提示

本文档描述当前约定，不是绝对约束，会随项目开发修订。

根索引见 [`../AGENTS.md`](../AGENTS.md)。目录职责与命名见 [`README.md`](README.md)。
跨模块规则正文在 [`agents/`](agents/)。

## 标准交付流水线

```text
对齐问答（一次一问，A/B/C + 推荐）
  → 调研落盘     docs/research/（开源对照、诊断；种类前缀 opensource-/note-）
  → 决策与架构   docs/adr/ + docs/design/（为什么 vs 系统是什么）
  → spec 确认    docs/specs/；用户「同意/认可」后标注「已定稿」
  → 实施计划     docs/plans/（切片、验收、commit；含 plan_id）
  → 建 Issue     GitHub；vertical slice 开工前建票
  → 实现         按计划分片；commit subject 带 issue 号
  → 验收         specs 口径 + design/roadmap 出口
  → 回填         docs/benchmark/ checkpoint + 设计文档更新 + AGENTS 修订（若约定变化）
```

细则：[`agents/design-alignment.md`](agents/design-alignment.md)、
[`agents/research-and-planning.md`](agents/research-and-planning.md)、
[`agents/preferences.md`](agents/preferences.md)。

## 目录索引

| 路径 | 职责 |
|---|---|
| [`README.md`](README.md) | 文档地图与命名规则 |
| [`agents/preferences.md`](agents/preferences.md) | 工作偏好权威正文 |
| [`agents/git-workflow.md`](agents/git-workflow.md) | Git 工作流（短分支、`--no-ff`、会话约定） |
| [`agents/design-alignment.md`](agents/design-alignment.md) | 定稿标记、实现对齐、ADR 冲突 |
| [`agents/research-and-planning.md`](agents/research-and-planning.md) | 调研纪律与计划落盘 |
| [`agents/cpp-style.md`](agents/cpp-style.md) / [`cpp-naming.md`](agents/cpp-naming.md) | C++ 风格与命名 |
| [`adr/`](adr/) | 为什么选 X |
| [`design/`](design/) | 系统是什么 |
| [`specs/`](specs/) | 这一片必须交付什么 |
| [`plans/`](plans/) | 怎么执行 |
| [`research/`](research/) | 查到了什么 |
| [`benchmark/`](benchmark/) | 跑出了什么数字 |
| [`learn/`](learn/) | 这一环怎么走通（教学叙事） |

## 禁止

- 跳过逐条确认直接写实现代码
- 把聊天结论当作唯一真相而不落盘
- 未确认事项标注「已定稿」
- 自行发明已知卡点的解法而不对照开源证据
