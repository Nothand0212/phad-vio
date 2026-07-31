# `docs/` — agent 提示

## 约定

- 调研笔记与设计稿放 `docs/research/`，文件名**不带**日期前缀（如 `m2.3-vo-backend-design.md`）；日期写在正文元信息
- 工作流：对齐问答 → research 笔记 → design 文档 → `docs/plans/` 实施计划 → 分片实现
- 实施计划命名与格式见 `docs/plans/README.md`（`YYYY-MM-DD_<slug>_<plan_id>.plan.md` + YAML frontmatter）
- `.cursor/plans/` 是工作副本，`docs/plans/` 是入库权威副本，用同一 `plan_id` 对照
- 面向 agent 的持久规则优先写 `docs/agents/`（跨库）或各模块目录下的 `AGENTS.md`（模块作用域），并在根 `AGENTS.md` 链接
- `docs/architecture.md` 与 `docs/roadmap.md` 是模块边界与里程碑顺序的权威来源
