# Domain Docs

本项目采用 single-context domain documentation 布局。

## 探索代码前

依次读取与任务相关的：

- 根目录 `AGENTS.md`（权威文档与模块索引）；
- [`docs/README.md`](../README.md)（选对目录）；
- 根目录 `CONTEXT.md`（存在时）；
- `docs/adr/` 中影响当前工作区域的 ADR；
- `docs/design/` 中与当前工作相关的活架构（roadmap / architecture）。

文件不存在时静默继续，不把缺失本身当作错误，也不提前创建空文档。
`CONTEXT.md` 应在领域术语和规则真正明确后由 domain-modeling 工作流按需创建。

## 文件结构

```text
/
├── AGENTS.md                 # 根：skills、偏好摘要、现行事实、模块索引
├── CONTEXT.md                # 按需
├── docs/
│   ├── README.md             # 文档地图（问题 / 寿命 / 命名）
│   ├── AGENTS.md             # 交付流水线
│   ├── adr/
│   ├── design/               # 活架构（roadmap / architecture / conventions）
│   ├── specs/                # 已收口验收合同
│   ├── plans/
│   ├── research/
│   ├── benchmark/
│   └── agents/               # 跨库 agent 规则
├── phad/<lib>/AGENTS.md      # 模块作用域 agent 提示
├── apps/AGENTS.md
├── tests/AGENTS.md
└── scripts/AGENTS.md
```

模块细节写在对应目录 `AGENTS.md`，勿堆进根 `AGENTS.md`。

## 领域词汇

issue 标题、重构建议、假设和测试名称应使用 `CONTEXT.md` glossary 定义的
领域术语，避免漂移到 glossary 明确拒绝的同义词。

若需要的概念尚未定义，应判断：

1. 是否正在引入项目并不使用的语言；
2. 是否确实存在需要 domain-modeling 补充的领域缺口。

## ADR 冲突

若建议或实现与已有 ADR 冲突，必须明确指出冲突，不能静默覆盖既有决策。
冲突时以 `docs/design/architecture.md` 与 `docs/design/roadmap.md` 为准并回写 ADR。
