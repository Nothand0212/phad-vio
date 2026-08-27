# docs

人与 agent 的文档入口。先按「回答什么问题」选目录，再打开具体文件。

本文档描述当前约定，不是绝对约束，会随项目开发修订。

| 目录 | 回答的问题 | 寿命 |
|---|---|---|
| [research/](research/) | 我们查到了什么（调研、开源对照、诊断、历史切片笔记） | 只追加 |
| [adr/](adr/) | 为什么选 X 而不是 Y | 落盘后不改正文；推翻则新 ADR supersede |
| [design/](design/) | 系统是什么（模块、数据流、坐标系、roadmap） | 持续修订 |
| [specs/](specs/) | 这一片必须交付什么（验收合同） | 开工时冻结口径 |
| [plans/](plans/) | 怎么执行这一片（切片、commit、交接） | 做完可归档 |
| [benchmark/](benchmark/) | 跑出了什么数字（行为 checkpoint） | 只追加，供进化对比 |
| [agents/](agents/) | agent 怎么工作 | 约定变化时回写 |

`spec` 不是架构。活架构在 `design/`；里程碑 / 切片验收合同在 `specs/`。历史切片
`*-design.md` 留在 `research/`（当时的笔记，不是活架构）。

操作索引（不迁入上表）：[`worktrees.md`](worktrees.md)、
[`historical-branch-evidence.md`](historical-branch-evidence.md)。

## 流水线

```text
对齐问答
  → research（开源对照、诊断）
  → adr + design（为什么 / 系统是什么）
  → specs（确认后冻结）
  → plans（切片与验收）
  → 实现
  → benchmark（行为 checkpoint）
benchmark 反证可触发 adr 回顾
```

细则见 [`AGENTS.md`](AGENTS.md)、[`agents/research-and-planning.md`](agents/research-and-planning.md)、
[`agents/design-alignment.md`](agents/design-alignment.md)。

## 命名

日期只标识**首次落库**的 Asia/Shanghai 日历日（`git log --diff-filter=A --follow --format=%as`），
不随修订改名。可复现身份（git commit、参数快照、数据集）写正文元信息。

| 种类 | 规则 | 例 |
|---|---|---|
| 活文档 | 稳定 ASCII 名，无日期 | `design/roadmap.md` |
| ADR | `NNNN-ascii-slug.md`（顺序号是决策身份） | `adr/0001-gtsam-vio-backend.md` |
| spec / research | `YYYY-MM-DD-<topic>.md` | `specs/2026-08-25-m4-minimal-full-state-vio.md` |
| plan | `YYYY-MM-DD_<slug>_<plan_id>.plan.md`（含 Cursor `plan_id`） | 见 [`plans/README.md`](plans/README.md) |
| benchmark | 见 [`benchmark/README.md`](benchmark/README.md)（commit + `config_hash`） | |

- `<topic>`：ASCII 小写字母、数字、连字符。
- `README.md` 与诊断产物不受此规则约束。
- 大体积 dump 不进 git，路径写在 benchmark 正文。

research 的 `<topic>` 用种类前缀：

- `opensource-`：`*-open-source-refs.md`、`*-comparison.md`
- `note-`：其余（design / diagnosis / probe / baseline 叙事 / result / postmortem / handoff / attribution 等）
