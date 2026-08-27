# 调研纪律与计划落盘

本文档描述当前约定，不是绝对约束，会随项目开发修订。

文档地图见 [`../README.md`](../README.md)。偏好正文见 [`preferences.md`](preferences.md)。

## 本仓调研

新调研写入 [`../research/`](../research/)，命名 `YYYY-MM-DD-<kind>-<topic>.md`：

| 种类前缀 | 适用 |
|---|---|
| `opensource-` | 开源对照、`*-comparison` |
| `note-` | 设计笔记、诊断、probe、叙事 baseline、result、postmortem、handoff、attribution |

日期取首次落库日，修订不改名。可复现身份写正文元信息。

卡住时先对照开源项目的 fix 历史，再提方案；证据不可达时显式标注推断，提请用户判断。

## 与其他目录的分工

| 问题 | 目录 |
|---|---|
| 查到了什么 | `docs/research/` |
| 为什么选 X | `docs/adr/` |
| 系统是什么 | `docs/design/` |
| 这一片必须交付什么 | `docs/specs/` |
| 怎么执行 | `docs/plans/` |
| 跑出了什么数字 | `docs/benchmark/` |

历史切片 `*-design.md` 留在 `research/`。数字账本权威在 `benchmark/`；research
里的叙事 baseline / results 指回对应 checkpoint。

## 计划落盘

- 确认后的实施计划写入 [`../plans/`](../plans/)，命名与格式见该目录 README
  （`YYYY-MM-DD_<slug>_<plan_id>.plan.md`）。
- 该片的验收合同写入 [`../specs/`](../specs/)。
- 计划必须含可验证验收，否则视为未完成设计。
