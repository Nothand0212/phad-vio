# 实施计划约定

本目录存放可入库的实施计划（implementation plan）。Cursor 侧 `.cursor/plans/`
是编辑与执行时的工作副本；稳定后按本约定落到 `docs/plans/`，便于版本管理与
跨会话复用。

## 文件命名

```text
YYYY-MM-DD_<slug>_<plan_id>.plan.md
```

| 段 | 规则 | 示例 |
|---|---|---|
| `YYYY-MM-DD` | 计划落库日期（ISO） | `2026-07-30` |
| `slug` | 小写、`snake_case`；用英文或里程碑代号描述主题；不含空格 | `m2.1_eval_visualization_baseline` |
| `plan_id` | 8 位十六进制，与 Cursor 计划 id 一致（文件名末尾那段） | `d818d653` |
| 后缀 | 固定为 `.plan.md` | |

完整示例：

```text
2026-07-30_m2.1_eval_visualization_baseline_d818d653.plan.md
```

约定：

- 一个 Cursor 计划对应一个落库文件；`plan_id` 用于双向对照。
- 修订同一计划时**原地更新**该文件，不要另起一个新 `plan_id` 文件，除非范围
  已实质变成另一个计划。
- 历史/已失效计划可在正文开头加一行状态说明（例如「历史计划，接口部分已失效」），
  文件名不必改。

## 文件格式

对齐 [Cursor 计划文件](https://docs.cursor.com) 的结构：**YAML frontmatter + Markdown
正文**。Frontmatter 供工具解析 todos；正文给人与 agent 阅读。

### Frontmatter（必填）

```yaml
---
name: <短标题>
overview: <一两句总览：要交付什么、本次切哪一片>
todos:
  - id: <kebab-case-id>
    content: <可执行的一步：做什么 + 关键产物/验收>
    status: pending   # pending | in_progress | completed | cancelled
isProject: false
---
```

字段说明：

| 字段 | 要求 |
|---|---|
| `name` | 人类可读短标题；可与里程碑名一致 |
| `overview` | 范围边界写清楚（含「不做什么」若有必要） |
| `todos` | 有序任务列表；`id` 稳定、唯一；`content` 写动作而非愿望 |
| `todos[].status` | 仅用 `pending` / `in_progress` / `completed` / `cancelled` |
| `isProject` | 本仓库落库计划固定为 `false` |

### 正文（推荐结构）

Frontmatter 之后用 Markdown。推荐顺序（可按计划规模裁剪）：

```markdown
# <与 name 一致或略展开的标题>

## 切片划分

- **Slice ①（本次）**：…
- **Slice ②**：…
- **Slice ③**：…

## 已对齐的决策

| 决策点 | 结论 |
|---|---|
| … | … |

## 步骤 N：<主题>

具体改动、关键路径、接口草图、错误路径、测试与验收。

## 数据流（可选）

可用 mermaid 画模块边界与数据走向。

## 后续切片概要（可选）

未纳入本次 todos 的后续片，只写边界，不写实现细节。
```

正文原则：

- 写**可执行**步骤：改哪些文件、什么接口、如何验证。
- 决策表只记已对齐结论，不展开辩论过程。
- 代码示例保持最小可读草图；完整实现留给源码。
- 手工验证命令（如 `evo`）单独成节，并标明是否进 CI。

### 最小完整模板

````markdown
---
name: M2.1 评估可视化底座
overview: 实现真值加载 + TUM + SE3 对齐 + ATE；RPE 与可视化为后续片。
todos:
  - id: trajectory-type
    content: 新增 phad/common/trajectory.hpp（带校验工厂）
    status: pending
  - id: cmake-tests
    content: 接入 CMake 与 tests/eval 合成 fixture
    status: pending
isProject: false
---

# M2.1 评估与可视化底座

## 切片划分

- **Slice ①（本次）**：真值加载、TUM、关联、对齐、ATE、CLI、测试。
- **Slice ②**：RPE。
- **Slice ③**：实时面板与 Python 绘图。

## 已对齐的决策

| 决策点 | 结论 |
|---|---|
| 指标 | ATE 平移 RMSE 为主 |

## 步骤 1：…

…
````

## 与 Cursor 计划的关系

| 位置 | 用途 |
|---|---|
| `.cursor/plans/<name>_<plan_id>.plan.md` | Cursor UI / agent 执行时的工作副本 |
| `docs/plans/YYYY-MM-DD_<slug>_<plan_id>.plan.md` | 入库、评审、跨会话权威副本 |

落库时：保留同一 `plan_id`；补上日期与英文 `slug`；frontmatter 与 todos 与
Cursor 侧保持同步；正文可按本目录读者需要略作整理（例如补 roadmap 链接、
evo 命令清单），但不要改写已对齐决策。
