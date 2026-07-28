# Issue tracker: GitHub

本项目的 issues 和 PRDs 存储在 GitHub Issues 中。所有操作使用 `gh` CLI。

当前项目尚未初始化为 Git 仓库，也没有 GitHub remote。在执行 issue 操作前，
必须先完成仓库初始化并配置 GitHub remote；不得猜测目标仓库。

## Conventions

- 创建：`gh issue create --title "..." --body "..."`
- 查看：`gh issue view <number> --comments`
- 列举：`gh issue list --state open --json number,title,body,labels,comments`
- 评论：`gh issue comment <number> --body "..."`
- 添加标签：`gh issue edit <number> --add-label "..."`
- 删除标签：`gh issue edit <number> --remove-label "..."`
- 关闭：`gh issue close <number> --comment "..."`

目标仓库由 `git remote -v` 确定。没有 remote 时停止写入并报告缺失配置。

## Pull requests as a triage surface

**PRs as a request surface: no.**

默认不将外部 Pull Request 纳入 issue triage 队列。

GitHub 的 issue 和 Pull Request 共用编号空间。遇到裸编号 `#42` 时，
先执行 `gh pr view 42`，失败后再执行 `gh issue view 42`。

## Skill operations

- “publish to the issue tracker”：创建 GitHub issue。
- “fetch the relevant ticket”：执行 `gh issue view <number> --comments`。
- 多行 issue body 使用 heredoc，避免转义破坏内容。

## Wayfinding operations

- Map：使用带 `wayfinder:map` 标签的单一 issue。
- Child ticket：优先使用 GitHub sub-issue；不可用时，在 map 中维护任务列表，
  并在 child body 顶部写入 `Part of #<map>`。
- Child 标签：`wayfinder:research`、`wayfinder:prototype`、
  `wayfinder:grilling` 或 `wayfinder:task`。
- Blocking：优先使用 GitHub native issue dependencies；不可用时，在 child
  body 顶部维护 `Blocked by: #<n>`。
- Claim：`gh issue edit <n> --add-assignee @me`。
- Resolve：先评论结果，再关闭 issue，最后更新 map 的 Decisions-so-far。

