# Git 工作流

本文档描述当前约定，不是绝对约束，会随项目开发修订。

## 分支

- 在主 checkout 上使用短生命周期功能分支，不用 git worktrees（除非用户明确要求）。
- 按 vertical slice 逐片提交；commit subject 带对应 issue 号（如 `(#22)`）。
- 里程碑完成后本地 merge 到 `main`，再删除功能分支。
- 除非用户明确要求，否则不 push 到 remote。

## Merge 必须保留图（强制）

本地把功能分支合入 `main` 时，**必须**使用 `--no-ff`，生成 merge commit，使 Git Graph
上可见「分叉 → 合回」：

```bash
git checkout main
git merge --no-ff <feature-branch>
git branch -d <feature-branch>
```

禁止依赖默认的 fast-forward：即便 `main` 是功能分支的祖先，也要用 `--no-ff`，
否则图上只剩一条直线，看不出里程碑边界。

Merge commit message 沿用仓库既有风格，例如：

```text
Merge branch 'm3.2-stereo-pair-synchronizer'

M3.2 stereo pair synchronizer: ... (#22).
```

## 新里程碑分支

merge 完成后，从最新 `main` 开下一里程碑分支（如 `m3.3-vo-hardening`），再开始对齐与实施。
