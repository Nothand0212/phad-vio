# C++ 代码风格

格式以仓库根目录 `.clang-format` 为准；本文提炼 agent 必须遵守的约定。命名见 `docs/agents/cpp-naming.md`。

## 缩进与空白

- 缩进：**2 个空格**，禁止 Tab
- 命名空间内代码需要缩进（`NamespaceIndentation: All`）
- 续行缩进：4 个空格
- 最多保留 2 个连续空行
- 控制语句关键字与括号之间有空格：`if ( cond )`、`while ( true )`
- 圆括号、方括号内侧有空格：`( foo )`、`[ i ]`（与 `.clang-format` 一致）

## 大括号（Allman）

所有大括号另起一行，包括 class / struct / 函数 / if / for / while / switch / namespace / enum；`else` / `catch` 前的 `}` 也换行。

```cpp
// GOOD
if ( condition )
{
  doSomething();
}
else
{
  handleElse();
}

// BAD — K&R / 同行大括号
if ( condition ) {
  doSomething();
}
```

## 循环

- **禁止** `for ( ;; )`；无限循环写 `while ( true )`
- 有明确边界时优先范围 for 或带条件的 `for` / `while`

```cpp
// BAD
for ( ;; )
{
  if ( done )
  {
    break;
  }
}

// GOOD
while ( true )
{
  if ( done )
  {
    break;
  }
}
```

## 指针与引用

- `*` / `&` 靠右对齐类型侧空格风格以 clang-format 为准：`Type* ptr`、`Type& ref`（`PointerAlignment: Right`）

## 说明

- 提交前对改动文件跑 clang-format；不要手改与格式化结果冲突的风格。
- Cursor 镜像：`.cursor/rules/cpp-style.mdc`（须与本文档保持同步）。
