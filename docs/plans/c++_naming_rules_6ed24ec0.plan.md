---
name: C++ Naming Rules
overview: 将你给出的四条 C++ 命名规范写成项目 Cursor rule，并按该规范把现有成员命名从尾缀 `_` 改为 `m_` 蛇形；函数统一为小驼峰。
todos:
  - id: add-naming-rule
    content: 新增 .cursor/rules/cpp-naming.mdc（函数小驼峰、变量蛇形、成员 m_ 蛇形、类型大驼峰）
    status: pending
  - id: rename-members
    content: 将类成员从尾缀 _ 改为 m_ 蛇形（euroc_dataset、dataset_error 等）
    status: pending
  - id: rename-functions
    content: 将 PascalCase 函数/方法改为小驼峰，并更新全部调用点与测试
    status: pending
  - id: verify-build
    content: 编译并运行相关测试确认无行为回归
    status: pending
isProject: false
---

# C++ 命名规范落地

## 规范（强制）

| 类别 | 格式 | 示例 |
|------|------|------|
| 函数 / 方法 | 小驼峰 | `loadStereo`, `parseTimestamp` |
| 局部 / 参数变量 | 蛇形 | `line_number`, `sensor_id` |
| 类成员变量 | `m_` + 蛇形 | `m_calibration`, `m_imu_measurements` |
| 类型（class/struct/enum/using） | 大驼峰 | `EurocDataset`, `DatasetErrorCode` |

补充约定（与现有代码一致，写入规则以免歧义）：
- 枚举值：`k` + 大驼峰（已有，如 `kRootNotFound`）
- 常量：`k` + 大驼峰或 `SCREAMING_SNAKE`（按现有风格保留 `kCameraHeader` 一类）
- 命名空间：蛇形（`phad::io::dataset`）

## 交付物

### 1. 新增 Cursor rule

创建 [`.cursor/rules/cpp-naming.mdc`](.cursor/rules/cpp-naming.mdc)：

- `globs: **/*.{h,hpp,c,cpp,cc,inl}`
- `alwaysApply: false`（打开 C/C++ 文件时生效）
- 内容为上表 + 正反例，并明确：**禁止**成员尾缀 `_`（如 `calibration_`），必须用 `m_`

这与用户级 C++ 规则中的 `m_` / camelCase 方法 / PascalCase 类型一致，用于项目内持久约束。

### 2. 对齐现有代码命名

主要改动点（当前不符）：

- 成员：[`euroc_dataset.hpp`](phad/io/dataset/euroc/euroc_dataset.hpp) 的 `calibration_` → `m_calibration` 等；[`dataset_error.hpp`](phad/io/dataset/dataset_error.hpp) 的 `value_` / `error_` → `m_value` / `m_error`
- 自由函数 / 方法：[`euroc_dataset.cpp`](phad/io/dataset/euroc/euroc_dataset.cpp) 中 PascalCase（如 `MakeError`, `ParseTimestamp`, `RemoveCarriageReturn`）→ 小驼峰（`makeError`, `parseTimestamp`, `removeCarriageReturn`）
- 方法：`Open` → `open`；保留已是小驼峰的 `loadStereo` / `imuMeasurements` 等
- 同步更新测试与调用处：[`euroc_dataset_test.cpp`](tests/io/dataset/euroc/euroc_dataset_test.cpp)、[`euroc_mh01_test.cpp`](tests/io/dataset/euroc/euroc_mh01_test.cpp)、[`phad_euroc_inspect.cpp`](apps/phad_euroc_inspect.cpp)

**不改**：类型名、枚举值、局部蛇形变量、文件名。

### 3. 验证

- 全量编译 + 跑现有 dataset / timestamp 测试，确认仅重命名、无行为变化。

## 范围边界

- 只做命名对齐 + Cursor rule，不做格式化大扫、不做逻辑重构。
- 与「大括号 Allman / 2 空格」等既有格式规则并存，本改动不碰缩进与括号风格。
