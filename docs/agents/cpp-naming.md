# C++ 命名规范

| 类别 | 风格 | 示例 |
|------|------|------|
| 函数 / 方法 | camelCase | `loadStereo`, `parseTimestamp`, `open` |
| 局部变量 / 参数 | snake_case | `line_number`, `sensor_id` |
| 类 / 结构体成员变量 | `m_` + snake_case | `m_calibration`, `m_imu_measurements` |
| 类型（class / struct / enum / using） | PascalCase | `EurocDataset`, `DatasetErrorCode` |

## 补充约定

- 枚举值：`k` + PascalCase（如 `kRootNotFound`）
- 常量：`k` + PascalCase（如 `kCameraHeader`）
- 命名空间：snake_case（如 `phad::io::dataset`）

## 禁止写法

```cpp
// BAD — 成员使用尾缀下划线
EurocCalibration calibration_;
std::optional<T> value_;

// GOOD
EurocCalibration m_calibration;
std::optional<T> m_value;

// BAD — 函数使用 PascalCase
DatasetError MakeError(...);
static DatasetResult<EurocDataset> Open(...);

// GOOD
DatasetError makeError(...);
static DatasetResult<EurocDataset> open(...);

// BAD — 方法使用 snake_case
bool has_value() const;

// GOOD
bool hasValue() const;
```

## 说明

- 不要重命名第三方 API（如 `std::optional::has_value`、`YAML::Node::IsSequence`）。
- Cursor 镜像：`.cursor/rules/cpp-naming.mdc`（须与本文档保持同步）。
