# C++ 命名规范

| 类别 | 风格 | 示例 |
|------|------|------|
| 函数 / 方法 | camelCase | `takeStereo`, `parseTimestamp`, `open` |
| 局部变量 / 参数 | snake_case | `line_number`, `sensor_id` |
| 类 / 结构体成员变量 | `m_` + snake_case | `m_calibration`, `m_imu_measurements` |
| 类型（class / struct / enum / using） | PascalCase | `StereoImuDataset`, `DatasetErrorCode` |

## 补充约定

- 枚举值：`k` + PascalCase（如 `kRootNotFound`）
- 常量：`k` + PascalCase（如 `kCameraHeader`）
- 命名空间：snake_case（如 `phad::io::dataset`）

## 坐标变换记号例外

表示坐标系间刚体变换的标识符采用
`T_<target_frame>_<source_frame>`，作为普通 camelCase / snake_case
规则的受控例外。该变换把 `source_frame` 中表达的量转换到
`target_frame`：

```cpp
RigidTransform T_B_left_camera;
RigidTransform T_W_B;

const RigidTransform& T_B_left_camera() const;

RigidTransform m_T_B_left_camera;
RigidTransform m_T_B_right_camera;
```

- `T` 保持大写；`W`、`B` 等规范单字母 frame symbol 保持大写；
  `left_camera` 等描述性 frame token 使用 snake_case。
- 局部变量、参数和 accessor 可直接使用该记号；成员变量只在前面增加
  `m_`。
- target/source 顺序必须遵循 `docs/conventions.md`，不得使用
  `extrinsics`、`camera_transform`、`T_camera` 等无法从名称确定方向的写法。
- 该例外只适用于坐标变换记号；其他函数和变量仍遵循本文件的一般规则。

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
static DatasetResult<StereoImuDataset> Open(...);

// GOOD
DatasetError makeError(...);
static DatasetResult<StereoImuDataset> open(...);

// BAD — 方法使用 snake_case
bool has_value() const;

// GOOD
bool hasValue() const;
```

## 说明

- 不要重命名第三方 API（如 `std::optional::has_value`、`YAML::Node::IsSequence`）。
- Cursor 镜像：`.cursor/rules/cpp-naming.mdc`（须与本文档保持同步）。
