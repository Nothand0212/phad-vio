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

## 简洁与缩写

优先短而清楚的名字；不要为了“自解释”把标识符拉得很长。

- **能短则短**：去掉对上下文无贡献的词（单位全文、重复角色词、冗余修饰）。
- **流行缩写优先**：机器人 / VIO / SLAM / 计算机视觉领域里通用、文献与代码里常见的缩写直接用，不要再写全称。
  例如：`imu`、`acc` / `gyr`、`nd`（noise density）、`rw`（random walk / bias random walk）、`fx` / `fy` / `cx` / `cy`、`T_B_*`。
- **不要自造缩写**：只有本文件、本 PR 或口头约定才懂的缩写禁止；宁可稍长，也不要歧义。
- **单位放注释，不塞进长名字**：物理单位用简短 header / 成员注释说明即可，避免
  `accelerometer_noise_density_mps2_per_sqrt_hz` 这类标识符。需要消歧时可用短后缀
  （如 `rate_hz`、`accel_mps2`、`gyro_radps`），不要把完整量纲短语嵌进名字。
- **对外格式键保持原样**：EuRoC / TUM-VI 等数据集 YAML、CSV 列名按格式原文读写；缩写只用于内部 API、成员与 `field_path`。
- **同一概念全仓一致**：选定缩写后，参数、成员、accessor、错误路径用同一套词根（如 `acc_nd` / `m_acc_nd` / `accNd()` / `imu.acc_nd`）。

```cpp
// BAD — 过长，单位全文塞进标识符
double accelerometer_noise_density_mps2_per_sqrt_hz;
double accelerometerNoiseDensityMps2PerSqrtHz() const;

// GOOD — 领域缩写 + 注释承载单位
double m_acc_nd;  // m/s²/√Hz
double accNd() const noexcept;
```

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
