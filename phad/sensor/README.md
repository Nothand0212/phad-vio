# `phad::sensor` 传感器合同

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录定义与数据来源无关的测量、图像与标定类型。`phad::io` 的 adapter
把 EuRoC / TUM VI 等格式规范化后填入这些类型；frontend / estimator 只依赖
本库，不依赖具体数据集字段名。

CMake target：`phad_sensor`（alias `phad::sensor`），公开依赖 Eigen。

## 职责边界

| 做 | 不做 |
|---|---|
| IMU / 双目测量、图像缓冲、标定 POD | 从磁盘读 CSV/YAML（归 `phad::io`） |
| 标定参数校验工厂与 `CalibrationError` | 投影 / 反投影几何（归 `phad::camera`） |
| `CameraId`、`ImageFrameEvent`（未配对单路） | 时间同步、packet 构造（归 `phad::sync`） |
| `RigidTransform`、`StereoImuCalibration` | 数据集路径 / adapter |

## 文件布局

| 文件 | 作用 |
|---|---|
| `imu_measurement.hpp` | 时间戳 + `accel_mps2` / `gyro_radps`（`std::array`） |
| `stereo_frame.hpp` | `Image`（uint8/uint16）+ 已配对 `StereoFrame` |
| `camera_id.hpp` | `CameraId`、`ImageFrameEvent`（单路、未配对） |
| `camera_parameters.hpp` | 针孔 + radtan / equidistant 参数 |
| `imu_parameters.hpp` | 采样率与噪声密度 / 随机游走 |
| `rigid_transform.hpp` | 校验过的 4×4 刚体变换 |
| `stereo_imu_calibration.hpp` | 左右相机 + IMU + `T_B_*` |
| `calibration_error.hpp` | `CalibrationError` / `CalibrationResult` |

## 合同约定

### 类型形态

- **测量叶子**（`ImuMeasurement` 等）保持 POD + STL（`std::array`），便于序列化
  与简单拷贝。
- **标定与几何**已用 Eigen（`RigidTransform` 内部 `Isometry3d`）；Eigen 是本库
  的 public 依赖。
- 内部 API 用领域缩写（`accNd`、`gyrRw`、`fxPixels`）；单位写在注释里。
  外部数据集 YAML/CSV 键名保持原文完整拼写，转换在 `io` adapter 内完成。

### `Image` / `StereoFrame`

- 保留原始无符号灰度深度：`PixelType::kUint8` 或 `kUint16`。
- 访问像素须用与类型一致的 `pixels<T>()`；类型不符返回 `nullopt`，禁止把
  16-bit 静默截断为 8-bit。

### 标定工厂

- `CameraParameters`、`ImuParameters`、`RigidTransform`、`StereoImuCalibration`
  均经 `create(...)` 校验（有限、正值、旋转合法、双目基线非零等）。
- 失败带 `field_path`，便于 adapter 映射回 YAML 字段。

### 相机模型参数 vs 运行时模型

| 层 | 类型 | 职责 |
|---|---|---|
| `sensor` | `CameraModelParameters`（variant） | 可序列化的标定数值 |
| `camera` | `CameraModel` | `project` / `backProject` |

## 相关入口

| 位置 | 用途 |
|---|---|
| `phad::io` | 填充本目录类型 |
| `phad::camera` | 由 `CameraModelParameters` 构造运行时模型 |
| `tests/sensor/` | 参数校验与 header 合同单测 |

坐标系与 SI 单位见 [`docs/conventions.md`](../../docs/conventions.md)。
