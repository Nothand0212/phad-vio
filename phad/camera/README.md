# `phad::camera` 运行时相机模型与立体校正

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录把 `sensor::CameraModelParameters` 变成可调用的几何模型：
`project`（相机系 3D → 像素）与 `backProject`（像素 → 单位方位）；并提供
整幅双目立体校正与校正后标定，供 frontend / M2.3 estimator 使用。
标定数值留在 `phad::sensor`；投影、畸变求解与 remap 留在这里。

CMake target：`phad_camera`（alias `phad::camera`），公开依赖 `phad::sensor`；
OpenCV（`core` / `imgproc` / `calib3d`）为 PRIVATE。

## 职责边界

| 做 | 不做 |
|---|---|
| 针孔 + radial-tangential / equidistant 的投影与反投影 | 持有图像或数据集路径 |
| 域外 / 非有限输入的显式错误 | 在线标定、外参优化 |
| `createCameraModel(...)` 工厂 | 特征跟踪 / 关键帧（归 `phad::frontend`） |
| `StereoRectifier` 的同模型双目校正、native-depth remap 与校正后标定 | 数据集 adapter 选择与 raw 图像 I/O |

## 文件布局

| 文件 | 作用 |
|---|---|
| `camera_model.hpp` / `.cpp` | `CameraModel` 接口、错误类型、具体模型与工厂 |
| `rectified_stereo_calibration.hpp` / `.cpp` | 校正后共享内参 + baseline + `T_B_left_rectified` |
| `stereo_rectifier.hpp` / `.cpp` | 同模型双目校正、native-depth remap（PIMPL 藏 OpenCV 表） |

## 合同约定

### 投影接口

```cpp
auto model = phad::camera::createCameraModel(parameters.modelParameters());
auto uv    = model->project(point_camera);   // CameraModelResult<Vector2d>
auto ray   = model->backProject(pixel);      // CameraModelResult<Vector3d>
```

- `CameraModel` 不可拷贝、不可移动；以 `unique_ptr` 持有。
- `project` / `backProject` 返回 `CameraModelResult`，失败码包括
  `kNonFiniteInput`、`kOutsideModelDomain`、`kNumericalFailure`。
- 反投影产出单位方位（bearing）；实现侧用重投影容差自检数值稳定性。

### 立体校正

```cpp
auto rectifier = phad::camera::StereoRectifier::create(stereo_imu_calib);
auto rectified = rectifier.value().rectify(raw_stereo_frame);
const auto& calib = rectifier.value().calibration();  // RectifiedStereoCalibration
```

- 左右相机必须同为 `PinholeRadialTangentialParameters` 或同为
  `PinholeEquidistantParameters`，并使用相同的正 calibrated raw image size。
- radtan 使用 `cv::stereoRectify(..., CALIB_ZERO_DISPARITY, alpha=0)` 与
  `cv::initUndistortRectifyMap()`；equidistant 使用
  `cv::fisheye::stereoRectify(..., CALIB_ZERO_DISPARITY, balance=0.0,
  fov_scale=1.0)` 与 `cv::fisheye::initUndistortRectifyMap()`。两条路径都以
  calibrated raw image size 生成 map，并用 `cv::remap(..., INTER_LINEAR,
  BORDER_CONSTANT)` 处理图像。
- raw stereo 两侧均接收单通道、相同 pixel type 的 `uint8` 或 `uint16`。`uint8`
  以 `CV_8UC1` remap 后直接输出；`uint16` 以 `CV_16UC1` 完成 native-depth
  remap，再逐像素执行
  `static_cast<std::uint8_t>(src >> 8)`，输出 canonical `uint8` stereo。
- OpenCV 的 `R,T` 约定为 left→right：`p_right = R * p_left + T`。
- `T_B_left_rectified = T_B_left * T_left_left_rect`，其中 `R = R1ᵀ`、`t = 0`。
- `StereoRectifier` 的 public surface 保持 `CameraModelResult`：model、size、
  frame shape、pixel type 与 typed pixel span 合同错误使用
  `kOutsideModelDomain`；OpenCV、rectification geometry 或校正标定数值错误使用
  `kNumericalFailure`，`detail` 带 stage、side 与原始 OpenCV cause（适用时）。
- public header 不出现 OpenCV；map、model dispatch 与 pixel canonicalization
  均由 PIMPL / `.cpp` 持有，OpenCV 维持 PRIVATE 链接。

### 与 `sensor` 的对应

| `sensor::CameraModelParameters` | 运行时模型 |
|---|---|
| `PinholeRadialTangentialParameters` | EuRoC 风格 radtan |
| `PinholeEquidistantParameters` | TUM VI / Kalibr equidistant |

工厂按 variant 分派；不支持的参数类型不应静默落到错误模型。

## 相关入口

| 位置 | 用途 |
|---|---|
| `phad::sensor::CameraParameters` | 标定输入 |
| `phad::frontend::StereoTracker` | 消费校正后帧与标定 |
| `tests/camera/` | 投影往返、恒等 remap、MH_01 行对齐 smoke |
