# `phad::camera` 运行时相机模型

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录把 `sensor::CameraModelParameters` 变成可调用的几何模型：
`project`（相机系 3D → 像素）与 `backProject`（像素 → 单位方位）。
标定数值留在 `phad::sensor`；投影算法与畸变求解留在这里。

CMake target：`phad_camera`（alias `phad::camera`），公开依赖 `phad::sensor`。

## 职责边界

| 做 | 不做 |
|---|---|
| 针孔 + radial-tangential / equidistant 的投影与反投影 | 持有图像或数据集路径 |
| 域外 / 非有限输入的显式错误 | 在线标定、外参优化 |
| `createCameraModel(...)` 工厂 | 立体校正、undistort 整图管线（未实现则不算本库职责） |

## 文件布局

| 文件 | 作用 |
|---|---|
| `camera_model.hpp` / `.cpp` | `CameraModel` 接口、错误类型、具体模型与工厂 |

## 合同约定

### 接口

```cpp
auto model = phad::camera::createCameraModel(parameters.modelParameters());
auto uv    = model->project(point_camera);   // CameraModelResult<Vector2d>
auto ray   = model->backProject(pixel);      // CameraModelResult<Vector3d>
```

- `CameraModel` 不可拷贝、不可移动；以 `unique_ptr` 持有。
- `project` / `backProject` 返回 `CameraModelResult`，失败码包括
  `kNonFiniteInput`、`kOutsideModelDomain`、`kNumericalFailure`。
- 反投影产出单位方位（bearing）；实现侧用重投影容差自检数值稳定性。

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
| `tests/camera/` | 投影往返与错误路径单测 |
