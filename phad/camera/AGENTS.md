# `phad::camera` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- 立体校正归本库：`StereoRectifier` 产出 `RectifiedStereoCalibration`；frontend 只消费校正后帧
- `StereoRectifier` 接收同模型的 radtan 或 equidistant 双目标定；左右 calibrated raw image size 必须一致
- raw stereo 支持同类型单通道 `uint8` / `uint16`；以 native depth remap，`uint16` remap 后逐像素 `>> 8` canonicalize 为 `uint8` 输出
- `StereoRectifier` 用 PIMPL 藏 OpenCV；public header 不出现 OpenCV，暴露既有 typed `CameraModelResult`；OpenCV PRIVATE 链接
- `create()` / `rectify()` 的合同错误使用 `CameraModelError`：输入 model、size、shape、pixel type 或 typed span 不符为 `kOutsideModelDomain`，OpenCV 与数值阶段为 `kNumericalFailure`，`detail` 保留 stage、side 与 library cause
- 估计器外参 `body_P_sensor` 必须用 `T_B_left_rectified()`，勿用未校正 `T_B_left`
