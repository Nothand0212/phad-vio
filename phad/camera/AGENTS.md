# `phad::camera` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- 立体校正归本库：`StereoRectifier` 产出 `RectifiedStereoCalibration`；frontend 只消费校正后帧
- `StereoRectifier` 用 PIMPL 藏 OpenCV；public header 只用 Eigen 与 POD；OpenCV PRIVATE 链接
- 估计器外参 `body_P_sensor` 必须用 `T_B_left_rectified()`，勿用未校正 `T_B_left`
