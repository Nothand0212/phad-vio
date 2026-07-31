# `phad::frontend` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- 负责 feature tracking；**不**做关键帧决策、不产出位姿、不定义 `KeyframeMeasurement`
- 输入假定为校正后 `StereoFrame` + `RectifiedStereoCalibration`
- `StereoTracker` 用 PIMPL 藏 OpenCV；public header 只用 Eigen / POD；OpenCV PRIVATE
- `FrameTracks` → `KeyframeMeasurement` 的组装在 `apps/stereo_vo_glue.hpp`，不在本库
