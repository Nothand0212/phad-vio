# `phad::frontend` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- 负责 feature tracking；**不**做关键帧决策、不产出位姿、不定义 `KeyframeMeasurement`
- 输入假定为校正后 `StereoFrame` + `RectifiedStereoCalibration`
- `StereoTracker` 用 PIMPL 藏 OpenCV；public header 只用 Eigen / POD；OpenCV PRIVATE
- `dropTracks(span<const LandmarkId>)`：按 id 擦除 LiveTrack；未知 id 忽略；**不**改 `next_id`；空 span no-op。无 estimator 知识
- `FrameTracks` → `KeyframeMeasurement` 的组装在 `apps/stereo_vo_glue.hpp`，不在本库
