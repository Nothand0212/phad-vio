# `phad::frontend` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- 负责 feature tracking；**不**做关键帧决策、不产出位姿、不定义 `KeyframeMeasurement`
- 输入假定为校正后 `StereoFrame` + `RectifiedStereoCalibration`
- `StereoTracker` 用 PIMPL 藏 OpenCV；public header 只用 Eigen / POD；OpenCV PRIVATE
- `dropTracks(span<const LandmarkId>)`：按 id 擦除 LiveTrack；未知 id 忽略；**不**改 `next_id`；空 span no-op。无 estimator 知识
- `markEvictable(span<const LandmarkId>)`：标记 LiveTrack 可被 GFTT 懒腾槽挤出；未知 id 忽略；**不**立即删除；空 span no-op。占用预算按 non-evictable 计；缺槽时按 id 升序驱逐，再 `goodFeaturesToTrack`。`FrameStats.evicted` 记本帧驱逐个数
- `FrameTracks` → `KeyframeMeasurement` 的组装在 `apps/stereo_vo_glue.hpp`，不在本库
