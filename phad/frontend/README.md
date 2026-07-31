# `phad::frontend` 双目前端

本文档描述当前约定，不是绝对约束，会随项目开发修订。

本目录在校正后图像上做左目时序 LK 跟踪与右目每帧重匹配，产出带
`LandmarkId` 的 track 表与帧级指标。不判断关键帧、不产出位姿、不定义
`KeyframeMeasurement`。M2.3 由 `apps/stereo_vo_glue.hpp` 消费
`FrameTracks`，仍不在 frontend 内定义估计器测量类型。

CMake target：`phad_frontend`（alias `phad::frontend`），公开依赖
`phad::common`、`phad::camera`；OpenCV（`core` / `imgproc` / `video`）为
PRIVATE。

## 职责边界

| 做 | 不做 |
|---|---|
| GFTT 检测与按 track 长度涂 mask 补点 | 关键帧决策 |
| 左目时序 LK + 前后向一致性 | 位姿 / IMU 预测 |
| 右目每帧从当前左目重匹配与几何门限 | `KeyframeMeasurement` 合同 |
| 单调不复用的 `LandmarkId` 与 `FrameStats` | 无限累积历史观测 |

输入假定为 `StereoRectifier` 输出的校正后 `StereoFrame` 与
`RectifiedStereoCalibration`。

## 文件布局

| 文件 | 作用 |
|---|---|
| `stereo_tracks.hpp` | `LandmarkId`、`StereoStatus`、`TrackObservation`、`FrameStats`、`FrameTracks` |
| `stereo_tracker.hpp` / `.cpp` | `StereoTracker`（PIMPL 藏上一帧与 OpenCV 缓冲） |

## 数据流

```text
StereoFrame (rectified) ──► StereoTracker::process
                                │
                                ▼
                           FrameTracks
                     (observations + FrameStats)
                                │
              ┌─────────────────┴─────────────────┐
              ▼                                   ▼
   phad_stereo_frontend_probe         apps/stereo_vo_glue.hpp
        CSV + summary                  KeyframeMeasurement
                                              │
                              ┌───────────────┴───────────────┐
                              ▼                               ▼
                   phad_stereo_vo_probe                phad_euroc_runner
```

## `StereoStatus`

| 值 | 含义 |
|---|---|
| `kValid` | 右目匹配通过全部几何门限；`disparity_px` 有效 |
| `kNoRightMatch` | 右目 LK 失败或前后向不一致；track 保留 |
| `kInvalidDisparity` | 行差超限或视差非正 / 过小 |
| `kDepthOutOfRange` | `z = fx * baseline / disparity` 越界 |

## CSV 合同（probe）

```bash
phad_stereo_frontend_probe <sequence-root> --frames-csv <path> [--tracks-csv <path>]
```

`--frames-csv` 每帧一行：

```text
timestamp_ns,tracked,detected,valid,no_right_match,invalid_disparity,
depth_out_of_range,fb_rejected,epipolar_median_px,epipolar_p95_px
```

`--tracks-csv` 在 track 死亡（及序列结束 flush）时追加：

```text
id,first_timestamp_ns,last_timestamp_ns,length
```

离线绘图：`scripts/plot_tracks.py --frames-csv ... [--tracks-csv ...]`。

## 相关入口

| 位置 | 用途 |
|---|---|
| `phad::camera::StereoRectifier` | 前置整幅校正 |
| `apps/phad_stereo_frontend_probe.cpp` | 无窗口指标导出 |
| `apps/phad_euroc_runner.cpp` | 校正图 + track 叠加 |
| `tests/frontend/` | 合成单测与 MH_01 门控测试 |
