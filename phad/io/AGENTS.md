# `phad::io` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- dataset 只做单路清单自洽（递增、无重复、PNG 存在等）；左右不等长允许 `open` 成功
- 公共 API：`peekImageTimestamp(CameraId)` / `takeImage(CameraId)`；`summary` 为 `imu` / `left` / `right`
- `SensorEvent` 为 `ImuMeasurement | ImageFrameEvent`（不是已配对 `StereoFrame`）
- `DatasetReplaySource` 按时间归并，同 stamp 顺序 **IMU → Left → Right**
- 旧 `joinStereo` 与 `DatasetErrorCode::kStereoTimestampMismatch` 已删除；配对归 `phad::sync`
- EuRoC ASL native 下 cam0/cam1 行数可不一致（如 MH_04 / V1_02 / V2_03），属官方清单不对称；排障时先对照开源与本地清单，勿直接断定下载损坏
- 外部 YAML/CSV 键名按原文完整拼写；单位与坐标系转换在 adapter 内完成
