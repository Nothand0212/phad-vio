# `apps/` — agent 提示

本目录是 composition root；不在此写单独 README（见根目录偏好）。

## 约定

- 编排与落盘放在 app / 薄静态库，不反向注入 `frontend` / `estimator`
- `OfflineVoSession`（target `phad_offline_vo_session`）只跑 pipeline，不算 ATE/RPE、不落盘；不链接 `phad_eval`
- `StereoPairStream` 组合 `SensorSource` + `phad::sync`；`io` 与 `sync` 互不依赖
- `FrameTracks` → `KeyframeMeasurement` 经 `stereo_vo_glue.hpp` 组装
- `phad_stereo_vo_probe` 与 `phad_vo_bench` 共用 session，保证 diag 列合同一致
- 行为保持型重构：先在当前 commit 产出仓库外参考产物，再改代码并以逐字节 diff 验收
