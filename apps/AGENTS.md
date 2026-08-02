# `apps/` — agent 提示

本目录是 composition root；不在此写单独 README（见根目录偏好）。

## 约定

- 编排与落盘放在 app / 薄静态库，不反向注入 `frontend` / `estimator`
- `OfflineVoSession`（target `phad_offline_vo_session`）只跑 pipeline，不算 ATE/RPE、不落盘；不链接 `phad_eval`
- `StereoPairStream` 组合 `SensorSource` + `phad::sync`；`io` 与 `sync` 互不依赖
- `FrameTracks` → `KeyframeMeasurement` 经 `stereo_vo_glue.hpp` 组装
- `phad_stereo_vo_probe` 与 `phad_vo_bench` 共用 session，保证 diag 列合同一致
- Slice ④c：`estimator.update` 返回后若
  `!diagnostics.culled_landmark_ids.empty()`，session 调用
  `tracker.dropTracks(...)`（composition root 回传；**不**进 `warnings`）。
  `block_culled_rebirth=false` 时仍 drop（设计 §5）；真复现 tip 伪永久需另
  关 drop（YAGNI，本片不加 session 开关）。bench
  `--allow-culled-rebirth` 仅把 `estimator.block_culled_rebirth` 设为
  `false`；默认不传 flag 即 estimator 默认 `true`。该键进
  `flattenConfig` → `config_hash`。Slice ④d：
  `--outlier-avg-reproj-px <v>`（须 `> 0`）覆盖
  `estimator.outlier_avg_reproj_px`（默认 `4.0`），同样进 `config_hash`
- 行为保持型重构：先在当前 commit 产出仓库外参考产物，再改代码并以逐字节 diff 验收

## `diag.csv` 合同（M3.3）

M3.3 Slice ① **有意**将 `diag.csv` 从 **13 列扩到 14 列**：在既有列尾追加
`segment_id`（与 `UpdateDiagnostics.segment_id` 一致）。

M3.3 Slice ③ **有意**再扩到 **16 列**：在 `segment_id` 后追加
`pnp_success,pnp_inliers`（`pnp_success` 为 0/1 整数；与
`UpdateDiagnostics` 同名字段一致）。

M3.3 Slice ④ **有意**再扩到 **18 列**：在 `pnp_inliers` 后追加
`outliers_culled,reproj_rms_after_cull_px`（整数 + 浮点；与
`UpdateDiagnostics` 同名字段一致）。这不是格式漂移——bench / probe 共用
`writeDiagCsv()`，旧 13/14/16 列参考产物不再适用于 Slice ④ 及之后。

Slice ④b（剔点后 LM₂）**保持 18 列**：不追加 `outlier_reopt` /
`outlier_reopt_failed`。成功 reopt 只进 `FrameCounts.outlier_reopts` →
`summary.json` 的 `robustness.outlier_reopts`，以及 probe stdout
`outlier_reopts=`；失败回退不计、也不进 `warnings`。

健康序列（如 MH_01）仍应 `segment_id` 全程为 0；断裂序列上段边界表现为
该列跳变。`summary.json` 侧对应 `robustness.reanchors` /
`robustness.pnp_successes` / `robustness.pnp_fallbacks` /
`robustness.outliers_culled` / `robustness.outliers_culled_unique` /
`robustness.outlier_reopts` 与 `trajectory.segments`（见
`phad/bench/README.md`）。

`OfflineVoSession` 结束时的 `vo segments summary: segments=… reanchors=…
seed_rejected=…` 只在 `reanchors > 0 || seed_rejected > 0` 时写入
`warnings`；`vo pnp summary: pnp_successes=… pnp_fallbacks=…` 只在
`pnp_fallbacks > 0` 时写入。剔点 / reopt 累计只进 `FrameCounts` /
`summary.json` 的 `robustness.outliers_culled` /
`robustness.outliers_culled_unique` / `robustness.outlier_reopts`，
**不**写入 `warnings`（健康序列上正常 cull / reopt 仍可保持
`kCompleted`）。干净的单段且 PnP 全成功跑不产生 segments / pnp warning。
probe stdout 的计数摘要不受此限制，始终打印（含 `outliers_culled` /
`outliers_culled_unique` / `outlier_reopts`）。
