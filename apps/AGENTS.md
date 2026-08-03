# `apps/` — agent 提示

本目录是 composition root；不在此写单独 README（见根目录偏好）。

## 约定

- 编排与落盘放在 app / 薄静态库，不反向注入 `frontend` / `estimator`
- `OfflineVoSession`（target `phad_offline_vo_session`）只跑 pipeline，不算 ATE/RPE、不落盘；不链接 `phad_eval`
- `StereoPairStream` 组合 `SensorSource` + `phad::sync`；`io` 与 `sync` 互不依赖
- `FrameTracks` → `KeyframeMeasurement` 经 `stereo_vo_glue.hpp` 组装
- `phad_stereo_vo_probe` 与 `phad_vo_bench` 共用 session，保证 diag 列合同一致
- Slice ④c：`estimator.update` 返回后若
  `!diagnostics.culled_landmark_ids.empty()` 且
  `session.drop_culled_tracks`（默认 `true`），session 调用
  `tracker.dropTracks(...)`（composition root 回传；**不**进 `warnings`）。
  `block_culled_rebirth=false` 时默认仍 drop。bench
  `--allow-culled-rebirth` 仅把 `estimator.block_culled_rebirth` 设为
  `false`；`--no-drop-culled-tracks` 把 `session.drop_culled_tracks` 设为
  `false`（A/B；两键均进 `flattenConfig` → `config_hash`）。Slice ④d：
  `--outlier-avg-reproj-px <v>`（须 `> 0`）覆盖
  `estimator.outlier_avg_reproj_px`（默认 `4.0`），同样进 `config_hash`。
  Slice ④e：`--max-outlier-reopts <n>`（须 `≥ 0`）覆盖
  `estimator.max_outlier_reopts`（默认 `3`），进 `flattenConfig` →
  `config_hash`；非法值非 0 退出。Slice ④f：**两级门**——先
  `session.drop_culled_tracks`（默认 `true`）；若为 true 且
  `culled_landmark_ids` 非空，再比 `outliers_culled`（仅 mean-cull，**不是**
  `culled_landmark_ids.size()`）与 `session.skip_drop_min_culled`（默认
  **4**）：`N > 0` 且 `outliers_culled ≥ N` 时**跳过**本帧
  `dropTracks` 并 `++drops_skipped`；否则仍 drop。`N = 0` 关闭阈值跳过（非空
  列表必 drop，与 ④c 一致）。`--no-drop-culled-tracks` 把
  `drop_culled_tracks` 设为 `false` → **永不** drop，**不**计入
  `drops_skipped`（A/B 诊断臂）。`--skip-drop-min-culled <n>`（须 `≥ 0`）覆盖
  阈值；非法值非 0 退出。两 session 键均进 `flattenConfig` →
  `config_hash`。跳过 drop **不**进 `warnings`。
- Slice ④g 编排已回退：默认恢复 ④f「skip 则不 drop」；`deferred_drops` /
  `deferred_drop_ids` 计数合同仍保留（默认恒 0）。事后诊断见
  `docs/research/m3.3-slice4g-postmortem.md`（④g ≡ ④e bit-identical）。
- 子集延后 drop 探针：`--defer-drop-topk <k>`（默认 0；bench / probe）；
  skip 后按 `LandmarkId` 排序取前 k 入 `pending_drop`，下帧 `process` 前
  flush；**不**进 `flattenConfig` / `config_hash`。见
  `docs/research/m3.3-post4g-next-knife-candidates.md`。
- 懒腾槽探针：`--evict-skip-culled`（flag，默认关；bench / probe）；skip 时对
  完整 `culled_landmark_ids` 调 `markEvictable`，GFTT 需要槽位时按 id 升序
  挤出；**不**进 `flattenConfig` / `config_hash`。见
  `docs/research/m3.3-evict-skip-culled-probe-design.md`。
- 多帧 zombie 龄 drop 探针：`--zombie-drop-age <n>`（默认 0；bench / probe）；
  skip 入表龄=1，连续仍在 `FrameTracks` 则 +1，`≥n` 精确 `dropTracks`；
  **不**进 `flattenConfig` / `config_hash`。见
  `docs/research/m3.3-zombie-drop-age-probe-design.md`。
- MH_05 Probe B：`--probe-b <path>` 为 CLI-only（bench / probe）；**不**进
  `flattenConfig` / `config_hash`，**不**改 18 列 `diag.csv`。
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

Slice ④b / ④e（剔点后 reopt）**保持 18 列**：不追加 `outlier_reopt` /
`outlier_reopt_rounds` / `outlier_reopt_failed`。成功 reopt **次数**
（Σ `outlier_reopt_rounds`，非「触发过 reopt 的帧数」）进
`FrameCounts.outlier_reopts` → `summary.json` 的
`robustness.outlier_reopts`，以及 probe stdout `outlier_reopts=`；失败回退
不计、也不进 `warnings`。

健康序列（如 MH_01）仍应 `segment_id` 全程为 0；断裂序列上段边界表现为
该列跳变。`summary.json` 侧对应 `robustness.reanchors` /
`robustness.pnp_successes` / `robustness.pnp_fallbacks` /
`robustness.outliers_culled` / `robustness.outliers_culled_unique` /
`robustness.outlier_reopts` / `robustness.drops_skipped`（Slice ④f：因
`outliers_culled ≥ N` 跳过 drop 的**帧数**）/
`robustness.evictable_marked` / `robustness.tracks_evicted`（懒腾槽探针）/
`robustness.zombie_age_drops` / `robustness.zombie_age_drop_ids`（龄 drop
探针）与 `trajectory.segments`（见 `phad/bench/README.md`）。

`OfflineVoSession` 结束时的 `vo segments summary: segments=… reanchors=…
seed_rejected=…` 只在 `reanchors > 0 || seed_rejected > 0` 时写入
`warnings`；`vo pnp summary: pnp_successes=… pnp_fallbacks=…` 只在
`pnp_fallbacks > 0` 时写入。剔点 / reopt 累计只进 `FrameCounts` /
`summary.json` 的 `robustness.outliers_culled` /
`robustness.outliers_culled_unique` / `robustness.outlier_reopts` / `robustness.drops_skipped`，
**不**写入 `warnings`（健康序列上正常 cull / reopt / skip-drop 仍可保持
`kCompleted`）。干净的单段且 PnP 全成功跑不产生 segments / pnp warning。
probe stdout 的计数摘要不受此限制，始终打印（含 `outliers_culled` /
`outliers_culled_unique` / `outlier_reopts` / `drops_skipped` /
`evictable_marked` / `tracks_evicted` / `zombie_age_drops` /
`zombie_age_drop_ids`）。
