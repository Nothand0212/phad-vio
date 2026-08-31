# `tests/` — agent 提示

## 约定

- 在 `build/` 下：`ctest --output-on-failure -L unit`；也可直接跑各 suite 二进制
- 当前 unit suite 含：`phad_sensor_tests`、`phad_camera_tests`、`phad_io_dataset_tests`、`phad_sync_tests`、`phad_eval_tests`、`phad_viz_tests`、`phad_frontend_tests`、`phad_estimator_tests`、`phad_bench_tests`、`phad_apps_tests`
- 共用 fixture：`tests/common/synthetic_trajectory.hpp`、`tests/frontend/synthetic_stereo.hpp`（命名空间 `phad::testing`）
- 本地 EuRoC：`/home/lin/Projects/data/thidparty/euroc/native/<sequence>`
- MH_01 门控：`PHAD_ENABLE_MH01_TESTS=ON` + `PHAD_EUROC_MH01_PATH` → `-L mh01`
- TUM VI corridor1 opt-in 门控：配置 `PHAD_ENABLE_TUMVI_CORRIDOR1_TESTS=ON`，构建 `phad_tumvi_corridor1_test` 与 `phad_tumvi_corridor1_product_test`；以 `PHAD_TUMVI_CORRIDOR1_PATH=/home/lin/Projects/data/thidparty/tum/dataset-corridor1_512_16` 运行 `ctest --output-on-failure -L tumvi-corridor1`
- `tumvi-corridor1` 含 manifest/calibration/image integration test 与 product test。product test 固定处理首 100 个 stereo frames，要求 `error == nullopt`、`image_frames == diag.size() == emitted_stereo == 100`、四个 dropped counter 与 `failed` 均为 0，且 `sum(diag.num_observations) > 0`、`sum(diag.num_disparity) > 0`
- VO / bench 产物常用 `/home/lin/Projects/data/phad-bench`（`PHAD_BENCH_ROOT`）
- MH_01 行为对拍参考：`/home/lin/Projects/data/phad-bench/MH_01_easy/0b0cd34/default_030a0197/{est.tum,diag.csv}`
