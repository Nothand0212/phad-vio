# `tests/` — agent 提示

## 约定

- 在 `build/` 下：`ctest --output-on-failure -L unit`；也可直接跑各 suite 二进制
- 当前 unit suite 含：`phad_sensor_tests`、`phad_camera_tests`、`phad_io_dataset_tests`、`phad_sync_tests`、`phad_eval_tests`、`phad_viz_tests`、`phad_frontend_tests`、`phad_estimator_tests`、`phad_bench_tests`、`phad_apps_tests`
- 共用 fixture：`tests/common/synthetic_trajectory.hpp`、`tests/frontend/synthetic_stereo.hpp`（命名空间 `phad::testing`）
- 本地 EuRoC：`/home/lin/Projects/data/thidparty/euroc/native/<sequence>`
- MH_01 门控：`PHAD_ENABLE_MH01_TESTS=ON` + `PHAD_EUROC_MH01_PATH` → `-L mh01`
- VO / bench 产物常用 `/home/lin/Projects/data/phad-bench`（`PHAD_BENCH_ROOT`）
- MH_01 行为对拍参考：`/home/lin/Projects/data/phad-bench/MH_01_easy/0b0cd34/default_030a0197/{est.tum,diag.csv}`
