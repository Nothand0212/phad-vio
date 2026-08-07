# `scripts/` — agent 提示

## 约定

- 离线脚本：`plot_trajectory.py`（TUM）、`plot_errors.py`（`--errors-csv`）、`bench_table.py`（扫 `summary.json`）、`segment_ate_decomp.py`（按 `segment_id` 逐段独立对齐 ATE，量化 re-anchor 锚偏移；用法：`python3 scripts/segment_ate_decomp.py <run_dir> <euroc_root>`，只依赖 numpy）
- 依赖在本地 venv（`scripts/requirements.txt`），**不进** CMake / CI
- 系统 `python3` 的 matplotlib 与 numpy 版本不兼容，必须用 venv（如仓库根 `.venv` 或 `scripts` 自建）
