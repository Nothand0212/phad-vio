# `phad::bench` — agent 提示

模块合同见同目录 `README.md`（若有）与 `docs/research/m3.1-vo-regression-benchmark-design.md`。

## 约定

- 纯逻辑库：run 身份、config 快照与 hash、路径模板、`summary.json` schema；**零** `phad::*` 依赖（勿链接 `phad_eval` 等）
- `nlohmann_json` 3.11 仅 PRIVATE；public header 不暴露 json 类型
- `summary.json` 可选 `sync` 段（与 `StereoPairDiagnostics` 对齐）；`bench_table.py` 不强制读 sync
- 路径模板：`<bench_root>/<sequence>/<commit_short>[_dirty]/<config_label>_<hash8>/`
