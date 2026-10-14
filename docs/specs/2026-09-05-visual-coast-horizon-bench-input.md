# Visual coast horizon bench 输入合同

状态：已定稿（GitHub #57）

## 目标

`phad_vo_bench` 将既有 visual coast horizon 暴露为显式、可审计的单变量实验输入，
供同一 source snapshot 上的 500/2000 ms V2_03 A/B 使用。本片不改变 estimator
状态机、factor、恢复、segment 生命周期或默认值。

## 输入与装配

- CLI：`--visual-coast-horizon-ms <n>`。
- `<n>` 必须是正整数，且 `n * 1'000'000` 可由 `std::int64_t` nanoseconds
  精确表示；非法格式、非正值与溢出值以 CLI exit 2 拒绝。
- 未提供参数时，继续使用
  `EstimatorOptions::m_visual_coast_horizon_ns = 500'000'000`。
- 显式值只装配到
  `OfflineVoSessionOptions::estimator.m_visual_coast_horizon_ns` 既有 seam。

## Canonical identity

`flattenConfig()` 始终写入实际生效的
`estimator.visual_coast_horizon_ns`。因此默认路径与显式 500 ms 的 canonical text
及 hash 相同；当前 formal-moving 默认 identity 为 `4e2b30f2`。2000 ms 的
canonical text 与 hash 必须不同。`meta.json` 的 `config`、
`config_canonical_text` 和 `config_hash` 复用现有 snapshot pipeline，不保留旧 hash
alias 或特判。

## 验收

1. usage、合法值、非法格式、非正值与溢出值的 CLI 测试通过。
2. 默认/500/2000 ms 的 config、canonical text、meta 与 hash 合同通过。
3. 既有 500 ms estimator 边界语义回归通过。

同源 V2_03 A/B、停止门和结果回填仍按 GitHub #57 执行，不属于本代码切片的本地
定向测试。
