# `phad::eval` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- 提供 TUM 读写、时间关联、固定尺度 SE3 对齐、ATE 与固定时间间隔 RPE（默认 1 s）
- **不做**距离间隔 RPE
- 用 evo 交叉验证：`phad_euroc_gt_export` 出 TUM，再比 `evo_ape tum <gt> <est> -a` 与 `phad_traj_eval`；MH_01 上六位有效数字一致
- EuRoC 真值四元数未严格归一化（偏差可达约 `1.3e-4`）：单位性检查识别损坏记录，读入后统一归一化
- Eigen 是 public 依赖（消费 `common::Trajectory`）
