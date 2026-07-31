# `phad::common` — agent 提示

## 约定

- 轨迹载体是 `Trajectory`：时间戳严格递增，位姿为 `Eigen::Isometry3d` 的 `T_W_B`，经 `Trajectory::create` 集中校验
- `LandmarkId` 定义在本库；frontend / estimator 共用，勿在下游库另造 ID 类型
- Eigen 是 public 依赖
