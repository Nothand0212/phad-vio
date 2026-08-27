# `phad::estimator` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- 负责 factor graph 与 landmark lifecycle；不依赖 `phad::frontend`
- GTSAM 4.3 经 `find_package(GTSAM 4.3 REQUIRED)` **PRIVATE** 链接；include dir 标 SYSTEM 以挡警告刷屏
- 构建 GTSAM 时须 `GTSAM_USE_SYSTEM_EIGEN=ON`，与项目 `Eigen3::Eigen` 3.4 为同一份
- `StereoVoEstimator` 用 PIMPL 藏 GTSAM；public header 只用 Eigen / POD
- `phad::sensor` 是公开叶子类型依赖；gyro 已在 body/IMU frame，不套用相机外参旋转
- 最小 gyro 模式只允许 shared fixed bias + pose-only AHRS factor：不读 accel / `gyr_rw`，
  不引入 velocity、gravity、bias key 或 random walk；`off` 必须在入口直接绕过全部 gyro 逻辑
