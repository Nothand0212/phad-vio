## Agent skills

### Incremental development

针对当前 milestone，先实现最小可运行的 vertical slice，再根据观测到的需求与失败逐步演进。详见
`docs/agents/incremental-development.md`。

### Issue tracker

Issues 与 PRDs 在 GitHub Issues 中跟踪。详见 `docs/agents/issue-tracker.md`。

### Triage labels

使用五个默认的 canonical triage labels。详见 `docs/agents/triage-labels.md`。

### Domain docs

本仓库为 single-context repository。详见 `docs/agents/domain.md`。

### C++ naming

C++ identifiers 遵循项目命名规则。详见 `docs/agents/cpp-naming.md`。

### C++ style

C++ formatting 与 control-flow style 遵循项目规则。详见 `docs/agents/cpp-style.md`。

## Learned User Preferences

- `AGENTS.md` 与 `docs/agents/` 下面向 agent 的文档优先使用中文；专有名词、技术术语与 code identifiers 保留英文
- 应将 agents 需遵守的 Cursor rules 持久化到 `docs/agents/`，并在 `AGENTS.md` 中链接
- C++ 的 go-to-definition 与 references 优先使用 clangd；禁用 Microsoft C/C++ IntelliSense 以避免冲突
- C++ identifiers 宜短，使用常见领域缩写（`imu`、`acc`、`gyr`、`nd`、`rw`、`fx`）；避免自造缩写与拼写出的单位后缀
- 在主 checkout 上使用短生命周期分支，而非 git worktrees；按 vertical slice 逐片提交（commit subject 带对应 issue 号，如 `(#18)`），本地 merge 到 `main` 后删除分支，除非明确要求否则不 push 到 remote
- `phad/` 下每个库目录都要有 README（职责边界、文件布局、数据流、格式合同），顶部声明「本文档描述当前约定，不是绝对约束，会随项目开发修订」；`apps/`、`tests/` 与 adapter 子目录不单独写
- 每个 milestone 与 vertical slice 开工前先逐项对齐：一次只问一个问题、给 A/B/C 选项并标明推荐，设计稿分段（模块边界 → 数据流 → 错误与诊断 → 测试与验收）逐段确认后再动手
- 关键设计决策前先检索开源参考实现（GTSAM examples、Kimera-VIO、ORB-SLAM3、VINS-Fusion），把对照笔记写进 `docs/research/` 再定方案；审阅设计文档时直接指出问题并与用户讨论改法，不擅自定稿
- 模块边界优先低耦合、高内聚、深模块与单向依赖；benchmark/编排放 composition root（app 或零 `phad::*` 依赖的纯逻辑库），不反向注入 frontend/estimator
- 探针/session 等行为保持型重构：先在当前 commit 产出仓库外参考产物，再改代码并以逐字节 diff 验收
- 排障时先对照开源实现与本地清单，区分数据损坏与 loader 合同过严，勿直接断定数据源损坏

## Learned Workspace Facts

- clangd 通过 `.clangd` 配置，`CompilationDatabase: build`；根目录 `compile_commands.json` 是指向 `build/compile_commands.json` 的 symlink，且已被 gitignore；改 CMake 或源文件后用 `cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 重新生成；仓库只有单一根 `CMakeLists.txt`（无 per-module 版本），且开启 `-Wconversion -Wsign-conversion -Wpedantic`
- 叶子 `sensor::*` 参数类型保持 POD + STL（`std::array`）；Eigen 已是 `phad_sensor`、`phad_common`、`phad_eval` 的 public 依赖（`RigidTransform`、`common::Trajectory` 用 `Eigen::Isometry3d`）；spdlog 1.17.0 备在 gitignore 的 `thirdparty/` 尚未接入；GTSAM 4.3.0 已装在 `/usr/local`，经 `find_package(GTSAM 4.3 REQUIRED)` 由 `phad_estimator` PRIVATE 链接并把 include dir 标 SYSTEM 以挡警告刷屏，且其 Eigen 必须与 `Eigen3::Eigen` 3.4 是同一份（构建 GTSAM 时用 `GTSAM_USE_SYSTEM_EIGEN=ON`）；系统已装 `nlohmann_json` 3.11（`find_package(nlohmann_json 3.11)`），供 `phad_bench` / `phad_vo_bench` PRIVATE 链接，public header 不暴露 json 类型
- 轨迹的通用载体是 `phad::common::Trajectory`（时间戳严格递增，位姿为 `Eigen::Isometry3d` 的 `T_W_B`），由 `Trajectory::create` 集中校验；`phad::eval` 提供 TUM 读写、时间关联、固定尺度 SE3 对齐、ATE 与固定时间间隔 RPE（默认 1 s；距离间隔 RPE 有意不做），并用 evo 交叉验证（`phad_euroc_gt_export` 导出真值为 TUM，再比较 `evo_ape tum <gt> <est> -a` 与 `phad_traj_eval` 的统计量，MH_01 上两者六位有效数字一致）；EuRoC 真值四元数未严格归一化（偏差可达 1.3e-4），单位性检查用于识别损坏记录，读入后统一归一化
- `phad::viz` 拆成可无显示测试的 `TrajectoryPanel`（俯视 x-y，渲染到 `cv::Mat`）与薄的 `ImageWindow`（highgui 窗口与退出键）；`tests/viz` 只渲染内存画布，不开窗口。GUI 端到端（`phad_euroc_runner` 回放）需要人工确认，agent 不擅自弹窗；`phad_viz` 把 `opencv_core` 设为 PUBLIC，新库不再照做：`camera::StereoRectifier`、`frontend::StereoTracker` 与 `estimator::StereoVoEstimator` 都用 PIMPL 藏实现细节（`cv::Mat` / GTSAM），public header 只用 Eigen 与 POD，重依赖 PRIVATE 链接
- 离线脚本在 `scripts/`（`plot_trajectory.py` 消费 TUM，`plot_errors.py` 消费 `phad_traj_eval --errors-csv`，`bench_table.py` 扫 bench 根目录下 `summary.json` 拼对照表），依赖装在本地 venv（`scripts/requirements.txt`），不进 CMake 也不进 CI；系统 python3 的 matplotlib 与 numpy 版本不兼容，必须用 venv
- 调研笔记与设计稿都放 `docs/research/`，文件名不带日期前缀（如 `m2.3-vo-backend-open-source-refs.md`、`m2.3-vo-backend-design.md`）；日期只写在文档正文的元信息里。工作流为 对齐问答 → research 笔记 → design 文档 → `docs/plans/` 实施计划 → 分片实现；实施计划命名与格式由 `docs/plans/README.md` 规定（`YYYY-MM-DD_<slug>_<plan_id>.plan.md` + YAML frontmatter），`.cursor/plans/` 是工作副本、`docs/plans/` 是入库权威副本，用同一 `plan_id` 对照
- C++ naming 与 style 的权威文档为 `docs/agents/cpp-naming.md` 与 `docs/agents/cpp-style.md`，分别由 `.cursor/rules/cpp-naming.mdc`、`.cursor/rules/cpp-style.mdc` 镜像；格式细节遵循根目录 `.clang-format`；内部 C++ APIs 用缩写（`accNd()`、`m_acc_nd`），外部 dataset YAML/CSV keys 保留完整拼写，单位在注释中说明
- 单元测试在 `build/` 下通过 `ctest --output-on-failure -L unit` 运行，也可直接运行各 suite 二进制（`phad_sensor_tests`、`phad_camera_tests`、`phad_io_dataset_tests`、`phad_eval_tests`、`phad_viz_tests`、`phad_frontend_tests`、`phad_estimator_tests`、`phad_bench_tests`、`phad_apps_tests`）；共用 fixture 在 `tests/common/synthetic_trajectory.hpp` 与 `tests/frontend/synthetic_stereo.hpp`（`phad::testing` 命名空间）；本地 EuRoC 序列在 `/home/lin/Projects/data/thidparty/euroc/native/<sequence>`，`PHAD_ENABLE_MH01_TESTS=ON` 加 `PHAD_EUROC_MH01_PATH` 启用 `-L mh01` 测试（`phad_euroc_mh01_test`、`phad_eval_mh01_test`、`phad_frontend_mh01_test`；`phad_apps_tests` 的 `max_frames` 用例也依赖该路径）；VO 基线用 `phad_stereo_vo_probe` / `phad_vo_bench` 出 TUM/诊断与 summary，再经 `phad_traj_eval` / `plot_trajectory.py` / `bench_table.py` 对照；本地 bench 根目录常用 `/home/lin/Projects/data/phad-bench`（`PHAD_BENCH_ROOT`）
- `docs/architecture.md` 与 `docs/roadmap.md` 是模块边界与里程碑顺序的权威来源：frontend 负责 feature tracking 与 keyframe 决策，estimator 负责 factor graph 与 landmark lifecycle；`LandmarkId` 在 `phad::common`；立体校正归 `phad::camera`（`StereoRectifier` 产出 `RectifiedStereoCalibration`），frontend 只消费校正后帧，estimator 不依赖 frontend——`FrameTracks` → `KeyframeMeasurement` 由 `apps/stereo_vo_glue.hpp` 组装；离线编排在 `apps/offline_vo_session`（`phad_offline_vo_session`，不链 `phad_eval`），由 `phad_stereo_vo_probe` 与 `phad_vo_bench` 共用；`body_P_sensor` 必须用 `T_B_left_rectified()`（勿用未校正 `T_B_left`）
- 里程碑为 M1 数据 IO（已完成）→ M2 双目 VO 最小闭环（M2.1–M2.3 已完成，issue `#20`，计划 `docs/plans/2026-07-31_m2.3_vo_backend_dcdbfc71.plan.md`；MH_01 基线 ATE translation RMSE ≈ 0.15 m，拒帧/连通性/RMS 只记录不门控）→ M3.1 回归 Benchmark（已完成，issue `#21`，设计 `docs/research/m3.1-vo-regression-benchmark-design.md`，计划 `docs/plans/2026-07-31_m3.1_vo_regression_benchmark_7c4e91a2.plan.md`；`phad::bench` + `OfflineVoSession` + `phad_vo_bench` + `scripts/bench_table.py`；MH_01 ATE translation RMSE ≈ 0.150155 m）→ M3.2 双目配对同步器（issue `#22`，设计 `docs/research/stereo-pair-synchronizer-design.md`，计划 `docs/plans/2026-07-31_m3.2_stereo_pair_synchronizer_5b7d1c93.plan.md`；左右配对移入新库 `phad::sync`，dataset 改左右分路 per-camera API，删 `joinStereo` 与 `kStereoTimestampMismatch`，解开 MH_04 / V1_02 / V2_03 三条拒开序列）→ M3.3 VO 加固 → M4 接入 IMU → M5 正式初始化 → M6 边缘化 → M7 线程 → M8 解耦回环 → M9 研究平台；每个里程碑以真实序列上的 ATE 或可观察行为收尾；前端用 OpenCV、后端用 GTSAM，自研实现以调库版本为对拍 oracle，不通过对拍则两者并存
- EuRoC ASL native 下 cam0/cam1 `data.csv` 行数可不一致（如 MH_04 / V1_02 / V2_03）；`joinStereo` 当前要求等长且按下标 exact timestamp，否则报 `camera manifests have different record counts` 并在 `open()` 失败；开源常见用时间戳交集或毫秒容差配对，同一目录仍可跑——失败未必等于数据整包损坏
- Issues 落在 remote `origin`（`Nothand0212/phad-vio`）；环境变量 `GITHUB_TOKEN` 指向的 fine-grained PAT 没有 Issues 写权限，需用 `env -u GITHUB_TOKEN gh ...` 改走 keyring 凭据
