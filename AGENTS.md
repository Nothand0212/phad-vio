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
- 在主 checkout 上使用短生命周期分支，而非 git worktrees；本地 merge 到 `main` 后删除分支，除非明确要求否则不 push 到 remote

## Learned Workspace Facts

- clangd 通过 `.clangd` 配置，`CompilationDatabase: build`；根目录 `compile_commands.json` 是指向 `build/compile_commands.json` 的 symlink，且已被 gitignore；改 CMake 或源文件后用 `cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 重新生成
- 叶子 `sensor::*` 参数类型保持 POD + STL（`std::array`）；Eigen 已是 `phad_sensor`、`phad_common`、`phad_eval` 的 public 依赖（`RigidTransform`、`common::Trajectory` 用 `Eigen::Isometry3d`）；GTSAM 4.3a1 与 spdlog 1.17.0 已备在 gitignore 的 `thirdparty/`，但尚未接进 `CMakeLists.txt`，到 M2.3 VO 后端时接入
- 轨迹的通用载体是 `phad::common::Trajectory`（时间戳严格递增，位姿为 `Eigen::Isometry3d` 的 `T_W_B`），由 `Trajectory::create` 集中校验；`phad::eval` 提供 TUM 读写、时间关联、固定尺度 SE3 对齐与 ATE，并用 evo 交叉验证（`phad_euroc_gt_export` 导出真值为 TUM，再比较 `evo_ape tum <gt> <est> -a` 与 `phad_traj_eval` 的统计量，MH_01 上两者六位有效数字一致）
- `phad::viz` 拆成可无显示测试的 `TrajectoryPanel`（俯视 x-y，渲染到 `cv::Mat`）与薄的 `ImageWindow`（highgui 窗口与退出键）；`tests/viz` 只渲染内存画布，不开窗口。GUI 端到端（`phad_euroc_runner` 回放）需要人工确认，agent 不擅自弹窗
- 离线绘图脚本在 `scripts/`（`plot_trajectory.py` 消费 TUM，`plot_errors.py` 消费 `phad_traj_eval --errors-csv`），依赖装在本地 venv（`scripts/requirements.txt`），不进 CMake 也不进 CI；系统 python3 的 matplotlib 与 numpy 版本不兼容，必须用 venv
- 跨 suite 共用的合成轨迹 fixture 在 `tests/common/synthetic_trajectory.hpp`（`phad::testing` 命名空间）
- EuRoC 真值四元数未严格归一化（偏差可达 1.3e-4），单位性检查用于识别损坏记录而非校核数值精度，读入后统一归一化
- C++ naming 与 style 的权威文档为 `docs/agents/cpp-naming.md` 与 `docs/agents/cpp-style.md`，分别由 `.cursor/rules/cpp-naming.mdc`、`.cursor/rules/cpp-style.mdc` 镜像；格式细节遵循根目录 `.clang-format`
- 单元测试在 `build/` 下通过 `ctest --output-on-failure -L unit` 运行，也可直接运行各 suite 二进制（`phad_sensor_tests`、`phad_io_dataset_tests`、`phad_eval_tests`）；本地 EuRoC 序列在 `/home/lin/Projects/data/thidparty/euroc/native/<sequence>`，`PHAD_ENABLE_MH01_TESTS=ON` 加 `PHAD_EUROC_MH01_PATH` 启用 `-L mh01` 测试
- 内部 C++ APIs 使用缩写命名（`accNd()`、`m_acc_nd`、错误路径 `imu.acc_nd`），外部 dataset YAML/CSV keys 保留原始完整拼写；单位在注释中说明
- `docs/architecture.md` 与 `docs/roadmap.md` 是模块边界与里程碑顺序的权威来源：frontend 负责 feature tracking 与 keyframe 决策，estimator 负责 factor graph 与 landmark lifecycle
- 里程碑为 M1 数据 IO（已完成）→ M2 双目 VO 最小闭环 → M3 VO 加固 → M4 接入 IMU → M5 正式初始化 → M6 边缘化 → M7 线程 → M8 解耦回环 → M9 研究平台；每个里程碑以真实序列上的 ATE 或可观察行为收尾
- 前端用 OpenCV、后端用 GTSAM；自研实现以调库版本为对拍 oracle，不通过对拍则两者并存
- 实施计划的命名与格式由 `docs/plans/README.md` 规定：文件名为 `YYYY-MM-DD_<slug>_<plan_id>.plan.md`，内容为 YAML frontmatter（`name` / `overview` / `todos` / `isProject`）加 Markdown 正文；`.cursor/plans/` 是工作副本，`docs/plans/` 是入库权威副本，两者用同一 `plan_id` 对照
- Issues 落在 remote `origin`（`Nothand0212/phad-vio`）；环境变量 `GITHUB_TOKEN` 指向的 fine-grained PAT 没有 Issues 写权限，需用 `env -u GITHUB_TOKEN gh ...` 改走 keyring 凭据
