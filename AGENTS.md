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

- `docs/agents/` 下面向 agent 的文档优先使用中文；专有名词、技术术语与 code identifiers 保留英文
- 应将 agents 需遵守的 Cursor rules 持久化到 `docs/agents/`，并在 `AGENTS.md` 中链接
- C++ 的 go-to-definition 与 references 优先使用 clangd；禁用 Microsoft C/C++ IntelliSense 以避免冲突
- C++ identifiers 宜短，使用常见领域缩写（`imu`、`acc`、`gyr`、`nd`、`rw`、`fx`）；避免自造缩写与拼写出的单位后缀
- 在主 checkout 上使用短生命周期分支，而非 git worktrees；本地 merge 到 `main` 后删除分支，除非明确要求否则不 push 到 remote

## Learned Workspace Facts

- clangd 通过 `.clangd` 配置，`CompilationDatabase: build`；根目录 `compile_commands.json` 是指向 `build/compile_commands.json` 的 symlink，且已被 gitignore
- 修改 CMake 或源文件后，用 `cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 重新生成
- 核心 `sensor::*` 与 calibration types 保持 POD + STL（`std::array`）；Eigen/GTSAM 推迟到 EuRoC loader（M1）阶段之后
- C++ naming 权威文档为 `docs/agents/cpp-naming.md`，并由 `.cursor/rules/cpp-naming.mdc` 镜像
- C++ style 权威文档为 `docs/agents/cpp-style.md`，并由 `.cursor/rules/cpp-style.mdc` 镜像；格式细节遵循根目录 `.clang-format`
- 单元测试在 `build/` 下通过 `ctest --output-on-failure -L unit` 运行；也可直接运行各 suite 二进制，如 `phad_sensor_tests`、`phad_io_dataset_tests`
- 内部 C++ APIs 使用缩写命名（`accNd()`、`m_acc_nd`、错误路径 `imu.acc_nd`），外部 dataset YAML/CSV keys 保留原始完整拼写；单位在注释中说明
- `docs/architecture.md` 与 `docs/roadmap.md` 是模块边界与 phase 顺序的权威来源：frontend 负责 feature tracking 与 keyframe 决策，estimator 负责 factor graph 与 landmark lifecycle
