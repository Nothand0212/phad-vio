# AGENTS.md

文档地图见 [`docs/README.md`](docs/README.md)。

- 不要为了向后兼容而保留旧实现。应移除废弃的代码路径，而不是新增兼容层、兼容兜底或迁移逻辑。
- 选择能够完整满足当前需求的最简单实现。避免引入投机性的抽象、配置项和间接层。
- 以分层方式演进系统。先实现端到端可用的最小版本，再在一个已经可工作的产品之上逐步叠加新能力。不要为了尚未完成的复杂性而牺牲一个可工作的产品。
- 保持组件模块化，并清晰分离关注点。
- 当成熟且维护良好的库能够降低整体复杂度或提升可靠性时，优先采用它们。没有明确理由时，不要重复实现通用功能。
- 在自行实现功能或引入新依赖前，优先利用项目中已有的依赖。未经查阅文档与类型定义，不要假定某个库缺少所需能力。
- 架构决策应着眼长期。不要采用只适用于当下、并且预期日后必须替换的权宜之计。
- 在设计方案前，先研究成熟产品如何解决同类问题。优先遵循其经过验证的模式与约定，而非从零创造一套方法。

## 权威文档

| 主题 | 文档 |
|---|---|
| 文档地图 | [`docs/README.md`](docs/README.md) |
| 里程碑路线图 | [`docs/design/roadmap.md`](docs/design/roadmap.md) |
| 目标架构 | [`docs/design/architecture.md`](docs/design/architecture.md) |
| 坐标系 / 单位 | [`docs/design/conventions.md`](docs/design/conventions.md) |
| M4 full-state VIO 合同 | [`docs/specs/2026-08-25-m4-minimal-full-state-vio.md`](docs/specs/2026-08-25-m4-minimal-full-state-vio.md) |
| 世界系连续合同 | [`docs/specs/2026-08-26-m4-world-frame-continuity.md`](docs/specs/2026-08-26-m4-world-frame-continuity.md) |
| M4 锚点 checkpoint | [`docs/benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md`](docs/benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md) |
| 交付流水线 | [`docs/AGENTS.md`](docs/AGENTS.md) |
| C++ 命名 / 风格 | [`docs/agents/cpp-naming.md`](docs/agents/cpp-naming.md)、[`docs/agents/cpp-style.md`](docs/agents/cpp-style.md) |

## Agent skills

### Incremental development

针对当前 milestone，先实现最小可运行的 vertical slice，再根据观测到的需求与失败逐步演进。详见
`docs/agents/incremental-development.md`。

### Evidence-gated integration

接入会改变既有输出的新传感器、先验、模型、factor 或 optimizer 时，按资格阶梯逐层取得权限。
详见 `docs/agents/evidence-gated-integration.md`。

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

### Git workflow

短生命周期分支；合入 `main` 必须 `--no-ff` 保留 merge 图。详见
`docs/agents/git-workflow.md`。

### Remote CI

局域网服务器上的精确源码快照、容器隔离和 EuRoC 并行实验见
`docs/agents/remote-ci.md`。

## 模块 AGENTS 索引

模块专属约定写在对应目录的 `AGENTS.md`（合同细节仍以各目录 `README.md` 为准）。改某库前先读该目录提示：

| 目录 | 说明 |
|---|---|
| [`phad/common/AGENTS.md`](phad/common/AGENTS.md) | `Trajectory`、`LandmarkId` |
| [`phad/sensor/AGENTS.md`](phad/sensor/AGENTS.md) | 测量 / 图像 / 标定叶子类型 |
| [`phad/camera/AGENTS.md`](phad/camera/AGENTS.md) | 立体校正、rectified 外参 |
| [`phad/frontend/AGENTS.md`](phad/frontend/AGENTS.md) | 跟踪边界、PIMPL |
| [`phad/estimator/AGENTS.md`](phad/estimator/AGENTS.md) | GTSAM、图优化边界 |
| [`phad/sync/AGENTS.md`](phad/sync/AGENTS.md) | 双目 / 多源同步 |
| [`phad/io/AGENTS.md`](phad/io/AGENTS.md) | dataset / replay / EuRoC 清单 |
| [`phad/eval/AGENTS.md`](phad/eval/AGENTS.md) | TUM / ATE / RPE |
| [`phad/viz/AGENTS.md`](phad/viz/AGENTS.md) | 可视化与无窗测试 |
| [`phad/bench/AGENTS.md`](phad/bench/AGENTS.md) | 回归 bench 纯逻辑库 |
| [`apps/AGENTS.md`](apps/AGENTS.md) | composition root / session / stream |
| [`tests/AGENTS.md`](tests/AGENTS.md) | ctest、门控序列、对拍路径 |
| [`scripts/AGENTS.md`](scripts/AGENTS.md) | 离线绘图与 bench 表、venv |
| [`docs/AGENTS.md`](docs/AGENTS.md) | 交付流水线；文档地图见 [`docs/README.md`](docs/README.md) |

## 已学到的用户偏好（摘要）

权威正文见 [`docs/agents/preferences.md`](docs/agents/preferences.md)。

- 面向 agent 的文档以中文为主；术语与 identifiers 保留英文。
- 跨库规则在 `docs/agents/`；模块约定在各目录 `AGENTS.md`，勿堆进根文件。
- 开工前逐项对齐（一次一问、A/B/C + 推荐）；设计稿分段确认后再动手；写 spec/计划前先建 GitHub issue。
- 重大设计先开源一手对照，笔记落 `docs/research/`；不擅自定稿。
- 短生命周期分支、commit 带 issue 号、合入 `main` 必须 `--no-ff`、默认不 push：见 [`docs/agents/git-workflow.md`](docs/agents/git-workflow.md)。
- C++ 用 clangd；命名/格式见 `docs/agents/cpp-naming.md` 与 `docs/agents/cpp-style.md`。
- EuRoC / milestone baseline 须含 `meta.json` 参数快照（与 `config_hash` 同源）；验收不够先诊断再扩序列。

## Learned Workspace Facts

- clangd 通过 `.clangd` 配置，`CompilationDatabase: build`；根目录 `compile_commands.json` 是指向 `build/compile_commands.json` 的 symlink（gitignore）；改 CMake 或源文件后用 `cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 重新生成；仓库只有单一根 `CMakeLists.txt`，开启 `-Wconversion -Wsign-conversion -Wpedantic`。
- C++ naming / style 权威为 `docs/agents/cpp-naming.md` 与 `docs/agents/cpp-style.md`；格式遵循根 `.clang-format`。依赖细节见各模块 `AGENTS.md`。
- M4 minimal full-state VIO 锚定基线：`c999f58` / `default_0337287b`（见 [`docs/benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md`](docs/benchmark/m4/minimal-full-state-vio_c999f58_0337287b.md)）；EuRoC 11/11 均值 ATE ≈0.579，段内加权 RMS ≈0.134（相对 ORB-SLAM3 stereo ≈1.6×）；主失败模式是视觉中断后 segment 重启把世界系重置到原点；下一步优先世界系连续（[`docs/specs/2026-08-26-m4-world-frame-continuity.md`](docs/specs/2026-08-26-m4-world-frame-continuity.md)），M5 正式动态初始化排其后；双目尺度已知、body≡IMU、单一 `VioEstimator::update()`、滑窗 10、视觉短时丢失最多 500 ms IMU coast。
- `LandmarkId` 现为 frontend track 与 estimator map 共用身份；TrackId≠LandmarkId 为已确认的中期债。前端 OpenCV、后端 GTSAM，自研以调库版为对拍 oracle。
- Issues 在 remote `origin`（`Nothand0212/phad-vio`）；`GITHUB_TOKEN` 指向的 fine-grained PAT 无 Issues 写权限时，用 `env -u GITHUB_TOKEN gh ...` 改走 keyring 凭据。
