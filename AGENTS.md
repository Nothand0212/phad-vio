# AGENTS.md

本文档描述当前约定，不是绝对约束，会随项目开发修订。

面向 AI coding agents 的**根索引**。设计正文与流程细则不在此堆叠；权威在链接目标。
目录职责与命名见 [`docs/README.md`](docs/README.md)。

phad-vio：以学习与验证为目标、从零实现的 GTSAM-based stereo VIO。
**当前阶段：M4 最小 full-state VIO 已锚定**（`c999f58` / `default_0337287b`）；
下一步优先世界系连续（[`docs/specs/2026-08-26-m4-world-frame-continuity.md`](docs/specs/2026-08-26-m4-world-frame-continuity.md)），
M5 正式动态初始化排其后。未授权不得 push。

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
| 工作偏好 | [`docs/agents/preferences.md`](docs/agents/preferences.md) |
| C++ 命名 / 风格 | [`docs/agents/cpp-naming.md`](docs/agents/cpp-naming.md)、[`docs/agents/cpp-style.md`](docs/agents/cpp-style.md) |

## 工程原则

- 移除废弃路径，不为向后兼容保留旧实现或兼容层。
- 用能满足当前需求的最简单实现；避免投机性抽象、配置项与间接层。
- 分层演进：先端到端可工作的最小版本，再叠加能力。
- 组件模块化，清晰分离关注点；优先成熟库，不重复造轮子。
- 先查项目已有依赖与类型定义，再自行实现或引入新依赖。
- 架构决策着眼长期；设计前先对照成熟产品的已验证模式。

## 硬约束

- **先确认、后落定**：逐条对齐后再写实现；聊天结论不是唯一真相。见 [`docs/agents/preferences.md`](docs/agents/preferences.md)、[`docs/agents/design-alignment.md`](docs/agents/design-alignment.md)。
- **短分支 + issue**：vertical slice 开工前建 GitHub issue；commit subject 带 issue 号；合入 `main` 必须 `merge --no-ff`。见 [`docs/agents/git-workflow.md`](docs/agents/git-workflow.md)。
- **禁止**未授权 `git push`。

## Agent skills

| Skill | 何时读 | 文档 |
|---|---|---|
| Incremental development | 开 milestone / vertical slice | [`docs/agents/incremental-development.md`](docs/agents/incremental-development.md) |
| Evidence-gated integration | 接入会改变既有输出的传感器 / factor / 先验 | [`docs/agents/evidence-gated-integration.md`](docs/agents/evidence-gated-integration.md) |
| Issue tracker | 建票、评论、关票（`gh`；必要时 `env -u GITHUB_TOKEN`） | [`docs/agents/issue-tracker.md`](docs/agents/issue-tracker.md) |
| Triage labels | 贴/改五个 canonical 标签 | [`docs/agents/triage-labels.md`](docs/agents/triage-labels.md) |
| Domain docs | 探索代码前读 ADR / design / CONTEXT | [`docs/agents/domain.md`](docs/agents/domain.md) |
| Remote CI | 局域网精确快照、容器隔离、EuRoC 并行实验 | [`docs/agents/remote-ci.md`](docs/agents/remote-ci.md) |

C++ 命名 / 风格与 Git 工作流见上表「权威文档」与「硬约束」，不另开 skill 段。

## 模块 AGENTS 索引

模块专属约定写在对应目录的 `AGENTS.md`（合同细节以各目录 `README.md` 为准）。改某库前先读该目录提示：

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
| [`docs/AGENTS.md`](docs/AGENTS.md) | 交付流水线 |

## 已学到的用户偏好（摘要）

权威正文见 [`docs/agents/preferences.md`](docs/agents/preferences.md)。

- 文档以中文为主；术语与 identifiers 保留英文。跨库规则在 `docs/agents/`，模块约定在各目录 `AGENTS.md`。
- 教学叙事在 `docs/learn/`：按 pipeline 分章，须分解到代码与 spec 的模块内部；配图用 Archify。权威合同仍在 design/specs。
- 开工前逐项对齐（一次一问、A/B/C + 推荐）；设计稿分段确认后再动手；写 spec/计划前先建 issue。
- 重大设计先开源一手对照，笔记落 `docs/research/`；不擅自定稿。
- EuRoC / milestone 数值、QA 与逐序列结果以 checkpoint / evidence JSON 为准，不用聊天摘要替代；baseline 须含 `meta.json` 参数快照（与 `config_hash` 同源）。已合入 ≠ formal gate 已通过。验收不够先诊断再扩序列；实验默认本机，先说明问题、成本与停止条件。

## 现行工作区事实

- clangd：`.clangd` 中 `CompilationDatabase: build`；根 `compile_commands.json` → `build/compile_commands.json`（gitignore）。改 CMake/源后重新 `cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`。单一根 `CMakeLists.txt`，开启 `-Wconversion -Wsign-conversion -Wpedantic`。
- 格式：根 `.clang-format`；命名/风格权威见上表。依赖细节见各模块 `AGENTS.md`。
- 当前 main 基线：`#47` mapped-landmark bearing continuity（`db22656` / merge `34c3091`，`default_0337287b`）；EuRoC-11 算术均值 ATE ≈0.301。已合入但 formal gate 仍 FAIL（V2_03 endpoint coverage）；V2_03 末端长期 initializing、未建新 root。旧 1a NavState handoff 不可原样恢复。下一片 `#48` cold-root seed Observe（`#42` child），默认不改 active-map 语义。产品合同：双目尺度已知、body≡IMU、单一 `VioEstimator::update()`、滑窗 10、视觉短时丢失最多 500 ms IMU coast。M4 最小 full-state 历史锚点：`c999f58`。
- `LandmarkId` 现为 frontend track 与 estimator map 共用身份；TrackId≠LandmarkId 为中期债。前端 OpenCV、后端 GTSAM，自研以调库版为对拍 oracle。
- Issues：`origin` = `Nothand0212/phad-vio`；`GITHUB_TOKEN` 无 Issues 写权限时用 `env -u GITHUB_TOKEN gh ...`。
- Remote CI SSH 身份读本机共享配置：`PHAD_REMOTE_CI_CONFIG`，否则 `$XDG_CONFIG_HOME/phad-remote-ci/config.json`，再否则 `~/.config/phad-remote-ci/config.json`；缺文件即失败。仓库只留 `REMOTE_ROOT` / 数据集路径。
