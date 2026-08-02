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

### Git workflow

短生命周期分支；合入 `main` 必须 `--no-ff` 保留 merge 图。详见
`docs/agents/git-workflow.md`。

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
| [`docs/AGENTS.md`](docs/AGENTS.md) | research / plans / architecture 文档流 |

## Learned User Preferences

- `AGENTS.md` 与 `docs/agents/` 下面向 agent 的文档优先使用中文；专有名词、技术术语与 code identifiers 保留英文
- 跨库规则持久化到 `docs/agents/` 并在根 `AGENTS.md` 链接；**模块作用域**约定写在对应目录 `AGENTS.md`，由根索引链接，勿把库细节堆进根文件
- C++ 的 go-to-definition 与 references 优先使用 clangd；禁用 Microsoft C/C++ IntelliSense 以避免冲突
- C++ identifiers 宜短，使用常见领域缩写（`imu`、`acc`、`gyr`、`nd`、`rw`、`fx`）；避免自造缩写与拼写出的单位后缀（细则见 `docs/agents/cpp-naming.md`）
- 在主 checkout 上使用短生命周期分支，而非 git worktrees；实施计划开工前先建 GitHub issue；按 vertical slice 逐片提交（commit subject 带对应 issue 号，如 `(#24)`；里程碑总图 issue 可留作 epic），本地 merge 到 `main` 时**必须** `git merge --no-ff`（禁止 fast-forward，以便 Git Graph 可见分叉/合回），再删除分支；除非明确要求否则不 push 到 remote。详见 `docs/agents/git-workflow.md`
- `phad/` 下每个库目录都要有 README（职责边界、文件布局、数据流、格式合同），顶部声明「本文档描述当前约定，不是绝对约束，会随项目开发修订」；`apps/`、`tests/` 与 adapter 子目录不单独写
- 每个 milestone 与 vertical slice 开工前先逐项对齐：一次只问一个问题、给 A/B/C 选项并标明推荐；默认先落地推荐项，验收不够再切备选；设计稿分段（模块边界 → 数据流 → 错误与诊断 → 测试与验收）逐段确认后再动手；底层代码已演进时，勿直接执行基于旧代码的设计/计划，应重新对齐或重写
- 重大设计决策前先检索开源参考实现（GTSAM examples、Kimera-VIO、ORB-SLAM3、VINS-Fusion），把对照笔记写进 `docs/research/` 再定方案；开源多为研究型代码，对照后以工程判断定案，勿盲抄其权宜实现；审阅设计文档时直接指出问题并与用户讨论改法，不擅自定稿
- 模块边界优先低耦合、高内聚、深模块与单向依赖；benchmark/编排放 composition root（见 `apps/AGENTS.md`），不反向注入 frontend/estimator
- 排障时先对照开源实现与本地清单，区分数据损坏与 loader 合同过严，勿直接断定数据源损坏
- EuRoC / milestone baseline 文档须含可复现的运行时参数快照（取自对应 run 的 `meta.json`：`config` / `config_canonical_text`，与 `config_hash` 同源），不能只写 hash 名；跨切片时标出相对上一基线的增量键；消化已知发散债时，编码后先跑诊断序列（如 MH_05），不够则停、不扩全序列

## Learned Workspace Facts

- clangd 通过 `.clangd` 配置，`CompilationDatabase: build`；根目录 `compile_commands.json` 是指向 `build/compile_commands.json` 的 symlink，且已被 gitignore；改 CMake 或源文件后用 `cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 重新生成；仓库只有单一根 `CMakeLists.txt`（无 per-module 版本），且开启 `-Wconversion -Wsign-conversion -Wpedantic`
- spdlog 1.17.0 备在 gitignore 的 `thirdparty/`，尚未接入；GTSAM / Eigen / nlohmann_json 等依赖细节见各模块 `AGENTS.md`（`estimator`、`sensor`、`bench` 等）
- C++ naming / style 权威为 `docs/agents/cpp-naming.md` 与 `docs/agents/cpp-style.md`（`.cursor/rules/` 镜像）；格式遵循根 `.clang-format`
- 里程碑顺序与出口以 `docs/roadmap.md` 为准：M1→M2.x→M3.1→M3.2 已完成；M3.3 Slice ①②③④d 已完成；④/④b MH_05 不够；④c（`block_culled_rebirth`+`dropTracks`）部分完成后由 ④d 收口；④e（`0ced28b`/`default_3a21162e`）MH_05 持平（≈4.565）→ 部分完成；④f（`c446ac5`/`default_a5e90dc7`，`skip_drop_min_culled=4`）MH_01 硬门 PASS、MH_05 软门 PASS（ATE ≈3.057 vs ④e ≈4.565）→ **已完成**、全序列未跑；`LandmarkId` 现为 frontend track 与 estimator map 共用身份，TrackId≠LandmarkId 为已确认的中期债；下一建议去 Huber 末轮 / 观测级外点续片（⑤ 门控已满足但与 M4 耦合，单独对齐后再开）；前端 OpenCV、后端 GTSAM，自研以调库版为对拍 oracle
- Issues 在 remote `origin`（`Nothand0212/phad-vio`）；`GITHUB_TOKEN` 指向的 fine-grained PAT 无 Issues 写权限时，用 `env -u GITHUB_TOKEN gh ...` 改走 keyring 凭据
