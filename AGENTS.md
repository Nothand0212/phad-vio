## Agent skills

### Issue tracker

Issues and PRDs are tracked in GitHub Issues. See `docs/agents/issue-tracker.md`.

### Triage labels

Use the five default canonical triage labels. See `docs/agents/triage-labels.md`.

### Domain docs

This is a single-context repository. See `docs/agents/domain.md`.

### C++ naming

C++ identifiers follow project naming rules. See `docs/agents/cpp-naming.md`.

## Learned User Preferences

- Prefer Chinese for agent-facing docs under `docs/agents/`; keep English for technical terms and code identifiers
- Persist Cursor rules that agents should follow into `docs/agents/` and link them from `AGENTS.md`
- Prefer clangd for C++ go-to-definition and references; keep Microsoft C/C++ IntelliSense disabled to avoid conflicts

## Learned Workspace Facts

- clangd is configured via `.clangd` with `CompilationDatabase: build`; root `compile_commands.json` is a symlink to `build/compile_commands.json` and is gitignored
- After CMake or source-file changes, regenerate with `cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
- Core `sensor::*` and calibration types stay POD + STL (`std::array`); Eigen/GTSAM are deferred past the EuRoC loader (M1) phase
- C++ naming authority is `docs/agents/cpp-naming.md`, mirrored by `.cursor/rules/cpp-naming.mdc`

