# 将 Dataset 迁移至 IO 模块

## Summary

修复 dataset 的模块归属：离线 dataset 是 `io` 的一种输入形式，不应作为与
`io` 平级的顶层模块。

将现有实现从 `phad/dataset` 迁移至 `phad/io/dataset`，同步迁移 C++
namespace、include path、CMake target、测试和文档。保持数据加载、校验、
惰性解码及错误行为不变。

## Interface Changes

- 公开类型迁移至 `phad::io::dataset`：
  - `StereoImuDataset`
  - `StereoImuCalibration`
  - `StereoFrameRef`
  - `DatasetResult<T>`、`DatasetError` 和 `DatasetErrorCode`
  - `EurocDataset` 兼容 facade
- Adapter 入口改为：
  ```cpp
  phad::io::dataset::euroc::open(sequence_root);
  phad::io::dataset::tum_vi::open(sequence_root);
  ```
- 公开 include path 改为：
  ```cpp
  #include "phad/io/dataset/stereo_imu_dataset.hpp"
  #include "phad/io/dataset/euroc/euroc_dataset.hpp"
  #include "phad/io/dataset/tum_vi/tum_vi_dataset.hpp"
  ```
- CMake 实体 target 改为 `phad_io_dataset`，公开 alias 改为
  `phad::io_dataset`。
- 直接删除旧 `phad::dataset`、`phad/dataset/*` 和 `phad::dataset` CMake
  target，不提供 forwarding header 或 namespace alias。
- `phad::sensor` 核心数据类型保持不变。

## Implementation Changes

- 整体移动实现至 `phad/io/dataset`，内部 namespace 对齐为
  `phad::io::dataset::internal`；测试移动至 `tests/io/dataset`。
- 更新 inspector、测试、CMake source list、链接目标及全部 include/namespace
  引用；单元测试 target 同步改名为 `phad_io_dataset_tests`。
- 在架构文档中明确：
  ```text
  phad::io
  ├── dataset
  ├── serial        # future
  ├── ros           # future
  └── SensorSource  # future seam
  ```
  `phad::sensor` 仅保存来源无关的数据合同。
- 更新 README、roadmap、C++ naming 示例及 Cursor rule 镜像。对旧 dataset
  调研和原 M1 计划增加“模块路径已被本计划取代”的说明，避免历史文档继续
  被当作当前命名依据。
- 本次不新增 `SensorSource`、`DatasetReplaySource`、虚基类、streaming、
  背压或 ROS/串口实现；不改变随机访问和惰性图像解码合同。
- 不顺带重构 parser、错误模型、像素所有权或 adapter 实现。

## Test Plan

- 使用 `rg` 检查活动代码、构建配置和当前架构文档中不存在旧
  `phad/dataset`、`phad::dataset`、`phad_dataset` 和 `tests/dataset` 引用；
  允许历史文档中的明确迁移说明。
- 重新生成 `build/compile_commands.json`，编译 `phad_io_dataset`、inspector
  和全部测试。
- 运行现有 23 个 unit tests，确认 EuRoC/TUM VI 的正常路径、校验失败、
  错误上下文、越界和 uint8/uint16 解码行为无回归。
- 启用并运行真实数据集集成测试：
  - EuRoC MH_01：3682 stereo frames、36820 IMU samples；
  - TUM VI corridor1：5990 stereo frames、59721 IMU samples；
  - 验证抽样图像仍分别为 uint8 和 uint16。
- 对移动后的 C++ 文件执行 `clang-format --dry-run --Werror`，并执行
  `git diff --check`。
- 检查最终差异以确认主要变化为 rename、namespace/include/CMake 引用迁移，
  没有非预期行为修改。

## Assumptions

- 当前基线为提交 `1af2e50`，迁移被视为早期项目中的一次有意公开接口破坏。
- `io` 是外部数据进入或离开系统的顶层基础设施模块；dataset 是其离线输入
  子模块。
- CMake 使用 `phad::io_dataset`，避免把多层 C++ namespace 机械映射成多段
  `::` target 名。
- 不执行 commit、push、rebase 或其他远程操作。
