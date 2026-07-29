# SensorSource 与 DatasetReplaySource

## Summary

实现 pull-based `phad::io::SensorSource` seam 和离线
`DatasetReplaySource`。具体 adapter 处理来源差异，统一输出规范化
IMU/双目事件；本次不包含同步、线程、背压、节流或 VO 算法。

## Public Interface

- 将 `StereoImuCalibration` 的规范定义迁至 `phad::sensor`；在
  `phad::io::dataset` 保留同名 alias。
- 在 `phad/io/sensor_source.hpp` 定义 `SensorEvent`、`EndOfStream`、
  `SensorSourceError`、`SensorReadResult`，以及只开放 `calibration()` 和
  `next()` 的抽象 `SensorSource`。
- `next()` 返回拥有数据的值：IMU 按值复制，双目帧移动图像缓冲。
- 新增 CMake interface target `phad::io`；`phad::io_dataset` 公开依赖它。

## Implementation

- 新增不可复制、可移动的
  `phad::io::dataset::DatasetReplaySource final`，从
  `const StereoImuDataset&` 复制 calibration 并创建自持有的独立 reader；
  source 仅维护 reader、一条 IMU lookahead 与自身 terminal state。
- `next()` 输出 timestamp 较早的事件；timestamp 相同时固定先输出 IMU。
- stereo 仅在即将输出时调用 reader 的 `takeStereo()`，成功后才推进。
- 两个流耗尽后稳定返回 `EndOfStream`。
- reader error 转换为来源无关的 `kReadFailed`，保留 sensor id、timestamp、
  1-based record number 与原始 cause，不携带 filesystem path；此后重复
  返回相同 terminal error，不跳帧。
- 更新架构与路线图，标记 source seam 和 dataset replay 已落地。

## Test Plan

- 新增 `tests/io/dataset/dataset_replay_source_test.cpp`，通过公开
  `euroc::open()` 构造最小临时 dataset。
- 通过 `SensorSource&` 验证标定、全局事件顺序、相同 timestamp 的
  IMU-first 规则、无重复遗漏、惰性解码、terminal error、dataset 所有权、
  稳定 EOF 和空流。
- 重新生成 CMake，构建并运行默认单元测试及 EuRoC/TUM VI 真实数据集集成
  测试；执行 clang-format dry-run、`git diff --check` 和最终差异审查。

## Assumptions

- 首版为单线程、无节流的 pull replay，不增加 lifecycle、seek、callback、
  queue 或 backpressure。
- `SensorSource` 不执行 IMU 分段、边界插值或 `StereoImuPacket` 构造。
- adapter-specific 输入仍由各自构造或 `open()` 处理。
- dataset 使用不可变 handle 与独立顺序 reader；本计划不扩展 reader 的
  single-pass、校验、错误和像素所有权合同。
- 不执行 commit、push 或 PR。
