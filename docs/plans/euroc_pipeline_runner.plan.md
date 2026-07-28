# EuRoC Pipeline 可视化 Runner

## Summary

新增 `phad_euroc_runner`，从命令行指定的 EuRoC sequence root 读取数据，经
`DatasetReplaySource` 的 `SensorSource` 接口完整拉取事件流；忽略 IMU
显示但确保其经过 pipeline，收到双目帧后将左右灰度图水平拼接并通过
`cv::imshow` 实时播放。

## Implementation Changes

- 新增 `apps/phad_euroc_runner.cpp`，用法为
  `phad_euroc_runner <sequence-root>`；保留现有 `phad_euroc_inspect`。
- 通过 `euroc::open()` 打开数据集，移动构造 `DatasetReplaySource`，后续
  只依赖 `SensorSource&::next()`，不绕过 source 直接调用 `loadStereo()`。
- 明确分派 `ImuMeasurement`、`StereoFrame`、`EndOfStream` 和
  `SensorSourceError`；只对双目帧执行拼接和显示。
- 按双目时间戳与 wall clock deadline 实时节流；解码耗时超过帧间隔时立即
  显示下一帧，不额外累积延迟。支持 `Esc`、`q` 或关闭窗口提前正常退出。
- 对非单通道、非 `uint8`、左右尺寸不一致等意外输入显式报错，不进行隐式
  转换或静默降级。
- 更新 CMake，为 OpenCV 增加 `core`、`highgui` component，新增并链接
  `phad_euroc_runner` target；不修改公共 API。

## Test Plan

- 重新执行 CMake 配置并构建 `phad_euroc_runner`。
- 运行默认单元测试，确认 `SensorSource`、EuRoC loader 和现有 dataset
  行为无回归。
- 验证无参数调用打印 usage 并返回参数错误。
- 使用真实 EuRoC sequence 人工验证双目拼接、实时播放、提前退出和 EOF。
- 执行 `git diff --check` 并检查最终差异。

## Assumptions

- 输入路径是单个 EuRoC sequence root，即包含 `mav0/` 的目录。
- 首版只显示原始双目灰度图，不校正、缩放、叠加时间戳或保存视频。
- “实时连续”采用数据集双目时间戳节奏，首帧立即显示。
- 保留现有未提交的 `SensorSource`/`DatasetReplaySource` 工作并在其上增加
  runner。
