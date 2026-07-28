# 通用 Stereo-IMU 数据集与 EuRoC/TUM VI Adapter

> 模块归属已由
> [`dataset_io_module_relocation.plan.md`](dataset_io_module_relocation.plan.md)
> 修正：本文中的 dataset 公开接口现位于 `phad::io::dataset`。

## Summary

1. 引入统一的 `StereoImuDataset` 值类型。
2. 将 EuRoC 与 TUM VI 实现为独立 adapter，均返回该通用类型。
3. 保留 `EurocDataset` 兼容入口；不使用虚基类、模板特化或自动格式探测。

## Interface Changes

- 新增 `StereoImuCalibration` 与 `StereoImuDataset`，统一提供：
  - `calibration()`
  - `imuMeasurements()`
  - `stereoIndex()`
  - `loadStereo(index)`
- 新增：
  ```cpp
  phad::io::dataset::euroc::open(sequence_root);
  phad::io::dataset::tum_vi::open(sequence_root);
  ```
  均返回 `DatasetResult<StereoImuDataset>`。
- 保留 `EurocDataset::open()` 和现有访问方法，通过组合委托给通用对象。
- `EurocCalibration` 改为 `StereoImuCalibration` 的兼容别名。
- `PixelType` 增加 `kUint16`；`Image` 使用 uint8/uint16 variant，并提供：
  ```cpp
  template<typename T>
  std::optional<std::span<const T>> pixels() const noexcept;
  ```
- `DistortionModel` 增加 `kEquidistant`。

## Implementation

- 通用模块集中负责规范化数据所有权、双目索引访问和类型严格的惰性图像解码。
- 内部共享 CSV、整数时间戳、严格排序、安全路径、exact stereo join、错误构造及刚体变换验证；格式专属 YAML 解析留在各 adapter。
- EuRoC adapter 保持当前目录、标定、uint8 图像和 identity IMU extrinsics 合同。
- TUM VI adapter：
  - 读取 `mav0/{cam0,cam1,imu0}/data.csv`；
  - 从 `dso/camchain.yaml` 读取相机标定；
  - 计算 `T_B_camera = inverse(T_cam_imu)`，令 body 与 IMU frame 重合；
  - 从 `dso/imu_config.yaml` 读取频率及 noise 参数；
  - 使用 20 Hz camera、200 Hz IMU、equidistant distortion 和 uint16 图像合同。
- 不纳入 mocap、曝光时间、vignette、photometric response、去畸变、同步插值或自动格式探测。
- 更新 CMake、现有 inspector、架构与路线图文档；保留无关工作区修改。

## Test Plan

- 通用模块覆盖 uint8/uint16 无损解码、typed pixels、错误像素类型、尺寸、损坏图像、越界和内存所有权。
- 现有 EuRoC 测试迁移到新 adapter，并增加兼容 facade smoke test。
- 新增 TUM VI 小型 fixture，覆盖标定映射、外参求逆、noise、exact join、uint16 解码及主要失败路径。
- 可选真实数据集测试：
  - MH_01：3682 个 stereo frames、36820 条 IMU；
  - corridor1：5990 个 stereo frames、59721 条 IMU，抽样图像为 `512×512 CV_16UC1`。
- 重新生成 CMake、编译并运行默认测试及两个真实数据集集成测试；当前 19 个默认测试必须保持通过。

## Assumptions

- TUM VI 图像保持原始 16-bit 线性强度，不在 adapter 中降为 uint8。
- Kalibr `T_cam_imu` 表示 IMU 到 camera 的变换，需要求逆后写入项目的 `T_B_camera`。
- 格式由调用方显式选择，选择错误时返回带路径上下文的错误。
