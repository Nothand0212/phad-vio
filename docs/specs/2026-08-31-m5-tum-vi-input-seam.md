# M5 Slice A：TUM VI product input seam

- 日期：2026-08-31（Asia/Shanghai）
- 状态：**已定稿**（用户于 2026-08-31 确认 concrete signature、test target 与执行细节）
- Issue：[#53](https://github.com/Nothand0212/phad-vio/issues/53)
- Wayfinding map：[#42](https://github.com/Nothand0212/phad-vio/issues/42)
- 产品基线：`main@46a84b589a2ca49d56f8502641abd5897f3bb39e`
- 产品代码锚点：`34c309196440710be86d0a05e32901d58bfdd9aa`
- 决策依据：[M5 产品切片能力与回归测试迁移图](../research/2026-08-31-note-m5-product-slice-capability-test-map.md)

本文冻结 #53 的产品行为与验收合同；实现与验证必须从本文及对应 implementation plan 派生。

## 1. 目标

让 TUM VI calibrated raw stereo 进入现有产品 pipeline：app composition root 显式打开 concrete dataset adapter，把格式中立的 `StereoImuDataset` 交给同一个 `OfflineVoSession`；camera module 将 equidistant `uint16` raw stereo 转换为 frontend 可消费的 rectified `uint8` stereo。

本片结束时，TUM VI 与 EuRoC 共用：

```text
concrete adapter open
  → StereoImuDataset
  → DatasetReplaySource
  → StereoPairStream / sync
  → StereoRectifier
  → StereoTracker
  → existing estimator call
```

成功只表示 product input seam 可达，不表示 estimator 已建立 initial moving root。Initial moving root 由 [#51](https://github.com/Nothand0212/phad-vio/issues/51) 单独交付。

## 2. 模块与 interface

### 2.1 Dataset adapter selection

Concrete adapter selection 属于 app composition root。Caller 必须显式执行：

```cpp
auto dataset = phad::io::dataset::euroc::open( sequence_root );
// 或
auto dataset = phad::io::dataset::tum_vi::open( sequence_root );
```

不得根据目录内容猜测格式，也不得在 `OfflineVoSession` 内建立 adapter enum/switch 或转发 facade。

### 2.2 Offline session seam

`OfflineVoSession` 按值接收已打开的 immutable dataset handle：

```cpp
[[nodiscard]] OfflineVoSessionResult runOfflineVoSession(
    io::dataset::StereoImuDataset dataset,
    const OfflineVoSessionOptions& options );
```

合同如下：

- `OfflineVoSessionOptions::sequence_root` 删除；不保留 overload、compatibility wrapper 或 fallback。
- Session 在本次同步调用期间拥有 dataset handle，并从它构造现有 `DatasetReplaySource(const StereoImuDataset&)`；不修改 source seam。
- 现有 EuRoC app / bench callers 在 composition root 完成 `euroc::open()` 后调用 session。
- TUM VI opt-in product test 在 test composition root 完成 `tum_vi::open()` 后调用同一 session。
- Session 不计算 ATE/RPE、不加载 ground truth、不猜数据集类型，也不新增持久产物 schema。

### 2.3 Stereo rectifier seam

`StereoRectifier` public interface 保持不变：

```cpp
auto rectifier = StereoRectifier::create( stereo_imu_calibration );
auto rectified = rectifier.value().rectify( raw_stereo_frame );
```

调用方只学习“支持的 raw 灰度双目输入 → rectified `uint8` stereo”合同；OpenCV model dispatch、map、native-depth remap 与 pixel canonicalization 全部属于 PIMPL / `.cpp` implementation。

## 3. Calibration 与 geometry

### 3.1 支持的 model pairing

左右相机必须同时为下列同一种 model：

- `PinholeRadialTangentialParameters`；
- `PinholeEquidistantParameters`。

Mixed model pair、左右 image size 不同或非正 image size 返回 `kOutsideModelDomain`。不增加 model conversion、automatic approximation 或 caller-provided rectification policy。

### 3.2 Radtan control

Radtan path 保持现有行为：

- `cv::stereoRectify(..., CALIB_ZERO_DISPARITY, alpha=0)`；
- `cv::initUndistortRectifyMap()`；
- `cv::remap(..., INTER_LINEAR, BORDER_CONSTANT)`；
- `uint8` input 以 `CV_8UC1` remap 并产出 `uint8`。

现有 `RectifiedStereoCalibration`、baseline 与 `T_B_left_rectified` 方向合同不变。

### 3.3 Equidistant path

Equidistant path 复用 OpenCV fisheye implementation：

- `cv::fisheye::stereoRectify()`，flags 为 `CALIB_ZERO_DISPARITY`；
- output size 等于 calibrated raw image size；
- `balance=0.0`、`fov_scale=1.0`；
- `cv::fisheye::initUndistortRectifyMap()`；
- `cv::remap(..., INTER_LINEAR, BORDER_CONSTANT)`。

OpenCV 的 `R,T` 继续表达 left-camera → right-camera：`p_right = R * p_left + T`。`T_B_left_rectified` 仍由 left physical-camera extrinsic 与 `R1` 形成：

```text
R_left_left_rect = R1^T
T_B_left_rectified = T_B_left * T_left_left_rect
```

Output `fx/fy/cx/cy`、positive baseline、image size 与 `T_B_left_rectified` 必须 finite 且满足 `RectifiedStereoCalibration::create()`。

## 4. Pixel contract

### 4.1 Raw input

每侧 image 必须满足：

- width / height 与 rectifier calibration 精确相同；
- `channels == 1`；
- `PixelType` 为 `kUint8` 或 `kUint16`；
- 左右 `PixelType` 相同；
- typed pixel span 的 element count 精确等于 `width * height`。

`phad::io` 继续保留 source pixel depth；TUM VI adapter 产出 `uint16`，EuRoC adapter 产出 `uint8`。Dataset adapter、sync、session 与 frontend 不执行 raw depth conversion。

### 4.2 Native-depth remap

- `uint8` input：构造 `CV_8UC1`，remap 后直接构造 output `Image<uint8_t>`。
- `uint16` input：构造 `CV_16UC1`，先以 `CV_16U` 完成 remap，再量化 output。

不得在 geometric remap 前把 `uint16` 转为 `uint8`，也不得建立 float intermediate path。

### 4.3 `uint16 → uint8` mapping

每个 rectified `uint16` pixel 使用下式：

```cpp
const auto dst = static_cast<std::uint8_t>( src >> 8 );
```

等价数学合同：

```text
dst = floor(src / 256)
```

- 低 8 位直接截断；
- 合法 `uint16` 输入天然映射到 `[0,255]`，无需额外 saturation branch；
- 不使用 `cv::Mat::convertTo(CV_8U, 1.0 / 256.0)`，因为其 floating-to-integer 路径采用最近整数舍入，不等价于高位提取；
- mapping 不依赖 frame、sequence、histogram、percentile、exposure 或强度样本统计。

## 5. Error contract

保持 `CameraModelResult<T>` / `CameraModelError` 与现有三类 code；不新增 rectifier-specific public error taxonomy。

| 分类 | code | `detail` 必含上下文 |
|---|---|---|
| unsupported / mixed camera model、invalid image size | `kOutsideModelDomain` | `create` stage、expected / actual model 或 size |
| frame size、channel、left/right pixel type、typed-span count 不满足合同 | `kOutsideModelDomain` | `rectify` stage、`left` / `right`、expected / actual |
| OpenCV exception、non-finite projection、invalid rectified calibration | `kNumericalFailure` | stage、原始 library cause；适用时包含 side |

`detail` 是人类诊断文本，不冻结整句文案，不作为 parser、schema 或测试的唯一 oracle。OpenCV exception 保留原始 `what()` cause。合法 `uint16` 高位提取本身没有 recoverable conversion failure。

## 6. Session terminal semantics

`OfflineVoSessionResult::trajectory` 继续为 optional。

- Session pipeline 成功运行、但 estimator 在 bounded prefix 内始终 `kInitializing` 时，返回 `error = nullopt`、`trajectory = nullopt`。
- `kInvalidInput` / `kFailed` 继续成为 session hard error。
- 已产生 accepted poses 时，trajectory construction 失败仍为 session hard error。
- 需要轨迹的 `phad_stereo_vo_probe`、`phad_vo_bench` 等 composition root 在 session 返回后显式检查 `trajectory.has_value()`，缺失时保持失败。
- Session 不用 “no accepted poses” 伪装 pipeline error，也不为 input-seam gate 制造 identity / empty trajectory 成功值。

## 7. Diagnostics 与持久化

- 不新增 rectifier regular diagnostics、`VoDiagRow` fields、`diag.csv` columns 或 `summary.json` fields。
- `timing.rectify` 继续记录整个 native-depth remap + output canonicalization 的耗时。
- Raw `uint16`、rectified `uint8` 与 frontend consumption 由定向 tests / opt-in acceptance evidence 证明。
- 强度分布可在失败诊断中观察，但不改变 mapping，不进入长期 schema。

## 8. 验收合同

### 8.1 Deterministic camera tests

必须覆盖：

1. Equidistant stereo synthetic projection / rectification oracle；至少 20 个可见对应点，且每个校正后对应点满足 `abs(y_left - y_right) <= 1.0 px`。
2. Positive baseline、finite rectified intrinsics、image size 与 `T_B_left_rectified` 方向。
3. Identity / controlled mapping 下的 high-byte values：`0→0`、`255→0`、`256→1`、`511→1`、`512→2`、`65280→255`、`65535→255`。
4. 至少一个 fractional remap case 区分“native `uint16` interpolation 后 shift”与“先 shift 后 interpolation”。
5. Mixed model、mismatched size、multi-channel、left/right mixed pixel type 与 pixel-count mismatch 的 typed failure。
6. Public test surface 只经过 `StereoRectifier::create()` / `rectify()`；不公开 production injection seam。

### 8.2 EuRoC control

- 现有 zero-distortion / pure-translation identity mapping 保持 byte-equivalent。
- `phad_camera_mh01_test` 的 row alignment、baseline 与 extrinsic checks 通过。
- `phad_frontend_mh01_test` 通过；radial-tangential / `uint8` product path 无行为回归。
- Session signature migration 后，现有 EuRoC apps/tests 的 observable result 不变。

### 8.3 TUM VI bounded product gate

Opt-in gate 复用：

```text
PHAD_ENABLE_TUMVI_CORRIDOR1_TESTS=ON
PHAD_TUMVI_CORRIDOR1_PATH=<audited corridor1_512_16 root>
```

固定从首帧处理 100 个 stereo frames。必须满足：

- `session.error == nullopt`；
- `counts.image_frames == 100`；
- `diag.size() == 100`；
- `sync.emitted_stereo == 100`；
- `dropped_left == dropped_right == 0`；
- `dropped_left_overflow == dropped_right_overflow == 0`；
- `counts.failed == 0`；
- `sum(diag.num_observations) > 0`；
- `sum(diag.num_disparity) > 0`。

Trajectory presence、`counts.ok`、initialization verdict、ATE/RPE、coverage 与 completion 不参与本片判定。

固定 prefix 首次失败时必须保存公开结构字段与 test output，定位最早失败边界；不得在同一实现轮静默延长 prefix、换序列、改变 high-byte mapping 或追加经验 coverage threshold。

## 9. Scope boundary

本片不交付：

- initial moving root、post-outage cold-root recovery 或 cross-root world-frame continuity；
- gyro-bias、gravity、velocity candidate solve 或 estimator diagnostics redesign；
- full photometric response / vignette calibration；
- per-frame / per-sequence adaptive intensity normalization；
- TUM VI ground truth、trajectory、ATE/RPE 或 full-sequence benchmark；
- second backend、second session pipeline、new persistent diagnostics schema 或 compatibility layer。

## 10. 完成定义

仅当以下条件同时满足，#53 才可形成 checkpoint：

1. §2–§7 product contracts 已实现，public headers 不暴露 OpenCV。
2. §8.1 deterministic tests 全部通过。
3. §8.2 EuRoC control 全部通过。
4. §8.3 TUM VI 100-frame product gate 通过。
5. 受影响模块 README / AGENTS 与 capability map 回填实际行为；验证命令、版本、dataset identity 与结果进入 checkpoint。
6. Diff 完整审查，未混入与 #53 无关的工作区修改。
