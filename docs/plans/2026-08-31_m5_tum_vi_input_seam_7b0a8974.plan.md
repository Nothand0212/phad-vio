---
name: M5 Slice A TUM VI input seam
overview: 以公开 StereoRectifier 与 OfflineVoSession seams 逐个完成 RED→GREEN，交付 equidistant uint16 raw stereo 到 rectified uint8 frontend 的真实产品输入闭环。
todos:
  - id: freeze-controls
    content: 核对 #53 authority、工作区与现有 EuRoC camera/session controls，记录定向构建和测试基线
    status: pending
  - id: rectify-uint16
    content: 先写 high-byte 与 native-depth-order RED，再让 StereoRectifier 支持 uint16 输入并保持 radtan uint8 control
    status: pending
  - id: rectify-equidistant
    content: 先写 equidistant geometry/error RED，再接入 OpenCV fisheye maps、校正标定与 typed failure
    status: pending
  - id: inject-dataset
    content: 先写 session/caller RED，再把 StereoImuDataset 显式传入 OfflineVoSession 并收口 no-trajectory terminal semantics
    status: pending
  - id: tumvi-product-gate
    content: 新增 corridor1_512_16 首 100 帧 opt-in product test，执行固定结构 predicate 并保留失败证据
    status: pending
  - id: regression-review-checkpoint
    content: 运行 unit、MH_01、TUM VI gates 与静态检查，审查 diff，回填模块文档和 #53 checkpoint
    status: pending
isProject: false
---

# M5 Slice A：TUM VI product input seam implementation plan

- Plan ID：`7b0a8974`
- Issue：[#53](https://github.com/Nothand0212/phad-vio/issues/53)
- Wayfinding map：[#42](https://github.com/Nothand0212/phad-vio/issues/42)
- Spec：[2026-08-31-m5-tum-vi-input-seam.md](../specs/2026-08-31-m5-tum-vi-input-seam.md)
- Confirmed map：[2026-08-31-note-m5-product-slice-capability-test-map.md](../research/2026-08-31-note-m5-product-slice-capability-test-map.md)
- Product baseline：`main@46a84b589a2ca49d56f8502641abd5897f3bb39e`

状态：**已定稿**（用户于 2026-08-31 确认；后续实施按 todos 与 RED→GREEN 顺序执行）。

## 1. 唯一产品 RED

TUM VI adapter 已能产出 equidistant calibration 与 `uint16` raw images，但真实产品 caller 尚不能经共同 session 把这些 frames 送入 frontend：

```text
app concrete open
  → StereoImuDataset
  → OfflineVoSession
  → replay / sync
  → StereoRectifier
  → StereoTracker
```

当前最早分叉是 app/session 固定 EuRoC open；直接 TUM VI path 随后在 rectifier 的 equidistant / `uint16` 合同处失败。本计划只闭合这条 input seam。Estimator initial moving root、trajectory 与 ATE 属 #51。

## 2. 已确认 seams

Tests 只经过以下 public interfaces：

1. `StereoRectifier::create(calibration)` 与 `StereoRectifier::rectify(raw_frame)`；
2. `runOfflineVoSession(StereoImuDataset, OfflineVoSessionOptions)`；
3. app / test composition root 的 `euroc::open()` 或 `tum_vi::open()`。

不直接测试 anonymous helpers、OpenCV map storage、private PIMPL state 或内部调用次数。不为 fault injection 新增 production seam。

## 3. 实施分片与 commits

| commit | vertical slice | subject |
|---|---|---|
| 1 | Camera RED→GREEN：native-depth remap、high-byte mapping、equidistant geometry 与 errors | `camera: support TUM VI stereo rectification (#53)` |
| 2 | Session RED→GREEN：显式 dataset dependency 与 empty-trajectory semantics | `apps: inject datasets into offline session (#53)` |
| 3 | 真实 prefix gate、回归、文档与 checkpoint | `tests: gate TUM VI product input seam (#53)` |

每个 commit 前必须对应 tests GREEN、diff 聚焦且可独立 review。不得把三个 commits 压成一次大改，也不得在 RED 尚未成立时预写后续 implementation。

## 4. Step 0：冻结 control

### 4.1 工作区与 authority

实施应从 latest main 创建短生命周期 `codex/m5-tum-vi-input-seam` 分支；若设计文档尚未进入 main，先以已确认 spec/map 的 commit 作为明确起点。禁止在包含其他未提交产品修改的树上开始机械迁移。

开始前记录：

```bash
git status --short --branch
git rev-parse HEAD
git diff --check
```

核对 #53、spec、confirmed map、camera/io/apps/tests 模块 README/AGENTS 与当前 CMake。

### 4.2 Control commands

在任何产品修改前配置 compile database，并运行当前可达 controls：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DPHAD_BUILD_TESTS=ON
cmake --build build --target phad_camera_tests phad_apps_tests phad_frontend_tests -j2
./build/phad_camera_tests
./build/phad_apps_tests
./build/phad_frontend_tests
```

若本地 MH_01 配置可用，再记录当前 `phad_camera_mh01_test` 与 `phad_frontend_mh01_test` control；缺少 dataset 时明确记为未运行，不伪造 PASS。

停止条件：现有定向 unit control 不通过时，先定位 baseline/环境问题，不开始 #53 implementation。

## 5. Cycle 1：high-byte 与 native-depth remap

### 5.1 RED

在 `tests/camera/stereo_rectifier_test.cpp` 通过 public rectifier seam 增加最小失败测试：

1. Identity / controlled remap 接受 `uint16` left/right；
2. output pixel type 为 `uint8`；
3. literal boundaries 精确满足：

```text
0→0, 255→0, 256→1, 511→1, 512→2, 65280→255, 65535→255
```

4. 一个 fractional-map fixture 必须区分：

```text
uint16 remap → shift
```

与：

```text
shift → uint8 remap
```

Expected values 使用已算好的 literal 或 test-owned OpenCV `CV_16U` reference path 后显式 shift；不得调用 production helper 或按 production 代码重算。

先运行单 test，确认失败原因是当前 `uint8`-only interface，而不是 fixture / calibration 无效。

### 5.2 GREEN

修改 `phad/camera/stereo_rectifier.cpp` 的 private image↔Mat conversion：

- 支持 `CV_8UC1` 与 `CV_16UC1`，保持 source depth；
- 左右 frame 必须 single-channel、同 size、同 pixel type；
- remap 后由单一 private output path 构造 `Image<uint8_t>`；
- `uint16` 使用显式 typed loop `static_cast<uint8_t>(src >> 8)`；
- 不使用 `convertTo(..., 1.0 / 256.0)`、float intermediate、histogram 或 adaptive mapping。

只实现使本 cycle tests GREEN 的代码；此时不接 equidistant geometry。

### 5.3 验证

```bash
cmake --build build --target phad_camera_tests -j2
./build/phad_camera_tests --gtest_filter='StereoRectifierTest.*Uint16*'
```

GREEN 后运行完整 `phad_camera_tests`，确认现有 radtan / `uint8` tests 不变。

## 6. Cycle 2：equidistant geometry 与 errors

### 6.1 RED — model dispatch

把现有 `RejectsEquidistantCalibration` 改写为 success contract，并增加：

- left/right equidistant pair 成功 create；
- mixed radtan/equidistant pair 为 `kOutsideModelDomain`；
- mismatched calibrated size 为 `kOutsideModelDomain`。

### 6.2 RED — independent geometry oracle

使用 project `CameraModel::project()` 从已知 3D points 生成 left/right equidistant raw observations 或 rasterized markers；rectification 后通过独立 peak / correspondence extraction 检查：

- 至少 20 个可见对应点，且每个 `abs(y_left - y_right) <= 1.0 px`；
- disparity sign 与 positive baseline 一致；
- output calibration finite、size 正确；
- `T_B_left_rectified` 与 `R1` 方向合同一致；
- non-identity physical extrinsic 不被当作 identity。

Expected geometry 不读取 PIMPL maps，不复用 production `fromGrayMat` 或 output calibration builder。

### 6.3 RED — image contract

Public `rectify()` tests 覆盖：

- raw size mismatch；
- multi-channel metadata；
- left/right mixed pixel type；
- metadata/pixel-count mismatch；
- stage/side/expected/actual detail context。

只断言 code 与必备 context，不冻结完整句子。

### 6.4 GREEN

在 `phad/camera/stereo_rectifier.cpp` 内：

- 对同-model pair 分派 radtan 或 fisheye implementation；
- 为 equidistant 参数构造 OpenCV K 与 `D=[k1,k2,k3,k4]`；
- 使用 `cv::fisheye::stereoRectify(..., CALIB_ZERO_DISPARITY, size, balance=0.0, fov_scale=1.0)`；
- 使用 `cv::fisheye::initUndistortRectifyMap()` 与既有 `remap`；
- 共用一条 P1/P2、baseline、`T_B_left_rectified` 校验与 output-calibration path；
- 捕获 `cv::Exception` 为 `kNumericalFailure` 并保留原始 cause。

Public header 不出现 OpenCV，也不新增 model-specific method。

### 6.5 验证与 commit 1

```bash
cmake --build build --target phad_camera_tests -j2
./build/phad_camera_tests
git diff --check
```

检查 spec 第 3–5 节全部满足后形成 commit 1。

## 7. Cycle 3：显式 dataset dependency

### 7.1 RED — session interface

在 `tests/apps/offline_vo_session_test.cpp` 先迁移一个最小 fixture：

```cpp
auto opened = io::dataset::euroc::open( fixture.root() );
auto result = runOfflineVoSession( std::move( opened ).value(), options );
```

RED 应来自旧 signature 仍要求 `options.sequence_root`。随后逐 test 迁移；不要一次写完所有 tests 后再实现。

### 7.2 GREEN — session ownership

修改：

- `apps/offline_vo_session.hpp/.cpp`；
- `apps/phad_stereo_vo_probe.cpp`；
- `apps/phad_vo_bench.cpp`；
- 相关 tests / helpers。

具体合同：

- `OfflineVoSessionOptions::sequence_root` 删除；
- `runOfflineVoSession(StereoImuDataset dataset, const Options&)` 接收 value handle；
- session 在调用期持有 dataset，并用现有 `DatasetReplaySource(const StereoImuDataset&)` 构造 source；不修改 source seam；
- composition roots 显式 `euroc::open()` 并保留当前 error/exit/output policy；
- 不保留旧 overload 或 wrapper。

旧 `OfflineVoSessionTest.MissingSequenceReturnsError` 不再属于 session seam：删除或迁到已有 adapter / composition-root test，不通过兼容入口维持它。

### 7.3 RED→GREEN — no-trajectory terminal semantics

先增加 public session test：合法 dataset/prefix 全程 `kInitializing` 时，

```text
error == nullopt
trajectory == nullopt
```

然后删除 `poses.empty() → SessionError("no accepted poses to write")`。保留：

- accepted poses 存在时的 `Trajectory::create()` error；
- probe / bench 对 `trajectory.has_value()` 的显式失败；
- `kInvalidInput/kFailed` session hard-stop。

### 7.4 验证与 commit 2

```bash
cmake --build build --target phad_apps_tests phad_bench_tests -j2
./build/phad_apps_tests
./build/phad_bench_tests
git diff --check
```

逐 caller 检查 missing dataset、session error、missing trajectory 与成功 trajectory 的 observable policy。GREEN 后形成 commit 2。

## 8. Cycle 4：TUM VI 100-frame product gate

### 8.1 Test target

在现有 `PHAD_ENABLE_TUMVI_CORRIDOR1_TESTS` 选项下新增独立 executable，例如：

```text
phad_tumvi_corridor1_product_test
```

它链接真实 product modules：`phad::io_dataset`、`phad_offline_vo_session` 及其 transitive camera/frontend/estimator dependencies；CTest label 继续使用 `tumvi-corridor1`。不复制 session pipeline。

### 8.2 RED

Test composition root：

1. 从 `PHAD_TUMVI_CORRIDOR1_PATH` 调用 `tum_vi::open()`；
2. 设置 `max_frames=100`；
3. 调用共同 `runOfflineVoSession()`；
4. 断言 spec 第 8.3 节的所有 structural predicates。

旧代码应先因 equidistant/session seam 不可达而 RED。若 RED 来自 dataset identity 变化，停止并核对已有 corridor1 audit，不修改 product contract。

### 8.3 GREEN / stop rule

只修复使固定 100-frame path 达到 spec 的 in-scope defect。首次运行若在公开 predicate 上失败：

- 保存 gtest output、status histogram、sync fields、observation/disparity sums 与 first failing stage；
- 一次只诊断最早分叉；
- 不延长 prefix、不换 sequence、不调 frontend/estimator threshold、不增加 regular diagnostics。

产品 gate 不要求 trajectory、`kOk`、ATE/RPE、completion 或 coverage。

### 8.4 验证与 commit 3

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DPHAD_BUILD_TESTS=ON \
  -DPHAD_ENABLE_TUMVI_CORRIDOR1_TESTS=ON
cmake --build build --target phad_tumvi_corridor1_test \
  phad_tumvi_corridor1_product_test -j2
PHAD_TUMVI_CORRIDOR1_PATH="$PHAD_TUMVI_CORRIDOR1_PATH" \
  ctest --test-dir build --output-on-failure -L tumvi-corridor1
```

只有 fixed prefix GREEN 后形成 commit 3。

## 9. Regression ladder

按风险由小到大执行：

### 9.1 Unit

```bash
cmake --build build --target phad_sensor_tests phad_camera_tests \
  phad_io_dataset_tests phad_sync_tests phad_frontend_tests \
  phad_estimator_tests phad_apps_tests phad_bench_tests -j2
ctest --test-dir build --output-on-failure -L unit
```

### 9.2 EuRoC MH_01 opt-in control

仅在本地 audited path 可用时重新配置：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DPHAD_BUILD_TESTS=ON \
  -DPHAD_ENABLE_MH01_TESTS=ON
cmake --build build --target phad_camera_mh01_test \
  phad_frontend_mh01_test phad_apps_mh01_test -j2
PHAD_EUROC_MH01_PATH="$PHAD_EUROC_MH01_PATH" \
  ctest --test-dir build --output-on-failure -L mh01
```

Control 要求现有 radtan / `uint8` reference behavior 不变；任何差异先定位 camera path，不扩 TUM VI scope。

### 9.3 Static / diff checks

```bash
git diff --check
git status --short
```

对受影响 C++ files 运行项目 `.clang-format`；刷新 `build/compile_commands.json`。执行 Standards/Spec review，确认：

- public header 无 OpenCV；
- 无 concrete adapter switch 进入 session；
- 无 pre-remap `uint16→uint8`；
- 无 compatibility overload、第二 pipeline 或新 persistent schema；
- unrelated worktree changes 不进入 commits / handoff。

## 10. Docs 与 checkpoint

实现稳定后更新：

- `phad/camera/README.md`：equidistant whole-image support 与 raw→rectified pixel contract；
- `phad/io/README.md`：raw depth ownership 不变；
- `apps/AGENTS.md`：composition root 打开 concrete adapter，session 接收 dataset；
- 相关 module AGENTS（仅持久约定变化时）；
- confirmed capability/test map：回填 actual files/commands/results；
- `docs/design/roadmap.md`：只回填可观察 product checkpoint；
- `docs/benchmark/m5/...`：记录 clean commit、dataset identity、commands、100-frame predicates、EuRoC controls 与未运行项。

Checkpoint 形成后在 #53 评论结果并关闭 issue；随后更新 #42 progress。#51 的 TUM VI product verification 才取得前置条件。

## 11. 明确不做

- 不实现 #51 initial moving root；
- 不增加 TUM VI GT/ATE/RPE/full-sequence benchmark；
- 不增加 vignette/response calibration 或 adaptive normalization；
- 不调 frontend/estimator thresholds；
- 不新增 backend、session pipeline、persistent diagnostics schema 或 compatibility layer；
- 不 push、PR、merge，除非届时用户另行授权。
