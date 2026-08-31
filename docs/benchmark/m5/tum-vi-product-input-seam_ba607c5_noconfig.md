# M5 TUM VI Product Input Seam（`noconfig`）

日期：2026-08-31

状态：**complete**；#53 的 bounded product-input checkpoint。它确认 TUM VI
equidistant `uint16` raw stereo 可通过共同的 offline session 到达 frontend。它不
衡量 trajectory、ATE/RPE、completion 或 full-sequence 表现，也不声明 initial moving
root 已实现。

相关：

- issue：[#53](https://github.com/Nothand0212/phad-vio/issues/53)
- spec：[M5 Slice A：TUM VI product input seam](../../specs/2026-08-31-m5-tum-vi-input-seam.md)
- implementation plan：[M5 Slice A implementation plan](../../plans/2026-08-31_m5_tum_vi_input_seam_7b0a8974.plan.md)
- capability/test map：[M5 产品切片能力与回归测试迁移图](../../research/2026-08-31-note-m5-product-slice-capability-test-map.md)

## 1. 身份与环境

| 项 | 值 |
|---|---|
| code commit | `ba607c5738b3c5d9bc9cb0423199eee67648b0b3`（short `ba607c5`；clean clone） |
| source tree | `7950f03a1125280169170513aad0a399198d0e76` |
| camera implementation | `bc07ff8` |
| session/composition implementation | `f41fe64` |
| build | RelWithDebInfo；GCC 13.3；CMake 3.31.11；OpenCV 4.6 |
| config identity | `noconfig`：test 不产生 `meta.json` 或与运行同源的 config hash |
| explicit options | `tum_vi::open()`；`max_frames=100`；`estimator.m_enable_moving_bootstrap=true` |
| TUM VI dataset | `/home/lin/Projects/data/thidparty/tum/dataset-corridor1_512_16` |
| TUM VI source identity | ordered manifest/calibration aggregate SHA256 `77f8f6a0951fb19e2951f48cfa9ab64fd025cac78da61fa6d526f45babc0984a` |
| manifest counts | left / right / IMU = `5990 / 5990 / 59721` |
| camera timestamp boundary | `1520531829251142058` → `1520532128710396829` ns |
| IMU timestamp boundary | `1520531829221612058` → `1520532128752735058` ns |

## 2. Acceptance 配置边界

该 product test 不是 `phad_vo_bench` run，不生成 `meta.json`、`summary.json` 或
`ConfigSnapshot`，因此不声明 config hash。可复现身份由 clean code commit 与
[`tum_vi_corridor1_product_test.cpp`](../../../tests/apps/tum_vi_corridor1_product_test.cpp)
共同固定。Test composition root 的显式选择只有：

```text
dataset_adapter=tum_vi::open
session.max_frames=100
estimator.m_enable_moving_bootstrap=true
```

`m_enable_moving_bootstrap=true` 复用 M4 canonical product path；其实现和默认值均未
在 #53 修改。其余 `OfflineVoSessionOptions`、`EstimatorOptions` 与
`StereoTrackerOptions` 使用 `ba607c5` 源码中的默认值。CMake option 与 dataset
environment variable 是 test-only 入口，同样不构成持久 config schema。

## 3. 实际验证

### 3.1 Clean-clone unit regression

在 `/tmp/phad53-clean-ba607c5.LKVWPm/repo` 的 detached clean clone 配置：

```bash
cmake -S . -B build-unit -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPHAD_BUILD_TESTS=ON
cmake --build build-unit --target phad_sensor_tests phad_camera_tests \
  phad_io_dataset_tests phad_sync_tests phad_frontend_tests \
  phad_estimator_tests phad_apps_tests phad_bench_tests -j2
ctest --test-dir build-unit --output-on-failure -L unit
```

配置和最终整组 targets build 均返回 0，`compile_commands.json` 已生成。CTest 返回
0：357/357 passed，0 failed，real time `34.14 s`（unit process time `33.51 s`）。
三个 opt-in MH 环境测试被 skipped：

- `OfflineVoSessionTest.EmptyProbeBPathDoesNotCreateFile`
- `OfflineVoSessionTest.ValidProbeBPathWritesJsonlLines`
- `OfflineVoSessionTest.MaxFramesLimitsCountsAndCoverageSpan`

首次构建 `phad_apps_tests` 的 transitive `phad_eval` archive link 曾出现一次
`Bus error`（rc 2）；相同命令 retry 后成功，随后完整八目标命令复跑返回 0。该现象
在本 checkpoint 中作为构建环境 caveat 记录，不作为源码失败或 unit failure。

### 3.2 EuRoC controls

实际命令：

```bash
cmake -S . -B build-mh01 -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPHAD_BUILD_TESTS=ON \
  -DPHAD_ENABLE_MH01_TESTS=ON
cmake --build build-mh01 --target phad_camera_mh01_test \
  phad_frontend_mh01_test phad_apps_tests -j2
PHAD_EUROC_MH01_PATH=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  ctest --test-dir build-mh01 --output-on-failure \
  -R 'Mh01RectifyTest|Mh01FrontendTest'
PHAD_EUROC_MH01_PATH=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  ./build-mh01/phad_apps_tests --gtest_filter='OfflineVoSessionTest.*'
```

实际 formal controls 为 `phad_camera_mh01_test` 与 `phad_frontend_mh01_test`：2/2
passed，累计 `225.18 s`。它们覆盖 radtan / `uint8` rectification 与 frontend control。
共同 session 的 `OfflineVoSessionTest.*` 另外 10/10 passed（`1.846 s`）。

`phad_apps_mh01_test` 未计为 formal PASS。当前工作树与
`main@46a84b5` 都有相同 static timeout / old control；这是 #53 之前已存在的陈旧
测试状态，不能据此表述 apps MH_01 control 已验证。

### 3.3 TUM VI bounded product gate

实际命令：

```bash
cmake -S . -B build-tumvi -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPHAD_BUILD_TESTS=ON \
  -DPHAD_ENABLE_TUMVI_CORRIDOR1_TESTS=ON
cmake --build build-tumvi --target phad_tumvi_corridor1_test \
  phad_tumvi_corridor1_product_test -j2
PHAD_TUMVI_CORRIDOR1_PATH=/home/lin/Projects/data/thidparty/tum/dataset-corridor1_512_16 \
  ./build-tumvi/phad_tumvi_corridor1_product_test \
  --gtest_output=xml:/tmp/phad53-clean-tumvi.xml
PHAD_TUMVI_CORRIDOR1_PATH=/home/lin/Projects/data/thidparty/tum/dataset-corridor1_512_16 \
  ctest --test-dir build-tumvi --output-on-failure -L tumvi-corridor1
```

TUM VI formal test 以 opt-in dataset root 运行，固定从 `corridor1_512_16` 首帧开始
处理 100 个 stereo frames。CTest XML：

```text
/tmp/phad53-clean-tumvi.xml
SHA256 df706172cd411cd687d41ddc36ea73f7fd87653b04c49aef7af63511a500efa4
```

loader + product 共 2/2 passed。product predicate 的实际值：

| predicate | 结果 |
|---|---:|
| session error | none |
| image frames | 100 |
| diagnostic rows | 100 |
| emitted stereo pairs | 100 |
| left/right drops | 0 / 0 |
| left/right overflow drops | 0 / 0 |
| failed | 0 |
| observation sum | 18502 |
| disparity sum | 8209 |
| observed `ok` | 100（observe-only，不是 gate） |

dataset identity 以如下固定顺序的五个文件 SHA-256 hex 无分隔连接后再次取
SHA-256：`mav0/cam0/data.csv`、`mav0/cam1/data.csv`、`mav0/imu0/data.csv`、
`dso/camchain.yaml`、`dso/imu_config.yaml`；结果即第 1 节记录的
`77f8f6a0...984a`。

该 gate 不要求 trajectory presence、initialization verdict、ATE、RPE、coverage 或
completion；它们均未在本 checkpoint 执行或判定。

## 4. 范围与完整性

这是 #53 的 camera/frontend/product-input evidence，不是 full-dataset benchmark，
因此没有 raw bench artifact root、`meta.json` 或 `summary.json`。实际显式 options
与无-hash 身份边界完整列于第 2 节。

未执行：TUM VI full sequence、ground truth/trajectory comparison、ATE/RPE、M5
initial moving root 和其后 cold-root / world-frame continuity 验收。它们不应从本次
100-frame structural product gate 推断出来。
