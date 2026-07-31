# Stereo Pair Synchronizer 设计

日期：2026-07-31  
状态：已确认并入库（§1–§4 分段确认，审阅后收紧见 §0 与 §7）；里程碑
[M3.2](../roadmap.md)，实施计划
[M3.2 双目配对同步器](../plans/2026-07-31_m3.2_stereo_pair_synchronizer_5b7d1c93.plan.md)  
相关：

- handoff：[`euroc-stereo-manifest-asymmetry-handoff.md`](euroc-stereo-manifest-asymmetry-handoff.md)
- 开源对照：[`euroc-stereo-manifest-asymmetry-open-source-refs.md`](euroc-stereo-manifest-asymmetry-open-source-refs.md)
- [`architecture.md`](../architecture.md) §3.1–§3.2（本稿修订 `pushStereo` → `pushImage` 方向）
- [`roadmap.md`](../roadmap.md) M3.2（本稿）、M3.3 VO 加固、M4 IMU 包络

## 0. 决策摘要

| 项 | 选择 |
|---|---|
| 左右配对位置 | **Synchronizer**（方案 A），非 adapter `joinStereo` |
| 同步族 | **B 族**：两侧 front 比较；偏早且超出容差则丢该侧；也可左等右、右等左 |
| 默认容差 | `tol_ns = 0`（exact）；soft 仅显式配置 |
| 配对成功时间戳 | 使用 **Left** stamp（与 VINS-Fusion 一致） |
| 本 slice 范围 | `StereoPairSynchronizer`（StereoOnly）+ offline replay 接入；不解锁 ROS / IMU 包络 |
| 动机 | 兼容 EuRoC ASL / 未来 ROS / 其它源：皆经同步器再下发算法；顺便修复官方不等长清单导致的 `open` 失败 |

审阅后补充的决策（§7 有展开）：

| 项 | 选择 |
|---|---|
| sync 落点 | 新库 `phad/sync/`（target `phad_sync`，alias `phad::sync`），只依赖 `phad::sensor` / `phad::common`；配 README |
| 新类型归属 | `CameraId`、`ImageFrameEvent` 进 `phad::sensor`；`io::SensorEvent` 的 `StereoFrame` 换成 `ImageFrameEvent` |
| dataset 公共 API | 换成 per-camera：`peekImageTimestamp(CameraId)` / `takeImage(CameraId)`；`summary` 变 `imu / left / right`；`takeStereo()` 与 `joinStereo` 退役 |
| 调用点兼容 | `apps/` 提供薄 `StereoPairStream`（source + sync 组合），供 session / probe / runner / 真序列测试复用；`io` 不依赖 `sync` |
| sync payload | 已解码 `Image`；接受 orphan 帧「先解码再丢」的浪费，换在线离线同一条路径 |
| `pushImage` 返回值 | 只表达校验结果；丢弃与溢出进计数器，配对产出经 `tryPop()` 观察 |
| 输出获取 | 只有 `tryPop()`，不提供回调 |
| 错误码清理 | 删除 `DatasetErrorCode::kStereoTimestampMismatch`（退役后无触发者） |

实施前对齐补充（2026-07-31，写入计划）：

| 项 | 选择 |
|---|---|
| `summary.json` sync 段 | `schema_version` 保持 1；新增可选 `"sync"`（字段对齐 `StereoPairDiagnostics`）；`bench_table.py` 不读 |
| 首次 drop / overflow warning | `sync` 只计数；stream/session 写入 `summary.warnings` / `meta.warnings`；库不接日志 |
| emit=0 | `StereoPairStream::next()` → `EndOfStream`；零配对由 session / `summary.sync` 体现 |
| MH_01 逐字节参考 | 既有 M3.1 产物 `…/MH_01_easy/2b28616/default_0885385a/{est.tum,diag.csv}` |

根因回顾：`MH_04` / `V1_02` / `V2_03` 官方 ASL 与 bag 左右清单本身不对称；旧 `joinStereo`（等长 + 下标 exact）过严，且 index-zip 在 `V2_03` 上会静默错配。详见 handoff 与开源对照。

---

## 1. 模块边界

### 1.1 职责切分

| 模块 | 负责 | 不负责 |
|---|---|---|
| **`phad::io::dataset`（EuRoC/TUM VI）** | 解析标定与 CSV；单路清单自洽（递增、无重复、PNG 存在）；产出/回放 **单路** 图像事件 + IMU | 左右配对；因不等长拒开；容差同步 |
| **未来 `phad::io::ros` 等** | topic/驱动 → 同一规范化事件 | 自写 `sync_process` |
| **`phad::sync::StereoPairSynchronizer`（本 slice；独立库，只依赖 `sensor`/`common`）** | B 族左右配对；有界队列 API；诊断计数 | EuRoC 路径、ROS 名、读 PNG、IMU 切片 |
| **frontend / estimator** | 已配对 `StereoFrame`（现 VO） | 数据源、队列、丢帧策略 |
| **app glue / `OfflineVoSession` / bench** | Source → Sync → 现有 VO 管线 | 同步算法本身 |

### 1.2 公开合同（相对 architecture 的修订方向）

```text
pushImu(ImuMeasurement)         // M4；本 slice 不暴露
pushImage(CameraId, ImageFrame) // Left / Right
→ tryPop() → StereoFrame        // 本 slice；不提供回调
→ （M4）StereoImuPacket
```

- `phad::sensor::StereoFrame` 语义不变：**已配对**双目。
- 配对只在 sync 内部完成。
- 纯 VO：StereoOnly 模式，避免提前吞下整个 M4。

### 1.3 与当前代码的边界变化

1. **`joinStereo` 删除**：不保留「仅作诊断」的降级版本，避免两套配对策略并存；
   `DatasetErrorCode::kStereoTimestampMismatch` 一并删除（退役后无触发者）。
2. **Dataset 索引**：改为左右（+IMU）**分路清单**；replay 按时间归并事件进 sync。
3. **Dataset 公共 API 随之改变**（本条是本 slice 最大的一处 churn）：
   - `StereoImuDatasetReader::peekStereoTimestamp()` / `takeStereo()` →
     `peekImageTimestamp(CameraId)` / `takeImage(CameraId)`；
   - `StereoImuDatasetSummary` 的 `stereo` → `left` / `right`（`imu` 不变）；
     `phad_euroc_inspect` 改印两路计数，并可另印交集帧数作诊断；
   - 受影响调用点：`apps/offline_vo_session.cpp`、`apps/phad_stereo_frontend_probe.cpp`、
     `apps/phad_euroc_inspect.cpp`、`apps/phad_euroc_runner.cpp`、
     `phad/io/dataset/dataset_replay_source.cpp`，以及 `tests/io/dataset/euroc`、
     `tests/io/dataset/tum_vi`、`tests/camera/euroc_mh01_rectify_test.cpp`、
     `tests/frontend/euroc_mh01_frontend_test.cpp`。
   - 保留 `takeStereo()` 不是选项：那等于把配对留在 dataset 内，与 §1.1 的
     边界自相矛盾。
4. **`apps/StereoPairStream`**：薄组合件（`SensorSource` + `StereoPairSynchronizer`），
   对外只提供「给我下一对 `StereoFrame`」，让上述事件循环的改动收敛到一两行。
   它属于 composition root 一侧，`io` 与 `sync` 都不感知它。
5. **依赖方向**：`sync` 不依赖 `io`；`io` 不依赖 `sync`；app 为 composition root。

### 1.4 本 slice 非目标

- ROS adapter、真机推流  
- IMU 边界插值 / `StereoImuPacket`  
- 改写官方 CSV / Kimera 策展包作主路径  
- 默认开启 soft-tol  
- 改 frontend/estimator 算法逻辑  

---

## 2. 数据流

### 2.1 规范化事件

```text
ImageFrameEvent { CameraId id; Timestamp t; Image image; }  // id ∈ {Left, Right}
ImuEvent        { ImuMeasurement }                          // M4
```

Adapter 只做来源翻译；不保证左右成对。

### 2.2 Offline 路径（本 slice）

```text
open(dataset)
  → 分路清单 cam0[] / cam1[] / imu[]   // 允许不等长
  → Replay：按时间归并 pushImage(Left|Right, ...)
  → StereoPairSynchronizer（B 族）
  → tryPop() → StereoFrame
  → rectify → track → glue → estimate
```

**Replay 归并：** 每次取全局最早候选；同 stamp 顺序 **IMU → Left → Right**（无 IMU 时 Left→Right）。

**解码时机（已知取舍）：** 事件里带的是已解码 `Image`，因此 orphan 帧会先解码再被
sync 丢掉（`V2_03_difficult` 约 415 张，占该序列 cam1 的 18%）。M1 的
「`peek` 不触发 I/O」在 per-camera API 上仍成立，但 offline 全序列回放的总解码量
不再等于配对帧数。选择接受：换来在线与离线共用同一条 push/pop 路径；按时间归并
使队列深度仍只有个位数，内存有界性不受影响。若将来解码成为瓶颈，再考虑让 sync
对 payload 泛化（push 句柄、pop 后解码），本 slice 不做。

### 2.3 Synchronizer 内部（StereoOnly）

```text
pushImage(id, frame):
  enqueue left_q | right_q
  drain():
    while both non-empty:
      if |tL - tR| <= tol_ns:   # 默认 0
        emit StereoFrame{ t = tL, left, right }
      else if tL < tR: drop left
      else: drop right
```

离线全量 push 时与双指针 merge 等价；在线与 VINS 双队列 B 族同构。

| 配置 | 本 slice |
|---|---|
| `tol_ns` | 默认 0 |
| 队列 | API 支持有界；offline 默认很大/无界，避免与溢出策略搅在一起 |
| 输出 stamp | Left（`tol_ns = 0` 时两侧相同，此项只在 soft 配置下有意义） |
| 结束 | `flush()`：**两侧队列全部剩余**条目计入 `dropped_*`，不只是 front，不伪造 pair |

### 2.4 多源预留

ROS / 其它源仅替换「谁调用 `push*`」；算法只见 sync 输出。

### 2.5 接到 VO / bench

`OfflineVoSession` / probe：事件循环 `push` → `tryPop` → 现有管线。  
`StereoFrame` 合同对 frontend 不变。

### 2.6 明确禁止的数据流

- `joinStereo` 等长失败导致 `open` 失败  
- `min(len)` 下标 zip  
- sync 内读文件  

---

## 3. 错误与诊断

### 3.1 硬失败 vs 软失败

| 层 | 硬失败 | 软失败（计数，继续） |
|---|---|---|
| Adapter / open | 标定坏、CSV 坏、单路逆序/重复、缺 PNG、空清单 | 左右不等长、orphan stamp |
| Synchronizer | 逆序 push、非法 stamp/`CameraId`、非法配置 | B 族丢弃；队列 overflow drop-oldest |
| Session | 读图 I/O 失败；VO 原有错误 | emit 变少（反映 completion） |

### 3.2 诊断计数（最小集）

```text
pushed_left / pushed_right
emitted_stereo
dropped_left / dropped_right
dropped_left_overflow / dropped_right_overflow
max_left_queue / max_right_queue
```

结束 summary 一条；首次 drop / 首次 overflow 各 warning 一次（避免刷屏）。

### 3.3 结果码

一次 `pushImage` 可能同时「输入合法 + 丢掉一帧 + 凑出一对」，单个枚举表达不了，
因此把三件事分开：

| 观察什么 | 从哪看 |
|---|---|
| 输入是否被接受 | `pushImage` 返回值：`kOk` / `kOutOfOrder` / `kDuplicate` / `kInvalidStamp`（`kOutOfOrder`、`kDuplicate` 按 **单相机** 单调性判定，且 sticky，与 dataset reader 的 terminal error 语义一致） |
| 是否丢了帧 / 溢出 | 诊断计数器（§3.2），不占返回值 |
| 是否产出配对 | `tryPop()` 是否给出 `StereoFrame` |

### 3.4 旧错误码

`kStereoTimestampMismatch` 不再因不等长在 `joinStereo` 触发。  
耗尽后零配对 → 「无有效 stereo」，非 manifest mismatch。

---

## 4. 测试与验收

### 4.1 单元测试（不读磁盘）

- 等长 exact：emit=N，drop=0  
- MH_04 式左首 orphan；V1_02 式右尾 orphan；V2_03 式大量右多余（断言 emit 集合 = ∩，无 index 错配）  
- 交错 push 顺序仍能配  
- `tol_ns=0` 拒绝 1ns 差；soft 仅显式测  
- 单相机逆序 / 重复 → sticky error；有界队列溢出计数  
- `flush()` 把两侧全部剩余计入 `dropped_*`（尾部 orphan 也要被算上）  

### 4.2 Adapter 回归

- MH_01 经 sync：`est.tum` 与 `diag.csv` 相对 M3.1 基线**逐字节相同**（等长序列的
  配对结果与旧 `joinStereo` 完全一致，因此这条比「ATE ≈」更强也更便宜）  
- 合成不等长：`open` 成功  
- 单路非法（逆序、重复、缺 PNG、坏 CSV/YAML）仍硬失败  
- 原先断言 `kStereoTimestampMismatch` 的用例改写为「open 成功 + 配对软丢弃」  
- TUM VI 等长路径绿  

### 4.3 真序列 bench

配对与丢弃数由交集唯一确定，因此精确断言，不用「≈」或「≥」：

| 序列 | cam0 / cam1 | emit | drop_left | drop_right |
|---|---|---:|---:|---:|
| MH_04_difficult | 2033 / 2032 | 2032 | 1 | 0 |
| V1_02_medium | 1710 / 1711 | 1710 | 0 | 1 |
| V2_03_difficult | 1922 / 2336 | 1921 | 1 | 415 |
| MH_01_easy | 3682 / 3682 | 3682 | 0 | 0 |

三条原失败序列还需 `open` 成功并产出 `summary.json`；MH_01 见 §4.2 的逐字节要求。
全量 11 序列：`open` 全过；`bench_table.py` 刷新表。VO 质量差另题。

### 4.4 完成清单

- [ ] `StereoPairSynchronizer` 单测绿  
- [ ] MH_01 `est.tum` / `diag.csv` 逐字节不回归  
- [ ] 三条原失败序列产出 `summary.json`，emit / drop 精确匹配 §4.3  
- [ ] 诊断可见 dropped_* / emitted  
- [ ] architecture §3.2 / `phad/io` README / `phad/sync` README 合同更新  
- [ ] 未改 frontend/estimator 算法  

---

## 5. 与开源及后续里程碑

- **VINS-Fusion**：在线 B 族双队列 + 3 ms；我们默认 exact，算法同构、容差更严。  
- **ORB/Basalt**：离线左时间表 + exact 右存在；`tol=0` 时结果与 ∩ / 左驱动 exact 一致。  
- **Kimera**：整理包是数据逃生口；库默认应用 sync 消化官方不对称。  
- **M4**：同一模块扩展 `pushImu` + 图像边界 IMU → `StereoImuPacket`，替换架构里过时的「仅 `pushStereo`」表述。M4 因此不再需要新建 synchronizer，只补 IMU 侧。

### 5.1 已知限制

`tol_ns > 0` 时 B 族是**贪心**配对而非最优匹配：容差窗内有多个候选时按到达顺序
成对，可能不是时间上最近的一对。VINS-Fusion 同样如此。默认 `tol_ns = 0` 下不存在
该歧义，因此本 slice 只给 soft 路径一个显式配置的 smoke test，不追求最优匹配。

## 6. 实施顺序

1. `phad::sync` 的 `StereoPairSynchronizer` + 纯内存单测（不碰 `io`）  
2. `sensor` 新类型；dataset 分路索引与 per-camera API；replay 归并；删 `joinStereo`；测试迁移  
3. `apps/StereoPairStream` 接入 session / probe / runner；MH_01 逐字节回归  
4. 三条原失败序列 + 11 序列 bench；文档一致性  

任务级拆解见
[M3.2 双目配对同步器](../plans/2026-07-31_m3.2_stereo_pair_synchronizer_5b7d1c93.plan.md)。

## 7. 审阅记录（2026-07-31）

设计通过审阅，方向不变；下列缺口已就地补进本稿，供实施时对照：

| # | 问题 | 处置 |
|---|---|---|
| 1 | 未交代 `takeStereo()` / `summary.stereo` 这两处公共合同的破坏与调用点范围 | §1.3：改 per-camera API，churn 由 `apps/StereoPairStream` 收敛 |
| 2 | sync 落点、`CameraId` / `ImageFrameEvent` 归属、`SensorEvent` 变更未定 | §0 决策表：新库 `phad::sync`；新类型进 `phad::sensor`；`phad_euroc_runner` 也走 sync |
| 3 | 惰性解码在 offline 全序列回放上事实失效，未记录 | §2.2「解码时机」：接受并量化（V2_03 约 415 张） |
| 4 | `pushImage` 单枚举混装校验错误、丢弃与溢出 | §3.3 拆成返回值 / 计数器 / `tryPop()` 三处观察 |
| 5 | 「`tryPop()` / 回调」二义 | §1.2：只保留 `tryPop()` |
| 6 | 验收用「≈」「≥」，且 `flush()` 只算 front 会漏掉尾部 orphan | §4.3 精确表；§2.3 改为全部剩余计入 |
| 7 | `kStereoTimestampMismatch` 退役后成为死码，现有用例仍断言它 | §1.3 删枚举；§4.2 改写用例 |
| 8 | soft tol 的贪心配对未标注为限制 | §5.1 |
| 9 | `summary.json` sync 字段、warning 落点、emit=0、MH_01 参考路径未钉死 | §0 实施前对齐表；计划正文已收紧 |
