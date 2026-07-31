# EuRoC 双目 manifest 不对称：开源对照

日期：2026-07-31  
状态：调研完成；合同改动待 A/B/C 决策（见 handoff）  
相关：[`euroc-stereo-manifest-asymmetry-handoff.md`](./euroc-stereo-manifest-asymmetry-handoff.md)、[`euroc-dataset-loader-design.md`](./euroc-dataset-loader-design.md)

## 1. 问题摘要

EuRoC ASL 的 `cam0/data.csv` 与 `cam1/data.csv` **并不总是等长或按下标对齐**，尽管套件对外宣称硬件同步双目。本地原生 ASL（与 rosbag topic 计数交叉核实）上：

| sequence | cam0 | cam1 | ∩ | 不对称形态 |
|---|---:|---:|---:|---|
| MH_04_difficult | 2033 | 2032 | 2032 | 1 条 cam0-only 在 **index 0** |
| V1_02_medium | 1710 | 1711 | 1710 | 1 条 cam1-only 在 cam0 时间轴之后 |
| V2_03_difficult | 1922 | 2336 | 1921 | 起始 1 条 cam0-only；**415** 条 cam1-only（414 落在 cam0 跨度内） |

交集时间戳对应 PNG 齐全。不对称往往**不是**「尾部多一行」：对 `V2_03` 截断到 `min(len)` 再按下标 zip，前缀几乎全部 timestamp 错位（首差约 50 ms）——index-zip 会静默配对错误。要求「等长 + 同下标 exact」的 loader（当前 phad `joinStereo`、开启 sanity 的 Kimera）会拒开或 FATAL；以 cam0 时间表 / 预计算交集列表 / ROS soft sync 驱动的系统仍能跑。

## 2. 项目对照表

| Project | 配对策略 | 容差 | 来源 |
|---|---|---|---|
| **EuRoC paper / ASL** | 宣称硬件同步双目；每路相机自有 `data.csv`。**未规定**不等长 CSV 的 join 算法。独立 AEC → 亮度差，与 shutter 无关。 | N/A | [IJRR 2016](https://doi.org/10.1177/0278364915620033)；[datasets hub](https://ethz-asl.github.io/datasets/)；[MAV page](https://projects.asl.ethz.ch/datasets/euroc-mav/) |
| **ORB-SLAM3** | 外部时间戳列表 `EuRoC_TimeStamps/*.txt`；同一 stamp 拼 `cam0`/`cam1` 路径。不读双 CSV、不 index-zip。本地：`MH04`/`V102`/`V203` 列表长度 **2032/1710/1921**，集合等于 `cam0∩cam1`。 | exact（路径/文件名一致）；缺右图 → `imread` 空 → 硬失败 | [stereo_euroc.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/Stereo/stereo_euroc.cc)；`Examples/Stereo/EuRoC_TimeStamps/` |
| **ORB-SLAM2** | 同 ORB-SLAM3 stereo EuRoC 模式 | exact | `ORB_SLAM2/Examples/Stereo/stereo_euroc.cc` |
| **Basalt** | **左目驱动**（只读 cam0 `data.csv` 作 `image_timestamps`）；查询时用同名文件读 cam1；缺文件 → 空 `ImageData`；光流遇 null 跳过。 | exact path；无 soft dt | `basalt/include/basalt/io/dataset_io_euroc.h`；optical flow null skip |
| **Kimera-VIO** | README 承认官方 **MH_04 / V2_03** 左右帧数不同，建议用其整理版。可选 `sanityCheckCamSize` **截到 min 再 index-zip**；`sanityCheckCamTimestamps` 同下标不等则 FATAL。当前 `parseDataset()` 里 sanity **被注释掉**。FAQ 复现 Left:2033 / Right:2032。 | 开启检查时 exact@index；否则靠 curated data | [README](https://github.com/MIT-SPARK/Kimera-VIO/blob/master/README.md)；[faq](https://github.com/MIT-SPARK/Kimera-VIO/blob/master/docs/faq.md)；`EurocDataProvider.cpp` |
| **VINS-Fusion** | EuRoC 示例走 **rosbag**，无 ASL CSV stereo loader。`sync_process`：双队列，`|t0-t1| > 0.003` 丢较早侧，否则配对并用 **img0 stamp**。 | **3 ms** | [rosNodeTest.cpp](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/rosNodeTest.cpp)；[issue #137](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/issues/137) |
| **OpenVINS** | ROS：`ApproximateTime`。Bag 串行：对其它 cam 找 **≥ current 且 ≤ 0.02 s** 最近；不全则 skip。ASL helper 只服务 GT CSV。 | **~20 ms**（bag）；ApproximateTime（ROS） | `ov_msckf` serial / visualizer；`ov_core/.../dataset_reader.h` |
| **OKVIS2-X** | 各 `cam{i}/data.csv` 独立；多迭代器推进最早侧，容差内成组，否则 warning `"without correspondence"`。 | **10 ms** | `DatasetReader.cpp` |
| **rpg_svo_pro** | `combine_images.py`：**左目驱动 exact** 匹配右列表；`associate_timestamps.py` 最近邻。 | stereo combine：exact；associate 默认 **0.02 s** | `svo_benchmarking/.../combine_images.py`；`vikit_py/.../associate_timestamps.py` |
| **evo** | 仅轨迹 est↔GT 关联，非 stereo IO | 默认 `--t_max_diff` **0.01 s** | [evo wiki](https://github.com/MichaelGrupp/evo/wiki/evo_traj) |
| **phad-vio（当前）** | 等长 **且** `left[i].ts == right[i].ts`；否则 `kStereoTimestampMismatch`。设计文案要 exact join / 集合匹配，实现是等长下标检查（未做集合交集）。M1 明确拒绝 VINS 3 ms NN。 | exact；不等长即拒 | handoff；`euroc-dataset-loader-design.md` §3.3/§7；`joinStereo` |

## 3. 共识（offline ASL IO）

对希望「真正同一 timestamp 的双目对」的离线 EuRoC/ASL loader：

1. **不要**按下标 zip；等长也不足以保证正确配对（见 V2_03）。
2. 优先 **exact timestamp 集合交集**（或左目驱动、仅当右目存在**同一 ns stamp** 才出对）。
3. 诊断：`dropped_left` / `dropped_right`（可选 orphan 列表）；交集非空则允许 open。
4. **soft sync（3–20 ms）** 留给 bag/live；原生 EuRoC 上可配对帧本就是 exact ns（本地对 V2_03 做 NN@3 ms：1921 exact、0 soft、1 miss）。
5. Kimera「下我们整理包」是**数据策展**逃生口，不宜当作库的默认合同。

这与 ORB-SLAM3 实际下发的 times 文件规模、Basalt/SVO combine 的「左时间轴 + exact 右存在」、以及设计文案中的 exact join 一致，且不必假装每一行 CSV 都有配偶。

## 4. 陷阱

| 陷阱 | 危害 | 证据 |
|---|---|---|
| 等长 index-zip / truncate-to-min | V2_03 上前缀几乎全错配 | 本地审计；Kimera sanity |
| 只拼 cam0 路径且不查存在性 | MH_04 左 orphan → 缺右图 | ORB 硬失败；Basalt null |
| 假定不对称只在尾部 | MH_04 orphan 在首行；V2_03 大量 cam1-only 在跨度内 | only_cam0 / only_cam1 |
| 对 ASL 做 soft NN | 几乎不增加可用对，却引入假配对风险 | VINS/OpenVINS/OKVIS 容差面向流式 |
| 整序列拒开 | MH_04/V1_02 只丢 1 帧 | ∩ vs CSV 长度 |
| IMU 时间基 | stereo stamp 应保持真实 capture 时间；soft 配对后仍用左时间却配上错右图 → 三角化坏、IMU 仍信左时间 | VINS 用 img0 time + ≤3 ms gate |
| 「ASL 坏、bag 好」 | bag 计数与 ASL 不对称一致 | handoff §3.2 |

## 5. 修复选项排序（exact-timestamp stereo IO）

1. **Exact timestamp-set intersection（推荐）**  
   双 CSV 建 map/set；保留双边共有 stamp；排序；可选校验 PNG；计数 drops。对齐 ORB 列表、SVO combine、设计文案；不引入 VINS soft sync。

2. **Left-driven exact lookup（若固定左目为 VO host，与 1 等价）**  
   遍历 cam0，仅当 cam1 有同 stamp 才入 manifest。语义贴近 Basalt / ORB times file。

3. **Curated / 外部 manifest（Kimera Drive；ORB times）**  
   保持严合同；文档标明三条官方 ASL 不支持。逃生口，不宜默认。

4. **维持现状（等长 + index exact）**  
   fail-closed；挡住社区已知不对称的 3/11 序列。

5. **Soft NN（≤3–10 ms）— 最后手段 / 独立模式**  
   对齐 VINS/OKVIS/OpenVINS bag；**原生 ASL 不需要**；仅考虑非 ASL 或 live adapter，需显式 flag + design 审阅。

## 6. 对 phad-vio 的结论

社区对 **ASL offline stereo** 的惯例是 **timestamp 同一性**（交集或左驱动 exact 存在），不是等长 index-zip。soft 容差出现在 **ROS/bag** 前端。设计已要求 exact join；实现 **集合交集 + drop 计数** 即可弥合设计与 `joinStereo` 的落差，且无需采纳 VINS 3 ms 配对。
