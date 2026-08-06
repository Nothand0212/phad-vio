# 调研：纯 stereo VO 的 landmark 三角化与深度估计工程实践

日期：2026-08-06
范围：单帧立体 backproject vs 多帧三角化、逆深度参数化、三角化视差阈值、观测数量门限、立体深度噪声模型。核心问题：我们的「单帧 backproject + 窗口 BA」在 EuRoC V 系列上是否显著差于「多帧三角化」？

## 结论摘要（核心问题先行）

**「单帧 backproject + 窗口 BA」在 V 系列上确实显著差于 ORB-SLAM2/SVO/VINS 的深度链路——但差距不在「单帧 vs 多帧」这个二选一，而在「单帧立体的使用方式」：**

- ORB-SLAM2 stereo 的**初始化本身就是单帧 backproject**（`z = mbf/disparity`），V 系列 ATE 0.02–0.05 m。VINS-Fusion 的 stereo 分支也是单帧（start_frame 处左右双观测 DLT）。**单帧 backproject 不是原罪。**
- 真正的差异是：(a) 我们**所有** landmark 永远来自单帧 backproject，ORB-SLAM2 新点主要来自多帧三角化（带 1.15° 视差角门限 + cheirality + 重投影门限），运动视差不如立体视差时才回退立体深度——「取条件更好的那个」；(b) SVO 用深度滤波多帧融合、收敛后才进图，VINS 用逆深度参数化让滑窗 BA 持续修正；(c) 我们的远点门限太松（min_disparity_px=0.5 → 允许 1.3px 视差的 40m 点直接当 anchor）。
- V 系列快速运动恰好多帧视差大（帧间平移基线 > 立体基线 0.11 m 轻松满足），**多帧三角化/多帧深度融合的条件远好于单帧立体**，这正是深度滤波和 ORB-SLAM2 在 V 系列占优的机制原因。

## 五个假设逐一验证

### 假设 1：单帧立体 backproject vs 多帧三角化——部分支持（修正为「两者结合 + 择优」）

证据（ORB-SLAM2 源码：LocalMapping.cc、Frame.cc、Tracking.cc）：

1. **ORB-SLAM2 也是单帧 backproject 初始化**：`StereoInitialization` 仅检查 `z > 0` 就 `UnprojectStereo`。所以「单帧 backproject」本身在 V 系列可达 0.02–0.05 m。
2. **但它新增点走多帧三角化**：`LocalMapping::CreateNewMapPoints` 对共视关键帧对做 DLT/SVD 三角化，门限为：基线 < 立体基线则跳过（说明立体已够）、单目情形 `cosParallax < 0.9998`（≈1.15° 最小视差角）、cheirality 双深度为正、重投影误差门限（5.991σ² mono / 7.8σ² stereo）、尺度一致性。当运动视差角 < 立体视差角时直接用 `UnprojectStereo`——**按条件择优**。
3. **VINS-Fusion**（feature_manager.cpp）：stereo 特征在 start_frame 用左+右两个相机的 4 方程 DLT/SVD 最小二乘三角化（比解析 backproject 对噪声略稳），观测 ≥4 时多视图 SVD。
4. **SVO2 论文（TRO 2017）**明确说明立体系统的优势："inter camera triangulations allow to quickly initialize new 3D landmarks in case of **on-spot rotations**"——原地旋转（无平移、帧间视差为零）时，单帧立体是唯一能快速初始化 landmark 的手段；反过来，平移快速时（V 系列的特征）多帧三角化条件优于立体。
5. **SVO1 论文（ICRA 2014）**对深度滤波 vs 两帧三角化的对比："the main advantage ... over the standard approach of triangulating points from two views is that we observe **far fewer outliers as every filter undergoes many measurements until convergence**"，且 "**very little motion is required** to significantly reduce the uncertainty in depth"——快速运动下多帧融合收敛极快。

量化依据（EuRoC：f≈458.7 px，b≈0.11–0.12 m → f·b≈50–55 px·m）：
- 单帧立体深度误差 δz/z = δd/d = δd·z/(f·b)：z=5 m（d≈10 px）亚像素 δd=0.3 px → 3%；z=10 m → 6%；z=25 m → 15%；z=40 m（d≈1.3 px）→ 24%，运动模糊下 δd 到 0.5–1 px 时 10 m 处就有 10–20%。
- 帧间三角化视差角 vs 立体视差角：当帧间平移 > 立体基线 0.11 m 时多帧条件即更优；V 系列快速运动帧间平移常达 0.3–1 m。**对远点，V 系列上多帧三角化显著更稳；对近点（z<3 m，d>16 px）单帧立体已极好。**

实现成本：多帧三角化本身用 gtsam 现成工具即可（两帧位姿 + DLT），中等（2–4 天）；见「最小改动」第 2 条。

### 假设 2：逆深度参数化——支持，但优先级中等

证据：
- Civera et al.（TRO 2008）：低视差（远点）时 XYZ 深度的后验「在最小深度处陡升、向无穷长尾」——非高斯，EKF/非线性最小二乘线性化差；逆深度 ρ=1/d 下测量方程近似线性，单高斯即可覆盖「近到无穷」，远点先作为 bearing 约束参与运动估计，之后「从无穷平滑收敛」。
- SVO1/2：深度滤波的 Gaussian 分量直接建在逆深度上（"we use inverse depth coordinates to deal with large scene depths"，SVO2 Sec. VI）。
- VINS-Fusion：滑窗状态对特征存 inverse depth（`setDepth: estimated_depth = 1.0 / x`）。
- DSO/VI-DSO：所有点逆深度参数化、平均深度归一化，初始深度随机，尺度独立优化。
- **对 stereo 的额外论证**：视差测量 d = f·b·(1/z) 本就线性于逆深度——逆深度是立体测量**天然**的线性参数化，比 XYZ 更符合残差高斯假设。

对我们（GTSAM LM + `gtsam::Point3` XYZ，stereo_vo_estimator.cpp L810-814）的意义：XYZ 与逆深度在 batch LM 下解空间相同，差异在**收敛性**：10–40 m 远点初始深度误差 15–24% 时，XYZ 线性化差、易被拉爆或收敛到偏解（有 huber 与重投影 cull 兜底，但会悄悄损失远点约束）。GTSAM unstable 有现成 InvDepthFactor3。成本：高（~1–2 周，需改 factor 与 anchor 管理），**建议放在门限/鲁棒化之后**。

### 假设 3：三角化视差阈值——需修正认知：ORB-SLAM2 没有显式 min-disparity 阈值

证据（Frame.cc）：`ComputeStereoMatches` 的 disparity 区间是 `[minD=0, maxD=mbf/minZ)`——**没有最小视差门限**。它的防御是：抛物线亚像素细化（`deltaR = (dist1-dist3)/(2*(dist1+dist3-2*dist2))`，与我们 refineSubpixel 公式相同）+ 中值 outlier 剔除（`thDist = 1.5*1.4*median`）+ 多帧三角化的 1.15° 视差角门限 + cheirality + LocalBA 反复优化 + culling。垃圾远点靠「优化+剔除」消化，而非源头拒绝。

我们前端实际**比 ORB-SLAM2 更严**（stereo_tracker.hpp L50-63）：min_disparity_px=0.5、uniqueness 0.5、双向一致 0.5 px、对极行差 1.5 px、SAD 亚像素。**问题在阈值取值**：min_disparity_px=0.5 + max_depth_m=40 → 放行 d≈1.3 px 的 40 m 点直接 seed 为世界系 XYZ anchor；这类点单帧深度误差 20%+，快速旋转时被 BA 当硬约束拉偏轨迹。SVO 用深度滤波等收敛才转点，VINS 用逆深度持续修正——我们两者都没有。

实现成本：把 min_disparity_px 提到 1.5–2 px（深度 ≤25–33 m）、max_depth_m 收到 20–25 m 即可，**纯参数 A/B（<1 天）**。

### 假设 4：landmark 观测数量门限——支持，我们是 2，ORB-SLAM2 stereo 是 >3

证据（LocalMapping.cc `MapPointCulling`）：mono 模式 nThObs=2，**stereo 模式 nThObs=3**——点龄 ≥2 关键帧时观测 ≤3 即剔除；另有 `GetFoundRatio() < 0.25`（被跟踪到的帧占比）剔除。我们 types.hpp L35 `min_landmark_observations=2`（仅作图门限，stereo_vo_estimator.cpp L797-801），剔除机制是均值重投影 4 px + cheirality（近似 found ratio 的作用）。

V 系列影响：快速旋转/运动模糊下立体误匹配率高，2 观测点很多是「单次深度硬 anchor」，一次坏深度就污染 2 帧的位姿。建议 2→3（参数改动）+ 可选统计 found ratio 剔除。

### 假设 5：立体深度噪声模型——低优先级；我们 BA 已在像素域，正确

证据：SVO 深度滤波的测量方差 τ² 由「图像平面 1 像素视差方差」几何推导（`px_error_angle = atan(px_noise/(2*focal_length))*2` + 正弦定理 `computeTau`，depth_filter.cpp）——**1 px 视差噪声是行业标准假设**。深度误差 ∝ z²·δd/(f·b)，因此噪声模型只在「初始化质量/远点权重」层面有意义：BA 残差在像素域（我们 `stereo_sigma_px=1.0` + GenericStereoFactor 的 (uL,vL,uR) 三维残差，已隐式建模视差噪声）就够；若要精细化，方向是逆深度参数化后 σ 自然对应（与假设 2 合并）。

## 参考实现对照表（针对我们 LK + 固定窗口 batch BA 的现状）

| 环节 | 我们（stereo_vo_estimator.cpp） | ORB-SLAM2 stereo | SVO 1/2 | VINS-Fusion stereo |
|---|---|---|---|---|
| 立体匹配 | LK + SAD 亚像素 + 双向 + uniqueness + 行差（完善） | 描述子 + 亚像素 + 中值剔除 | patch 相关 + 极线搜索 | LK + 行搜索 |
| 初始深度 | 单帧 backproject（唯一来源） | 单帧 backproject（初始化）+ **多帧三角化（新点，1.15° 门限+cheirality）** | **深度滤波多帧 Bayesian 融合，收敛后才转点** | start_frame 双观测 DLT + 滑窗持续修正 |
| 参数化 | 世界系 XYZ（Point3） | XYZ + LocalBA 反复优化 | 逆深度 + Gaussian | 逆深度（host 帧） |
| 观测门限 | 2（窗口内） | stereo >3 + found ratio ≥0.25 | 收敛（σ 门限）才进图 | 滑窗持续观测 |
| 远点防御 | max_depth 40 m / min_disp 0.5 px（太松） | 亚像素 + 优化 + culling | 逆深度 + 多帧 | 逆深度 + 深度>0 检查 |

## 最小改动建议（按成本/收益排序）

1. **纯参数 A/B（<1 天，零代码）**：
   - `min_disparity_px` 0.5 → 1.5~2.0（拒绝 d<2 px 即 z>25–33 m 的点当 anchor）
   - `max_depth_m` 40 → 20~25
   - `min_landmark_observations` 2 → 3
2. **seed 深度鲁棒化——「多帧三角化」的最小实现（2–4 天，不改参数化）**：前端已有 `track.length`（光流跟踪帧数）。把「关键帧 seed 即 backproject」改为：新 landmark 稳定跟踪 ≥3 帧后 seed，初始深度用**多帧视差的中位数**（鲁棒，抗单帧误匹配），或直接用当前帧 PnP 位姿 + 上一帧观测做两帧 DLT 三角化（条件好时即假设 1 的收益）。改动集中在 seedSegment（L452-498）与新点 seed 分支（L1271-1301）。
3. **（后续）逆深度参数化（1–2 周）**：Point3 → 锚定首观测帧的逆深度；gtsam unstable 有现成 InvDepthFactor3；收益集中在 10–40 m 远点的收敛与「从无穷收敛」行为。
4. **（不推荐首期）** 完整 SVO 式 Gaussian+Beta 深度滤波——模块大，收益与第 2 条重叠，只有「显式 outlier 建模」（自相似纹理环境）是增量。

**注意事项**：V 系列 0.5–8.8 m 的 ATE 量级远超深度噪声可解释的范围——深度链路是显著贡献者之一，但与 roadmap 已诊断的因素（关键帧选择对旋转不敏感、非 KF 位姿质量、CV 链污染、无 IMU/无回环）耦合；A/B 时应与 ⑤c（Basalt 7KF+3temporal 驱逐、非关键帧进窗口）基线对比，并保留 MH_01 不回归锚（ATE 不劣于 0.0988 m）。

## 本地代码事实（调研基准）

- 前端 `phad/frontend/stereo_tracker.cpp`：LK 光流 + goodFeaturesToTrack；立体匹配 = SAD 全局极小（单峰）+ 抛物线亚像素（refineSubpixel L178-205，与 ORB-SLAM2 deltaR 公式一致）+ uniqueness（0.5）+ 双向一致（0.5 px）+ 对极行差（1.5 px）+ min_disparity_px=0.5 + depth range 0.3–40 m。
- 后端 `phad/estimator/stereo_vo_estimator.cpp`：GTSAM LM，窗口 10 帧（≤7 关键帧 Basalt 式驱逐，L1307-1350），GenericStereoFactor（uL,vL,uR 三维残差，stereo_sigma_px=1.0），landmark = gtsam::Point3 世界系 XYZ（L810-814），min_landmark_observations=2（L797-801），最老帧 PriorFactor 锚定（σ_rot=1e-4 rad, σ_trans=1e-4 m），seed = 单帧 backproject（seedSegment L452-498；keyframe-only seed L1268-1301），PnP 初始化 + huber + mean-reproj cull（4 px）+ cheirality + rebirth 封锁。
- 配置 `phad/estimator/types.hpp` L32-59、`phad/frontend/stereo_tracker.hpp` L50-63。

## Sources

### 源码（primary）
- ORB-SLAM2 LocalMapping.cc（CreateNewMapPoints 门限、MapPointCulling）：https://raw.githubusercontent.com/raulmur/ORB_SLAM2/master/src/LocalMapping.cc
- ORB-SLAM2 Frame.cc（ComputeStereoMatches 亚像素/中值剔除、UnprojectStereo）：https://raw.githubusercontent.com/raulmur/ORB_SLAM2/master/src/Frame.cc
- ORB-SLAM2 Tracking.cc（StereoInitialization z>0 检查）：https://raw.githubusercontent.com/raulmur/ORB_SLAM2/master/src/Tracking.cc
- SVO 1.0 depth_filter.cpp（seed 状态 mu/sigma2=z_range²/36、1px 视差噪声→τ、Gaussian+Beta 更新、收敛判据）：https://raw.githubusercontent.com/uzh-rpg/rpg_svo/master/svo/src/depth_filter.cpp
- VINS-Fusion feature_manager.cpp（stereo DLT/SVD 三角化、inverse depth 参数化、removeBackShiftDepth 深度传播）：https://raw.githubusercontent.com/HKUST-Aerial-Robotics/VINS-Fusion/master/vins_estimator/src/estimator/feature_manager.cpp

### 论文
- Civera, Davison, Montiel, "Inverse Depth Parametrization for Monocular SLAM", TRO 2008：http://webdiis.unizar.es/%7Ejcivera/papers/civera_etal_tro08.pdf
- Forster, Pizzoli, Scaramuzza, "SVO: Fast Semi-Direct Monocular Visual Odometry", ICRA 2014：https://rpg.ifi.uzh.ch/docs/ICRA14_Forster.pdf
- Forster, Zhang, Gassner, Werlberger, Scaramuzza, "SVO: Semi-direct Visual Odometry for Monocular and Multi-Camera Systems", TRO 2017：https://rpg.ifi.uzh.ch/docs/TRO17_Forster-SVO.pdf
- von Stumberg, Usenko, Cremers, "VI-DSO: Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization", ICRA 2018（ar5iv 全文）：https://ar5iv.labs.arxiv.org/html/1804.05625

### 二手资料
- VINS Feature Manager 源码解析（博客）：https://longer95479.github.io/vins-feature-manager/
- EuRoC V1_03/V2_03 为 difficult 序列、弱纹理（Semantic Scholar，RGB-D SLAM 论文引用）：https://www.semanticscholar.org/paper/Toward-Accurate%2C-Efficient%2C-and-Robust-RGB-D-and-in-Zhao-Gu/88b9a308e907efd5f80cbbe9369fdda24d484996
