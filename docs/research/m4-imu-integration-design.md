# M4:接入 IMU 设计(grill 定案)

日期: 2026-08-07
状态: 设计定案(grill-with-docs 会话产出;代码落地前若底层演进需重新对齐)
关联: issue [#27](https://github.com/Nothand0212/phad-vio/issues/27)(KF 设计,含 D3/D4)、[#23](https://github.com/Nothand0212/phad-vio/issues/23)(M3.3 总图)
前置: [M3.3 关键帧策略设计](m3.3-keyframe-design.md) §3 D3/D4、§4(M4 接口边界)
开源对照: [M3.3 关键帧开源对照](m3.3-keyframe-open-source-refs.md)(VINS / Kimera / Basalt 的 IMU 预积分与 KF 交互);GTSAM 本地 `thirdparty/gtsam-4.3a1/gtsam/navigation/{CombinedImuFactor,PreintegrationParams,PreintegratedRotation}.h`
相关: [sync AGENTS](../../phad/sync/AGENTS.md)、[estimator AGENTS](../../phad/estimator/AGENTS.md)、[sync README](../../phad/sync/README.md)

## 0. 决策来源

本设计由 2026-08-07 grill-with-docs 会话逐轮定案(每轮确认后再进下一轮),
决策链见 §1 定案总表。核心动机来自 pre-M4 小片实测:

- **pre-M4 round 2**(`prem4-round2_8906684_402d1925`): 匹配/播种层的 7 个变体
  在 V2_03 全灭 —— ATE 损失 100% 来自 re-anchor 对齐税(+2.9~+5.6 m),且与
  re-anchor 数量/门槛无单调关系;首段饿死不是绑定约束(Exp E 覆盖 0.766
  仍微劣于门 0.628)。**结论: 对齐税是结构性的,纯 VO 内无解,re-anchor 锚
  必须来自 IMU 预积分外推。** 这是 M4 的第一动机,先于纯精度提升。
- **M3.3 KF 设计 D3**: 预积分绑定**连续帧对**(KF→非 KF→KF 全链),KF 决定
  锚定/驱逐而非预积分边界;M4.2 落地。
- **M3.3 KF 设计 D4**: Basalt 的 KF 冷却(`min_frames_after_kf=5`)我们缺失,
  列入 M4 集成评估项(M4.4)。

## 1. 定案决策总表

### 1.1 根决策

| # | 决策点 | 定案 |
|---|---|---|
| D1 | 切片 | **三段式 + 收尾**: M4.1 数据路径 → M4.2 估计器机制(伪初始化,数字只记录不门控)→ M4.3 静止初始化 + 端到端门 → M4.4 Rule 4 升级 |
| D2 | 验收门 | **三重门**: ① MH_01 ATE **≤ 0.100 m**(绝对门)② 退化场景可测改善 ③ 机制证据(段结构 / 段间错位 / bias 收敛) |
| D3 | IMU 开关 | `estimator.enable_imu`(默认 true,进 `config_hash`);IMU-off 时 `est.tum`/`diag.csv` 相对 M3.3 基线(`4cf55ca/default_773ea011`)**逐字节相同** |

### 1.2 M4.1 数据路径

| # | 决策点 | 定案 |
|---|---|---|
| D4 | Packet 携带内容 | `StereoImuPacket` = `StereoFrame` + 帧间**原始 IMU 样本**(含两端插值样本);预积分在 estimator 构造,不在 sync |
| D5 | 插值规则 | 段 `[t_prev, t_cur]` 左端样本归**新段**;右端 = 恰在 t_cur 的插值样本;段内 ΣΔt ≡ 图像间隔;插值在 `tryPop()` 配对时完成 |
| D6 | 大间断 | 帧间 IMU 缺样本 > 阈值(默认 0.5 s)→ packet 标 `imu_gap`;estimator **跳预积分因子不跳帧**,视觉照常;阈值进 config |
| C1 | 实现位置 | sync 内完成(IMU 独立队列,未配对时只缓存) |
| C2 | 越界样本 | 早于首帧 / 晚于末帧的样本计入 `dropped_imu`,不参与插值 |
| C12 | 显式出口 | M4.1 后 `est.tum`/`diag.csv` 字节级不变(IMU 恒流但估计器未用) |

### 1.3 M4.2 估计器机制

| # | 决策点 | 定案 |
|---|---|---|
| D7 | 状态与因子 | X/V/B 三变量;`CombinedImuFactor(X_i,V_i,X_j,V_j,B_i,B_j)` + `BetweenFactor<ConstantBias>`(bias random walk);噪声单位转换只发生在 Params 构造一处 |
| D8 | 位姿初值 | **IMU 预积分外推优先**;PnP/恒速降级为 `imu_gap`/`enable_imu=false` 时的兜底;`pnp_success` 诊断语义重新定义(见 §4.5) |
| D9 | re-anchor | **整体退役**: 链跨段保持,锚 = 预积分外推位姿;事务回滚只回滚 landmark/观测,位姿与 IMU 段不回滚;仅 IMU 失效(`imu_gap`)或 IMU 关闭时启用 CV 锚兜底 |
| C3 | 预积分重算 | 每次窗口变更对窗口内所有相邻对重做预积分(10 帧 × ~10 样本,微秒级,不缓存) |
| C4 | 伪初始化 | `bias=0`、`v0=0`、`gravity=9.81007`(EuRoC 官方值),常数进 config 可调 |
| C5 | 初始 yaw | 置 0,由 ATE 的 SE3 对齐吸收(静止初始化只可观 roll/pitch) |
| C7 | bias 因子噪声 | `BetweenFactor<ConstantBias>` sigma 从 random walk 参数推导(见 §4.3);V/B prior 用中等 sigma(非 gauge 紧) |
| C11 | gauge prior | 最老帧 X prior 保留,sigma 从 `1e-4` 放松到 `1e-2` 量级 |
| C15 | bias prior 不扫参 | 按 C7 推荐值执行;仅当 M4.3 MH_01 门不过时再扫参 |
| C16 | 非 KF 帧初值 | 与 D8 相同: IMU 外推;非 KF 与 KF 无差别(⑤c 全帧 BA 保持) |

### 1.4 M4.3 初始化与端到端

| # | 决策点 | 定案 |
|---|---|---|
| D10 | 静止初始化期间视觉 | **等待**: 静止检测完成时刻 t0 为第一个状态帧,其前图像帧丢弃(约 10–20 帧);检测所用 IMU 段仍进入第一条预积分段 |
| D11 | 退化测试 | **双轨**: 出口门挂天然序列(MH_05 IMU-on 相对 M3.3 基线显著改善);合成注入做 CI 级机制测试(`--dropout-start/--dropout-frames`,session 级、不进 `config_hash`,参照 `enable_probe_b` 模式) |
| D12 | 机制证据 | 三合一: ① `segments`/`reanchors` 恒 1/0(IMU-on,D9 推论)② 逐段 ATE 分解的段间错位显著下降 ③ `diag.csv` bias 三轴 per-frame 输出 + 收敛用例 |
| C8 | 初始化丢帧计数 | `init_dropped_frames` 进 `summary.json`,不进 `config_hash` |
| C9 | 静止检测失败 | 返回明确错误+原因(不静默用伪初始化冒充成功);bench 中该序列 `kFailed` |
| C10 | 重力幅值 | 静止段测量 `\|mean(accel)\|` 优先,EuRoC 常数 9.81007 兜底;进 config |
| C13 | completion 语义 | 初始化丢帧使 completion 略降,只记录不门控 |
| C14 | TUM VI | M4 门只挂 EuRoC;TUM VI 全序列 record-only(T_BS 已隐含 identity,零变换成立) |

### 1.5 M4.4 收尾

| # | 决策点 | 定案 |
|---|---|---|
| D13 | Rule 4 升级时机 | M4.3 完成后独立小片: 旋转补偿来源 BA 位姿 → IMU 预积分;全序列 record-only + MH_01 不劣化;KF 冷却评估一并做(见 §6) |

## 2. 事实与参数(EuRoC / GTSAM 核实)

### 2.1 EuRoC IMU 标定(MH_01 实测)

来自 `mav0/imu0/sensor.yaml`,经现有 adapter 加载为 `ImuParameters`:

| 参数 | 值 | 单位 | 说明 |
|---|---|---|---|
| rate_hz | 200 | Hz | 图像 20 Hz 的 10× |
| gyr_nd | 1.6968e-4 | rad/s/√Hz | gyro 白噪声密度 |
| gyr_rw | 1.9393e-5 | rad/s²/√Hz | gyro bias 扩散 |
| acc_nd | 2.0000e-3 | m/s²/√Hz | accel 白噪声密度 |
| acc_rw | 3.0000e-3 | m/s³/√Hz | accel bias 扩散 |
| T_BS | identity | — | EuRoC adapter 硬性要求,非 identity 报 `kUnsupportedImuExtrinsics` |

**body-frame contract 已由 M1/M3.2 保证**: body 系 = IMU 系,IMU 测量零变换;
视觉因子的 `body_P_sensor = T_B_left_rectified()` 与 `StereoImuCalibration` 的
`T_B_left_camera` 共享同一 B(见 m2.3-vo-backend-design.md §2.1)。GTSAM
`ImuFactor` 的 body 系直接吻合,**不存在 IMU 坐标变换层**。

### 2.2 GTSAM 4.3a1 API(本地核实)

- `PreintegratedCombinedMeasurements::Params`(即 `PreintegrationCombinedParams`,
  继承 `PreintegrationParams`): `MakeSharedU(g)` = Z-up 世界系(gravity = `[0,0,-g]`);
  命名构造以外的构造须显式传重力向量;
- `setAccelerometerCovariance(Matrix3)` / `setGyroscopeCovariance(Matrix3)`: 连续
  时间噪声密度平方 —— `PreintegratedRotation.h` 注释明确 "The units for stddev
  are σ = rad/s/√Hz",即 **EuRoC 密度直接平方填入**;
- `setIntegrationCovariance(Matrix3)`: 随机游走密度平方(旋转积分误差);
- `setBodyPSensor(Pose3)`: body→sensor 变换,IMU 测量旋转到 sensor 系;本项目
  body=IMU 系,不设(或设 identity);
- `PreintegratedCombinedMeasurements(p, biasHat)`: 构造后 `integrateMeasurement(
  acc, gyro, dt)` 增量积分(GTSAM 注释建议"收到即积分";本项目在 estimator 按
  段积分,段长 ~0.05 s,开销可忽略);
- `CombinedImuFactor(X_i, V_i, X_j, V_j, B_i, B_j, preint)`: 六变量因子;
- `setBiasAccOmegaInit` **已废弃**("deprecated and no longer used"): 伪初始化
  bias 初值通过 V/B 变量初值表达,不进入 Params。

### 2.3 噪声单位换算(只此一处)

| Params 字段 | 填入值 | MH_01 数值 |
|---|---|---|
| `accelerometerCovariance` | `acc_nd² · I₃` | `4.0e-6 · I₃` |
| `gyroscopeCovariance` | `gyr_nd² · I₃` | `2.879e-8 · I₃` |
| `integrationCovariance` | `gyr_rw² · I₃`(旋转积分随机游走) | `3.761e-10 · I₃` |
| gravity | `MakeSharedU(\|g\|)` | `9.81007`(M4.3 起静止段测量优先,C10) |

`BetweenFactor<ConstantBias>` 噪声: bias random walk 离散化,sigma ≈
`rw · √dt`(gyr: `1.9393e-5·√0.05 ≈ 4.3e-6` rad/s;acc: `3.0e-3·√0.05 ≈
6.7e-4` m/s² 量级);V/B prior 用中等 sigma(如 `PriorFactor<Velocity>` 1.0 m/s、
`PriorFactor<ConstantBias>` gyro 1e-3 rad/s / acc 1e-2 m/s² 量级)。

## 3. M4.1 数据路径(`phad::sync` 扩展)

### 3.1 范围

- `StereoPairSynchronizer::pushImu(ImuMeasurement)`: 单相机单调性校验(逆序/
  重复/非有限 → sticky,沿用 M3.2 `PushStatus` 模式);IMU 队列有界 +
  drop-oldest 溢出计数(`dropped_imu` / `dropped_imu_overflow`);
- 配对成功(`tryPop()`)时从 IMU 队列切段 `[t_prev, t_cur]`(left stamp),做
  两端线性插值,构造 `StereoImuPacket`;
- 大间断检测与 `imu_gap` 标记(D6);
- 诊断: `pushed_imu` / `dropped_imu` / `dropped_imu_overflow` /
  `max_imu_queue` / `imu_gap_count`,并入现有 diagnostics 结构。

### 3.2 `StereoImuPacket` 语义

```text
StereoImuPacket {
  StereoFrame             frame;        // 已配对双目(left stamp = t_cur)
  std::vector<ImuMeasurement> samples;  // 原始样本 + 两端插值样本
  Timestamp               t_prev;       // 上一帧 left stamp(首帧为 t0)
  bool                    imu_gap;      // 段不完整(大间断)
}
```

- 段 `[t_prev, t_cur]` 的样本序列: 左端样本 = 恰在 t_prev 的原样本(若无则
  插值);中间 = 严格落在 `(t_prev, t_cur)` 的原样本;右端 = 恰在 t_cur 的
  插值样本。相邻段共享边界样本,但 ΣΔt 计算只按段内相邻样本差累加,
  **ΣΔt ≡ t_cur − t_prev 恒成立**(roadmap 测试项);
- `pushImu` 越界样本(早于首帧 / 晚于末帧)计入 `dropped_imu`(C2);
- sync 仍是纯队列: 不知道 bias/重力,不构造预积分(C1)。

### 3.3 测试矩阵(roadmap 逐项)

| 用例 | 断言 |
|---|---|
| 恰在样本上 | 图像时刻与原样本重合 → 直接用原样本,无插值误差 |
| 两样本之间 | 线性插值,ΣΔt ≡ 图像间隔 |
| 缺样 | 段内样本数少于期望 → 仍成立,ΣΔt 精确 |
| 重复 / 逆序 IMU | sticky 拒绝(pushImu 校验层) |
| 大间断 | `imu_gap = true`,estimator 跳因子(§4.6) |
| 越界样本 | 计数进 `dropped_imu` |
| 溢出 | drop-oldest + `dropped_imu_overflow` 计数 |

### 3.4 出口

- 上表全过(纯内存单测,不碰 `io`);
- `est.tum`/`diag.csv` 相对 M3.3 基线字节级不变(C12);
- `apps/StereoPairStream` 扩展为产出 `StereoImuPacket`(或新增方法),session
  仍按 `StereoFrame` 消费(IMU 段暂存不消费)。

### 3.5 实施状态(M4.1,已落地)

- [x] `pushImu` + 独立 IMU 队列/sticky/溢出计数(`phad/sync`,issue #31);
- [x] `StereoImuPacket` 类型(`phad/sensor/stereo_imu_packet.hpp`);
- [x] 配对时切段与两端插值:`t_prev` 经 `m_last_emitted_left` 链式传递,右端
      经 `m_last_boundary` 跨段共享;ΣΔt ≡ 图像间隔(测试断言);
- [x] `imu_gap` 检测(段宽 > `imu_gap_ns` / 两端无法构造 / 段内无样本);
- [x] 测试矩阵(§3.3 逐项 + 首帧零段 + sticky 独立性 + flush 计数,12 项新增);
- [x] `apps/StereoPairStream::nextPacket()` 产出 packet,`next()` 取其 frame
      (session 仍按 `StereoFrame` 消费);
- [x] 字节级回归: MH_01 `est.tum`/`diag.csv` 与 `4cf55ca/default_773ea011`
      逐字节相同(plan #31 出口)。
- 说明: 切段时早于首帧的样本计入 `dropped_imu`(C2);首帧 packet 为零段
  (`t_prev == t_cur`);`enable_imu` 开关与 `imu_gap_ns` 阈值在 M4.1 均**不进**
  `config_hash`(估计器未消费 IMU,run 身份不变,字节回归基准可比);两者经
  `StereoPairSynchronizerOptions` / CLI 可配置,待 M4.2 消费 IMU 时再按 §7
  纳入 config_hash。

## 4. M4.2 估计器机制(`phad/estimator`)

### 4.1 状态与因子图

- 窗口帧 `WindowFrame` 增加: `velocity_W`、`bias`(ConstantBias)、帧间 IMU 段
  (`StereoImuPacket` 的 `samples`/`t_prev`);
- 变量: `X(k)`(Pose3,沿用 Symbol)、`V(k)`(Velocity3)、`B(k)`(ConstantBias);
- 每对相邻窗口帧: `CombinedImuFactor(X(k−1), V(k−1), X(k), V(k), B(k−1),
  B(k), preint(k−1,k))` + `BetweenFactor<ConstantBias>(B(k−1), B(k), Δ=0,
  rw_noise)`;
- 视觉因子不变(GenericStereoFactor + `body_P_sensor`);
- 首帧: `PriorFactor<Pose3>`(gauge,sigma 放松至 1e-2,C11)、`PriorFactor<
  Velocity>(0)`、`PriorFactor<ConstantBias>(bias_init)`;
- 窗口驱逐与 M3.3 一致(7 KF + 3 temporal);驱逐最老帧时,其 V/B 变量与
  预积分段一并移除,窗口内剩余相邻对按 C3 全量重做预积分(新基准 = 新最老
  帧的 V/B)。

### 4.2 预积分构造

```text
Params::MakeSharedU(g)                       // Z-up 世界系
  → setAccelerometerCovariance / setGyroscopeCovariance /
    setIntegrationCovariance(§2.3)
  → PreintegratedCombinedMeasurements(p, biasHat)   // biasHat = 基准帧 B 值
  → 对段内样本按序 integrateMeasurement(acc, gyro, dt)
```

预积分在 `update()` 构建因子图时对窗口内每对相邻帧即时构造(C3)。噪声单位
转换只在 §2.3 的构造函数一处发生(roadmap 硬性要求)。

### 4.3 因子与初值

- **位姿初值 = 预积分外推(D8)**: 从上一接受帧的 `(X, V, B)` 出发,
  `preint.Predict(pose, vel, bias)` 给出本帧初值;非 KF 与 KF 无差别(C16);
- PnP/恒速初值保留为兜底: `imu_gap`(D6)或 `enable_imu=false`(D3)时走原
  M3.3 链(`pnp_success` 语义: IMU-on 下表示"兜底被使用次数",恒为 0 属正常);
- bias random walk: `BetweenFactor<ConstantBias>`(§2.3 数值);V/B prior 中等
  sigma(不扫参,C15)。

### 4.4 re-anchor 退役与事务语义(D9)

- `enable_imu=true` 时 re-anchor 代码路径**不可达**: 位姿链永不重置。窗口
  重建(观测不足触发,现 `min_seed_observations` 路径)只重建 landmark 表与
  观测,位姿从最后接受帧连续外推;
- **事务回滚语义修改**(本 slice 主要代码改动): 回滚不再覆盖位姿与 IMU 段,
  只回滚 landmark/观测/窗口帧结构;失败帧的 IMU 段照收,挂到待定段,下帧从
  最后接受位姿继续预积分;
- 兜底: `imu_gap` 或 `enable_imu=false` 时恢复原 re-anchor/CV 锚语义(与
  D3 的开关共用同一段代码);
- 诊断: `segments`/`reanchors` 在 IMU-on 下恒为 1/0,语义保留(D12 ①)。

### 4.5 伪初始化(M4.2 用,临时)

- `bias = 0`、`v0 = 0`、`g = 9.81007`(C4,C5),进 config;
- 目的: 机制先跑通(MH_01 开头静止,零 bias 初值也能收敛),M4.3 用检测值
  替换;M4.2 出口不挂 ATE 门。

### 4.6 与既有机制的交互

- 视觉退化/观测不足: 不拒帧(链跨段保持);观测全清时图 = 仅 IMU 因子 +
  prior,LM 有解(重力固定 + 最老帧 gauge prior);
- `imu_gap`: 跳该帧对的预积分因子,视觉照常,位姿初值走 PnP 兜底(D8)。

### 4.7 测试(合成对拍,新测试文件,不动现有)

| 用例 | 断言 |
|---|---|
| 静止 | 预积分 Predict 位移 ≈ 0,旋转 ≈ 0 |
| 匀速 | 位移 = v·Δt,误差 < 容差 |
| 恒定角速度 | 旋转 = ω·Δt,位置闭合(圆形/直线) |
| 已知 bias | first-order bias correction 与重新积分一致 |
| rad/s vs deg/s 错用 | 测试可检出(积分结果差 57.3×) |
| covariance | 对称且特征值在容差内非负 |
| ΣΔt | packet 段内 ΣΔt ≡ 图像间隔(§3.2 不变式) |

## 5. M4.3 静止初始化与端到端

### 5.1 静止初始化流程

```text
收集前 0.5 s IMU(100 样本)
  → gyro/accel 方差 < 阈值(gyro std < 1e-2 rad/s、accel std < 2e-2 m/s²,
    进 config,M4.3 扫参)→ 静止
  → gyro_bias = mean(gyro)
  → gravity_dir = 归一化 mean(accel)(IMU 系)
  → R_W_I0: 世界 Z 与 -gravity 对齐(roll/pitch);yaw = 0(C5)
  → v0 = 0;bias0 = (mean(accel) − g·gravity_dir 的残差, acc_bias, gyro_bias)
  → 建立 priors;t0 = 检测完成时刻
检测失败 → 返回错误+原因(C9),不静默降级
```

- 视觉等待(D10): t0 前图像帧丢弃,`init_dropped_frames` 进 summary(C8);
  t0 到首帧图像之间的 IMU 段仍进入第一条预积分段;
- 重力幅值: `|mean(accel)|` 优先,9.81007 兜底(C10);
- 运动中误初始化拒绝(roadmap 测试项): 合成数据构造运动段,断言拒绝。

### 5.2 端到端三重门(D2)

| 门 | 判定 | 数据来源 |
|---|---|---|
| ① MH_01 ATE ≤ 0.100 m | IMU-on 全序列 `phad_vo_bench` | `summary.json` ATE trans RMSE |
| ② 退化改善 | MH_05 IMU-on vs M3.3 基线显著改善;dropout 注入测试断言连续性与恢复性 | `summary.json` + 注入测试 |
| ③ 机制证据 | segments/reanchors 恒 1/0;逐段 ATE 分解段间错位显著下降;bias 从扰动初值收敛(见 5.3) | `diag.csv`(新增 bias 列)/ 逐段脚本 |

### 5.3 dropout 注入测试(D11)

- session 级 CLI: `--dropout-start-frame` / `--dropout-frames` /
  `--dropout-keep-ratio`(0 = 全清;0.3 = 部分退化),不进 `config_hash`
  (参照 `enable_probe_b` 模式);
- 对照 = 冻结语义(IMU-off 注入期位姿停在最后接受帧,恢复时重锚,产生对齐税);
  IMU-on 注入期位姿由预积分外推,恢复无缝;
- 断言: ① 注入区间内 IMU-on 轨迹相对真值偏差 ≤ 阈值;② 注入结束 50 帧内
  ATE 回到基线水平(无永久损伤);
- bias 收敛用例: 伪初始化给零 bias,跑 30 s 后 bias 与静止段检测值差 <
  阈值(gyro < 1e-3 rad/s 量级);`diag.csv` 增 bias 三轴 per-frame 列(IMU-off
  填 0/空,保证字节级回归成立,Q3/C12)。

## 6. M4.4 收尾小片

- Rule 4 旋转补偿来源: BA 位姿 → IMU 预积分(keyframe-design D3 已定,时机
  D13);A/B 同序列对比,全序列 record-only + MH_01 不劣化;
- KF 冷却(`min_frames_after_kf=5`)评估(D4): 有 IMU 数据后,用旋转段 KF 洪峰
  的触发率/密度数据决定是否加入;
- 30px 视差阈值复评: 旋转段由 IMU 承载后成为纯平移段参数,全表重测决定是否
  对齐 VINS 10px(开放问题 4)。

## 7. 模块边界与改动面

| 模块 | 改动 |
|---|---|
| `phad::sync` | `pushImu`、IMU 队列/插值/`imu_gap`、`StereoImuPacket`、诊断扩展(M4.1) |
| `phad/sensor` | `StereoImuPacket` 类型(或 `StereoFrame` 扩展) |
| `apps/StereoPairStream` | 产出 `StereoImuPacket`;session 消费侧暂不消费 IMU(M4.1)→ 消费(M4.2) |
| `phad/estimator` | X/V/B、预积分构造(§2.3 换算一处)、CombinedImuFactor、BetweenFactor、初值链切换(D8)、回滚语义(D9)、bias 诊断列(M4.3) |
| `apps/offline_vo_session` | dropout 注入 CLI(D11)、`enable_imu` 传递 |
| `phad::bench` | config 快照新键(自动);`init_dropped_frames` 进 summary schema(C8) |
| `phad::eval` | 不变(总图约束) |
| 测试 | `tests/sync`(packet 矩阵)、`tests/estimator`(合成对拍)、注入测试 |

`config_hash` 新增键(进 run 身份): `estimator.enable_imu`、imu 预积分参数
(重力/init bias/v0)、静止检测参数(§5.1)、gap 阈值(D6)。不进 `config_hash`:
dropout 注入参数、`init_dropped_frames`(C8)。

## 8. 测试与验收汇总

| 层 | 测试 | 出口 |
|---|---|---|
| M4.1 | packet 矩阵(§3.3)+ 字节级回归 | `est.tum`/`diag.csv` 不变 |
| M4.2 | 合成对拍(§4.7)+ MH_01 跑通 | 数字只记录不门控 |
| M4.3 | 三重门(§5.2)+ 注入测试 + bias 收敛 + 误初始化拒绝 | 三重门全过 |
| M4.4 | Rule 4 A/B | 全序列 record-only + MH_01 不劣化 |

## 9. 不做

- 不做在线 time offset / 外参估计(M9);
- 不做边缘化 / fixed-lag(M6);
- 不做 TUM VI 无静止段初始化(M5);
- 不扫 bias prior(仅门不过时按 C15 扫参);
- 不新建 synchronizer(只扩 `StereoPairSynchronizer`);
- 不改 `phad::eval`;不做 Kalman 类滤波(ADR-0001 冲突)。

## 10. 开放问题

1. **KF 冷却是否需要**(M4.4 用 IMU 数据决定,见 §6);
2. **30px 视差阈值复评**(M4.4);
3. **accel bias 可观测性**: 静止初始化只能粗糙给出 acc_bias(mean(accel) − g
   残差),优化中与 roll/pitch 的耦合如何收敛 —— M4.3 实测关注;
4. **IMU 失效兜底的对拍**: `imu_gap` 期间 CV 锚行为与 M3.3 的一致性,列入
   M4.2 测试矩阵补项。
