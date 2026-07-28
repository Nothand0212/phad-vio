# 实现阶段与验收标准

本路线以“先证明最小数学闭环，再增加系统复杂度”为原则。阶段完成指
实现、定向测试和文档中的验收条件全部满足，不以“代码已写”代替完成。

## 阶段 0：设计合同

交付：

- 项目目标与非目标；
- 坐标系、pose 方向、重力和单位合同；
- 时间同步区间和边界插值合同；
- 最小状态和模块职责；
- 分阶段验收标准；
- GTSAM 后端选型 ADR。

验收：

- 任意 pose 符号都能从名称判断变换方向；
- IMU 静止读数的符号无歧义；
- bias 顺序和表达坐标系无歧义；
- 图像间 IMU 区间端点无歧义；
- 未确定的数据集噪声语义被明确标记，而非填入默认值。

当前状态：文档已建立，仍需在首次实现前由维护者审阅待确认事项。

## 阶段 1：EuRoC 数据加载

范围：

- CMake 项目骨架和测试框架；
- timestamp、IMU measurement、camera calibration 和 IMU calibration
  核心类型；
- 具体的 EuRoC dataset adapter，不提前创建通用 `DatasetInterface`；
- 从原生 EuRoC/ASL 目录解析 cam0、cam1、imu0 的 `sensor.yaml` 和
  `data.csv`；
- 使用整数纳秒建立严格有序的 IMU 和双目帧索引；
- 按 timestamp 对 cam0/cam1 做 exact join；
- 全清单校验和双目图像惰性解码；
- 数据集摘要检查工具。

测试：

- 最小 fixture 的标定、IMU 和双目索引加载；
- timestamp 保持整数纳秒且每个流严格递增；
- 左右目 timestamp 完全匹配；
- 图像只在请求时解码，且尺寸和 pixel type 符合标定；
- 缺文件、损坏图像、错误列数和非法数值；
- timestamp 溢出、重复和逆序；
- 左右目缺帧或 timestamp 不一致；
- 非法外参、相机模型和 IMU noise 字段；
- 可选的 `MH_01_easy` 本地集成测试验证 3682 对双目帧和 36820 条
  IMU 测量。

阶段出口：

- 能从原生 `MH_01_easy` 确定性加载标定、IMU 索引和双目索引；
- 第一、中间和最后一对图像均可按需解码；
- adapter 外不存在 EuRoC 路径、CSV 字段、坐标或单位转换；
- 常驻内存不随已遍历图像总数线性增长；
- 尚不执行 IMU 分段、边界插值、初始化、特征提取或 VIO。

详细设计与证据见
[EuRoC 数据集加载器设计调研](research/euroc-dataset-loader-design.md)。

## 阶段 2：纯 IMU 预积分

范围：

- bias 类型；
- IMU 输入校验；
- GTSAM preintegration 参数构造；
- `ImuFactor + bias BetweenFactor` 的两状态 batch graph；
- 合成数据生成器。

测试：

1. 静止：
   - 姿态保持；
   - 速度和位置保持；
   - PIM 总时间正确。
2. 匀速：
   - 零净加速度时速度保持；
   - 位置按 \(p=p_0+vt\) 变化。
3. 恒定角速度：
   - 旋转角和旋转轴正确；
   - rad/s 与 deg/s 错用能被测试发现。
4. 已知 bias：
   - 使用正确 bias 时恢复运动；
   - first-order bias correction 与重新积分结果在规定容差内一致。
5. 失败路径：
   - 逆序/重复 timestamp；
   - 非有限测量；
   - 非正 \(\Delta t\)；
   - 非法 noise 配置。
6. 数值检查：
   - covariance 对称；
   - covariance 特征值在数值容差内非负。

阶段出口：

- 合成场景结果与解析解一致；
- 所有 noise 字段的单位和转换有测试证据；
- 尚不消费图像或引入线程。

## 阶段 3：纯双目 batch bundle adjustment

范围：

- 针孔双目模型和标定；
- stereo observation；
- 三角化；
- 显式 landmark；
- `GenericStereoFactor`；
- 3 至 5 帧的合成 batch graph。

测试：

- 已知轨迹和 landmark 生成观测；
- 扰动初值后恢复 pose 和 landmark；
- 优化后重投影 error 下降；
- 只有一个明确的 gauge-fixing prior；
- behind-camera landmark 显式报告 cheirality；
- 错误外参用例失败，且诊断能定位外参合同。

阶段出口：

- 视觉图独立于 IMU 正确；
- 能按 landmark id 汇总完整 feature track；
- 不引入真实 LK 跟踪。

## 阶段 4：batch stereo VIO

范围：

- 合并阶段 2 的 \(X/V/B\) 状态与阶段 3 的 stereo factors；
- 5 至 10 个关键帧；
- 使用 `PIM.predict()` 构造新状态初值；
- bias random walk；
- 每类 factor 的 error 诊断。

测试：

- 合成视觉惯性轨迹恢复；
- 视觉短时退化时 IMU 保持状态连续；
- IMU bias 能从扰动初值收敛；
- 视觉与 IMU 时间错位用例产生可检测误差；
- 单独关闭一种 factor 后的行为符合预期。

阶段出口：

- 单线程 batch 图稳定可复现；
- 优化前后状态和每类 factor error 可检查；
- 不引入 smoother、smart factor 或线程。

## 阶段 5：数据同步与初始化

范围：

- 消费阶段 1 dataset adapter 的规范化输出；
- sensor synchronizer；
- 图像边界 IMU 插值；
- 起始静止初始化；
- 数据集片段回放。

测试：

- packet 的 IMU \(\sum\Delta t\) 等于图像时间间隔；
- 图像时刻恰好落在 IMU 样本上；
- 图像时刻落在两个 IMU 样本之间；
- 缺样、重复、逆序和大间断；
- 静止检测成功与运动中误初始化拒绝；
- 外参、时间偏移和噪声单位与数据集标定文件一致。

阶段出口：

- 真实数据的初始状态合理；
- 回放可重复；
- adapter 外不存在数据集特有坐标或单位转换。

## 阶段 6：Fixed-lag smoother

范围：

- `IncrementalFixedLagSmoother`；
- variable timestamps；
- 旧状态边缘化；
- bounded state 和 factor 生命周期。

测试：

- 长回放中状态数量保持有界；
- 边缘化时轨迹无不合理跳变；
- bias、velocity 连续；
- 与短片段 batch 结果在规定容差内一致；
- 内存不随帧数线性增长。

## 阶段 7：Smart stereo factor

范围：

- `SmartStereoProjectionPoseFactor`；
- feature track 到 factor 的生命周期；
- factor slot 更新与删除；
- 退化三角化处理。

测试：

- 与显式 landmark 版本的 pose 结果对照；
- 短 track、低视差、cheirality 和 rank deficiency；
- fixed-lag 边缘化后的 track 更新；
- factor 数量与活跃 track 生命周期一致。

## 阶段 8：LK visual frontend

范围：

- 特征检测和双目 LK；
- 时序跟踪；
- RANSAC 几何验证；
- stable landmark id；
- IMU rotation prediction；
- 关键帧策略。

测试与指标：

- 合成和数据集片段的 track 连续性；
- 左右目 epipolar error；
- RANSAC 内点率；
- landmark id 不复用；
- 低纹理、快速旋转和短时遮挡；
- frontend 输出严格满足 `KeyframeMeasurement` 合同。

## 阶段 9：线程与运行系统

范围：

- frontend/backend bounded queues；
- lifecycle 和 shutdown；
- 不可变估计快照；
- 日志、轨迹输出和可视化。

验收：

- 单线程与多线程在确定性输入上的结果差异可解释；
- 队列满时显式背压或报错；
- 后台异常不会变成假成功；
- shutdown 不丢失已接受但未处理的数据；
- ThreadSanitizer 或等价检查覆盖关键共享路径。

## 阶段 10：回环与全局图

范围：

- 独立的关键帧 global pose graph；
- odometry 和 loop constraints；
- world/odometry 校正变换；
- 局部 VIO 与全局修正解耦。

验收：

- 无回环时不改变局部 VIO 行为；
- 错误回环可拒绝；
- 正确回环降低全局漂移；
- global correction 不直接破坏 fixed-lag graph 中 \(X/V/B\) 的一致性。

## 暂缓项

在前述阶段完成前不排入实现：

- 在线空间外参和 time offset 标定；
- 单目尺度初始化；
- rolling shutter；
- 多相机；
- GNSS、轮速计或 LiDAR 融合；
- 稠密地图、mesh 和语义；
- 自研非线性求解器。
