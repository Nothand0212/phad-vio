# `phad::sensor` — agent 提示

模块合同见同目录 `README.md`。此处只记 agent 易踩的持久约定。

## 约定

- 叶子测量类型（`ImuMeasurement` 等）保持 POD + STL（`std::array`），便于序列化与简单拷贝
- `CameraId`、`ImageFrameEvent`（未配对单路）属本库；已配对双目是 `StereoFrame`
- Eigen 是本库 public 依赖（`RigidTransform` 等）；不要为叶子测量再引入重依赖
- 内部 API 用领域缩写（`accNd`、`gyrRw`、`fxPixels`）；单位写在注释里
- 外部数据集 YAML/CSV 键名保持原文完整拼写，转换在 `phad::io` adapter 内完成
