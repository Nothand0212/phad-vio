# phad-vio 领域语言

本上下文描述双目视觉惯性里程计中的传感器、标定与估计概念，用于统一代码、
issue 和设计文档中的术语。

## 传感器与标定

**Body Frame（B）**：
VIO 的基准坐标系，固定为经过 adapter 规范化后的 IMU 坐标系。
_避免_：Base Frame、Rig Frame。

**相机外参（Camera Extrinsics）**：
把相机坐标系中的量转换到 IMU/body frame 的固定刚体变换。
_避免_：未在名称中标明方向的 Camera Transform。

**相机参数（Camera Parameters）**：
描述一台物理相机自身的成像参数、图像规格和采样频率，不包含它与其他传感器
之间的安装关系。
_避免_：Camera Calibration、Camera Type。

**相机模型（Camera Model）**：
定义三维相机坐标与二维图像坐标之间映射规律的几何模型。
_避免_：Camera Type、相机参数。

**IMU 参数（IMU Parameters）**：
描述 IMU 自身的采样特性和噪声特性，不包含恒为 identity 的 body-to-IMU
变换。
_避免_：IMU Calibration、未区分传感器属性与算法配置的 IMU Params。

**双目—IMU 标定（Stereo-IMU Calibration）**：
由左右相机参数、单个 IMU 参数，以及左右相机到 IMU/body frame 的外参组成
的固定传感器组合。
_避免_：Sensor Rig、Multi-Camera Calibration。
