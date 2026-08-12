# 本地 Git worktree 用途登记

日期：2026-08-12

本文档记录本机当前已注册 worktree 的用途，避免把诊断实验、历史原型和可合入开发混用。
它是操作清单，不是架构或 milestone 状态的权威来源；实际 HEAD 与占用关系仍以
`git worktree list` 为准。worktree 的用途或处置状态变化时应同步更新本表。

| 路径 | 分支 / HEAD | 用途 | 当前处置 |
|---|---|---|---|
| 仓库根目录 | `m4` / `7f102cb` | 保存 M4.3/M4.4 P2b 尚未提交的大工作区，以及本轮 fixed-lag candidate 诊断文档 | 冻结取证；不作为新实现基线，不覆盖其已有修改 |
| `../phad-vio-m4-imu-discussion` | `m4-imu-discussion` / `3aa7107` | M4 IMU 接入设计与开源实现对照（issue #28）的历史讨论分支 | 只读参考 |
| `../phad-vio-prototype-ticket-11` | detached / `cf5e8cd` | sensor public calibration interface 的历史原型审阅 | 与当前 M4 重启无关；保留待所有者处置 |
| `.worktree/m3-vo-control` | `diag/m3-vo-control` / `cff47ad` | 干净 pre-M4/M3 VO 控制组；已用于复跑 MH_01，验证 `0.0809641/0.0177812 m` 基线 | 只读控制组；不在其中开发 |
| `.worktree/m44-vio-causal` | `diag/m44-vio-causal` / `7f102cb` | 承载 P2b candidate 的单变量因果实验，包括 AHRS、KF epoch、PnP、bias 与常速初值实验 | 冻结实验现场；不合入 production |
| `.worktree/main` | `main` / `7026ebf` | 本登记文档的 main 工作区；`7026ebf` 已含 M4.1 sync/`StereoImuPacket`，尚未含 P2b candidate | 只维护 main 侧登记；不在此实施新 slice |
| `.worktree/m4-minimal-gyro` | `m4-minimal-gyro-slice` / base `7026ebf` | 从 main 重新开始：保持 M3 `StereoVoEstimator` 视觉链，只实现最小 gyro-aided VO slice | 方案与计划 `c4e62b35` 已通过；issue #36 实施中 |
| `/tmp/phad-vio-sensor-boundaries.97tD7v` | detached / `1fa08e0` | Git 仍留有的临时 sensor-boundary worktree 登记，目录已不存在 | `prunable`；本轮不擅自清理 |

## 使用约束

- 新的可合入工作只在 `.worktree/m4-minimal-gyro` 进行，并以 `main@7026ebf` 为基线。
- 根目录 `m4` 和 `.worktree/m44-vio-causal` 只保留证据；不得从中整批复制 candidate
  架构或把实验性修改误当 production 变更。
- `.worktree/m3-vo-control` 仅用于重跑冻结 VO 对照；若控制组产物变化，应先停止实施并
  解释差异。
- 未经明确确认，不删除 worktree、不 commit、不 merge、不 push。
