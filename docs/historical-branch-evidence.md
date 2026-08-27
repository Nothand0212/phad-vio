# 历史分支证据索引

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：2026-08-27

## 用途与边界

`main` 是产品行为、性能门和后续实现的唯一权威。下表记录本地历史分支的
定位，便于追溯曾经的问题、假设、实验和实现候选；它们不构成可直接迁移到
产品的补丁来源。

分支已进入 `main` 只表示其指定 HEAD 已在产品历史中可达。未进入 `main`
只表示该 HEAD 不是当前产品历史的一部分，不单独说明质量结论。任何历史内容
如需重新采用，必须在当前模块边界重新提出，并以当前配置与性能门独立验证。

三个 dirty 现场已由各自的 evidence snapshot 提交固化：`m4`、
`m4-minimal-gyro-slice` 与 `diag/m44-vio-causal`。其 README 含现场级说明。

## 分支目录

下表的 HEAD 是整理开始时的来源快照；“已进入 main”按该来源快照相对
`main@6a261c75f61194f2e7f05d6639562b6aa3c89d26` 判定。

| 分支 | 来源 HEAD | 与 main | 定位 |
|---|---|---|---|
| `Nothand0212/m4-gyro-requal-plan` | `c0e214a04f85` | 已进入 | M4 Q1 gyro qualification 记录 |
| `Nothand0212/m4-online-gyro-bias` | `54481908b3d7` | 未进入 | online gyro-bias authority 迭代 |
| `Nothand0212/m4-online-gyro-bias-authority-v2` | `d1c1638d3a4e` | 未进入 | online gyro-bias authority v2 证据 |
| `Nothand0212/m4-online-gyro-bias-authority-v3` | `6f1636dc8fb3` | 未进入 | online gyro-bias authority v3 证据 |
| `Nothand0212/m4-online-gyro-bias-authority-v4` | `47e9497e5ddd` | 未进入 | online gyro-bias authority v4 证据 |
| `Nothand0212/m4-online-gyro-bias-authority-v5` | `904eb03d73be` | 未进入 | online gyro-bias authority v5 证据 |
| `Nothand0212/m4-online-gyro-bias-authority-v6` | `634c4ecfbc5c` | 已进入 | online gyro-bias synthetic authority checkpoint |
| `Nothand0212/m4-online-gyro-bias-synthetic` | `9d860e7739b5` | 未进入 | online gyro-bias synthetic qualification |
| `Nothand0212/m4-online-gyro-bias-synthetic-replay2` | `a7f7a34265dd` | 已进入 | M4 online gyro-bias replay2 checkpoint |
| `Nothand0212/m4-q2-gyro-predict` | `3785acfc3122` | 已进入 | M4 Q2 prediction qualification |
| `Nothand0212/m4-q3-gyro-align-plan` | `dbec0be89512` | 已进入 | M4 Q3 alignment protocol stop record |
| `Nothand0212/m4-q3-protocol-amendment-plan` | `e258a1135613` | 未进入 | M4 Q3 protocol amendment |
| `Nothand0212/m4-q3-runner-protocol` | `0c74233bf0b5` | 已进入 | M4 Q3 runner protocol checkpoint |
| `codex/m4-observation-intake-continuity-1b` | `f3e91387fabe` | 已进入 | M4 continuity Slice 1b accepted implementation |
| `codex/m4-world-frame-inheritance-1a` | `4e8be1b1768d` | 未进入 | M4 world-frame inheritance candidate |
| `diag/m3-vo-control` | `cff47ad4980b` | 已进入 | 冻结的 M3 VO control group |
| `diag/m44-vio-causal` | `7f102cb059ff` | 未进入 | M4.4 causal diagnostic evidence snapshot |
| `m2.3-vo-backend` | `64e769b2ac1e` | 已进入 | M2.3 VO backend checkpoint |
| `m3.3` | `e9d74ed7eaf0` | 已进入 | M3.3 merged milestone |
| `m3.3-mh02-divergence-diagnosis` | `4e425176568f` | 已进入 | MH_02 divergence diagnosis |
| `m3.3-pnp-stereo-arbitration` | `afe3829e7a72` | 已进入 | PnP / stereo arbitration fix |
| `m3.3-slice-5` | `cff47ad4980b` | 已进入 | M3.3 Slice 5 checkpoint |
| `m4` | `f61d7d1fc0ca` | 未进入 | M4.4 candidate and causal evidence snapshot |
| `m4-imu-discussion` | `3aa7107da4d0` | 未进入 | M4 IMU design and open-source comparison |
| `m4-minimal-gyro-slice` | `a8e892fcccbe` | 已进入 | minimal gyro evidence snapshot |

## Detached 证据 checkout

`/home/lin/Projects/lin_ws/slam_ws/phad-vio-prototype-ticket-11` 固定在
`cf5e8cda15dda6c7bb787515ec7f4e017bb72f80`，当前为干净 detached checkout。
它不是可推进的本地分支；保留它作为 ticket 11 prototype 的可复现提交入口。

## 使用方式

1. 先以 `main` 的当前规格、代码和 benchmark 判断产品行为。
2. 需要历史背景时，从本表定位分支，再阅读其 commit、README、research、plan
   或 evidence snapshot。
3. 不以文件重叠、分支名称或历史 benchmark 单独推导可合入性；实现复用必须经过
   当前产品的设计、测试和性能资格门。
