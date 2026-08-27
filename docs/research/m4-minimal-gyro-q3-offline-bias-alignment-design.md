# M4 Q3 offline constant-bias alignment 可执行设计

日期：2026-08-18

状态：**`PHAD-M4-Q3-GYRO-ALIGN-V1` 已预注册；`implementation_go=satisfied`；
`docs_gate=pending`，故 implementation 仍 STOP；qualification 与 result 均未执行。**

本文是 Q3 唯一 normative executable authority。2026-08-18 用户明确：不存在外部需求记录，
唯一目标是最终做出 VIO，工程实现选择授权团队。因此旧 O1E external locator/owner/requirement
路线已被取代，qualification authority 与 weight 均为 `0`；它只保留在历史调研中，不是 blocker，
也不得继续询问 locator。该后置用户指令同时构成
[Issue #39](https://github.com/Nothand0212/phad-vio/issues/39) 要求的 separate explicit Q3
implementation go；它仅对 Q3 生效，并 supersede Q2 result ledger 当时的 Q3 plan/design-only
权限上限，不修改、否认或追溯改写 Q2 的 technical verdict、waiver 与历史结论。

协议 ID 固定为 `PHAD-M4-Q3-GYRO-ALIGN-V1`。本文不写自己的未来 hash，避免自哈希
循环；本文所在 docs commit 与 clean worktree 形成后，必须在任何 dependency probe、
hardening 或 RED 之前由独立 preflight 将实际 protocol identity 写入唯一且不可覆盖的
write-once receipt。Git objects 是 identity 权威；receipt 是它与后续 build、qualification
及 result ledger 的可审计绑定，详见 §10。

关联文档：

- [Q1 Observe 结果](m4-minimal-gyro-q1-observe-result.md)
- [Q2 Predict 结果](m4-minimal-gyro-q2-known-bias-predict-result.md)
- [一手资料与历史调研](m4-minimal-gyro-q3-offline-bias-alignment-research.md)
- [实施计划](../plans/2026-08-13_m4_gyro_q3_align_0e897ba8.plan.md)
- [证据门控](../agents/evidence-gated-integration.md)

## 1. Claim 与权限

Q3 只检验以下命题：在冻结 visual-posterior proxy 上，用前 100 s eligible prefix 拟合的
constant gyro bias nuisance estimate，能否在不重叠的 suffix 1 s calendar blocks 上改善 rotation
consistency。

本结果不声称估计 physical gyro bias，不证明 online initialization、因子、posterior 或产品收益。
`PASS` 只授权另写 Q4 controlled-factor plan；它不授权 Q4 implementation，也不允许修改当前 VO。
Q3 qualification 不读取 GT、ATE、RPE、dataset 或任何四个冻结输入之外的 runtime 文件。

上述 `implementation_go=satisfied` 只授权：在 fresh docs review、全部返修、独立 docs verifier、
七文件精确 docs commit 与 clean worktree 全部闭合后，依次进入 eval hardening、exact RED、GREEN
synthetic verification 与恰好一次 frozen-input qualification。`docs_gate` 仍独立 pending，gate
闭合不会自动产生授权，而只是解锁已经取得的 Q3-only go；gate 闭合前仍禁止任何 implementation
动作。该 go 不授权 Q4 implementation、factor、posterior 或 production 行为。

## 2. Deep seam、CLI 与依赖

唯一纯分析接口是：

```cpp
GyroAlignmentResult analyzeGyroAlignment(const GyroAlignmentInput& input);
```

`GyroAlignmentInput` 必须已经 typed、parsed、cross-file joined，且不含 path、stream、threshold、
solver strategy 或 publish callback。`GyroAlignmentResult` 是纯值；分析器不打开文件、不写 artifact。

app 层新增唯一 runner deep module：

```text
apps/gyro_alignment_runner.hpp
apps/gyro_alignment_runner.cpp
```

它公开 owned-value `GyroAlignmentProtocolDescriptor`、唯一 production factory
`makeGyroAlignmentV1ProtocolDescriptor()` 与
`runGyroAlignment(descriptor, q1_dir, out_dir)`。descriptor 只包含 `protocol_id`、顺序和长度均冻结的
exact-four `basename + expected_sha256`、`protocol_identity`、`analyzer_identity` 与 `provenance`；所有
string、array 与 identity/provenance 成员都由 descriptor 自身持有。descriptor 不含 threshold、solver、
callback 或 path strategy。runner 是 app deep module，不进入 `phad/` libraries；它是 `lstat`、single-read
immutable buffers、hash-before-parse、strict parse 与 exact join、调用 analyzer、serialization、atomic
publish 和 stable error/exit mapping 的唯一所有者。analyzer 与下述 CLI 均不得复制这些职责。

唯一 CLI 是：

```text
phad_gyro_align <q1-run-dir> --out <new-output-dir>
```

factory 返回 owned value，并且是 exact V1 binding 的唯一 production authority：固化 V1 ID、§3 exact-four
basename/hash、preflight receipt 的 protocol identity、§3 Q1/Q2 provenance，以及由当前 build identity
生成的 analyzer identity。CMake/build 只把已冻结 receipt identity 与
`source_commit,source_tree,build_type,compiler,compiler_version` build identity 注入 runner target；factory
负责把它们物化为 descriptor，禁止其他 production 文件复制这些字段或另造 V1 descriptor。

`apps/phad_gyro_align.cpp` 必须是 thin main：只解析上述 exact CLI、调用
`makeGyroAlignmentV1ProtocolDescriptor()`、把返回值传给 runner 并返回其 exit；不得自行复制任一 V1
字段，也不得提供 test flag、environment override、descriptor CLI 或其他隐藏入口。production CLI 与
artifact 合同保持不变。依赖与 target 方向冻结为：

```text
phad_gyro_alignment        -> phad::estimator + Eigen3::Eigen
phad_gyro_alignment_runner -> phad_gyro_alignment + phad::eval
                              + nlohmann_json::nlohmann_json + OpenSSL::Crypto
phad_gyro_align            -> phad_gyro_alignment_runner
Q3 tests                   -> phad_gyro_alignment_runner
```

Q3 analyzer 不得直接依赖 GTSAM；rotation integration 只经 Q2 helper，其他矩阵、SVD 与 principal
SO(3) Log 只用 Eigen/std。runner 与 CLI 都不得依赖 `phad::io_dataset`、GT reader、ATE 或 RPE。

OpenSSL 只用于 Q3 runner 的 SHA-256/artifact identity，不扩散到其他产品库。future CMake 必须使用
`find_package(OpenSSL REQUIRED COMPONENTS Crypto)` 并新增上述 runner target；实施前须实际 probe
`OpenSSL::Crypto`，若不可用立即 STOP，不新增自研 SHA、shell helper 或替代依赖。

这一 seam 是必要而非测试专用绕路：V1 四个真实 SHA 已冻结，pure synthetic CLI success path 因而
不可达；把 protocol execution 收进可直接传 owned descriptor 的 runner，既让 synthetic tests 走与
生产完全相同的读取、校验、分析和发布路径，也保持 pure analyzer 与 production CLI 合同不变。
方案 B（conditional compilation/test-only flag 绕过 V1 identity）和方案 C（不测试成功的 CLI/runner
transaction path）均否决；禁止 hidden fallback、conditional compile 或 hash bypass。

synthetic tests 必须传固定非资格协议 ID
`PHAD-M4-Q3-GYRO-ALIGN-SYNTHETIC-TEST-V1`、exact-four fixture basenames 与由 fixture bytes 独立预冻结的
SHA-256；它们直接构造 synthetic descriptor，不得调用 V1 factory。synthetic protocol identity、
analyzer identity 与 provenance 使用 test 中独立预冻结且与 V1 binding 不同的 fixture-owned values；
它不得只把 factory descriptor 的 ID 改为 synthetic 而复用其余 V1 fields。其 artifact 只能作为
测试证据，V1 consumer/verifier 必须按 ID 拒绝；V1 qualification 仍必须同时
满足 exact `PHAD-M4-Q3-GYRO-ALIGN-V1`、冻结 protocol identity 与 §3 四个真实 hashes。runner 必须拒绝
用 V1 ID 搭配 synthetic/nonfrozen hashes 的 descriptor，synthetic descriptor 也不得冒用 V1 ID；
descriptor 身份不得改变 analyzer 的 threshold 或 solver 合同。

runner 必须在任何 `lstat`、open/read、output-directory 创建或其他 filesystem side effect 之前完整验证
descriptor。只接受 exact V1 factory binding 或上述固定 synthetic ID；unknown ID、V1 任一 canonical
ID/basename/hash/protocol identity/Q1-Q2 provenance/analyzer build identity mismatch，以及 synthetic 的
basename/order/exact-four shape 或字段 shape 不合法，都唯一返回
`HARD_ERROR/INVALID_PROTOCOL_DESCRIPTOR`, exit `1`，且不产生 artifact。synthetic fixture hashes/identity
的具体值允许由 tests 预冻结，但不因此获得资格；valid descriptor 下实际 input bytes 与其 expected hash
不符才是 `INPUT_HASH_MISMATCH`，不得用该 code 误报 descriptor 错误。

## 3. 冻结输入、provenance 与 parse 顺序

CLI 只能从 `<q1-run-dir>` 打开以下四个 basename；path 必须是该目录的直接 child，`lstat` 必须是
regular file 且不是 symlink：

| 顺序 | filename | expected SHA-256 |
|---:|---|---|
| 0 | `est.tum` | `18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321` |
| 1 | `diag.csv` | `1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb` |
| 2 | `gyro_packets.csv` | `fbf574545e418fc19d100b7b05f79546a5336420bca60852770605b42702a3fe` |
| 3 | `gyro_samples.csv` | `da35227b40a1ab47c94217b4210b5445d11927a864d6634f8b824812ccec0344` |

固定 provenance：

```text
source_run=q1_observe_final_q1_final_verifier
source_commit=74270572cc1fcc2eac82559117efd0951c800ab9
git_tree_object=e4379bf44db8d1127450d674b60b2e451fbe0aef
git_ls_tree_sha256=bb5a7854e4a3a89a52c8c0332e965474b8b9b2f2d4cbe0b83bcd7224e35f94ec
meta_sha256=bbeaa21edaee34733f61b8ef093d45d5b8f4268fc4c0a36da261ee6664e9fab1
config_hash=402d1925
input_manifest_v2_sha256=aee187f5fd147a1f44e6da2ff68a27cc0eed0c6de8cbbb22264ea7d899bfe5c5
q2_commit=d1c4385809a6bf461f1c6b8acd81870f98634aa0
q2_tree=9c9c991b21c4d40e8c5f2ce974334b76e1a42b64
```

`lstat` 通过后，每个文件必须只 open/read 一次，并把全部 bytes 保存在各自 immutable binary
buffer；随后关闭 file descriptor。size、SHA-256、strict grammar、parse 与全部下游 join 必须基于
同一个 buffer，禁止 hash 后按 path reopen、重新读取或让 parser 自行打开 path。四项 size/hash
全部命中后才允许解析。顺序固定为 TUM/CSV grammar 与 header → field types/finite → per-file
order/count/duration/endpoints → cross-file exact join → eligibility。provenance 是 Q1 ledger 引用，
不冒充四文件可重新证明的 runtime 字段。

为此，eval hardening 在 `phad::eval` namespace 新增明确的 public bytes seam：

```cpp
EvalResult<common::Trajectory> readTumBytes(std::string_view bytes, const std::filesystem::path& source_label);
```

`source_label` 只用于诊断，不得被打开。现有 path-based `readTum(path)` 自己完成一次 read 后委托
`readTumBytes`，且不得解包、折叠或丢失现有 `EvalResult<common::Trajectory>` typed error；通用 TUM
合同保持不变。Q3 adapter 则在同一 bytes 上先执行本节 stricter grammar，
再调用 bytes parser，禁止为了 Q3 改窄通用 `readTum` 的既有语法。三个 CSV adapter 同样直接消费
各自已 hash 的 immutable buffer。

### 3.1 Exact timestamp 与 join

所有内部时间戳及 JSON/CSV 输出时间均为 signed `int64` nanoseconds。CSV timestamp token 必须是
base-10 signed integer、无空白、无 `+`、无前导零（数值 `0` 除外）、禁止 `-0`，且可无损解析为
`int64`。Q3 TUM timestamp token 必须满足 `-?(0|[1-9][0-9]*)\.[0-9]{9}`，且禁止
`-0.000000000`；禁止舍入或浮点转换。例如 `-1.500000000` 精确还原为
`-1500000000 ns`。seconds×`1e9`、
符号处理和 fraction add/sub 都必须 checked；overflow 是
`HARD_ERROR`。

现有 `readTum` 的 signed multiply/add 尚未 checked。实施 analyzer 前必须先在
`phad/eval/tum_io.hpp`、`phad/eval/tum_io.cpp` 与 `tests/eval/tum_io_test.cpp` 增加上述 bytes seam、
checked arithmetic 和边界 tests；未完成该 hardening 不得进入 Q3 RED/GREEN。

精确 join：

- packet `t_cur_ns` 与唯一 diag `timestamp_ns` 一一对应，禁止任一侧 orphan；`vo_segment_id` 与
  diag `segment_id` 相等；
- valid packet 的 `t_prev_ns` 与 `t_cur_ns` 均须各命中唯一 endpoint diag；两端 diag 均为 `ok` 且
  两端 pose 均存在时，才可继续判定 structural eligibility。任一端 diag 为 `rejected` 或任一端 pose
  不存在时，该 nonfirst interval 按 §4 唯一记为 `diag_rejected_no_pose`，不得改用更早或更晚的 pose
  跨过该 endpoint 配对；两端 segment 不同只在前述条件通过后作 `segment_boundary` local exclusion，
  packet `vo_segment_id` 必须等于 current endpoint diag segment；
- 每条 pose 反向命中唯一 `ok` diag；`status=rejected` 恰无 pose，若却有 pose 则是 hard error；
  `status=failed` 是已知但不可局部
  排除的 hard error；unknown status 也是 hard error；
- pose、diag、packet 按 timestamp/index 严格递增且无 duplicate；sample 按
  `(packet_index,sample_index)` canonical order，index 从零连续。

### 3.2 Exact input grammar

三个 CSV header 必须逐 bytes 等于：

```text
timestamp_ns,status,num_obs,num_landmarks,num_shared,low_connectivity,window_size,prior_key,reproj_rms_before_px,reproj_rms_after_px,num_cheirality,lm_iterations,max_window_pose_shift_m,segment_id,pnp_success,pnp_inliers,outliers_culled,reproj_rms_after_cull_px,is_keyframe,num_disparity
packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,sample_count,sum_dt_ns,interval_ns,status
packet_index,sample_index,timestamp_ns,gyr_x_radps,gyr_y_radps,gyr_z_radps
```

`est.tum` 不允许 comment/blank data line；每行恰为八个 ASCII whitespace-separated fields：canonical
timestamp、`tx ty tz qx qy qz qw`。所有 floating token 都以 `std::from_chars` general format 消费
完整 token，且必须 finite；不允许 locale decimal comma。CSV 的 index/count/segment 列使用 canonical
unsigned decimal，signed duration/timestamp 使用 §3.1 grammar；所有 boolean 列只允许 `0|1`。

行只允许 LF 或 CRLF；无 quoted field、embedded comma、blank data row 或 extra column。packet status
只允许 `first_zero|valid|gap|empty_nonfirst`，diag status 只允许 `ok|rejected|failed`。
`gyro_packets.csv.status` 决定 interval first-match 的前两级：`gap`、`empty_nonfirst`；仅 `valid` 才继续
读取 prev/current 两端对应的 `diag.csv.status` 与 pose presence。此时任一端
`status=rejected` 或缺 pose 都唯一映射到 `diag_rejected_no_pose`；`rejected` endpoint 本身没有 pose
是预期输入状态，不得当作 join orphan、补 pose 或跨帧配对。只有两端 `status=ok` 且 pose 都存在，
才继续检查 segment 与 seven-point near-pi。
`first_zero` 必须且只能是 packet 0，零 interval、零 sample、`imu_gap=0`；它是 structural sentinel，
不属于 interval universe，也不计为 exclusion。它的 `t_cur_ns` 无条件定义 §5 的 `t0_ns`，不得由
eligibility、near-pi、fit status 或科学结果移动。

所有 nonfirst packet 必须 `t_prev<t_cur` 且 `interval_ns=t_cur-t_prev`。`valid` 必须
`imu_gap=0`、至少两条 sample、`sum_dt_ns=interval_ns`，且 sample 0/last timestamp 精确等于
endpoints。`gap` 必须且只在 `imu_gap=1` 时使用；其 samples（可为空）仍须 count/order/finite 且
`sum_dt_ns` 等于相邻 sample checked delta 之和。`empty_nonfirst` 必须 `imu_gap=0`、零 sample、
`sum_dt_ns=0`。所有非空 sample group 严格递增、三轴 finite。
全部整数 arithmetic checked；TUM 四元数及所有 pose translation/rotation field finite，四元数 norm
须满足 `abs(norm-1)<=1e-3`，再按 `qw,qx,qy,qz` 构造并归一化为 `R_WB`。任何
schema/order/count/duration/endpoint 破坏均为硬错误。

## 4. Eligibility、local exclusion 与 corruption

一个 interval **structurally eligible** 当且仅当：packet status 为 `valid`；prev/current 两个 endpoint
diag 均为 `ok`；两个 endpoint pose 均存在；packet/diag segment 相等且两 endpoint 位于同一
segment；所有 input checks 与 Q2 helper 调用成功。fit-eligible set 在 solve 前一次冻结：还要求
在七点 stencil 前通过本节的 finite-difference branch-safety guard，然后 `b=0` 与三轴
`+/-h` 的七次 packet residual 都可成功评估，之后 candidate `bhat` 不得反向改变该 set。
validation block 先要求
其 packets 全部 structurally eligible，再要求 `b=0` 与已经取得资格的 full-fit `bhat` 两次 block
residual 都不 near-pi；该 block-level local exclusion 不反向改变任何 fit。后文未限定的 eligible
按所在阶段分别指 frozen fit-eligible interval 或完整 validation block 的 structurally eligible
intervals。

唯一允许的 local exclusion reason 是：

```text
gap
empty_nonfirst
diag_rejected_no_pose
segment_boundary
incomplete_calendar_block
near_pi
```

`diag_rejected_no_pose` 是 interval-level 合并原因：prev/current 任一 endpoint diag 为 `rejected`，
或任一 endpoint pose 不存在，即命中该原因；不得为两个 endpoint 各计一次。对
`ok(t0) -> rejected(t1) -> ok(t2)`，`[t0,t1]` 与 `[t1,t2]` 两个相邻 interval 都命中该原因（除非
各自先命中更高优先级的 `gap`/`empty_nonfirst`），后者不得借用 `t0` pose 与 `t2` pose 组成跨帧
residual。exact near-pi 基础边界为 `eps_pi=sqrt(numeric_limits<double>::epsilon())`。对每个已通过
更高优先级 first-match 检查的 structurally eligible packet，在进入 `b=0,x/y/z +/-h`
七点 stencil 与任何 SVD 之前，固定 `h=1e-5 rad/s`，以该 packet 已 checked 且
严格为正的 `interval_ns` 计算权威 `T=checked(interval_ns * 1e-9)` 秒，再计算
`m_fd=nextafter(h*T,+infinity)`。`T`、`h*T` 或 `m_fd` 非 finite 是
`NONFINITE_COMPUTATION/HARD_ERROR`。先评估中心 arm 的 `(r0,theta0)` 及其 Q2/SO(3)/finite
合同；若 `pi-theta0 <= eps_pi+m_fd`，则按既有 first-match 唯一 local exclude 为
`near_pi`，不构造 `J`、不进入 SVD。等号必须排除。未排除时，该单 arm
margin 以 SO(3) geodesic triangle inequality 保证每个单轴 `+h` 或 `-h` arm 仍满足
`pi-theta > eps_pi`；margin 是 `nextafter(h*T,+infinity)`，不是 `2*h*T`。

通过 guard 后仍必须实际评估 **exactly seven points**：已评估的中心 `b=0`，以及
`x,y,z` 每轴分别的 `+h` 和 `-h`；不得评估、添加或用任何双轴/三轴 compound
arm 代替单轴 arm。每点仍执行实际 Q2 integration、finite、SO(3) 与 §6 Log 检查；
任一 typed Q2 error、nonfinite 或 SO(3) contract failure 仍是 `HARD_ERROR`，不得被
guard 隐藏。interval-level `near_pi` 只来自上述 solve 前 guard，并从 frozen fit set
排除；candidate `bhat` 不使用 `m_fd`，其 exact nonlinear residual 仍只按
`pi-theta <= eps_pi` 判定，首次触发时按 §6 记 fit observability failure，不是 local
exclusion。validation 中任一 block arm
触发 near-pi 时仍整 block 排除，不改成 hard error。incomplete block 只在 validation block 层计一次。

计数规则固定：每个 nonfirst interval 在 solve 前按 `gap -> empty_nonfirst ->
diag_rejected_no_pose -> segment_boundary -> near_pi` 的 first-match 只记一个 interval reason；只有
未命中 `gap`/`empty_nonfirst` 的 `valid` interval 才检查 prev/current 两个 endpoint，任一端 rejected
或缺 pose 即止于 `diag_rejected_no_pose`，不得再计 segment/near-pi；这里的
`near_pi` 仅指 frozen seven-point branch-safety guard。validation calendar slot 若边界/连续性不完整或含前
四类 excluded interval，记一次 `incomplete_calendar_block`；若结构完整但任一 block-arm Log
near-pi，只记一次 block-level `near_pi`。这些计数不参与 gate 分母。

以下全部是 `HARD_ERROR`，不得转成 exclusion：input hash、grammar/header/schema、unknown enum、
index/order/count/duration/endpoints、exact join、nonfinite、timestamp overflow、invalid quaternion；diag
`rejected` 却有 pose；任何 `failed` diag；任何 Q2 typed contract error；Eigen/Log/SVD 或
loss 中出现 nonfinite。

## 5. Split、calendar blocks 与 support

input contract 通过后，无条件令唯一 `first_zero` sentinel 的 `t_cur_ns` 为 `t0_ns`。按 canonical
packet order，对每个 structurally eligible packet 先执行 §4 的 `eps_pi+m_fd` branch-safety
guard；命中则 local exclude，否则完成 exact seven-point residual 并称为 fit-eligible。
`t0_ns` 不依赖该过程；以 checked add
得到：

```text
t_mid_ns   = t0_ns + 50_000_000_000
t_split_ns = t0_ns + 100_000_000_000
```

即使没有 nonfirst packet，或全部 nonfirst interval 都被合法 local exclude，`t0_ns`、`t_mid_ns`
与 `t_split_ns` 仍为非 null。该 split 是 sentinel-anchored protocol range；不得从 eligibility 或
结果另造、替换或移动 split。上述任一 checked add overflow 才是 `HARD_ERROR`；empty fit-universe
本身不是 hard error。

`full` 只含完整落入 `[t0,t_split]` 的 eligible intervals；`early` 只含完整落入
`[t0,t_mid]`；`late` 只含完整落入 `[t_mid,t_split]`。straddle interval 不切、不重采样；跨 `t_mid`
者可进入 full，但不进 early/late；跨 `t_split` 者不进 fit 或 validation。

validation 仅用 `t_split` 后的绝对 calendar blocks：

```text
B_k = [t_split_ns + k*1_000_000_000,
       t_split_ns + (k+1)*1_000_000_000], k = 0,1,...
```

令 `max_nonfirst_t_cur_ns` 为所有 nonfirst input packets 的最大 `t_cur_ns`，不受 local exclusion
影响。无 nonfirst packet 或 `max_nonfirst_t_cur_ns <= t_split_ns` 时，
`validation_slots_total=0`；否则以 checked subtraction 得正值
`delta_ns=max_nonfirst_t_cur_ns-t_split_ns`，并以 checked arithmetic 精确计算：

```text
validation_slots_total = ceil(delta_ns / 1_000_000_000)
                       = 1 + (delta_ns - 1) / 1_000_000_000
```

这恰是所有 slot 左端 `< max_nonfirst_t_cur_ns` 的 `k` 数量。slots 为 `0` 时
`validation_last_end_ns=null`；否则它无条件等于 checked
`t_split_ns + validation_slots_total*1_000_000_000`，即最后一个 slot 的右端，且不依赖 full fit。
乘加全部 checked。一个 block 只有首 endpoint 等于左边界、末 endpoint 等于右边界、内部 packets
exact 首尾相接、全部 eligible 且同 segment时才完整。缺块保留原 `k`，禁止压紧、merge、partial
integration 或 interpolation。block IMU rotation 按 packet 时间顺序 right-compose；visual rotation
只取 block 两端 pose。

minimum support 是 outcome-independent 工程边界：full eligible duration `>=90 s`，early/late 各
`>=45 s`，完整 validation blocks `>=60`。duration 是各 eligible packet 的 exact `interval_ns`
之和，checked 且不重复 endpoint。任一 support 不足为 `INCONCLUSIVE`。

## 6. Residual、loss 与 solver

任意 packet或block (u) 的 residual 为：

\[
r_u(b)=\operatorname{Log}\!\left(
R^{imu}_u(b)^\top R^{vis}_u\right)\in\mathbb R^3,
\qquad \ell_u(b)=r_u(b)^\top r_u(b)\ [\mathrm{rad}^2].
\]

每个 packet 或 block 的端点必须满足 `t_i < t_j`。TUM pose 是 body pose `T_WB`，visual rotation
唯一精确定义为：

\[
R^{vis}_u = R_{WB}(t_i)^\top R_{WB}(t_j).
\]

Q1 frozen input 中 body 即 IMU；不得应用 `T_B_left`、camera/body 变换或猜测任何 extrinsic。
`Rimu` 只能由 Q2 `integrateGyroRotation` 得到；block 必须按 packet 时间升序 right-compose 为
`Rimu_0 * Rimu_1 * ...`。residual matrix 唯一定义为 `Q = Rimu.transpose() * Rvis`；不得改变
visual/IMU transpose、乘法顺序或 axis。本文所有三向量分量顺序固定为 `x,y,z`。

`Log(Q)` 的唯一允许实现是 Eigen/std 下述 principal SO(3) Log。先要求 `Q` 全部 finite，再计算
`Q.transpose()*Q-I` 的 Frobenius norm 与 `det(Q)`；所有中间量必须 finite，且必须同时满足：

```text
frobenius_norm(Q.transpose()*Q - I) <= 1e-10
abs(det(Q) - 1)                     <= 1e-10
```

任一 finite 检查或 SO(3) 合同失败都返回 `NONFINITE_COMPUTATION/HARD_ERROR`；禁止投影、正交化或
修复 `Q`。随后严格按 `x,y,z` 构造并计算：

```text
v = 0.5 * [Q(2,1)-Q(1,2), Q(0,2)-Q(2,0), Q(1,0)-Q(0,1)]
s = norm(v)
c = std::clamp(0.5 * (trace(Q)-1), -1, 1)
theta = std::atan2(s, c)
```

`v,s,c,theta` 及 trace、norm、determinant、后续 scale/Log 必须 finite。fit stencil 中心点先按
§4 的 `pi-theta0 <= eps_pi+m_fd` 执行 branch-safety local exclusion；已通过 guard 的六个
single-axis arms 仍必须实际检查 `pi-theta > eps_pi`。candidate `bhat` 与 validation
arms 不使用 `m_fd`，仍只按 `pi-theta <= eps_pi` 执行各自的 §4/§6 分类。只有相应
near-pi 分类未命中时才继续：若 `theta <= eps_pi`，则 `Log(Q)=v`；否则必须 `s>0`，
且 `Log(Q)=(theta/s)*v`，`s=0` 为 `NONFINITE_COMPUTATION/HARD_ERROR`。禁止 `acos`、Eigen
`AngleAxis`/`Quaternion` 重投影、GTSAM `Logmap` 或任何替代分支。`1e-10` 是 team-owned 数值
合同，不是 scientific gate，不得从 qualification outcome 调整。

不得另写积分器、再次减 bias或改变上述 right-compose/transpose/order。

每个 loss 固定按 `x*x`、`y*y`、`z*z` 顺序以 binary64 相加；fit sum 按 packet index，validation
sum 按 block index，axis sum 按 block index。每次 addition 后检查 finite；禁止并行 reduction 或
按数值大小重排。

input contract 与 fit-universe 预计算（包括三个 fit 各自冻结的 rows、duration 与 count）全部完成后，
必须按 `full -> early -> late` 固定顺序分别求值。`support_insufficient`、`rank_deficient`、
`ill_conditioned`、`observability_failed`，以及 `solved` 但 `nonlinear_nonincrease=false`，都是该
sibling 的非 hard 结果；记录后必须继续求值后续 sibling，不得因已能确定 overall soft verdict 而
短路。三个 fit 各自最多独立执行一次 zero-linearized、unweighted LS；support 不足的 sibling 不进入
solver，其余 fit set 按 `packet_index` 升序，每个 residual 按 axis `x,y,z` 堆叠为 `r0`。只有
已通过 §4 `eps_pi+m_fd` guard 且完成 exact seven-point evaluation 的 rows 才能构造
Jacobian；命中 guard 的 row 不构造 `J`、不进入 SVD，而是按原 first-match 减少
support 并走 `FIT_SUPPORT`。Jacobian 使用同一 residual 与同一 Q2 helper，在 `b=0`
对 axis `x,y,z` 依次做 central finite difference：

\[
J_{:,a}=\frac{r(+h e_a)-r(-h e_a)}{2h},\qquad h=10^{-5}\ \mathrm{rad/s}.
\]

用 `Eigen::JacobiSVD` 求

\[
\hat b=J^+(-r_0).
\]

禁止 iteration、robust loss、weights、restart、clamp、bounds projection、fallback 或替代 solver。
SVD singular values 降序；`sigma_max` 必须 finite 且 `>0`，nonzero 判定严格为
`sigma_i > 3*epsilon_double*sigma_max`，且 rank 必须等于 3。condition 定义为
`sigma_max/sigma_min`，必须 finite 且 `<=1e6`。`bhat` 三轴必须 finite；本协议不另设 bias magnitude
bound。support/rank/condition 不足或 solver 无法观察为 `INCONCLUSIVE`。

每个 full/early/late fit 都须在 solve 前冻结的同一组 rows 上，用 exact nonlinear residual 复算
prefix loss；不得因 candidate 结果删除 row 或重跑 solve。若任一 frozen row 首次在 candidate
`bhat` 的 exact nonlinear Log 触发 near-pi，则该 fit 的 status 为 `observability_failed`，reason 为
`FIT_OBSERVABILITY`，overall 为 `INCONCLUSIVE`；candidate bias 不发布，loss/nonincrease 不发布，
也不得以删 row 后的 support 或 loss 继续判定。只有全部 frozen rows 可评估时才计算
`sum(loss(bhat)) <= sum(loss(0))`；等号通过。支持充分且 fit 为 `solved`、但
`nonlinear_nonincrease=false` 时为 `HYPOTHESIS_FAIL`。相反，任一 precompute 或 fit 阶段的 Q2 typed
error、Eigen/Log/SVD contract failure 或 nonfinite computation 必须立即停止整个 run，返回
`HARD_ERROR` 且不发布任何 artifact；它优先于此前已记录的全部 sibling soft status，后续 sibling
不再求值。

## 7. Scientific gates 与常数依据

没有 hard error 时，必须先完成 full、early、late 三个 sibling fit。validation block residual 只有
full fit 为 `solved` 时才可按 §9 的既有 prerequisite 求值；否则对应 nullable validation/support 字段
保持 null。只有 identity、contract、三次 fit 和全部 support 先成立，才评估 validation gates；
否则 `gates.evaluated=false` 且其余 nullable gate 字段保持 null。上述 validation/gates prerequisite
都不允许短路任一 sibling fit。validation 只用 full-fit `bhat`：

1. `sum(loss_fit) <= 0.8 * sum(loss_zero)` 且 `sum(loss_zero) > 0`；
2. `3 * n_improved >= 2 * n_blocks`，其中 improved 定义为该 block
   `loss_fit <= loss_zero`；
3. 对 axis `x,y,z` 分别满足 `sum(r_fit_axis^2) <= sum(r_zero_axis^2)`；
4. `max_axis(abs(b_early - b_late)) <= 1e-3 rad/s`。

等号按上式通过。四门全部通过才是 `PASS`；否则是 `HYPOTHESIS_FAIL`。

`20%` aggregate reduction 与 `2/3` improved blocks 是团队选择的最小、易解释效应门：前者要求
总 squared rotation inconsistency 有实质下降，后者阻止少数 block 独占收益。per-axis non-worsening
阻止 aggregate 掩盖单轴退化。`1e-3 rad/s` half-fit 差异门是 1 s 积分约 `1e-3 rad` 的工程量级，
不是物理 bias 规格。`1e6` 限制数值放大；`90/45 s` 要求各 fit 使用目标窗口至少 90% 的 eligible
时长；`60` blocks 给出至少一分钟独立 calendar units。以上都是 team-owned、outcome-independent
engineering boundaries，不冒充外部产品需求或统计显著性。冻结后不得在真实 qualification 同轮
放宽、重算或挑选门。

本最小协议不使用 HAC、TOST 或 power calculation：它不声称总体推断、equivalence 或显著性，
而是对一份固定资格输入执行确定性 effect/support gate。block 是唯一 validation unit，轴门以
全 blocks 的 sum 聚合；不把 packet 与 axis 冒充独立样本。因此增加 HAC/TOST/power 只会引入
未被当前 VIO 决策需要的 alpha、margin、covariance 与小样本假设，予以删除。

## 8. Verdict、error taxonomy 与 exit

CLI grammar 错误返回 usage `64`，不创建 output directory。接受 CLI 后，任何阶段发现 hard error 都
立即优先返回；若无 hard error，则必须完成三个 sibling fit 后才按下列唯一 precedence 汇总：

接受 CLI 后的第一项 runtime 动作必须是 §2 descriptor validation；其失败优先于
`INPUT_NOT_FOUND`、`INPUT_HASH_MISMATCH`、`OUTPUT_EXISTS` 与全部其他 runtime code，并保证尚未发生
filesystem side effect。

1. 任一 integrity/contract/finite/I/O/publish error → `HARD_ERROR`, exit `1`；
2. 否则任一 fit/validation support、rank、condition、observability不足 → `INCONCLUSIVE`, exit `3`；
3. 否则任一 nonlinear prefix 或 scientific gate 未达 → `HYPOTHESIS_FAIL`, exit `2`；
4. 否则 → `PASS`, exit `0`。

因此 sibling 顺序不改变 soft verdict：任一 `INCONCLUSIVE` 原因始终压过任一
`HYPOTHESIS_FAIL` 原因；但后续 sibling 的 hard error 始终压过此前任何 soft 状态。
一个 run 只发布一个 verdict；禁止在 JSON、manifest 与 process exit 中混用状态或以局部成功伪造
overall success。`HARD_ERROR` 不发布 completion manifest。

stable error taxonomy：

```text
INVALID_PROTOCOL_DESCRIPTOR,
INPUT_NOT_FOUND, INPUT_NOT_REGULAR_FILE, INPUT_HASH_MISMATCH,
INPUT_IO, INPUT_GRAMMAR, INPUT_SCHEMA, UNKNOWN_ENUM, DIAG_FAILED, INDEX_ORDER,
COUNT_MISMATCH, DURATION_MISMATCH, ENDPOINT_MISMATCH, JOIN_MISMATCH,
TIMESTAMP_OVERFLOW, NONFINITE_INPUT, INVALID_QUATERNION,
DIAG_POSE_CONTRADICTION, Q2_INTEGRATION_ERROR, NONFINITE_COMPUTATION,
OUTPUT_EXISTS, OUTPUT_IO, OUTPUT_PUBLISH, OUTPUT_REHASH_MISMATCH
```

local exclusion reason 只用 §4 的六值 enum，不得混入 error taxonomy。

## 9. Artifact schema 与 atomic publish

输出目录在启动时必须不存在；若已存在，返回 `HARD_ERROR/OUTPUT_EXISTS`，且不覆盖。成功或科学
非 PASS run 只包含三个文件：

```text
gyro_alignment.json
gyro_alignment_blocks.csv
gyro_alignment.manifest.json
```

### 9.1 Summary JSON v1

UTF-8、LF、无 BOM；使用 `nlohmann::ordered_json` 按下表 insertion order 构造，并以
`dump(2, ' ', false, error_handler_t::strict)` 加单个末尾 LF 序列化；consumer 仍按 key 名解析。
`schema_version` 固定字符串 `phad.gyro_alignment.v1`。以下 key 全部 required，写出顺序固定：

| key | type | required content / units |
|---|---|---|
| `schema_version` | string | exact schema id |
| `protocol_id` | string | production/qualification 必须为 `PHAD-M4-Q3-GYRO-ALIGN-V1`；synthetic runner test artifact 必须为 `PHAD-M4-Q3-GYRO-ALIGN-SYNTHETIC-TEST-V1`，且不可被 V1 consumer/verifier 接受 |
| `protocol_identity` | object | string `design_commit,design_tree,design_blob,design_sha256` |
| `analyzer_identity` | object | string `source_commit,source_tree,build_type,compiler,compiler_version` |
| `summary` | object | string `claim="frozen_visual_proxy_suffix_rotation_consistency"`; bool `offline_future_data=true,physical_bias=false,online_initializer=false,product_benefit=false`; string `bias_role="visual_posterior_aligned_nuisance",pass_authority="q4_controlled_factor_plan_only"` |
| `provenance` | object | 依次为 string `q1_source_run,q1_source_commit,q1_git_tree_object,q1_git_ls_tree_sha256,q1_meta_sha256,q1_config_hash,q1_input_manifest_v2_sha256,q2_commit,q2_tree` |
| `inputs` | array[4] | filename order；object `name:string,size_bytes:uint64,sha256:string` |
| `ranges` | object | sentinel-anchored int64 `t0_ns,t_mid_ns,t_split_ns`；nullable int64 `validation_last_end_ns`，按 §5 slot 公式写出 |
| `eligibility` | object | 按下述层级与 insertion order 记录 interval 与 block 计数 |
| `fit` | object | 按 `full,early,late`；每项见下 |
| `support` | object | int64 `full_duration_ns,early_duration_ns,late_duration_ns`; nullable uint64 `validation_blocks`; nullable bool `sufficient` |
| `gates` | object | 下述 exact fields |
| `verdict` | object | enum string `status=PASS|HYPOTHESIS_FAIL|INCONCLUSIVE`; int `exit_code=0|2|3`; string array `reason_codes`; string `detail` |

`eligibility` object 的 required insertion order 与单位固定为：

1. `packet_total_nonfirst:uint64`：input contract 全部通过后，除 `first_zero` sentinel 外的 packet row 数；
2. `packet_structurally_eligible:uint64`：对全部 nonfirst packets 应用 interval first-match 后，未命中
   `gap|empty_nonfirst|diag_rejected_no_pose|segment_boundary` 的数量；其中
   `diag_rejected_no_pose` 检查 prev/current 两个 endpoint，任一端 rejected 或缺 pose 都只计该
   interval 一次，计数时尚未评估 seven-point near-pi；
3. `packet_fit_eligible:uint64`：在上一层中再通过 solve 前 frozen `eps_pi+m_fd`
   branch-safety guard 并完成 exact seven-point evaluation 的数量；candidate `bhat` 不得
   改变该数；
4. `interval_excluded_counts:object`：按顺序写 uint64 `gap,empty_nonfirst,
   diag_rejected_no_pose,segment_boundary,near_pi`，每项单位都是 nonfirst interval，first-match
   恰记一次；这里的 `near_pi` 只计 solve 前 seven-point branch-safety guard exclusion；

上述 interval 层级必须使用 checked `uint64` 加法并精确满足以下恒等式；第一式的四个 reason 名均指
`interval_excluded_counts` 中的同名字段。任一加法 overflow 或恒等式不成立都是 `HARD_ERROR`：

```text
packet_total_nonfirst = gap + empty_nonfirst + diag_rejected_no_pose + segment_boundary + packet_structurally_eligible
packet_structurally_eligible = packet_fit_eligible + interval_excluded_counts.near_pi
```

5. `validation_slots_total:uint64`：严格按 §5 的 checked ceil 公式，记录从 `k=0` 开始所有左端
   `< max_nonfirst_t_cur_ns` 的 suffix calendar slots 数，包含尾部不完整 slot；无 nonfirst 或最大值
   `<=t_split_ns` 时为 `0`；
6. `validation_blocks_eligible:null|uint64`：full fit 未 `solved` 时为 null；否则为通过 structural
   completeness 且 zero/full-bias 两臂 near-pi 检查的 block 数；
7. `block_excluded_counts:object`：依次为 nullable uint64 `incomplete_calendar_block,near_pi`。
   `incomplete_calendar_block` 在 input/split 已知后即写实际 slot 数；`near_pi` 在 full fit 未
   `solved` 时为 null，否则写 block 数。`incomplete_calendar_block` 只依赖 sentinel-anchored split、
   slot 与 structural/local-exclusion checks，因此不依赖 full fit。仅当 full fit 为 `solved`、三项均
   可评估时，每个 slot 按 incomplete → near-pi first-match，且精确满足
   `validation_slots_total = incomplete_calendar_block + near_pi + validation_blocks_eligible`。

若 full fit 尚未 `solved`，`support.validation_blocks` 与 `support.sufficient` 都为 null；否则前者
逐值等于 `eligibility.validation_blocks_eligible`，后者对 90/45 s 与 60 blocks 三类 support
联合判定。任何 `HARD_ERROR` 都不发布 partial summary，因此上述计数只描述已通过 input contract
的同一 immutable buffers。

每个 fit object 的 required field 顺序固定为：`status,bias_radps,singular_values,rank,condition,
eligible_duration_ns,packet_count,loss_zero_rad2,loss_fit_rad2,nonlinear_nonincrease`。
`eligible_duration_ns:int64` 与 `packet_count:uint64` 对所有 status 均非 null；其他字段的 exact
nullability 为：

| `status` | `bias_radps` | `singular_values` | `rank` | `condition` | `loss_zero_rad2` / `loss_fit_rad2` / `nonlinear_nonincrease` |
|---|---|---|---|---|---|
| `support_insufficient` | null | null | null | null | 全 null |
| `rank_deficient` | null | array[3] | uint64 `<3` | null | 全 null |
| `ill_conditioned` | null | array[3] | uint64 `3` | double | 全 null |
| `observability_failed` | null | array[3] | uint64 `3` | double | 全 null |
| `solved` | array[3] double | array[3] | uint64 `3` | double | double / double / bool，全部非 null |

每个无 hard error 的 completed run 中，`full/early/late` 必须各恰有
`support_insufficient|rank_deficient|ill_conditioned|observability_failed|solved` 之一；public artifact
不存在表示 sibling 短路的 fit status。`solved` 且 `nonlinear_nonincrease=false` 仍保持
`status=solved`，并由 verdict reason 表达 soft failure。
`observability_failed` 只用于 frozen row 在 candidate `bhat` exact nonlinear residual 首次 near-pi。
该 candidate bias 不得出现在任何 artifact。JSON 禁止用 NaN/Inf 代替 unavailable。

`gates` required 顺序是：bool `evaluated`；nullable double
`validation_loss_zero_rad2,validation_loss_fit_rad2`；nullable uint64
`blocks_improved,blocks_total`；nullable array[3] double
`axis_loss_zero_rad2,axis_loss_fit_rad2`；nullable double
`half_bias_max_abs_diff_radps`；nullable bool `aggregate_reduction,improved_fraction,axis_nonworsening,
half_stability,all_passed`。`evaluated=false` 时 verdict-changing actual/booleans 全为 null；所有 JSON
double 必须 finite。

empty fit-universe 精确定义为 input contract 通过、但 `packet_fit_eligible=0`。此时禁止尝试 solver，
并按下列唯一编码发布科学非 PASS summary：

- `ranges.t0_ns/t_mid_ns/t_split_ns` 仍按 sentinel 与 checked add 为非 null；
  `validation_last_end_ns` 严格按 §5 slot 公式，slots 为 `0` 时才为 null；
- `full/early/late` 均为 `status=support_insufficient`、`eligible_duration_ns=0`、`packet_count=0`，
  其余 fit 字段全为 null；
- `support.validation_blocks=null`、`support.sufficient=null`；
  `eligibility.validation_blocks_eligible=null`、`block_excluded_counts.near_pi=null`，但
  `incomplete_calendar_block` 仍按 sentinel-anchored split 与 structural checks 写实际值；
- `gates.evaluated=false`，其余所有 nullable actual/boolean（包括 `blocks_total`）均为 null；
- `verdict.status=INCONCLUSIVE`、`exit_code=3`、`reason_codes=["FIT_SUPPORT"]`，不得添加其他 reason。

没有 nonfirst packet与全部 nonfirst interval 合法 local-excluded 都适用本合同；它们不得导致另造
split、升级为 `HARD_ERROR` 或尝试 solve。

`reason_codes` 按下列顺序筛选写出：`FIT_SUPPORT,VALIDATION_SUPPORT,FIT_RANK,FIT_CONDITION,
FIT_OBSERVABILITY,PREFIX_NONLINEAR_LOSS,VALIDATION_AGGREGATE,VALIDATION_FRACTION,VALIDATION_AXIS,
HALF_STABILITY`。`FIT_SUPPORT`、`FIT_RANK`、`FIT_CONDITION`、`FIT_OBSERVABILITY` 分别当且仅当
至少一个 fit.status 为 `support_insufficient`、`rank_deficient`、`ill_conditioned`、
`observability_failed`；`PREFIX_NONLINEAR_LOSS` 当且仅当至少一个 `solved` fit 的
`nonlinear_nonincrease=false`。`VALIDATION_SUPPORT` 只在既有 validation prerequisite 允许求值且
support 不足时出现。全部 reason 必须在三个 sibling fit 完成后按上述固定顺序汇总，soft 状态的
发现先后不改变 §8 precedence；`PASS` 必须为空。
`HARD_ERROR` 不产生可完成的 summary/manifest，错误仅以 §8 stable error code 与 process exit 报告。

### 9.2 Blocks CSV v1

header 必须逐 bytes 等于：

```text
schema_version,block_index,t_start_ns,t_end_ns,segment_id,packet_count,r_zero_x_rad,r_zero_y_rad,r_zero_z_rad,r_fit_x_rad,r_fit_y_rad,r_fit_z_rad,loss_zero_rad2,loss_fit_rad2,improved
```

每个完整且 zero/full-fit 两臂都已评估的 validation block 一行，按 `block_index` 严格递增；缺块或
full fit 未 solved 时不产生 data row。`schema_version`
每行固定 `1`；index/count/segment 为 unsigned decimal，timestamps 为 canonical signed decimal；double
用 classic locale 和 `max_digits10`、必须 finite；`improved` 为 `0|1`。CSV 不复制 bias、support、
aggregate gates 或 verdict summary。`packet_count` 只计该完整 block 内的 structurally eligible
interval；含 `diag_rejected_no_pose` 的 calendar slot 是 incomplete，因此不得产生 row，也不得出现
`t_start_ns <= rejected_timestamp_ns <= t_end_ns` 的跨 rejected endpoint block row。该 interval bucket 的
census 只写 summary JSON 的 `eligibility.interval_excluded_counts.diag_rejected_no_pose`，不向 Blocks
CSV 追加或暗藏新的 exclusion 字段。FD branch-safety guard 命中仍只使用既有
`near_pi`，不新增 status、reason、CSV 列或任何 schema 字段。

### 9.3 Manifest v1

`schema_version` 固定 `phad.gyro_alignment.manifest.v1`。manifest 的 `protocol_id` 必须逐值等于同一 run
summary 与 descriptor 的 ID；production/qualification 只能是 V1，synthetic runner test 只能是 §2 的
非资格 ID。required key/顺序为：

```text
schema_version:string
protocol_id:string
protocol_identity:object(design_commit,design_tree,design_blob,design_sha256:string)
verdict:object(status:string,exit_code:int)
science_outputs:array[2] of object(name:string,size_bytes:uint64,sha256:string)
inputs:array[4] of object(name:string,size_bytes:uint64,sha256:string)
```

`science_outputs` 顺序严格为 summary JSON、blocks CSV；`inputs` 顺序同 §3。manifest 不列自己，
不自哈希，也不包含 GT、ATE/RPE 或 dataset locator。

### 9.4 Publish transaction

在新建 output directory 内，两个 science output 各写唯一 temp path，依次 flush、close、rename；
随后重新打开 final bytes 计算 size/SHA-256 并与内存 ledger 比较。只有两项都成功，才写 manifest
temp，flush、close、rename 为最终名。任一阶段失败：返回 `HARD_ERROR`，不得留下 final manifest；
best-effort 删除本次创建的 temp/final science files，清理失败仍不得发布 manifest。consumer 只有验证 manifest schema、protocol
identity 与列出的全部 input/output size/hash 后才接受结果。

## 10. TDD、synthetic verification 与唯一真实 run

任何 implementation edit 前都有不可跳过的 docs gate。若 pre-gate 检查意外发现已有本轮
`docs/research/m4-minimal-gyro-q3-offline-bias-alignment-result.md` 或等价 Q3 V1 result
ledger，必须阻断 gate 并保留现场；禁止删除、改名或改写 ledger 绕过阻断。

本次 protocol amendment/link correction 已改变 candidate bytes，因此旧 candidate 上已有的 Standards/
Spec/docs-verifier gate 结论与 protocol preflight identity 均不能授权后续实现或 qualification；状态仍为
`docs_gate=pending`。旧 protocol/preflight receipts 必须原样保留，不得修改、覆盖、truncate 或删除；
新 candidate 必须按本节重新走完整 gate，并在闭合后写入新的 write-once identity receipt。

相对本轮 fixed point 的七份 docs 是一个不可拆分的 candidate：每一轮 fresh Standards
review 与 fresh Spec review 必须审查同一 candidate 的七文件 tree/blob bytes。任一轴产生
任一 finding 并导致任一返修时，两轴对旧 bytes 的全部结论同时失效；修订后
必须对新的同一 candidate 从头重跑两轴，直到同一 tree/blob bytes 上同时得到
Standards `0 findings` 和 Spec `0 findings`。随后由独立 docs verifier 只读复核这些
完全相同的 bytes 并 PASS；verifier 不得编辑任何文件。只能将该 bytes 不变地形成
exact seven-doc commit，然后确认 `HEAD` 等于该 docs commit 且 worktree clean。任一
tree/blob identity 或 byte 在两轴、verifier、commit、`HEAD` 或 clean 检查之间变化，必须
返回本循环起点，不得局部补审。gate 闭合前不得执行 dependency probe、eval
hardening、RED 或 GREEN。

docs commit+`HEAD`+clean 闭合后，且在任何 dependency probe、hardening 或 RED 前，
由独立 preflight 在 repo 外或 git-ignored build evidence 中创建唯一 write-once protocol
identity receipt。receipt 至少包含 `protocol_id`、docs commit/tree、七个 path 各自的
git blob 与 SHA-256，并显式包含 `design_commit,design_tree,design_blob,design_sha256`、
生成命令、cwd、时间、effective environment 和 tool versions。若目标 receipt 已存在，
禁止覆盖、truncate 或删除后重建；必须只读验证，内容不一致则 STOP。Git objects
始终是权威，receipt 只提供可审计绑定。该 identity 必须原样进入 compiled V1 factory 与 binary build
evidence；one-shot 前不得声称尚不存在的 qualification CLI artifact
已经包含该值。真实 one-shot 产出后，artifact 的 `protocol_identity` 必须与 compiled factory/build
evidence 逐值相等，result ledger 再原样引用 receipt；不得等到 artifact 或 ledger 才首次声称 identity。

自 preflight receipt 形成起，七 docs bytes 与它们的 blob identities 成为后续实现和
qualification 的冻结输入。此后任一 byte/identity 变化立即使 docs gate 失效；必须
STOP 并回到 Step 0 的完整双轴循环。已有 Q3-only go、hardening/RED/GREEN receipt
或“仅改文档”都不能绕过，重进 Step 0 也不授权覆盖或删除旧 receipt。

固定顺序为：**docs gate 循环 → preflight identity receipt → dependency probe/eval
hardening → RED → GREEN → implementation clean commit/tree 与 preflight identity 复核 →
独立 one-shot qualification → result ledger**。其中：

1. dependency probe 与前置 hardening：future CMake 先以
   `find_package(OpenSSL REQUIRED COMPONENTS Crypto)` 实际 probe `OpenSSL::Crypto`；通过后只修改
   `phad/eval/tum_io.hpp`、`phad/eval/tum_io.cpp` 与 `tests/eval/tum_io_test.cpp`，先为 bytes seam、
   单次读取、signed seconds×`1e9`/fraction add/sub overflow 写边界 test，再完成实现并固定 receipt。
   hardening 全部通过前不得进入 RED。
2. RED：只修改 `CMakeLists.txt`、`tests/apps/gyro_alignment_test.cpp`、
   `tests/apps/gyro_alignment_cli_test.cpp`，一次性写入最终 targets 与最终 tests。CMake 的 production
   targets 引用全部五个尚不存在的 app 文件。唯一有效 RED 是在五文件尚不存在时执行 fresh configure，
   configure 非零退出且完整 log 同时命中下述 required semantic pair：

   ```text
   failure_class=CMAKE_MISSING_PRODUCTION_SOURCE
   required_missing_source=(Cannot find source file) associated with apps/gyro_alignment.cpp
   required_empty_target=(No SOURCES given to target:) associated with phad_gyro_alignment
   ```

   pair 的可执行匹配固定为：

   ```sh
   perl -e '$s = do { local $/; <> }; exit !(
     $s =~ /Cannot find source file:\s*(?:[^\r\n]*\/)?apps\/gyro_alignment\.cpp(?:\s|$)/ &&
     $s =~ /No SOURCES given to target:\s*phad_gyro_alignment(?:\s|$)/
   )' <configure-log>
   ```

   该 matcher 容许 CMake 在 label/path 间换行，也容许 path 带绝对前缀；不依赖跨 target diagnostic
   emission order。该 class 唯一表示 missing production implementation。receipt 必须额外原样记录
   actual first diagnostic 作为证据，但它不参与 pass/fail。另保留 exact command、cwd、environment、fresh
   build dir、stdout/stderr、exit 与 log SHA-256。RED 阶段
   不编译或运行 tests；interface/symbol/test assertion 或“冻结行为尚未实现”均不是本协议的 RED 首败。
   dependency、compiler 或 unrelated failure 也无效。exact RED receipt 必须独立审核通过后才允许 GREEN。
3. GREEN：只新增 `apps/gyro_alignment.hpp`、`apps/gyro_alignment.cpp`、
   `apps/gyro_alignment_runner.hpp`、`apps/gyro_alignment_runner.cpp` 与
   `apps/phad_gyro_align.cpp`；不得在 GREEN 再改 tests/CMake。完成 synthetic verification 后
   形成 implementation clean commit/tree，并在 qualification 前复核该 `HEAD`、clean worktree、
   receipt 与该 `HEAD` 内七个 frozen docs blobs 全部一致。任一不一致都 STOP 并回到
   Step 0，不得进入 qualification。

synthetic tests 必须以 §2 固定的非资格 protocol ID 和 fixture hashes 走 runner 的真实
`lstat -> single read -> hash-before-parse -> strict parse/join -> analyzer -> serialize -> atomic publish`
路径，并覆盖 V1 ID 搭配 synthetic hashes 必须拒绝、V1 consumer/verifier 必须拒绝 synthetic artifact、
one-read immutable-buffer identity、strict-Q3/generic-TUM seam、known-bias oracle、zero bias、calendar gap、
half/full split、first_zero-only empty fit-universe、全部 nonfirst contract-valid 但合法 local-excluded 的
empty fit-universe、rank deficient、condition、candidate-bias `observability_failed` 且 candidate `bhat`
不使用 `m_fd`、principal Log analytic oracle、near-zero/near-pi branches、SO(3) contract hard error、
validation block near-pi local exclusion、层级 eligibility schema/nullability、support boundaries、
`full -> early -> late` sibling completion 与 soft no-short-circuit、早先 sibling soft failure 后后续 sibling
hard error 必须 overall `HARD_ERROR` 且无 artifact、`INCONCLUSIVE`/`HYPOTHESIS_FAIL` 两种先后组合仍由
前者优先、manifest-last 与 output-exists。公开 `GyroAlignmentResult`、artifact schema 与
`PASS|HYPOTHESIS_FAIL|INCONCLUSIVE|HARD_ERROR` 四态必须完整测试。

GREEN tests 还必须把 `makeGyroAlignmentV1ProtocolDescriptor()` 返回值与 independent frozen oracle
逐字段对拍：V1 ID、exact-four basename/hash、receipt protocol identity、全部 Q1/Q2 provenance，以及
由同一 build evidence 冻结的 analyzer build identity。随后对每个类别及其每个成员构造一次只改变
单字段的 V1 mutant，全部必须在零 filesystem side effect 前得到
`HARD_ERROR/INVALID_PROTOCOL_DESCRIPTOR`, exit `1`、无 artifact；unknown ID 与 synthetic
basename/order/shape mutants 同样覆盖，且不得误报 `INPUT_HASH_MISMATCH`。source/link review 必须证明
thin main 唯一调用 V1 factory，且 binary/main 中没有第二份 V1 ID、hash、identity 或 provenance binding。
synthetic tests 仍直接构造非资格 descriptor，不与 factory 共用 V1 constants。

边界证据不改变 §4、§6、§7 的公式、`<=` comparator 或 equality-pass/exclude 语义，但必须遵守以下
可执行性规则：

- integer gate 与 exact integer arithmetic 的边界必须直接命中并 exact 断言，例如
  `n_improved=40,n_blocks=60`、validation support `60` blocks，以及 exact `90/45 s` duration；
- 对 aggregate loss、per-axis loss、half-bias、condition、nonlinear nonincrease、near-pi margin 等由
  多步 binary64 运算组成的 gate，只有独立构造的 physical typed-input fixture 在所有支持工具链上
  稳定 bit-exact 可达时，才要求 equality fixture；否则必须在 RED 前预冻结两个 physical typed-input
  fixtures，从 pass/fail 两侧夹住阈值。独立 oracle 对每侧记录实际 metric、阈值相对位置，以及
  非零 ULP margin 和数值 margin；两侧都必须执行 production analyzer，再由 source review 确认
  production comparator 是 `<=`。fixture 不得在 runtime 搜索，不得从 DUT output 反求，不得增加
  stats-centered public seam、scalar comparator、callback、counter 或 test-only hook；
- `exactly seven points`、guard 后不构造 `J`/不执行 SVD，以及实际 `full -> early -> late` 顺序，使用
  联合证据：mutation-sensitive black-box physical fixtures、test-local independent mutant oracle 与
  source review。不得为这些内部事实新增 public instrumentation seam。black-box 断言必须落在公开
  result、四态、error、artifact 或 census 上，而不是 production callback/counter。

FD branch-safety 仍须对 axis `x/y/z` 和两个 signed crossing direction 分别构造 `T=1 s`、
`theta0=pi-5e-6` 的 physical fixture；旧七点各自通过 `eps_pi` 但跨越 principal branch，production
结果必须唯一分类为 `near_pi`，并用上述联合证据证明 guard 阻止后续 Jacobian/SVD。另以
`T=0.1 s` 与 `T=1 s` 的预冻结 physical fixtures 验证 `m_fd` 按 `h*T` 缩放；first-match census、
`FIT_SUPPORT` 与公开 schema 不变。near-pi composite equality 本身仍服从上一段的 bit-exact-or-bracket
规则，不得以运行时 `nextafter` 搜索或不可重复 hex 推导冒充可达 equality。

rank-deficient fixture 可以循环生成 `4500` 条、每条 `T=0.02 s` 的 physical rows；每个 half 恰
`2250` 条，每 row 中心旋转为 `2*pi` 且 `Rvis=I`，从而 exact 满足 `90/45 s` support，且无需 public
matrix seam。condition 与 candidate observability fixture 只需在 RED 中冻结 physical typed inputs
及独立 oracle，不要求本文抄录完整 hex。

mutants 至少覆盖 bias sign、visual transpose/frame/`T_B_left`、compose order、axis order、rad/deg、
double subtraction、gap compression、遗漏 FD margin、FD `ns/s` 单位错误、用 `2*h*T` 过度排除、
遗漏任一 signed axis arm、添加/替换 compound arm、hash-after-parse、hash 后 path reopen 与 early
manifest；每类必须被独立 oracle 拒绝。`<=` 改为 `<` 仅在独立证明 bit-exact physical equality
fixture 稳定可达时用 mutation test 拒绝，否则由双侧 bracket 与 source review 提供证据。
deletion check 只能在 repo 外的 GREEN archive 中执行：保留最终 `CMakeLists.txt` 与两个 tests，只删除
五个 app 文件，再用 fresh build dir configure；必须复现与 RED receipt 完全相同的
`CMAKE_MISSING_PRODUCTION_SOURCE` failure class 与上述 required semantic pair；deletion run 的 actual
first diagnostic 仍须原样记录，但允许与 RED 不同且不参与 acceptance。不得在工作仓库删除文件，也不得
把 test failure 当 deletion 证据。CLI target 的 transitive link map 必须证明无
`phad::io_dataset`、GT reader、ATE 或 RPE symbol。

另有一个冻结 `ok(t0) -> rejected(t1) -> ok(t2)` endpoint oracle：在各 interval 未先命中 gap/empty
的 fixture 中，相对三端均 ok 的 control，断言 `packet_total_nonfirst` 不变、
`interval_excluded_counts.diag_rejected_no_pose` 恰增加 `2`、
`packet_structurally_eligible` 与 `packet_fit_eligible` 均恰减少 `2`，其他 interval exclusion census
不变，且 §9.1 两条 interval 恒等式仍成立。两条相邻 interval 均不进入 structural/fit rows，绝不生成 `[t0,t2]` 跨 rejected frame
的 residual 或跨越 `t1` 的 Blocks CSV row；fixture 保留足够 support 裕量，并断言正确 local exclusion
不改变 control 的既定 support 判定与最终 verdict。

实现和 synthetic verification 固定成 clean commit/tree 并通过上述 preflight identity 复核后，
独立 verifier 才能执行**恰好一次**真实 qualification。不得预跑、窥视、重跑选优或在
同轮调门。资格前任一七 docs blobs、receipt、implementation `HEAD` 或 clean worktree
不一致，都必须 STOP 并回到 Step 0；禁止执行真实 run。结果无论四态哪一种都
原样进入 immutable result ledger，并原样引用早已创建的 preflight receipt。one-shot
之后新增 ledger 不破坏七文档 freeze，但不授权回填计划 todo，也不授权第二次 run。

## 11. Future allowlist 与 forbidden scope

post-gate implementation 阶段唯一可修改 allowlist：

```text
CMakeLists.txt
apps/gyro_alignment.hpp
apps/gyro_alignment.cpp
apps/gyro_alignment_runner.hpp
apps/gyro_alignment_runner.cpp
apps/phad_gyro_align.cpp
tests/apps/gyro_alignment_test.cpp
tests/apps/gyro_alignment_cli_test.cpp
phad/eval/tum_io.hpp
phad/eval/tum_io.cpp
tests/eval/tum_io_test.cpp
```

七份预注册 docs 不在 post-gate implementation allowlist，必须保持 §10 冻结 identity。真实且唯一一次
qualification 完成后，允许新增并写入
`docs/research/m4-minimal-gyro-q3-offline-bias-alignment-result.md` 作为真实 Q3 result ledger；它是
post-qualification evidence output，不是被冻结七文档之一，也不是借机改写七文档的权限。

forbidden：其他 source/tests/CMake、现有 app/session/bench/frontend/estimator/sensor/sync/io 行为、
Q2 helper public contract、配置 hash、dataset/output/control、GT/ATE/RPE、factor、posterior、Q4 source、
online initialization、runtime tuning knob、fallback、第二积分器与新 production dependency。当前 docs
修订不代表这些未来实施步骤已经完成。
