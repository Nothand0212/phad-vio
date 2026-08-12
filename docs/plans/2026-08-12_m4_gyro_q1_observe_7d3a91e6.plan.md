---
name: M4 gyro Q1 Observe
overview: 仅实施 Q1 Observe：在不改变 M3 visual posterior、配置身份和三主产物的前提下，同时冻结 packet summary 与可脱离 dataset 重放的 raw gyro samples。Q2–Q5 只记录未来依赖与禁止越过的边界，不进入本计划 todos。
todos:
  - id: freeze-fixed-point
    content: 确认 GitHub issue 36 已同步新权威链接；以 a8e892f 为 design review / pre-Q1 code fixed point，按相对 7026ebf 的 runtime source diff 及 MH_01 control 指纹冻结实际实施起点
    status: pending
  - id: contract-red
    content: 先为 gyro_packets.csv / gyro_samples.csv 的 validator/writer、round-trip、local-invalid、hard-fail 与可控 open/post-flush 失败补 portable RED tests；成功 CLI 双 CSV 与 byte gate 只进 MH_01 qualification
    status: pending
  - id: implement-current-edge
    content: 仅在 apps composition root 采集 StereoImuPacket、校验 Q1 合同并写两个 CLI-only artifacts；estimator 仍只接收原 visual measurement
    status: pending
  - id: run-qualification-gate
    content: 跑定向 unit 与 MH_01 CLI qualification，核验 config_hash=402d1925、config 完整 object/canonical text 精确不变及 est.tum/kf.tum/diag.csv 的 cmp/SHA256 byte gate
    status: pending
  - id: record-stop-go
    content: 记录命令、输入/产物 SHA256、status 计数和 pass/fail/inconclusive；只有 Q1 pass 才给 Q2 known-bias synthetic integration 发出 go
    status: pending
isProject: false
---

# M4 gyro Q1 Observe 实施计划

状态：**待实施；当前本地唯一已定义的 M4 gyro 可执行范围，开工前还需同步 issue 权威链接**

计划 ID：`7d3a91e6`

跟踪 issue：[#36](https://github.com/Nothand0212/phad-vio/issues/36)

远程前置：截至 2026-08-12 本计划修订时，#36 的 body 仍把历史
`c4e62b35` 作为权威计划并授权 `off/shadow/fused` 旧路线。coordinator 尚在等待
用户对远程写入的授权；在 #36 的 body/验收标准改为本计划与修订设计的链接、
并且只读回验成功前，**不得开始 Q1 实施**。本地文档修正不表示 issue 已同步。

上位合同：

- [证据门控的信息接入](../agents/evidence-gated-integration.md)
- [M4 gyro measurement / factor 资格实验设计（修订版）](../research/m4-minimal-gyro-slice-design.md)
- [MH_01 控制组](../research/m4-minimal-gyro-mh01-control.md)

历史计划：
[M4 最小 gyro-aided VO `c4e62b35`](2026-08-12_m4_minimal_gyro_slice_c4e62b35.plan.md)
只供审计，已失效且不得执行；其中旧实现状态不是本计划的资格证据。

## 1. 本片问题、权限与成功定义

本片只回答：**M4.1 产出的每个 `StereoImuPacket` 是否能以确定、可重放、可审计的
时间/单位/区间/gap 合同落盘，同时保持现有 M3 VO 输出完全不变？**

本片给 gyro 的唯一权限是采集、校验和落盘。gyro 不得进入 estimator input、初值、state、
residual、factor、KF decision、PnP、cull/reopt、re-anchor 或 tracker feedback。

Q1 `pass` 必须同时满足：

1. fixed point、source diff 和 control 指纹成立；
2. 正例、边界例、local-invalid 与 hard-fail tests 通过；
3. MH_01 成功运行至 EOS，两个 gyro artifact 内部可离线重建且全量不变量成立；
4. `meta.json.config_hash == "402d1925"`，`meta.json.config` 完整 object 与 control
   精确相同，`config_canonical_text` 与 control 逐字节一致；
5. `est.tum`、`kf.tum`、`diag.csv` 与权威 control 分别 `cmp` 相同且 SHA256 精确命中。

本片不以 ATE 改善为目标，也不产生 bias、PIM、alignment、factor 或 gyro posterior。

## 2. Fixed point 与冻结证据

### 2.1 代码身份

- design review / pre-Q1 code fixed point：`a8e892fcccbe`（短 hash `a8e892f`）。
- M3 runtime 比较锚：`main@7026ebf`；它是 source diff 基准，不是对实施时 `HEAD`
  的相等要求。
- 已验证事实：`a8e892f` 相对 `7026ebf` 在
  `CMakeLists.txt apps phad tests scripts` 无差异，因此运行代码等价。
- 实际 Q1 实施起点可以是 `a8e892f` 的 docs-only descendant；只要开工前下述
  runtime source diff 为空、control 指纹不变，`HEAD != a8e892f` 不构成阻塞。
- Q1 实现后另行记录实现/run commit；它相对 `7026ebf` 只能包含第 3.1 节
  allowlist 内的 Q1 差异，不要求实现 commit 等于 design fixed point。
- 独立 verifier 已在新 build 运行 `ctest -L unit`：55 项中 52 passed、3 skipped，
  exit 0；这只证明 Q0 fixed point 可用，不冒充 Q1 已实现或已通过。

在任何 Q1 source 写入前重新记录：

```bash
git rev-parse HEAD
git merge-base --is-ancestor a8e892f HEAD
git status --short
git diff --exit-code 7026ebf -- CMakeLists.txt apps phad tests scripts
```

开工前最后一条必须为空 diff。实现后的 qualification run 则以
`git diff --name-status 7026ebf -- CMakeLists.txt apps phad tests scripts` 和第 3.1 节
allowlist 对拍，并以第 2.2 节 control hashes 判定 runtime 冻结是否仍成立。
若 ancestry、runtime diff、control 指纹或工作区范围与这些规则冲突，停止并重新冻结；
不得把旧 dirty worktree 的 C++、tests、scripts 或 build 复制进来。

### 2.2 MH_01 control

```text
dataset=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy
control=/home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control
config_hash=402d1925
est.tum.sha256=18fc4aa6f54977b7bb8386f0b13f4b50b3d67efa5cc61f99982d2ea9bf349321
kf.tum.sha256=4a4a1ba8f7fb2729e482d5aca644c9195ad605ccc0e94ffb9e1e670d1f898bb9
diag.csv.sha256=1da9df9ae56dd9d552187128f1295d5153aa29c379cc366147ab1910116c51eb
```

`meta.json.config` 作为完整 JSON object 与 control 精确比较，
`config_canonical_text` 作为完整 string 逐字节比较；不硬编码 key 数，也不能只比较
hash 名。当前 control 的 `config` 与 canonical text 实测均为 45 个键/行，
该数字只作本次 control 诊断注记，不是实施合同。

## 3. 修改边界

### 3.1 Source allowlist

只允许以下实现/测试路径发生 Q1 所需修改：

| 路径 | 唯一允许的改动 |
|---|---|
| `apps/offline_vo_session.hpp` | 增加 Q1 packet/sample row 值类型、结果集合与纯 writer/validator 声明；不增加 estimator gyro API |
| `apps/offline_vo_session.cpp` | 将 session 的读取从 `next()` 换为同源 `nextPacket()`；frame 仍走原 rectify/tracker/glue/estimator 路径；从 packet 拷贝 Q1 artifact rows |
| `apps/phad_vo_bench.cpp` | 增加 `--gyro-observe` CLI-only flag；成功 run 时在既有 output dir 写两个独立 CSV |
| `tests/apps/gyro_observe_artifact_test.cpp` | portable validator/writer schema、round-trip、边界、local-invalid、hard-fail 与可控 writer 失败 |
| `tests/apps/offline_vo_session_test.cpp` | 仅在现有文件最小覆盖 packet 合同或失败传播；不用 tiny fixture 充当成功 bench artifact 资格证据 |
| `tests/apps/phad_vo_bench_cli_test.cpp` | portable CLI presence 与失败路径；缺 dataset 时不伪造成功 artifact |
| `tests/apps/gyro_observe_mh01_test.cpp` | 仅在 `PHAD_ENABLE_MH01_TESTS=ON` + `PHAD_EUROC_MH01_PATH` 下跑成功 CLI、双 CSV 与三主产物 byte gate；归入 `mh01` label |
| `CMakeLists.txt` | 将 portable tests 接入既有 `phad_apps_tests`；按仓库既有 MH_01 模式在 `PHAD_ENABLE_MH01_TESTS` 下增加 `phad_apps_mh01_test` 并标记 `mh01`；不加生产依赖 |

若实现不需要其中某文件，就不改。若必须改 allowlist 外 source，视为合同冲突，停止并重新评审。

### 3.2 Forbidden list

- 禁止修改 `phad/estimator/**`、`phad/frontend/**`、`phad/sync/**`、
  `phad/sensor/**`、`phad/bench/**` 与 `scripts/**`；
- 禁止新增 `GyroMode`、gyro estimator options、alignment window、PIM、bias、factor、
  graph state、packet merge、全帧 state 或第二套 estimator；
- 禁止改 `diag.csv` schema、`est.tum`/`kf.tum` 格式、M3 config defaults 或
  `flattenConfig()` 的 canonical key 集；
- 禁止让 artifact writer 读取 estimator PIMPL。`vo_segment_id` 只能取同一帧已公开的
  `UpdateDiagnostics.segment_id`，packet 的其他列全部取自 `StereoImuPacket`；
- 禁止从旧 dirty worktree 导入任何 C++、test、script 或 build 产物。

## 4. 两个同级 Q1 artifact

`--gyro-observe` 启用时，两文件必须同时生成；缺任一文件的 run 不可进入 Q2。该 flag
可映射为 `OfflineVoSessionOptions::collect_gyro_observe=false` 这类 CLI-only collection
开关，但不调用 `ConfigSnapshot::set()`、不成为产品配置，也不改变
`config_canonical_text` / `config_hash`。

### 4.1 共同序列化规则

- 编码为无 BOM 的 ASCII/UTF-8；换行固定 LF；逗号分隔；header 与下列字符串逐字节一致；
- packet 与 sample 均按 `StereoPairStream::nextPacket()` 的产生顺序写；index 从 0 开始、连续；
- bool 固定写 `0|1`；整数用十进制、无 `+` 和千分位；
- gyro double 使用 classic locale 和 `std::numeric_limits<double>::max_digits10`，禁止
  fixed precision、NaN、Inf 或 locale decimal comma；解析后必须 bit-exact round-trip；
- 相邻 packet 共享的边界 sample 在两个 packet group 中各写一次，不跨 packet 去重；
- 即使没有 sample，`gyro_samples.csv` 也保留 header；每个 CSV 先写入与其最终路径
  同目录的 temp file，只在该文件 write/flush/close 成功后 rename 到最终路径。
  rename 只保证**单文件** publish 的原子性，不宣称两个 CSV 具有跨文件事务原子性；
- 任一 CSV 的 open/write/flush/close/rename 失败均返回 `SessionError` 并使整次 run
  非零退出。失败时可能已 publish 其中一个文件；消费方只能在 run 成功、
  两文件都存在且完整验证通过时认定 Observe 成功，不得将失败 run 的
  单文件或 temp file 当作资格证据。

### 4.2 `gyro_packets.csv`

header：

```text
packet_index,t_prev_ns,t_cur_ns,vo_segment_id,imu_gap,sample_count,sum_dt_ns,interval_ns,status
```

| 列 | 类型 / 单位 | 精确来源与语义 |
|---|---|---|
| `packet_index` | uint64 / 无单位 | emitted packet 的 0-based 连续序号 |
| `t_prev_ns` | int64 / ns | `packet.t_prev.nanoseconds()` |
| `t_cur_ns` | int64 / ns | `packet.frame.timestamp.nanoseconds()` |
| `vo_segment_id` | uint32 / 无单位 | 同一 packet 的 `update.diagnostics.segment_id`；只作 join，不反向控制 dump 或 VO |
| `imu_gap` | bool `0|1` | `packet.imu_gap` |
| `sample_count` | uint64 / count | `packet.samples.size()` |
| `sum_dt_ns` | int64 / ns | 按 sample 顺序对相邻 timestamp 差做 checked integer 累加；少于 2 个 sample 为 0 |
| `interval_ns` | int64 / ns | checked `t_cur_ns - t_prev_ns` |
| `status` | enum string | 下述四个可落盘状态之一 |

先执行本节 hard-failure 校验，再按以下优先级分类可落盘 status：

1. `first_zero`：仅 `packet_index=0`，且 `t_prev_ns==t_cur_ns`、`sample_count=0`、
   `imu_gap=0`；
2. `gap`：非首包且 `imu_gap=1`；区间缺证据，属于 local-invalid，视觉继续；
3. `empty_nonfirst`：非首包、`imu_gap=0`、`sample_count=0`；属于 local-invalid，
   不得作为后续机制证据；
4. `valid`：非首包、`imu_gap=0`、至少 2 个 sample，首末 timestamp 精确等于端点，
   严格递增且 `sum_dt_ns==interval_ns>0`。

下列情况不降级为普通 status，而是 hard failure：首包不满足 `first_zero`；非首包 interval
非正、反向或 checked subtraction/accumulation overflow；sample timestamp
duplicate/out-of-order；gyro
非有限；非 gap 的 endpoint mismatch 或 interval unclosed。错误 detail 使用稳定前缀
`gyro observe:` 加具体 code（如 `sample_duplicate`、`sample_out_of_order`、
`sample_nonfinite`、`endpoint_mismatch`、`interval_unclosed`）；不得静默改成 `gap/off`。
来自 `StereoPairStream` 的 IMU sticky error 同样保持 terminal error，不伪造 packet row。

### 4.3 `gyro_samples.csv`

header：

```text
packet_index,sample_index,timestamp_ns,gyr_x_radps,gyr_y_radps,gyr_z_radps
```

| 列 | 类型 / 单位 | 精确来源与语义 |
|---|---|---|
| `packet_index` | uint64 / 无单位 | 外键到 `gyro_packets.csv.packet_index` |
| `sample_index` | uint64 / 无单位 | `packet.samples` 内 0-based 连续序号 |
| `timestamp_ns` | int64 / ns | `sample.timestamp.nanoseconds()` |
| `gyr_x_radps` | double / rad/s | `sample.gyro_radps[0]` |
| `gyr_y_radps` | double / rad/s | `sample.gyro_radps[1]` |
| `gyr_z_radps` | double / rad/s | `sample.gyro_radps[2]` |

这里的 raw gyro samples 指 M4.1 packet 已发布的 `samples`（含必要的线性插值端点），不是
重新读取 dataset CSV；现有 packet 未暴露 original/interpolated provenance，因此 Q1 不虚构
该字段。Q1 不落 accel，因为本资格链的当前新信息只有 gyro rotation measurement。

脱离 dataset 的 replay 必须只用两 CSV 完成：按 `packet_index` 分组、按 `sample_index`
恢复顺序，断言 sample group 数量等于 `sample_count`，重算 status / `sum_dt_ns` / endpoints，
并以 `t_cur_ns` join `diag.csv.timestamp_ns`、以 `vo_segment_id` 检查 segment。后续 Q2/Q3
不得再次实现 sync 或读取 EuRoC IMU CSV。

### 4.4 Hash 合同

Q1 run 完成后对两个 CSV 的**完整文件 bytes**分别计算 SHA256；hash 与文件大小、row count、
首末 packet timestamp 一并写入 `docs/research/m4-minimal-gyro-q1-observe-result.md`。后续工具
消费前必须重算并匹配；任何字节变化都需要新的 Q1 资格 run。artifact SHA 不写入
`flattenConfig()`，也不修改 `meta.json.config` / `config_canonical_text`；配置身份与输入证据
身份保持分离。

## 5. RED → GREEN 实施顺序

### 5.1 `freeze-fixed-point`

执行第 2 节命令，复核 control 目录 `meta.json`、三个 SHA256 和 canonical text。此时不以
旧 plan 的 pending/completed 状态或旧 dirty build 作证据。

### 5.2 `contract-red`

先提交会失败的最小测试（本轮不 commit，仅保持 RED 证据可审查）。portable unit
只覆盖纯 validator/writer 与失败路径：

- 正常：首包零段；两端精确的 constant-rate packet；多 packet 顺序；共享端点保留；
- 边界：gap 有/无 samples、非首空段、负/零/极小 gyro、`max_digits10` round-trip；
- 失败：首包非零、反向 interval、timestamp duplicate/out-of-order、非有限三轴、错误端点、
  `sum_dt` 不闭合、checked integer overflow；
- writer 失败的 portable 硬测只覆盖可控的 open error 与 post-flush publish/rename
  error。若注入 write/close error 需要新 seam，本片不为此引入投机性 seam；
- portable CLI 只验证 usage 含 `--gyro-observe` 及缺 dataset / writer 失败时非零退出、
  不伪造成功 artifact。

tiny EuRoC fixture 不足以证明生产 bench 成功路径能完整产生双 CSV，也不能
代替三主产物 byte gate。成功 CLI 的双 CSV、默认/Observe `config` 完整 object、
`config_canonical_text`、`config_hash` 与三主产物字节不变性，只在
`PHAD_ENABLE_MH01_TESTS=ON` + `PHAD_EUROC_MH01_PATH` 的 MH_01 qualification
与第 6.2 节真实 run 中验证；不得用 unit 结果代替。

### 5.3 `implement-current-edge`

最小实现顺序：

1. session 改取 `StereoImuPacket`，立即保留 packet 引用/值；只有 `packet.frame` 进入原
   rectify→tracker→glue→estimator 路径；
2. 同一 update 返回后，使用公开 diagnostics 补 `vo_segment_id`，运行纯 Q1 validator，
   收集 packet/sample rows；不把 samples 传入 estimator；
3. bench 只在 `--gyro-observe` 且 session 成功时 publish 两个 CSV；每个文件按
   第 4.1 节同目录 temp+rename 独立原子 publish，任一 writer/publish error 传播为
   整次 run 失败；
4. 保持 `writeDiagCsv()`、TUM writer、`flattenConfig()` 和现有浮点求值顺序不变。

若切换 `nextPacket()` 本身导致三主产物变化，先定位 ownership/order 问题；不增加兼容分支或
绕过 byte gate。

## 6. 验证与真实 MH_01 gate

### 6.1 定向构建与测试

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DPHAD_BUILD_TESTS=ON \
  -DPHAD_ENABLE_MH01_TESTS=ON
cmake --build build --target phad_sync_tests phad_apps_tests phad_apps_mh01_test phad_vo_bench -j2
build/phad_apps_tests --gtest_filter='*GyroObserve*:*OfflineVoSession*:*VoBenchCli*'
ctest --test-dir build -L unit --output-on-failure -j2
PHAD_EUROC_MH01_PATH=/home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  ctest --test-dir build -L mh01 --output-on-failure -j2
```

`ctest -R phad_(sync|apps)_tests` 会误选 NOT_BUILT placeholder，不能代替 `-L unit` 的有效
执行；报告实际通过/跳过/失败数与 exit code。`-L unit` 不得包含成功 CLI
artifact qualification；该测试只在 `PHAD_EUROC_MH01_PATH` 存在时由 `-L mh01`
执行。

### 6.2 MH_01 实跑

使用全新、不覆盖 control 的目录；`Q1_OUT` 在结果文档中替换为实际绝对路径：

```bash
build/phad_vo_bench \
  /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --gt-euroc /home/lin/Projects/data/thidparty/euroc/native/MH_01_easy \
  --out <Q1_OUT> \
  --errors-csv \
  --gyro-observe
```

随后执行：

```bash
cmp /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control/est.tum <Q1_OUT>/est.tum
cmp /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control/kf.tum <Q1_OUT>/kf.tum
cmp /home/lin/Projects/data/phad-bench/MH_01_easy/ec74a84/m4_minimal_gyro_control/diag.csv <Q1_OUT>/diag.csv
sha256sum <Q1_OUT>/est.tum <Q1_OUT>/kf.tum <Q1_OUT>/diag.csv \
  <Q1_OUT>/gyro_packets.csv <Q1_OUT>/gyro_samples.csv
```

另以 JSON parser 断言 `meta.json.config_hash == "402d1925"`，将 control 与 Q1 的
`meta.json.config` 作为完整 JSON object 比较，并将 `config_canonical_text` 作为完整
string 逐字节比较；禁止用 grep 单独看 hash 或硬编码 key count 代替。离线
replay validator 必须在 dataset path 不可访问的条件下只读复制出的两 CSV +
`diag.csv` 并通过。

## 7. Pass / fail / inconclusive 与 Q2 stop/go

| 结论 | 判定 | 动作 |
|---|---|---|
| `pass` | 第 1 节五项全部满足；hard failure 为 0；所有 packet/sample join 与 hash 成立 | 记录 Q1 证据，允许**另起独立计划**进入 Q2 |
| `fail` | 任一 byte/SHA/canonical gate 不同，或发现时间、单位、顺序、finite、endpoint、Σdt 合同错误 | 保留负结果，停止；只修 Q1 最早失败，不实现 Q2+ |
| `inconclusive` | build/dataset/output/磁盘等外部条件使 MH_01 未完整到 EOS，或证据文件/支持范围不完整但未证明合同错误 | 记录缺口和复现方式，保持 Q1 pending；不得以 unit tests 代替真实 run 放行 |

`gap` 与 `empty_nonfirst` 是局部无资格，不改变 visual run；必须报告 count/duration。
它们不会自动把 Q1 伪装成成功：若仍能证明序列化与来源合同，Q1 可 pass，但后续机制只能消费
`valid` packet；若 valid support 不足，后续层必须给 `inconclusive`。

Q1 `pass` 只授权下一问题：**在 known-bias synthetic motion 上，gyro 积分是否回收解析
rotation？** 不授权真实序列 bias fit、factor 或 posterior。

## 8. Q2–Q5 未来依赖与边界（不属于本计划 todos）

### Q2 Predict

- 依赖：Q1 pass 与冻结的两个 artifact schema/hash。
- 唯一硬门：known-bias synthetic integration；覆盖静止、恒角速度、非零已知 bias、不同 dt、
  方向与 rad/s↔deg/s 反例，解析 `SO(3)` oracle 独立于待测 helper。
- 禁止：从 MH_01 fit bias、读取 ATE、建立 factor 或改变 posterior。

### Q3 Align

- 依赖：Q2 hard gate pass；另建独立 plan，在任何 factor-enabled 结果产生前冻结统计实现。
- 使用 researcher 的预注册协议：
  `Log(ΔR_imu(b)^T ΔR_vis)` residual；rank-revealing decomposition 的 rank=3 与
  relative numerical tolerance；`condition <= 1e6`；fit 前 100 s 并拆成两个时间半段；
  fit-half Mahalanobis 相容；suffix 上 adjacent 与非重叠 1 s paired comparison；至少 60 个
  validation 1 s blocks；1 s 分块 HAC covariance；对均值/轴向/时间/角速度结构的 22 项
  family 使用 Holm 校正。Q3 计划必须在编码前枚举 22 个假设、alpha、effect-size 门、
  minimum support 及 pass/fail/inconclusive，不能看到 Q4/Q5 结果后回改。
- 这是使用未来 suffix 的 offline oracle，不是 online initialization 资格。

### Q4 Constrain

- 依赖：Q3 pass；先过 synthetic fixed-graph residual/Jacobian/translation-invariance 硬门，
  再做 frozen-input 实验。
- active 分母只含 graph-state-compatible packets；要求 `attached/eligible=1.0` 且
  `attached>0`。另报 compatible/all-valid packet count 与 duration，纯描述、不设 0.95 门。
- 禁止为覆盖率 merge packets、跨 rejected frame 拼区间、创建全帧 state、改 KF schedule，
  或把 frozen-input 称为同图。

### Q5 Feedback

- 依赖：Q4 mechanism pass；才允许 gyro 在默认关闭的 natural closed-loop 实验中影响
  posterior。
- 与 Q0 比较 trajectory、schedule、PnP、cull/reopt、re-anchor 和首次 lifecycle 分叉；
  product guardrail 不能替代 Q2–Q4 mechanism evidence。
- 未过门保持默认关，不扫 covariance/Huber/frontend 阈值；Q6 default 不在本资格系列范围。

## 9. 收尾检查

```bash
git diff --check
git status --short
git diff -- docs
```

结果文档必须明确区分已执行与未执行命令、实际 exit code、artifact 绝对路径与 SHA256、
未执行的 full EuRoC/Sanitizer，以及剩余风险。未实际完成 MH_01 gate 时不得把 todo 标为
completed，也不得发出 Q2 go。
