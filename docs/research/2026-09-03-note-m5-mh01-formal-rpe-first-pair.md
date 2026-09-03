# M5 MH_01 formal RPE 首对分叉

- Issue：[#51](https://github.com/Nothand0212/phad-vio/issues/51)
- 状态：43-frame baseline已资格化；selector warm-up单变量诊断完成并停止于prefix review
- 范围：default-off initial-moving path；不使用PnP retry候选证据作formal抵消
- Reference control artifact：`/tmp/phad-m5-mh01-control.cUe1VW/artifacts/run-1`
- Reference candidate artifact：`/tmp/phad-m5-mh01-candidate-gravity-align`

## 1. 首个数值分叉

Control与candidate的3618个1秒RPE pair keys完全相同，`candidate-only=0`、
`control-only=0`。首个pair为：

```text
(1403636580863555584, 1403636581863555584)
```

它对应MH_01 processed image / diag `i=22→42`：

| metric | control | candidate | candidate − control |
|---|---:|---:|---:|
| translation RPE | `0.013573365482 m` | `0.015256666938 m` | `+0.001683301457 m` (`+1.683301457 mm`) |
| rotation RPE | `0.106736646048°` | `0.054029041578°` | `-0.052707604470°` |

定位为control `est.tum:22,42`、candidate `est.tum:17,37`、两侧
`diag.csv:24,44`，GT为`state_groundtruth_estimate0/data.csv:7,207`。RPE计算继续
使用[`computeRpe()`](../../phad/eval/rpe.cpp)的既有1秒partner与relative-pose合同。

这个pair的21帧candidate PnP全部为primary `accepted`；首个post-root nonaccepted
PnP在`i=278`。因此首对分叉属于default-off accepted path，与room holdout retry
归因链正交。

## 2. Replay qualification

Reference control由clean
`ba607c5738b3c5d9bc9cb0423199eee67648b0b3`与`config_hash=0337287b`生成；
input manifest文件SHA-256为
`edd7053e1152e1cd167cec3564e694696d7d5f25fddb0f4f90b1453a310992db`。
Reference candidate只记录
`git_commit=643fd9095e6bc753868814e3d9393d9e549a5066`、`git_dirty=true`与
`config_hash=0337287b`，没有可独立重建该dirty tree的source archive。因此它提供
数值与output oracle，但不能单独作为下一次实验的source identity。

Reference candidate截至`i=42`的prefix oracle固定为：

| artifact prefix | rows | SHA-256 |
|---|---:|---|
| `diag.csv` | header + 43 | `dde502ec23ef8401d30024aa02a14e1854e5c05d1db054d9a3d82bd307696f7c` |
| `est.tum` | 37 | `1f032910d0e2c4464c30d861d2d18d73439a904bbead148c09627c24d35b16b6` |
| `kf.tum` | 9 | `a5f9c6a423d6fc854cebe98a57d773faa1edbb7fee9fd15199dd941b13adcb7c` |

43-frame diagnostic启动前必须创建新的content-addressed source snapshot，并冻结
source manifest/archive、built binary、canonical config、dataset/input manifest、
compiler/libraries与evaluator SHA。使用同一个带test-only intervention seam的binary
先运行baseline arm；只有上述三个prefix SHA逐字复现时，才冻结该arm新生成的
versioned canonical state encoding，并在同一identity下运行intervention。复现失败时
结果为`INVALID`，不得把reference artifact与新run拼接为一个实验。

## 3. 最早可见前驱

`OfflineVoSession`在调用estimator前决定keyframe，只有accepted update之后才更新
external keyframe selector。Selector在`m_total_keyframes < 2`时无条件选择keyframe，
而initializing packet不推进该状态：

```text
control:   i1 root → i2 second accepted → normal selector logic
candidate: i6 moving root → selector still counts one → i7 forced second keyframe
```

最早差异依次为：`i=7` keyframe flag、`i=8` cull count、`i=9` observation count。
Moving candidate staging与landmark seed使用该keyframe flag，window eviction又优先移除
non-keyframe，因此selector warm-up状态是首个common pair前最早、可独立干预的因果边。

这是一项待证伪假设：现有artifact没有control root的完整`X/V/B`与逐因子向量，不能
据此排除root state对后续误差的剩余贡献。

## 4. 43-frame执行结果

本次执行使用content-addressed source
`sha256:e3d83d41e616b1239cac1e28a898c8e692db0a2dc1d7164a4c25f40973078011`；
`phad_vo_bench` SHA-256为
`09d606d3c11c629725d023053820170c0fd249165b5148c017968e7356cbc5e1`，
RPE evaluator source `rpe.cpp` SHA-256为
`e130cb0b435fd11464698aec1eeeb08565767ad1a1dc1f2930c62416e6a2050c`；
固定pair复算所用`phad_traj_eval` binary SHA-256为
`005098a8f6972fcced2ed1081b238ef95740d2295743452f90f5ff23706c3463`。
Canonical config为`0337287b`，MH_01 ordered input manifest仍为
`edd7053e1152e1cd167cec3564e694696d7d5f25fddb0f4f90b1453a310992db`。

Baseline逐字复现§2的三个prefix SHA，43帧中`i=0..5`为initializing、
`i=6..42`为ok，first-ok为`i=6`。两臂在hook前的完整canonical state SHA-256均为
`21b4a7859f8cbd4aebff717386e25e3553a27d4d2e917df2faf2e23ae6b53f3f`；
baseline hook diff为空，intervention hook diff唯一为：

```text
/keyframe_selector/total_keyframes: 1 -> 2
```

`i=0..6`诊断输出逐字相等；`i=7`的keyframe flag从baseline的`true`变为
intervention的`false`，证明seam已生效。两臂hard lifecycle相同：43帧、37个ok、
0 rejected、0 failed、1 segment、0 reanchor。`i=42`的baseline/intervention完整状态
SHA-256分别为`14003c553f3442d4188ea3b038f3d93922e7fc318991523b2d1bb78be5817d0d`
与`8f9c590a486e6bc7f64c22ff5eb2c6e4c758934789e19b5c9971e6c542046d0d`。

固定pair使用同一evaluator独立复算，结果为：

| arm | translation RPE | rotation RPE |
|---|---:|---:|
| control | `0.013573365482 m` | `0.106736646048°` |
| qualified baseline | `0.015256666938 m` | `0.054029041609°` |
| selector warm-up | `0.016059173513 m` | `0.059629471507°` |

Selector warm-up相对baseline增加`0.000802506575 m`，相对control仍高
`0.002485808031 m`。Rotation按合同只记录，不参与本次裁决。

## 5. 判定与停止

`m_total_keyframes: 1→2`足以改变`i=7`的keyframe选择，但不足以解释首个common
RPE pair的translation excess；本次充分性假设被证伪。按停止合同，本轮停在prefix
review，#51 formal gate保持RED。后续若继续，应在独立切片中评审root `X/V/B`或
`i=8` cull feedback这一条因果边。

正式审计入口为
`/tmp/phad-vio-m5-51-replay-gsNT3m/receipts/formal-diagnostic-evidence-closure.json`，
SHA-256为
`272d6cae3d9f62f59b8f312149b84638c54c401bf1e9a63afb8dad731baaadcc`。
该closure封存42个文件与11个目录，并重验source transport、7376个ordered input、
147个runtime libraries、canonical state digests和cross-receipt hashes。
