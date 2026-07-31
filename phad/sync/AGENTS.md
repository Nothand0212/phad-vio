# `phad::sync` — agent 提示

模块合同见同目录 `README.md`。

## 约定

- 左右 / 多源配对只在本库完成；**不**在 `phad::io` dataset adapter 内再写第二套策略
- 只依赖 `phad::sensor` / `phad::common`，不依赖 `phad::io`
- M3.2 StereoOnly：`StereoPairSynchronizer` 为 B 族双队列，默认 `tol_ns = 0`（exact）；soft 仅显式配置
- `pushImage` 只表达校验结果；丢弃 / 溢出进计数器；配对经 `tryPop()`；不提供回调
- 库本身不打日志；首次 drop / overflow warning 由 apps（`StereoPairStream` / session）写入
- M4 在同一模块扩展 `pushImu` 与 `StereoImuPacket`，不新建 synchronizer
