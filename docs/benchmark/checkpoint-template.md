# <Milestone> <Checkpoint> Benchmark（`<config>`）

本文档描述当前约定，不是绝对约束，会随项目开发修订。

日期：<YYYY-MM-DD>

状态：<complete / partial / not_run>；<一句话判定>

相关：

- issue：<#n>
- predecessor：<checkpoint 文档>
- design / diagnosis：<文档链接>

## 1. 身份与执行

| 项 | 值 |
|---|---|
| commit | `<full>`（short `<short>`；`git_dirty=false`） |
| config | `<label>` / `<hash>` |
| predecessor | `<commit>/<config>` |
| dataset | <dataset identity> |
| bench root | `<path>` |
| artifact path | `<root>/<sequence>/<commit>/<label_hash>/` |
| execution | <serial/parallel、日期、工具> |
| gate | <硬门、软门、record-only 范围> |

实际命令：

```bash
<command>
```

## 2. 配置快照

相对 predecessor 的增量键：

| 键 | predecessor | 当前 | 说明 |
|---|---:|---:|---|
| `<key>` | `<old>` | `<new>` | `<reason>` |

完整 `config_canonical_text`：

```text
<canonical config>
```

CLI-only 参数：<无 / 明细>。

## 3. 全量质量表

| sequence | status | ATE (m) | RPE (m) | completion | coverage | segments | reanchors | ok / image | rejected | failed |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ... | ... | ... | ... | ... | ... | ... | ... | ... | ... | ... |

## 4. Robustness

| sequence | culled / unique | reopts | drops skipped | zombie drops / ids | PnP ok / fallback | cheirality | low connectivity |
|---|---:|---:|---:|---:|---:|---:|---:|
| ... | ... | ... | ... | ... | ... | ... | ... |

## 5. 相对 predecessor

| sequence | ATE Δ (m) | RPE Δ (m) | completion Δ | segments Δ | reanchors Δ | failed Δ | 判读 |
|---|---:|---:|---:|---:|---:|---:|---|
| ... | ... | ... | ... | ... | ... | ... | ... |

跨 coverage、segments 或匹配集合变化时，只陈述数字变化，不把 ATE 差异直接归因为
单一机制。

## 6. 判定与风险

- <门控结果>；
- <改善>；
- <回归>；
- <不可比较项>；
- <后续约束>。

## 7. 完整性核验

| 检查 | 结果 |
|---|---|
| command rc | <n/n> |
| artifacts | <meta/summary count> |
| code identity | <一致性> |
| config identity | <一致性> |
| canonical config | <一致性> |
| raw artifacts | `<path>` |
| 未执行项 | <无 / 原因> |

