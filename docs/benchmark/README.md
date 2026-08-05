# Benchmark 记录约定

本文档描述当前约定，不是绝对约束，会随项目开发修订。

`docs/benchmark/` 是关键行为 checkpoint 的 benchmark 权威入口。它保存可审查、
可复现、可比较的结果摘要；算法调研、设计和失败机理分析仍放在
`docs/research/`，实施步骤仍放在 `docs/plans/`。

Issue：[#26](https://github.com/Nothand0212/phad-vio/issues/26)。

## 1. 什么必须形成 checkpoint

| 修改类型 | 要求 |
|---|---|
| 算法行为、默认配置、生命周期、同步或评估合同变化 | 定向门控后，在 clean commit 上跑当前 milestone 的全量数据集 |
| 有独立名称、commit 和后续比较价值的失败方案 | 稳定可运行时也保存全量结果，不因判定失败而删除历史 |
| 已崩溃、产物无效或目标序列无法完成 | 可以停止；checkpoint 写 `not_run`、停止位置、原因与已有证据 |
| 参数扫描、临时 probe、一次性诊断臂 | 只跑诊断序列；只有入选正式候选后才升级为全量 checkpoint |
| 纯文档、纯测试或已证明行为不变的重构 | 不要求全量 benchmark |

“关键修改”以外部可观察行为为准，不等同于每个 Git commit。多个机械或测试提交
可以共同指向一个算法 checkpoint；两个 config hash 相同但代码行为不同的提交必须
是两个 checkpoint。

## 2. 执行顺序

1. dirty 工作区完成 TDD、最小编译和诊断序列；
2. 通过当前 slice 定义的硬门/软门，或明确得到稳定的失败 checkpoint；
3. 提交算法代码，得到不可变 commit；
4. 在该 **clean commit** 上串行运行全量数据集；
5. 从 `meta.json` / `summary.json` 生成 checkpoint 文档；
6. 单独提交 benchmark 文档。

这样算法提交可以作为 clean run identity，结果文档也能引用已经存在的 commit，避免
用 `<base>_dirty` 冒充正式身份。禁止为了得到 clean 标记而隐藏、撤销或覆盖用户
修改；需要复现历史 commit 时使用不影响主工作区的独立临时 clone。

## 3. 存储边界

Git 内保存：

- commit/config/dataset/命令身份；
- `meta.json.config_canonical_text` 完整快照；
- 全量质量表与 robustness 表；
- 相对直接 predecessor 的逐列差异；
- gate 判定、失败状态、未运行原因；
- raw artifact root 与完整性核验。

Git 外保存：

- `est.tum`、`gt.tum`、`diag.csv`、逐帧 probe、图片与其它大体积产物；
- 每次 run 原始 `meta.json` / `summary.json`。

原始产物默认使用项目数据盘下的独立 bench root。文档中的绝对路径只描述本次本机
证据位置；可复现合同由 dataset 名、commit、config snapshot 和命令共同组成。

## 4. 目录与命名

```text
docs/benchmark/
├── README.md
└── <milestone>/
    ├── README.md
    └── <checkpoint>_<commit7>_<config8>.md
```

示例：

```text
docs/benchmark/m3.3/slice-4f_c446ac5_a5e90dc7.md
```

checkpoint 文档是追加式历史记录。算法后续被回退或取代时更新 milestone 索引中的
状态和 successor，不改写原始实测数字。

## 5. 每份 checkpoint 的必填字段

### 身份

- full commit SHA、short SHA、clean/dirty；
- config label/hash 与 predecessor；
- 数据集根、序列清单、run 日期、bench 工具；
- raw artifact root/path template；
- 实际命令和所有 CLI override。

### 参数

- 完整 `config_canonical_text`，不能只写 hash；
- 与 predecessor 的增量键；
- CLI-only、不会进入 config hash 的探针必须单独列出。

### 结果

- status、ATE、RPE、completion、coverage；
- segments、reanchors、ok/image、rejected、failed；
- 与算法相关的 robustness 计数；
- predecessor 的绝对差异；跨 coverage 时明确禁止直接归因。

### 完整性

- 全量命令 rc、`meta.json`/`summary.json` 数量；
- 代码/config identity 一致性；
- canonical config 一致性；
- 未运行、失败或无法比较的字段及原因。

## 6. 当前全量数据集

M3.3 使用 EuRoC 11 条序列，固定顺序：

```text
MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult
V1_01_easy V1_02_medium V1_03_difficult
V2_01_easy V2_02_medium V2_03_difficult
```

所有序列数据只作 record-only，除非 slice 设计另行声明门限。全量表不能替代前置
诊断门，也不能用低 completion 的 ATE 单独证明改进。

