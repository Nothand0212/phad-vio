---
type: "query"
date: "2026-08-25T14:30:27.893795+00:00"
question: "main 的文档中，最新的 EuRoC 数据集 benchmark 精度数据是多少？"
contributor: "graphify"
outcome: "useful"
source_nodes: ["M3.3 Checkpoint Lineage", "EuRoC 11-Sequence Suite", "Benchmark Checkpoint Policy"]
---

# Q: main 的文档中，最新的 EuRoC 数据集 benchmark 精度数据是多少？

## Answer

Expanded from original query via graph vocab: [euroc, benchmark, accuracy, ate, rpe, baseline]. main@ec74a84 尚无 M4 全量精度 checkpoint；最新默认行为是 pre-M4 round 2 回退并逐项等同 clean Slice 7 e77ee5d/default_402d1925。EuRoC 11/11 ATE 均值 1.027152273 m（文档四舍五入 1.027 m），RPE 均值 0.224063636 m；逐序列权威表位于 docs/benchmark/m3.3/slice-7_e77ee5d_402d1925.md:31。

## Outcome

- Signal: useful

## Source Nodes

- M3.3 Checkpoint Lineage
- EuRoC 11-Sequence Suite
- Benchmark Checkpoint Policy