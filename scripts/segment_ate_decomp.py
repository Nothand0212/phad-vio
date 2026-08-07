#!/usr/bin/env python3
"""V1_03 逐段 ATE 分解 — 量化 re-anchor 锚偏移贡献。

用法:
  python3 scripts/segment_ate_decomp.py <run-dir> <euroc-root> [--phad_traj_eval path]

run-dir 需含 diag.csv + est.tum(phad_vo_bench 产物)。
每 segment 独立 Umeyama 对齐 GT 计算 ATE(复用 phad_traj_eval,口径与
summary.json 一致);输出: 每段 ATE + 全局对照 + 段间偏移贡献估计。

关键诊断意义:
  - 若某段独立 ATE 大 → 该段轨迹本身偏(锚误差 / 局部退化)
  - 若全局 ATE 远大于各段独立 ATE 的加权 → 段间相对错位(锚偏移)
    贡献主体(每段各自对齐后互相抵消)。
"""
import argparse
import csv
import re
import subprocess
import sys
import tempfile
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run_dir", type=Path, help="phad_vo_bench 产物目录(diag.csv+est.tum)")
    p.add_argument("euroc_root", type=Path, help="EuRoC 序列根(如 .../V1_03_difficult)")
    p.add_argument("--phad_traj_eval", type=Path, default=Path("build/phad_traj_eval"),
                   help="phad_traj_eval 可执行路径")
    return p.parse_args()


def load_diag_segments(run_dir: Path):
    """diag.csv → {timestamp_ns: segment_id}"""
    mapping = {}
    with open(run_dir / "diag.csv") as f:
        for row in csv.DictReader(f):
            mapping[int(row["timestamp_ns"])] = int(row["segment_id"])
    return mapping


def load_est_tum(run_dir: Path):
    """est.tum → [(ts_ns, line), ...],按 segment_id 分组"""
    seg_map = load_diag_segments(run_dir)
    segs = {}
    for line in open(run_dir / "est.tum"):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        ts_s = float(line.split()[0])
        ts_ns = int(round(ts_s * 1e9))
        seg = seg_map.get(ts_ns)
        if seg is None:
            continue
        segs.setdefault(seg, []).append(line)
    return segs


def run_ate(eval_bin: Path, est_tum: Path, euroc_root: Path):
    """返回 (ate_rmse, matched) 或 None(对齐失败/无匹配)"""
    proc = subprocess.run(
        [str(eval_bin), "--est", str(est_tum), "--gt-euroc", str(euroc_root)],
        capture_output=True, text=True, timeout=120)
    m = re.search(r"ATE translation \[m\]\s+rmse ([\d.]+)", proc.stdout)
    if not m:
        return None
    return float(m.group(1))


def main():
    args = parse_args()
    segs = load_est_tum(args.run_dir)
    if not segs:
        print("错误: est.tum 与 diag.csv 时间戳无交集", file=sys.stderr)
        return 1

    # 全局 ATE(应与 summary.json 一致,验证口径)
    global_ate = run_ate(args.phad_traj_eval, args.run_dir / "est.tum", args.euroc_root)

    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for seg in sorted(segs):
            lines = segs[seg]
            seg_file = Path(tmp) / f"seg{seg}.tum"
            seg_file.write_text("\n".join(lines) + "\n")
            ate = run_ate(args.phad_traj_eval, seg_file, args.euroc_root)
            results.append((seg, len(lines), ate))

    total = sum(n for _, n, _ in results)
    print(f"run_dir : {args.run_dir}")
    print(f"全局 ATE(对齐全轨迹): {global_ate if global_ate else 'N/A'}")
    print(f"segment 独立 ATE(每段各自对齐):")
    print(f"  {'seg':>3}  {'frames':>6}  {'ATE(m)':>8}  {'帧占比':>7}  {'加权ATE²':>9}")
    print(f"  {'---':>3}  {'------':>6}  {'-------':>8}  {'-----':>7}  {'-------':>9}")
    wsum = 0.0
    for seg, n, ate in results:
        frac = n / total
        wsum += frac * (ate * ate if ate else 0.0)
        print(f"  {seg:>3}  {n:>6}  {ate if ate else float('nan'):>8.3f}  {frac:>6.1%}  "
              f"{(ate * ate if ate else float('nan')):>9.3f}")

    print(f"\n参考: 各段独立 ATE 的加权 RMS = {wsum ** 0.5:.3f} m")
    if global_ate:
        print(f"全局 ATE − 加权段 RMS = {global_ate - wsum ** 0.5:+.3f} m "
              f"(>0 → 段间相对错位/锚偏移贡献)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
