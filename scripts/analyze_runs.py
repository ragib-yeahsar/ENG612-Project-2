#!/usr/bin/env python3
"""Compare wall-following runs from the controller's own CSV logs.

Usage:  python3 analyze_runs.py out_dir label1=file1.csv label2=file2.csv ...

Input columns: t,dist,err,v,w,front   (written by wall_follower_node.py)
Outputs: metrics.csv, distance_vs_time.png, error_hist.png in out_dir.
Corner turns (v = 0 and |w| = 0.8 rad/s, either wall) and the 3 s after them are reported
separately from the straights.
"""
import csv, math, os, sys, statistics
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:          # tables still work without plots
    plt = None

TARGET = 0.30
LIMIT = 0.05


def load(path):
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(path))]
    if len(rows) > 1 and rows[1]['t'] - rows[0]['t'] > 5:
        rows = rows[1:]      # simulator clock jump on the first row
    t0 = rows[0]['t']
    for r in rows:
        r['t'] -= t0
    return rows


def turns(rows):
    segs, start, prev = [], None, 0
    for i, r in enumerate(rows):
        if r['v'] == 0 and abs(abs(r['w']) - 0.8) < 1e-6:
            if start is None or i - prev > 5:
                if start is not None:
                    segs.append((rows[start]['t'], rows[prev]['t']))
                start = i
            prev = i
    if start is not None:
        segs.append((rows[start]['t'], rows[prev]['t']))
    return segs


def metrics(rows):
    tr = turns(rows)
    in_turn = lambda t, k=0: any(a - 0.3 <= t <= b + k for a, b in tr)
    moving = [r for r in rows if r['t'] > 3.5]
    non_turn = [r for r in moving if not in_turn(r['t'])]
    straights = [r for r in moving if not in_turn(r['t'], 3)]
    rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float('nan')
    e_all = [r['err'] for r in non_turn]
    e_str = [r['err'] for r in straights]
    exits_min, exits_max = [], []
    for a, b in tr:
        w = [r['dist'] for r in rows if b <= r['t'] <= b + 10]
        if w:
            exits_min.append(min(w)); exits_max.append(max(w))
    starts = [a for a, b in tr]
    gaps = [y - x for x, y in zip(starts, starts[1:])]
    lap = round(4 * sum(gaps) / len(gaps), 0) if gaps else ''
    return {
        'duration_s': round(rows[-1]['t'], 1),
        'corners': len(tr),
        'lap_s': lap,
        'straight_rms_mm': round(rms(e_str) * 1000, 1),
        'straight_p95_mm': round(sorted(abs(x) for x in e_str)[int(.95 * len(e_str))] * 1000, 1),
        'nonturn_rms_mm': round(rms(e_all) * 1000, 1),
        'nonturn_max_cm': round(max(abs(x) for x in e_all) * 100, 1),
        'within_5cm_pct': round(100 * sum(abs(x) <= LIMIT for x in e_all) / len(e_all), 1),
        'exit_min_dist_m': round(min(exits_min), 3) if exits_min else '',
        'exit_max_dist_m': round(max(exits_max), 3) if exits_max else '',
        'steer_std': round(statistics.pstdev([r['w'] for r in straights]), 3),
    }, tr


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    runs = {}
    for a in sys.argv[2:]:
        label, path = a.split('=', 1)
        runs[label] = load(path)
    table = {}
    if plt is None:
        for label, rows in runs.items():
            table[label] = metrics(rows)[0]
        write_table(out, table)
        return
    fig, axes = plt.subplots(len(runs), 1, figsize=(10, 2.4 * len(runs)), sharex=False)
    if len(runs) == 1:
        axes = [axes]
    for ax, (label, rows) in zip(axes, runs.items()):
        m, tr = metrics(rows)
        table[label] = m
        ax.axhspan(TARGET - LIMIT, TARGET + LIMIT, color='#cfe8cf', label='±5 cm band')
        ax.axhline(TARGET, color='k', lw=0.8, ls='--', label='target 0.30 m')
        for a, b in tr:
            ax.axvspan(a, b, color='#e6e6e6')
        ax.plot([r['t'] for r in rows], [r['dist'] for r in rows], lw=1.0, color='#1f5fa8')
        ax.set_ylim(0.15, 0.45)
        ax.set_ylabel('distance (m)')
        ax.set_title(label, fontsize=10, loc='left')
    axes[-1].set_xlabel('time (s)  (grey = corner turns)')
    axes[0].legend(loc='upper right', fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'distance_vs_time.png'), dpi=150)

    fig2, ax2 = plt.subplots(figsize=(7, 3.5))
    for label, rows in runs.items():
        _, tr = metrics(rows)
        e = [r['err'] * 100 for r in rows if r['t'] > 3.5 and not any(a - 0.3 <= r['t'] <= b + 3 for a, b in tr)]
        ax2.hist(e, bins=60, range=(-8, 8), histtype='step', density=True, label=label)
    ax2.axvline(-5, color='r', ls=':'); ax2.axvline(5, color='r', ls=':')
    ax2.set_xlabel('distance error on straights (cm)'); ax2.set_ylabel('density')
    ax2.legend(fontsize=8)
    fig2.tight_layout()
    fig2.savefig(os.path.join(out, 'error_hist.png'), dpi=150)

    write_table(out, table)


def write_table(out, table):
    cols = list(next(iter(table.values())).keys())
    with open(os.path.join(out, 'metrics.csv'), 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['run'] + cols)
        for label, m in table.items():
            w.writerow([label] + [m[c] for c in cols])
    print('run,' + ','.join(cols))
    for label, m in table.items():
        print(label + ',' + ','.join(str(m[c]) for c in cols))


if __name__ == '__main__':
    main()
