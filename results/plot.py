#!/usr/bin/env python3
"""Make a success-rate bar chart from results/trials.csv -> results/success_rate.png.

Run:  python3 results/plot.py
"""

import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(__file__)
POSITIONS = ['top-left', 'top-middle', 'top-right',
             'bottom-left', 'bottom-middle', 'bottom-right']


def load():
    return list(csv.DictReader(open(os.path.join(HERE, 'trials.csv'))))


def rate(rows):
    return 100.0 * sum(1 for r in rows if r['result'] == 'success') / len(rows)


def main():
    rows = load()

    # Success rate by shelf position, split by arm.
    by = defaultdict(list)
    for r in rows:
        by[(r['arm'], r['state_position'])].append(r)
    right = [rate(by[('right', p)]) for p in POSITIONS]
    left = [rate(by[('left', p)]) for p in POSITIONS]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5),
                                   gridspec_kw={'width_ratios': [3, 1]})

    x = range(len(POSITIONS))
    w = 0.38
    ax1.bar([i - w / 2 for i in x], right, w, label='Right arm (alcohols)',
            color='#6b3a2d')
    ax1.bar([i + w / 2 for i in x], left, w, label='Left arm (mixers)',
            color='#3a6b3a')
    ax1.set_xticks(list(x))
    ax1.set_xticklabels(POSITIONS, rotation=30, ha='right')
    ax1.set_ylabel('Success rate (%)')
    ax1.set_ylim(0, 100)
    ax1.set_title('Success rate by shelf position')
    ax1.axhline(rate(rows), color='grey', ls='--', lw=1,
                label=f'Overall {rate(rows):.1f}%')
    ax1.legend()
    for i, v in enumerate(right):
        ax1.text(i - w / 2, v + 1, f'{v:.0f}', ha='center', fontsize=8)
    for i, v in enumerate(left):
        ax1.text(i + w / 2, v + 1, f'{v:.0f}', ha='center', fontsize=8)

    # Top vs bottom shelf (the headline finding).
    top = rate([r for r in rows if r['state_position'].startswith('top')])
    bottom = rate([r for r in rows if r['state_position'].startswith('bottom')])
    ax2.bar(['Top\nshelf', 'Bottom\nshelf'], [top, bottom],
            color=['#2d4f6b', '#8a5a2d'])
    ax2.set_ylim(0, 100)
    ax2.set_ylabel('Success rate (%)')
    ax2.set_title('Top vs bottom shelf')
    for i, v in enumerate([top, bottom]):
        ax2.text(i, v + 1, f'{v:.1f}%', ha='center')

    fig.suptitle('Bimanual Bartender — success rate across conditions '
                 f'(n = {len(rows)} trials)', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = os.path.join(HERE, 'success_rate.png')
    fig.savefig(out, dpi=130)
    print(f'Wrote {out}')


if __name__ == '__main__':
    main()
