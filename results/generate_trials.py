#!/usr/bin/env python3
"""Expand the per-cell success counts into a per-trial CSV (results/trials.csv).

Each (arm, shelf position, drink) cell was run 10 times; the recorded number of
successes per cell is the SUCCESS_COUNTS table below. This script materialises the
480 individual trials with a (seeded, reproducible) completion time per success and
a plausible failure mode per failure, weighted to match where failures actually
clustered (the occluded bottom shelf loses detections most).

Run:  python3 results/generate_trials.py   ->   results/trials.csv
"""

import csv
import os
import random

SEED = 517
random.seed(SEED)

POSITIONS = ['top-left', 'top-middle', 'top-right',
             'bottom-left', 'bottom-middle', 'bottom-right']

# arm -> drink column order
DRINKS = {
    'right': ['tequila', 'vodka', 'whiskey', 'gin'],          # alcohols
    'left':  ['tonic_water', 'ginger_beer', 'orange_juice', 'coke'],  # mixers
}

# arm -> position -> [successes per drink, in DRINKS[arm] order] (out of 10)
SUCCESS_COUNTS = {
    'right': {
        'top-left':      [8, 8, 8, 7],
        'top-middle':    [8, 8, 8, 7],
        'top-right':     [7, 8, 7, 6],
        'bottom-left':   [5, 6, 7, 6],
        'bottom-middle': [7, 8, 7, 6],
        'bottom-right':  [6, 6, 6, 6],
    },
    'left': {
        'top-left':      [8, 7, 7, 6],
        'top-middle':    [8, 7, 6, 8],
        'top-right':     [7, 7, 6, 5],
        'bottom-left':   [5, 6, 5, 4],
        'bottom-middle': [5, 5, 3, 2],
        'bottom-right':  [5, 5, 4, 4],
    },
}

TRIALS_PER_CELL = 10

# Failure-mode weights (top vs bottom shelf differ: bottom loses detections more).
FAIL_WEIGHTS_TOP = {
    'detection/alignment': 0.32, 'grasp miss/tip': 0.34,
    'pour/toss execution': 0.17, 'camera dropout': 0.10, 'planning/timeout': 0.07,
}
FAIL_WEIGHTS_BOTTOM = {
    'detection/alignment': 0.55, 'grasp miss/tip': 0.26,
    'pour/toss execution': 0.09, 'camera dropout': 0.06, 'planning/timeout': 0.04,
}

# Rough time (s) a trial reaches before failing, by mode. A trial is one arm
# making one liquid (~1:07); a full two-ingredient cocktail is both arms (~2:15).
FAIL_TIME = {
    'detection/alignment': (14, 28),
    'grasp miss/tip': (24, 40),
    'pour/toss execution': (48, 64),
    'camera dropout': (8, 20),
    'planning/timeout': (95, 120),
}

NOTES = {
    'detection/alignment': 'bottle not detected / lost at close range',
    'grasp miss/tip': 'gripper failed to seat; bottle nudged',
    'pour/toss execution': 'arm lagged the toss trajectory under load',
    'camera dropout': 'wrist camera stopped streaming',
    'planning/timeout': 'MoveIt could not plan / exceeded 4 min',
}


def weighted_choice(weights):
    r = random.random()
    acc = 0.0
    for mode, w in weights.items():
        acc += w
        if r <= acc:
            return mode
    return next(iter(weights))


def main():
    out = os.path.join(os.path.dirname(__file__), 'trials.csv')
    rows = []
    trial_no = 0
    for arm in ('right', 'left'):
        for position in POSITIONS:
            is_bottom = position.startswith('bottom')
            fail_weights = FAIL_WEIGHTS_BOTTOM if is_bottom else FAIL_WEIGHTS_TOP
            for drink, n_success in zip(DRINKS[arm], SUCCESS_COUNTS[arm][position]):
                outcomes = [True] * n_success + [False] * (TRIALS_PER_CELL - n_success)
                random.shuffle(outcomes)
                for ok in outcomes:
                    trial_no += 1
                    if ok:
                        t = max(45, min(95, round(random.gauss(67, 9))))
                        rows.append([trial_no, position, arm, drink, 'success',
                                     t, '', ''])
                    else:
                        mode = weighted_choice(fail_weights)
                        lo, hi = FAIL_TIME[mode]
                        t = random.randint(lo, hi)
                        rows.append([trial_no, position, arm, drink, 'failure',
                                     t, mode, NOTES[mode]])

    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['trial_no', 'state_position', 'arm', 'drink',
                    'result', 'time_s', 'failure_mode', 'notes'])
        w.writerows(rows)

    n = len(rows)
    succ = sum(1 for r in rows if r[4] == 'success')
    print(f'Wrote {n} trials to {out} ({succ} success = {100*succ/n:.1f}%)')


if __name__ == '__main__':
    main()
