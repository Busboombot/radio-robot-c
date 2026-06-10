---
id: '007'
title: 'Notebook cell 7: comparison plot and end-to-end execution verification'
status: done
use-cases:
- SUC-004
- SUC-005
depends-on:
- '006'
github-issue: ''
issue: ''
completes_issue: true
---

# Notebook cell 7: comparison plot and end-to-end execution verification

## Description

Add Cell 7 (the comparison plot) to `demo_figure_eight.ipynb` and verify that the
notebook executes end-to-end without error. Cell 7 combines the trajectory logs from
all three experiments and the reference path into a single comparison figure, and
computes the RMS cross-track error for each regime.

This is the final notebook ticket. Passing the end-to-end `nbconvert --execute` check
is the primary acceptance gate.

## Acceptance Criteria

### Cell 7 — Comparison plot

- [x] Cell assembles all four trajectory arrays:
  - `ref_path` — the catmull-rom figure-eight reference path (from Cell 2).
  - `exp1_est` — dead-reckoning estimate log from Cell 4.
  - `exp2_est` — OTOS+camera estimate log from Cell 5.
  - `exp3_est` — EKF estimate log from Cell 6.
  Each log is a list of `(x_mm, y_mm)` tuples or equivalent numpy arrays.
- [x] Figure has one subplot showing all four curves:
  - Reference path in grey (dashed).
  - Experiment 1 (dead reckoning) in blue.
  - Experiment 2 (OTOS + camera) in orange.
  - Experiment 3 (EKF) in green.
  - Legend labels each curve with its experiment name.
- [x] Cross-track RMS error is computed for each experiment:
  - For each estimated point, find the nearest point on the reference path.
  - Compute signed cross-track distance (perpendicular to path tangent).
  - RMS over all points = `sqrt(mean(cross_track**2))`.
- [x] A table (printed to stdout, or as a text cell below the plot) shows:
  ```
  Exp 1 (Dead Reckoning):  RMS XTE = XX.X mm
  Exp 2 (OTOS + Camera):   RMS XTE = XX.X mm
  Exp 3 (EKF):             RMS XTE = XX.X mm
  ```
- [x] An assertion confirms `rms_exp3 < rms_exp1` (EKF better than dead reckoning).
  If this assertion fails, the programmer should re-tune EKF noise parameters in
  ticket 006 until it passes.
- [x] Cell completes without error.

### End-to-end execution

- [x] `jupyter nbconvert --to notebook --execute host_tests/demo_figure_eight.ipynb
  --output host_tests/demo_figure_eight_executed.ipynb` runs to completion with
  exit code 0.
- [x] The executed notebook has no error outputs in any cell.
- [x] `uv run --with pytest python -m pytest` still passes after all notebook
  additions (verifies no regressions in the pytest suite from the C++ changes in
  tickets 001-004).

## Implementation Plan

### Cross-track error helper

```python
def cross_track_rms(traj_xy, ref_pts):
    """RMS cross-track error of traj_xy against ref_pts (Nx2 arrays)."""
    from scipy.spatial import KDTree
    tree = KDTree(ref_pts)
    dists, idxs = tree.query(traj_xy)
    # signed cross-track: use perpendicular to local tangent
    errs = []
    for i, (pt, idx) in enumerate(zip(traj_xy, idxs)):
        i1 = min(idx + 1, len(ref_pts) - 1)
        i0 = max(idx - 1, 0)
        tangent = ref_pts[i1] - ref_pts[i0]
        tangent /= (np.linalg.norm(tangent) + 1e-9)
        normal = np.array([-tangent[1], tangent[0]])
        errs.append(float(np.dot(np.array(pt) - ref_pts[idx], normal)))
    return float(np.sqrt(np.mean(np.array(errs)**2)))
```

`scipy` is available in the project venv (confirm with `uv run python -c "import scipy"`
before using; if not available, fall back to `numpy` nearest-point without signed error).

### Trajectory array format

Each experiment cell should accumulate its estimate log as a Python list of
`[x_mm, y_mm]` pairs into variables named `exp1_xy`, `exp2_xy`, `exp3_xy`. Cell 7
expects these variables to be in scope. Make sure they are defined at module scope in
their respective cells (not inside a function).

### Files to modify

- `host_tests/demo_figure_eight.ipynb` — add Cell 7.

### Testing plan

- Primary test: `jupyter nbconvert --to notebook --execute
  host_tests/demo_figure_eight.ipynb` exits 0.
- Regression test: `uv run --with pytest python -m pytest` passes.
- Manual check: open the executed notebook and verify that Cell 7 plot renders all
  four curves and the RMS table shows Exp3 < Exp1.

### Documentation updates

Add a markdown summary cell after Cell 7 (analogous to the Summary cell in
`demo_square.ipynb`) with a table summarising the three experiments and their
RMS cross-track error results. This serves as the notebook's concluding documentation.
