# results/ -- index of saved verification output

Everything in this folder is captured, verbatim, output from actually
running this project's code (tests, experiments, and the
`pid_controller.py` stability check). Nothing here was hand-edited or
re-derived; it exists so the numbers quoted in `VERIFICATION_LOG.md` and
the eventual report have a durable, re-checkable source, instead of only
living in the plot images in `plots/` or in a terminal scrollback that's
already gone.

Two kinds of file live here:
- **Raw text logs** (`pytest_output.txt`, `stability_check.txt`,
  `test_results.txt`): kept as plain console output, not reformatted,
  because they're read top-to-bottom as a log, not queried by field.
- **Structured JSON** (`exp4_tau_sweep_metrics.json`,
  `fuzzy_vs_pid_metrics.json`, `other_metrics.json`): the numeric output
  that used to live in seven separate `expN_summary.txt` files has been
  reformatted into three JSON files, grouped by what the numbers are
  used to compare, not by which script produced them. The original
  `expN_summary.txt` files have been deleted now that their numeric
  content lives in these JSON files; nothing in them is lost -- every
  number below traces back to a specific print statement in the
  corresponding `experiments/expN_*.py` script.

All files were generated with the `Platoon/` virtual environment active.

## Raw text logs

| File | Command that produced it | What it contains | Backs up (paper figure/table/eq, or VERIFICATION_LOG.md section) |
|---|---|---|---|
| `pytest_output.txt` | `pytest tests/ -v` | Full verbose output of all 26 unit tests, one line per test, plus the warnings summary. All 26 pass. This is the **current, authoritative** test-run record. | Confirms every module (spacing, vehicle model, PID, fuzzy layer, delay, metrics, platoon chaining) matches its cited equation; in particular `test_paper_baseline_fails_its_own_stability_condition` and `test_paper_baseline_error_grows_over_time` are the automated proof behind VERIFICATION_LOG.md Section 2 (Eq 13 finding). |
| `test_results.txt` | `pytest tests/ -v` (earlier run, migrated from `tests/test_results.txt`) | An earlier snapshot of the same verbose test run, kept for continuity with earlier verification passes. Superseded by `pytest_output.txt` above. | Same coverage as `pytest_output.txt`. |
| `stability_check.txt` | `python pid_controller.py` | The `check_stability()` demo in `pid_controller.py`'s `__main__` block, run on the paper's own baseline (kp=2, ki=3, kd=0.5, h=1.5). Shows the two Eq 13 conditions it fails and why. | Equation 13 (Routh-Hurwitz stability margin) vs. Equation 22 (baseline parameters). VERIFICATION_LOG.md Section 2. |

## Structured JSON

### `exp4_tau_sweep_metrics.json`
Source: `python experiments/exp4_tau_sweep.py`.
A flat list of 20 objects (10 tau values x 2 controllers, "long" format).
Each object:
```
{"tau": <float>, "controller": "PID" | "Fuzzy-PID", "Ep": <float>, "Mp": <float>}
```
Note: the script's printed table only reports `Ep` and `Mp` per tau, not
`sigma_p` -- so `sigma_p` is not included here rather than being
fabricated as null. If `sigma_p` is needed per-tau, re-run
`all_metrics()` from `metrics.py` directly, which does compute it.
**Backs up:** Figure 7 (vehicle platoon performance vs. tau / delay
coefficient) and, via the sharp jump in Ep/Mp from tau=0.5 upward, the
Eq13 instability-boundary finding in VERIFICATION_LOG.md Section 2.

### `fuzzy_vs_pid_metrics.json`
Source: `exp3_fuzzy_vs_pid.py`, `exp5_constant_delay.py`,
`exp6_random_delay.py`, combined into one file since all three answer
the same question (does Fuzzy-PID show a tighter error band than plain
PID?) under three different delay conditions.
A list of 3 objects, one per scenario:
```
{
  "scenario": "no_delay" | "constant_delay_0.5s" | "random_delay",
  "source_experiment": "<script name>",
  "PID": {"spacing_error_min_m": <float>, "spacing_error_max_m": <float>},
  "Fuzzy-PID": {"spacing_error_min_m": <float>, "spacing_error_max_m": <float>},
  "paper_reference": {"PID": {...}, "Fuzzy-PID": {...}} | null
}
```
`paper_reference` is `null` for `random_delay` because
`exp6_random_delay.py` does not print a paper comparison value (the
paper's Figures 9-10 give no single-number range to compare against in
the same way Figures 5/6/8 do).
**Backs up:** Figure 6 comparison (`no_delay`), Figure 8
(`constant_delay_0.5s`), Figures 9-10 (`random_delay`). This is the
primary evidence for VERIFICATION_LOG.md Section 8, Exp3's "changed
conclusion" (Fuzzy-PID is ~10% tighter under `no_delay` and
`constant_delay_0.5s`, but *worse* than plain PID under `random_delay` --
both directions are in this one file, not just the favorable case).

### `other_metrics.json`
Source: `exp1_single_vehicle_pid.py`, `exp2_platoon_baseline_pid.py`,
`exp7_heterogeneous_platoon.py` -- the three experiments whose numeric
output didn't fit the tau-sweep or fuzzy-vs-PID groupings above.
A list of 8 objects, distinguished by the `"experiment"` key:
- 1 object from exp1: single-vehicle spacing error range, final
  velocity, and time-to-target-speed, plus the paper's Figure 5a
  reference range.
- 3 objects from exp2 (one per vehicle pair: Lead-2nd, 2nd-3rd, 3rd-4th):
  spacing error range and peak absolute error in the 10-30s window.
- 4 objects from exp7 (one per vehicle pair plus one "platoon-wide"
  summary row): `Ep_ours`/`Ep_paper`, `Mp_ours`/`Mp_paper`,
  `sigma_p_ours`/`sigma_p_paper`.

  Note on exp7's numbers: the original console output right-aligned
  each field to a fixed width, and several of "our" values are large
  enough (the platoon is unstable at these tau values -- see Section 2
  of VERIFICATION_LOG.md) that they overflowed their column and ran
  together with the adjacent paper-reference number with no separating
  space (e.g. the raw text `377525.8842    0.11499503889.5681` is
  actually four separate numbers: `Ep_ours=377525.8842`,
  `Ep_paper=0.1149`, `Mp_ours=9503889.5681`, and the start of the next
  field). These were reconciled by hand against the script's known
  column widths and format spec before being split into the JSON below;
  the split was double-checked by reconstructing the original text
  string from the parsed values and confirming it matches byte-for-byte.
**Backs up:** exp1 -> Figure 5 (spacing error / velocity tracking under
plain PID). exp2 -> Figure 6a-c (PID curves) / string-stability check.
exp7 -> Table 3 (simulation parameters) and Table 4 (heterogeneous
platoon results), VERIFICATION_LOG.md Section 8, Exp7.

## How to regenerate any of these

```bash
cd platoon_project
source Platoon/bin/activate
pytest tests/ -v 2>&1 | tee results/pytest_output.txt
python pid_controller.py > results/stability_check.txt 2>&1
# exp1/exp2/exp7 -> other_metrics.json; exp3/exp5/exp6 -> fuzzy_vs_pid_metrics.json;
# exp4 -> exp4_tau_sweep_metrics.json. Run each experiments/expN_*.py script
# and transcribe its printed numeric output into the corresponding JSON
# structure documented above (no expN_summary.txt intermediate is kept).
```

## Note on scope

This folder only captures and reformats output that already existed as
console prints in the current code -- nothing here changes any
simulation, controller, or fuzzy-logic logic, no experiments were
re-run to produce this pass's JSON files (they were built directly from
the previously captured `expN_summary.txt` text), and no new numbers
were computed beyond what each script already prints when run normally.
For the reasoning behind each number (why it does or doesn't match the
paper, and which findings are CHANGED / CONFIRMED / UNRESOLVED), see
`../VERIFICATION_LOG.md` -- this README only maps files to claims, it
doesn't repeat the analysis.
