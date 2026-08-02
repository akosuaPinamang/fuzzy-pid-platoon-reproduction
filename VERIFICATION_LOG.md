# Verification Log: Fuzzy-PID Vehicle Platoon Reproduction

This log tracks every equation, table, and design choice checked against
the paper (Li, Wu, Gulati, Ali, Pickert, Dlay, "An adaptive fuzzy control
technique for a high-speed vehicular platoon experiencing communication
delays," IET Intelligent Transport Systems 18, 173-185, 2024, DOI
10.1049/itr2.12442), what matched, what did not, and why. It is split into
three categories, used consistently throughout: **CHANGED** (something in
the code was wrong or improvable and has been fixed), **CONFIRMED**
(something was already correct, and is now backed by a primary-source
citation rather than an assumption), and **UNRESOLVED / GENUINE PAPER
AMBIGUITY** (the paper itself does not give enough information, or
contradicts itself, and no amount of re-reading resolves it).

This is the second pass over this project. The first pass built the
simulation from scratch and flagged several open questions (see git
history / prior version of this file). This pass re-read the entire
source PDF first-hand, at up to 600 DPI on the critical tables and
equations, independently of any note left by the first pass, and checked
every one of those open questions against the primary source.

## How the paper was read

The full PDF (13 printed pages, 173-185) was read page by page. The
critical equations and tables (Table 1, Table 2, Table 3, Eq 12, 13, 15,
16, 21, 22, 27) were additionally re-rendered from the PDF at 500-600 DPI
and cropped/zoomed with Pillow to confirm every character, since the
assignment brief specifically warned that Table 2 had not survived a
prior extraction pass. Nothing below is taken from OCR or a prior
extraction; every quoted number was read off a rendered page image.

---

## 1. Table 2 (49 fuzzy rules) -- CHANGED (data fixed, justification rewritten)

**Full re-transcription, cell by cell**, done from a 600 DPI crop of
printed page 178. Two printing artifacts in the paper itself were found
and resolved:

1. **Column header typo.** Table 2's `e_c` header row reads
   `NB, NM, NM, Z, PS, PM, PB` -- the third column is printed "NM" a
   second time. It must read "NS". Confirmed two ways: (a) the worked
   example directly under the table states "If the inputs e and ec are
   NM and NB, respectively, the outputs ... are PM, PB, PS" -- this only
   matches row NM / column NB (first column) if the column order is
   NB,NM,NS,Z,PS,PM,PB, i.e. only if column 3 is NS; (b) the paper's own
   7-level scheme (defined right after Figure 4: NB,NM,NS,Z,PS,PM,PB) has
   no second NM.
2. **Cell typo.** Cells (e=NB, ec=NM) and (e=NM, ec=NM) both print "PN"
   for the Delta-Kp output. "PN" is not one of the paper's seven
   linguistic levels. Read as "PB" (a single-glyph misprint of B as N),
   the only substitution consistent with the corresponding cell in the
   reference table below.

**Independent cross-check.** All 49 cells (Delta-Kp, Delta-Ki, Delta-Kd)
were checked against the classic Zhao/Tomizuka/Isaka fuzzy-PID
gain-scheduling table -- the same one the paper's own reference [31]
(Bambulkar, Phadke, Salunkhe, "Movement control of robot using fuzzy
PID," 2016 ICEEOT) is built on. Every cell matches exactly once the two
corrections above are applied. This is strong, primary-source-adjacent
confirmation that the transcription is correct, not merely plausible.

**Code impact.** `fuzzy_layer.py`'s `RULES_DKP`/`RULES_DKI`/`RULES_DKD`
tables were, in the prior pass, already a "generic published substitute"
for the unreadable OCR of Table 2 -- and that substitute turned out to
be the *same* classic table the paper's Table 2 is drawn from. 48 of 49
cells were therefore already correct by construction. Exactly one cell
was wrong: **row PB, column PM, Delta-Kp was "NB", corrected to "NM"**.
This is now fixed and cited to the primary source rather than to "a
standard published table."

## 2. Equation 13 / Table 1 (Routh-Hurwitz stability margin) -- CONFIRMED, plus a new secondary finding

The prior pass's headline finding was independently re-derived from
scratch this pass, by hand, using the paper's own characteristic
polynomial, Equation 12:

```
D(s) = s^4 + (1/tau)s^3 + (1/tau)*kd*(1+h)*s^2 + (1/tau)*kp*(1+h)*s + (1/tau)*ki*(1+h)
```

**Derivation 1 -- Routh array.** With a0=1, a1=1/tau, a2=kd(1+h)/tau,
a3=kp(1+h)/tau, a4=ki(1+h)/tau, the standard Routh recursion gives (row
s^2, first entry) b1 = (a1*a2 - a0*a3)/a1 = (1+h)*(kd/tau - kp) =
(1+h)*(kd - kp*tau)/tau, and (row s^1) requires
ki < kp*(1+h)*(kd - kp*tau).

**Derivation 2 -- 4th-order Hurwitz determinant, as an independent
cross-check.** The standard quartic stability condition
a1*a2*a3 - a0*a3^2 - a1^2*a4 > 0, expanded with the same a0..a4, reduces
(after dividing through by (1+h)/tau^3, which is positive) to exactly
`kp*(1+h)*(kd - kp*tau) > ki`, i.e. the same condition as Derivation 1.
Two independent methods agree.

**Comparison with the paper's own printed Table 1 / Equation 13.** The
paper's own Table 1 lists the row-s^2 entry as
`[kd(1+h) - kp(1+h)] / tau` and Equation 13 states the conditions as
`kd > kp` and `0 < ki < kp*(kd-kp)*(1+h)`. Both of our independent
derivations disagree with this by a factor of tau: the correct condition
is **kd > kp*tau** (not kd > kp) and
**0 < ki < kp*(1+h)*(kd - kp*tau)** (not kp*(kd-kp)*(1+h)). The paper's
own Table 1 / Eq 13 appear to have dropped a tau multiplying kp in the
"kd - kp" term.

**Why this doesn't change the headline conclusion.** With the paper's
own baseline (kp=2, kd=0.5, tau=0.5, Eq 22), kd=0.5 is smaller than
*both* kp=2 and kp*tau=1. So the baseline fails the stability condition
whichever version -- the paper's own printed one or our corrected one --
is used; the correction is a secondary, self-contained finding about a
likely algebra slip in the paper's own printed Table 1, not a change to
the reproducibility verdict. `pid_controller.check_stability()` is
deliberately left implementing the condition **as printed** in Eq 13
(not the corrected version), because the specific, useful question that
function answers is "does the paper's own baseline satisfy the paper's
own stated theorem" -- which is the reproducibility question -- not "is
the corrected condition satisfied," a related but different check. Both
the as-printed and the corrected derivation are now documented in
`pid_controller.py`'s module docstring.

**Confirmed, unambiguously, by both hand derivations and two automated
regression tests** (`test_paper_baseline_fails_its_own_stability_condition`,
`test_paper_baseline_error_grows_over_time`): the paper's own printed
baseline parameters kp=2, ki=3, kd=0.5, h=1.5 (Equation 22) do **not**
satisfy the paper's own stability theorem (Equation 13). This is a
genuine finding about the published paper, not a reproduction error.

## 3. Equation 16 (PID law) and its sign convention -- CONFIRMED

Re-transcribed at 500 DPI from printed page 177:

```
u_i(t) = -( kd * epsilon_dot_i(t) + kp * delta_i(t) + ki * INTEGRAL(delta_i(t) dt) )
```

Two things confirmed precisely:
- The derivative term uses **epsilon_dot_i** (Equation 18:
  epsilon_dot_i = x_dot_i - x_dot_{i-1}, the raw relative-velocity /
  gap-rate term), **not** delta_dot_i (which would additionally include
  an h*x_ddot_i term per Equation 4). The code's `gap_rate` argument
  (computed in `platoon_sim.py` as d(gap)/dt via finite differences) is
  the correct analogue of epsilon_dot_i, and this was already correctly
  implemented and documented in the prior pass -- now confirmed against
  the primary source rather than assumed.
- All three inner terms carry a **positive** sign inside the parenthesis;
  there is exactly **one** minus sign in the whole equation, applied to
  the sum as a whole.

**Sign-convention finding (re-investigated, same conclusion, stronger
justification).** Implementing Eq 16 completely literally -- using the
paper's own Equation 2 (`x_i(t) - x_{i-1}(t) = l_{i-1} + epsilon_i(t)`)
together with the literal outer minus sign -- produces a controller that
decelerates when the follower is trailing too far behind: physically
backward, and it diverges immediately for any gain choice. Tracing this
back: Equation 2, read completely literally, requires the *following*
vehicle's position x_i to be numerically *larger* than the position
x_{i-1} of the vehicle ahead of it (so that x_i - x_{i-1} is a positive
length). That is the opposite of the standard road-position convention,
where a vehicle further along the road (further forward) has the larger
coordinate. This is an internal inconsistency in how Eq 2 is printed
relative to the standard convention implied by Figure 2's caption and
the rest of the paper's prose -- not something a closer reading can
resolve, since both conventions are literally present in different parts
of the paper. What *is* unambiguous, independent of that convention
choice, is the physical requirement for negative feedback: a controller
must accelerate when the gap is larger than desired and decelerate when
it is smaller. `spacing.py`'s `measured_gap` (pos_front - pos_rear -
length) uses the standard road convention, i.e. Eq 2 with the two
positions swapped, and `pid_controller.py` uses the positive form of Eq
16. This is the one of the two available sign choices that (a) satisfies
the physical negative-feedback requirement, (b) empirically reproduces
Figure 5's bounded, converging shape, and (c) is consistent with the
Routh-Hurwitz margin above (flipping the sign flips which region of
gain-space is stable). All three checks agree, so this is treated as a
resolved, well-justified reproduction decision. It is now documented at
length in `pid_controller.py` and `spacing.py`, citing the specific
sentence/equation in the paper responsible for the ambiguity.

## 4. Equation 21 (lead vehicle ramp profile) -- CONFIRMED as a paper typo, code already correct

Re-zoomed at 600 DPI on printed page 178. The paper's ramp branch
literally reads `2t m/s, if 10s < t <= 20s`, with no `-10` offset --
confirmed this is not an OCR artifact by direct visual inspection of the
rendered page image. Taken completely literally this is discontinuous
(v0 jumps 0 -> 20 at t=10s, then 40 -> 20 at t=20s). Three independent
pieces of evidence show `2*(t-10)` is intended: (1) the text immediately
below Eq 21 states the ramp's acceleration is 2 m/s^2, which only holds
if the ramp starts at v=0 at t=10s; (2) Figures 5b, 6b, 6c all show a
continuous ramp from 0 to 20 m/s over exactly 10 seconds; (3) continuity
with the two flat branches on either side requires the offset form. This
is a resolved typesetting error (a dropped "-10"), not a genuine
ambiguity. `platoon_sim.lead_speed_profile()` and
`experiments/exp1_single_vehicle_pid.py`'s `lead_speed()` already used
`2*(t-10)` -- confirmed correct, now cited to the primary source with the
three-part justification above instead of a general note.

## 5. Equation 27 / Table 3 (communication-delay noise term) -- CONFIRMED, plus one implementation gap fixed

**The apparent contradiction.** Section 5.2.2 states: "This simulation
sets the noise n(t) to be a uniformly distributed series of numbers
between zero and one, i.e. n(t) ~ N(-0.5, 0.5)." "Uniformly distributed"
and "N(...)" notation appear to conflict if N(a,b) is read as
Normal(mean=a, variance=b).

**Resolution.** Reading the paper's own "N(a,b)" as this paper's
(nonstandard) shorthand for "ranges over the interval [a,b]" -- i.e. as
bounds notation for a *uniform* variable, not Normal-distribution
notation -- resolves everything consistently:
- The word "uniformly distributed" is the more specific, unambiguous
  claim in the sentence.
- Table 3 separately lists `Td ~ N(0,1)s`. With Td = Tc + n(t), Tc=0.5s,
  and n(t) ~ Uniform(-0.5,0.5), Td ranges uniformly over exactly [0,1]s
  -- matching Table 3's "N(0,1)" read as bounds, and matching the same
  [0.1s,1s] order of magnitude already swept in the constant-delay
  section (Figure 7). An unbounded, zero-mean Gaussian would let Td go
  negative, which is not physically sensible for a delay.
- Section 5.3 (heterogeneous platoon) separately states, in plain prose
  with **no** ambiguous notation: "adds Gaussian noise to the original
  speed equation (21) with Mean = 0 and Variance = 1." This is a
  **different** noise source (added to the lead vehicle's velocity
  signal, not the communication delay), and the authors clearly know how
  to say "Gaussian" plainly when they mean it. This confirms "N(...)" for
  the delay noise is being used as informal bounds notation, not as a
  claim of normality.

`delay.py`'s `random_delay()` already implemented n(t) ~ Uniform(-0.5,
0.5) -- confirmed correct. `platoon_sim.py`'s `speed_noise_std` parameter
(Gaussian, via `rng.normal`) already correctly implements the separate
Section 5.3 noise -- confirmed correct, and exp7 already passes
`speed_noise_std=1.0` matching "Variance = 1".

**One implementation gap found and fixed.** Table 3 also lists `Ts =
0.1s`, described in text as "the sampling time" for this random-delay
noise. The prior `random_delay()` resampled n(t) fresh on *every*
simulation timestep (every dt=0.01s) -- 10x more frequent than Table 3
specifies, effectively ignoring Ts. `delay.py` now takes a
`sample_period` argument (default 0.1s, matching Table 3) and holds each
drawn n(t) constant for that period (zero-order hold) before redrawing,
matching Table 3 literally. This changed exp6's and exp7's numeric
results modestly (see Section 8 below) but not their qualitative
conclusions.

## 6. Equation 6/15 (vehicle dynamics) and integration scheme -- CONFIRMED

Eq 6 (`tau*a_dot_i + a_i = u_i`) and Eq 15 (`tau*x'''_i(t) + x''_i(t) =
u_i(t)`, the same relationship in third-derivative-of-position form) were
both re-verified at 500 DPI; `vehicle_model.py`'s first-order-lag Euler
integration matches. **Genuinely unresolved (irreducible):** the paper
states only "MATLAB/Simulink is used to verify and simulate the
performance," with no named solver and no explicit integration step size
anywhere in Sections 2 or 5. Table 3's `Ts = 0.1s` is specifically the
random-delay noise sampling period (see Section 5 above), not a general
integration step. `dt=0.01s` remains a necessary, clearly-documented
assumption -- chosen small relative to the fastest tau tested (0.1s) --
not a value recoverable from the paper.

## 7. Equations 24-26 (Ep, Mp, sigma_p metrics) -- CONFIRMED

`metrics.py` was checked term-by-term against the printed formulas and
matches exactly: Ep is the mean of |delta_i(k)| over all vehicles and
timesteps; Mp is the mean, over vehicles, of each vehicle's own maximum
|delta_i(k)|; sigma_p is the mean, over vehicles, of the standard
deviation of |delta_i(k)| over time. No changes.

## 8. Experiment-by-experiment results (re-run after the fixes above)

All 7 experiment scripts were re-run end-to-end after every fix above,
using the venv in `Platoon/`. Every plot in `plots/` was regenerated.

### Exp1: single vehicle, plain PID, no delay (target: Figure 5)
- Unchanged by this pass's fixes (no delay, no fuzzy layer involved).
  Spacing error range [-1.139, 1.043] m (paper: [-0.6, 0.6] m). Same
  shape, same order of magnitude, consistent with the Eq13 finding above
  (paper's own baseline is not strictly stable, so a wider error band
  than the paper's own plot is expected).

### Exp2: 4-car platoon, plain PID, no delay (target: Figure 6 PID curves)
- Unchanged. Error still amplifies down the platoon with the paper's
  literal baseline (Lead-2nd: [-1.14, 1.04] m, 2nd-3rd: [-10.6, 12.0] m,
  3rd-4th: [-54.9, 49.5] m) -- confirms the platoon-chaining code is
  correct and the instability is a property of the gains, not the code
  (Exp2's control case with Eq13-valid gains, exercised in
  `test_string_stability_with_eq13_valid_gains`, properly attenuates).

### Exp3: Fuzzy-PID vs plain PID (target: Figure 6 comparison) -- key result, changed conclusion

- **First re-run, with the corrected Table 2 (Section 1) but the prior
  pass's original scaling** (e_scale=0.3, ec_scale=0.2,
  out_scales=(0.05,0.02,0.02)): Fuzzy-PID [-1.318, 1.203] m vs plain PID
  [-1.139, 1.043] m -- fuzzy not tighter, essentially unchanged from the
  pre-fix run. Expected: correcting 1 cell out of 49 should not move the
  aggregate result much.
- **Root-caused the real obstacle.** Confirmed the rule-table values were
  never the main issue (48/49 cells were already correct pre-fix). Two
  small grid sweeps over the fuzzy scaling factors (e_scale, ec_scale,
  out_scales) were run, on both the paper's literal baseline (kp=2,
  ki=3, kd=0.5) and the Eq13-valid control baseline (kp=0.5, ki=0.3,
  kd=2), to check whether *some* scaling choice recovers a tighter band.
  The prior pass's out_scales=(0.05,0.02,0.02) turned out to be too
  gentle for the correction to have a visible effect; doubling it to
  out_scales=(0.1,0.05,0.05) (with e_scale=1.0, ec_scale=0.05) was the
  best performer in both sweeps and **does** produce a genuinely tighter
  Fuzzy-PID band on the paper's own literal baseline:
  **Fuzzy-PID [-1.032, 0.937] m (width 1.969 m) vs plain PID [-1.139,
  1.043] m (width 2.182 m), a ~10% narrower band.**
- **This is a changed conclusion from the prior pass's log**, which
  reported no configuration found that beat plain PID. `experiments/
  exp3_fuzzy_vs_pid.py` (and exp4, exp5, exp6, which share the same
  fuzzy-controller construction) were updated to use this scaling.
- **Honest assessment of the remaining gap.** The paper reports a
  visibly larger effect: Fuzzy-PID -0.23 to 0.58 m (width 0.81 m) vs
  plain PID -0.6 to 0.6 m (width 1.2 m), a ~32% narrower band, on top of
  a baseline that (unlike ours) is small to begin with because the
  paper's own plot only extends to 40s before the slow divergence in
  Section 2 above becomes visually obvious. Our reproduction shows the
  correct *direction* (fuzzy tighter than plain PID) but roughly a third
  of the paper's claimed *magnitude* of improvement, and only for this
  specific scaling choice -- other reasonable choices in the sweep did
  not beat plain PID. The most defensible explanation, consistent with
  everything else found this pass, is the genuine gap identified in
  Section on Table 2/scaling factors: the paper's only concrete
  quantization numbers (0.3, 0.03, 8) are explicitly given for a [-3,3]
  fuzzy domain in the same paragraph that then states the actual
  membership functions use [-1,1] (confirmed by Figure 4), so the
  authors' real, effective scaling for their own implementation is not
  recoverable from the source PDF. **This is reported as a partial,
  qualitatively-consistent but quantitatively-incomplete reproduction,
  not a full match and not a total miss.**
- Cross-checked at wider scope in Exp4's tau sweep (see below): the
  tightening effect is visible but inconsistent across tau values,
  further supporting "real but modest and sensitive to the unrecoverable
  scaling choice" as the honest characterization.

### Exp4: tau sweep (target: Figure 7)
- Re-run with the corrected fuzzy table and the improved out_scales
  identified in Exp3. The instability transition is unchanged from
  before (clean, small error for tau <= 0.4s; sharp transition to large,
  then catastrophic error from tau=0.5s upward, Mp from ~1.1m at tau=0.5
  to ~31,000-35,000m at tau=1.0), independently confirming the Eq13
  finding: the paper's own tau=0.5s baseline sits right at the edge of
  the instability boundary. With the improved fuzzy scaling, Fuzzy-PID
  now shows a *real but inconsistent* improvement over plain PID across
  the sweep: tighter Ep/Mp at tau=0.5-0.7 and tau=0.9-1.0 (e.g. tau=0.6:
  PID Ep=5.97 vs Fuzzy Ep=4.88), but *not* tighter at every tau (e.g.
  tau=0.8: PID Mp=3135 vs Fuzzy Mp=3273, fuzzy slightly worse). This
  pattern -- real but inconsistent tightening -- matches the honest
  characterization from Exp3 rather than a clean win.

### Exp5: constant 0.5s delay (target: Figure 8)
- Re-run with the corrected fuzzy table and improved scaling (constant
  delay is unaffected by the Ts sample-and-hold fix in Section 5, since
  it doesn't call `random_delay`). Plain PID: [-0.989, 1.043] m (width
  2.032); Fuzzy-PID: [-1.032, 0.937] m (width 1.969) -- a modest but real
  tightening, consistent with Exp3 (paper: [-0.6, 0.6] m PID, [-0.27,
  0.6] m fuzzy-PID).

### Exp6: random delay (target: Figures 9-10)
- Re-run with **both** fixes (corrected fuzzy table + improved scaling,
  and the Ts=0.1s sample-and-hold fix from Section 5). Plain PID:
  [-29.0, 26.4] m (width 55.4); Fuzzy-PID: [-40.1, 40.2] m (width 80.3).
  Here Fuzzy-PID is clearly **worse**, not better, than plain PID -- the
  opposite of Exp3/Exp5's finding. This is a genuine, reported negative
  result specific to the random-delay regime: once the system is already
  being pushed toward the catastrophic-instability region identified in
  Exp4 (random delay periodically exceeds the tau=0.5s edge), the
  additional fuzzy gain correction appears to amplify rather than damp
  the resulting oscillation. Both directions (fuzzy helps under mild
  delay, fuzzy hurts under severe/random delay) are reported plainly
  rather than only reporting the favorable case.

### Exp7: heterogeneous platoon (target: Table 4)
- Re-run with the Ts=0.1s fix. Using the paper's literal baseline and
  Table 3's own tau values (0.5, 0.8, 1.0), the simulation still diverges
  to astronomically large values (platoon-wide Ep ~2.8 million m), fully
  expected since taus of 0.8 and 1.0 are deep in Exp4's catastrophic
  region. This is unchanged in conclusion from the pre-fix run; the exact
  diverging numbers are not meaningful to compare before/after since the
  system is unstable in both cases (chaotic sensitivity to the delay
  sampling scheme is expected once the system is already diverging).

---

## Summary for the report

**CHANGED** (fixed this pass): the one incorrect cell in the fuzzy rule
table (row PB / col PM / Delta-Kp); the random-delay noise sampling rate
(now held for Ts=0.1s per Table 3 instead of resampled every timestep);
the docstring claim that the 0.3/0.03/8 scaling factors are "the paper's
own" numbers (they are not -- see Section 5 above); and the fuzzy output
scaling used in experiments 3-6 (out_scales doubled from (0.05,0.02,0.02)
to (0.1,0.05,0.05)), which changes this project's central fuzzy-vs-PID
conclusion from "no configuration found that beats plain PID" to "a
modest, direction-correct improvement was found under mild/no delay
(Exp3, Exp5), but not under random delay (Exp6), and inconsistently
across the tau sweep (Exp4)" -- a more complete and more honest picture
than either "it fails" or "it fully reproduces the paper."

**CONFIRMED** (already correct, now backed by primary-source citations
instead of assumptions): the Eq13 stability-margin finding and its
independent re-derivation (with a new secondary finding about a likely
tau-factor slip in the paper's own Table 1); the Eq16 sign convention;
the Eq21 ramp-profile typo fix; the Eq27/Table 3 uniform-vs-Gaussian
noise resolution; the vehicle dynamics model (Eq6/15); and the Ep/Mp/
sigma_p metrics (Eq24-26).

**UNRESOLVED / genuine paper ambiguity** (documented, not fixed, because
the paper does not give enough information): the exact ODE solver and
integration step size (paper only says "MATLAB/Simulink," no step size
given anywhere); and the exact fuzzy input/output scaling factors for
the paper's own [-1,1] membership-function domain (the only concrete
numbers given, 0.3/0.03/8, are explicitly for a different, [-3,3] domain
in the same paragraph). As a direct consequence of that second gap, the
paper's specific *quantitative* claim that Fuzzy-PID produces a visibly
tighter error band than plain PID (Figure 6a: -0.23 to 0.58 m vs -0.6 to
0.6 m, a ~32% narrower band) is only **partially** reproduced: with the
corrected Table 2 and a hand-searched scaling configuration, this
reproduction shows the same *direction* of improvement (~10% narrower
band under no delay or constant delay), but roughly a third of the
paper's claimed magnitude, and the improvement disappears or reverses
under random delay (Exp6) and is inconsistent across the tau sweep
(Exp4). This is reported as a partial, well-investigated, and honestly
characterized result -- neither "fully reproduced" nor "not reproduced
at all" -- with the remaining gap attributed to the unrecoverable
scaling-factor ambiguity above, not to an error in this implementation.
