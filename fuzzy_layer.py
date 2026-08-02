"""
fuzzy_layer.py

Fuzzy inference block: reads (e, ec) = (spacing error, its rate of change)
and outputs (delta_kp, delta_ki, delta_kd), the corrections added to the
base PID gains each step (Equation 23). Mamdani inference over the paper's
49-rule Table 2, with centroid defuzzification.

Membership functions: triangular, 7 levels (NB..PB) over [-1, 1], per
Figure 4. Two printing errors in the paper's Table 2 were found and
corrected here (a duplicated column header and a "PN" cell); the full
transcription and cross-check are in VERIFICATION_LOG.md.
"""
import numpy as np
from scipy.interpolate import interpn
import skfuzzy as fuzz
import skfuzzy.control as ctrl

LEVELS = ["NB", "NM", "NS", "Z", "PS", "PM", "PB"]

# Standard published fuzzy-PID rule tables (rows = e, cols = ec), using the
# same 7-level linguistic scheme as the paper. This is the commonly cited
# generic fuzzy-PID rule structure, not the paper's own unrecoverable table.
RULES_DKP = [
    ["PB", "PB", "PM", "PM", "PS", "Z",  "Z" ],
    ["PB", "PB", "PM", "PS", "PS", "Z",  "NS"],
    ["PM", "PM", "PM", "PS", "Z",  "NS", "NS"],
    ["PM", "PM", "PS", "Z",  "NS", "NM", "NM"],
    ["PS", "PS", "Z",  "NS", "NS", "NM", "NM"],
    ["PS", "Z",  "NS", "NM", "NM", "NM", "NB"],
    ["Z",  "Z",  "NM", "NM", "NM", "NM", "NB"],  # row PB: col PM corrected NB->NM (see note above)
]
RULES_DKI = [
    ["NB", "NB", "NM", "NM", "NS", "Z",  "Z" ],
    ["NB", "NB", "NM", "NS", "NS", "Z",  "Z" ],
    ["NB", "NM", "NS", "NS", "Z",  "PS", "PS"],
    ["NM", "NM", "NS", "Z",  "PS", "PM", "PM"],
    ["NM", "NS", "Z",  "PS", "PS", "PM", "PB"],
    ["Z",  "Z",  "PS", "PS", "PM", "PB", "PB"],
    ["Z",  "Z",  "PS", "PM", "PM", "PB", "PB"],
]
RULES_DKD = [
    ["PS", "NS", "NB", "NB", "NB", "NM", "PS"],
    ["PS", "NS", "NB", "NM", "NM", "NS", "Z" ],
    ["Z",  "NS", "NM", "NM", "NS", "NS", "Z" ],
    ["Z",  "NS", "NS", "NS", "NS", "NS", "Z" ],
    ["Z",  "Z",  "Z",  "Z",  "Z",  "Z",  "Z" ],
    ["PB", "NS", "PS", "PS", "PS", "PS", "PB"],
    ["PB", "PM", "PM", "PM", "PS", "PS", "PB"],
]


def _build_membership(universe_name, rng):
    var = ctrl.Antecedent(rng, universe_name) if universe_name in ("e", "ec") \
        else ctrl.Consequent(rng, universe_name)
    centers = np.linspace(-1, 1, 7)
    width = centers[1] - centers[0]
    for level, c in zip(LEVELS, centers):
        var[level] = fuzz.trimf(rng, [c - width, c, c + width])
    return var


class FuzzyGainCorrector:
    """
    The fuzzy system is evaluated once over a grid of (e, ec) values to
    build a lookup table, then interpolated during the simulation. This is
    how fuzzy controllers are usually deployed (a precomputed table) and it
    avoids scikit-fuzzy's per-call slowdown in a tight loop.
    """
    def __init__(self, e_scale=1.0, ec_scale=0.5, out_scales=(1.0, 1.0, 1.0), grid_size=21):
        """
        e_scale, ec_scale : factors mapping the real error / error-rate onto
            the fuzzy universe [-1, 1]. Hand-tuned: the paper states its
            scaling factors for a contradictory [-3,3] domain, so they are
            not directly reusable (see VERIFICATION_LOG.md).
        out_scales : multipliers on the fuzzy output to give the actual
            delta_kp, delta_ki, delta_kd.
        grid_size : lookup-table resolution per axis.
        """
        self.e_scale = e_scale
        self.ec_scale = ec_scale
        self.out_scales = out_scales

        rng = np.linspace(-1, 1, 201)
        self.e_var = _build_membership("e", rng)
        self.ec_var = _build_membership("ec", rng)
        self.dkp_var = _build_membership("dkp", rng)
        self.dki_var = _build_membership("dki", rng)
        self.dkd_var = _build_membership("dkd", rng)

        rules = []
        for i, e_level in enumerate(LEVELS):
            for j, ec_level in enumerate(LEVELS):
                rules.append(ctrl.Rule(
                    self.e_var[e_level] & self.ec_var[ec_level],
                    [self.dkp_var[RULES_DKP[i][j]],
                     self.dki_var[RULES_DKI[i][j]],
                     self.dkd_var[RULES_DKD[i][j]]]
                ))
        self.system = ctrl.ControlSystem(rules)

        # Precompute the lookup table once.
        self.grid = np.linspace(-1, 1, grid_size)
        self.dkp_table = np.zeros((grid_size, grid_size))
        self.dki_table = np.zeros((grid_size, grid_size))
        self.dkd_table = np.zeros((grid_size, grid_size))
        # A fresh ControlSystemSimulation per grid point avoids scikit-fuzzy's
        # cache-growth slowdown when one object is reused across many calls.
        for a, e_val in enumerate(self.grid):
            for b, ec_val in enumerate(self.grid):
                sim = ctrl.ControlSystemSimulation(self.system)
                sim.input["e"] = e_val
                sim.input["ec"] = ec_val
                sim.compute()
                self.dkp_table[a, b] = sim.output["dkp"]
                self.dki_table[a, b] = sim.output["dki"]
                self.dkd_table[a, b] = sim.output["dkd"]

    def _interp(self, table, e_fuzzy, ec_fuzzy):
        return float(interpn(
            (self.grid, self.grid), table, [[e_fuzzy, ec_fuzzy]],
            method="linear", bounds_error=False, fill_value=None
        )[0])

    def compute(self, e_raw, ec_raw):
        e_fuzzy = np.clip(e_raw * self.e_scale, -1, 1)
        ec_fuzzy = np.clip(ec_raw * self.ec_scale, -1, 1)
        dkp = self._interp(self.dkp_table, e_fuzzy, ec_fuzzy) * self.out_scales[0]
        dki = self._interp(self.dki_table, e_fuzzy, ec_fuzzy) * self.out_scales[1]
        dkd = self._interp(self.dkd_table, e_fuzzy, ec_fuzzy) * self.out_scales[2]
        return dkp, dki, dkd


class FuzzyPIDController:
    """
    Combines a base PIDController with a FuzzyGainCorrector.
    Equation 23: Kp = kp_base + delta_kp (same pattern for Ki, Kd).
    Gains are recomputed fresh every step from the CURRENT (e, ec), then
    used for that step's control output. This matches the paper's
    description of continuous, real-time gain adjustment.
    """
    def __init__(self, kp_base, ki_base, kd_base, dt, e_scale=1.0, ec_scale=0.5,
                 out_scales=(1.0, 1.0, 1.0), shared_fuzzy=None):
        """
        shared_fuzzy : an existing FuzzyGainCorrector instance to reuse.
            Pass the SAME instance to every vehicle's controller in a
            platoon so the (slow-ish, one-time) lookup table is only
            built once instead of once per vehicle.
        """
        self.kp_base, self.ki_base, self.kd_base = kp_base, ki_base, kd_base
        self.dt = dt
        self.integral = 0.0
        self.fuzzy = shared_fuzzy if shared_fuzzy is not None else \
            FuzzyGainCorrector(e_scale=e_scale, ec_scale=ec_scale, out_scales=out_scales)

    def reset(self):
        self.integral = 0.0

    def compute(self, delta, gap_rate):
        dkp, dki, dkd = self.fuzzy.compute(delta, gap_rate)
        kp = self.kp_base + dkp
        ki = self.ki_base + dki
        kd = self.kd_base + dkd
        self.integral += delta * self.dt
        u = (kd * gap_rate + kp * delta + ki * self.integral)
        return u


if __name__ == "__main__":
    fz = FuzzyGainCorrector()
    # Checkpoint: spot check a handful of rules against the tables above.
    tests = [
        (-1.0, -1.0, "NB,NB -> expect dkp=PB(+), dki=NB(-), dkd=PS(+ small)"),
        (0.0, 0.0, "Z,Z -> expect all near zero"),
        (1.0, 1.0, "PB,PB -> expect dkp=NB(-), dki=PB(+), dkd=PB(+)"),
    ]
    for e, ec, desc in tests:
        dkp, dki, dkd = fz.compute(e, ec)
        print(f"e={e:+.1f} ec={ec:+.1f}: dkp={dkp:+.3f} dki={dki:+.3f} dkd={dkd:+.3f}  ({desc})")
