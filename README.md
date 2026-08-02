# Adaptive Fuzzy-PID Vehicular Platoon: A From-Scratch Reproduction

A complete, independent reproduction of the adaptive Fuzzy-PID platoon
controller proposed in:

> H. Li, H. Wu, I. Gulati, S. A. Ali, V. Pickert, and S. Dlay,
> "An adaptive fuzzy control technique for a high-speed vehicular platoon
> experiencing communication delays," IET Intelligent Transport Systems,
> vol. 18, pp. 173-185, 2024. DOI: 10.1049/itr2.12442.

Every equation was reimplemented from scratch in Python. The project uses
only classical intelligent-systems techniques: a rule-based fuzzy inference
layer (Mamdani inference, centroid defuzzification) driving a conventional
PID loop. No machine learning is used at any point.

## Key finding

The paper's own baseline gains (kp = 2, ki = 3, kd = 0.5) do not satisfy
the paper's own Routh-Hurwitz stability condition (Equation 13). Followed
literally, the platoon diverges. This was verified two independent ways and
is encoded as an automated check in `pid_controller.check_stability`. The
same code is string-stable when supplied with gains that satisfy the
condition, as demonstrated in `experiments/exp8_corrected_baseline.py`.

## Repository structure

    vehicle_model.py     First-order vehicle dynamics (Equation 6)
    spacing.py           Constant time headway and spacing error (Equations 1 to 4)
    pid_controller.py    PID law and the stability check (Equations 16 and 13)
    fuzzy_layer.py       49-rule Mamdani gain scheduler (Table 2, Equation 23)
    delay.py             Constant and random communication delay (Equation 27)
    platoon_sim.py       Chains the four vehicles and applies the V2V delay
    metrics.py           Performance measures Ep, Mp, sigma_p (Equations 24 to 26)

    experiments/         One script per reproduced figure (exp1 to exp7), plus
                         exp8, which verifies the stability finding
    tests/               26 unit tests, each module against its cited equation
    plots/               Generated figures
    results/             Captured metrics (JSON) and logs

    VERIFICATION_LOG.md  Every equation and design decision checked against the paper

## Running it

    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt

    # unit tests
    python -m pytest tests/ -v

    # reproduce the figures (each saves to plots/ and prints its metrics)
    python experiments/exp1_single_vehicle_pid.py
    python experiments/exp2_platoon_baseline_pid.py
    python experiments/exp3_fuzzy_vs_pid.py
    python experiments/exp4_tau_sweep.py
    python experiments/exp5_constant_delay.py
    python experiments/exp6_random_delay.py
    python experiments/exp7_heterogeneous_platoon.py
    python experiments/exp8_corrected_baseline.py

Each module can also be run directly (for example `python pid_controller.py`)
to run its built-in self-check.

## Authors

Akosua Pinamang Atta-Boateng (22424189), Rock Gberbie Ayiku (22424759),
and Frank Kwasi Eguasi Tandoh (22425049).
Course: CSCD612, Intelligent Systems.

## Note

The source paper is copyrighted and is not included in this repository.
