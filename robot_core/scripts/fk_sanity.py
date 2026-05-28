"""Forward-kinematics sanity check — prints FK for representative configs.

Offline, no hardware. Lets you eyeball that the model behaves sensibly:
representative joint configurations (zero, factory point, single-axis extremes),
then the calibration report against the real measured pairs.

Run it::

    python -m robot_core.scripts.fk_sanity
"""

from __future__ import annotations

from robot_core.kinematics import evaluate, forward_kinematics, load_calibration_pairs

# (label, J1, J2, J3, J4) — within the theoretical joint limits.
REPRESENTATIVE = [
    ("all zero",          0.0,   0.0,   0.0,   0.0),
    ("factory (0,0,60,0)", 0.0,   0.0,  60.0,   0.0),
    ("J1 max +160",     160.0,   0.0,  60.0,   0.0),
    ("J1 min -160",    -160.0,   0.0,  60.0,   0.0),
    ("J2 max +85",        0.0,  85.0,  60.0,   0.0),
    ("J2 min -25",        0.0, -25.0,  60.0,   0.0),
    ("J3 max +105",       0.0,   0.0, 105.0,   0.0),
    ("J3 min -25",        0.0,   0.0, -25.0,   0.0),
    ("J4 max +180",       0.0,   0.0,  60.0, 180.0),
    ("reach forward",     0.0,  60.0,  10.0,   0.0),
]


def main() -> None:
    print("=== Forward kinematics — representative configurations ===")
    print(f"{'config':22s}{'J1':>7}{'J2':>7}{'J3':>7}{'J4':>7}    "
          f"{'x':>9}{'y':>9}{'z':>9}{'r':>9}")
    for label, j1, j2, j3, j4 in REPRESENTATIVE:
        x, y, z, r = forward_kinematics(j1, j2, j3, j4)
        print(f"{label:22s}{j1:7.1f}{j2:7.1f}{j3:7.1f}{j4:7.1f}    "
              f"{x:9.2f}{y:9.2f}{z:9.2f}{r:9.2f}")

    print("\n=== Calibration check vs real measured pairs ===")
    report = evaluate(load_calibration_pairs())
    print(report.format())


if __name__ == "__main__":
    main()
