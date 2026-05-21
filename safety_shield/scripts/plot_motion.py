import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
CSV_PATH = SCRIPT_DIR / "../data/debug_outputs/jumping_measurement_motion_debug.csv"
JOINT = 0  # 0-indexed

df = pd.read_csv(CSV_PATH)
t = df["t"]

signals = [
    ("Position", f"pos_{JOINT}", "rad"),
    ("Velocity", f"vel_{JOINT}", "rad/s"),
    ("Acceleration", f"acc_{JOINT}", "rad/s²"),
    ("Jerk", f"jerk_{JOINT}", "rad/s³"),
]

fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

for ax, (title, col, unit) in zip(axes, signals):
    series = df[col]
    ax.plot(t, series, label=title)
    ax.set_ylabel(unit)
    ax.set_title(f"Joint {JOINT} — {title}")

    # Shade regions where is_safe == 0
    unsafe = df["is_safe"] == 0
    in_region = False
    start = None
    for idx in range(len(t)):
        if unsafe.iloc[idx] and not in_region:
            start = t.iloc[idx]
            in_region = True
        elif not unsafe.iloc[idx] and in_region:
            ax.axvspan(start, t.iloc[idx], color="red", alpha=0.2,
                       label="unsafe" if idx < 20 else "")
            in_region = False
    if in_region:
        ax.axvspan(start, t.iloc[-1], color="red", alpha=0.2)

    ax.legend(loc="upper right")

axes[-1].set_xlabel("Time (s)")

plt.tight_layout()
out = SCRIPT_DIR / f"../data/debug_outputs/joint_{JOINT}_motion_safety.png"
plt.savefig(out, dpi=150)
plt.show()
