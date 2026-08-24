#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


CSV_FILE = "logs/data_control.csv"


def get_column(df, candidates):
    """
    Return the first matching column.
    """
    for name in candidates:
        if name in df.columns:
            return name

    return None


df = pd.read_csv(CSV_FILE)


# --------------------------------------------------
# Time
# --------------------------------------------------

time_col = get_column(
    df,
    [
        "timestamp"
    ],
)

if time_col is None:
    t = range(len(df))
    xlabel = "Sample"

else:
    t = df[time_col] * 1e-9
    xlabel = "Time (s)"


# --------------------------------------------------
# Find columns
# --------------------------------------------------

z_actual = get_column(
    df,
    ["z"],
)

z_target = get_column(
    df,
    ["z_target"],
)

y_actual = get_column(
    df,
    ["y"],
)

y_target = get_column(
    df,
    ["y_target"],
)

vz_actual = get_column(
    df,
    ["vz"],
)

vz_target = get_column(
    df,
    ["vz_target"],
)

vy_actual = get_column(
    df,
    ["vy"],
)

vy_target = get_column(
    df,
    ["vy_target"],
)

roll_actual = get_column(
    df,
    ["roll"],
)

roll_target = get_column(
    df,
    ["roll_target"],
)


# --------------------------------------------------
# Plot helper
# --------------------------------------------------

def plot_individual(
    actual,
    target,
    title,
    ylabel,
):

    plt.figure(
        figsize=(10, 5)
    )

    if actual is not None:

        plt.plot(
            t,
            df[actual],
            label="Actual",
            linewidth=2,
        )

    if target is not None:

        plt.plot(
            t,
            df[target],
            "--",
            label="Target",
            linewidth=2,
        )

    plt.title(title)

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

    plt.grid(True)
    plt.legend()

    plt.tight_layout()


# --------------------------------------------------
# Individual plots
# --------------------------------------------------

plot_individual(
    z_actual,
    z_target,
    "Z Position",
    "Position (m)",
)

plot_individual(
    y_actual,
    y_target,
    "Y Position",
    "Position (m)",
)

plot_individual(
    vz_actual,
    vz_target,
    "Z Velocity",
    "Velocity (m/s)",
)

plot_individual(
    vy_actual,
    vy_target,
    "Y Velocity",
    "Velocity (m/s)",
)

plot_individual(
    roll_actual,
    roll_target,
    "Roll",
    "Roll (deg)",
)

# --------------------------------------------------
# Actual Roll and Actual Y Velocity in same plot
# --------------------------------------------------

if roll_actual is not None and vy_actual is not None:

    fig, ax1 = plt.subplots(
        figsize=(12, 6)
    )

    # Roll - left y-axis
    ax1.plot(
        t,
        df[roll_actual],
        color="blue",
        linewidth=2,
        label="Roll Actual",
    )

    ax1.set_xlabel(xlabel)
    ax1.set_ylabel("Roll (deg)")
    ax1.grid(True)

    # Y velocity - right y-axis
    ax2 = ax1.twinx()

    ax2.plot(
        t,
        -df[vy_actual],
        color="orange",
        linewidth=2,
        label="Vy Actual",
    )

    ax2.set_ylabel("Y Velocity (m/s)")

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    ax1.legend(
        lines1 + lines2,
        labels1 + labels2,
        loc="best",
    )

    plt.title(
        "Actual Roll vs Actual Y Velocity"
    )

    plt.tight_layout()


# plt.figure()
# plt.plot(t, np.gradient(df[y_actual],t))
# plt.plot(t, df[vy_actual])



# --------------------------------------------------
# Show all figures
# --------------------------------------------------

plt.show()