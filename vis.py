import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

RESULTS = "results"

CASES_2D = [
    ("2d_raw.csv", "2D Raw Measurements (odometry)"),
    ("2d_batch.csv", "2D Batch (Levenberg-Marquardt)"),
    ("2d_incremental.csv", "2D Incremental (iSAM2)"),
]

CASES_3D = [
    ("3d_raw.csv", "3D Raw Measurements (odometry)"),
    ("3d_batch.csv", "3D Batch (Levenberg-Marquardt)"),
    ("3d_incremental.csv", "3D Incremental (iSAM2)"),
]

# parking-garage.g2o: x/y span ~270 m (ground plane), z spans ~12 m (garage levels),
# so the bird's-eye view is x-y and z is elevation.
BIRDVIEW = ("x", "y")
ELEVATION = "z"


def load(csv_file):
    path = os.path.join(RESULTS, csv_file)
    if not os.path.exists(path):
        print(f"skipping {path} (not found)")
        return None
    return pd.read_csv(path)


def draw_2d(ax, df, title):
    ax.plot(df["x"], df["y"], color="tab:blue", lw=0.8, alpha=0.6, label="Path", zorder=1)

    step = max(1, len(df) // 250)
    sub = df.iloc[::step]
    ax.quiver(sub["x"], sub["y"], np.cos(sub["theta"]), np.sin(sub["theta"]),
              color="tab:red", angles="xy", scale=30, width=0.003,
              label="Heading", zorder=2)

    mark_endpoints(ax, df["x"].values, df["y"].values)
    finish(ax, title, "X (m)", "Y (m)")


def draw_3d_birdview(ax, df, title, zlim):
    u, v = BIRDVIEW
    ax.plot(df[u], df[v], color="0.6", lw=0.6, alpha=0.7, zorder=1)
    sc = ax.scatter(df[u], df[v], c=df[ELEVATION], cmap="viridis",
                    s=4, vmin=zlim[0], vmax=zlim[1], zorder=2)

    mark_endpoints(ax, df[u].values, df[v].values)
    finish(ax, title, f"{u.upper()} (m)", f"{v.upper()} (m)")
    return sc


def draw_3d_sideview(ax, df, title, zlim):
    u = BIRDVIEW[0]
    ax.plot(df[u], df[ELEVATION], color="tab:blue", lw=0.7, alpha=0.7)
    mark_endpoints(ax, df[u].values, df[ELEVATION].values)
    ax.set_ylim(zlim[0] - 1, zlim[1] + 1)
    finish(ax, title, f"{u.upper()} (m)", f"{ELEVATION.upper()} (m)", equal=False)


def mark_endpoints(ax, xs, ys):
    ax.scatter(xs[0], ys[0], color="tab:green", marker="o", s=70,
               label="Start", zorder=5, edgecolors="k", linewidths=0.5)
    ax.scatter(xs[-1], ys[-1], color="purple", marker="X", s=70,
               label="End", zorder=5, edgecolors="k", linewidths=0.5)


def finish(ax, title, xlabel, ylabel, equal=True):
    ax.set_title(title, fontsize=10)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if equal:
        ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(fontsize=7, loc="best")


def save(fig, name):
    path = os.path.join(RESULTS, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {path}")


def visualize_2d():
    frames = [(load(f), t) for f, t in CASES_2D]
    frames = [(d, t) for d, t in frames if d is not None]
    if not frames:
        return

    for df, title in frames:
        fig, ax = plt.subplots(figsize=(9, 8))
        draw_2d(ax, df, title)
        save(fig, title.split("(")[0].strip().lower().replace(" ", "_") + ".png")

    fig, axes = plt.subplots(1, len(frames), figsize=(7 * len(frames), 7))
    for ax, (df, title) in zip(np.atleast_1d(axes), frames):
        draw_2d(ax, df, title)
    fig.suptitle("2D Pose Graph SLAM - Intel dataset", fontsize=13)
    save(fig, "2d_comparison.png")


def visualize_3d():
    frames = [(load(f), t) for f, t in CASES_3D]
    frames = [(d, t) for d, t in frames if d is not None]
    if not frames:
        return

    zs = np.concatenate([d[ELEVATION].values for d, _ in frames])
    zlim = (float(zs.min()), float(zs.max()))

    for df, title in frames:
        fig, axes = plt.subplots(2, 1, figsize=(9, 11),
                                 gridspec_kw={"height_ratios": [3, 1]})
        sc = draw_3d_birdview(axes[0], df, title + " - bird's-eye (X-Y)", zlim)
        fig.colorbar(sc, ax=axes[0], label="Z elevation (m)", shrink=0.8)
        draw_3d_sideview(axes[1], df, "side view (X-Z)", zlim)
        save(fig, title.split("(")[0].strip().lower().replace(" ", "_") + ".png")

    fig, axes = plt.subplots(1, len(frames), figsize=(7 * len(frames), 7))
    for ax, (df, title) in zip(np.atleast_1d(axes), frames):
        sc = draw_3d_birdview(ax, df, title, zlim)
    fig.colorbar(sc, ax=list(np.atleast_1d(axes)), label="Z elevation (m)", shrink=0.7)
    fig.suptitle("3D Pose Graph SLAM - parking garage (bird's-eye X-Y)", fontsize=13)
    save(fig, "3d_comparison.png")


if __name__ == "__main__":
    visualize_2d()
    visualize_3d()
