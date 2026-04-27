import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pandas as pd

# =========================
# YOUR GEOMETRY PARAMETERS (cm)
# =========================
SCALE = 1.0
LINK_RADIUS = 0.7 * SCALE
TELES_INNER_RADIUS = 0.6 * SCALE

# this matches your current code where you hard-set inner_len = 15
INNER_TELESCOPE_LEN = 15.0 * SCALE
# =========================


def unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("Zero-length vector encountered while normalizing.")
    return v / n


def segment_segment_distance(p1, q1, p2, q2, eps=1e-12):
    """
    Minimum distance between 3D segments S1: p1->q1 and S2: p2->q2.
    Returns (distance, c1, c2) where c1 and c2 are closest points on each segment.
    Implementation: standard closest-points on two segments with clamping.
    """
    p1 = np.asarray(p1, float); q1 = np.asarray(q1, float)
    p2 = np.asarray(p2, float); q2 = np.asarray(q2, float)

    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2

    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))

    # Handle degenerate segments
    if a < eps and e < eps:
        c1 = p1
        c2 = p2
        return float(np.linalg.norm(c1 - c2)), c1, c2

    if a < eps:
        # First segment is a point
        s = 0.0
        t = np.clip(f / e, 0.0, 1.0) if e > eps else 0.0
    else:
        c = float(np.dot(d1, r))
        if e < eps:
            # Second segment is a point
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b

            if abs(denom) > eps:
                s = np.clip((b * f - c * e) / denom, 0.0, 1.0)
            else:
                # Nearly parallel
                s = 0.0

            t = (b * s + f) / e

            if t < 0.0:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = np.clip((b - c) / a, 0.0, 1.0)

    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    return float(np.linalg.norm(c1 - c2)), c1, c2


def fk_points_and_axes(theta1, theta2, theta3, d4):
    """
    Returns the key points/axes you need:
      p1, p2: link-2 segment endpoints
      p4_base: telescope base point
      k3: joint-3 axis unit vector in world
    Uses your exact transforms and your L-shape construction.
    """
    L1 = 35
    L2 = 25
    L3 = 2
    L4 = 10

    T01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                    [np.sin(theta1),  np.cos(theta1), 0, 0],
                    [0,               0,              1, L1],
                    [0,               0,              0, 1]])

    T12 = np.array([[np.cos(theta2), -np.sin(theta2), 0, L2],
                    [0,               0,              1, 0],
                    [-np.sin(theta2), -np.cos(theta2), 0, 0],
                    [0,               0,              0, 1]])

    T23 = np.array([[np.cos(theta3), -np.sin(theta3), 0, 0],
                    [0,               0,              1, L3],
                    [-np.sin(theta3), -np.cos(theta3), 0, 0],
                    [0,               0,              0, 1]])

    T02 = T01 @ T12
    T03 = T02 @ T23

    p1 = T01[:3, 3]
    p2 = T02[:3, 3]
    p3 = T03[:3, 3]

    R03 = T03[:3, :3]
    i3 = unit(R03[:, 0])
    k3 = unit(R03[:, 2])

    p4_base = p3 + i3 * L4
    # p4 = p4_base + k3 * d4  # not needed for this distance

    return p1, p2, p4_base, k3


def inner_telescope_segment(p4_base, k3, d4, inner_len=INNER_TELESCOPE_LEN):
    """
    Match your CURRENT visualization logic:
      inner_center = p4_base + k3 * (d4 - inner_len*0.5)
      height = inner_len along k3
    So endpoints are center ± k3*(inner_len/2).
    """
    k3 = unit(k3)
    inner_center = p4_base + k3 * (d4 - 0.5 * inner_len)
    a = inner_center - k3 * (0.5 * inner_len)
    b = inner_center + k3 * (0.5 * inner_len)
    return a, b


def compute_distance_over_time(t, th1, th2, th3, d4):
    t = np.asarray(t, float)
    th1 = np.asarray(th1, float)
    th2 = np.asarray(th2, float)
    th3 = np.asarray(th3, float)
    d4  = np.asarray(d4, float)

    n = len(t)
    dist = np.zeros(n, float)

    # optional: closest points (for debugging)
    c12 = np.zeros((n, 3), float)   # closest on link12
    cTel = np.zeros((n, 3), float)  # closest on telescope

    for i in range(n):
        p1, p2, p4_base, k3 = fk_points_and_axes(th1[i], th2[i], th3[i], d4[i])

        # link-2 centerline segment
        s1a, s1b = p1, p2

        # telescope inner centerline segment
        s2a, s2b = inner_telescope_segment(p4_base, k3, d4[i], inner_len=INNER_TELESCOPE_LEN)

        di, ci1, ci2 = segment_segment_distance(s1a, s1b, s2a, s2b)
        dist[i] = di
        c12[i] = ci1
        cTel[i] = ci2

    return dist, c12, cTel


def plot_distance(t, dist):
    plt.figure(figsize=(8, 8))
    plt.plot(t, dist, linewidth=4)
    plt.xlabel("t(s)")
    plt.ylabel("centerline distance (cm)")
    plt.xlim(0,max(t))
    plt.grid(True)

    # simple collision indicator based on radii (centerline threshold)
    threshold = LINK_RADIUS + TELES_INNER_RADIUS
    plt.axhline(threshold, linestyle="--",color = 'r', linewidth=4)
    #plt.title(f"Link12 ↔ inner telescope distance (threshold = {threshold:.2f} cm)")
    # annotation pointing to the y-axis at the threshold
    # write text at the y-axis at the threshold
    plt.text(
        0.0, threshold,
        "threshold",
        color="r",
        ha="right",
        va="center",
        transform=plt.gca().get_yaxis_transform()
    )
    plt.show()


# -------------------------
# EXAMPLE USAGE
# Replace these lists with your trajectory data.
# -------------------------

#reading data from the generated trajectory


in_path = Path("trajectory.csv")
in_path = (
    Path(__file__).resolve().parent
    / ".."
    / "manipulator_planning"
    / "data"
    / "trajectory.csv"
).resolve()
df = pd.read_csv(in_path)




if __name__ == "__main__":

    t = df["time"].tolist()
    th1 = df["theta1"].tolist()         # rad
    th2 = (df["theta2"]-np.pi/2).tolist()  # rad #covertinng from ros2 convetion to dh convention
    th3 = (df["theta3"]+np.pi/2).tolist()         # rad #covertinng from ros2 convetion to dh convention
    d4 = (df["d4"]*100).tolist()  # cm


    dist, _, _ = compute_distance_over_time(t, th1, th2, th3, d4)
    plot_distance(t, dist)
