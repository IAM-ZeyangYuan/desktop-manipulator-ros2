import numpy as np
from scipy.interpolate import CubicSpline


def parabolic_blend(y0, y1, t0, delta_t, t):
    """Single parabolic blend between two values."""
    a0 = y0 + 2 * (y1 - y0) * t0**2 / delta_t**2
    a1 = -4 * (y1 - y0) * t0 / delta_t**2
    a2 = 2 * (y1 - y0) / delta_t**2
    return a0 + a1 * t + a2 * t**2


def parabolic_blend_segment(y0, y1, t0, delta_t, t):
    """Piecewise parabolic blend (your pbblend function)."""
    mid = t0 + delta_t / 2.0
    if t <= mid:
        return parabolic_blend(y0, y1, t0, delta_t, t)
    else:
        mirror_t = 2 * t0 + delta_t - t
        return y0 + y1 - parabolic_blend(y0, y1, t0, delta_t, mirror_t)


def interpolate_parabolic(waypoints, delta_t, points_per_segment=100):
    """
    Interpolate a joint that uses parabolic blend (your theta2 / d4 pattern).
    Waypoints come in pairs: each pair holds the same value, transitions happen
    between pairs.

    Returns a 1D numpy array of interpolated values.
    """
    n = len(waypoints)
    pp = points_per_segment
    segments = []

    for i in range(n - 1):
        if waypoints[i] == waypoints[i + 1]:
            # hold segment
            count = pp - 1 if (i == 0 or i == n - 2) else pp - 2
            segments.extend([waypoints[i]] * count)
        else:
            # blend segment
            t_start = i * delta_t
            t_arr = np.linspace(t_start, t_start + delta_t, pp)
            for t in t_arr:
                segments.append(
                    parabolic_blend_segment(waypoints[i], waypoints[i + 1], t_start, delta_t, t)
                )

    return np.array(segments)


def interpolate_cubic_spline(waypoints, delta_t, num_points):
    """
    Interpolate a joint using cubic spline (your theta1 / theta3 pattern).
    """
    n = len(waypoints)
    t_knots = np.linspace(0, (n - 1) * delta_t, n)
    cs = CubicSpline(t_knots, waypoints, bc_type=((1, 0), (1, 0)))
    t_out = np.linspace(0, (n - 1) * delta_t, num_points)
    return cs(t_out), t_out


def plan_trajectory(theta1_wp, theta2_wp, theta3_wp, d4_wp, delta_t,
                    points_per_segment=100):
    """
    Given waypoint arrays and delta_t, produce the full interpolated trajectory.

    Returns (times, theta1, theta2, theta3, d4) as numpy arrays.
    """
    n = len(theta1_wp)

    # theta2 and d4 use parabolic blend
    comb2 = interpolate_parabolic(theta2_wp, delta_t, points_per_segment)
    comb4 = interpolate_parabolic(d4_wp, delta_t, points_per_segment)

    num_points = len(comb2)

    # theta1 and theta3 use cubic spline
    comb1, times = interpolate_cubic_spline(theta1_wp, delta_t, num_points)
    comb3, _ = interpolate_cubic_spline(theta3_wp, delta_t, num_points)

    return times, comb1, comb2, comb3, comb4