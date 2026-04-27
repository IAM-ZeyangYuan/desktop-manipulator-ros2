import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

import workspace as ws


plt.rcParams.update({
    'figure.figsize': (6, 4),
    'figure.dpi': 100,
    'font.size': 18,
    'axes.linewidth': 1.0,
    'lines.linewidth': 3,
    'grid.linestyle': ':',
    'grid.color': 'gray',
    'grid.alpha': 0.5,
    
    'font.weight': 'bold',          # <-- global text
    'axes.labelweight': 'bold',      # <-- x/y labels
    'axes.titleweight': 'bold'      # <-- titles
})
plt.rcParams['mathtext.default'] = 'bf'

def build_lambdified_Je():
    """
    Closed-form ^eJ (6x4) from your derived result.
    Joint order: [theta1, theta2, theta3, d4]
    """
    theta1, theta2, theta3, d4 = sp.symbols("theta1 theta2 theta3 d4", real=True)

    # Link lengths from workspace_w_arm.py
    L2, L3, L4 = ws.L2, ws.L3, ws.L4

    # Shorthand that appears repeatedly in your matrix:
    A = -L2 + (L3 + L4/2 + d4) * sp.sin(theta2)

    # Your ^eJ as shown (linear part first 3 rows, angular part last 3 rows)
    Je = sp.Matrix([
        [ A*sp.sin(theta3),                  -(L3 + L4/2 + d4)*sp.cos(theta3),  0,    0],
        [ A*sp.cos(theta3) - (L4/2)*sp.cos(theta2), (L3 + L4/2 + d4)*sp.sin(theta3),  L4/2, 0],
        [ -(L4/2)*sp.sin(theta2)*sp.sin(theta3),     (L4/2)*sp.cos(theta3),     0,    1],
        [ -sp.sin(theta2)*sp.cos(theta3),     -sp.sin(theta3),                  0,    0],
        [  sp.sin(theta2)*sp.sin(theta3),     -sp.cos(theta3),                  0,    0],
        [ -sp.cos(theta2),                     0,                               1,    0],
    ])

    f_Je = sp.lambdify((theta1, theta2, theta3, d4), Je, "numpy")
    return f_Je


def main():
    f_Je = build_lambdified_Je()

    rng = np.random.default_rng(ws.SEED)
    N = ws.N

    t1 = rng.uniform(*ws.theta1_range, N)
    t2 = rng.uniform(*ws.theta2_range, N)
    t3 = rng.uniform(*ws.theta3_range, N)
    d4 = rng.uniform(*ws.d4_range, N)

    mags_pos = np.zeros((N, 4), dtype=float)  # ||Jp_col||
    mags_ang = np.zeros((N, 4), dtype=float)  # ||Jw_col||

    for i in range(N):
        Je = np.asarray(f_Je(t1[i], t2[i], t3[i], d4[i]), dtype=float)  # 6x4

        Jp = Je[0:3, :]   # linear part
        Jw = Je[3:6, :]   # angular part

        mags_pos[i, :] = np.linalg.norm(Jp, axis=0)
        mags_ang[i, :] = np.linalg.norm(Jw, axis=0)

    # Simple stats you can explain
    pos_mean = mags_pos.mean(axis=0)
    ang_mean = mags_ang.mean(axis=0)
    pos_p90  = np.percentile(mags_pos, 90, axis=0)
    ang_p90  = np.percentile(mags_ang, 90, axis=0)

    joints = ["θ1", "θ2", "θ3", "d4"]
    joints_latex = [r"$\theta_1$",r"$\theta_2$",r"$\theta_3$",r"$d_4$"]
    x = np.arange(len(joints))

    # Print numbers (useful for portfolio captions)
    print("\nPositional sensitivity: mean(||Jp column||) over sampled joint configurations")
    for name, v in zip(joints, pos_mean):
        print(f"  {name}: {v:.4f}")

    print("\nAngular sensitivity: mean(||Jω column||) over sampled joint configurations")
    for name, v in zip(joints, ang_mean):
        print(f"  {name}: {v:.4f}")

    # ---- Plot: Positional ----
    plt.figure(figsize=(8,8))
    plt.bar(x, pos_mean)
    plt.plot(x, pos_p90, marker="o", linestyle="None",color = 'red')
    plt.xticks(x, joints_latex)
    plt.ylabel("magnitude(cm/cm or cm/rad)")
    plt.title("Average positional Jacobian column norm per joint")
    plt.legend(["90th percentile", "mean"], loc="best")
    
  

    # ---- Plot: Angular ----
    plt.figure(figsize=(8,8))
    plt.bar(x, ang_mean)
    plt.plot(x, ang_p90, marker="o", linestyle="None",color = 'red')
    plt.xticks(x, joints_latex)
    plt.ylabel("magnitude(rad/cm or rad/rad)")
    plt.title("Average angular Jacobian column norm per joint")
    plt.legend(["90th percentile", "mean"], loc="best")
  
    
    plt.show()


if __name__ == "__main__":
    main()