# armcode_sympy.py
# Python/SymPy replacement for armcode.m (symbolic DH, velocity, acceleration recursion)

import sympy as sp

def dh_transform(a, alpha, d, theta):
    """Standard DH homogeneous transform (matches the MATLAB matrix in your file)."""
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, a],
        [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), -sp.sin(alpha), -sp.sin(alpha)*d],
        [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha),  sp.cos(alpha),  sp.cos(alpha)*d],
        [0, 0, 0, 1]
    ])

def main():
    # --- Symbols (mirrors your MATLAB syms) ---
    theta1, theta2, theta3, theta4 = sp.symbols('theta1 theta2 theta3 theta4')
    L0, L1, L2, L2a, L2b, L3, L4, L5, Le = sp.symbols('L0 L1 L2 L2a L2b L3 L4 L5 Le')
    d1, d2, d3, d4 = sp.symbols('d1 d2 d3 d4')
    a, b = sp.symbols('a b')

    theta1_dot, theta2_dot, theta3_dot, theta4_dot = sp.symbols('theta1_dot theta2_dot theta3_dot theta4_dot')
    d1_dot, d2_dot, d3_dot, d4_dot = sp.symbols('d1_dot d2_dot d3_dot d4_dot')

    theta1_acc, theta2_acc, theta3_acc = sp.symbols('theta1_acc theta2_acc theta3_acc')
    d3_acc, d4_acc = sp.symbols('d3_acc d4_acc')

    # --- DH table (matches your MATLAB DH) ---
    # DH = [a, alpha, d, theta]
    DH = sp.Matrix([
        [0,     0,     L1,     theta1],
        [L2,  -sp.pi/2,     0,     theta2],
        [0,  -sp.pi/2, L3,      theta3],
        [L4,   0, d4, 0],
        [-L4/2,   0, L4/2, 0]
        
    ])


    n = DH.rows  # number of transforms

    # --- Build per-link transforms and overall transform ---
    Ts = []
    T_all = sp.eye(4)

    for i in range(n):
        ai, alpi, di, thei = DH[i, 0], DH[i, 1], DH[i, 2], DH[i, 3]
        Ti = dh_transform(ai, alpi, di, thei)
        Ts.append(Ti)
        T_all = T_all * Ti

    T_sim = sp.simplify(T_all)            # ^0_n T
    R_for_J = T_sim[:3, :3]               # ^0_n R

    # --- Jacobian-related recursion (velocities) ---
    # You hard-coded these in MATLAB; keep the same structure.
    theta_dot = sp.Matrix([theta1_dot, theta2_dot, theta3_dot, 0,0])
    d_dot     = sp.Matrix([0, 0, 0, d4_dot,0])

    k = sp.Matrix([0, 0, 1])

    jOj = sp.Matrix([0, 0, 0])  # angular velocity in current frame
    jVj = sp.Matrix([0, 0, 0])  # linear velocity in current frame

    Vs = [jVj]  # Vs[j] corresponds to frame j (0-based: frame 0 stored at index 0)
    Os = [jOj]

    for j in range(n):
        P = Ts[j][:3, 3]                 # position of origin of frame (j+1) in frame j
        Rm_m1 = Ts[j][:3, :3]            # ^{m-1}_m R; we want ^m_{m-1}R, so invert
        R = sp.simplify(Rm_m1.inv())     # ^m_{m-1} R

        j1Vj1 = R * (jVj + jOj.cross(P)) + d_dot[j] * k
        j1Oj1 = R * jOj + theta_dot[j] * k

        j1Vj1 = sp.simplify(j1Vj1)
        j1Oj1 = sp.simplify(j1Oj1)

        Vs.append(j1Vj1)
        Os.append(j1Oj1)

        jVj, jOj = j1Vj1, j1Oj1

    # --- Acceleration recursion ---
    theta_acc = sp.Matrix([theta1_acc, theta2_acc, theta3_acc, 0,0])
    d_acc     = sp.Matrix([0, 0, 0, d4_acc,0])

    jO_dotj = sp.Matrix([0, 0, 0])
    jV_dotj = sp.Matrix([0, 0, 0])

    V_dots = [jV_dotj]
    O_dots = [jO_dotj]

    for j in range(n):
        P = Ts[j][:3, 3]
        Rm_m1 = Ts[j][:3, :3]
        R = sp.simplify(Rm_m1.inv())

        # Matches your MATLAB expression:
        # j1V_dotj1 = R*(jV_dotj + cross(jO_dotj,P)+cross(Os{j},cross(Os{j},P)))
        #            + 2*cross(Os{j+1},d_dot(j)*k)+d_acc(j)*k;
        term_centripetal = Os[j].cross(Os[j].cross(P))
        j1V_dotj1 = (
            R * (jV_dotj + jO_dotj.cross(P) + term_centripetal)
            + 2 * (Os[j+1].cross(d_dot[j] * k))
            + d_acc[j] * k
        )

        # Matches your MATLAB expression:
        # j1O_dotj1 = R*jO_dotj + cross(R*Os{j},theta_dot(j)*k) + theta_acc(j)*k;
        j1O_dotj1 = R * jO_dotj + (R * Os[j]).cross(theta_dot[j] * k) + theta_acc[j] * k

        j1V_dotj1 = sp.simplify(j1V_dotj1)
        j1O_dotj1 = sp.simplify(j1O_dotj1)

        V_dots.append(j1V_dotj1)
        O_dots.append(j1O_dotj1)

        jV_dotj, jO_dotj = j1V_dotj1, j1O_dotj1

    # --- Outputs (end-effector = last frame) ---
    print("\n=== T_sim ( ^0_n T ) ===")
    sp.pprint(T_sim)

    print("\n=== End-effector rotation R_for_J ( ^0_n R ) ===")
    sp.pprint(R_for_J)

    print("\n=== End-effector linear velocity (in frame n) Vs[n] ===")
    sp.pprint(Vs[-1])

    print("\n=== End-effector angular velocity (in frame n) Os[n] ===")
    sp.pprint(Os[-1])

    print("\n=== End-effector linear acceleration (in frame n) V_dots[n] ===")
    sp.pprint(V_dots[-1])

    print("\n=== End-effector angular acceleration (in frame n) O_dots[n] ===")
    sp.pprint(O_dots[-1])

if __name__ == "__main__":
    main()
