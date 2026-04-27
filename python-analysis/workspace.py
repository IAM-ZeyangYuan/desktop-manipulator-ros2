import numpy as np
import pyvista as pv
import os

# --- Import your robot visuals (must be in same directory) ---
# Uses: build_meshes_and_frames(...) and make_frame_triads(...) from your file. :contentReference[oaicite:1]{index=1}
import sys
from pathlib import Path

# add this file's directory to Python path
THIS_DIR = Path(__file__).resolve().parent
sys.path.append(str(THIS_DIR))

import manipulator_visual as av
from manipulator_visual import build_meshes_and_frames, make_frame_triads




# =========================
# Workspace sampling params
# =========================
theta1_range = (-np.pi, np.pi)
theta2_range = (-np.pi, 0)
theta3_range = (-np.pi,np.pi)
d4_range     = (5,15)

N = 35_00 #350_000 increase to reduce pitting; 100k–500k typical
SEED = 0

# Alpha shape param (units = your link units, likely cm)
# Smaller: tighter but can get bumpy/fragmented; Larger: smoother but can close donut tunnel.
ALPHA = 7.5

# Smoothing (Taubin: less shrink than Laplacian)
TAUBIN_ITERS = 400
TAUBIN_PASSBAND = 0.035

# =========================
# Robot pose to overlay
# =========================
ROBOT_THETA1 = 0.0
ROBOT_THETA2 = -np.pi / 2
ROBOT_THETA3 = np.pi / 2
ROBOT_D4     = 5.0

ROBOT_OPACITY = 0.99
WORKSPACE_OPACITY = 0.25


# -----------------------------
# FK position (same chain as your original workspace code)
# -----------------------------
L1 = 35
L2 = 25
L3 = 2
L4 = 10

def fk_position(theta1, theta2, theta3, d4):
    T01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                    [np.sin(theta1),  np.cos(theta1), 0, 0],
                    [0,               0,              1, L1],
                    [0,               0,              0, 1]])

    T12 = np.array([[ np.cos(theta2), -np.sin(theta2), 0, L2],
                    [ 0,               0,              1, 0 ],
                    [-np.sin(theta2), -np.cos(theta2), 0, 0 ],
                    [ 0,               0,              0, 1 ]])

    T23 = np.array([[ np.cos(theta3), -np.sin(theta3), 0, 0 ],
                    [ 0,               0,              1, L3],
                    [-np.sin(theta3), -np.cos(theta3), 0, 0 ],
                    [ 0,               0,              0, 1 ]])

    T34 = np.array([[1, 0, 0, L4],
                    [0, 1, 0, 0 ],
                    [0, 0, 1, d4],
                    [0, 0, 0, 1 ]])

    T4e = np.array([[1, 0, 0, -L4/2],
                    [0, 1, 0, 0    ],
                    [0, 0, 1,  L4/2],
                    [0, 0, 0, 1    ]])

    T0e = T01 @ T12 @ T23 @ T34 @ T4e
    return T0e[:3, 3]


#joint sweep curves baby
def joint_sweep_curve(
    joint_index,
    joint_range,
    fixed_pose,
    n=300
):
    """
    joint_index: 0=theta1, 1=theta2, 2=theta3, 3=d4
    joint_range: (min, max)
    fixed_pose: (theta1, theta2, theta3, d4)
    """
    t = np.linspace(joint_range[0], joint_range[1], n)

    curve = np.zeros((n, 3))
    for i, val in enumerate(t):
        pose = list(fixed_pose)
        pose[joint_index] = val
        curve[i] = fk_position(*pose)

    return curve



# -----------------------------
# Alpha shape surface
# -----------------------------
def alpha_surface(cloud: pv.PolyData, alpha: float) -> pv.PolyData:
    ug = cloud.delaunay_3d(alpha=alpha)

    # Extract boundary surface as PolyData (renderable)
    surf = ug.extract_surface().clean(tolerance=1e-8)

    if surf.n_points == 0 or surf.n_cells == 0:
        return surf

    # Keep only the largest component (removes scraps)
    surf = surf.connectivity(extraction_mode="largest").clean(tolerance=1e-8)

    # Smooth for visual quality (keeps donut topology; no hole filling)
    surf = surf.smooth_taubin(n_iter=TAUBIN_ITERS, pass_band=TAUBIN_PASSBAND)

    # Normals for smooth shading (prevent triangle-edge normal splitting)
    surf = surf.compute_normals(
        auto_orient_normals=True,
        consistent_normals=True,
        split_vertices=False
    )
    return surf

# for calculatign the workspace volumne
def alpha_volume_from_tets(cloud: pv.PolyData, alpha: float) -> float:
    # 1) Alpha-filtered tetrahedralization (UnstructuredGrid)
    ug = cloud.delaunay_3d(alpha=alpha)

    if ug.n_cells == 0:
        return 0.0

    # 2) (Optional but often helpful) keep largest connected region of the tetra mesh
    # This removes isolated scraps that can inflate/garble the volume.
    ug = ug.connectivity(extraction_mode="largest")

    # 3) Compute per-cell volumes and sum
    ug_sizes = ug.compute_cell_sizes(length=False, area=False, volume=True)
    return float(ug_sizes.cell_data["Volume"].sum())







def add_base_frame_grid(
    plotter: pv.Plotter,
    size=120.0,
    spacing=10.0,
    z=0.0,
    color="lightgray"
):
    half = size / 2.0

    for x in np.arange(-half, half + spacing, spacing):
        plotter.add_lines(
            np.array([[x, -half, z], [x, half, z]]),
            color=color,
            width=1
        )

    for y in np.arange(-half, half + spacing, spacing):
        plotter.add_lines(
            np.array([[-half, y, z], [half, y, z]]),
            color=color,
            width=1
        )









def main():
    rng = np.random.default_rng(SEED)

    # ---- Sample workspace point cloud ----
    t1 = rng.uniform(*theta1_range, N)
    t2 = rng.uniform(*theta2_range, N)
    t3 = rng.uniform(*theta3_range, N)
    d4 = rng.uniform(*d4_range, N)

    points = np.empty((N, 3), dtype=np.float64)
    for i in range(N):
        points[i] = fk_position(t1[i], t2[i], t3[i], d4[i])

    cloud = pv.PolyData(points).clean(tolerance=1e-8)
    print("Point cloud:", cloud.n_points, "points")
    
    V = alpha_volume_from_tets(cloud, ALPHA)
    print(f"Alpha-tet workspace volume = {V:.3f} (units^3)")
    
    R_max = np.linalg.norm(points, axis=1).max()
    print(f"Max reach from base = {R_max:.3f} (units)")
    
    ratio_VR = V/(4/3*np.pi*R_max**3)
    print(f"Ratio = {ratio_VR:.3f} (no units)")
    

    # ---- Build workspace surface ----
    ws = alpha_surface(cloud, ALPHA)
    if ws.n_cells == 0:
        raise RuntimeError("Workspace surface is empty. Increase ALPHA and/or N.")
    print("Workspace surface:", ws.n_points, "verts,", ws.n_cells, "cells; alpha =", ALPHA)

    # ---- Build robot meshes + frames (your existing function) ----
    # This returns meshes and frame transforms; base is T00 = I at world origin. :contentReference[oaicite:2]{index=2}
    robot_meshes, robot_frames = build_meshes_and_frames(
        ROBOT_THETA1, ROBOT_THETA2, ROBOT_THETA3, ROBOT_D4
    )
    # Frames list includes base and EE frames (and intermediate ones). :contentReference[oaicite:3]{index=3}
    
    
    
    
    
    
    # Current robot pose (same as arm visualization)
    fixed_pose = (
        ROBOT_THETA1,
        ROBOT_THETA2,
        ROBOT_THETA3,
        ROBOT_D4
    )
    
    # Joint ranges (same ones you used for workspace)
    joint_ranges = [
        theta1_range,
        theta2_range,
        theta3_range,
        d4_range
    ]
    
    curves = [
        joint_sweep_curve(0, joint_ranges[0], fixed_pose),  # theta1
        joint_sweep_curve(1, joint_ranges[1], fixed_pose),  # theta2
        joint_sweep_curve(2, joint_ranges[2], fixed_pose),  # theta3
        joint_sweep_curve(3, joint_ranges[3], fixed_pose),  # d4
    ]
    
        
        
        
        
        
    
    
    
    # ---- Plot ----
    p = pv.Plotter()

    # Axes widget (PyVista axes in the scene) :contentReference[oaicite:4]{index=4}
    p.add_axes()

        
    # Base-frame grid (XY plane of robot base)
    add_base_frame_grid(
        p,
        size=160.0,     # adjust to cover workspace footprint
        spacing=10.0,
        z=0.0
    )
    
    





    # Add workspace surface first (transparent “envelope”)
    p.add_mesh(
        ws,
        color='#000000',
        opacity=WORKSPACE_OPACITY,
        smooth_shading=True,
        specular=0.05,
        specular_power=8,
        show_edges=False,
        show_scalar_bar=False
    )


    for name, mesh in robot_meshes:
        color = None
        if name == "pris_outer":
            color = av.COLOR_TELES_OUTER
        elif name.startswith("rev_hub_"):
            color = av.COLOR_REV_HUB
    
        p.add_mesh(
            mesh,
            smooth_shading=getattr(av, "SMOOTH_SHADING", True),
            color=color,                 # <-- this is what arm_visual.py does
            opacity=ROBOT_OPACITY,
            show_edges=False,
            show_scalar_bar=False
        )


    # Add explicit frame triads so you can visually confirm coincidence
    # Base frame is robot_frames[0] == T00 (identity at origin). :contentReference[oaicite:5]{index=5}
    # ---- Base frame triads (ensure always visible) ----
    base_T = robot_frames[0]
    
    triad_actors = []
    for triad_mesh, triad_color in make_frame_triads(base_T, scale=5):
        actor = p.add_mesh(triad_mesh, color=triad_color)
        triad_actors.append(actor)
    
    # Disable depth testing so frame is not hidden by robot/workspace
    for a in triad_actors:
        try:
            vtk_actor = a.actor if hasattr(a, "actor") else a
            vtk_actor.GetProperty().SetDepthTest(False)
        except Exception:
            pass

        

    # # (Optional) show end-effector frame too
    # ee_T = robot_frames[-1]
    # for triad_mesh, triad_color in make_frame_triads(ee_T, scale=7.0):
    #     p.add_mesh(triad_mesh, color=triad_color)


    curve_colors = [
        "green",        # theta1
        "green",      # theta2
        "green",       # theta3
        "green"      # d4
    ]
    
    curve_labels = [
        "θ1 sweep",
        "θ2 sweep",
        "θ3 sweep",
        "d4 sweep"
    ]
    
    for curve, color, label in zip(curves, curve_colors, curve_labels):
        poly = pv.PolyData(curve)
        poly.lines = np.hstack([[curve.shape[0]] + list(range(curve.shape[0]))])
        p.add_mesh(
            poly,
            color=color,
            line_width=2.5,
            label=label
        )






    save_path = Path(__file__).parent / "workspace_sweep.png"



    def save_shot():
        p.screenshot(str(save_path))
        print("Saved workspace_half.png")




    p.view_isometric()
    p.enable_depth_peeling(number_of_peels=8)
    

    p.add_key_event("s", save_shot)   # press 's' to save
    print("Camera position:", p.camera_position)
    p.camera_position = [(59.84855053025886, -162.22119009525784, 148.47003353255485),
     (0.0, 0.0, 28.38278404226783),
     (-0.1975162417316588, 0.5351454565994735, 0.8213444311207948)]
    p.show(window_size=(800, 800))


if __name__ == "__main__":
    main()