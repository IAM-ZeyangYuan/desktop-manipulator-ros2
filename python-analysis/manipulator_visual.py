import numpy as np
import pyvista as pv

from pathlib import Path
# =========================
# USER-EDITABLE VISUAL PARAMS 
# =========================
SCALE = 1.0  # units are cm

# Links
LINK_RADIUS = 0.7 * SCALE

# Revolute joint visual markers (pin + hub)
JOINT_HUB_RADIUS = 0.8 * SCALE
JOINT_HUB_LEN = 2.0 * SCALE
PIN_RADIUS = 0.25 * SCALE
PIN_LEN = JOINT_HUB_LEN * 1.001

# Prismatic telescope (coaxial with prismatic axis)
TELES_OUTER_RADIUS = 1.30 * SCALE
TELES_INNER_RADIUS = 0.6 * SCALE
TELES_COLLAR_LEN = 2.00 * SCALE
TELES_OUTER_LEN = 15.0 * SCALE
TELES_WALL_THICK = TELES_OUTER_RADIUS - TELES_INNER_RADIUS

# Rendering
CYL_RES = 64
SMOOTH_SHADING = True

# Coordinate triads
SHOW_FRAMES = True
FRAME_SCALE = 5.0 * SCALE
SHOW_ONLY_BASE_AND_EE = False

# Debug point spheres
SHOW_DEBUG_POINT_SPHERES = True
BALL_RADIUS = 1.0 * SCALE

# Camera preset (matched to trajectory_animation.py)
USE_FIXED_CAMERA = True
_z_focus = 30 * SCALE
_dist = 220 * SCALE
_cam_pos = (0.35 * _dist, -1.0 * _dist, 0.55 * _dist)
FIXED_CAMERA_POSITION = [_cam_pos, (0, 0, _z_focus), (0, 0, 1)]
FIXED_CAMERA_POSITION = [(59.84855053025886, -162.22119009525784, 148.47003353255485),
 (0.0, 0.0, 28.38278404226783),
 (-0.1975162417316588, 0.5351454565994735, 0.8213444311207948)]

# Colors
COLOR_TELES_OUTER = "orange"
COLOR_REV_HUB = "purple"

# Per-joint wraparound height offsets along local +z (cm)
THETA_Z_LIFT = {
    1: 3 * SCALE,   # θ1
    2: 7.5 * SCALE,   # θ2
    3: 4 * SCALE,   # θ3
}

# =========================


def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("Zero-length vector encountered while normalizing.")
    return v / n


def cylinder_between(p0, p1, radius, res=48) -> pv.PolyData:
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    v = p1 - p0
    L = float(np.linalg.norm(v))
    if L < 1e-9:
        v = np.array([0.0, 0.0, 1.0])
        L = 1e-6
    direction = v / np.linalg.norm(v)
    center = (p0 + p1) / 2.0
    return pv.Cylinder(center=center, direction=direction, radius=radius, height=L, resolution=res)


def theta_wrap_arrow(center: np.ndarray,
                     x_axis: np.ndarray,
                     y_axis: np.ndarray,
                     z_axis: np.ndarray,
                     radius: float,
                     z_offset: float,
                     t_start: float = 0.2 * np.pi,
                     t_end: float = 1.4 * np.pi,
                     n: int = 64,
                     v_opening_angle: float = np.deg2rad(55.0),
                     v_size: float = 0.55):
 
    c = np.asarray(center, float) + z_offset * unit(np.asarray(z_axis, float))
    x = unit(np.asarray(x_axis, float))
    y = unit(np.asarray(y_axis, float))
    z = unit(np.asarray(z_axis, float))

    ts = np.linspace(t_start, t_end, n)
    pts = np.array([c + radius * (np.cos(t) * x + np.sin(t) * y) for t in ts])

    # Arc polyline
    arc = pv.PolyData(pts)
    arc.lines = np.hstack(([n], np.arange(n))).astype(np.int64)

    # Tangent direction at end of arc (increasing angle)
    t = t_end
    tangent = unit(-np.sin(t) * x + np.cos(t) * y)
    tip = pts[-1]

    # V chevron at tip; point it opposite tangent so it reads like an arrowhead
    d = -tangent
    a = 0.5 * v_opening_angle

    def rot_about_z(v, ang):
        # Rodrigues rotation about z
        return (v * np.cos(ang) +
                np.cross(z, v) * np.sin(ang) +
                z * np.dot(z, v) * (1.0 - np.cos(ang)))

    arm1 = unit(rot_about_z(d, +a))
    arm2 = unit(rot_about_z(d, -a))

    L = (v_size * radius)
    p0 = tip
    p1 = tip + L * arm1
    p2 = tip + L * arm2

    vpoly = pv.PolyData(np.vstack([p1, p0, p2]))
    vpoly.lines = np.array([3, 0, 1, 2], dtype=np.int64)

    return arc, vpoly


def make_frame_triads(T: np.ndarray, scale: float):
    o = T[:3, 3]
    R = T[:3, :3]

    tip_length = 0.25
    tip_radius = 0.025 * scale
    shaft_radius = 0.010 * scale

    x_arrow = pv.Arrow(start=o, direction=R[:, 0],
                       tip_length=tip_length, tip_radius=tip_radius,
                       shaft_radius=shaft_radius, scale=scale)
    y_arrow = pv.Arrow(start=o, direction=R[:, 1],
                       tip_length=tip_length, tip_radius=tip_radius,
                       shaft_radius=shaft_radius, scale=scale)
    z_arrow = pv.Arrow(start=o, direction=R[:, 2],
                       tip_length=tip_length, tip_radius=tip_radius,
                       shaft_radius=shaft_radius, scale=scale)

    return [(x_arrow, "red"), (y_arrow, "green"), (z_arrow, "blue")]


def build_meshes_and_frames(theta1, theta2, theta3, d4):
    # Fixed params (cm)
    L1 = 35
    L2 = 25
    L3 = 2
    L4 = 10

    # Transforms (as given)
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

    # Cumulative
    T00 = np.eye(4)
    T02 = T01 @ T12
    T03 = T02 @ T23

    # Origins
    p0 = T00[:3, 3]
    p1 = T01[:3, 3]
    p2 = T02[:3, 3]
    p3 = T03[:3, 3]

    R03 = T03[:3, :3]
    i3 = unit(R03[:, 0])
    j3 = unit(R03[:, 1])
    k3 = unit(R03[:, 2])

    p4_base = p3 + i3 * L4
    p4 = p4_base + k3 * d4

    p_mid = p4 - i3 * (L4 / 2.0)
    pe = p_mid + k3 * (L4 / 2.0)

    meshes = []

    meshes.append(("link01", cylinder_between(p0, p1, LINK_RADIUS, CYL_RES)))
    meshes.append(("link12", cylinder_between(p1, p2, LINK_RADIUS, CYL_RES)))
    meshes.append(("link23", cylinder_between(p2, p3, LINK_RADIUS, CYL_RES)))

    meshes.append(("L4_segment", cylinder_between(p3, p4_base, LINK_RADIUS, CYL_RES)))

    if SHOW_DEBUG_POINT_SPHERES:
        meshes.append(("ball_p4", pv.Sphere(radius=BALL_RADIUS, center=p4, theta_resolution=32, phi_resolution=32)))

    meshes.append(("ee_h", cylinder_between(p4, p_mid, TELES_INNER_RADIUS, CYL_RES)))
    meshes.append(("ee_v", cylinder_between(p_mid, pe, TELES_INNER_RADIUS, CYL_RES)))
    meshes.append(("ball_ee", pv.Sphere(radius=BALL_RADIUS, center=pe, theta_resolution=32, phi_resolution=32)))
    meshes.append(("ball_mid", pv.Sphere(radius=BALL_RADIUS, center=p_mid, theta_resolution=32, phi_resolution=32)))

    # Revolute joint markers + arc+V
    joint_frames = [T01, T02, T03]
    joint_points = [p1, p2, p3]

    for j, (Tj, pj) in enumerate(zip(joint_frames, joint_points), start=1):
        z_axis = unit(Tj[:3, 2])
        x_axis = unit(Tj[:3, 0])
        y_axis = unit(Tj[:3, 1])

        hub = pv.Cylinder(center=pj, direction=z_axis, radius=JOINT_HUB_RADIUS,
                          height=JOINT_HUB_LEN, resolution=CYL_RES)
        pin = pv.Cylinder(center=pj, direction=z_axis, radius=PIN_RADIUS,
                          height=PIN_LEN, resolution=CYL_RES)
        meshes.append((f"rev_hub_{j}", hub))
        meshes.append((f"rev_pin_{j}", pin))

        # Wrap-around arrow: smaller + lifted
        arc_r = 1.6 * SCALE
        z_lift = THETA_Z_LIFT.get(j, 1.4 * SCALE)  # fallback if missing
        
        arc, vpoly = theta_wrap_arrow(
            pj,
            x_axis,
            y_axis,
            z_axis,
            radius=arc_r,
            z_offset=z_lift
        )


        meshes.append((f"theta_arc_{j}", arc))
        meshes.append((f"theta_v_{j}", vpoly))  

    # Prismatic telescope
    outer = pv.Cylinder(center=p4_base, direction=k3,
                        radius=TELES_OUTER_RADIUS, height=TELES_COLLAR_LEN * 1.001, resolution=CYL_RES)
    meshes.append(("pris_outer", outer))

    collar = pv.Cylinder(center=p4_base, direction=k3,
                         radius=1.1 * TELES_OUTER_RADIUS * 1.05, height=TELES_COLLAR_LEN, resolution=CYL_RES)
    meshes.append(("pris_collar", collar))

    inner_len = 15
    inner_center = p4_base + k3 * (d4 - inner_len * 0.5)
    inner = pv.Cylinder(center=inner_center, direction=k3,
                        radius=TELES_INNER_RADIUS, height=inner_len, resolution=CYL_RES)
    meshes.append(("pris_inner", inner))

    # Frames to show
    T03_base = np.eye(4)
    T03_base[:3, :3] = R03
    T03_base[:3, 3] = p4_base

    Te = np.eye(4)
    Te[:3, :3] = R03
    Te[:3, 3] = pe

    frames = [T00, T01, T02, T03, T03_base, Te]
    return meshes, frames


def main():
    theta1 = 0.0
    theta2 = -np.pi/2
    theta3 = np.pi / 2 
    d4 = 15
    meshes, frames = build_meshes_and_frames(theta1, theta2, theta3, d4)

    plotter = pv.Plotter()
    plotter.add_axes()

    ROBOT_OPACITY = 0.6

    for name, mesh in meshes:
        # Arc + V: identical style
        if name.startswith("theta_arc_") or name.startswith("theta_v_"):
            plotter.add_mesh(mesh, color="black", line_width=5, render_lines_as_tubes=True,opacity = 0)
            continue

        color = None
        if name == "link12" or name == "pris_inner":
            color = "red"
            # Increase shininess and reflection
            plotter.add_mesh(mesh, smooth_shading=SMOOTH_SHADING, color=color, opacity=0.28, 
                             specular=1.0, specular_power=28, 
                             ambient=0.3, diffuse=0.1)#, reflective=True)
        elif name == "pris_outer":  
            color = COLOR_TELES_OUTER
            plotter.add_mesh(mesh, smooth_shading=SMOOTH_SHADING, color=color, opacity=1)
        elif name.startswith("rev_hub_"):
            color = COLOR_REV_HUB
            plotter.add_mesh(mesh, smooth_shading=SMOOTH_SHADING, color=color, opacity=1)
            
        else:
            plotter.add_mesh(mesh, smooth_shading=SMOOTH_SHADING, color=color, opacity=1)

    # Frame arrows visible through robot
    triad_actors = []
    if SHOW_FRAMES:
        idxs = [0, len(frames) - 1] if SHOW_ONLY_BASE_AND_EE else list(range(len(frames)))
        for i in idxs:
            for triad_mesh, triad_color in make_frame_triads(frames[i], scale=FRAME_SCALE):
                a = plotter.add_mesh(triad_mesh, color=triad_color)
                triad_actors.append(a)

    for a in triad_actors:
        try:
            vtk_actor = a.actor if hasattr(a, "actor") else a
            vtk_actor.GetProperty().SetDepthTest(False)
        except Exception:
            pass



    
    
    
    
    save_path = Path(__file__).parent / "manipulator_visual.png"



    def save_shot():
        plotter.screenshot(str(save_path))
        print("Saved")

    dis = 0.67

    plotter.add_key_event("s", save_shot)   # press 's' to save
    print("Camera position:", plotter.camera_position)
    plotter.camera_position = [(42.66913371927163, -93.83761353138125, 94.49837548612774),
      (13.89282032302755, 1.25, 25.798715591430664),
      (-0.13016905334550416, 0.5544656948626487, 0.8219633877318475)]
    plotter.show(window_size=(800, 800))


if __name__ == "__main__":
    main()
