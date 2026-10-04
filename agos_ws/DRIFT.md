# DRIFT.md — AGOS Crawler: PVC Pipe Simulation

> Instructions and project context for Drift AI. Read this file before changing anything in the repository. Where this file conflicts with an assumption, this file wins. Open conflicts are listed in [Section 6](#6-known-issues--open-items).

---

## 1. Project Overview

- **AGOS** is a small continuous-track crawler robot built for pipe inspection and in-pipe crawling.
- The simulation environment places AGOS inside **PVC pipe obstacles** and supports:
  - navigation through straight pipe sections at configurable inclines,
  - real-time tracking (odometry, TF tree, IMU, camera, joint states),
  - repeatable test runs for validating locomotion and state estimation.
- Target stack (change here first if the stack changes):
  - ROS 2 **Jazzy** + Gazebo **Harmonic** (`gz sim`) via `ros_gz_sim` / `ros_gz_bridge`
  - Robot description in **URDF/Xacro** (an MJCF port is optional, see Section 5)

---

## 2. Robot Specifications (AGOS)

| Property | Value |
|---|---|
| Type | Continuous-track crawler (differential drive) |
| Nominal footprint | ~0.20 m (L) x 0.15 m (W) x 0.10 m (H) — **see Section 6, Issue 1** |
| Mass | ~2.5 kg |
| Actuators | 2 x `continuous` joints: `left_track_joint`, `right_track_joint` |
| Drive model | Differential: left/right track velocity commands |

### 2.1 Actuators

- `left_track_joint` and `right_track_joint` are `continuous` joints about the lateral axis (y).
- Command interface: velocity. Subscribe to `/cmd_vel` (`geometry_msgs/Twist`) and convert to left/right track velocities.
- Tracks are modelled as **driven road-wheel sets with high-friction contact** by default (cheap and stable). A full link-per-segment track is out of scope unless explicitly requested.

### 2.2 CAD Assets

Reference meshes live in `./meshes/`:

- `agos_chassis.stl`
- `track.stl`

Mesh rules:

- Meshes are **visual references**. Collision geometry must be simplified (boxes/cylinders, or a decimated convex hull), never the raw CAD.
- Check mesh units before use. STL exports from CAD are usually **millimetres**, so reference them with `scale="0.001 0.001 0.001"` unless verified otherwise.
- Keep each simulation mesh under ~50k triangles.

### 2.3 Frames

| Frame | Purpose |
|---|---|
| `base_footprint` | Ground-projected origin (z = 0 on the pipe floor contact), child of `odom` |
| `base_link` | Robot body origin, fixed child of `base_footprint` |
| `chassis_link` | Chassis visual/collision/inertial body, fixed to `base_link` |
| `camera_link` | Front-facing optical camera mount, fixed to `chassis_link` |
| `imu_link` | 9-DOF IMU mount, fixed to `chassis_link` (near the centre of mass) |

Use REP-103 conventions: x forward, y left, z up. If an optical frame is needed, add `camera_optical_frame` with the standard rotation from `camera_link`.

---

## 3. Environment & PVC Pipe Obstacles

### 3.1 Pipe Specs

| Property | Value |
|---|---|
| Geometry | Hollow cylinder |
| Inner diameter | 0.15 m (6 in) |
| Length | 2.0 m |
| Wall thickness | 0.005 m (outer diameter = 0.16 m) |
| Pipe axis | World x-axis at 0° incline |

### 3.2 Surface Physics

- Material contact: PVC plastic.
- Static and dynamic friction coefficient: **0.4** (`mu = mu2 = 0.4`).
- Apply the same friction to the pipe inner wall and to the track/wheel contact surfaces, and record the combined behaviour you observe. Do not silently retune friction to make a test pass.

### 3.3 Incline

- Configurable pipe pitch from **0° to 15°**.
- Expose it as a launch argument: `incline_deg:=0.0` (validate the range 0–15 and reject values outside it).
- Rotate the pipe model about its entry end so that the spawn pose stays consistent across inclines.

### 3.4 Collision (important)

- The pipe collision must be a **hollow mesh** (or a set of box/cylinder-segment primitives forming a ring), **not** a solid cylinder.
- A solid cylinder collision would make the robot stand on or inside the wall and silently break every test.
- Verify by spawning the robot inside the pipe and confirming that it rests on the inner floor at the expected height.
- The pipe is static, so a concave trimesh collision is acceptable. Never use a concave mesh on a moving link.

---

## 4. Tracking & Telemetry Specs

### 4.1 Sensors and Topics

| Sensor | Topic | Message type | Notes |
|---|---|---|---|
| Front-facing optical camera | `/camera/image_raw` | `sensor_msgs/Image` | Add `/camera/camera_info`. Mount at `camera_link`. |
| 9-DOF IMU | `/imu/data` | `sensor_msgs/Imu` | Gyro + accel + orientation. Magnetometer, if simulated, goes on `/imu/mag` (`sensor_msgs/MagneticField`). Frame: `imu_link`. |
| Track encoders | `/joint_states` | `sensor_msgs/JointState` | Positions and velocities for both track joints. |

### 4.2 Odometry and Ground Truth

- Publish fused visual/wheel odometry to **`/odom`** (`nav_msgs/Odometry`).
  - Suggested fusion: `robot_localization` EKF combining encoder-derived odometry, IMU, and visual odometry (if available).
- Publish the simulator's true pose separately to **`/ground_truth/odom`**. Keep it distinct from `/odom` so estimation error can be measured.
- Dynamic TF tree:

```
odom
 └── base_footprint
      └── base_link
           └── chassis_link
                ├── camera_link
                └── imu_link
```

- `odom -> base_footprint` is dynamic and must have **exactly one publisher**. If the EKF publishes it, disable TF publishing in the diff-drive plugin.
- `base_footprint -> base_link` and everything below are published by `robot_state_publisher` from the URDF.
- Set `use_sim_time:=true` on every node.

---

## 5. Instructions for Drift AI

### 5.1 Repository Layout

```
agos_sim/
├── DRIFT.md
├── urdf/                # or ./description/ — pick one and stay consistent
│   ├── agos.urdf.xacro
│   ├── agos_materials.xacro
│   ├── agos_tracks.xacro
│   └── agos_sensors.xacro
├── meshes/
│   ├── agos_chassis.stl
│   └── track.stl
├── worlds/
│   └── pvc_pipe.sdf
├── config/
│   ├── bridge.yaml
│   └── ekf.yaml
└── launch/
    └── sim.launch.py
```

### 5.2 Rules

1. Keep the URDF/Xacro (or MJCF) **clean and modular** inside `./urdf/` or `./description/`. Split chassis, tracks, sensors, and materials into separate included files. Use Xacro properties for every dimension, mass, and friction value. No magic numbers inline.
2. Maintain **accurate collision boundaries inside the pipe**: hollow pipe collision, simplified robot collision, and no collision geometry that overlaps the pipe wall at spawn.
3. Compute inertia from mass and dimensions (or from the CAD with a tool such as MeshLab). Do not leave placeholder inertia values.
4. Use the exact topic and frame names in this file. Do not rename them without updating this file.
5. When editing a file, return the **complete corrected file**, not a partial diff.
6. After any change to geometry, run these checks and report the results:
   - `check_urdf` / `gz sdf --check` passes,
   - the robot spawns inside the pipe without ejecting or sinking,
   - `ros2 run tf2_tools view_frames` shows the tree in Section 4.2,
   - all topics in Section 4.1 publish at a sensible rate.
7. If the request conflicts with this file or the physical geometry, say so before implementing it.

### 5.3 Suggested Test Matrix

| Test | Incline | Expected |
|---|---|---|
| Straight traverse | 0° | Reaches the far end, with small lateral drift |
| Incline climb | 5°, 10°, 15° | Climbs without slipping, or log slip onset |
| Hold on incline | 15° | Stationary when the command is zero |
| Odometry error | all | `/odom` vs `/ground_truth/odom` drift logged |

---

## 6. Known Issues & Open Items

### Issue 1 — Robot width does not fit the pipe

The nominal width (0.15 m) equals the pipe inner diameter (0.15 m). A track-width body cannot fit in a circular bore: the usable floor width shrinks quickly towards the pipe bottom (about 0.12 m at 3 cm above the lowest point, less lower down), and the top corners of a 0.10 m tall box have only ~0.14 m of width available.

**Action:** Either enlarge the pipe or reduce AGOS. For the stated 0.15 m inner diameter, plan on a robot width of roughly 0.09 m or less at the track contact, and verify with the spawn test in Section 5.2. Do not "fix" it by shrinking collision geometry below the visual geometry without recording that decision here.

### Issue 2 — Uploaded CAD does not match the nominal dimensions

`AGOS_V1.stl` has these properties:

- bounding box about 381 x 1397 x 476 (in file units; millimetres would give ~0.38 x 1.40 x 0.48 m),
- about 10 million triangles, ~500 MB.

That is far larger than the ~0.2 x 0.15 x 0.1 m specification and far too heavy for simulation. It may be a full assembly, a different scale, or a different (e.g. drain-scale) version of the robot.

**Action:** Confirm the intended scale, split the CAD into `agos_chassis.stl` and `track.stl`, and decimate each to under ~50k triangles before putting them in `./meshes/`.

### Issue 3 — Track modelling

Real tracked locomotion is expensive and unstable in rigid-body simulators. The default here is a wheel-set approximation with tuned friction. If track-segment dynamics are needed later, record it as a separate task.
