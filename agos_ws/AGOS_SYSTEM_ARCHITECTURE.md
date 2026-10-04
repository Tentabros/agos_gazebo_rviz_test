# AGOS — System Architecture & Operational Methodology

Continuous-track crawler for in-pipe inspection. This document details the
distributed hardware/software architecture, sensor payload integration, and
data flow across the tethered control link, building on the specifications
already fixed in `DRIFT.md` (frames, topics, mass/footprint, pipe environment).

---

## 1. Architecture Overview — Three-Tier Compute Model

AGOS splits compute across three tiers connected by a single tethered Cat6
Ethernet run between the crawler and the operator position. The tether carries
both power conductors (separate from data pairs) and the Ethernet data link —
this document covers the data/compute side only.

```
┌───────────────────────────┐        Cat6 tether        ┌──────────────────────────┐
│   ESP32 micro-ROS Nodes   │  UART/USB (on-board link)  │                          │
│  (motor + encoder tier)   │ ─────────────────────────▶ │   Raspberry Pi 5         │
│                           │                             │   (edge compute tier)    │
└───────────────────────────┘                             │                          │
                                                           │  - sensor drivers        │
                                                           │  - TF + odometry fusion  │
                                                           │  - DDS participant       │
                                                           └────────────┬─────────────┘
                                                                        │ Ethernet (Cat6)
                                                                        │ ROS 2 DDS (RTPS/UDP)
                                                                        ▼
                                                           ┌──────────────────────────┐
                                                           │     Control Station      │
                                                           │  (operator / offload)    │
                                                           │  - RViz2 visualization   │
                                                           │  - heavy perception      │
                                                           │  - teleop + mission mgmt │
                                                           │  - logging / rosbag      │
                                                           └──────────────────────────┘
```

| Tier | Hardware | Role | OS / Runtime |
|---|---|---|---|
| Actuation tier | 2x ESP32 (micro-ROS) | Closed-loop track velocity control, encoder sampling | FreeRTOS + micro-ROS |
| Edge compute tier | Raspberry Pi 5 | Sensor drivers, TF tree, state estimation, DDS bridging | Ubuntu 24.04 + ROS 2 Jazzy |
| Operator tier | Control Station (laptop/workstation) | Visualization, heavy perception, teleop, mission control, data logging | ROS 2 Jazzy (same DDS domain) |

### Why this split
- The ESP32s own the hard-real-time loop (PID velocity control, encoder
  edge-counting) that cannot tolerate OS jitter — this stays off the Pi 5's
  general-purpose scheduler.
- The Pi 5 owns everything that must run **on the crawler** regardless of
  tether health: sensor acquisition, the static/dynamic TF publication chain,
  and local odometry fusion, so the robot's internal state estimate does not
  depend on the Control Station being reachable.
- The Control Station owns everything that is **heavy, operator-facing, or
  safely deferrable**: RViz, mapping/registration on the depth stream,
  recording, and the human-in-the-loop teleop interface. Offloading these
  keeps the Pi 5's CPU/thermal budget free for the sensor and TF pipeline.

---

## 2. Hardware / Software Breakdown by Tier

### 2.1 Actuation Tier — ESP32 micro-ROS Nodes

Two ESP32 modules, each responsible for one track side (left / right), each
running a micro-ROS node transported over a micro-ROS agent hosted on the
Pi 5 (serial or UDP transport, per board wiring).

| Node | Subscribes | Publishes | Function |
|---|---|---|---|
| `left_track_node` | `/agos/track/left/cmd_vel` (`std_msgs/Float32`) | `/agos/track/left/encoder` (`std_msgs/Int32` or custom), rolled into `/joint_states` on the Pi 5 | PID velocity loop driving the left track motor driver, quadrature encoder sampling |
| `right_track_node` | `/agos/track/right/cmd_vel` | `/agos/track/right/encoder` | Same, right side |

- Per-wheel command/encoder topics are internal to the actuation tier and are
  **not** the public interface — `DRIFT.md` fixes the public contract as a
  single `geometry_msgs/Twist` on `/cmd_vel` and a fused `sensor_msgs/JointState`
  on `/joint_states`. A differential-drive mixer node on the Pi 5 converts
  `/cmd_vel` → left/right wheel velocity setpoints, and re-assembles the two
  encoder streams into the single `/joint_states` message with entries for
  `left_track_joint` and `right_track_joint` (matching the `continuous` joints
  defined in the URDF).
- micro-ROS keeps the ESP32 nodes as first-class DDS participants over the
  Pi 5's micro-ROS agent, so the mixer/fusion node on the Pi 5 can treat them
  like any other ROS 2 publisher/subscriber — no custom serial protocol needs
  to leak above the agent boundary.
- The ESP32 tier does not know about TF, frames, or the pipe environment —
  it only sees velocity setpoints and raw encoder counts. This keeps the
  hard-real-time code small and independent of the rest of the stack.

### 2.2 Edge Compute Tier — Raspberry Pi 5

Runs the full ROS 2 Jazzy graph that owns the robot's local state:

| Responsibility | Node(s) | Notes |
|---|---|---|
| micro-ROS agent | `micro_ros_agent` | Bridges both ESP32 DDS participants onto the Pi 5's DDS domain |
| Differential drive mixer | `agos_drive_mixer` | `/cmd_vel` → per-track setpoints; merges encoder feedback → `/joint_states` |
| `robot_state_publisher` | — | Publishes `base_footprint → base_link → chassis_link → {camera_link, imu_link}` from the URDF (static/fixed joints) |
| RGB-D driver | Orbbec Gemini 335 vendor node (OpenNI2/Orbbec SDK ROS 2 wrapper) | Publishes `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info` |
| 2D LiDAR driver | Okdo LiDAR ROS 2 driver | Publishes `/scan` (`sensor_msgs/LaserScan`) |
| IMU driver | 9-DOF IMU driver node | Publishes `/imu/data`, `/imu/mag` |
| State estimation | `robot_localization` EKF node | Fuses encoder odometry + IMU (+ visual odometry if enabled) → `/odom`, broadcasts `odom → base_footprint` |
| Ground truth passthrough (sim only) | Gazebo plugin bridge | `/ground_truth/odom`, kept separate per `DRIFT.md` |
| DDS participant | (implicit — `rmw_fastrtps_cpp` or configured RMW) | Exposes every Pi 5 topic/service to the Control Station over the Ethernet tether |

The Pi 5 is the **single authority** for `odom → base_footprint` (the EKF
publishes it; TF publishing is disabled in the diff-drive-equivalent mixer to
avoid a duplicate authority, per `DRIFT.md`). Everything from
`base_footprint` down is published by `robot_state_publisher` from static
URDF joints.

### 2.3 Operator Tier — Control Station

Connected to the same ROS 2 DDS domain over the Cat6 tether (same subnet,
same `ROS_DOMAIN_ID`, no NAT between the two ends). Hosts anything that is
compute-heavy or operator-facing:

| Responsibility | Typical tooling | Why it's offloaded here |
|---|---|---|
| Visualization | RViz2 (TF tree, RGB-D point cloud, LaserScan, odometry path) | GUI compositing and large point-cloud rendering are unnecessary load on the Pi 5 |
| Heavy perception (optional) | Depth-based obstacle/defect detection, point-cloud registration, visual-odometry front end | CPU/GPU-bound work the Control Station's hardware handles far better than the Pi 5 |
| Mission / teleop control | Joystick/keyboard teleop node publishing `/cmd_vel`, waypoint or incline-test sequencing scripts | Human-in-the-loop command authority should not live on the tethered crawler |
| Logging | `ros2 bag record` of all topics in the sensor table + `/tf`, `/odom`, `/ground_truth/odom` | Storage and post-run analysis belong off the embedded platform |
| Diagnostics | `ros2 topic hz`, `ros2 doctor`, bandwidth monitors | Confirms the tether and DDS graph are healthy during a run |

If the depth stream is used for SLAM or defect-mapping rather than simple
display, that processing runs here too — the Pi 5 only drivers the sensor and
republishes raw/rectified data; it does not run registration or mapping
locally.

---

## 3. Sensor Payload Integration

All sensors mount on `chassis_link` per the frame tree in `DRIFT.md`,
extended here with the LiDAR frame the proposal adds:

```
odom
 └── base_footprint
      └── base_link
           └── chassis_link
                ├── camera_link ── camera_optical_frame   (Orbbec Gemini 335)
                ├── imu_link                               (9-DOF IMU)
                └── lidar_link                             (Okdo 2D LiDAR)
```

### 3.1 Orbbec Gemini 335 (RGB-D)

| Item | Value |
|---|---|
| Mount frame | `camera_link`, front-facing, fixed joint to `chassis_link` |
| Optical frame | `camera_optical_frame` — rotated `rpy="-1.5708 0 -1.5708"` from `camera_link` (REP-103 body axes → camera optical axes) |
| Topics | `/camera/image_raw` (RGB), `/camera/depth/image_raw`, `/camera/camera_info`, optionally `/camera/points` (registered point cloud) |
| Driver location | Pi 5 (USB3 to the Gemini 335) |
| Consumption | RViz2 / perception pipeline on the Control Station |
| Integration note | Depth and color `camera_info` must both be bridged/published — downstream point-cloud and rectification consumers fail silently without intrinsics |

### 3.2 Okdo 2D LiDAR

| Item | Value |
|---|---|
| Mount frame | `lidar_link`, fixed joint to `chassis_link`, oriented to scan the plane ahead of the crawler inside the pipe bore |
| Topic | `/scan` (`sensor_msgs/LaserScan`) |
| Driver location | Pi 5 (serial/USB to the LiDAR) |
| Use | Forward obstacle/defect range profile inside the pipe; feeds any future Nav2-style obstacle layer or manual operator range display |
| Integration note | `frame_id` in the published scan must be `lidar_link` to match the TF tree — a mismatched frame silently drops the message in any TF-aware consumer |

### 3.3 9-DOF IMU

| Item | Value |
|---|---|
| Mount frame | `imu_link`, fixed to `chassis_link`, placed near the robot's centre of mass per `DRIFT.md` |
| Topics | `/imu/data` (`sensor_msgs/Imu` — gyro, accel, orientation), `/imu/mag` (`sensor_msgs/MagneticField`) |
| Driver location | Pi 5 |
| Use | Primary input (with encoders) to the EKF producing `/odom`; critical on inclined pipe sections (5°–15°) where wheel-only odometry slips |

### 3.4 Wheel/Track Encoders

| Item | Value |
|---|---|
| Source | Quadrature encoders on each track, sampled by the ESP32 tier |
| Topic | `/joint_states` (`sensor_msgs/JointState`) — positions and velocities for `left_track_joint`, `right_track_joint` |
| Assembly point | Pi 5 drive-mixer node merges both ESP32 encoder streams into one message |
| Use | Wheel odometry term in the EKF; direct feedback for the track velocity controllers on the ESP32s |

---

## 4. Data Flow Across the Cat6 Tether (ROS 2 DDS)

### 4.1 Physical / Transport Layers

```
[ESP32 L] ─┐
           ├─ micro-ROS transport (UART/USB) ─▶ [micro-ROS Agent on Pi 5]
[ESP32 R] ─┘

[Pi 5] ───────────── Cat6 Ethernet (1000BASE-T) ─────────────▶ [Control Station]
           ROS 2 DDS (RTPS over UDP/IP), single ROS_DOMAIN_ID, same subnet
```

- The ESP32-to-Pi5 link is a **local transport** (serial/UDP) terminated by
  the micro-ROS agent; it is not itself DDS traffic, but the agent exposes
  those nodes as ordinary DDS participants once bridged.
- The Pi5-to-Control-Station link is the **Cat6 tether**, carrying native
  ROS 2 DDS (RTPS) traffic over a direct or switched Ethernet connection.
  Because it's a wired, single-hop link, standard discovery multicast works
  without the packet-loss concerns of a wireless link — no special DDS
  discovery tuning is required beyond matching `ROS_DOMAIN_ID` on both ends.
- `use_sim_time` is set uniformly across every node (Pi 5 and Control
  Station) so logged/sim and live runs produce comparable timestamps, per
  `DRIFT.md`.

### 4.2 Logical Data Flow

```
ESP32 (L/R) --encoders--> Pi5 mixer --> /joint_states ---------------------\
                                                                             \
IMU driver ------------> /imu/data, /imu/mag -------------------------------+--> EKF (Pi5) --> /odom --(TF: odom->base_footprint)
                                                                             /
(wheel odom term computed inside EKF from /joint_states) ------------------/

RGB-D driver -----------> /camera/image_raw, /camera/depth/image_raw,
                           /camera/camera_info  -----------\
                                                             \
LiDAR driver ------------> /scan  ----------------------------+---> [Cat6 / DDS] ---> Control Station
                                                             /                          (RViz2, perception, logging)
robot_state_publisher ---> /tf, /tf_static (static chain) --/

Control Station teleop ---> /cmd_vel ---(Cat6 / DDS)---> Pi5 mixer ---> per-track setpoints ---> ESP32 (L/R)
```

- **Outbound (crawler → Control Station):** all sensor topics, `/tf`,
  `/tf_static`, `/odom`, `/ground_truth/odom`, `/joint_states`, diagnostics.
- **Inbound (Control Station → crawler):** `/cmd_vel` from teleop or a
  mission-sequencing script; any dynamic reconfiguration/parameter calls.
- Only one node publishes `odom → base_footprint` (the EKF on the Pi 5);
  `base_footprint → base_link → chassis_link → {camera_link, imu_link,
  lidar_link}` come from `robot_state_publisher` on the Pi 5 and are relayed
  to the Control Station over the same DDS graph — RViz2 does not need its
  own TF source.

### 4.3 QoS and Bandwidth Considerations

| Topic | Suggested QoS | Approx. rate | Notes |
|---|---|---|---|
| `/camera/image_raw`, `/camera/depth/image_raw` | Best Effort, volatile | 15–30 Hz | Largest bandwidth consumer on the tether; consider compressed image transport if the Cat6 link is shared with other traffic |
| `/scan` | Best Effort, volatile | 5–15 Hz | Small payload, low bandwidth impact |
| `/imu/data` | Best Effort, volatile | 50–200 Hz | High rate, small messages |
| `/joint_states` | Reliable, volatile | 20–50 Hz | Must not be silently dropped — control/diagnostics depend on it |
| `/odom`, `/ground_truth/odom` | Reliable, volatile | 20–50 Hz | Reliable so downstream consumers never miss a pose update |
| `/tf`, `/tf_static` | Reliable (`/tf` volatile, `/tf_static` transient-local) | event-driven / 1x on start | Standard ROS 2 TF QoS |
| `/cmd_vel` | Reliable, volatile | operator rate (≤20 Hz) | Must never be silently dropped — a missed command on an incline is a safety issue |

A Gigabit Cat6 link comfortably carries the RGB-D stream plus all other
topics; the practical constraint is Pi 5 CPU/USB throughput for the RGB-D
driver itself, not the tether.

---

## 5. Operational Methodology

### 5.1 Startup Sequence

1. Power the crawler; ESP32 nodes boot and begin publishing idle encoder
   state once the micro-ROS agent on the Pi 5 comes up.
2. Pi 5 boots ROS 2 Jazzy stack: micro-ROS agent → drive mixer →
   `robot_state_publisher` → sensor drivers (RGB-D, LiDAR, IMU) → EKF.
3. Control Station joins the same `ROS_DOMAIN_ID` over the Cat6 link,
   launches RViz2 (Fixed Frame `odom` or `base_footprint`), confirms TF tree
   completeness and sensor topic rates before any motion command is issued.
4. Operator verifies `/odom` vs `/ground_truth/odom` agreement (sim) or
   sanity (hardware) before entering the pipe bore.

### 5.2 Closed-Loop Operation

- **Teleoperated run:** operator command on the Control Station → `/cmd_vel`
  over the tether → Pi 5 mixer → per-track setpoints → ESP32 PID loops →
  encoder feedback closes the loop locally at the actuation tier, while the
  Pi 5 EKF closes the loop at the localization level using the same
  encoder + IMU data.
- **Incline behaviour:** at non-zero pipe incline, the IMU orientation term
  in the EKF is weighted more heavily to catch track slip that pure wheel
  odometry would mask — directly supporting the incline test matrix defined
  in `DRIFT.md` (0°, 5°, 10°, 15°).
- **Fault/disconnection handling:** because the Pi 5 owns local TF and
  odometry independent of the Control Station, a transient tether interruption
  does not corrupt the crawler's internal state — only the operator's
  visibility and command authority are lost until the link recovers. A
  watchdog on `/cmd_vel` staleness should command zero track velocity if no
  command is received within a bounded timeout, so the crawler does not run
  open-loop on a stale command after a tether fault.

### 5.3 Data Logging & Post-Run Analysis

- The Control Station records a `ros2 bag` of every topic in the sensor
  table plus `/tf`, `/odom`, and `/ground_truth/odom` for each test run.
- Odometry error (the gap between `/odom` and `/ground_truth/odom`) is the
  primary metric from the test matrix in `DRIFT.md`, computed offline from
  the recorded bag rather than on the Pi 5 in real time.

---

## 6. Constraints Carried Over from DRIFT.md

These are not solved by the architecture above — they remain open physical
and modelling constraints that the hardware/software split does not change:

- **Footprint vs. pipe bore (Issue 1):** the nominal 0.15 m robot width
  equals the 0.15 m pipe inner diameter. The sensor payload (Gemini 335,
  Okdo LiDAR) must be packaged within whatever reduced track-contact width
  is ultimately adopted (≈0.09 m or less), not the nominal 0.15 m footprint —
  sensor placement on `chassis_link` should be re-checked once the chassis
  width is finalized.
- **CAD mismatch (Issue 2):** the uploaded `AGOS V1.stl` bounding box
  (~0.38 × 1.40 × 0.48 m, ~10M triangles) does not match the nominal
  specification and is far too heavy for either simulation or an embedded
  Pi 5/ESP32 pipeline that expects lightweight collision primitives. This
  must be resolved (confirm scale, split into `agos_chassis.stl` /
  `track.stl`, decimate under ~50k triangles) before the sensor mounts
  described here are finalized against real geometry.
- **Track modelling (Issue 3):** the architecture above assumes the wheel-set
  friction approximation for the tracks, consistent with `DRIFT.md`'s stated
  scope — the ESP32 actuation-tier design (per-side velocity + encoder loop)
  maps directly onto the two `continuous` joints (`left_track_joint`,
  `right_track_joint`) and does not depend on true track-segment dynamics.
