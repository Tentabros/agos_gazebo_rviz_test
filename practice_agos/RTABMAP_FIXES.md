# RTAB-Map + Nav2 simulation fixes (practice_agos/agos_sim)

> **Update (network round):** `./agos start` now replaces the terminals in
> "How to run" below, and the default world is the larger network. See
> `NETWORK_NAV_EXPLORE.md`.

All changes are in `practice_agos/agos_sim`. They were traced from the code and
checked for syntax (Python compiles, XML/YAML parse), but not run in ROS, so
please do the quick checks at the end after rebuilding.

## Outline

| # | Issue | Effect you saw | File changed |
|---|---|---|---|
| 1 | `/clock` was never bridged from Gazebo | Every node's clock stuck at 0, so TF lookups failed, which kept Nav2 inactive and hid data in RViz | `launch/sim.launch.py` |
| 2 | Nav2 started with `bringup_launch.py` | Also started map_server + AMCL, a second localization system that fights RTAB-Map and waits for a map file and initial pose | `launch/slam_nav.launch.py` |
| 3 | Camera topic names didn't match between bridge, RTAB-Map and RViz | Depth point cloud blank in RViz; names fragile | `sim.launch.py`, `slam_nav.launch.py`, `rviz/agos.rviz` |
| 4 | RTAB-Map sync and 2D settings missing | Sensor sets rarely matched; map could tilt/smear | `launch/slam_nav.launch.py` |
| 5 | Nav2 stock params for a different robot | Footprint far too small; on your ROS version Nav2 sends `TwistStamped`, which the bridge ignores, so goals don't move the robot | `launch/slam_nav.launch.py` |
| 6 | World is an open plane with three objects | 2D LiDAR scan matching has too little to lock on to | `worlds/agos_world.sdf` |
| 7 | RViz had no map, costmap or plan display, and used `odom` as fixed frame | You couldn't see what RTAB-Map built | `rviz/agos.rviz` |
| 8 | Small packaging issues | `launch/*` glob can pick up `__pycache__`; missing launch dependencies | `setup.py`, `package.xml` |

---

## 1. `/clock` was missing (the root cause of the inactive lifecycle manager)

**What was wrong.** `sim.launch.py` built the bridge from a hard-coded list of
`arguments`. That list bridged `/cmd_vel`, `/odom`, the sensors, and so on, but
**not `/clock`**. `config/bridge.yaml` did include `/clock`, but the launch file
never loaded it.

**Why it matters.** Every node (robot_state_publisher, RTAB-Map, RViz, all of
Nav2) runs with `use_sim_time: True`. That setting means "don't use the PC's
clock, use the time published on `/clock`". With nothing on `/clock`, each node's
clock stays at **0 seconds**, while the sensor and odometry messages coming out
of Gazebo carry real simulation timestamps (for example 125.4 s).

- TF stamps no longer line up. RTAB-Map stamps `map -> odom` with its own clock
  (≈0), while `odom -> base_footprint` from Gazebo carries ≈125 s. Asking for
  `map -> base_link` then fails with "extrapolation" errors.
- Nav2's costmaps refuse to finish activating until they can look up
  `map -> base_link`. The lifecycle manager is waiting for them, so it never
  reaches **active**. That is the "inactive lifecycle manager" you found.
- Timers that run on a sim-time clock that never advances don't fire, so some
  periodic work in Nav2 can stall too.
- RViz drops messages it can't place in time, which is why objects (scan, cloud)
  looked missing or flickered.

**Change.** The bridge now loads the YAML file, which already lists `/clock`
plus every other topic:

```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': bridge_config}],
)
```

The separate `odom_tf_bridge` (which publishes `odom -> base_footprint`) is
unchanged. The stray `/tf` bridge argument went away with the argument list;
nothing in Gazebo publishes on that topic.

## 2. Nav2 was launched with the full bringup (map_server + AMCL)

**What was wrong.** `slam_nav.launch.py` included `nav2_bringup/bringup_launch.py`
with `map: ''`. That launch file is built for driving on a map **you saved
earlier**. Besides the navigation servers, it starts a localization part:
`map_server` (loads a map file) and `AMCL` (localizes on that map), with its own
lifecycle manager.

**Why it matters.**
- With an empty map path, map_server has nothing to serve and AMCL has no map
  and no initial pose. That localization lifecycle manager can't bring its nodes
  up cleanly.
- If AMCL does start publishing (for example after a "2D Pose Estimate" click),
  it publishes `map -> odom` too. RTAB-Map also publishes `map -> odom`. Two
  publishers of one transform make the robot and map jump back and forth.

**Change.** It now includes `nav2_bringup/navigation_launch.py`, which starts only
the navigation servers (planner, controller, behaviors, BT navigator, smoother,
etc.) and **`lifecycle_manager_navigation`**. RTAB-Map is the only source of
`/map` and `map -> odom`. This is the standard RTAB-Map + Nav2 pairing.

Nav2 is also delayed 5 s (`TimerAction`) so RTAB-Map is already publishing
`map -> odom` when the costmaps start. That way they don't sit in the "waiting
for transform" loop.

## 3. Camera topic names didn't agree

**What was wrong.** There were two naming schemes in the package:

| Data | Bridge arguments (what actually ran) | bridge.yaml / RViz |
|---|---|---|
| RGB | `/camera/image` | `/camera/image_raw` |
| Depth | `/camera/depth_image` | `/camera/depth/image_raw` |
| Point cloud | `/camera/points` | `/camera/depth/points` |

RTAB-Map happened to be remapped to the first scheme. RViz's Depth PointCloud
display subscribed to `/camera/depth/points`, which nothing published, so that
display was always empty.

**Change.** Everything now uses the `bridge.yaml` names: `/camera/image_raw`,
`/camera/depth/image_raw`, `/camera/camera_info`, `/camera/depth/points`.
RTAB-Map's remappings and the RViz Camera RGB display were updated to match.

## 4. RTAB-Map settings

All in `RTABMAP_PARAMS` at the top of `slam_nav.launch.py`, shared by `rtabmap`
and `rtabmap_viz` so they always agree.

| Parameter | Value | Why |
|---|---|---|
| `approx_sync` | `true` | RTAB-Map needs RGB + depth + scan + odom together. The camera (now 10 Hz, 15 Hz at first), the LiDAR (10 Hz) and odom (50 Hz) never share a timestamp exactly. Approximate sync pairs the closest ones. Without it, matched sets are rare and the map barely updates. |
| `sync_queue_size` / `topic_queue_size` | `10` / `2` (were `30` / `10`) | Room to pair the camera with the LiDAR. The smaller queues (changed in the network round, see `NETWORK_NAV_EXPLORE.md`) keep RTAB-Map on the newest data. |
| `odom_sensor_sync` | `true` | Corrects for odometry and sensor stamps being slightly apart. |
| `Reg/Force3DoF` | `true` | AGOS is a ground robot, so it only moves in x, y and yaw. Without this, scan matching can add small z/roll/pitch errors that tilt the graph and make the 2D grid smear or duplicate obstacles. |
| `Grid/Sensor` | `0` | Builds the occupancy grid from the LiDAR. Replaces the older `Grid/FromDepth: false`, which current RTAB-Map renamed. |
| `Optimizer/GravitySigma` | `0` | No IMU constraints, since the robot is already locked to 2D. |
| `odom_frame_id`, `odom_tf_linear_variance` / `odom_tf_angular_variance` | `odom`, `0.0002` / `0.00005` | Added in the network round. Odometry now comes from the `odom -> base_footprint` transform (the EKF) with a realistic uncertainty. Taken from `/odom`, it carried the EKF's variance of 1e-9, so RTAB-Map trusted it completely and rejected every loop closure. |
| `RGBD/NeighborLinkRefining`, `RGBD/ProximityBySpace` | `false` (was `true` / default) | Changed in the network round. LiDAR ICP between neighbouring map nodes cannot tell how far the robot moved in a round pipe; in a test mission it turned the map 6.5 deg off. Loop closures (`Reg/Strategy` `1`: ICP on the LiDAR scan) still correct drift, within 12 m (`Rtabmap/LoopGPS` below). |
| `GridGlobal/FootprintRadius` | `0.25` (was `0`) | Added with the whole-network exploration. Cells within 0.25 m of where the robot has been are free on `/map`. Back in a pipe after a long drive, the map has drifted and RTAB-Map draws its wall a second time, a little apart; that wall could cross the path just driven, and Nav2 then refused to plan ("Start occupied"). |
| `Rtabmap/LoopGPS`, `RGBD/LocalRadius`, and the `odom_gps` node | `true` (the default), `12` (was `10`) | Added with the whole-network exploration. The pipes and manholes in the simulation look alike, and late in a whole-network run RTAB-Map closed a loop between two places 33 m apart, which turned the map 100 deg. `odom_gps` (started by `slam_nav.launch.py` when it makes a new map) gives RTAB-Map the odometry as a GPS fix, and with `Rtabmap/LoopGPS` RTAB-Map only looks for loop closures among the map nodes whose fix is within `RGBD/LocalRadius` of the robot's. Back at the start after the whole network (400 m) the odometry was 9 m off, and 12 m leaves room for that. 15 m, tried first, let in a match 14 m away by odometry (18 m in fact) at the end of a run, after the right closures there had been undone (next row). RTAB-Map uses the fixes for nothing else (`Optimizer/PriorsIgnored` is true by default). |
| `RGBD/OptimizeMaxError` | `5` (was `3`, the default) | Added with the whole-network exploration. After each loop closure RTAB-Map optimizes the map and undoes the closure if any link is then stretched more than this many standard deviations. Back at the start after the whole network, each of the right closures stretched an odometry link 3.0001 to 3.0005 times, so all 47 were undone; the map stayed 9 m off and a wrong match got in. Replaying that run's map data with `5` and the 12 m radius, every closure it kept was right (777, checked against Gazebo's true poses), and the wrong matches it was offered, between places 6 to 18 m apart, stretched links 7.5 to 22 times. |
| `Icp/Strategy` | `0` | Added with the whole-network exploration. PCL's ICP, the one every test here ran with. RTAB-Map built with libpointmatcher (as the ROS packages are) uses that by default instead (`1`), with other settings, which was not tested. |
| `Reg/Strategy`, `Grid/3D`, `Grid/RayTracing`, `Grid/RangeMax` | unchanged | These were already right. |

The `-d` argument is kept. It deletes `~/.ros/rtabmap.db` on every start, so each
run begins with a fresh map (except with `localization:=true`, which reuses it).

## 5. Nav2 parameters for AGOS

**What was wrong.** It used Nav2's stock `nav2_params.yaml` as-is. That file is
tuned for a TurtleBot (`robot_radius: 0.22`). AGOS is about 0.60 x 0.42 m.
Also, your workspace builds against **ROS 2 Lyrical**. Recent Nav2 versions send
`cmd_vel` as `geometry_msgs/TwistStamped` by default, while the Gazebo bridge
and DiffDrive plugin use plain `geometry_msgs/Twist`. Both use the topic
`/cmd_vel` but the types differ, so the bridge never receives Nav2's commands
and the robot won't move to a goal.

**Change.** At launch, `_write_nav2_params()` loads the stock file, then:
- sets every `robot_radius` to **0.37 m**, half the robot's diagonal;
- sets `enable_stamped_cmd_vel: false`, both where the file lists it and
  globally (the `/**` section), so Nav2 publishes plain `Twist`.

It writes the result to `/tmp/agos_nav2_params.yaml` and passes that to Nav2.
Your keyboard teleop keeps working unchanged. When you want full Nav2 tuning,
copy that generated file into `config/` and point the launch file at it.

## 6. World: added a boundary room

**What was wrong.** The world was an open 100 m plane with a box, a sphere, a
cylinder and a short tunnel. RTAB-Map registers consecutive LiDAR scans with ICP
(`Reg/Strategy: 1`). With most beams hitting nothing, each scan is only a few
short arcs. Inside the tunnel it sees two parallel walls, which tell ICP nothing
about forward motion. ICP then fails or slides, links get rejected, and objects
appear in the wrong place or twice.

**Change.** Added a static 10 m x 8 m room with 1 m walls (`boundary_room`)
around the existing objects. Every scan now has corners and long walls to lock
on to. The original objects are untouched.

## 7. RViz

- **Fixed Frame `odom` -> `map`.** You now see the SLAM-corrected view. If you
  run `sim.launch.py` without SLAM, set it back to `odom` in the left panel.
- **Added `RTAB-Map Grid`** (`/map`, transient-local QoS so the latest map shows
  up even when RViz starts late).
- **Added `Global Costmap`** (`/global_costmap/costmap`) and **`Global Plan`**
  (`/plan`), so you can see where Nav2 thinks obstacles are and the path it chose.
- Camera RGB topic fixed (see 3).

## 8. Packaging

- `setup.py`: `glob('launch/*')` -> `glob('launch/*.launch.py')`. A
  `launch/__pycache__` folder (Python already created one in your install
  folder) would make `colcon build` fail with "not a regular file" if it ever
  appears in the source `launch/` folder.
- `package.xml`: added `launch`, `launch_ros`, `ament_index_python` and
  `python3-yaml`, which the launch files import.

---

## How to run and check

```bash
cd ~/practice_agos
colcon build --symlink-install
source install/setup.bash

# Terminal 1
ros2 launch agos_sim sim.launch.py
# Terminal 2
ros2 launch agos_sim slam_nav.launch.py        # add nav2:=false for SLAM only
# Terminal 3 (drive around)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Quick checks:

```bash
ros2 topic echo /clock --once                     # must print a time, not hang
ros2 run tf2_ros tf2_echo map base_link           # should print a transform
ros2 lifecycle get /controller_server             # -> active [3]
ros2 lifecycle get /bt_navigator                  # -> active [3]
ros2 topic hz /map                                # RTAB-Map grid updates while moving
```

RTAB-Map only adds a new map node after the robot moves about 0.1 m or turns
about 0.1 rad (`RGBD/LinearUpdate`, `RGBD/AngularUpdate`). A stationary robot
won't grow the map.

## Things noticed but not changed

- **Point cloud orientation.** The camera is stamped with
  `camera_optical_frame`, which is correct for the images RTAB-Map uses. If the
  *Depth PointCloud* display in RViz looks rotated 90°, Gazebo is producing that
  cloud in the camera body frame. RTAB-Map doesn't use that topic, so mapping is
  unaffected. (Inferred, not verified on your machine.)
- **Box and sphere are dynamic.** If the robot bumps them they move or roll, and
  RTAB-Map will see ghosts. Add `<static>true</static>` if you want a fixed scene.
- **Skid-steer odometry.** The DiffDrive plugin assumes no slip, but tracks slip
  when turning, so wheel odometry alone over-reports rotation. The EKF takes the
  heading from the IMU gyro, which does not slip. Spin slowly while mapping.
- **Flat-coloured objects.** They give the camera few visual features for loop
  closure. Textured models would help once you move to a pipe/drain world.
- **`DRIFT.md` says Jazzy + Harmonic**, but the build logs show `/opt/ros/lyrical`.
