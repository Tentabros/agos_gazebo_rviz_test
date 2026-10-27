# agos_robot_laptop: the AGOS test robot's laptop side

Nav2, RViz and the autonomous explorer for the test robot. The robot's
Raspberry Pi runs `agos_robot_pi` (the sensors, the odometry and RTAB-Map)
and sends the map here through Zenoh. This folder stands on its own: it needs
nothing from `practice_agos` (the simulation), and the two can sit side by
side.

The whole set-up, wiring included, is in `../GUIDE.md`. In short:

```
./agos_laptop tether        # once: the Ethernet cable to the Pi (laptop 10.77.0.1, Pi 10.77.0.2)
./agos_laptop build         # once (Docker: ./agos_laptop image first, or let start do it)
./agos_laptop check         # ROS, Nav2, Zenoh, graphics, and whether the Pi answers
./agos_laptop ping          # is the robot running?
./agos_laptop start         # Nav2 + RViz: drive with 2D Goal Pose
./agos_laptop explore       # the same, plus autonomous exploration
./agos_laptop teleop        # (second terminal) drive with the keyboard
./agos_laptop help          # every command
```

Over Wi-Fi instead of the cable, give the Pi's address once:
`./agos_laptop start 192.168.1.42` (it is remembered; `./agos_laptop start tether`
goes back to the cable). On Windows, `agos_laptop.cmd` runs the script in WSL.

| File | What |
|---|---|
| `agos_laptop` | the script |
| `agos_laptop.cmd` | the script from Windows (WSL + Docker) |
| `docker/Dockerfile` | the image when ROS 2 is not installed: ROS 2 Lyrical, Nav2, RViz, Zenoh |
| `agos_robot_nav/launch/robot.launch.py` | Zenoh router, Nav2, RViz, explorer |
| `agos_robot_nav/config/nav2_base.yaml` | the simulation's Nav2 tuning (a copy of practice_agos's `nav2_agos.yaml`) |
| `agos_robot_nav/config/nav2_robot.yaml` | the test robot's size and speeds, over the base: edit this one |
| `agos_robot_nav/rviz/robot.rviz` | the RViz layout |
| `agos_robot_nav/agos_robot_nav/` | explorer, frontiers (TAD) and nav_supervisor, copied from practice_agos |

The explorer and frontier code are copies of practice_agos's at the time of
the Fix RTAB-Map thread. If you improve them there, copy `explorer.py` and
`frontiers.py` over the ones here (only the `agos_sim` import differs).
