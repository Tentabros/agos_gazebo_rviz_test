# agos_robot_pi: the AGOS test robot's Raspberry Pi 4

The Pi side of the test robot: the Gemini 335 and RPLiDAR C1 drivers, LiDAR
odometry (icp_odometry with the camera's IMU), RTAB-Map with the simulation's
settings, the micro-ROS agent for the ESP32-C3 motor node, and a monitor.
Nav2, the explorer and RViz run on the laptop (agos_robot_laptop: `./agos_laptop start`), connected by an Ethernet cable (`./agos_robot tether`) or Wi-Fi.

The whole set-up, wiring included, is in `../GUIDE.md`. In short:

```
./agos_robot setup          # once: udev rules, USB buffer, agent image (sudo)
./agos_robot tether         # once: the Ethernet cable to the laptop (Pi 10.77.0.2)
./agos_robot check          # builds the Docker image the first time, then checks everything
./agos_robot start          # everything; Ctrl+C stops it and saves the map
./agos_robot status         # one status line
./agos_robot motor-test     # drive each way briefly (wheels off the floor first)
./agos_robot help           # every command
```

| File | What |
|---|---|
| `agos_robot` | the script |
| `robot.launch.py` | everything that runs on the Pi |
| `config/robot.yaml` | where the wheels and sensors sit: measure your robot |
| `uros_relay.py` | carries /cmd_vel from Zenoh to the micro-ROS agent's DDS, and the ESP32's heartbeat back |
| `robot_monitor.py` | /agos_robot/status, a CSV file per run in `logs/`, `./agos_robot stats` |
| `odom_gps.py` | the odometry as a GPS fix for RTAB-Map (a copy of practice_agos's) |
| `motor_test.py` | `./agos_robot motor-test` |
| `docker/` | the Pi's image: ROS 2 Lyrical, Orbbec driver, sllidar (patched), RTAB-Map, Zenoh |
| `udev/` | /dev/rplidar, /dev/esp32 and Orbbec's camera rule |
