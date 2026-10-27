# AGOS test robot: Raspberry Pi 4, laptop and ESP32-C3 as one system

(The same guide, editable: https://claude.ai/artifact/C98boDpb8zZsQiXhSLE8v4)

This guide builds a small driving test robot from your RPLiDAR C1, Orbbec Gemini 335, Raspberry Pi 4, an ESP32-C3 SuperMini, an L298N and two yellow TT gear motors. It uses the latest code from the Fix RTAB-Map thread: RTAB-Map runs on the Pi with the simulation's settings, the Pi and the laptop talk through Zenoh over an Ethernet cable (the tether), and Nav2, the explorer and RViz run on the laptop. The laptop side is its own folder, so your practice_agos (the simulation) stays exactly as it is.

All the code is in `AGOS-robot.zip`:

| Folder | Goes on | What it is |
|---|---|---|
| `esp32_motor_node/` | the ESP32-C3 (flashed from the laptop) | micro-ROS firmware: `/cmd_vel` to the two motors, with a safety stop |
| `agos_robot_pi/` | the Raspberry Pi 4 | sensors, LiDAR odometry, RTAB-Map, the micro-ROS agent, a monitor, one script: `./agos_robot` |
| `agos_robot_laptop/` | the laptop, next to practice_agos | Nav2, the explorer and RViz for the real robot, and the tether set-up: `./agos_laptop` |

## 1. How the system fits together

```
 Laptop (Ubuntu or Windows + WSL)              Raspberry Pi 4 (on the robot)
 agos_robot_laptop: ./agos_laptop start        agos_robot_pi: ./agos_robot start
   Nav2 (MPPI, Hybrid-A*), explorer, RViz        Gemini 335 driver   -> colour, depth, IMU
   teleop (./agos_laptop teleop)                 sllidar (C1)        -> /scan
        |                                        Madgwick filter     -> /imu/data
        |  /cmd_vel                              icp_odometry        -> odom -> base_footprint
        v                                        RTAB-Map            -> /map, map -> odom
   Zenoh router == Ethernet tether, TCP 7447 ==> Zenoh router
   10.77.0.1                                     10.77.0.2
        ^                                        uros_relay (Zenoh <-> DDS)
        |  /map, /scan, /tf, /odom                 micro-ROS agent (Docker)
                                                     | USB
                                                 ESP32-C3 SuperMini (micro-ROS)
                                                     | PWM + direction
                                                 L298N -> left and right TT motors
```

Why it is split this way:

- **RTAB-Map stays on the Pi**, as in the Fix RTAB-Map thread. The camera's images never cross the link: only the scan, the transforms, the odometry and the map go to the laptop (well under 1 MB/s). The camera image goes too, only while RViz shows it.
- **The Pi and the laptop are joined by an Ethernet cable**, on their own two-address network: the laptop is 10.77.0.1 and the Pi 10.77.0.2. There is no router and no Wi-Fi to drop out, the delay is steady, and it works anywhere, a sewer included. Both keep their Wi-Fi for the internet. Wi-Fi still works as the link instead: give `./agos_laptop` the Pi's Wi-Fi address.
- **The laptop side is a separate folder**, `agos_robot_laptop`. It has its own copy of the explorer and of the simulation's Nav2 tuning, so it needs nothing from practice_agos, and practice_agos needs nothing from it.
- **Odometry comes from the LiDAR.** The TT motors have no encoders, so `icp_odometry` matches each C1 scan to the last ones, started level by the camera's IMU. The motors run open loop, and Nav2 corrects them from where the LiDAR says the robot is.
- **The ESP32 speaks micro-ROS through an agent on the Pi.** The agent speaks only DDS, and the rest of AGOS speaks Zenoh. A small two-part relay (`uros_relay.py`) carries `/cmd_vel` to the ESP32 and its heartbeat back. Nothing else crosses, and the ESP32 side has its own ROS domain (77), so nobody else's ROS 2 on the same network can drive the robot.
- **The ESP32 stops the motors by itself** when no `/cmd_vel` arrives for 0.7 s (a pulled cable, a crashed laptop, a closed teleop window), and whenever it loses the agent.

## 2. Parts and wiring

### Parts

- Raspberry Pi 4 (4 GB or more) with a heatsink and fan, a 16 GB+ microSD card, and Raspberry Pi OS 64-bit or Ubuntu 24.04/26.04 64-bit.
- Orbbec Gemini 335, its USB-C to USB-A **USB 3** cable, and a **powered USB 3 hub**.
- SLAMTEC RPLiDAR C1 and its USB adapter.
- ESP32-C3 SuperMini and a USB-C **data** cable (charge-only cables do not work).
- L298N motor driver module, 2 yellow TT gear motors (1:48) with 65 mm wheels, a caster, and a chassis.
- Motor battery: 2S Li-ion (7.4 V, 18650 cells in a holder with protection), or 6 x AA NiMH (7.2 V), with an on/off switch.
- Pi power on the move: a USB-C power bank that gives 5 V at 3 A, or a 5 V 5 A buck converter from a second pack. The powered hub needs 5 V too (from the same converter or a second bank port).
- Two 0.1 µF ceramic capacitors (one across each motor's terminals, against electrical noise), jumper wires, and a small switch.
- **The tether:** an Ethernet cable (Cat 5e or Cat 6), long enough for where the robot will drive (5-10 m for a room), preferably a thin, flexible one. Any cable works, crossover or not. A laptop without an Ethernet port needs a USB-C or USB-A Ethernet adapter (a gigabit one with a Realtek or ASIX chip works on Ubuntu and Windows without drivers).

### Wiring the ESP32-C3 SuperMini to the L298N

Take **both ENA and ENB jumpers off** the L298N first: the ESP32 drives those pins with PWM.

| ESP32-C3 SuperMini | L298N | Job |
|---|---|---|
| GPIO 5 | ENA | left motor speed (PWM) |
| GPIO 6 | IN1 | left motor direction |
| GPIO 7 | IN2 | left motor direction |
| GPIO 10 | ENB | right motor speed (PWM) |
| GPIO 3 | IN3 | right motor direction |
| GPIO 4 | IN4 | right motor direction |
| GND (G) | GND | **common ground**: required |
| USB-C | (to the Pi, black USB 2 port) | power and micro-ROS |

| L298N | Goes to |
|---|---|
| OUT1, OUT2 | left motor |
| OUT3, OUT4 | right motor |
| +12V (VS) | motor battery + (through the switch) |
| GND | motor battery − **and** the ESP32's GND |
| +5V | nothing (leave the 5V-EN jumper on; do not feed the ESP32 from it while it is on USB) |

The pins avoid the C3's strapping pins (GPIO 2, 8 and 9), USB (18, 19) and the UART (20, 21). GPIO 8 is the on-board LED, which the firmware uses for status. Every pin is set in `esp32_motor_node/include/config.h`.

### Wiring the Pi and sensors

- The Gemini 335 goes into the **powered USB 3 hub**, and the hub goes into a **blue** USB 3 port on the Pi. The Pi 4's ports share 1.2 A, and the camera alone peaks at 1.3 A.
- The C1's USB adapter goes into a black USB 2 port on the Pi.
- The ESP32 goes into the other black USB 2 port.
- The Pi takes 5.1 V at 3 A on its USB-C port. An under-powered Pi slows itself down and drops USB devices; `./agos_robot check` reports it. The tether carries no power: the Pi still runs from the robot's own supply.
- The tether goes into the Pi's Ethernet port. Tie the cable to the chassis just behind the Pi (a cable tie round a standoff) so a pull on it tugs the robot, not the Pi's socket, and lead it off the back, where it cannot reach the wheels or the C1's scan.

### Mounting

- Mount the C1 on top, level, with a clear 360° view. Nothing on the robot may rise into its scan plane.
- Mount the camera at the front, below the C1's scan plane, looking forward.
- Measure, from the floor point under the middle of the two wheels (x forward, y left, z up):
  - the C1's scan centre (its spinning axis, at the height of its window),
  - the camera's `camera_link` (the left IR lens seen from behind the camera),
  - the wheel separation (middle of tyre to middle of tyre).
- Write them into `agos_robot_pi/config/robot.yaml`. Write the wheel separation into `esp32_motor_node/include/config.h` too.

## 3. Install

### 3a. The laptop

The laptop side goes in its own folder next to practice_agos. Leave practice_agos as it is: the simulation keeps working from it as before.

```
unzip -o ~/Downloads/AGOS-robot.zip -d ~
cd ~/AGOS-robot/agos_robot_laptop
./agos_laptop build
./agos_laptop check
```

On Windows with WSL, run these in the Ubuntu app, with the zip taken from your Windows Downloads folder as the Fix thread's README shows (`wslpath`). `agos_laptop.cmd` in the folder runs the same script from PowerShell (`.\agos_laptop start`).

`./agos_laptop` uses ROS 2 Lyrical if it is installed, and Docker if not, as practice_agos's `./agos` does:

- **Docker** (Windows + WSL, or any Linux with Docker): the first `./agos_laptop build` builds its own image, `agos-laptop:lyrical`, with Nav2, RViz, Zenoh and teleop (10-20 minutes, once). It is smaller than the simulation's image: no Gazebo, no RTAB-Map.
- **ROS 2 installed on Ubuntu**: you need these on top of what practice_agos uses (`./agos_laptop check` lists anything missing):

  ```
  sudo apt install ros-lyrical-navigation2 ros-lyrical-nav2-bringup ros-lyrical-rmw-zenoh-cpp ros-lyrical-image-transport-plugins ros-lyrical-teleop-twist-keyboard python3-colcon-common-extensions
  ```

For the ESP32, install **VS Code with the PlatformIO extension** (on Windows itself, not in WSL, which cannot see USB devices without extra tools), or the command line tool: `pip install --user platformio`. On Linux, also install PlatformIO's udev rules and add yourself to the `dialout` group:

```
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo usermod -aG dialout $USER
```

### 3a (alternative). An Ubuntu desktop instead of the laptop

A desktop PC running Ubuntu does the laptop's whole job (Nav2, the explorer, RViz, flashing the ESP32), and usually better: a stronger CPU for Nav2 and a real GPU for RViz. Use Ubuntu 26.04 (the version ROS 2 Lyrical is built for) with a working graphics driver: check with `glxinfo -B | grep renderer` (from `mesa-utils`); it should name your GPU, not `llvmpipe`.

**Option A: ROS 2 installed natively (recommended on a desktop).**

```
sudo apt update && sudo apt install -y curl software-properties-common
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y \
    ros-lyrical-desktop ros-lyrical-navigation2 ros-lyrical-nav2-bringup \
    ros-lyrical-teleop-twist-keyboard ros-lyrical-rmw-zenoh-cpp \
    ros-lyrical-image-transport-plugins \
    python3-colcon-common-extensions python3-numpy mesa-utils
```

(To run the simulation on the desktop too, also install `ros-lyrical-rtabmap-ros ros-lyrical-robot-localization ros-lyrical-ros-gz ros-lyrical-xacro ros-lyrical-joint-state-publisher-gui` for practice_agos.)

Then unpack and build (`./agos_laptop` finds the native ROS and uses it instead of Docker):

```
unzip -o ~/Downloads/AGOS-robot.zip -d ~
cd ~/AGOS-robot/agos_robot_laptop
./agos_laptop build && ./agos_laptop check
```

**Option B: Docker, like the laptop.** Install Docker (`curl -fsSL https://get.docker.com | sh`, then `sudo usermod -aG docker $USER` and log out and in), unpack as above, and run `./agos_laptop build`: it builds the image with every package the first time.

**Tether.** Most desktops have an Ethernet port, but it is usually already plugged into your router. Use a second port: a USB Ethernet adapter, or a PCIe network card. Then run `./agos_laptop tether <port>` (section 3d), naming the second port, which `nmcli device status` lists (e.g. `enx00e04c680001` for a USB adapter). A desktop on the far side of the room needs a long cable; Wi-Fi (3d) may suit it better.

**ESP32.** PlatformIO runs natively on Ubuntu (VS Code with the PlatformIO extension, or `pip install --user platformio`); install the udev rules and the `dialout` group exactly as above. Plugged into the desktop, the ESP32 shows up as `/dev/ttyACM0`.

**Running.** Everything after this is the same as on the laptop: wherever this guide says "on the laptop", use the desktop.

### 3b. The ESP32-C3

1. Open `esp32_motor_node/include/config.h`. Set `WHEEL_SEPARATION`. Leave the rest for now.
2. Plug the ESP32 into the laptop and flash it:

   ```
   cd esp32_motor_node
   pio run -e serial -t upload
   ```

   The first build downloads and compiles micro-ROS (Kilted, to match the agent) and takes 5-15 minutes. In VS Code: PlatformIO icon, `serial`, Upload.
3. If the upload cannot find the board, put it in download mode: hold BOOT, tap RESET, release BOOT, then upload again. Press RESET afterwards.
4. Check: the blue LED **blinks slowly**. It is looking for the micro-ROS agent, which runs on the Pi. The serial monitor shows nothing readable: micro-ROS uses the port in binary.

### 3c. The Raspberry Pi 4

1. Flash Raspberry Pi OS (64-bit) Lite, or Ubuntu Server 24.04/26.04 64-bit, with Raspberry Pi Imager. Turn on SSH and set your Wi-Fi in the Imager's settings. The Pi needs the internet once, for Docker and the image; Wi-Fi is the easy way.
2. Install Docker, then log in again:

   ```
   curl -fsSL https://get.docker.com | sh
   sudo usermod -aG docker $USER
   sudo reboot
   ```

3. Copy the zip to the Pi and unpack it (from the laptop: `scp AGOS-robot.zip <you>@<pi-address>:~`):

   ```
   sudo apt install -y unzip
   unzip -o ~/AGOS-robot.zip -d ~
   cd ~/AGOS-robot/agos_robot_pi
   ```

4. One-time setup: udev rules for `/dev/rplidar`, `/dev/esp32` and the camera, the 128 MB USB buffer the camera needs, and the micro-ROS agent's image:

   ```
   ./agos_robot setup
   ```

   Then unplug and plug in the C1 and the ESP32.
5. Build the image (20-40 minutes, once) and check everything:

   ```
   ./agos_robot check
   ```

   It should list `/dev/rplidar`, `/dev/esp32`, the Gemini 335 on a 5000M (USB 3) port, a 128 MB USB buffer, no throttling, Docker, every ROS package, and the Pi's addresses. It ends with the command to type on the laptop.
6. Set up the tether (section 3d).

If you already installed ROS 2 Lyrical natively on the Pi (the earlier sensor guide), `./agos_robot` uses it instead of Docker. It then also needs `sudo apt install ros-lyrical-rtabmap-slam ros-lyrical-rtabmap-odom ros-lyrical-rtabmap-sync ros-lyrical-imu-filter-madgwick ros-lyrical-rmw-zenoh-cpp ros-lyrical-robot-state-publisher`, and sllidar_ros2 built in `~/agos_ws`. The micro-ROS agent always runs in Docker.

### 3d. The tether, and the clocks

The cable gives the two machines a network of their own: the laptop is **10.77.0.1** and the Pi **10.77.0.2**, fixed, with no router and no DHCP. Neither uses the cable for the internet: each keeps its Wi-Fi for that. Each side has a one-time command that also makes the laptop the Pi's clock: Nav2 on the laptop uses the Pi's timestamps, so the two clocks must agree to within a few tens of milliseconds, and the Pi has no battery clock.

**On the Pi** (it finds NetworkManager on Raspberry Pi OS Bookworm and later, netplan on Ubuntu Server, or dhcpcd on older Raspberry Pi OS):

```
./agos_robot tether
```

It gives `eth0` the address 10.77.0.2 and sets chrony to follow the laptop at 10.77.0.1, with internet time as the fallback. (`./agos_robot tether <interface>` for a port other than `eth0`.)

**On an Ubuntu laptop** (outside Docker; it uses NetworkManager, which Ubuntu desktops have):

```
./agos_laptop tether
```

It finds the Ethernet port (`./agos_laptop tether <port>` to name one), gives it 10.77.0.1 with no gateway, and runs chrony as a time server for 10.77.0.0/24, opening UDP 123 in ufw if ufw is on. It asks for your sudo password.

**On Windows (WSL):** the Ethernet port belongs to Windows, so `./agos_laptop tether` only prints what to type. In PowerShell run as Administrator (the adapter's name is in Settings > Network & internet > Advanced network settings; a USB adapter may be `Ethernet 2`):

```
New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 10.77.0.1 -PrefixLength 24
Set-ItemProperty "HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\NtpServer" -Name Enabled -Value 1
Set-ItemProperty "HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\Config" -Name AnnounceFlags -Value 5
Set-Service w32time -StartupType Automatic; Restart-Service w32time
New-NetFirewallRule -DisplayName "AGOS robot clock" -Direction Inbound -Protocol UDP -LocalPort 123 -RemoteAddress 10.77.0.0/24 -Action Allow
```

The first line gives the port its address and the rest make Windows Time serve the time to the Pi. WSL and Docker reach the Pi through Windows: the laptop connects out to the Pi, so nothing has to reach into WSL.

**Check it.** Plug in the cable. `./agos_robot check` on the Pi should say `Tether: the laptop answers at 10.77.0.1`, and `./agos_laptop check` should say the Pi answers at 10.77.0.2. On the Pi, `chronyc sources` should list 10.77.0.1 with `^*` (the source in use) after a minute.

**Wi-Fi instead of the cable.** Put both on the same Wi-Fi (a home router or a phone hotspot; university and office networks often stop devices reaching each other), reserve the Pi's address in the router, and give it to the laptop once: `./agos_laptop start <pi-address>`. It is remembered; `./agos_laptop start tether` goes back to the cable. For the clocks, either both use internet time (`timedatectl` should say `System clock synchronized: yes` on each) or make the laptop the Pi's clock over Wi-Fi: after `./agos_laptop tether`, add `allow 192.168.0.0/16` to `/etc/chrony/conf.d/agos.conf` on the laptop and restart chrony, then run `./agos_robot setup <laptop-address>` on the Pi.

## 4. First start, step by step

Do these in order the first time. Each step checks one part before the next one depends on it.

**Step 1. Start the robot, wheels off the floor.** Put the robot on a box so the wheels spin freely. Switch on the motor battery. On the Pi:

```
cd ~/AGOS-robot/agos_robot_pi
./agos_robot start
```

The log shows the camera, the C1, the odometry and RTAB-Map starting, then `uros_relay` halves ready. The ESP32's LED turns **solid**: it is connected to the agent. If it keeps blinking: `./agos_robot agent-log`.

**Step 2. Check the status.** In a second SSH terminal on the Pi:

```
./agos_robot status
```

You should see the LiDAR at about 10 Hz, the IMU at about 200 Hz, odometry at about 10 Hz, RTAB-Map's update time, and `ESP32: ok, 0 commands`.

**Step 3. Check the motors.**

```
./agos_robot motor-test
```

It drives forwards, backwards, left and right for 2 s each and says what each wheel should do.

- A wheel turns the wrong way: set its `INVERT_LEFT` or `INVERT_RIGHT` to 1 in `config.h` and flash again (or swap that motor's two wires).
- Left and right are swapped: swap the motors' plugs (OUT1/OUT2 with OUT3/OUT4).
- A wheel hums but does not turn: raise `MIN_DUTY` (step 7).

**Step 4. Connect the laptop.** With the tether plugged in, on the laptop:

```
cd ~/AGOS-robot/agos_robot_laptop
./agos_laptop ping
./agos_laptop start
```

`ping` prints the Pi's status line. `start` opens RViz with the map, the scan, the robot model, the costmaps and the camera. Nav2 starts by itself once RTAB-Map has made its first map. Over Wi-Fi, add the Pi's address the first time (`./agos_laptop start <pi-address>`).

**Step 5. Check the sensor directions.** Put a box about 1 m straight in front of the robot. In RViz, its scan points must lie ahead of the robot, on the red x axis.

- Behind: set the C1's `yaw` in `robot.yaml` to `3.14159`.
- On the left: `-1.5708`.
- On the right: `1.5708`.

Restart `./agos_robot start` after each change. The camera image must show the same box. If the camera is tipped down on its mount, set its `pitch` (positive tips it down).

**Step 6. Drive by keyboard.** Put the robot on the floor. In a second laptop terminal:

```
./agos_laptop teleop
```

Drive slowly round the room (`i` forwards, `,` backwards, `j`/`l` turn, `k` stop, `q`/`z` faster or slower by 10 %). It starts at 0.15 m/s; plain teleop would start at 0.5 m/s, the robot's full speed. Keep the cable slack behind the robot as it goes. The map grows in RViz as the robot moves. Releasing the key stops the robot within 0.7 s.

**Step 7. Calibrate the motors.**

- **Top speed.** Run `./agos_robot motor-test duty` with 2 m free ahead. Measure how far the robot drove in the 2 s, divide by 2, and set `MAX_WHEEL_SPEED` in `config.h`.
- **Slowest speed.** With the wheels off the floor, run `./agos_robot motor-test 0.03 0.4`. If a wheel does not turn, raise `MIN_DUTY` by 0.05 and flash again. Repeat until both turn.
- **Straight line.** If the robot curves when driving straight, the side it curves towards is weaker. Swap the two motors to check whether the motor or the wiring is the cause. Nav2 corrects small differences from the LiDAR odometry.

Flash the ESP32 again after each change. Unplug it from the Pi and flash from the laptop, or install PlatformIO on the Pi and flash from there.

**Step 8. Navigate.** In RViz, click **2D Goal Pose** and drag on the map. Nav2 plans with Hybrid-A* and drives with MPPI, forwards only, at up to 0.25 m/s (`agos_robot_laptop/agos_robot_nav/config/nav2_robot.yaml`).

**Step 9. Explore by itself.**

```
./agos_laptop explore
```

The explorer drives to frontier after frontier until nothing is left, comes back to the start, and writes a report to `agos_robot_laptop/missions/`. It uses the whole-map mode, which suits a room better than the sewer legs; `./agos_laptop explore legs leg_length:=5.0` runs the proposal's mission instead. Watch the cable: the explorer does not know it is there, so stay close enough to keep it out from under the wheels.

**Step 10. Reuse a map.** Ctrl+C on the Pi saves the map to `agos_robot_pi/maps/rtabmap.db`. Next time, `./agos_robot start localization:=true` finds the robot in that map instead of making a new one. Without that option, every start begins a new map.

## 5. Everyday use

| Where | Command | What |
|---|---|---|
| Pi | `./agos_robot start` | everything on the robot (Ctrl+C stops it and saves the map) |
| Pi | `./agos_robot start localization:=true` | reuse the last map |
| Pi | `./agos_robot start stationary:=true` | keep adding map nodes while standing still |
| Pi | `./agos_robot start camera:=false` | LiDAR only (no camera, no IMU) |
| Pi | `./agos_robot start detection_rate:=0.5` | lighter RTAB-Map if the Pi falls behind |
| Pi | `./agos_robot status` / `stats` | live status / totals of the last run |
| Pi | `./agos_robot stop` | stop everything, the agent included |
| Laptop | `./agos_laptop start` | Nav2 + RViz (2D Goal Pose) |
| Laptop | `./agos_laptop explore` | autonomous exploration |
| Laptop | `./agos_laptop teleop` | keyboard driving (second terminal) |
| Laptop | `./agos_laptop ping` | is the robot there and running? |
| Laptop | `./agos_laptop stop` | stop the laptop side |
| Laptop | `./agos_laptop start <pi-address>` / `start tether` | use Wi-Fi / go back to the cable |

All laptop commands are run in `agos_robot_laptop`. The simulation is untouched: `./agos start`, `./agos explore`, and `./agos_pi sim <pi-address>` in practice_agos work as before. Don't run both at once: each starts a Zenoh router on the laptop's port 7447.

## 6. What you must do yourself

These steps need your hands, your hardware or your accounts. I cannot do them from here, and I could not test any of the code on real hardware.

- **Buy and assemble** the parts in section 2: chassis, motors, L298N, batteries, power bank or 5 V converter, powered USB 3 hub, and USB 3 and USB-C data cables.
- **Wire and solder**, following the tables in section 2. Double-check the common ground and that both ENA/ENB jumpers are off before you switch on. Keep the motor battery off while you wire. Make sure no bare wire can touch the Pi.
- **Measure your robot** (wheel separation, sensor positions, chassis size) and enter the numbers in `robot.yaml`, `config.h` and the footprint in `nav2_robot.yaml`. The values there are placeholders for a typical 2WD kit.
- **Flash the ESP32** from your laptop (section 3b), and flash it again after each change to `config.h`.
- **Install the Pi's operating system**, Docker and the zip, run `./agos_robot setup` (which needs your sudo password), and build the image. The build downloads about 2 GB.
- **Update the Gemini 335's firmware** to 1.8.10 or later with Orbbec Viewer on the laptop, if the earlier sensor guide's step was not done yet.
- **Set up the tether:** buy the cable (and a USB Ethernet adapter if the laptop has no port), run `./agos_robot tether` on the Pi and `./agos_laptop tether` on the laptop (or the PowerShell lines on Windows), and check that the Pi's clock follows the laptop (`chronyc sources`). Fix the cable to the chassis.
- **Run the checks in section 4**, in order. Fix the motor directions and the C1's yaw from what you see, and calibrate `MAX_WHEEL_SPEED` and `MIN_DUTY`.
- **Look after safety:**
  - Test with the wheels off the floor first, and keep a hand near the battery switch.
  - Keep the tether out of the wheels and away from people's feet. Pulling the cable out stops the robot within 0.7 s, so it is also an emergency stop.
  - Charge Li-ion cells only in a proper charger, and never leave them charging unattended.
  - The C1 is a Class 1 laser. Don't look into it from close up for long.
- **Tune for your space.** Start with low speeds. Raise `vx_max` in `nav2_robot.yaml` only once navigation is reliable.

## 7. Troubleshooting

| What you see | What to do |
|---|---|
| ESP32 LED keeps blinking slowly | The agent cannot reach it. `./agos_robot check` must show `/dev/esp32` (use a data cable, run `setup`, replug). `./agos_robot agent-log` should say `session established` |
| LED solid, but the motors never move | `./agos_robot status` must show `ESP32: ok, N commands` with N growing while you drive. If N stays 0, the relay gets nothing: is `./agos_laptop teleop` or Nav2 running, and is the laptop connected (`./agos_laptop ping`)? Is the motor battery on, and are the ENA/ENB jumpers off with the wires on GPIO 5 and 10? |
| Motors stutter while you hold a teleop key | Your keyboard's repeat delay is longer than 0.7 s: raise `CMD_TIMEOUT_MS` in `config.h` |
| A wheel hums but does not turn | Raise `MIN_DUTY` (step 7). The battery may also be flat |
| ESP32 resets or disconnects when the motors start | Motor noise or a voltage dip. Fit the 0.1 µF capacitors, keep the motor wires short and twisted, and power the motors from their own battery, with only GND shared |
| `No /dev/rplidar` | Run `./agos_robot setup`, then replug. `lsusb` must list `10c4:ea60` |
| Camera missing, or `not seen on a 5000M port` | Use the powered USB 3 hub on a blue port and a USB 3 cable. `./agos_robot setup` sets the 128 MB USB buffer (`check` shows it) |
| `./agos_laptop ping`: no answer | Is `./agos_robot start` running? On the cable: do both `check`s see each other (link lights on both sockets; `ip addr` shows 10.77.0.1 and 10.77.0.2)? On Wi-Fi: is the address right (`./agos_robot check`)? On the Pi, `sudo ufw allow 7447/tcp` if ufw is on |
| Tether: no answer, though the cable is in | Re-run `./agos_robot tether` and `./agos_laptop tether`. On the laptop, `nmcli connection show` must list `agos-tether` as active on the Ethernet port. A USB adapter can change its name: `./agos_laptop tether <new-name>`. On Windows, `Get-NetIPAddress -IPAddress 10.77.0.1` must find it |
| The laptop loses the internet when the cable is in | The cable became its default route. `./agos_laptop tether` sets `never-default`; if you set the address by hand, leave the gateway empty |
| RViz: no map, Nav2 waits for map -> base_footprint | RTAB-Map has no data yet. `status` shows the odometry rate and whether it is `LOST`. Drive a little: RTAB-Map adds the first map node once odometry runs |
| `Lookup would require extrapolation` in Nav2 | The clocks disagree. On the Pi, `chronyc sources` should show `^*` on 10.77.0.1 (or on an internet server over Wi-Fi). After a long time unplugged: `sudo chronyc makestep` (section 3d) |
| Odometry `LOST` | The C1 sees too little structure (a big empty room, or long bare corridors) or the robot turned too fast. Drive slower, add furniture or boxes, or restart |
| `THROTTLED`, or RTAB-Map updates over 1000 ms | Add a fan, use a proper 5.1 V 3 A supply, try `detection_rate:=0.5`, or turn off the IR cameras (already off by default) |
| Robot drives in circles under Nav2 | One motor is reversed (step 3), or left and right are swapped |

## 8. What I checked, and what I could not

**Checked here:**

- Every Python file passes flake8 and pep257 with the settings practice_agos's own tests use. The explorer and frontier tests pass in their new home, `agos_robot_nav` (31 passed).
- The Pi's launch file was run with stand-ins for ROS. It makes the right nodes in each mode: mapping and localization, with and without the camera. The URDF has one tree from `base_footprint`. All of RTAB-Map's own parameters are strings.
- The laptop's launch file starts the Zenoh router towards 10.77.0.2 by default, or towards any address or Zenoh endpoint given. With `nav2:=false` it starts only RViz. It refuses to start without Zenoh.
- Its merged Nav2 parameters have the robot's footprint and speeds, and the `Reverse` controller has the robot's limits. Unstamped `/cmd_vel` is kept for the ESP32. Every `use_sim_time` is false, the costmaps' too (the stock file sets them true).
- `./agos_laptop` and `./agos_robot tether` were run with stand-ins for ros2, colcon, nmcli, sudo and chrony: the address handling (tether default, remembered Wi-Fi address, `tether` to go back), the explore modes, `ping`, and both tether set-ups. The netplan file it writes is valid YAML.
- The relay was tested end to end over its local socket, both ways.
- The firmware was compiled against stand-in headers that copy the real rclc, rmw_microros and micro_ros_platformio signatures, for USB and Wi-Fi and for Arduino-ESP32 core 2 and 3. It was then run in a simulated loop:
  - forward, turn and reverse duties come out right;
  - the motors stop after 0.7 s with no command;
  - they stop when the agent is lost;
  - the node reconnects when the agent comes back.
- Both shell scripts pass `bash -n` and shellcheck, apart from its usual note that it cannot follow ROS's setup files. The sllidar patch was run against Slamtec's current CMakeLists.txt.

**Errors found and fixed during the check:**

- The earlier zip lost the scripts' executable bit, so `./agos_robot` would have said `Permission denied`. The zip now keeps it (if a copy still complains: `chmod +x agos_robot agos_laptop`).
- `base_link` was offset from the turning point. Nav2 steers `base_link`, so it now sits straight above `base_footprint`, and only the drawn chassis is offset.
- The firmware could pass a null context when a connection attempt failed halfway. It now checks first.
- Fast DDS shared memory fails between the agent's container (running as root) and the robot's. Both now use UDP only.
- The Nav2 overlay set a `transform_tolerance` on MPPI that the current Nav2 no longer has. It was removed.
- Two docstrings broke the pep257 check. They are fixed.

**Not checked:**

- Nothing ran on real hardware or on real ROS 2 from here. This environment has no Pi, no camera, no ROS packages, no ESP32 toolchain, and no network card to tether.
- The Windows Time server set-up follows Microsoft's documented registry settings, but was not tried against chrony on a Pi from here. If `chronyc sources` on the Pi never shows `^*` on 10.77.0.1, run `w32tm /query /configuration` in PowerShell and check that `NtpServer` is `Enabled: 1`.
- The ROS package index was unreachable from here. The Lyrical package names come from the earlier, verified sensor guide and the Fix thread.
- Whether micro_ros_platformio builds for the ESP32-C3 is not certain. Its board list names the classic ESP32. The library builds with whatever compiler the board uses, and C3 builds are reported to work, but if the build fails, that is the first thing to report back.
- Whether the Pi 4 keeps up with the camera, the LiDAR odometry and RTAB-Map together is measured by `./agos_robot stats`. If it falls behind, use `detection_rate:=0.5` or `camera:=false`.
