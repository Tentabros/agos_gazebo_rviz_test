// AGOS test robot: settings for the ESP32-C3 motor node. Change them here,
// then build and flash again (see platformio.ini).
#pragma once

// ---------------------------------------------------------------- wiring
// ESP32-C3 SuperMini pin -> L298N pin. GPIO 2, 8 and 9 are strapping pins
// (8 is also the on-board LED, 9 the BOOT button), 18/19 are USB and 20/21
// the UART: none of those is used for the motors.
#define PIN_LEFT_EN    5    // -> ENA (remove the ENA jumper on the L298N)
#define PIN_LEFT_IN1   6    // -> IN1
#define PIN_LEFT_IN2   7    // -> IN2
#define PIN_RIGHT_EN  10    // -> ENB (remove the ENB jumper on the L298N)
#define PIN_RIGHT_IN1  3    // -> IN3
#define PIN_RIGHT_IN2  4    // -> IN4
#define PIN_LED        8    // on-board blue LED, lit when the pin is LOW
#define LED_ON       LOW

// A motor that turns the wrong way: set its INVERT to 1 (or swap its two
// wires on the L298N's screw terminals)
#define INVERT_LEFT    0
#define INVERT_RIGHT   0

// ---------------------------------------------------------------- robot
// Distance between the middle of the left and right tyres (m). Measure it;
// it must match wheel_separation in agos_robot_pi/config/robot.yaml.
#define WHEEL_SEPARATION  0.135f
// The wheel's speed (m/s) at full power (MAX_DUTY) on the floor. Measure it:
// see the guide, "Calibrate the motors". Yellow TT motors (1:48) with 65 mm
// wheels do about 0.4-0.6 m/s on a 7.4 V pack through an L298N.
#define MAX_WHEEL_SPEED   0.50f

// ---------------------------------------------------------------- PWM
#define PWM_FREQ_HZ  5000      // the L298N switches cleanly up to ~25 kHz
#define PWM_BITS     10        // duty 0..1023
// Fraction of full power the motors get at most. The TT motors are rated
// 3-6 V and the L298N drops about 2 V: a full 2S Li-ion pack (8.4 V) then
// gives them ~6.4 V at 1.0, so 0.9 keeps them near 6 V. 6 x AA NiMH (7.2 V):
// 1.0. A 9-12 V supply: about 0.6 (9 V) or 0.5 (12 V).
#define MAX_DUTY     0.90f
// Below this a TT motor hums but does not turn (gearbox friction): any
// command that is not zero gets at least this much. Measure it (guide).
#define MIN_DUTY     0.35f
// Commands smaller than this fraction of MAX_WHEEL_SPEED count as stop
#define DEADBAND     0.02f
// The most the duty may change per second (1.0 = from 0 to full in 1 s):
// keeps current spikes and wheel slip down. Nav2 already smooths its speeds.
#define SLEW_PER_S   4.0f

// ---------------------------------------------------------------- safety
// Stop the motors when no /cmd_vel arrived for this long (ms): a crashed
// laptop, a lost Wi-Fi link or a closed teleop window stops the robot.
// teleop_twist_keyboard repeats a held key, so keep this above your
// keyboard's repeat delay (usually 250-600 ms).
#define CMD_TIMEOUT_MS  700

// ---------------------------------------------------------------- micro-ROS
#define NODE_NAME        "esp32_motors"
#define CMD_VEL_TOPIC    "cmd_vel"
#define HEARTBEAT_TOPIC  "esp32/heartbeat"
#define HEARTBEAT_MS     500
// The micro-ROS side's own ROS 2 domain: UROS_DOMAIN_ID in agos_robot_pi/robot.launch.py.
// Not 0, so that nobody else's ROS 2 on the same network can drive the robot.
#define ROS_DOMAIN_ID    77

// Only for the "wifi" env in platformio.ini: the network and the Pi
#define WIFI_SSID        "your-network"
#define WIFI_PASSWORD    "your-password"
#define AGENT_IP         192, 168, 1, 42   // the Pi's address, with commas
#define AGENT_PORT       8888
