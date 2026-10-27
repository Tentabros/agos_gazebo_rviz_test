// AGOS test robot: micro-ROS motor node for an ESP32-C3 SuperMini driving two
// DC motors through an L298N.
//
//   /cmd_vel          geometry_msgs/Twist   in:  linear.x (m/s), angular.z (rad/s)
//   /esp32/heartbeat  std_msgs/Int32        out: /cmd_vel messages received so far
//
// It reaches ROS 2 through the micro-ROS agent on the Raspberry Pi (USB
// serial, or Wi-Fi with the "wifi" env), which agos_robot starts. The motors
// stop when no /cmd_vel came for CMD_TIMEOUT_MS, and whenever the agent is
// lost; the node reconnects by itself when the agent comes back.
//
// LED: blinking slowly = looking for the agent, on = connected,
//      flickering = connected and driving.
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

#include "config.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL) && !defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#error "Build with the serial or wifi env (platformio.ini)"
#endif

// ------------------------------------------------------------------ motors

struct Motor {
  int en, in1, in2, invert, channel;
  float duty;  // -1..1, what the motor gets now (after the slew limit)
};

static Motor left_motor = {PIN_LEFT_EN, PIN_LEFT_IN1, PIN_LEFT_IN2, INVERT_LEFT, 0, 0.0f};
static Motor right_motor = {PIN_RIGHT_EN, PIN_RIGHT_IN1, PIN_RIGHT_IN2, INVERT_RIGHT, 1, 0.0f};
static const uint32_t PWM_MAX = (1u << PWM_BITS) - 1;

static void pwm_attach(const Motor & m) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(m.en, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(m.channel, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(m.en, m.channel);
#endif
}

static void pwm_write(const Motor & m, uint32_t value) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(m.en, value);
#else
  ledcWrite(m.channel, value);
#endif
}

static void motor_setup(Motor & m) {
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  digitalWrite(m.in1, LOW);
  digitalWrite(m.in2, LOW);
  pwm_attach(m);
  pwm_write(m, 0);
  m.duty = 0.0f;
}

// Drive one motor at duty -1..1 (sign = direction). 0 lets it coast.
static void motor_output(const Motor & m) {
  float d = m.invert ? -m.duty : m.duty;
  if (d > 0.0f) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (d < 0.0f) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  } else {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
  }
  pwm_write(m, (uint32_t)(fabsf(d) * PWM_MAX + 0.5f));
}

// Wheel speed (m/s) -> duty: 0 below the deadband, else MIN_DUTY..MAX_DUTY
static float speed_to_duty(float speed) {
  float f = speed / MAX_WHEEL_SPEED;
  if (f > 1.0f) f = 1.0f;
  if (f < -1.0f) f = -1.0f;
  if (fabsf(f) < DEADBAND) return 0.0f;
  float mag = MIN_DUTY + (MAX_DUTY - MIN_DUTY) * fabsf(f);
  return f > 0.0f ? mag : -mag;
}

// Move the motor's duty towards target by at most max_step. Crossing zero
// (a change of direction) goes through 0 first.
static void slew(Motor & m, float target, float max_step) {
  if ((m.duty > 0.0f && target < 0.0f) || (m.duty < 0.0f && target > 0.0f)) target = 0.0f;
  float diff = target - m.duty;
  if (diff > max_step) diff = max_step;
  if (diff < -max_step) diff = -max_step;
  m.duty += diff;
  // a running motor below MIN_DUTY only hums: jump the rest of the way
  if (target == 0.0f && fabsf(m.duty) < MIN_DUTY) m.duty = 0.0f;
  if (target != 0.0f && fabsf(m.duty) < MIN_DUTY) m.duty = target > 0.0f ? MIN_DUTY : -MIN_DUTY;
}

static void motors_stop_now() {
  left_motor.duty = 0.0f;
  right_motor.duty = 0.0f;
  motor_output(left_motor);
  motor_output(right_motor);
}

// ------------------------------------------------------------------ micro-ROS

#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { (void)(fn); }

static rclc_support_t support;
static rcl_init_options_t init_options;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_subscription_t cmd_sub;
static rcl_publisher_t heartbeat_pub;
static rcl_timer_t heartbeat_timer;
static rclc_executor_t executor;

static geometry_msgs__msg__Twist cmd_msg;
static std_msgs__msg__Int32 heartbeat_msg;

static volatile float target_left = 0.0f;   // wheel speeds wanted (m/s)
static volatile float target_right = 0.0f;
static uint32_t last_cmd_ms = 0;
static bool have_cmd = false;
static int32_t cmd_count = 0;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState state = WAITING_AGENT;

static void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float v = (float)msg->linear.x;
  float w = (float)msg->angular.z;
  // differential drive: each wheel = forward speed -/+ turn rate x half the track
  target_left = v - w * WHEEL_SEPARATION * 0.5f;
  target_right = v + w * WHEEL_SEPARATION * 0.5f;
  last_cmd_ms = millis();
  have_cmd = true;
  cmd_count++;
}

static void heartbeat_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    heartbeat_msg.data = cmd_count;
    RCSOFTCHECK(rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL));
  }
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
  // best effort: an old speed command is worthless, never resend it
  RCCHECK(rclc_subscription_init_best_effort(
      &cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), CMD_VEL_TOPIC));
  RCCHECK(rclc_publisher_init_best_effort(
      &heartbeat_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), HEARTBEAT_TOPIC));
  RCCHECK(rclc_timer_init_default2(
      &heartbeat_timer, &support, RCL_MS_TO_NS(HEARTBEAT_MS), heartbeat_callback, true));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_vel_callback,
                                         ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &heartbeat_timer));
  return true;
}

static void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  // the agent is gone: do not wait for it to confirm each deletion
  if (rmw_context != NULL) {
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }
  RCSOFTCHECK(rcl_subscription_fini(&cmd_sub, &node));
  RCSOFTCHECK(rcl_publisher_fini(&heartbeat_pub, &node));
  RCSOFTCHECK(rcl_timer_fini(&heartbeat_timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
  RCSOFTCHECK(rcl_init_options_fini(&init_options));
}

// ------------------------------------------------------------------ main

static void led(bool on) { digitalWrite(PIN_LED, on ? LED_ON : !LED_ON); }

void setup() {
  pinMode(PIN_LED, OUTPUT);
  led(false);
  motor_setup(left_motor);
  motor_setup(right_motor);

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
  static char ssid[] = WIFI_SSID;
  static char password[] = WIFI_PASSWORD;
  IPAddress agent_ip(AGENT_IP);
  set_microros_wifi_transports(ssid, password, agent_ip, AGENT_PORT);
#else
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
#endif
  delay(500);
  state = WAITING_AGENT;
}

void loop() {
  static uint32_t last_ping = 0;
  static uint32_t last_motor = 0;
  uint32_t now = millis();

  switch (state) {
    case WAITING_AGENT:
      motors_stop_now();
      led((now / 500) % 2);                     // slow blink
      if (now - last_ping > 500) {
        last_ping = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
      }
      break;
    case AGENT_AVAILABLE:
      if (create_entities()) {
        state = AGENT_CONNECTED;
        have_cmd = false;
      } else {
        destroy_entities();
        state = WAITING_AGENT;
      }
      break;
    case AGENT_CONNECTED:
      if (now - last_ping > 1000) {
        last_ping = now;
        if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
          state = AGENT_DISCONNECTED;
          break;
        }
      }
      RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
      break;
    case AGENT_DISCONNECTED:
      motors_stop_now();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }

  // Motors, every 10 ms
  now = millis();
  if (state == AGENT_CONNECTED && now - last_motor >= 10) {
    float dt = (now - last_motor) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    last_motor = now;
    bool fresh = have_cmd && (now - last_cmd_ms) < CMD_TIMEOUT_MS;
    float tl = fresh ? speed_to_duty(target_left) : 0.0f;
    float tr = fresh ? speed_to_duty(target_right) : 0.0f;
    if (!fresh) {
      // timed out: stop at once, not along the slew ramp
      motors_stop_now();
    } else {
      slew(left_motor, tl, SLEW_PER_S * dt);
      slew(right_motor, tr, SLEW_PER_S * dt);
      motor_output(left_motor);
      motor_output(right_motor);
    }
    bool moving = left_motor.duty != 0.0f || right_motor.duty != 0.0f;
    led(moving ? (now / 60) % 2 : true);
  } else if (state != AGENT_CONNECTED) {
    last_motor = now;
  }
}
