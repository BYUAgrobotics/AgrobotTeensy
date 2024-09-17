#include "battery_pub.h"
#include "range_pub.h"

#include <SoftwareSerial.h>
#include <agrobot_interfaces/msg/drive_command.h>
#include <agrobot_interfaces/msg/battery_status.h>
#include <agrobot_interfaces/msg/range_data.h>

#define ENABLE_DRIVE
// #define ENABLE_BATTERY
#define ENABLE_RANGE
#define ENABLE_BT_DEBUG

#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

// micro-ROS config values
#define BAUD_RATE 6000000
#define CALLBACK_TOTAL 1
#define SYNC_TIMEOUT 1000

// hardware pin values
#define BT_MC_RX 34
#define BT_MC_TX 35
#define VOLT_PIN 18
#define CURRENT_PIN 17
#define LED_PIN 13

// sensor baud rates
#define BT_DEBUG_RATE 9600

// sensor update rates
#define BATTERY_MS 1000 // (arbitrary) fastest update speed is 1 Hz
#define RANGE_MS 1000

// time of last received command (used as a fail safe)
unsigned long last_received = 0;

// micro-ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// message objects
agrobot_interfaces__msg__DriveCommand command_msg;

// subscriber objects
rcl_subscription_t command_sub;

// publisher objects
BatteryPub battery_pub;
RangePub range_pub;

// sensor objects
SoftwareSerial BTSerial(BT_MC_RX, BT_MC_TX);

// states for state machine in loop function
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} static state;

void error_loop() {
  while (1) {
    delay(100);
  }
}

// micro-ROS function that subscribes to requested command values
void cmd_sub_callback(const void *msgin) {

  last_received = millis();

  const agrobot_interfaces__msg__DriveCommand *msg =
      (const agrobot_interfaces__msg__DriveCommand *)msgin;

#ifdef ENABLE_DRIVE
  // TODO: write the motor values to the motors
#endif

#ifdef ENABLE_BT_DEBUG
  BTSerial.println(
      "FR: " + String(msg->fr_motor) + " FL: " + String(msg->fl_motor) +
      " BR: " + String(msg->br_motor) + " BL: " + String(msg->bl_motor));
#endif
}

bool create_entities() {

  // the allocator object wraps the dynamic memory allocation and deallocation
  // methods used in micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // synchronize timestamps with the Raspberry Pi
  // after sync, timing should be able to be accessed with "rmw_uros_epoch"
  // functions
  RCCHECK(rmw_uros_sync_session(SYNC_TIMEOUT));

  #ifdef ENABLE_BT_DEBUG
  if (!rmw_uros_epoch_synchronized()) {
    BTSerial.println("ERROR: Could not synchronize timestamps with agent");
  } else {
    BTSerial.println("ALERT: Timestamps synchronized with agent");
  }
  #endif

  // create publishers
  battery_pub.setup(node);
  range_pub.setup(node);

  // create subscribers
  RCCHECK(rclc_subscription_init_default(
      &command_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, DriveCommand),
      "drive_command"));

  // create executor
  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL,
                                 &allocator));

  // add callbacks to executor
  RCSOFTCHECK(
      rclc_executor_add_subscription(&executor, &command_sub, &command_msg,
                                     &cmd_sub_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // destroy publishers
  battery_pub.destroy(node);
  range_pub.destroy(node);

  // destroy everything else
  rcl_subscription_fini(&command_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {

  Serial.begin(BAUD_RATE);
  set_microros_serial_transports(Serial);

  // set up the indicator light
  pinMode(LED_PIN, OUTPUT);

  //////////////////////////////////////////////////////////
  // SENSOR SETUP CODE STARTS HERE
  // - Use the #define statements at the top of this file to
  //   enable and disable each sensor
  //////////////////////////////////////////////////////////

#ifdef ENABLE_DRIVE
  // TODO: motor setup code goes here
#endif

#ifdef ENABLE_BT_DEBUG
  BTSerial.begin(BT_DEBUG_RATE);
#endif

#ifdef ENABLE_BATTERY
  pinMode(CURRENT_PIN, INPUT);
  pinMode(VOLT_PIN, INPUT);
#endif

#ifdef ENABLE_RANGE
  // TODO: range sensor setup code goes here
#endif

  //////////////////////////////////////////////////////////
  // SENSOR SETUP CODE ENDS HERE
  //////////////////////////////////////////////////////////

  state = WAITING_AGENT;
}

//////////////////////////////////////////////////////////
// SENSOR VARIABLE UPDATE CODE STARTS HERE
// - Use the #define statements at the top of this file to
//   enable and disable each sensor
//////////////////////////////////////////////////////////

void read_battery() {

  // we did some testing to determine the below params, but
  // it's possible they are not completely accurate
  float voltage = (analogRead(VOLT_PIN) * 0.03437) + 0.68;
  float current = (analogRead(CURRENT_PIN) * 0.122) - 11.95;

  // publish the battery data
  battery_pub.publish(voltage, current);
}

void read_range() {
  // TODO: range sensor read code goes here

  // publish the range data
  range_pub.publish(0, 0, 0, 0);
}

//////////////////////////////////////////////////////////
// SENSOR VARIABLE UPDATE CODE ENDS HERE
//////////////////////////////////////////////////////////

void loop() {

  // blink the indicator light
  if (millis() % 1000 < 250) {
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  // fail safe for agent disconnect
  if (millis() - last_received > 2000) {

#ifdef ENABLE_DRIVE
    // TODO: write default values to the motors
#endif

#ifdef ENABLE_BT_DEBUG
    BTSerial.println("ALERT: No command received in 2 seconds, shutting down");
#endif
  }

  // state machine to manage connecting and disconnecting the micro-ROS agent
  switch (state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_AVAILABLE
                                        : WAITING_AGENT;);
    break;

  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    };
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_CONNECTED
                                        : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED) {

      //////////////////////////////////////////////////////////
      // EXECUTES WHEN THE AGENT IS CONNECTED
      //////////////////////////////////////////////////////////

#ifdef ENABLE_BATTERY
      EXECUTE_EVERY_N_MS(BATTERY_MS, read_battery());
#endif

#ifdef ENABLE_RANGE
      EXECUTE_EVERY_N_MS(RANGE_MS, read_range());
#endif

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

      //////////////////////////////////////////////////////////
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    break;
  }
}
