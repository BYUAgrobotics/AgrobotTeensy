#ifndef PUBLISHER
#define PUBLISHER

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

class Publisher {

public:
  // this needs to be defined by each publisher
  virtual void setup(rcl_node_t node) = 0;

  void destroy(rcl_node_t node) {
    RCCHECK(rcl_publisher_fini(&publisher, &node));
  }

protected:
  rcl_publisher_t publisher;

  void error_loop() {
    while (1) {
      delay(100);
    }
  }
};

#endif // PUBLISHER