#include "range_pub.h"

void RangePub::setup(rcl_node_t node) {

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, RangeData),
      "range_data"));
}

void RangePub::publish(float front_us, float back_us, float left_us, float right_us) {

  msg.front_us = front_us;
  msg.back_us = back_us;
  msg.left_us = left_us;
  msg.right_us = right_us;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}