//////////////////////////////////////////////////////////
// File: battery_pub.cpp
// Created by Nelson Durrant, Sep 2024
//////////////////////////////////////////////////////////

#include "battery_pub.h"

void BatteryPub::setup(rcl_node_t node) {

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(agrobot_interfaces, msg, BatteryStatus),
      "battery_data"));
}

void BatteryPub::publish(float voltage, float current) {

  msg.voltage = voltage;
  msg.current = current;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}