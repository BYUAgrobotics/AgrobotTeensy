#ifndef BATTERY_PUB
#define BATTERY_PUB

#include "publisher.h"
#include <Wire.h>
#include <agrobot_interfaces/msg/battery_status.h>

class BatteryPub : Publisher {

public:
  void setup(rcl_node_t node);
  void publish(float voltage, float current);
  using Publisher::destroy;

private:
  agrobot_interfaces__msg__BatteryStatus msg;
};

#endif // BATTERY_PUB