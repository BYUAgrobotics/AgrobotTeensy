#ifndef RANGE_PUB
#define RANGE_PUB

#include "publisher.h"
#include <Wire.h>
#include <agrobot_interfaces/msg/range_data.h>

class RangePub : Publisher {

public:
  void setup(rcl_node_t node);
  void publish(float front_us, float back_us, float left_us, float right_us);
  using Publisher::destroy;

private:
  agrobot_interfaces__msg__RangeData msg;
};

#endif // RANGE_PUB