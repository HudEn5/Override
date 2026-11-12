#pragma once
#include "drive-control.h"
#include <mutex>

// 离线测试替身：用宿主机互斥锁替代 VEX 锁，不连接或控制实车。
namespace vex {
using mutex = std::mutex;
enum timeUnits { msec };
class timer {
 public:
  double time(timeUnits);
};
void wait(double, timeUnits);
}
namespace hardware {
drive::feedback ReadDriveFeedback();
}
