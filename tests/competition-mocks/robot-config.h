#pragma once
#include <mutex>

namespace vex {
using mutex = std::mutex;
class competition {
 public:
  static bool bStopAllTasksBetweenModes;
  static bool bStopTasksBetweenModes;
  void (*auto_handler)() = nullptr;
  void (*driver_handler)() = nullptr;
  void autonomous(void (*fn)()) { auto_handler = fn; }
  void drivercontrol(void (*fn)()) { driver_handler = fn; }
};
}
extern vex::competition Competition;
