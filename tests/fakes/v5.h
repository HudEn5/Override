#pragma once

// 电脑端验证使用；真实 V5 构建不包含此目录。
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace vex {
enum class directionType { fwd, rev };
enum class gearSetting { ratio6_1, ratio18_1, ratio36_1 };
enum class velocityUnits { rpm };
enum class voltageUnits { mV };
enum class rotationUnits { deg };
enum class timeUnits { sec, msec };
enum class brakeType { coast, brake, hold };
enum controllerType { primary };
const directionType fwd = directionType::fwd;
const gearSetting ratio6_1 = gearSetting::ratio6_1;
const gearSetting ratio18_1 = gearSetting::ratio18_1;
const gearSetting ratio36_1 = gearSetting::ratio36_1;
const velocityUnits rpm = velocityUnits::rpm;
const rotationUnits deg = rotationUnits::deg;
const timeUnits sec = timeUnits::sec;
const brakeType coast = brakeType::coast, brake = brakeType::brake, hold = brakeType::hold;

struct Simulation {
  double now = 0;
  bool field = false, enabled = true, autonomous = true;
};
inline Simulation& simulation() { static Simulation state; return state; }

class motor {
 public:
  double encoder = 0, commandRpm = 0, responseScale = 1;
  bool connected = true, badFeedback = false;
  int spinCalls = 0, stopCalls = 0, resetCalls = 0, voltage = 0;
  brakeType lastStop = coast, stopping = coast;
  motor(int, gearSetting, bool) {}
  bool installed() const { return connected; }
  double position(rotationUnits) const {
    return badFeedback ? std::numeric_limits<double>::quiet_NaN() : encoder;
  }
  double velocity(velocityUnits) const { return commandRpm * responseScale; }
  void setStopping(brakeType mode) { stopping = mode; }
  void stop() { stop(stopping); }
  void stop(brakeType mode) { lastStop = mode; commandRpm = 0; voltage = 0; ++stopCalls; }
  void setPosition(double value, rotationUnits) { encoder = value; ++resetCalls; }
  void spin(directionType direction, double value, velocityUnits) {
    commandRpm = (direction == fwd ? value : -value); ++spinCalls;
  }
  void spin(directionType, int value, voltageUnits) { voltage = value; ++spinCalls; }
};

class triport { public: struct port {}; port A, B, C, D, E, F, G, H; };
class digital_out {
 public:
  bool value = false;
  explicit digital_out(triport::port&) {}
  void set(bool state) { value = state; }
};
class brain {
 public:
  triport ThreeWirePort;
  struct screen {
    void setCursor(int, int) {}
    template <typename... Args> void print(const char*, Args...) {}
  } Screen;
};
class controller {
 public:
  explicit controller(controllerType) {}
  struct axis { int value = 0; int position() const { return value; } } Axis1, Axis2, Axis3, Axis4;
  struct button { bool value = false; bool pressing() const { return value; } };
  button ButtonR1, ButtonR2, ButtonL1, ButtonL2, ButtonA, ButtonB, ButtonX, ButtonY;
  button ButtonUp, ButtonDown, ButtonLeft, ButtonRight;
};
class competition {
 public:
  bool isFieldControl() const { return simulation().field; }
  bool isCompetitionSwitch() const { return false; }
  bool isEnabled() const { return simulation().enabled; }
  bool isAutonomous() const { return simulation().autonomous; }
};
class timer {
  double started_ = simulation().now;
 public:
  double time(timeUnits unit) const {
    return (simulation().now - started_) * (unit == sec ? 1 : 1000);
  }
};
}  // namespace vex
