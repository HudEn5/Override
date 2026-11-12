#include "competition-control.h"
#include "robot-config.h"

namespace {
vex::mutex phase_mutex;
competition_request request;
void Request(runtime::competition_stage stage) {
  phase_mutex.lock();
  request.stage = stage;
  ++request.generation;
  phase_mutex.unlock();
}
}  // 匿名命名空间

void autonomous() { Request(runtime::competition_stage::autonomous); }
void usercontrol() { Request(runtime::competition_stage::driver); }

void RegisterCompetitionCallbacks() {
  // 常驻控制/采样线程由 main 管理，比赛回调本身不创建子线程。
  vex::competition::bStopAllTasksBetweenModes = false;
  vex::competition::bStopTasksBetweenModes = false;
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
}
competition_request ReadCompetitionRequest() {
  phase_mutex.lock();
  const competition_request value = request;
  phase_mutex.unlock();
  return value;
}
