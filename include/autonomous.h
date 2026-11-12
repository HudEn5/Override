#pragma once

#include "drive-straight.h"

struct autonomous_info {
  bool active = false;
  bool finished = false;
  unsigned int run = 0;
  unsigned int step = 0;
  const char* status = "IDLE";
  drive::straight_info drive;
};

// 比赛路线入口，只由主线程调用，不读取手柄，不创建电机写入线程。
class autonomous_program {
 public:
  void Configure(const drive::straight_config& value) { Chassis.Configure(value); }
  void Update(bool permitted, const drive::straight_feedback& feedback, double now);
  bool StartDriveStraight(double distance_mm, double maximum_rpm) {
    return info.active && Chassis.StartDriveStraight(distance_mm, maximum_rpm);
  }
  drive::straight_state DriveStraightState() const { return Chassis.Info().state; }
  void CancelDriveStraight() { Chassis.CancelDriveStraight(); }
  const autonomous_info& Info() const { return info; }

 private:
  void RunRoutine();  // 比赛路线写在 src/route.cpp，默认空路线。
  void FinishRoutine(const char* status = "DONE") {
    Chassis.CancelDriveStraight(); info.active = false; info.finished = true; info.status = status;
  }
  drive::straight_controller Chassis;
  autonomous_info info;
  bool previously_permitted = false;
  bool waiting_ready = false;
};
