#pragma once

#include "drive-straight.h"
#include "drive-turn.h"
#include "drive-corner.h"
#include "drive-push.h"
#include "route-commands.h"
#include <initializer_list>
#include <cstring>

enum class autonomous_motion { none, straight, turn, corner, push };

struct autonomous_info {
  bool active = false;
  bool finished = false;
  unsigned int run = 0;
  unsigned int step = 0;
  const char* status = "IDLE";
  double time_ms = 0;
  bool waiting = false;
  int result = 0;  // 0 未开始，1 运行，2 正常完成，3 取消，4 故障。
  drive::straight_info drive;
  drive::turn_info turn;
  drive::corner_info corner;
  drive::push_info push;
  autonomous_motion motion = autonomous_motion::none;
  int left_mV = 0, right_mV = 0;
};

// 比赛路线入口，只由主线程调用，不读取手柄，不创建电机写入线程。
class autonomous_program {
 public:
  void Configure(const drive::straight_config& value) {
    drive::turn_config turn;
    turn.pid = value.pid;
    turn.left_forward = value.left_forward; turn.right_forward = value.right_forward;
    turn.left_reverse = value.left_reverse; turn.right_reverse = value.right_reverse;
    Configure(value, turn);
  }
  void Configure(const drive::straight_config& value, const drive::turn_config& turn) {
    Chassis.Configure(value); Turning.Configure(turn);
    Pushing.Configure(drive::push_config());
  }
  void Configure(const drive::straight_config& value, const drive::turn_config& turn, const drive::corner_config& corner) {
    Configure(value, turn); Cornering.Configure(corner);
  }
  void Update(bool permitted, const drive::straight_feedback& feedback, double now);
  void CancelRoutine() { if (info.active) FinishRoutine("CANCEL"); }
  bool StartDriveStraight(double distance_mm, double maximum_rpm) {
    if (!info.active || Busy()) return false;
    info.motion = autonomous_motion::straight;
    return Chassis.StartDriveStraight(distance_mm, maximum_rpm);
  }
  bool StartTurnRelative(double angle_deg, double maximum_rpm) {
    if (!info.active || Busy()) return false;
    info.motion = autonomous_motion::turn;
    return Turning.StartTurnRelative(angle_deg, maximum_rpm);
  }
  bool StartTurnToHeading(double heading_deg, double maximum_rpm) {
    if (!info.active || Busy()) return false;
    info.motion = autonomous_motion::turn;
    return Turning.StartTurnToHeading(heading_deg, maximum_rpm);
  }
  drive::straight_state DriveStraightState() const { return Chassis.Info().state; }
  bool StartDriveCorner(double before_mm, double angle_deg, double after_mm, double maximum_rpm) {
    if (!info.active || Busy()) return false;
    info.motion = autonomous_motion::corner;
    return Cornering.StartDriveCorner(before_mm, angle_deg, after_mm, maximum_rpm);
  }
  drive::corner_state CornerState() const { return Cornering.Info().state; }
  void CancelCorner() { Cornering.CancelCorner(); }
  bool StartPushFor(double milliseconds, double volts = route::DEFAULT_PUSH_VOLTS) {
    if (!info.active || Busy()) return false;
    info.motion = autonomous_motion::push;
    return Pushing.StartPushFor(milliseconds, volts);
  }
  drive::push_state PushState() const { return Pushing.Info().state; }
  drive::turn_state TurnState() const { return Turning.Info().state; }
  void CancelDriveStraight() { Chassis.CancelDriveStraight(); }
  void CancelTurn() { Turning.CancelTurn(); }
  const autonomous_info& Info() const { return info; }

 private:
  void RunRoutine();  // 只在 src/route.cpp 编排动作，同一份路线用于实车调试和比赛。
  void RunRoute(std::initializer_list<route::command> commands);
  void FinishRoutine(const char* status = "DONE") {
    Chassis.CancelDriveStraight(); Turning.CancelTurn(); Cornering.CancelCorner();
    Pushing.Cancel();
    info.active = false; info.finished = true; info.status = status;
    info.waiting = false;
    info.result = !std::strcmp(status, "DONE") || !std::strcmp(status, "EMPTY") ? 2 :
        !std::strcmp(status, "CANCEL") || !std::strcmp(status, "DISABLED") ? 3 : 4;
  }
  bool Busy() const {
    return Chassis.Info().state == drive::straight_state::running || Turning.Info().state == drive::turn_state::running ||
           Cornering.Info().state == drive::corner_state::running || Pushing.Info().state == drive::push_state::running;
  }
  void Publish();
  drive::straight_controller Chassis;
  drive::turn_controller Turning;
  drive::corner_controller Cornering;
  drive::push_controller Pushing;
  autonomous_info info;
  bool previously_permitted = false;
  bool waiting_ready = false;
  bool route_step_started = false;
  double route_time = 0, wait_started = 0;
};
