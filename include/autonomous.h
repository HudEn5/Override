#pragma once

#include "drive-straight.h"
#include "drive-turn.h"
#include "drive-corner.h"

enum class autonomous_motion { none, straight, turn, corner };

struct autonomous_info {
  bool active = false;
  bool finished = false;
  unsigned int run = 0;
  unsigned int step = 0;
  const char* status = "IDLE";
  drive::straight_info drive;
  drive::turn_info turn;
  drive::corner_info corner;
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
    Chassis.Configure(value); Turn.Configure(turn);
  }
  void Configure(const drive::straight_config& value, const drive::turn_config& turn, const drive::corner_config& corner) {
    Configure(value, turn); Corner.Configure(corner);
  }
  void Update(bool permitted, const drive::straight_feedback& feedback, double now);
  bool StartDriveStraight(double distance_mm, double maximum_rpm) {
    if (!info.active || Busy()) return false;
    info.motion = autonomous_motion::straight;
    return Chassis.StartDriveStraight(distance_mm, maximum_rpm);
  }
  bool StartTurnRelative(double angle_deg, double maximum_rpm) {
    if (!info.active || Busy()) return false;
    info.motion = autonomous_motion::turn;
    return Turn.StartTurnRelative(angle_deg, maximum_rpm);
  }
  bool StartTurnToHeading(double heading_deg, double maximum_rpm) {
    if (!info.active || Busy()) return false;
    info.motion = autonomous_motion::turn;
    return Turn.StartTurnToHeading(heading_deg, maximum_rpm);
  }
  drive::straight_state DriveStraightState() const { return Chassis.Info().state; }
  bool StartDriveCorner(double before_mm, double angle_deg, double after_mm, double maximum_rpm) {
    if (!info.active || Busy()) return false;
    info.motion = autonomous_motion::corner;
    return Corner.StartDriveCorner(before_mm, angle_deg, after_mm, maximum_rpm);
  }
  drive::corner_state CornerState() const { return Corner.Info().state; }
  void CancelCorner() { Corner.CancelCorner(); }
  drive::turn_state TurnState() const { return Turn.Info().state; }
  void CancelDriveStraight() { Chassis.CancelDriveStraight(); }
  void CancelTurn() { Turn.CancelTurn(); }
  const autonomous_info& Info() const { return info; }

 private:
  void RunRoutine();  // 比赛路线写在 src/route.cpp，默认空路线。
  void FinishRoutine(const char* status = "DONE") {
    Chassis.CancelDriveStraight(); Turn.CancelTurn(); Corner.CancelCorner();
    info.active = false; info.finished = true; info.status = status;
  }
  bool Busy() const {
    return Chassis.Info().state == drive::straight_state::running || Turn.Info().state == drive::turn_state::running ||
           Corner.Info().state == drive::corner_state::running;
  }
  void Publish();
  drive::straight_controller Chassis;
  drive::turn_controller Turn;
  drive::corner_controller Corner;
  autonomous_info info;
  bool previously_permitted = false;
  bool waiting_ready = false;
};
