#pragma once

#include "drive-straight.h"
#include "performance-config.h"
#include "drive-turn.h"
#include "drive-corner.h"
#include <initializer_list>
#include <cstring>

namespace drive {
enum class push_state { idle, running, done, cancelled, error };
struct push_config {
  double maximum_duration_ms = 5000;
  double stale_ms = 100, max_update_ms = 200;
  double wheel_tolerance_rpm = 5, settle_ms = 100, settle_timeout_ms = 1500;
};
struct push_info {
  push_state state = push_state::idle;
  const char* status = "READY";
  double time_ms = 0, elapsed_ms = 0, duration_ms = 0;
  double left_rpm = 0, right_rpm = 0;
  bool settling = false;
  int left_mV = 0, right_mV = 0;
};

// 机械定位专用定时前推：固定同向电压，不做位置/速度/航向闭环，允许接触后停转。
// 只计算输出；时间到先撤电压，确认停稳后才把底盘交给下一个动作。
class push_controller {
 public:
  void Configure(const push_config& value) { config = value; Reset(); have_frame = have_sample = false; }
  const push_info& Info() const { return info; }
  void Observe(const straight_feedback& value, bool permitted, double now_ms) {
    frame = value; allowed = permitted; now = now_ms; have_frame = true;
    if (value.velocity.valid && std::isfinite(value.velocity.time_ms) &&
        (!have_sample || value.velocity.sequence != sequence)) {
      received_at = now_ms; sequence = value.velocity.sequence; have_sample = true;
    }
  }
  bool StartPushFor(double duration_ms, double volts) {
    if (info.state == push_state::running) return false;
    if (!Positive(config.maximum_duration_ms) || !Positive(config.stale_ms) || !Positive(config.max_update_ms) ||
        !Positive(config.wheel_tolerance_rpm) || !Nonnegative(config.settle_ms) || !Positive(config.settle_timeout_ms) ||
        !Nonnegative(duration_ms) || duration_ms > config.maximum_duration_ms || !Positive(volts) || volts < .001 || volts > 12)
      return Fail("PUSH CONFIG");
    if (!Check()) return false;
    if (!frame.stopped || std::fabs(frame.velocity.left_rpm) > config.wheel_tolerance_rpm ||
        std::fabs(frame.velocity.right_rpm) > config.wheel_tolerance_rpm) return Fail("MOVING");
    info = push_info(); info.duration_ms = duration_ms; info.time_ms = now;
    started_at = previous_time = now; settled_at = -1;
    voltage = static_cast<int>(std::lround(volts * 1000));
    info.state = duration_ms == 0 ? push_state::done : push_state::running;
    info.status = duration_ms == 0 ? "DONE" : "PUSH";
    Update(); return info.state != push_state::error;
  }
  void Update() {
    if (info.state != push_state::running) return;
    if (!Check()) return;
    if (now < previous_time || now - previous_time > config.max_update_ms) { Fail("CLOCK ERR"); return; }
    previous_time = now; info.time_ms = now; info.elapsed_ms = now - started_at;
    info.left_rpm = frame.velocity.left_rpm; info.right_rpm = frame.velocity.right_rpm;
    if (info.elapsed_ms < info.duration_ms) {
      info.left_mV = info.right_mV = voltage; info.status = "PUSH"; return;
    }
    info.left_mV = info.right_mV = 0; info.settling = true; info.status = "PUSH SETTLE";
    const bool stopped = frame.stopped && std::fabs(info.left_rpm) <= config.wheel_tolerance_rpm &&
                         std::fabs(info.right_rpm) <= config.wheel_tolerance_rpm;
    if (!stopped) settled_at = -1;
    else if (settled_at < 0) settled_at = now;
    if (stopped && now - settled_at >= config.settle_ms) {
      info.state = push_state::done; info.status = "DONE"; info.settling = false; return;
    }
    if (info.elapsed_ms >= info.duration_ms + config.settle_timeout_ms) Fail("PUSH STOP TIMEOUT");
  }
  void Cancel() {
    if (info.state == push_state::running) { info.state = push_state::cancelled; info.status = "CANCEL"; }
    info.left_mV = info.right_mV = 0; info.settling = false;
  }
  void Reset() { Cancel(); info = push_info(); }
 private:
  static bool Positive(double value) { return std::isfinite(value) && value > 0; }
  static bool Nonnegative(double value) { return std::isfinite(value) && value >= 0; }
  bool Check() {
    if (!have_frame || !allowed) return Fail("DISABLED");
    if (!Nonnegative(now)) return Fail("CLOCK ERR");
    if (!frame.motors_online) return Fail("MOTOR OFF");
    if (!frame.inertial_ready || !std::isfinite(frame.heading_deg)) return Fail("IMU ERROR");
    if (!frame.position.valid || !std::isfinite(frame.position.left_deg) || !std::isfinite(frame.position.right_deg))
      return Fail("POSITION ERR");
    if (!have_sample || !frame.velocity.valid || !Nonnegative(frame.velocity.time_ms) ||
        !std::isfinite(frame.velocity.left_rpm) || !std::isfinite(frame.velocity.right_rpm) ||
        now < received_at || now - received_at > config.stale_ms) return Fail("SPEED ERR");
    return true;
  }
  bool Fail(const char* reason) {
    info.state = push_state::error; info.status = reason; info.left_mV = info.right_mV = 0; info.settling = false;
    return false;
  }
  push_config config;
  push_info info;
  straight_feedback frame;
  bool allowed = false, have_frame = false, have_sample = false;
  unsigned int sequence = 0;
  int voltage = 0;
  double now = 0, started_at = 0, previous_time = 0, received_at = 0, settled_at = -1;
};
}  // 底盘动作命名空间

namespace route {
constexpr double DEFAULT_DRIVE_RPM = config::ROUTE_DRIVE_RPM;
constexpr double DEFAULT_TURN_RPM = config::ROUTE_TURN_RPM;
constexpr double DEFAULT_PUSH_VOLTS = 4.0;
enum class action { straight, turn, heading, corner, wait, push };
struct command {
  action kind;
  double first, second, third, speed;
};
// 单位统一为 mm、度、rpm、毫秒。正距离前进，正相对角逆时针。
inline command DriveStraight(double distance_mm, double maximum_rpm = DEFAULT_DRIVE_RPM) {
  return {action::straight, distance_mm, 0, 0, maximum_rpm};
}
inline command Turn(double angle_deg, double maximum_rpm = DEFAULT_TURN_RPM) {
  return {action::turn, angle_deg, 0, 0, maximum_rpm};
}
inline command TurnToHeading(double heading_deg, double maximum_rpm = DEFAULT_TURN_RPM) {
  return {action::heading, heading_deg, 0, 0, maximum_rpm};
}
// 一条动作连续通过直线、圆角、直线；两段独立直行加 Turn 会在中间停稳。
inline command DriveCorner(double before_mm, double angle_deg, double after_mm,
                           double maximum_rpm = DEFAULT_DRIVE_RPM) {
  return {action::corner, before_mm, angle_deg, after_mm, maximum_rpm};
}
inline command Wait(double milliseconds) { return {action::wait, milliseconds, 0, 0, 0}; }
// 只按时间前推用于机械定位；第二项可选，单位V；零时长跳过，单次最多5秒。
inline command PushFor(double milliseconds, double volts = DEFAULT_PUSH_VOLTS) {
  return {action::push, milliseconds, volts, 0, 0};
}
}  // 路线动作命名空间

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
