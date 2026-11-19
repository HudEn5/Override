#pragma once

#include "drive-straight.h"
#include "performance-config.h"
#include "drive-turn.h"
#include "drive-corner.h"
#include "drive-rolling.h"
#include <initializer_list>
#include <cstring>

namespace drive {
using push_state = motion_state;
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
  void Configure(const push_config& value) {
    config = value;
    Reset();
    have_frame = have_sample = false;
  }
  const push_info& Info() const {
    return info;
  }
  void Observe(const straight_feedback& value, bool permitted, double now_ms) {
    frame = value;
    allowed = permitted;
    now = now_ms;
    have_frame = true;
    if (value.velocity.valid && std::isfinite(value.velocity.time_ms) &&
        (!have_sample || value.velocity.sequence != sequence)) {
      received_at = now_ms;
      sequence = value.velocity.sequence;
      have_sample = true;
    }
  }
  bool StartPushFor(double duration_ms, double volts) {
    if (info.state == push_state::running)
      return false;
    if (!Positive(config.maximum_duration_ms) || !Positive(config.stale_ms) ||
        !Positive(config.max_update_ms) || !Positive(config.wheel_tolerance_rpm) ||
        !Nonnegative(config.settle_ms) || !Positive(config.settle_timeout_ms) || !Nonnegative(duration_ms) ||
        duration_ms > config.maximum_duration_ms || !Positive(volts) || volts < .001 || volts > 12)
      return Fail("PUSH CONFIG");
    if (!Check())
      return false;
    if (!frame.stopped || std::fabs(frame.velocity.left_rpm) > config.wheel_tolerance_rpm ||
        std::fabs(frame.velocity.right_rpm) > config.wheel_tolerance_rpm)
      return Fail("MOVING");
    info = push_info();
    info.duration_ms = duration_ms;
    info.time_ms = now;
    started_at = previous_time = now;
    settled_at = -1;
    voltage = static_cast<int>(std::lround(volts * 1000));
    info.state = duration_ms == 0 ? push_state::done : push_state::running;
    info.status = duration_ms == 0 ? "DONE" : "PUSH";
    Update();
    return info.state != push_state::error;
  }
  void Update() {
    if (info.state != push_state::running)
      return;
    if (!Check())
      return;
    if (now < previous_time || now - previous_time > config.max_update_ms) {
      Fail("CLOCK ERR");
      return;
    }
    previous_time = now;
    info.time_ms = now;
    info.elapsed_ms = now - started_at;
    info.left_rpm = frame.velocity.left_rpm;
    info.right_rpm = frame.velocity.right_rpm;
    if (info.elapsed_ms < info.duration_ms) {
      info.left_mV = info.right_mV = voltage;
      info.status = "PUSH";
      return;
    }
    info.left_mV = info.right_mV = 0;
    info.settling = true;
    info.status = "PUSH SETTLE";
    const bool stopped = frame.stopped && std::fabs(info.left_rpm) <= config.wheel_tolerance_rpm &&
                         std::fabs(info.right_rpm) <= config.wheel_tolerance_rpm;
    if (!stopped)
      settled_at = -1;
    else if (settled_at < 0)
      settled_at = now;
    if (stopped && now - settled_at >= config.settle_ms) {
      info.state = push_state::done;
      info.status = "DONE";
      info.settling = false;
      return;
    }
    if (info.elapsed_ms >= info.duration_ms + config.settle_timeout_ms)
      Fail("PUSH STOP TIMEOUT");
  }
  void Cancel() {
    if (info.state == push_state::running) {
      info.state = push_state::cancelled;
      info.status = "CANCEL";
    }
    info.left_mV = info.right_mV = 0;
    info.settling = false;
  }
  void Reset() {
    Cancel();
    info = push_info();
  }

 private:
  static bool Positive(double value) {
    return std::isfinite(value) && value > 0;
  }
  static bool Nonnegative(double value) {
    return std::isfinite(value) && value >= 0;
  }
  bool Check() {
    if (!have_frame || !allowed)
      return Fail("DISABLED");
    if (!Nonnegative(now))
      return Fail("CLOCK ERR");
    if (!frame.motors_online)
      return Fail("MOTOR OFF");
    if (!frame.inertial_ready || !std::isfinite(frame.heading_deg))
      return Fail("IMU ERROR");
    if (!frame.position.valid || !std::isfinite(frame.position.left_deg) ||
        !std::isfinite(frame.position.right_deg))
      return Fail("POSITION ERR");
    if (!have_sample || !frame.velocity.valid || !Nonnegative(frame.velocity.time_ms) ||
        !std::isfinite(frame.velocity.left_rpm) || !std::isfinite(frame.velocity.right_rpm) ||
        now < received_at || now - received_at > config.stale_ms)
      return Fail("SPEED ERR");
    return true;
  }
  bool Fail(const char* reason) {
    info.state = push_state::error;
    info.status = reason;
    info.left_mV = info.right_mV = 0;
    info.settling = false;
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
enum class action { straight, turn, heading, corner, wait, push, point, swing, drive_turn_drive };
enum class turn_mode { stop, rolling };
struct command {
  explicit command(action value, double timeout) : kind(value), timeout_ms(timeout) {}
  action kind;
  double timeout_ms;  // 整个命令的执行上限，毫秒；路线必须显式填写。
  double distance_mm = 0, angle_deg = 0, before_mm = 0, after_mm = 0;
  double x_mm = 0, y_mm = 0, duration_ms = 0, volts = DEFAULT_PUSH_VOLTS;
  double maximum_rpm = DEFAULT_DRIVE_RPM, turn_rpm = DEFAULT_TURN_RPM;
  double entry_speed_mm_s = 200;
  turn_mode transition = turn_mode::stop;
  drive::pivot_side fixed_side = drive::pivot_side::none;
};
inline command DriveStraight(double distance_mm, double maximum_rpm, double timeout_ms) {
  command result(action::straight, timeout_ms);
  result.distance_mm = distance_mm;
  result.maximum_rpm = maximum_rpm;
  return result;
}
inline command Turn(double angle_deg, double maximum_rpm, double timeout_ms) {
  command result(action::turn, timeout_ms);
  result.angle_deg = angle_deg;
  result.maximum_rpm = maximum_rpm;
  return result;
}
inline command TurnToHeading(double heading_deg, double maximum_rpm, double timeout_ms) {
  command result = Turn(heading_deg, maximum_rpm, timeout_ms);
  result.kind = action::heading;
  return result;
}
inline command DriveCorner(double before_mm, double angle_deg, double after_mm, double maximum_rpm,
                           double timeout_ms) {
  command result(action::corner, timeout_ms);
  result.before_mm = before_mm;
  result.angle_deg = angle_deg;
  result.after_mm = after_mm;
  result.maximum_rpm = maximum_rpm;
  return result;
}
inline command MoveToPoint(double x_mm, double y_mm, double maximum_rpm, double turn_rpm, double timeout_ms) {
  command result(action::point, timeout_ms);
  result.x_mm = x_mm;
  result.y_mm = y_mm;
  result.maximum_rpm = maximum_rpm;
  result.turn_rpm = turn_rpm;
  return result;
}
inline command Swing(double angle_deg, drive::pivot_side fixed, double maximum_rpm, double timeout_ms) {
  command result = Turn(angle_deg, maximum_rpm, timeout_ms);
  result.kind = action::swing;
  result.fixed_side = fixed;
  return result;
}
inline command Wait(double milliseconds, double timeout_ms) {
  command result(action::wait, timeout_ms);
  result.duration_ms = milliseconds;
  return result;
}
// stop分三步停稳交接；相邻rolling组合自动连成一条路线，仅最后停车。
inline command DriveTurnDrive(double before_mm, double angle_deg, double after_mm, turn_mode mode,
                              double entry_mm_s, double maximum_rpm, double timeout_ms) {
  command result(action::drive_turn_drive, timeout_ms);
  result.before_mm = before_mm;
  result.angle_deg = angle_deg;
  result.after_mm = after_mm;
  result.transition = mode;
  result.entry_speed_mm_s = entry_mm_s;
  result.maximum_rpm = maximum_rpm;
  return result;
}
inline command PushFor(double milliseconds, double volts, double timeout_ms) {
  command result(action::push, timeout_ms);
  result.duration_ms = milliseconds;
  result.volts = volts;
  return result;
}
}  // 路线动作命名空间

enum class route_result { idle, running, done, cancelled, error };
enum class point_phase { idle, facing, driving };

// 既有遥测编号保持；rolling=5独立于旧圆角corner=3。
enum class autonomous_motion { none, straight, turn, corner, push, rolling };

struct autonomous_info {
  bool active = false;
  bool finished = false;
  unsigned int run = 0;
  unsigned int step = 0;
  const char* status = "IDLE";
  double time_ms = 0;
  bool waiting = false;
  route_result result = route_result::idle;  // 0 未开始，1 运行，2 正常完成，3 取消，4 故障。
  drive::straight_info drive;
  drive::turn_info turn;
  drive::corner_info corner;
  drive::rolling_info rolling;
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
    turn.left_forward = value.left_forward;
    turn.right_forward = value.right_forward;
    turn.left_reverse = value.left_reverse;
    turn.right_reverse = value.right_reverse;
    Configure(value, turn);
  }
  void Configure(const drive::straight_config& value, const drive::turn_config& turn) {
    Rolling.Configure(value, turn, drive::corner_config());
    Chassis.Configure(value);
    Turning.Configure(turn);
    straight_settings = value;
    turn_settings = turn;
    Pushing.Configure(drive::push_config());
  }
  void Configure(const drive::straight_config& value, const drive::turn_config& turn,
                 const drive::corner_config& corner) {
    Configure(value, turn);
    Cornering.Configure(corner);
    Rolling.Configure(value, turn, corner);
  }
  void Update(bool permitted, const drive::straight_feedback& feedback, double now);
  void ConfigurePoint(const drive::point_config& value) {
    point_settings = value;
  }
  void ObservePose(const drive::pose& value) {
    position = value;
  }  // 主循环传入，不跨线程取估计器。
  void CancelRoutine() {
    if (info.active)
      FinishRoutine("CANCEL", route_result::cancelled);
  }
  bool StartDriveStraight(double distance_mm, double maximum_rpm) {
    if (!info.active || Busy())
      return false;
    info.motion = autonomous_motion::straight;
    Chassis.Configure(straight_settings);
    Chassis.Observe(frame, true, route_time);
    return Chassis.StartDriveStraight(distance_mm, maximum_rpm);
  }
  bool StartTurnRelative(double angle_deg, double maximum_rpm) {
    if (!info.active || Busy())
      return false;
    info.motion = autonomous_motion::turn;
    Turning.Configure(turn_settings);
    Turning.Observe(frame, true, route_time);
    return Turning.StartTurnRelative(angle_deg, maximum_rpm);
  }
  bool StartTurnToHeading(double heading_deg, double maximum_rpm) {
    if (!info.active || Busy())
      return false;
    info.motion = autonomous_motion::turn;
    Turning.Configure(turn_settings);
    Turning.Observe(frame, true, route_time);
    return Turning.StartTurnToHeading(heading_deg, maximum_rpm);
  }
  drive::straight_state DriveStraightState() const {
    return Chassis.Info().state;
  }
  bool StartDriveCorner(double before_mm, double angle_deg, double after_mm, double maximum_rpm) {
    if (!info.active || Busy())
      return false;
    info.motion = autonomous_motion::corner;
    return Cornering.StartDriveCorner(before_mm, angle_deg, after_mm, maximum_rpm);
  }
  drive::corner_state CornerState() const {
    return Cornering.Info().state;
  }
  void CancelCorner() {
    Cornering.CancelCorner();
    Publish();
  }
  bool StartRollingPath(const drive::rolling_leg* legs, unsigned count) {
    if (!info.active || Busy())
      return false;
    info.motion = autonomous_motion::rolling;
    return Rolling.StartPath(legs, count);
  }
  bool StartRollingDriveTurnDrive(double before_mm, double angle_deg, double after_mm, double maximum_rpm,
                                  double entry_mm_s, double timeout_ms) {
    if (!info.active || Busy())
      return false;
    info.motion = autonomous_motion::rolling;
    return Rolling.StartDriveTurnDrive(before_mm, angle_deg, after_mm, maximum_rpm, entry_mm_s, timeout_ms);
  }
  drive::rolling_state RollingState() const {
    return Rolling.Info().state;
  }
  void CancelRolling() {
    Rolling.Cancel();
    Publish();
  }
  bool StartPushFor(double milliseconds, double volts = route::DEFAULT_PUSH_VOLTS) {
    if (!info.active || Busy())
      return false;
    info.motion = autonomous_motion::push;
    return Pushing.StartPushFor(milliseconds, volts);
  }
  drive::push_state PushState() const {
    return Pushing.Info().state;
  }
  drive::turn_state TurnState() const {
    return Turning.Info().state;
  }
  void CancelDriveStraight() {
    Chassis.CancelDriveStraight();
    Publish();
  }
  void CancelTurn() {
    Turning.CancelTurn();
    Publish();
  }
  const autonomous_info& Info() const {
    return info;
  }

 private:
  void RunRoutine();  // 只在 src/route.cpp 编排动作，同一份路线用于实车调试和比赛。
  void RunRoute(std::initializer_list<route::command> commands);
  void FinishRoutine() {
    FinishRoutine("DONE", route_result::done);
  }
  void FinishRoutine(const char* status, route_result result = route_result::error) {
    CancelAll();
    point_stage = point_phase::idle;
    info.active = false;
    info.finished = true;
    info.status = status;
    info.waiting = false;
    info.result = result;
    Publish();  // 所有结束路径立即发布零输出，不等待下一次Update。
  }
  bool Busy() const {
    return Chassis.Info().state == drive::straight_state::running ||
           Turning.Info().state == drive::turn_state::running ||
           Cornering.Info().state == drive::corner_state::running ||
           Rolling.Info().state == drive::corner_state::running ||
           Pushing.Info().state == drive::push_state::running;
  }
  // 动作调度、终态读取和输出发布均使用info.motion这一个选择源。
  drive::motion_state ActiveState() const;
  const char* ActiveStatus() const;
  bool OwnershipValid() const;
  void CancelAll();  // 仅全局结束/取消使用；普通动作取消不影响其他控制器。
  bool StartPoint(const route::command& command);
  bool PointPoseReady() const;
  double PointBearingError(const route::command& command) const;
  double PointTurnTolerance(double distance_mm) const;
  void Publish();
  drive::pose position;
  point_phase point_stage = point_phase::idle;
  unsigned point_realignments = 0;
  drive::point_config point_settings;
  drive::straight_config straight_settings;
  drive::turn_config turn_settings;
  drive::straight_feedback frame;
  drive::straight_controller Chassis;
  drive::turn_controller Turning;
  drive::corner_controller Cornering;
  drive::rolling_controller Rolling;
  drive::push_controller Pushing;
  autonomous_info info;
  bool previously_permitted = false;
  bool waiting_ready = false;
  bool route_step_started = false;
  unsigned transfer_phase = 0, rolling_span = 1;
  double route_time = 0, wait_started = 0, action_started_at = 0;
};
