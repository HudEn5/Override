#pragma once

#include "drive-velocity.h"
#include "trapezoid-motion.h"

namespace drive {
using straight_state = motion_state;

struct straight_config {
  trapezoid_config motion;
  heading_config heading;
  velocity_pid_config pid;
  feedforward_config left_forward, right_forward, left_reverse, right_reverse;
  double max_heading_speed_fraction = 0.25;  // 目标转速纠偏比例，与航向电压比例分别限制。
  bool heading_enabled = true;
  bool retain_heading = false;  // TP 可保留本轮方向；比赛默认每个动作锁定起步方向。
};

struct straight_feedback {
  velocity_feedback velocity;  // 由调用方传入编码器位置差分速度。
  position_feedback position;
  bool motors_online = false;
  bool inertial_ready = false;
  bool stopped = false;
  double heading_deg = 0;
};

// 起步前先区分反馈故障和实际运动，避免掉线导致测速无效却显示MOVING。
inline const char* StartStatus(const straight_feedback& frame) {
  if (!frame.motors_online)
    return "MOTOR OFF";
  if (!frame.inertial_ready || !std::isfinite(frame.heading_deg))
    return "IMU WAIT";
  if (!frame.position.valid || !std::isfinite(frame.position.left_deg) ||
      !std::isfinite(frame.position.right_deg))
    return "POSITION ERR";
  if (!frame.velocity.valid || !std::isfinite(frame.velocity.left_rpm) ||
      !std::isfinite(frame.velocity.right_rpm))
    return "SPEED ERR";
  return frame.stopped ? nullptr : "MOVING";
}

struct straight_info {
  straight_state state = straight_state::idle;
  const char* status = "READY";
  int left_mV = 0, right_mV = 0;
  trapezoid_info motion;
  heading_info heading;
  velocity_loop_info velocity;
};

// 纯控制层，无手柄、场控、屏幕或电机访问；全部接口由同一主控制线程调用。
class straight_controller {
 public:
  void Configure(const straight_config& value) {
    config = value;
    Velocity.ConfigureForward(value.left_forward, value.right_forward);
    Velocity.ConfigureReverse(value.left_reverse, value.right_reverse);
    Velocity.ConfigurePID(value.pid);
    Heading.Configure(value.heading);
    Motion.Configure(value.motion);
    configured = true;
    have_frame = false;
    allowed = false;
    info = straight_info();
  }
  const straight_info& Info() const {
    return info;
  }
  void Reject(const char* status) {
    Fail(status);
  }

  // 每周期先发布一次快照和运行许可，再启动/更新。没有快照时不能起步。
  void Observe(const straight_feedback& value, bool permitted, double now_ms) {
    feedback = value;
    allowed = permitted;
    now = now_ms;
    have_frame = true;
    Velocity.Observe(value.velocity, now_ms);
    if (info.state != straight_state::running && config.heading_enabled) {
      int left = 0, right = 0;
      Heading.Update(false, value.inertial_ready, value.heading_deg, now_ms, left, right);
      info.heading = Heading.Info();
    }
  }

  // 距离 mm（负数后退），最高电机输出轴 rpm（正数）。正在运行时拒绝覆盖当前动作。
  bool StartDriveStraight(double distance_mm, double maximum_rpm) {
    if (info.state == straight_state::running)
      return false;
    if (!configured || !std::isfinite(distance_mm) || !std::isfinite(maximum_rpm) || maximum_rpm <= 0 ||
        maximum_rpm > 600 || !std::isfinite(config.max_heading_speed_fraction) ||
        config.max_heading_speed_fraction < 0 || config.max_heading_speed_fraction > 1)
      return Fail("CONFIG");
    if (!CheckFeedback())
      return false;
    Velocity.ResetControl();
    if (!config.retain_heading)
      Heading.Reset();
    trapezoid_config settings = config.motion;
    settings.distance_mm = std::fabs(distance_mm);
    Motion.Configure(settings);
    if (distance_mm == 0) {
      Motion.Complete();
      info.state = straight_state::done;
      info.status = "DONE";
      Publish();
      return true;
    }
    if (!Motion.Start(distance_mm > 0 ? 1 : -1, maximum_rpm, feedback.position, feedback.velocity, now,
                      feedback.stopped))
      return Fail(Motion.Info().status);
    info.state = straight_state::running;
    UpdateDriveStraight();
    return info.state == straight_state::running || info.state == straight_state::done;
  }

  void UpdateDriveStraight() {
    if (info.state != straight_state::running)
      return;
    if (!CheckFeedback())
      return;
    if (!Motion.Update(feedback.position, feedback.velocity, now)) {
      Fail(Motion.Info().status);
      return;
    }
    if (!Motion.Info().running) {
      info.state = straight_state::done;
      info.status = "DONE";
      StopOutput();
      Publish();
      return;
    }
    const double target = Motion.Info().target_rpm;
    const double acceleration = target == 0 ? 0 : Motion.Info().target_acceleration_rpm_s;
    if (!Velocity.Feedforward(target, target, info.left_mV, info.right_mV, acceleration, acceleration)) {
      Fail("FF ERROR");
      return;
    }
    if (config.heading_enabled) {
      const double limit = config.max_heading_speed_fraction * std::fabs(target) *
                           std::min(Velocity.Slope(true, target), Velocity.Slope(false, target));
      if (!Heading.Update(true, feedback.inertial_ready, feedback.heading_deg, now, info.left_mV,
                          info.right_mV, limit)) {
        Fail(Heading.Info().ready ? "HDG ERROR" : "IMU WAIT");
        return;
      }
    }
    const double yaw = config.heading_enabled ? Heading.Info().correction_mV : 0;
    if (!Velocity.ApplyPID(target - yaw / Velocity.Slope(true, target),
                           target + yaw / Velocity.Slope(false, target), feedback.velocity, now, info.left_mV,
                           info.right_mV)) {
      Fail("SPEED ERR");
      return;
    }
    info.status = Motion.Info().status;
    Publish();
  }

  void CancelDriveStraight() {
    if (info.state == straight_state::running) {
      Motion.Cancel();
      info.state = straight_state::cancelled;
      info.status = "CANCEL";
    }
    StopOutput();
    Publish();
  }
  void ResetReference() {
    Heading.Reset();
    info.heading = Heading.Info();
  }
  void Reset() {
    CancelDriveStraight();
    ResetReference();
    Motion.Reset();
    info = straight_info();
  }

 private:
  bool CheckFeedback() {
    if (!have_frame || !allowed)
      return Fail("DISABLED");
    if (!std::isfinite(now) || now < 0)
      return Fail("CLOCK ERR");
    if (!feedback.motors_online)
      return Fail("MOTOR OFF");
    if (!Velocity.Fresh(now))
      return Fail("SPEED ERR");
    if (!feedback.position.valid || !std::isfinite(feedback.position.left_deg) ||
        !std::isfinite(feedback.position.right_deg))
      return Fail("POSITION ERR");
    if (config.heading_enabled && (!feedback.inertial_ready || !std::isfinite(feedback.heading_deg)))
      return Fail("IMU WAIT");
    return true;
  }
  void StopOutput() {
    info.left_mV = info.right_mV = 0;
    Velocity.ResetControl();
    info.velocity = Velocity.Info();
    info.heading.active = false;
    info.heading.correction_mV = 0;
  }
  void Publish() {
    info.motion = Motion.Info();
    info.velocity = Velocity.Info();
    info.heading = Heading.Info();
    if (info.state != straight_state::running) {
      info.heading.active = false;
      info.heading.correction_mV = 0;
    }
  }
  bool Fail(const char* status) {
    Motion.Reject(status);
    info.state = straight_state::error;
    info.status = status;
    StopOutput();
    Publish();
    return false;
  }
  straight_config config;
  straight_feedback feedback;
  straight_info info;
  velocity_control Velocity;
  heading_hold Heading;
  trapezoid_motion Motion;
  bool configured = false, have_frame = false, allowed = false;
  double now = 0;
};
}  // 底盘直行控制命名空间
