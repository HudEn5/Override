#pragma once

#include "drive-control.h"
#include "velocity-control.h"

namespace drive {
// 共用的速度执行层：不读写设备，由主循环传入快照并统一发送电压。
class velocity_control {
 public:
  void ConfigureForward(const feedforward_config& left, const feedforward_config& right) {
    left_forward = left_reverse = left;
    right_forward = right_reverse = right;
  }
  void ConfigureReverse(const feedforward_config& left, const feedforward_config& right) {
    left_reverse = left; right_reverse = right;
  }
  void ConfigurePID(const velocity_pid_config& value) {
    config = value; left_pid.Configure(value); right_pid.Configure(value); Reset();
  }
  void Reset() { left_pid.Reset(); right_pid.Reset(); info = velocity_loop_info(); }
  const velocity_loop_info& Info() const { return info; }

  void Observe(const velocity_feedback& speed, double now) {
    const bool valid = speed.valid && std::isfinite(speed.time_ms) && speed.time_ms >= 0 &&
        std::isfinite(speed.left_rpm) && std::isfinite(speed.right_rpm) &&
        std::isfinite(now) && now >= 0;
    if (valid && (!have_speed || speed.sequence != observed_sequence)) {
      observed_sequence = speed.sequence; received_at = now; have_speed = true;
    }
    if (!valid) have_speed = false;
  }
  bool Fresh(double now) const {
    return have_speed && std::isfinite(now) && std::isfinite(config.stale_ms) &&
        config.stale_ms > 0 && now >= received_at && now - received_at <= config.stale_ms;
  }
  double Slope(bool left, double target) const {
    return Model(left, target).kV;
  }
  bool Feedforward(double left_target, double right_target, int& left, int& right) const {
    const feedforward_output l = motor_feedforward(Model(true, left_target)).Calculate(left_target, 0);
    const feedforward_output r = motor_feedforward(Model(false, right_target)).Calculate(right_target, 0);
    if (!l.valid || !r.valid || l.saturated || r.saturated) return false;
    left = static_cast<int>(std::lround(l.millivolts));
    right = static_cast<int>(std::lround(r.millivolts));
    return true;
  }
  // 基础电压可以已包含航向修正；对应的目标转速必须同样包含差速修正。
  bool ApplyPID(double left_target, double right_target, const velocity_feedback& speed,
                double now, int& left, int& right) {
    if (!Fresh(now)) return false;
    if (!info.active || speed.sequence != applied_sequence) {
      const double dt = info.active ? (speed.time_ms - info.time_ms) / 1000.0 : 0;
      info.left_target = left_target; info.right_target = right_target;
      info.left_error = left_target - speed.left_rpm;
      info.right_error = right_target - speed.right_rpm;
      const double l = std::fabs(static_cast<double>(left));
      const double r = std::fabs(static_cast<double>(right));
      if (!left_pid.Update(left_target, speed.left_rpm, dt,
              std::min(l * 0.5, 12000.0 - l), info.left_correction) ||
          !right_pid.Update(right_target, speed.right_rpm, dt,
              std::min(r * 0.5, 12000.0 - r), info.right_correction)) return false;
      info.time_ms = speed.time_ms; info.active = true; applied_sequence = speed.sequence;
    }
    left = static_cast<int>(std::lround(std::max(-12000.0,
        std::min(12000.0, left + info.left_correction))));
    right = static_cast<int>(std::lround(std::max(-12000.0,
        std::min(12000.0, right + info.right_correction))));
    return true;
  }

 private:
  const feedforward_config& Model(bool left, double target) const {
    return left ? (target < 0 ? left_reverse : left_forward) :
                  (target < 0 ? right_reverse : right_forward);
  }
  feedforward_config left_forward, right_forward, left_reverse, right_reverse;
  velocity_pid_config config;
  velocity_pid left_pid, right_pid;
  velocity_loop_info info;
  bool have_speed = false;
  unsigned int observed_sequence = 0, applied_sequence = 0;
  double received_at = 0;
};
}  // 底盘速度控制命名空间
