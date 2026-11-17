#pragma once

#include <algorithm>
#include <cmath>
#include "drive-control.h"

namespace drive {
struct velocity_pid_config {
  double kP = 6.0;  // mV/rpm；实车调试初值。
  double kI = 2.0;  // mV/(rpm·秒)。
  double kD = 0.0;  // mV/(rpm/秒)；先关闭，避免放大测速噪声。
  double max_correction_mV = 800.0;
  double max_integral_mV = 300.0;  // 积分项电压上限。
  double stale_ms = 100.0;         // 超过此时间没有新速度样本就停车。
};

struct velocity_feedback {
  bool valid = false;
  unsigned int sequence = 0;
  double time_ms = 0.0;
  double left_rpm = 0.0;
  double right_rpm = 0.0;
};

struct velocity_loop_info {
  bool active = false;
  double time_ms = 0.0;  // 使用速度采样线程的时间基准。
  double left_target_rpm = 0.0;
  double right_target_rpm = 0.0;
  double left_error_rpm = 0.0;
  double right_error_rpm = 0.0;
  double left_correction_mV = 0.0;
  double right_correction_mV = 0.0;
};

class velocity_pid {
 public:
  void Configure(const velocity_pid_config& value) {
    config = value;
    Reset();
  }
  void Reset() {
    integral = 0.0;
    initialized = false;
  }

  // 由新测速样本驱动；重复样本不能重复积分。微分对测量值求导，避免目标阶跃冲击。
  bool Update(double target, double measured, double dt, double headroom, double& correction) {
    correction = 0.0;
    if (!std::isfinite(target) || !std::isfinite(measured) || !std::isfinite(dt) ||
        !std::isfinite(headroom) || headroom < 0 || dt < 0 || dt > 0.2 || (initialized && dt <= 0) ||
        !ValidConfig())
      return false;
    const double error = target - measured;
    const double derivative = initialized ? -(measured - previous) / dt : 0.0;
    const double limit = std::min(config.max_correction_mV, headroom);
    const double candidate = Clamp(integral + config.kI * error * dt, config.max_integral_mV);
    const double requested = config.kP * error + candidate + config.kD * derivative;
    // 饱和且误差仍向饱和方向推动时冻结积分，反向误差允许积分退回。
    if (std::fabs(requested) <= limit || (requested > limit && error < 0) ||
        (requested < -limit && error > 0))
      integral = candidate;
    correction = Clamp(config.kP * error + integral + config.kD * derivative, limit);
    previous = measured;
    initialized = true;
    return std::isfinite(correction);
  }

 private:
  static double Clamp(double value, double limit) {
    return std::max(-limit, std::min(limit, value));
  }
  bool ValidConfig() const {
    return std::isfinite(config.kP) && config.kP >= 0 && std::isfinite(config.kI) && config.kI >= 0 &&
           std::isfinite(config.kD) && config.kD >= 0 && std::isfinite(config.max_correction_mV) &&
           config.max_correction_mV > 0 && std::isfinite(config.max_integral_mV) &&
           config.max_integral_mV >= 0;
  }
  velocity_pid_config config;
  double integral = 0.0;
  double previous = 0.0;
  bool initialized = false;
};
}  // 底盘速度控制命名空间

namespace drive {
struct heading_config {
  double kP = 80.0;  // mV/度，初始值需要实车验证。
  double kD = 10.0;  // mV/(度/秒)，抑制转向摆动。
  double rate_filter_ms = 60.0;
  double max_correction_mV = 500.0;
  double max_voltage_fraction = 0.25;  // 修正不超过较小侧基础电压的四分之一。
  double max_error = 15.0;             // 偏离过大时停车，等待人工重新定向。
};

struct heading_info {
  double time_ms = 0.0;
  double angle_deg = 0.0;
  double target_deg = 0.0;
  double error_deg = 0.0;
  double correction_mV = 0.0;
  bool ready = false;
  bool locked = false;
  bool active = false;
};

// 连续惯导角逆时针为正；前进、停车、后退共用同一参考角。
class heading_hold {
 public:
  void Configure(const heading_config& value) {
    config = value;
    Reset();
  }
  void Reset() {
    info = heading_info();
    have_previous = false;
    rate = 0.0;
  }
  const heading_info& Info() const {
    return info;
  }

  // 停车只记录角度，不输出转向；重新行驶才修正停车产生的偏差。
  bool Update(bool running, bool ready, double angle, double now, int& left, int& right,
              double target_limit = 12000.0) {
    info.active = false;
    info.correction_mV = 0.0;
    info.ready = ready && std::isfinite(angle) && std::isfinite(now);
    if (!info.ready) {
      Reset();
      return false;
    }
    info.ready = true;
    info.time_ms = now;
    info.angle_deg = angle;
    if (running && !info.locked) {
      info.target_deg = angle;
      info.locked = true;
    }
    info.error_deg = info.locked ? info.target_deg - angle : 0.0;
    const double dt = now - previous_time;
    if (running && have_previous && dt > 0.0 && dt <= 200.0) {
      const double measured_rate = (angle - previous_angle) * 1000.0 / dt;
      rate += dt / (config.rate_filter_ms + dt) * (measured_rate - rate);
    } else
      rate = 0.0;
    previous_time = now;
    previous_angle = angle;
    have_previous = true;
    if (!running)
      return true;
    if (std::fabs(info.error_deg) > config.max_error || !std::isfinite(target_limit) || target_limit < 0)
      return false;
    // 留出电压余量，同时避免修正使任一侧反转；正反行驶用同一转向符号。
    const double smaller =
        std::min(std::fabs(static_cast<double>(left)), std::fabs(static_cast<double>(right)));
    const double headroom =
        12000.0 - std::max(std::fabs(static_cast<double>(left)), std::fabs(static_cast<double>(right)));
    const double limit = std::max(
        0.0, std::min(target_limit, std::min(config.max_correction_mV,
                                             std::min(config.max_voltage_fraction * smaller, headroom))));
    info.correction_mV = std::max(-limit, std::min(limit, config.kP * info.error_deg - config.kD * rate));
    const int correction = static_cast<int>(std::lround(info.correction_mV));
    left -= correction;
    right += correction;
    info.active = true;
    return true;
  }

 private:
  heading_config config;
  heading_info info;
  bool have_previous = false;
  double previous_angle = 0.0;
  double previous_time = 0.0;
  double rate = 0.0;
};
}  // 底盘控制命名空间

namespace drive {
// 共用的速度执行层：不读写设备，由主循环传入快照并统一发送电压。
class velocity_control {
 public:
  void ConfigureForward(const feedforward_config& left, const feedforward_config& right) {
    left_forward = left_reverse = left;
    right_forward = right_reverse = right;
  }
  void ConfigureReverse(const feedforward_config& left, const feedforward_config& right) {
    left_reverse = left;
    right_reverse = right;
  }
  void ConfigurePID(const velocity_pid_config& value) {
    config = value;
    left_pid.Configure(value);
    right_pid.Configure(value);
    ResetControl();
    have_speed = false;
    observed_sequence = applied_sequence = 0;
    received_at = 0;
  }
  // 动作起停只清控制积分；重新ConfigurePID才丢弃采样基准。
  void ResetControl() {
    left_pid.Reset();
    right_pid.Reset();
    info = velocity_loop_info();
  }
  const velocity_loop_info& Info() const {
    return info;
  }

  void Observe(const velocity_feedback& speed, double now) {
    const bool valid = speed.valid && std::isfinite(speed.time_ms) && speed.time_ms >= 0 &&
                       std::isfinite(speed.left_rpm) && std::isfinite(speed.right_rpm) &&
                       std::isfinite(now) && now >= 0;
    if (valid && (!have_speed || speed.sequence != observed_sequence)) {
      observed_sequence = speed.sequence;
      received_at = now;
      have_speed = true;
    }
    if (!valid)
      have_speed = false;
  }
  bool Fresh(double now) const {
    return have_speed && std::isfinite(now) && std::isfinite(config.stale_ms) && config.stale_ms > 0 &&
           now >= received_at && now - received_at <= config.stale_ms;
  }
  double Slope(bool left, double target) const {
    return Model(left, target).kV;
  }
  bool Feedforward(double left_target_rpm, double right_target_rpm, int& left, int& right) const {
    const feedforward_output l =
        motor_feedforward(Model(true, left_target_rpm)).Calculate(left_target_rpm, 0);
    const feedforward_output r =
        motor_feedforward(Model(false, right_target_rpm)).Calculate(right_target_rpm, 0);
    if (!l.valid || !r.valid || l.saturated || r.saturated)
      return false;
    left = static_cast<int>(std::lround(l.millivolts));
    right = static_cast<int>(std::lround(r.millivolts));
    return true;
  }
  // 基础电压可以已包含航向修正；对应的目标转速必须同样包含差速修正。
  bool ApplyPID(double left_target_rpm, double right_target_rpm, const velocity_feedback& speed, double now,
                int& left, int& right, double correction_fraction = 0.5) {
    if (!std::isfinite(correction_fraction) || correction_fraction <= 0)
      return false;
    if (!Fresh(now))
      return false;
    if (!info.active || speed.sequence != applied_sequence) {
      const double dt = info.active ? (speed.time_ms - info.time_ms) / 1000.0 : 0;
      info.left_target_rpm = left_target_rpm;
      info.right_target_rpm = right_target_rpm;
      info.left_error_rpm = left_target_rpm - speed.left_rpm;
      info.right_error_rpm = right_target_rpm - speed.right_rpm;
      const double l = std::fabs(static_cast<double>(left));
      const double r = std::fabs(static_cast<double>(right));
      if (!left_pid.Update(left_target_rpm, speed.left_rpm, dt,
                           std::min(l * correction_fraction, 12000.0 - l), info.left_correction_mV) ||
          !right_pid.Update(right_target_rpm, speed.right_rpm, dt,
                            std::min(r * correction_fraction, 12000.0 - r), info.right_correction_mV))
        return false;
      info.time_ms = speed.time_ms;
      info.active = true;
      applied_sequence = speed.sequence;
    }
    left =
        static_cast<int>(std::lround(std::max(-12000.0, std::min(12000.0, left + info.left_correction_mV))));
    right = static_cast<int>(
        std::lround(std::max(-12000.0, std::min(12000.0, right + info.right_correction_mV))));
    return true;
  }

 private:
  const feedforward_config& Model(bool left, double target) const {
    return left ? (target < 0 ? left_reverse : left_forward) : (target < 0 ? right_reverse : right_forward);
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
