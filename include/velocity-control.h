#pragma once

#include <algorithm>
#include <cmath>

namespace drive {
struct velocity_pid_config {
  double kP = 6.0;       // mV/rpm；实车调试初值。
  double kI = 2.0;       // mV/(rpm·秒)。
  double kD = 0.0;       // mV/(rpm/秒)；先关闭，避免放大测速噪声。
  double max_correction = 800.0;
  double max_integral = 300.0;  // 积分项电压上限。
  double stale_ms = 100.0;     // 超过此时间没有新速度样本就停车。
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
  double left_target = 0.0;
  double right_target = 0.0;
  double left_error = 0.0;
  double right_error = 0.0;
  double left_correction = 0.0;
  double right_correction = 0.0;
};

class velocity_pid {
 public:
  void Configure(const velocity_pid_config& value) { config = value; Reset(); }
  void Reset() { integral = 0.0; initialized = false; }

  // 由新测速样本驱动；重复样本不能重复积分。微分对测量值求导，避免目标阶跃冲击。
  bool Update(double target, double measured, double dt, double headroom, double& correction) {
    correction = 0.0;
    if (!std::isfinite(target) || !std::isfinite(measured) || !std::isfinite(dt) ||
        !std::isfinite(headroom) || headroom < 0 || dt < 0 || dt > 0.2 ||
        (initialized && dt <= 0) || !ValidConfig()) return false;
    const double error = target - measured;
    const double derivative = initialized ? -(measured - previous) / dt : 0.0;
    const double limit = std::min(config.max_correction, headroom);
    const double candidate = Clamp(integral + config.kI * error * dt, config.max_integral);
    const double requested = config.kP * error + candidate + config.kD * derivative;
    // 饱和且误差仍向饱和方向推动时冻结积分，反向误差允许积分退回。
    if (std::fabs(requested) <= limit || (requested > limit && error < 0) ||
        (requested < -limit && error > 0)) integral = candidate;
    correction = Clamp(config.kP * error + integral + config.kD * derivative, limit);
    previous = measured;
    initialized = true;
    return std::isfinite(correction);
  }

 private:
  static double Clamp(double value, double limit) { return std::max(-limit, std::min(limit, value)); }
  bool ValidConfig() const {
    return std::isfinite(config.kP) && config.kP >= 0 &&
        std::isfinite(config.kI) && config.kI >= 0 &&
        std::isfinite(config.kD) && config.kD >= 0 &&
        std::isfinite(config.max_correction) && config.max_correction > 0 &&
        std::isfinite(config.max_integral) && config.max_integral >= 0;
  }
  velocity_pid_config config;
  double integral = 0.0;
  double previous = 0.0;
  bool initialized = false;
};
}  // 底盘速度控制命名空间
