#pragma once

#include <cmath>

namespace manual {
struct ArmPIDConfig {
  double kp;                    // mV/deg
  double ki;                    // mV/(deg*s)
  double kd;                    // mV/(deg/s)
  double max_voltage;           // 输出电压上限，mV
  double voltage_slew;          // 每秒允许变化的电压，mV/s
  double integral_voltage_max;  // 积分项电压上限，mV
  double integral_zone;         // 仅在此误差范围内积分，deg
  double D_filter_time;         // 测量微分的低通滤波时间常数，s
};

inline double LimitValue(double value, double low, double high) {
  return value < low ? low : (value > high ? high : value);
}

class arm_target_filter {
 public:
  void Reset(double angle) { target = angle; }

  double Update(double requested, double dt, double max_speed, double filter_time) {
    if (dt <= 0) return target;
    const double alpha = filter_time > 0 ? dt / (filter_time + dt) : 1.0;
    const double delta = LimitValue((requested - target) * alpha,
                                   -max_speed * dt, max_speed * dt);
    target += delta;
    return target;
  }

 private:
  double target = 0.0;
};

class arm_pid {
 public:
  explicit arm_pid(const ArmPIDConfig& config) : settings(config) {}

  void Reset(double input) {
    pre_input = input;
    I_voltage = 0.0;
    D = 0.0;
    output = 0.0;
  }

  double Update(double target, double input, double dt) {
    if (dt <= 0) return output;
    const double error = target - input;
    // 对测量值求微分，目标变化不会直接产生微分冲击。
    const double measured_D = -(input - pre_input) / dt;
    const double alpha = settings.D_filter_time > 0 ?
        dt / (settings.D_filter_time + dt) : 1.0;
    D += alpha * (measured_D - D);
    pre_input = input;

    const double low = LimitValue(output - settings.voltage_slew * dt,
                                  -settings.max_voltage, settings.max_voltage);
    const double high = LimitValue(output + settings.voltage_slew * dt,
                                   -settings.max_voltage, settings.max_voltage);
    double next_I = I_voltage;
    if (std::fabs(error) <= settings.integral_zone) {
      next_I = LimitValue(I_voltage + settings.ki * error * dt,
                          -settings.integral_voltage_max, settings.integral_voltage_max);
    }
    const double candidate = settings.kp * error + next_I + settings.kd * D;
    // 电压限幅或变化率限幅时，禁止积分继续向饱和方向累积。
    if (!((candidate > high && error > 0) || (candidate < low && error < 0)))
      I_voltage = next_I;
    output = LimitValue(settings.kp * error + I_voltage + settings.kd * D, low, high);
    return output;
  }

 private:
  ArmPIDConfig settings;
  double pre_input = 0.0;
  double I_voltage = 0.0;
  double D = 0.0;
  double output = 0.0;
};
}  // 手控逻辑命名空间
