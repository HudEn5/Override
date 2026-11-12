#pragma once

#include <cmath>

namespace drive {
// 默认静默；长按只切换一次，启动或重连时按住按钮也不会自动开始回传。
class telemetry_switch {
 public:
  bool Update(bool pressed, bool connected) {
    if (!connected) active = false;
    else if (previous_connected && pressed && !previous_pressed) active = !active;
    previous_pressed = pressed;
    previous_connected = connected;
    return active;
  }

 private:
  bool active = false;
  bool previous_pressed = false;
  bool previous_connected = false;
};

struct motor_feedback {
  bool online = false;
  double rpm = 0.0;
  double millivolts = 0.0;
  double position_deg = 0.0;
  double current_A = 0.0;
  bool diagnostic_valid = false;  // 仅给需要交叉验证的 LF、LM 读取角度和电流。
};

struct feedback {
  motor_feedback left[4];
  motor_feedback right[4];
};

struct side_sample {
  bool velocity_valid = false;
  unsigned int online_mask = 0;
  double raw_rpm[4] = {};
  double raw_average_rpm = 0.0;
  double rpm = 0.0;
  double acceleration = 0.0;  // rpm/s
  double millivolts = 0.0;   // 四台电机实测电压的平均值
  bool acceleration_valid = false;
};

class velocity_sampler {
 public:
  side_sample Update(const motor_feedback (&motors)[4], double time_ms) {
    side_sample result;
    for (int i = 0; i < 4; ++i) {
      if (motors[i].online && std::isfinite(motors[i].rpm) &&
          std::isfinite(motors[i].millivolts)) {
        result.online_mask |= 1u << i;
        result.raw_rpm[i] = motors[i].rpm;
        result.raw_average_rpm += motors[i].rpm / 4.0;
        result.millivolts += motors[i].millivolts / 4.0;
      }
    }
    // 整侧有效才计算平均速度，掉线时不把剩余电机误当成四台平均值。
    if (result.online_mask != 15 || !std::isfinite(time_ms) || time_ms < 0) {
      initialized = false;
      result.raw_average_rpm = result.millivolts = 0.0;
      return result;
    }
    const double dt = (time_ms - previous_time) / 1000.0;
    if (!initialized || dt <= 0 || dt > 0.2) {
      filtered_rpm = result.raw_average_rpm;
    } else {
      // 按实际 dt 滤波和求加速度。首样本、重连或长间隔后的加速度无效。
      const double previous_rpm = filtered_rpm;
      filtered_rpm += dt / (0.04 + dt) * (result.raw_average_rpm - filtered_rpm);
      result.acceleration = (filtered_rpm - previous_rpm) / dt;
      result.acceleration_valid = true;
    }
    previous_time = time_ms;
    initialized = true;
    result.velocity_valid = true;
    result.rpm = filtered_rpm;
    return result;
  }

 private:
  bool initialized = false;
  double previous_time = 0.0;
  double filtered_rpm = 0.0;
};

struct feedforward_config {
  // 用实测数据辨识后填写。kA 为 0 时仅包含静态和速度项。
  double kS = 0.0;  // mV
  double kV = 0.0;  // mV/rpm
  double kA = 0.0;  // mV/(rpm/s)
  double max_voltage = 12000.0;
  bool calibrated = false;
};

struct feedforward_output {
  bool valid = false;
  bool saturated = false;
  double requested_mV = 0.0;
  double millivolts = 0.0;
};

class motor_feedforward {
 public:
  explicit motor_feedforward(const feedforward_config& parameters) : settings(parameters) {}

  feedforward_output Calculate(double velocity_rpm, double acceleration_rpm_s) const {
    feedforward_output result;
    if (!settings.calibrated || !std::isfinite(settings.kS) || settings.kS < 0 ||
        !std::isfinite(settings.kV) || settings.kV <= 0 ||
        !std::isfinite(settings.kA) || settings.kA < 0 ||
        !std::isfinite(settings.max_voltage) || settings.max_voltage <= 0 ||
        settings.max_voltage > 12000 || !std::isfinite(velocity_rpm) ||
        !std::isfinite(acceleration_rpm_s)) return result;
    // 起步瞬间速度为零时按目标加速度判断静摩擦方向，静止目标输出为零。
    const double direction_input = std::fabs(velocity_rpm) > 1e-6 ?
                                   velocity_rpm : acceleration_rpm_s;
    const double direction = direction_input > 1e-6 ? 1.0 :
                             (direction_input < -1e-6 ? -1.0 : 0.0);
    result.requested_mV = settings.kS * direction + settings.kV * velocity_rpm +
                          settings.kA * acceleration_rpm_s;
    if (!std::isfinite(result.requested_mV)) return feedforward_output();
    result.millivolts = result.requested_mV;
    if (result.millivolts > settings.max_voltage) result.millivolts = settings.max_voltage;
    if (result.millivolts < -settings.max_voltage) result.millivolts = -settings.max_voltage;
    result.saturated = result.millivolts != result.requested_mV;
    result.valid = true;
    return result;
  }

 private:
  feedforward_config settings;
};
}  // 底盘控制命名空间
