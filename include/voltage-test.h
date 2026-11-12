#pragma once

#include "driver-control-logic.h"
#include "drive-control.h"
#include "heading-control.h"
#include "velocity-control.h"
#include <cmath>

struct voltage_test_info {
  bool visible = false;
  bool running = false;
  int millivolts = 0;
  bool feedforward = false;
  int selected_rpm = 100;
  double target_rpm = 0.0;  // 停止时为 0，运行时带前后方向符号。
  bool heading_enabled = false;
  drive::heading_info heading;
  bool speed_control = false;
  drive::velocity_loop_info velocity;
  const char* status = "READY";
};

// 测试控制器只生成电压，不读写设备；可比较前馈与前馈加速度闭环。
class voltage_test {
 public:
  voltage_test(const int (&voltages)[3], double timeout_ms) : timeout(timeout_ms) {
    for (int i = 0; i < 3; ++i) levels[i] = voltages[i];
    info.millivolts = levels[0];
  }

  const voltage_test_info& Info() const { return info; }

  void ConfigureHeading(const drive::heading_config& config) {
    heading.Configure(config);
    info.heading_enabled = true;
  }

  void ConfigureVelocityPID(const drive::velocity_pid_config& config) {
    velocity_config = config;
    left_pid.Configure(config);
    right_pid.Configure(config);
    velocity_configured = true;
  }

  void ConfigureFeedforward(const int (&rpm)[3], const drive::feedforward_config& left,
                            const drive::feedforward_config& right, double hold_ms) {
    for (int i = 0; i < 3; ++i) targets[i] = rpm[i];
    left_ff = drive::motor_feedforward(left);
    right_ff = drive::motor_feedforward(right);
    left_reverse_ff = left_ff;
    right_reverse_ff = right_ff;
    left_kV = left_reverse_kV = left.kV;
    right_kV = right_reverse_kV = right.kV;
    mode_hold_ms = hold_ms;
    info.selected_rpm = targets[ff_level];
  }

  // 不设置时后退沿用前进模型；有重复实测数据时可独立补偿正反向差异。
  void ConfigureReverseFeedforward(const drive::feedforward_config& left,
                                   const drive::feedforward_config& right) {
    left_reverse_ff = drive::motor_feedforward(left);
    right_reverse_ff = drive::motor_feedforward(right);
    left_reverse_kV = left.kV;
    right_reverse_kV = right.kV;
  }

  bool Update(const manual::Input& input, manual::Output& output, bool allowed,
              bool motors_online, double now_ms, bool stopped = true,
              bool inertial_ready = false, double angle = 0.0,
              const drive::velocity_feedback& speed = drive::velocity_feedback()) {
    const bool speed_valid = speed.valid && std::isfinite(speed.time_ms) &&
        std::isfinite(speed.left_rpm) && std::isfinite(speed.right_rpm);
    if (speed_valid && (!have_speed || speed.sequence != speed_sequence)) {
      speed_sequence = speed.sequence;
      speed_received_at = now_ms;
      have_speed = true;
    }
    if (!speed_valid) have_speed = false;
    const bool up = previous_allowed && input.voltage_up && !previous_up;
    const bool down = previous_allowed && input.voltage_down && !previous_down;
    const bool down_released = previous_down && !input.voltage_down;
    previous_up = input.voltage_up;
    previous_down = input.voltage_down;
    previous_allowed = allowed && motors_online;
    info.target_rpm = 0.0;
    const bool held = input.voltage_forward || input.voltage_reverse;
    if (held || input.voltage_up || input.voltage_down) info.visible = true;

    // 摇杆立即接回正常驾驶；按 A/B 先退出电压测试。
    if (output.L_voltage != 0 || output.R_voltage != 0 || input.turn_test || input.square_test) {
      info.visible = info.running = false;
      need_release = true;
      down_pending = false;
      ResetHeading();
      ResetSpeed();
      return false;
    }
    if (!allowed || !motors_online || !std::isfinite(now_ms)) {
      info.running = false;
      need_release = true;
      down_pending = false;
      ResetHeading();
      ResetSpeed();
      info.status = !allowed ? "DISABLED" : (!motors_online ? "MOTOR OFF" : "CLOCK ERR");
      if (!allowed || !info.visible) return false;
      output.L_voltage = output.R_voltage = 0;
      return true;
    }

    // 选择手势期间按运行键，先停止并要求释放，避免切换模式时意外起步。
    if (held && down_pending) {
      down_pending = false;
      need_release = true;
    }
    if (!held) {
      info.running = false;
      need_release = false;
      // 下键短按在松手时降档；停车长按只切换一次模式，不附带降档。
      if (down) {
        down_pending = !input.voltage_up && stopped;
        down_consumed = false;
        down_started_at = now_ms;
      }
      if (input.voltage_down && down_pending) {
        if (input.voltage_up || !stopped || now_ms < down_started_at) down_pending = false;
        else if (!down_consumed && now_ms - down_started_at >= mode_hold_ms) {
          if (info.feedforward && velocity_configured && !info.speed_control)
            info.speed_control = true;
          else {
            info.feedforward = !info.feedforward;
            info.speed_control = false;
          }
          ResetHeading();  // 主动退出/重进前馈模式时才建立新一轮参考。
          ResetSpeed();
          down_consumed = true;
        }
      }
      int& selected_level = info.feedforward ? ff_level : level;
      if (up && !input.voltage_down && selected_level < 2) ++selected_level;
      if (down_released) {
        if (down_pending && !down_consumed && !input.voltage_up && selected_level > 0)
          --selected_level;
        down_pending = false;
      }
      info.millivolts = levels[level];
      info.selected_rpm = targets[ff_level];
      info.status = "READY";
    } else if (need_release || input.voltage_forward == input.voltage_reverse) {
      info.running = false;
      need_release = true;
      info.status = "RELEASE";
    } else {
      down_pending = false;  // 运行中按下选择键不会累积切换或降档手势。
      const int requested_direction = input.voltage_forward ? 1 : -1;
      if (info.running && requested_direction != direction) {
        // 换向必须先松开运行键，不能在持续输出期间直接反向。
        info.running = false;
        need_release = true;
        info.status = "RELEASE";
      } else {
        if (!info.running) {
          direction = requested_direction;
          started_at = now_ms;
          info.running = true;
        }
        if (now_ms < started_at || now_ms - started_at >= timeout) {
          info.running = false;
          need_release = true;
          info.status = "TIMEOUT";
        } else {
          info.status = direction > 0 ? "FWD" : "REV";
        }
      }
    }

    if (!info.visible) return false;
    output.L_voltage = output.R_voltage = 0;
    if (info.running && info.feedforward) {
      const double target = direction * info.selected_rpm;
      const drive::feedforward_output left = (direction > 0 ? left_ff : left_reverse_ff).Calculate(target, 0.0);
      const drive::feedforward_output right = (direction > 0 ? right_ff : right_reverse_ff).Calculate(target, 0.0);
      if (!left.valid || !right.valid || left.saturated || right.saturated) {
        info.running = false;
        need_release = true;
        info.status = "FF ERROR";
      } else {
        output.L_voltage = static_cast<int>(std::lround(left.millivolts));
        output.R_voltage = static_cast<int>(std::lround(right.millivolts));
        info.target_rpm = target;
      }
    } else if (info.running) {
      output.L_voltage = output.R_voltage = direction * info.millivolts;
    }
    if (info.feedforward && info.heading_enabled) {
      const bool valid = heading.Update(info.running, inertial_ready, angle, now_ms,
                                        output.L_voltage, output.R_voltage);
      info.heading = heading.Info();
      if (!valid) {
        info.running = false;
        need_release = true;
        output.L_voltage = output.R_voltage = 0;
        info.target_rpm = 0.0;
        info.status = info.heading.ready ? "HDG ERROR" : "IMU WAIT";
      }
    }
    if (info.speed_control && info.running) {
      if (!ApplySpeed(speed, speed_valid, now_ms, output)) {
        info.running = false;
        need_release = true;
        output.L_voltage = output.R_voltage = 0;
        info.target_rpm = 0.0;
        info.status = "SPEED ERR";
        info.heading.active = false;
        ResetSpeed();
      }
    } else ResetSpeed();
    return true;
  }

 private:
  void ResetHeading() { heading.Reset(); info.heading = heading.Info(); }
  void ResetSpeed() {
    left_pid.Reset(); right_pid.Reset(); info.velocity = drive::velocity_loop_info();
  }
  bool ApplySpeed(const drive::velocity_feedback& speed, bool valid, double now,
                  manual::Output& output) {
    if (!valid || !have_speed || !std::isfinite(velocity_config.stale_ms) ||
        velocity_config.stale_ms <= 0 || now < speed_received_at ||
        now - speed_received_at > velocity_config.stale_ms) return false;
    drive::velocity_loop_info& loop = info.velocity;
    if (!loop.active || speed.sequence != applied_speed_sequence) {
      const double dt = loop.active ? (speed.time_ms - loop.time_ms) / 1000.0 : 0.0;
      // 航向外环产生差速目标，速度内环跟踪它，避免两个环互相抵消。
      const double yaw = info.heading.active ? info.heading.correction : 0.0;
      loop.left_target = info.target_rpm - yaw / (direction > 0 ? left_kV : left_reverse_kV);
      loop.right_target = info.target_rpm + yaw / (direction > 0 ? right_kV : right_reverse_kV);
      loop.left_error = loop.left_target - speed.left_rpm;
      loop.right_error = loop.right_target - speed.right_rpm;
      const double left_base = std::fabs(static_cast<double>(output.L_voltage));
      const double right_base = std::fabs(static_cast<double>(output.R_voltage));
      if (!left_pid.Update(loop.left_target, speed.left_rpm, dt,
              std::min(left_base * 0.5, 12000.0 - left_base), loop.left_correction) ||
          !right_pid.Update(loop.right_target, speed.right_rpm, dt,
              std::min(right_base * 0.5, 12000.0 - right_base), loop.right_correction)) return false;
      loop.time_ms = speed.time_ms;
      loop.active = true;
      applied_speed_sequence = speed.sequence;
    }
    output.L_voltage = static_cast<int>(std::lround(std::max(-12000.0,
        std::min(12000.0, output.L_voltage + loop.left_correction))));
    output.R_voltage = static_cast<int>(std::lround(std::max(-12000.0,
        std::min(12000.0, output.R_voltage + loop.right_correction))));
    return true;
  }
  drive::velocity_pid left_pid, right_pid;
  drive::velocity_pid_config velocity_config;
  bool velocity_configured = false;
  bool have_speed = false;
  unsigned int speed_sequence = 0;
  unsigned int applied_speed_sequence = 0;
  double speed_received_at = 0.0;
  double left_kV = 1.0, right_kV = 1.0;
  double left_reverse_kV = 1.0, right_reverse_kV = 1.0;
  drive::heading_hold heading;
  int levels[3];
  int level = 0;
  int targets[3] = {100, 150, 200};
  int ff_level = 0;
  drive::motor_feedforward left_ff{drive::feedforward_config()};
  drive::motor_feedforward right_ff{drive::feedforward_config()};
  drive::motor_feedforward left_reverse_ff{drive::feedforward_config()};
  drive::motor_feedforward right_reverse_ff{drive::feedforward_config()};
  double mode_hold_ms = 1000.0;
  double down_started_at = 0.0;
  bool down_pending = false;
  bool down_consumed = false;
  int direction = 1;
  double timeout;
  double started_at = 0.0;
  bool need_release = true;
  bool previous_up = false;
  bool previous_down = false;
  bool previous_allowed = false;
  voltage_test_info info;
};
