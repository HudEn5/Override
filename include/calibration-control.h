#pragma once

#include "driver-control-logic.h"
#include "drive-control.h"
#include "heading-control.h"
#include "velocity-control.h"
#include "drive-velocity.h"
#include <cmath>

struct calibration_info {
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
class calibration_control {
 public:
  calibration_control(const int (&voltages)[3], double timeout_ms) : timeout(timeout_ms) {
    for (int i = 0; i < 3; ++i) levels[i] = voltages[i];
    info.millivolts = levels[0];
  }

  const calibration_info& Info() const { return info; }

  void ConfigureHeading(const drive::heading_config& config) {
    heading.Configure(config);
    info.heading_enabled = true;
  }

  void ConfigureVelocityPID(const drive::velocity_pid_config& config) {
    velocity.ConfigurePID(config);
    velocity_configured = true;
  }

  void ConfigureFeedforward(const int (&rpm)[3], const drive::feedforward_config& left,
                            const drive::feedforward_config& right, double hold_ms) {
    for (int i = 0; i < 3; ++i) targets[i] = rpm[i];
    velocity.ConfigureForward(left, right);
    mode_hold_ms = hold_ms;
    info.selected_rpm = targets[ff_level];
  }

  // 不设置时后退沿用前进模型；有重复实测数据时可独立补偿正反向差异。
  void ConfigureReverseFeedforward(const drive::feedforward_config& left,
                                   const drive::feedforward_config& right) {
    velocity.ConfigureReverse(left, right);
  }

  bool Update(const manual::Input& input, manual::Output& output, bool allowed,
              bool motors_online, double now_ms, bool stopped = true,
              bool inertial_ready = false, double angle = 0.0,
              const drive::velocity_feedback& speed = drive::velocity_feedback()) {
    velocity.Observe(speed, now_ms);
    const bool up = previous_allowed && input.test_next && !previous_up;
    const bool down = previous_allowed && input.test_previous && !previous_down;
    const bool down_released = previous_down && !input.test_previous;
    previous_up = input.test_next;
    previous_down = input.test_previous;
    previous_allowed = allowed && motors_online;
    info.target_rpm = 0.0;
    const bool held = input.test_positive || input.test_negative;
    if (held || input.test_next || input.test_previous) info.visible = true;

    // 摇杆立即接回正常驾驶；按 A/B 先退出电压测试。
    if (output.L_voltage != 0 || output.R_voltage != 0 || input.test_start || input.test_cancel) {
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
        down_pending = !input.test_next && stopped;
        down_consumed = false;
        down_started_at = now_ms;
      }
      if (input.test_previous && down_pending) {
        if (input.test_next || !stopped || now_ms < down_started_at) down_pending = false;
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
      if (up && !input.test_previous && selected_level < 2) ++selected_level;
      if (down_released) {
        if (down_pending && !down_consumed && !input.test_next && selected_level > 0)
          --selected_level;
        down_pending = false;
      }
      info.millivolts = levels[level];
      info.selected_rpm = targets[ff_level];
      info.status = "READY";
    } else if (need_release || input.test_positive == input.test_negative) {
      info.running = false;
      need_release = true;
      info.status = "RELEASE";
    } else {
      down_pending = false;  // 运行中按下选择键不会累积切换或降档手势。
      const int requested_direction = input.test_positive ? 1 : -1;
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
    const double target = direction * info.selected_rpm;
    if (info.running && info.feedforward) {
      if (!velocity.Feedforward(target, target, output.L_voltage, output.R_voltage)) {
        info.running = false;
        need_release = true;
        info.status = "FF ERROR";
      } else {
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
      if (!ApplySpeed(speed, now_ms, output)) {
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
  void ResetHeading() {
    heading.Reset(); info.heading = heading.Info();
  }
  void ResetSpeed() {
    velocity.Reset(); info.velocity = velocity.Info();
  }
  bool ApplySpeed(const drive::velocity_feedback& speed, double now,
                  manual::Output& output) {
    // 外环的差速电压按当前运动方向的 kV 换算成两侧目标，内环跟踪它。
    const double yaw = info.heading.active ? info.heading.correction : 0;
    const double left_target = info.target_rpm - yaw / velocity.Slope(true, info.target_rpm);
    const double right_target = info.target_rpm + yaw / velocity.Slope(false, info.target_rpm);
    const bool valid = velocity.ApplyPID(left_target, right_target, speed, now,
                                        output.L_voltage, output.R_voltage);
    info.velocity = velocity.Info();
    return valid;
  }
  drive::velocity_control velocity;
  bool velocity_configured = false;
  drive::heading_hold heading;
  int levels[3];
  int level = 0;
  int targets[3] = {100, 150, 200};
  int ff_level = 0;
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
  calibration_info info;
};
