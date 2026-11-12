#pragma once

#include "driver-control-logic.h"
#include "drive-control.h"
#include "heading-control.h"
#include "velocity-control.h"
#include "drive-velocity.h"
#include "trapezoid-motion.h"
#include "drive-straight.h"
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
  bool trapezoid = false;
  drive::trapezoid_info motion;
  const char* status = "READY";
};

// 测试控制器只生成电压，不读写设备；可比较前馈与前馈加速度闭环。
class voltage_test {
 public:
  voltage_test(const int (&voltages)[3], double timeout_ms) : timeout(timeout_ms) {
    straight_settings.heading_enabled = false;
    for (int i = 0; i < 3; ++i) levels[i] = voltages[i];
    info.millivolts = levels[0];
  }

  const voltage_test_info& Info() const { return info; }

  void ConfigureHeading(const drive::heading_config& config) {
    heading.Configure(config);
    info.heading_enabled = true;
    straight_settings.heading = config;
    straight_settings.heading_enabled = true;
    Straight.Configure(straight_settings);
  }

  void ConfigureVelocityPID(const drive::velocity_pid_config& config) {
    velocity.ConfigurePID(config);
    velocity_configured = true;
    straight_settings.pid = config;
    Straight.Configure(straight_settings);
  }

  void ConfigureTrapezoid(const drive::trapezoid_config& config) {
    straight_settings.motion = config;
    straight_settings.retain_heading = true;
    Straight.Configure(straight_settings);
    trapezoid_configured = true;
  }

  void ConfigureFeedforward(const int (&rpm)[3], const drive::feedforward_config& left,
                            const drive::feedforward_config& right, double hold_ms) {
    for (int i = 0; i < 3; ++i) targets[i] = rpm[i];
    velocity.ConfigureForward(left, right);
    straight_settings.left_forward = straight_settings.left_reverse = left;
    straight_settings.right_forward = straight_settings.right_reverse = right;
    Straight.Configure(straight_settings);
    mode_hold_ms = hold_ms;
    info.selected_rpm = targets[ff_level];
  }

  // 不设置时后退沿用前进模型；有重复实测数据时可独立补偿正反向差异。
  void ConfigureReverseFeedforward(const drive::feedforward_config& left,
                                   const drive::feedforward_config& right) {
    velocity.ConfigureReverse(left, right);
    straight_settings.left_reverse = left; straight_settings.right_reverse = right;
    Straight.Configure(straight_settings);
  }

  bool Update(const manual::Input& input, manual::Output& output, bool allowed,
              bool motors_online, double now_ms, bool stopped = true,
              bool inertial_ready = false, double angle = 0.0,
              const drive::velocity_feedback& speed = drive::velocity_feedback(),
              const drive::position_feedback& position = drive::position_feedback()) {
    velocity.Observe(speed, now_ms);
    drive::straight_feedback frame;
    frame.velocity = speed; frame.position = position; frame.motors_online = motors_online;
    frame.inertial_ready = inertial_ready; frame.heading_deg = angle; frame.stopped = stopped;
    Straight.Observe(frame, allowed, now_ms);
    bool beginning = false;
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
      CancelMotion();
      return false;
    }
    if (!allowed || !motors_online || !std::isfinite(now_ms)) {
      info.running = false;
      need_release = true;
      down_pending = false;
      ResetHeading();
      ResetSpeed();
      CancelMotion();
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
          else if (info.speed_control && trapezoid_configured && !info.trapezoid)
            info.trapezoid = true;
          else {
            info.feedforward = !info.feedforward;
            info.speed_control = false;
            info.trapezoid = false;
          }
          ResetHeading();  // 主动退出/重进前馈模式时才建立新一轮参考。
          ResetSpeed();
          CancelMotion();
          Straight.Reset(); info.motion = Straight.Info().motion;
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
      info.status = MotionStatus("READY");
    } else if (need_release || input.voltage_forward == input.voltage_reverse) {
      info.running = false;
      need_release = true;
      info.status = MotionStatus("RELEASE");
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
          beginning = true;
        }
        if (!info.trapezoid && (now_ms < started_at || now_ms - started_at >= timeout)) {
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
    if (info.trapezoid) {
      if (info.running) {
        if (beginning) {
          if (!std::isfinite(straight_settings.motion.distance_mm) || straight_settings.motion.distance_mm <= 0)
            Straight.Reject("TP CONFIG");
          else Straight.StartDriveStraight(direction * straight_settings.motion.distance_mm, info.selected_rpm);
        }
        Straight.UpdateDriveStraight();
        const drive::straight_info& value = Straight.Info();
        info.running = value.state == drive::straight_state::running;
        info.status = value.status;
        if (!info.running) need_release = true;
      } else Straight.CancelDriveStraight();
      const drive::straight_info& value = Straight.Info();
      info.motion = value.motion; info.heading = value.heading; info.velocity = value.velocity;
      info.target_rpm = info.running ? value.motion.target_rpm : 0;
      output.L_voltage = value.left_mV; output.R_voltage = value.right_mV;
      return true;
    }
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
    if (!info.running) CancelMotion();
    return true;
  }

 private:
  const char* MotionStatus(const char* otherwise) const {
    return info.trapezoid && (Straight.Info().state == drive::straight_state::done ||
        Straight.Info().state == drive::straight_state::error) ? Straight.Info().status : otherwise;
  }
  void ResetHeading() {
    heading.Reset(); Straight.ResetReference(); info.heading = heading.Info();
  }
  void ResetSpeed() {
    velocity.Reset(); info.velocity = velocity.Info();
  }
  void CancelMotion() { Straight.CancelDriveStraight(); info.motion = Straight.Info().motion; }
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
  drive::straight_controller Straight;
  drive::straight_config straight_settings;
  bool trapezoid_configured = false;
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
  voltage_test_info info;
};
