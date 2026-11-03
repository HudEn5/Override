#pragma once

// 独立于硬件的手控逻辑，不依赖自动路线。
namespace manual {
// 目标角最多领先实测角的度数，超过后目标跟随实测，避免堵转时目标无限增大。
constexpr double DEFAULT_ARM_MAX_LEAD = 5.0;
// 按住时连续多少个控制周期未累计取得有效方向位移，就判定为堵转。
constexpr int DEFAULT_ARM_STALL_UPDATES = 50;
// 按指令方向累计达到该角度才记为有效进展，不要求单周期达到，单位 deg。
constexpr double ARM_STALL_EPSILON = 0.05;

struct Input {
  int y_axis = 0;
  int angular_axis = 0;
  double arm_angle = 0.0;  // 已按上升方向归一化的两侧平均角度
  bool arm_ready = true;   // 两侧电机在线且位置差不超限
  bool arm_up = false;
  bool arm_down = false;
  bool claw_close = false;
  bool claw_open = false;
  bool toggle = false;
};

struct Output {
  int L_voltage = 0;
  int R_voltage = 0;
  bool arm_enabled = false;
  double arm_target_angle = 0.0;
  int claw_voltage = 0;
  bool toggle_piston = false;
};

inline int AxisVoltage(int percent, int deadband, int max_voltage) {
  if (percent > 100) percent = 100;
  if (percent < -100) percent = -100;
  const int magnitude = percent < 0 ? -percent : percent;
  if (magnitude <= deadband) return 0;
  const int voltage = (magnitude - deadband) * max_voltage / (100 - deadband);
  return percent < 0 ? -voltage : voltage;
}

inline double LimitAngle(double angle, double min_angle, double max_angle,
                         bool upper_limit_enabled = true) {
  if (angle < min_angle) return min_angle;
  if (upper_limit_enabled && angle > max_angle) return max_angle;
  return angle;
}

class driver_control {
 public:
  explicit driver_control(bool toggle_piston) : _toggle(toggle_piston) {}

  Output Update(const Input& input, bool enabled, int deadband, int max_voltage,
                double arm_step, int claw_voltage, double min_angle, double max_angle,
                bool upper_limit_enabled = true,
                double max_lead = DEFAULT_ARM_MAX_LEAD,
                int stall_updates = DEFAULT_ARM_STALL_UPDATES) {
    Output output;
    // 手臂只有在此使能且两侧在线、位置差正常时才参与控制。
    const bool arm_control = enabled && input.arm_ready;
    // 禁用、重新启用或手臂从异常恢复时跟随实测位置，避免追赶旧目标。
    if (!enabled || !pre_enabled) {
      arm_target_angle = LimitAngle(input.arm_angle, min_angle, max_angle, upper_limit_enabled);
      ResetStall(input.arm_angle);
    } else if (arm_control && !pre_arm_control) {
      arm_target_angle = LimitAngle(input.arm_angle, min_angle, max_angle, upper_limit_enabled);
      ResetStall(input.arm_angle);
    }
    const bool pressing = input.arm_up != input.arm_down;
    // 换向也视为一次新的按下，清除堵转锁存并从实测位置重新起步。
    const bool direction_changed =
        pressing && pre_pressing && (input.arm_up != pre_arm_up);
    if (enabled) {
      const int vy = AxisVoltage(input.y_axis, deadband, max_voltage);
      const int omega = AxisVoltage(input.angular_axis, deadband, max_voltage);
      output.L_voltage = vy + omega;
      output.R_voltage = vy - omega;
      // 转向输入为正时右转：左侧正转，右侧反转。
      // 超过电压上限时，两侧同比例缩小，保持转向比例。
      const int L_voltage_abs = output.L_voltage < 0 ? -output.L_voltage : output.L_voltage;
      const int R_voltage_abs = output.R_voltage < 0 ? -output.R_voltage : output.R_voltage;
      const int peak = L_voltage_abs > R_voltage_abs ? L_voltage_abs : R_voltage_abs;
      if (peak > max_voltage) {
        output.L_voltage = output.L_voltage * max_voltage / peak;
        output.R_voltage = output.R_voltage * max_voltage / peak;
      }
      if (arm_control) {
        // 相反按键同时按下时，手臂目标不变，夹爪停止。
        if (pressing && (!pre_pressing || direction_changed)) {
          // 新一次按下或换向时从实测位置起步，避免旧目标把手臂反向拉回。
          arm_target_angle = LimitAngle(input.arm_angle, min_angle, max_angle, upper_limit_enabled);
          ResetStall(input.arm_angle);
        }
        if (pressing) {
          if (!_stalled) {
            // 从上次有效进展的位置累计位移，避免把缓慢移动误判为静止。
            const double progress = (input.arm_angle - _stall_reference) *
                                    (input.arm_up ? 1.0 : -1.0);
            if (progress >= ARM_STALL_EPSILON) {
              _stall_count = 0;
              _stall_reference = input.arm_angle;
            } else {
              ++_stall_count;
            }
            if (_stall_count >= stall_updates) {
              _stalled = true;
              // 只在触发时记录保持目标，锁存期间不再跟随实测位置漂移。
              arm_target_angle = input.arm_angle;
            }
          }
          // 堵转后保持触发时的位置，松开再按下或换向后才能继续。
          if (!_stalled) {
            arm_target_angle += input.arm_up ? arm_step : -arm_step;
            // 目标最多领先实测 max_lead 度；手臂被卡住时目标不会越推越远。
            const double lead_low = input.arm_angle - max_lead;
            const double lead_high = input.arm_angle + max_lead;
            if (arm_target_angle < lead_low) arm_target_angle = lead_low;
            if (arm_target_angle > lead_high) arm_target_angle = lead_high;
          }
        } else {
          ResetStall(input.arm_angle);
        }
      }
      if (input.claw_close != input.claw_open)
        output.claw_voltage = input.claw_close ? claw_voltage : -claw_voltage;
      // 启用或重连时已经按住的按钮，不计为一次新的按下。
      if (pre_enabled && input.toggle && !pre_toggle) {
        _toggle = !_toggle;
      }
    }
    // 松开按钮后保留目标；同时按下 R1/R2 也不改变目标。
    arm_target_angle = LimitAngle(arm_target_angle, min_angle, max_angle, upper_limit_enabled);
    output.arm_target_angle = arm_target_angle;
    output.arm_enabled = arm_control;
    // 禁用时保持气动状态，手臂退出位置运动，其余电机指令归零。
    output.toggle_piston = _toggle;
    pre_toggle = input.toggle;
    pre_enabled = enabled;
    pre_arm_control = arm_control;
    pre_pressing = pressing;
    pre_arm_up = input.arm_up;
    return output;
  }

 private:
  void ResetStall(double angle) {
    _stall_count = 0;
    _stalled = false;
    _stall_reference = angle;
  }

  bool _toggle;
  bool pre_toggle = false;
  bool pre_enabled = false;
  bool pre_arm_control = false;
  bool pre_pressing = false;
  bool pre_arm_up = false;
  bool _stalled = false;
  int _stall_count = 0;
  double _stall_reference = 0.0;
  double arm_target_angle = 0.0;
};
}  // 手控逻辑命名空间
