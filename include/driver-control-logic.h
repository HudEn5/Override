#pragma once

// 独立于硬件的手控逻辑，不依赖自动路线。
namespace manual {
struct Input {
  int y_axis = 0;
  int angular_axis = 0;
  bool arm_ready = true;  // 仅检查两侧电机是否在线，不使用编码器参与升降控制。
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
  int arm_voltage = 0;  // 上升为正，下降为负；硬件层再处理统一方向及电机反转配置。
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

class driver_control {
 public:
  explicit driver_control(bool toggle_piston) : _toggle(toggle_piston) {}

  Output Update(const Input& input, bool enabled, int deadband, int max_voltage,
                int arm_up_voltage, int arm_down_voltage, int claw_voltage) {
    Output output;
    output.arm_enabled = enabled && input.arm_ready;
    if (enabled) {
      const int vy = AxisVoltage(input.y_axis, deadband, max_voltage);
      const int omega = AxisVoltage(input.angular_axis, deadband, max_voltage);
      output.L_voltage = vy + omega;
      output.R_voltage = vy - omega;
      // 左右混控超出上限时同比例缩小，保持转向比例。
      const int L_abs = output.L_voltage < 0 ? -output.L_voltage : output.L_voltage;
      const int R_abs = output.R_voltage < 0 ? -output.R_voltage : output.R_voltage;
      const int peak = L_abs > R_abs ? L_abs : R_abs;
      if (peak > max_voltage) {
        output.L_voltage = output.L_voltage * max_voltage / peak;
        output.R_voltage = output.R_voltage * max_voltage / peak;
      }
      // 一个周期内直接响应按键；换向无需重新生成角度目标或等待滤波。
      if (output.arm_enabled && input.arm_up != input.arm_down)
        output.arm_voltage = input.arm_up ? arm_up_voltage : -arm_down_voltage;
      if (input.claw_close != input.claw_open)
        output.claw_voltage = input.claw_close ? claw_voltage : -claw_voltage;
      // 启用或重连时已按住 X，不计为新一次按下。
      if (pre_enabled && input.toggle && !pre_toggle) _toggle = !_toggle;
    }
    output.toggle_piston = _toggle;
    pre_toggle = input.toggle;
    pre_enabled = enabled;
    return output;
  }

 private:
  bool _toggle;
  bool pre_toggle = false;
  bool pre_enabled = false;
};
}  // 手控逻辑命名空间
