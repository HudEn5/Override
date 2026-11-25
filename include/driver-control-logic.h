#pragma once

// 独立于硬件的手控逻辑，不依赖自动路线。
namespace manual {
enum class stopping { configured, coast, brake, hold };
struct input {
  int y_axis = 0;
  int angular_axis = 0;
  bool arm_ready = true;  // 仅检查两侧电机是否在线，不使用编码器参与升降控制。
  bool arm_up = false;
  bool arm_down = false;
  bool claw_close = false;
  bool claw_open = false;
  bool toggle = false;
  bool test_start = false;
  bool test_cancel = false;
  bool telemetry = false;
  bool test_next = false;
  bool test_previous = false;
  bool test_positive = false;
  bool test_negative = false;
};

struct output {
  int left_mV = 0;
  int right_mV = 0;
  bool arm_enabled = false;
  int arm_mV = 0;  // 上升为正，下降为负；硬件层再处理统一方向及电机反转配置。
  int claw_mV = 0;
  stopping arm_stop = stopping::configured, claw_stop = stopping::configured;
  bool toggle_piston = false;
};

inline int AxisVoltage(int percent, int deadband, int max_mV) {
  if (percent > 100)
    percent = 100;
  if (percent < -100)
    percent = -100;
  const int magnitude = percent < 0 ? -percent : percent;
  if (magnitude <= deadband)
    return 0;
  const int voltage = (magnitude - deadband) * max_mV / (100 - deadband);
  return percent < 0 ? -voltage : voltage;
}

class driver_control {
 public:
  explicit driver_control(bool toggle_piston) : piston_state(toggle_piston) {}
  void SetPistonState(bool value) {
    piston_state = value;
  }

  output Update(const input& input, bool enabled, int deadband, int max_mV, int arm_up_mV, int arm_down_mV,
                int claw_mV) {
    output result;
    result.arm_enabled = enabled && input.arm_ready;
    if (enabled) {
      const int vy = AxisVoltage(input.y_axis, deadband, max_mV);
      const int omega = AxisVoltage(input.angular_axis, deadband, max_mV);
      result.left_mV = vy + omega;
      result.right_mV = vy - omega;
      // 左右混控超出上限时同比例缩小，保持转向比例。
      const int L_abs = result.left_mV < 0 ? -result.left_mV : result.left_mV;
      const int R_abs = result.right_mV < 0 ? -result.right_mV : result.right_mV;
      const int peak = L_abs > R_abs ? L_abs : R_abs;
      if (peak > max_mV) {
        result.left_mV = result.left_mV * max_mV / peak;
        result.right_mV = result.right_mV * max_mV / peak;
      }
      // 一个周期内直接响应按键；换向无需重新生成角度目标或等待滤波。
      if (result.arm_enabled && input.arm_up != input.arm_down)
        result.arm_mV = input.arm_up ? arm_up_mV : -arm_down_mV;
      if (input.claw_close != input.claw_open)
        result.claw_mV = input.claw_close ? claw_mV : -claw_mV;
      // 启用或重连时已按住 X，不计为新一次按下。
      if (previous_enabled && input.toggle && !previous_toggle)
        piston_state = !piston_state;
    }
    result.toggle_piston = piston_state;
    previous_toggle = input.toggle;
    previous_enabled = enabled;
    return result;
  }

 private:
  bool piston_state;
  bool previous_toggle = false;
  bool previous_enabled = false;
};
}  // 手控逻辑命名空间
