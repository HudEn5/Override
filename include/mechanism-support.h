#pragma once

#include <algorithm>
#include <cmath>

// 状态、触屏与线程通信辅助；动作和调参仍在 mechanisms.h。
namespace mechanisms {
enum class state { idle, running, done, cancelled, error };
class press_edge {
 public:
  bool update(bool pressed, bool allowed) {
    const bool fire = allowed && was_allowed && pressed && !previous;
    previous = pressed; was_allowed = allowed; return fire;
  }
 private:
  bool previous = true, was_allowed = false;
};
struct feedback {
  bool arm_online = false, arm_valid = false;
  double left_deg = 0, right_deg = 0;
  double left_deg_s = 0, right_deg_s = 0;
};
struct lift_info {
  state result = state::idle;
  const char* status = "NO ZERO";
  const char* command_status = "READY";
  bool homed = false;
  double left_deg = 0, right_deg = 0, target_deg = 0;
  double elapsed_ms = 0;
  double control_dt_ms = 0, requested_height_mm = 0;
  bool height_requested = false, height_mapped = false;
  int left_mV = 0, right_mV = 0;
  double position_deg() const { return 0.5 * left_deg + 0.5 * right_deg; }
};

class lift_pid {
 public:
  void reset() { integral = previous_error = 0; }
  template <typename Config>
  double update(double error, double rate, double dt_ms, const Config& c, bool integrate) {
    if (error * previous_error < 0 || c.kI == 0) integral = 0;
    if (error != 0) previous_error = error;
    const double base = c.kP * error - c.kD * rate;
    if (integrate && dt_ms > 0) {
      const double next = std::max(-c.maximum_integral_mV,
          std::min(c.maximum_integral_mV, integral + c.kI * error * dt_ms / 1000.0));
      const double raw = base + next;
      if ((raw >= -c.maximum_down_mV && raw <= c.maximum_up_mV) ||
          (raw > c.maximum_up_mV && error < 0) || (raw < -c.maximum_down_mV && error > 0)) integral = next;
    }
    return base + integral;
  }
 private:
  double integral = 0, previous_error = 0;
};

struct claw_info {
  state result = state::idle;
  const char* status = "READY";
  double duration_ms = 0, elapsed_ms = 0;
  int millivolts = 0;  // 正值闭合，负值张开；终态始终为 0。
};

// 主控屏幕底部三个按钮：确认零点、执行抬升、取消抬升。
constexpr int PANEL_TOP = 208, PANEL_HEIGHT = 32;
constexpr int PANEL_WIDTH = 480, BUTTON_WIDTH = 160;
static_assert(PANEL_TOP >= 0 && PANEL_TOP + PANEL_HEIGHT <= 240 &&
              PANEL_WIDTH <= 480 && BUTTON_WIDTH * 3 == PANEL_WIDTH,
              "Lift debug buttons must fit the V5 480x240 user drawing area");
enum class debug_action { none = 0, zero = 1, start = 2, cancel = 3 };

namespace detail {
// 返回 0（无点击）或第 1/2/3 个按钮；每个面板单独保存边沿状态。
inline int PanelButton(bool pressed, int x, int y, bool permitted, int top, bool& previous) {
  const bool new_press = pressed && !previous;
  previous = pressed;
  if (!permitted || !new_press || x < 0 || x >= PANEL_WIDTH ||
      y < top || y >= top + PANEL_HEIGHT) return 0;
  return x / BUTTON_WIDTH + 1;
}
}  // namespace detail

class debug_panel {
 public:
  debug_action Update(bool pressed, int x, int y, bool allowed) {
    return static_cast<debug_action>(detail::PanelButton(pressed, x, y, allowed, PANEL_TOP, previous_pressed));
  }

 private:
  bool previous_pressed = true;  // 启动时已经按住屏幕不能触发，先等一次松开。
};

constexpr int CLAW_PANEL_TOP = PANEL_TOP - PANEL_HEIGHT;
static_assert(CLAW_PANEL_TOP >= 0 && CLAW_PANEL_TOP + PANEL_HEIGHT <= PANEL_TOP,
              "Claw debug buttons must fit above the lift buttons");
enum class claw_debug_action { none = 0, open = 1, close = 2, cancel = 3 };

class claw_debug_panel {
 public:
  claw_debug_action Update(bool pressed, int x, int y, bool permitted) {
    return static_cast<claw_debug_action>(detail::PanelButton(pressed, x, y, permitted, CLAW_PANEL_TOP, previous_pressed));
  }

 private:
  bool previous_pressed = true;
};
// 主循环只提交一次性命令，线程消费后才执行；timeout 没有默认值。
enum class lift_command_kind { none, zero, angle, height, cancel };
struct lift_request {
  lift_command_kind kind = lift_command_kind::none;
  double target = 0, timeout_ms = 0;
  bool LiftTo(double degrees, double timeout) {
    kind = lift_command_kind::angle; target = degrees; timeout_ms = timeout; return true;
  }
  bool LiftHeightTo(double millimetres, double timeout) {
    kind = lift_command_kind::height; target = millimetres; timeout_ms = timeout; return true;
  }
  void zero_here() { kind = lift_command_kind::zero; }
  void cancel() { kind = lift_command_kind::cancel; }
};
struct lift_authority {
  bool manual_allowed = false, closed_loop_allowed = false, start_ready = false, field_connected = false;
  bool autonomous = false;
  const char* start_blocked = "NOT READY";
  double time_ms = -1;
};
struct lift_packet {
  lift_authority authority;
  lift_request command;
  double command_time_ms = -1;
  bool overflow = false;
  bool reset = false;
};
// 外部 mutex 保护；普通心跳不能覆盖未消费事件，失权与 cancel 优先于新动作。
class lift_mailbox {
 public:
  void publish(const lift_authority& next, const lift_request& command) {
    const bool changed = next.manual_allowed != packet.authority.manual_allowed ||
        next.closed_loop_allowed != packet.authority.closed_loop_allowed ||
        next.field_connected != packet.authority.field_connected || next.autonomous != packet.authority.autonomous;
    packet.authority = next;
    if (changed) { packet.reset = true; packet.command = lift_request(); packet.overflow = false; }
    if (command.kind == lift_command_kind::none) return;
    if (command.kind == lift_command_kind::cancel) {
      packet.command = command; packet.command_time_ms = next.time_ms;
    } else if (packet.command.kind != lift_command_kind::cancel) {
      if (packet.command.kind != lift_command_kind::none) {
        packet.command.cancel(); packet.overflow = true;
      } else packet.command = command;
      packet.command_time_ms = next.time_ms;
    }
  }
  lift_packet take() {
    const lift_packet result = packet;
    packet.command = lift_request(); packet.overflow = packet.reset = false;
    return result;
  }
 private:
  lift_packet packet;
};

}  // namespace mechanisms
