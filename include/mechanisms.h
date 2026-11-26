#pragma once

#include <algorithm>
#include <cmath>
#include "mechanism-control.h"

// 手臂与夹爪在这里计算输出，电机由 main 统一写入。
namespace mechanisms {
enum class state { idle, running, done, cancelled, error };
struct feedback {
  bool arm_online = false, arm_valid = false, claw_online = false;
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
  double reference_deg_s = 0, measured_deg_s = 0;
  double control_dt_ms = 0, requested_height_mm = 0;
  bool height_requested = false, height_mapped = false;
  int left_mV = 0, right_mV = 0;
  double position_deg() const {
    return 0.5 * left_deg + 0.5 * right_deg;
  }
};

struct claw_info {
  state result = state::idle;
  const char* status = "READY";
  double duration_ms = 0, elapsed_ms = 0;
  int millivolts = 0;  // 正值闭合或夹持，负值张开。
};

struct arm_tuning_info {
  bool visible = false;
  unsigned run = 0;
  int up_mV = 4000, down_mV = 2500;
  double time_ms = 0, elapsed_ms = 0, left_deg_s = 0, right_deg_s = 0;
  const char* status = "READY";
};

// R1/R2 点动测定电压响应。松键、超时和行程限制都撤压并等待按键释放。
class arm_tuning_test {
 public:
  arm_tuning_test(int minimum_up, int maximum_up, int down, double maximum_deg, double pulse_ms)
      : minimum_up_mV(minimum_up), maximum_up_mV(maximum_up), upper_deg(maximum_deg), duration_ms(pulse_ms) {
    info.up_mV = minimum_up;
    info.down_mV = down;
  }
  const arm_tuning_info& Update(bool enabled, const manual::input& input, const feedback& frame,
                                const lift_info& lift, double now, manual::output& output) {
    info.visible = enabled;
    if (!enabled) {
      direction = 0;
      released = false;
      info.status = "READY";
      return info;
    }
    output.arm_enabled = true;
    output.arm_mV = 0;
    output.arm_stop = manual::stopping::hold;
    info.time_ms = now;
    info.left_deg_s = frame.left_deg_s;
    info.right_deg_s = frame.right_deg_s;
    const int requested = static_cast<int>(input.arm_up) - static_cast<int>(input.arm_down);
    const int step = static_cast<int>(input.test_next && !last_up) -
                     static_cast<int>(input.test_previous && !last_down);
    last_up = input.test_next;
    last_down = input.test_previous;
    if (input.test_cancel || !frame.arm_online || !frame.arm_valid || !lift.homed) {
      released = false;
      return Stop(input.test_cancel ? "STOP" : "ARM NOT READY");
    }
    if (direction == 0) {
      if (!input.arm_up && !input.arm_down) {
        released = true;
        info.up_mV = std::max(minimum_up_mV, std::min(maximum_up_mV, info.up_mV + 1000 * step));
        return info;
      }
      if (requested == 0 || !released)
        return info;
      direction = requested;
      released = false;
      started_ms = now;
      ++info.run;
    }
    info.elapsed_ms = now - started_ms;
    if (requested != direction)
      return Stop("RELEASE");
    if (!std::isfinite(info.elapsed_ms) || info.elapsed_ms < 0 || info.elapsed_ms >= duration_ms)
      return Stop("PULSE END");
    if ((direction > 0 && std::max(lift.left_deg, lift.right_deg) >= upper_deg) ||
        (direction < 0 && std::min(lift.left_deg, lift.right_deg) <= 0.0))
      return Stop("ANGLE LIMIT");
    if (std::fabs(lift.left_deg - lift.right_deg) > 12)
      return Stop("ARM MISMATCH");
    output.arm_mV = direction > 0 ? info.up_mV : -info.down_mV;
    info.status = "RUN";
    return info;
  }

 private:
  const arm_tuning_info& Stop(const char* status) {
    direction = 0;
    info.status = status;
    return info;
  }
  arm_tuning_info info;
  const int minimum_up_mV, maximum_up_mV;
  const double upper_deg, duration_ms;
  double started_ms = 0;
  int direction = 0;
  bool released = false, last_up = false, last_down = false;
};

// 自动路线使用的具名动作编号；角度相对于启动时的机械起点，不是关节角或毫米。
namespace preset {
constexpr int lift_start = 0, lift_raised = 1;
constexpr int claw_open = 0, claw_close = 1;
}
constexpr double LIFT_START_DEG = 0;
constexpr double LIFT_RAISED_DEG = 240;
constexpr double CLAW_OPEN_MS = 200, CLAW_CLOSE_MS = 200;

struct height_point {
  double height_mm = 0, motor_deg = 0;
};
struct height_calibration {
  // 未测量时保持 0；实测同一最低零点的三组“升高毫米、屏幕平均角度”后填入。
  int point_count = 0;
  height_point points[3];
};
struct lift_config {
  // 两台电机通过同一根轴：使用相对零点的平均输出轴角度，不是高度或关节角度。
  double minimum_deg = 0, maximum_deg = 600;
  // 5 V 带 Pin 能通过全行程；下降实测 -2.5 V 过快，使用较低的前馈和目标速度。
  int up_cruise_mV = 5000, up_approach_mV = 5000;
  double up_support_zone_deg = 6;  // 距目标这么近时，上升电压至少给末段前馈值。
  int up_stall_assist_mV = 1000;  // 上升末段两侧停滞时的最大补力，仍受上升电压上限约束。
  int down_cruise_mV = 1400, down_approach_mV = 650;
  int maximum_up_mV = 6000, maximum_down_mV = 2500;
  double max_up_deg_s = 80, max_down_deg_s = 60;
  double approach_deg = 18, velocity_kP = 29, rate_filter_ms = 35;
  double tolerance_deg = 2, stopped_deg_s = 8, settle_ms = 200;
  double down_to_height_settle_ms = 120;  // 降到非零放料高度时的稳定确认时间。
  // 进内层窗口后由电机 hold 接管，留时间吸收皮筋回弹。
  double hold_capture_deg = 1, hold_recheck_ms = 240;
  double hold_drift_deg = 1;  // 停稳窗口内每侧编码器允许的位移。
  double maximum_gap_ms = 100;
  double sensor_grace_ms = 100;  // 升降中读数异常时先保持，允许短暂恢复。
  double assist_delay_ms = 120, assist_ramp_ms = 100, assist_progress_deg = 1;
  int assist_minimum_mV = 1500;
  height_calibration height;
};
struct claw_config {
  int voltage_mV = 6000;
  int hold_mV = 0;
  double maximum_duration_ms = 5000, maximum_gap_ms = 100;
};
class lift_controller {
 public:
  void configure(const lift_config& value) {
    config = value;
    cancel();
  }
  const lift_info& info() const {
    return value;
  }
  bool at_target() const {
    return value.result == state::done;
  }
  bool busy() const {
    return value.result == state::running;
  }
  void notice(const char* reason) {
    value.command_status = reason;
  }

  void observe(const feedback& frame, bool permitted, double time_ms) {
    data = frame;
    allowed = permitted;
    const double gap = have_frame ? time_ms - now : 0;
    value.control_dt_ms = gap;
    clock_valid =
        std::isfinite(time_ms) && time_ms >= 0 && (!have_frame || (gap >= 0 && gap <= config.maximum_gap_ms));
    now = time_ms;
    have_frame = true;
    if (!clock_valid || !data.arm_online) {
      if (value.homed || busy())
        fail(!clock_valid ? "CLOCK/STALE" : "MOTOR OFF");
      value.homed = false;
      return;
    }
    if (!valid_feedback()) {
      if (!allowed)
        cancel();
      else if (busy())
        pause_for_sensor();
      return;
    }
    if (sensor_bad_since >= 0)
      value.measured_deg_s = 0.5 * data.left_deg_s + 0.5 * data.right_deg_s;
    sensor_bad_since = -1;
    value.left_deg = data.left_deg - zero_left;
    value.right_deg = data.right_deg - zero_right;
    if (!allowed) {
      cancel();
      return;
    }
    if (busy())
      update();
  }

  // startup=true 仅由主循环在上电首次有效反馈时调用；当前位置作为相对零点，不找机械限位。
  bool zero_here(bool startup = false) {
    if (busy())
      return false;
    if (!have_frame || !clock_valid || !valid_feedback() || !valid_config() ||
        (!startup && (!allowed || !stopped())))
      return false;
    zero_left = data.left_deg;
    zero_right = data.right_deg;
    value = lift_info();
    value.homed = true;
    value.status = "ZEROED";
    return true;
  }

  // 绝对目标相对于本次 zero_here 的参考；忙时拒绝覆盖，显式 cancel 后才能换目标。
  bool LiftTo(double target_deg, double timeout_ms) {
    if (busy())
      return false;
    value.height_requested = value.height_mapped = false;
    if (!valid_config())
      return fail("CONFIG");
    if (!std::isfinite(timeout_ms) || timeout_ms <= 0)
      return fail("TIMEOUT CONFIG");
    if (!allowed)
      return fail("DISABLED");
    if (!have_frame || !clock_valid || !valid_feedback())
      return fail("ARM SENSOR");
    if (!value.homed)
      return fail("NO ZERO");
    if (!std::isfinite(target_deg) || target_deg < config.minimum_deg || target_deg > config.maximum_deg)
      return fail("TARGET LIMIT");
    raising = target_deg != 0 && target_deg > value.position_deg();
    settle_duration = target_deg > 0 && target_deg < value.position_deg() ? config.down_to_height_settle_ms
                                                                        : config.settle_ms;
    value.target_deg = target_deg;
    value.result = state::running;
    value.status = "MOVING";
    started = previous_update = now;
    sensor_bad_since = -1;
    timeout = timeout_ms;
    settle_started = -1;
    hold_started = -1;
    last_progress_ms = -1;
    value.measured_deg_s = 0.5 * data.left_deg_s + 0.5 * data.right_deg_s;
    update();
    return value.result != state::error;
  }
  // 毫米入口：仅在实测三点区间内插值，禁止用猜测比例或外推驱动机构。
  bool LiftHeightTo(double height_mm, double timeout_ms) {
    if (busy())
      return false;
    value.height_requested = true;
    value.height_mapped = false;
    value.requested_height_mm = height_mm;
    if (!std::isfinite(timeout_ms) || timeout_ms <= 0)
      return fail("TIMEOUT CONFIG");
    if (config.height.point_count == 0)
      return fail("HEIGHT NOT CAL");
    if (config.height.point_count != 3 || !valid_config())
      return fail("HEIGHT CONFIG");
    const height_point* p = config.height.points;
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(p[i].height_mm) || !std::isfinite(p[i].motor_deg) || p[i].height_mm < 0 ||
          p[i].motor_deg < config.minimum_deg || p[i].motor_deg > config.maximum_deg ||
          (i > 0 && (p[i].height_mm <= p[i - 1].height_mm || p[i].motor_deg <= p[i - 1].motor_deg)))
        return fail("HEIGHT CONFIG");
    }
    if (!std::isfinite(height_mm) || height_mm < p[0].height_mm || height_mm > p[2].height_mm)
      return fail("HEIGHT LIMIT");
    const int lo = height_mm <= p[1].height_mm ? 0 : 1;
    const double ratio = (height_mm - p[lo].height_mm) / (p[lo + 1].height_mm - p[lo].height_mm);
    const double degrees = p[lo].motor_deg + ratio * (p[lo + 1].motor_deg - p[lo].motor_deg);
    const bool accepted = LiftTo(degrees, timeout_ms);
    value.height_requested = value.height_mapped = true;
    value.requested_height_mm = height_mm;
    value.target_deg = degrees;
    return accepted;
  }
  void cancel() {
    if (busy()) {
      value.result = state::cancelled;
      value.status = "CANCEL";
    }
    value.left_mV = value.right_mV = 0;
  }
  void fault(const char* reason) {
    fail(reason);
  }

 private:
  bool valid_config() const {
    const double numbers[] = {config.minimum_deg,
                              config.maximum_deg,
                              config.max_up_deg_s,
                              config.max_down_deg_s,
                              config.approach_deg,
                              config.up_support_zone_deg,
                              config.velocity_kP,
                              config.rate_filter_ms,
                              config.tolerance_deg,
                              config.stopped_deg_s,
                              config.settle_ms,
                              config.down_to_height_settle_ms,
                              config.hold_capture_deg,
                              config.hold_recheck_ms,
                              config.hold_drift_deg,
                              config.maximum_gap_ms,
                              config.sensor_grace_ms,
                              config.assist_delay_ms,
                              config.assist_ramp_ms,
                              config.assist_progress_deg};
    for (double n : numbers)
      if (!std::isfinite(n))
        return false;
    return config.minimum_deg <= 0 && config.maximum_deg > 0 &&
           config.max_up_deg_s > 0 && config.max_down_deg_s > 0 && config.approach_deg > 0 &&
           config.velocity_kP > 0 && config.rate_filter_ms > 0 &&
           config.maximum_up_mV > 0 && config.maximum_up_mV <= 12000 && config.maximum_down_mV > 0 &&
           config.maximum_down_mV <= 12000 && config.tolerance_deg > 0 && config.stopped_deg_s > 0 &&
           config.up_approach_mV >= 0 && config.up_approach_mV <= config.up_cruise_mV &&
           config.up_cruise_mV <= config.maximum_up_mV && config.up_support_zone_deg > 0 &&
           config.up_stall_assist_mV >= 0 && config.up_stall_assist_mV <= config.maximum_up_mV &&
           config.down_approach_mV >= 0 &&
           config.down_approach_mV <= config.down_cruise_mV && config.down_cruise_mV <= config.maximum_down_mV &&
           config.hold_capture_deg > 0 && config.hold_capture_deg <= config.tolerance_deg &&
           config.hold_recheck_ms > 0 && config.hold_drift_deg > 0 &&
           config.settle_ms >= 0 && config.down_to_height_settle_ms >= 0 &&
           config.maximum_gap_ms > 0 && config.sensor_grace_ms >= 0 &&
           config.assist_delay_ms >= 0 && config.assist_ramp_ms > 0 && config.assist_progress_deg > 0 &&
           config.assist_minimum_mV > 0 && config.assist_minimum_mV <= 12000;
  }
  bool valid_feedback() const {
    return data.arm_online && data.arm_valid && std::isfinite(data.left_deg - zero_left) &&
           std::isfinite(data.right_deg - zero_right) && std::isfinite(data.left_deg_s) &&
           std::isfinite(data.right_deg_s) && std::fabs(data.left_deg_s) <= 720 &&
           std::fabs(data.right_deg_s) <= 720;
  }
  bool stopped() const {
    return std::fabs(data.left_deg_s) <= config.stopped_deg_s &&
           std::fabs(data.right_deg_s) <= config.stopped_deg_s;
  }
  void pause_for_sensor() {
    if (sensor_bad_since < 0)
      sensor_bad_since = now;
    value.elapsed_ms = now - started;
    value.left_mV = value.right_mV = 0;
    value.reference_deg_s = 0;
    value.status = "ARM WAIT";
    last_progress_ms = -1;
    settle_started = -1;
    previous_update = now;
    if (value.elapsed_ms >= timeout)
      fail("TIMEOUT");
    else if (now - sensor_bad_since >= config.sensor_grace_ms)
      fail("ARM SENSOR");
  }
  int voltage(double raw) const {
    raw = std::max(-static_cast<double>(config.maximum_down_mV),
                   std::min(static_cast<double>(config.maximum_up_mV), raw));
    // 共轴两台必须始终同电压；任一编码器触及方向边界，两台一起停止。
    if ((raw < 0 && (value.left_deg <= config.minimum_deg || value.right_deg <= config.minimum_deg)) ||
        (raw > 0 && (value.left_deg >= config.maximum_deg || value.right_deg >= config.maximum_deg)))
      return 0;
    return static_cast<int>(raw);
  }
  void apply_upward_assist(double remaining_deg) {
    if (value.left_mV < config.assist_minimum_mV) {
      last_progress_ms = -1;
      return;
    }
    // 两侧有任一侧继续上升就重新计时。经过一段停滞后才渐进补力。
    const bool start_tracking = last_progress_ms < 0;
    const double positions[] = {value.left_deg, value.right_deg};
    for (int side = 0; side < 2; ++side) {
      if (start_tracking || positions[side] - progress_deg[side] >= config.assist_progress_deg) {
        progress_deg[side] = positions[side];
        last_progress_ms = now;
      }
    }
    if (value.reference_deg_s > 0 && remaining_deg <= config.approach_deg + config.hold_capture_deg &&
        (raising || stopped())) {
      const double ramp = (now - last_progress_ms - config.assist_delay_ms) / config.assist_ramp_ms;
      const double assist = std::min(1.0, std::max(0.0, ramp));
      value.left_mV = value.right_mV = voltage(value.left_mV + config.up_stall_assist_mV * assist);
    }
  }
  void update() {
    const double dt = now - previous_update;
    previous_update = now;
    value.elapsed_ms = now - started;
    if (value.elapsed_ms > timeout) {
      fail("TIMEOUT");
      return;
    }
    if (value.left_deg < config.minimum_deg - config.tolerance_deg ||
        value.right_deg < config.minimum_deg - config.tolerance_deg ||
        value.left_deg > config.maximum_deg + config.tolerance_deg ||
        value.right_deg > config.maximum_deg + config.tolerance_deg) {
      fail("POSITION LIMIT");
      return;
    }
    const double error = value.target_deg - value.position_deg();
    const double rate = 0.5 * data.left_deg_s + 0.5 * data.right_deg_s;
    value.measured_deg_s += (rate - value.measured_deg_s) * dt / (config.rate_filter_ms + dt);
    const double remaining_deg = std::fabs(error);
    const bool in_window = remaining_deg <= config.tolerance_deg;
    // 从下方逼近时进入内窗口再保持；已越过目标但仍在到位范围内也可保持。
    const bool capture = raising ? in_window && error <= config.hold_capture_deg
                                 : remaining_deg <= config.hold_capture_deg ||
                                       (in_window && (value.target_deg == 0 || stopped()));
    const bool dropped = raising && error > config.tolerance_deg;
    value.reference_deg_s = 0;
    if (hold_started < 0 && capture)
      hold_started = now;
    if (hold_started >= 0 && (in_window || (!dropped && now - hold_started < config.hold_recheck_ms))) {
      value.left_mV = value.right_mV = 0;
      value.status = in_window ? "SETTLING" : "HOLDING";
      if (!in_window) {
        settle_started = -1;
      } else if (settle_started < 0 ||
                 std::fabs(value.left_deg - settle_left_deg) > config.hold_drift_deg ||
                 std::fabs(value.right_deg - settle_right_deg) > config.hold_drift_deg) {
        // 用两侧角度的变化判断停稳，避免瞬时速度跳动打断等待。
        settle_started = now;
        settle_left_deg = value.left_deg;
        settle_right_deg = value.right_deg;
      }
      if (in_window && now - settle_started >= settle_duration) {
        value.result = state::done;
        value.status = "DONE";
      }
    } else {
      hold_started = -1;
      settle_started = -1;
      value.status = "MOVING";
      const double fraction = std::min(1.0, (remaining_deg - config.hold_capture_deg) / config.approach_deg);
      const bool up = error > 0;
      const int near_mV = up ? config.up_approach_mV : config.down_approach_mV;
      const int cruise_mV = up ? config.up_cruise_mV : config.down_cruise_mV;
      const double feedforward = std::copysign(near_mV + (cruise_mV - near_mV) * fraction, error);
      value.reference_deg_s = std::copysign((up ? config.max_up_deg_s : config.max_down_deg_s) * fraction, error);
      const double raw = feedforward + config.velocity_kP * (value.reference_deg_s - value.measured_deg_s);
      if (!std::isfinite(raw)) {
        fail("CONTROL");
        return;
      }
      const bool needs_up_support = raising && up && remaining_deg <= config.up_support_zone_deg;
      const double supported = needs_up_support ? std::max(raw, static_cast<double>(config.up_approach_mV)) : raw;
      value.left_mV = value.right_mV = voltage(supported);
    }
    apply_upward_assist(remaining_deg);
    // 截止时刻已满足到位/停稳可DONE，仍在运行则与调度器一致立即撤压。
    if (busy() && value.elapsed_ms >= timeout)
      fail("TIMEOUT");
  }
  bool fail(const char* reason) {
    value.result = state::error;
    value.status = reason;
    value.left_mV = value.right_mV = 0;
    return false;
  }
  lift_config config;
  lift_info value;
  feedback data;
  bool have_frame = false, allowed = false, clock_valid = false, raising = false;
  double now = 0, zero_left = 0, zero_right = 0, started = 0, timeout = 0, settle_started = -1;
  double settle_left_deg = 0, settle_right_deg = 0, settle_duration = 0;
  double last_progress_ms = -1;
  double progress_deg[2] = {};
  double previous_update = 0, hold_started = -1, sensor_bad_since = -1;
};

// 夹爪与抬升调试放在同一文件；此处只计算输出，由主循环统一写电机。
class claw_controller {
 public:
  void configure(const claw_config& next) {
    config = next;
    cancel();
  }
  const claw_info& info() const {
    return value;
  }
  bool busy() const {
    return value.result == state::running;
  }
  void reject(const char* reason) {
    if (!busy())
      fail(reason);
  }

  void observe(bool motor_online, bool permitted, double time_ms) {
    online = motor_online;
    allowed = permitted;
    const double gap = have_frame ? time_ms - now : 0;
    clock_valid =
        std::isfinite(time_ms) && time_ms >= 0 && (!have_frame || (gap >= 0 && gap <= config.maximum_gap_ms));
    now = time_ms;
    have_frame = true;
    if ((busy() || value.millivolts != 0) && (!clock_valid || !online)) {
      fail(!clock_valid ? "CLOCK/STALE" : "MOTOR OFF");
      return;
    }
    if (!allowed) {
      cancel();
      return;
    }
    if (busy())
      update();
  }

  // duration 与 timeout 均为毫秒；timeout 必须显式指定，运行中拒绝覆盖。
  bool ClawOpen(double duration_ms, double timeout_ms) {
    return start(duration_ms, timeout_ms, -1, "OPENING");
  }
  bool ClawClose(double duration_ms, double timeout_ms) {
    return start(duration_ms, timeout_ms, 1, "CLOSING");
  }
  void cancel() {
    if (busy()) {
      value.result = state::cancelled;
      value.status = "CANCEL";
    }
    value.millivolts = 0;
  }

 private:
  bool valid_config() const {
    return config.voltage_mV > 0 && config.voltage_mV <= 12000 && config.hold_mV >= 0 &&
           config.hold_mV <= config.voltage_mV && std::isfinite(config.maximum_duration_ms) &&
           config.maximum_duration_ms > 0 && std::isfinite(config.maximum_gap_ms) &&
           config.maximum_gap_ms > 0;
  }
  bool start(double duration_ms, double timeout_ms, int sign, const char* status) {
    if (busy())
      return false;
    if (!valid_config())
      return fail("CONFIG");
    if (!std::isfinite(duration_ms) || duration_ms < 0 || duration_ms > config.maximum_duration_ms)
      return fail("DURATION CONFIG");
    if (!std::isfinite(timeout_ms) || timeout_ms <= 0)
      return fail("TIMEOUT CONFIG");
    if (!have_frame || !clock_valid || !allowed || !online)
      return fail("DISABLED/FEEDBACK");
    value = claw_info();
    value.result = state::running;
    value.status = status;
    value.duration_ms = duration_ms;
    started = now;
    timeout = timeout_ms;
    direction = sign;
    update();
    return value.result != state::error;
  }
  void update() {
    value.elapsed_ms = now - started;
    // 截止时刻完成可DONE；迟到或截止时尚未完成均超时，DONE不证明夹持成功。
    if (value.elapsed_ms > timeout || (value.elapsed_ms >= timeout && value.elapsed_ms < value.duration_ms)) {
      fail("TIMEOUT");
      return;
    }
    if (value.elapsed_ms >= value.duration_ms) {
      value.result = state::done;
      value.millivolts = direction > 0 ? config.hold_mV : 0;
      value.status = value.millivolts != 0 ? "GRIP" : "DONE";
      return;
    }
    value.millivolts = direction * config.voltage_mV;
  }
  bool fail(const char* reason) {
    value.result = state::error;
    value.status = reason;
    value.millivolts = 0;
    return false;
  }
  claw_config config;
  claw_info value;
  bool have_frame = false, online = false, allowed = false, clock_valid = false;
  double now = 0, started = 0, timeout = 0;
  int direction = 0;
};

inline drive::motion_state RouteState(state value) {
  switch (value) {
    case state::running:
      return drive::motion_state::running;
    case state::done:
      return drive::motion_state::done;
    case state::cancelled:
      return drive::motion_state::cancelled;
    case state::error:
      return drive::motion_state::error;
    default:
      return drive::motion_state::idle;
  }
}

// 每轮先 Observe 输入快照，调度器再 Update；没有新反馈时禁止继续使用旧数据。
class lift_adapter : public mechanism::controller {
 public:
  void Observe(const feedback& frame, bool permitted, double now) {
    data = frame;
    allowed = permitted;
    observed_ms = now;
    have_frame = true;
    if (!control.busy() || !permitted)
      Update(now);
  }
  bool Start(int preset_id, double timeout_ms, double now) override {
    Update(now);
    if (!have_frame || !std::isfinite(now) || now != observed_ms)
      return false;
    if (preset_id != preset::lift_start && preset_id != preset::lift_raised) {
      control.fault("LIFT PRESET");
      return false;
    }
    return control.LiftTo(preset_id == preset::lift_start ? LIFT_START_DEG : LIFT_RAISED_DEG, timeout_ms);
  }
  bool Start(const mechanism::request& action, double timeout_ms, double now) override {
    if (action.kind == mechanism::operation::preset)
      return Start(action.preset, timeout_ms, now);
    Update(now);
    if (!have_frame || !std::isfinite(now) || now != observed_ms)
      return false;
    if (action.kind != mechanism::operation::lift_angle) {
      control.fault("LIFT ACTION");
      return false;
    }
    return control.LiftTo(action.target, timeout_ms);
  }
  void Update(double now) override {
    if (!have_frame || !std::isfinite(now) || now != observed_ms) {
      control.fault("LIFT STALE");
      return;
    }
    control.observe(data, allowed, now);
    // 只建立一次起点，断线恢复不得悄悄把当前位置重新当成机械零点。
    if (!zeroed && control.zero_here(true))
      zeroed = true;
  }
  mechanism::status Info() const override {
    mechanism::status result;
    result.state = RouteState(control.info().result);
    result.reason = control.info().status;
    result.command.motor_mV = control.info().left_mV;
    result.command.stop = manual::stopping::hold;
    return result;
  }
  const lift_info& LiftInfo() const {
    return control.info();
  }
  void Cancel() override {
    control.cancel();
  }

 private:
  lift_controller control;
  feedback data;
  bool allowed = false, have_frame = false, zeroed = false;
  double observed_ms = -1;
};

class claw_adapter : public mechanism::controller {
 public:
  explicit claw_adapter(int close_direction) : direction(close_direction) {}
  void Observe(bool online, bool permitted, double now) {
    motor_online = online;
    allowed = permitted;
    observed_ms = now;
    have_frame = true;
    if (!control.busy() || !permitted)
      Update(now);
  }
  bool Start(int preset_id, double timeout_ms, double now) override {
    Update(now);
    if (direction != 1 && direction != -1) {
      control.reject("CLAW DIRECTION");
      return false;
    }
    if (control.busy())
      return false;
    control.configure(claw_config());  // 预设始终恢复原6V，不能继承上一条直接动作的电压。
    if (preset_id == preset::claw_open)
      return control.ClawOpen(CLAW_OPEN_MS, timeout_ms);
    if (preset_id == preset::claw_close)
      return control.ClawClose(CLAW_CLOSE_MS, timeout_ms);
    control.reject("CLAW PRESET");
    return false;
  }
  bool Start(const mechanism::request& action, double timeout_ms, double now) override {
    if (action.kind == mechanism::operation::preset)
      return Start(action.preset, timeout_ms, now);
    Update(now);
    if ((direction != 1 && direction != -1) || !std::isfinite(action.volts) || action.volts <= 0 ||
        action.volts > 12 || !std::isfinite(action.hold_volts) || action.hold_volts < 0 ||
        action.hold_volts > action.volts) {
      control.reject("CLAW VOLTAGE/DIR");
      return false;
    }
    if (action.kind != mechanism::operation::claw_open && action.kind != mechanism::operation::claw_close) {
      control.reject("CLAW ACTION");
      return false;
    }
    if (control.busy())
      return false;
    claw_config settings;
    settings.voltage_mV = static_cast<int>(std::round(action.volts * 1000));
    settings.hold_mV = static_cast<int>(std::round(action.hold_volts * 1000));
    control.configure(settings);
    return action.kind == mechanism::operation::claw_open ? control.ClawOpen(action.target, timeout_ms)
                                                          : control.ClawClose(action.target, timeout_ms);
  }
  void Update(double now) override {
    if (!have_frame || !std::isfinite(now) || now != observed_ms) {
      control.observe(false, false, now);
      return;
    }
    control.observe(motor_online, allowed, now);
  }
  mechanism::status Info() const override {
    mechanism::status result;
    result.state = RouteState(control.info().result);
    result.reason = control.info().status;
    result.command.motor_mV = direction * control.info().millivolts;
    result.command.stop = manual::stopping::hold;
    return result;
  }
  const claw_info& ClawInfo() const {
    return control.info();
  }
  void Cancel() override {
    control.cancel();
  }

 private:
  claw_controller control;
  int direction;
  bool motor_online = false, allowed = false, have_frame = false;
  double observed_ms = -1;
};
}  // 机构算法与适配器
