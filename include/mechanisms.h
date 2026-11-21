#pragma once

#include <algorithm>
#include <cmath>
#include "mechanism-support.h"

// 抬升与夹爪的参数、动作集中在这里；独立线程负责抬升反馈和电机写入。
// 本文件不等待、不创建线程、不访问电机；夹爪仍由原主循环更新。
namespace mechanisms {
// 日常调参入口：抬升为平均电机角度；夹爪为通电时长；timeout 单位毫秒且必填。
template <typename Lift>
inline bool StartLiftDebug(Lift& lift) {
  return lift.LiftTo(240, 3000);
}
template <typename Claw>
inline bool StartClawOpenDebug(Claw& claw) {
  return claw.ClawOpen(200, 1000);
}
template <typename Claw>
inline bool StartClawCloseDebug(Claw& claw) {
  return claw.ClawClose(200, 1000);
}

constexpr double LIFT_CONTROL_PERIOD_MS = 10, LIFT_COMMAND_LEASE_MS = 100;
constexpr bool LIFT_WITH_ROUTE = true;  // A 启动原底盘路线时，并行启动上方同一抬升调用。
struct height_point { double height_mm = 0, motor_deg = 0; };
struct height_calibration {
  // 未测量时保持 0；实测同一最低零点的三组“升高毫米、屏幕平均角度”后填入。
  int point_count = 0;
  height_point points[3];
};
struct lift_config {
  // 两台电机通过同一根轴：使用相对零点的平均输出轴角度，不是高度或关节角度。
  double minimum_deg = 0, maximum_deg = 600;
  double kP = 100, kI = 20, kD = 8;
  double maximum_integral_mV = 2000;  // 积分限幅，饱和时停止同向累积；无重力补偿。
  int maximum_up_mV = 12000, maximum_down_mV = 2500;
  double tolerance_deg = 2, stopped_deg_s = 8, settle_ms = 200;
  double maximum_gap_ms = 100;
  double stall_ms = 600, stall_progress_deg = 1;
  int stall_voltage_mV = 1500;
  // 上升起步仅助推一次；任一侧上移 1 度或满 100ms 后回到原闭环上限。
  int start_up_mV = 12000;
  double start_up_ms = 100, start_progress_deg = 1;
  height_calibration height;
};
struct claw_config {
  int voltage_mV = 6000;
  double maximum_duration_ms = 5000, maximum_gap_ms = 100;
};
class lift_controller {
 public:
  void configure(const lift_config& value) { config = value; cancel(); }
  const lift_info& info() const { return value; }
  bool at_target() const { return value.result == state::done; }
  bool busy() const { return value.result == state::running; }
  void notice(const char* reason) { value.command_status = reason; }

  void observe(const feedback& frame, bool permitted, double time_ms) {
    data = frame; allowed = permitted;
    const double gap = have_frame ? time_ms - now : 0;
    value.control_dt_ms = gap;
    clock_valid = std::isfinite(time_ms) && time_ms >= 0 &&
        (!have_frame || (gap >= 0 && gap <= config.maximum_gap_ms));
    now = time_ms; have_frame = true;
    if (!clock_valid || !valid_feedback()) {
      if (value.homed || busy()) fail(!clock_valid ? "CLOCK/STALE" : !data.arm_online ? "MOTOR OFF" : "FEEDBACK");
      value.homed = false;
      return;
    }
    value.left_deg = data.left_deg - zero_left;
    value.right_deg = data.right_deg - zero_right;
    if (!std::isfinite(value.left_deg) || !std::isfinite(value.right_deg)) {
      fail("FEEDBACK"); value.homed = false; return;
    }
    if (!allowed) { cancel(); return; }
    if (busy()) update();
  }

  // startup=true 仅由线程在上电首次有效反馈时调用；当前位置作为相对零点，不找机械限位。
  bool zero_here(bool startup = false) {
    if (busy()) return false;
    if (!have_frame || !clock_valid || !valid_feedback() || !valid_config() ||
        (!startup && (!allowed || !stopped()))) return false;
    zero_left = data.left_deg; zero_right = data.right_deg;
    value = lift_info(); value.homed = true; value.status = "ZEROED"; pid.reset();
    return true;
  }

  // 绝对目标相对于本次 zero_here 的参考；忙时拒绝覆盖，显式 cancel 后才能换目标。
  bool LiftTo(double target_deg, double timeout_ms) {
    if (busy()) return false;
    value.height_requested = value.height_mapped = false;
    if (!valid_config()) return fail("CONFIG");
    if (!std::isfinite(timeout_ms) || timeout_ms <= 0) return fail("TIMEOUT CONFIG");
    if (!ready()) return fail("DISABLED/FEEDBACK");
    if (!value.homed) return fail("NO ZERO");
    if (!std::isfinite(target_deg) || target_deg < config.minimum_deg || target_deg > config.maximum_deg)
      return fail("TARGET LIMIT");
    value.target_deg = target_deg; value.result = state::running; value.status = "MOVING";
    started = pid_time = now; pid.reset(); timeout = timeout_ms; settle_started = -1; progress_time = now;
    progress_left = value.left_deg; progress_right = value.right_deg;
    start_left = value.left_deg; start_right = value.right_deg;
    startup_active = config.start_up_mV > 0 && config.start_up_ms > 0 && stopped() &&
        target_deg - value.position_deg() > config.tolerance_deg;
    update();
    return value.result != state::error;
  }
  // 毫米入口：仅在实测三点区间内插值，禁止用猜测比例或外推驱动机构。
  bool LiftHeightTo(double height_mm, double timeout_ms) {
    if (busy()) return false;
    value.height_requested = true; value.height_mapped = false; value.requested_height_mm = height_mm;
    if (!std::isfinite(timeout_ms) || timeout_ms <= 0) return fail("TIMEOUT CONFIG");
    if (config.height.point_count == 0) return fail("HEIGHT NOT CAL");
    if (config.height.point_count != 3 || !valid_config()) return fail("HEIGHT CONFIG");
    const height_point* p = config.height.points;
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(p[i].height_mm) || !std::isfinite(p[i].motor_deg) ||
          p[i].height_mm < 0 || p[i].motor_deg < config.minimum_deg || p[i].motor_deg > config.maximum_deg ||
          (i > 0 && (p[i].height_mm <= p[i-1].height_mm || p[i].motor_deg <= p[i-1].motor_deg)))
        return fail("HEIGHT CONFIG");
    }
    if (!std::isfinite(height_mm) || height_mm < p[0].height_mm || height_mm > p[2].height_mm)
      return fail("HEIGHT LIMIT");
    const int lo = height_mm <= p[1].height_mm ? 0 : 1;
    const double ratio = (height_mm - p[lo].height_mm) / (p[lo+1].height_mm - p[lo].height_mm);
    const double degrees = p[lo].motor_deg + ratio * (p[lo+1].motor_deg - p[lo].motor_deg);
    const bool accepted = LiftTo(degrees, timeout_ms);
    value.height_requested = value.height_mapped = true; value.requested_height_mm = height_mm;
    value.target_deg = degrees;
    return accepted;
  }
  void cancel() {
    startup_active = false; pid.reset();
    if (busy()) { value.result = state::cancelled; value.status = "CANCEL"; }
    value.left_mV = value.right_mV = 0;
  }
  void fault(const char* reason) { fail(reason); }

 private:
  bool valid_config() const {
    const double numbers[] = {config.minimum_deg, config.maximum_deg, config.kP, config.kI, config.kD, config.maximum_integral_mV,
      config.tolerance_deg, config.stopped_deg_s, config.settle_ms,
      config.maximum_gap_ms, config.stall_ms, config.stall_progress_deg,
      config.start_up_ms, config.start_progress_deg};
    for (double n : numbers) if (!std::isfinite(n)) return false;
    return config.minimum_deg <= 0 && config.maximum_deg > 0 && config.kP > 0 && config.kI >= 0 && config.kD >= 0 &&
      config.maximum_integral_mV >= 0 && config.maximum_integral_mV <= 12000 && config.maximum_up_mV > 0 && config.maximum_up_mV <= 12000 &&
      config.maximum_down_mV > 0 && config.maximum_down_mV <= 12000 && config.tolerance_deg > 0 &&
      config.stopped_deg_s > 0 && config.settle_ms >= 0 &&
      config.maximum_gap_ms > 0 && config.stall_ms > 0 &&
      config.stall_progress_deg > 0 && config.stall_voltage_mV > 0 && config.stall_voltage_mV <= 12000 &&
      config.start_up_mV >= 0 && config.start_up_mV <= 12000 &&
      config.start_up_ms >= 0 && config.start_up_ms <= 500 && config.start_progress_deg > 0;
  }
  bool valid_feedback() const {
    return data.arm_online && data.arm_valid && std::isfinite(data.left_deg) && std::isfinite(data.right_deg) &&
      std::isfinite(data.left_deg_s) && std::isfinite(data.right_deg_s) &&
      std::fabs(data.left_deg_s) <= 720 && std::fabs(data.right_deg_s) <= 720;
  }
  bool ready() const { return have_frame && clock_valid && allowed && std::isfinite(now) && valid_feedback(); }
  bool stopped() const {
    return std::fabs(data.left_deg_s) <= config.stopped_deg_s && std::fabs(data.right_deg_s) <= config.stopped_deg_s;
  }
  int voltage(double raw, int up_limit_mV) const {
    raw = std::max(-static_cast<double>(config.maximum_down_mV),
                   std::min(static_cast<double>(up_limit_mV), raw));
    // 共轴两台必须始终同电压；任一编码器触及方向边界，两台一起停止。
    if ((raw < 0 && (value.left_deg <= config.minimum_deg || value.right_deg <= config.minimum_deg)) ||
        (raw > 0 && (value.left_deg >= config.maximum_deg || value.right_deg >= config.maximum_deg))) return 0;
    return static_cast<int>(raw);
  }
  void update() {
    const double dt = now - pid_time; pid_time = now;
    value.elapsed_ms = now - started;
    if (value.elapsed_ms >= timeout) { fail("TIMEOUT"); return; }
    if (value.left_deg < config.minimum_deg - config.tolerance_deg || value.right_deg < config.minimum_deg - config.tolerance_deg ||
        value.left_deg > config.maximum_deg + config.tolerance_deg || value.right_deg > config.maximum_deg + config.tolerance_deg) {
      fail("POSITION LIMIT"); return;
    }
    const double error = value.target_deg - value.position_deg();
    if (startup_active && (value.elapsed_ms >= config.start_up_ms ||
        value.left_deg - start_left >= config.start_progress_deg || value.right_deg - start_right >= config.start_progress_deg ||
        error <= config.tolerance_deg)) startup_active = false;
    if (std::fabs(error) <= config.tolerance_deg && stopped()) {
      if (settle_started < 0) settle_started = now;
      value.left_mV = value.right_mV = 0; value.status = "SETTLING"; pid.reset();
      if (now - settle_started >= config.settle_ms) { value.result = state::done; value.status = "DONE"; }
    } else {
      settle_started = -1; value.status = "MOVING";
      const double rate = 0.5 * data.left_deg_s + 0.5 * data.right_deg_s;
      const bool integrate = error > 0 ? value.left_deg < config.maximum_deg && value.right_deg < config.maximum_deg :
          value.left_deg > config.minimum_deg && value.right_deg > config.minimum_deg;
      const double raw = startup_active ? config.start_up_mV :
          pid.update(error, rate, dt, config, integrate);
      if (!std::isfinite(raw)) { fail("CONTROL"); return; }
      const int up_limit = startup_active ? config.start_up_mV : config.maximum_up_mV;
      if (startup_active) value.status = "STARTUP";
      value.left_mV = value.right_mV = voltage(raw, up_limit);
    }
    if (now - progress_time >= config.stall_ms) {
      if ((std::abs(value.left_mV) >= config.stall_voltage_mV && std::fabs(value.left_deg - progress_left) < config.stall_progress_deg) ||
          (std::abs(value.right_mV) >= config.stall_voltage_mV && std::fabs(value.right_deg - progress_right) < config.stall_progress_deg)) {
        fail("STALL"); return;
      }
      progress_time = now; progress_left = value.left_deg; progress_right = value.right_deg;
    }
  }
  bool fail(const char* reason) {
    startup_active = false; pid.reset();
    value.result = state::error; value.status = reason; value.left_mV = value.right_mV = 0; return false;
  }
  lift_config config;
  lift_info value;
  feedback data;
  lift_pid pid;
  bool have_frame = false, allowed = false, clock_valid = false, startup_active = false;
  double now = 0, zero_left = 0, zero_right = 0, started = 0, timeout = 0, settle_started = -1;
  double progress_time = 0, progress_left = 0, progress_right = 0;
  double start_left = 0, start_right = 0, pid_time = 0;
};

// 夹爪与抬升调试放在同一文件；此处只计算输出，由主循环统一写电机。
class claw_controller {
 public:
  void configure(const claw_config& next) { config = next; cancel(); }
  const claw_info& info() const { return value; }
  bool busy() const { return value.result == state::running; }
  void reject(const char* reason) { if (!busy()) fail(reason); }

  void observe(bool motor_online, bool permitted, double time_ms) {
    online = motor_online; allowed = permitted;
    const double gap = have_frame ? time_ms - now : 0;
    clock_valid = std::isfinite(time_ms) && time_ms >= 0 &&
        (!have_frame || (gap >= 0 && gap <= config.maximum_gap_ms));
    now = time_ms; have_frame = true;
    if (busy() && (!clock_valid || !online)) {
      fail(!clock_valid ? "CLOCK/STALE" : "MOTOR OFF"); return;
    }
    if (!allowed) { cancel(); return; }
    if (busy()) update();
  }

  // duration 与 timeout 均为毫秒；timeout 必须显式指定，运行中拒绝覆盖。
  bool ClawOpen(double duration_ms, double timeout_ms) {
    return start(duration_ms, timeout_ms, -1, "OPENING");
  }
  bool ClawClose(double duration_ms, double timeout_ms) {
    return start(duration_ms, timeout_ms, 1, "CLOSING");
  }
  void cancel() {
    if (busy()) { value.result = state::cancelled; value.status = "CANCEL"; }
    value.millivolts = 0;
  }

 private:
  bool valid_config() const {
    return config.voltage_mV > 0 && config.voltage_mV <= 12000 &&
        std::isfinite(config.maximum_duration_ms) && config.maximum_duration_ms > 0 &&
        std::isfinite(config.maximum_gap_ms) && config.maximum_gap_ms > 0;
  }
  bool start(double duration_ms, double timeout_ms, int sign, const char* status) {
    if (busy()) return false;
    if (!valid_config()) return fail("CONFIG");
    if (!std::isfinite(duration_ms) || duration_ms < 0 || duration_ms > config.maximum_duration_ms)
      return fail("DURATION CONFIG");
    if (!std::isfinite(timeout_ms) || timeout_ms <= 0) return fail("TIMEOUT CONFIG");
    if (!have_frame || !clock_valid || !allowed || !online) return fail("DISABLED/FEEDBACK");
    value = claw_info(); value.result = state::running; value.status = status;
    value.duration_ms = duration_ms; started = now; timeout = timeout_ms; direction = sign;
    update();
    return value.result != state::error;
  }
  void update() {
    value.elapsed_ms = now - started;
    // 同一周期既到 duration 又到 timeout 时，先报超时；DONE 仅表示定时结束，不能证明到位。
    if (value.elapsed_ms >= timeout) { fail("TIMEOUT"); return; }
    if (value.elapsed_ms >= value.duration_ms) {
      value.result = state::done; value.status = "DONE"; value.millivolts = 0; return;
    }
    value.millivolts = direction * config.voltage_mV;
  }
  bool fail(const char* reason) {
    value.result = state::error; value.status = reason; value.millivolts = 0; return false;
  }
  claw_config config;
  claw_info value;
  bool have_frame = false, online = false, allowed = false, clock_valid = false;
  double now = 0, started = 0, timeout = 0;
  int direction = 0;
};

}  // namespace mechanisms
