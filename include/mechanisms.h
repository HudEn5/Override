#pragma once

#include <algorithm>
#include <cmath>
#include "mechanism-control.h"

// 移植自同事 b70b202 的抬升/夹爪算法，仅计算输出；电机由 main 统一写入。
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
  double control_dt_ms = 0, requested_height_mm = 0;
  bool height_requested = false, height_mapped = false;
  int left_mV = 0, right_mV = 0;
  double position_deg() const {
    return 0.5 * left_deg + 0.5 * right_deg;
  }
};

class lift_pid {
 public:
  void reset() {
    integral = previous_error = 0;
  }
  template <typename Config>
  double update(double error, double rate, double dt_ms, const Config& c, bool integrate) {
    if (error * previous_error < 0 || c.kI == 0)
      integral = 0;
    if (error != 0)
      previous_error = error;
    const double base = c.kP * error - c.kD * rate;
    if (integrate && dt_ms > 0) {
      const double next = std::max(-c.maximum_integral_mV,
                                   std::min(c.maximum_integral_mV, integral + c.kI * error * dt_ms / 1000.0));
      const double raw = base + next;
      if ((raw >= -c.maximum_down_mV && raw <= c.maximum_up_mV) || (raw > c.maximum_up_mV && error < 0) ||
          (raw < -c.maximum_down_mV && error > 0))
        integral = next;
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
    if (!clock_valid || !valid_feedback()) {
      if (value.homed || busy())
        fail(!clock_valid ? "CLOCK/STALE" : !data.arm_online ? "MOTOR OFF" : "FEEDBACK");
      value.homed = false;
      return;
    }
    value.left_deg = data.left_deg - zero_left;
    value.right_deg = data.right_deg - zero_right;
    if (!std::isfinite(value.left_deg) || !std::isfinite(value.right_deg)) {
      fail("FEEDBACK");
      value.homed = false;
      return;
    }
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
    pid.reset();
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
    if (!ready())
      return fail("DISABLED/FEEDBACK");
    if (!value.homed)
      return fail("NO ZERO");
    if (!std::isfinite(target_deg) || target_deg < config.minimum_deg || target_deg > config.maximum_deg)
      return fail("TARGET LIMIT");
    value.target_deg = target_deg;
    value.result = state::running;
    value.status = "MOVING";
    started = pid_time = now;
    pid.reset();
    timeout = timeout_ms;
    settle_started = -1;
    progress_time = now;
    progress_left = value.left_deg;
    progress_right = value.right_deg;
    start_left = value.left_deg;
    start_right = value.right_deg;
    startup_active = config.start_up_mV > 0 && config.start_up_ms > 0 && stopped() &&
                     target_deg - value.position_deg() > config.tolerance_deg;
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
    startup_active = false;
    pid.reset();
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
                              config.kP,
                              config.kI,
                              config.kD,
                              config.maximum_integral_mV,
                              config.tolerance_deg,
                              config.stopped_deg_s,
                              config.settle_ms,
                              config.maximum_gap_ms,
                              config.stall_ms,
                              config.stall_progress_deg,
                              config.start_up_ms,
                              config.start_progress_deg};
    for (double n : numbers)
      if (!std::isfinite(n))
        return false;
    return config.minimum_deg <= 0 && config.maximum_deg > 0 && config.kP > 0 && config.kI >= 0 &&
           config.kD >= 0 && config.maximum_integral_mV >= 0 && config.maximum_integral_mV <= 12000 &&
           config.maximum_up_mV > 0 && config.maximum_up_mV <= 12000 && config.maximum_down_mV > 0 &&
           config.maximum_down_mV <= 12000 && config.tolerance_deg > 0 && config.stopped_deg_s > 0 &&
           config.settle_ms >= 0 && config.maximum_gap_ms > 0 && config.stall_ms > 0 &&
           config.stall_progress_deg > 0 && config.stall_voltage_mV > 0 && config.stall_voltage_mV <= 12000 &&
           config.start_up_mV >= 0 && config.start_up_mV <= 12000 && config.start_up_ms >= 0 &&
           config.start_up_ms <= 500 && config.start_progress_deg > 0;
  }
  bool valid_feedback() const {
    return data.arm_online && data.arm_valid && std::isfinite(data.left_deg) &&
           std::isfinite(data.right_deg) && std::isfinite(data.left_deg_s) &&
           std::isfinite(data.right_deg_s) && std::fabs(data.left_deg_s) <= 720 &&
           std::fabs(data.right_deg_s) <= 720;
  }
  bool ready() const {
    return have_frame && clock_valid && allowed && std::isfinite(now) && valid_feedback();
  }
  bool stopped() const {
    return std::fabs(data.left_deg_s) <= config.stopped_deg_s &&
           std::fabs(data.right_deg_s) <= config.stopped_deg_s;
  }
  int voltage(double raw, int up_limit_mV) const {
    raw = std::max(-static_cast<double>(config.maximum_down_mV),
                   std::min(static_cast<double>(up_limit_mV), raw));
    // 共轴两台必须始终同电压；任一编码器触及方向边界，两台一起停止。
    if ((raw < 0 && (value.left_deg <= config.minimum_deg || value.right_deg <= config.minimum_deg)) ||
        (raw > 0 && (value.left_deg >= config.maximum_deg || value.right_deg >= config.maximum_deg)))
      return 0;
    return static_cast<int>(raw);
  }
  void update() {
    const double dt = now - pid_time;
    pid_time = now;
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
    if (startup_active &&
        (value.elapsed_ms >= config.start_up_ms || value.left_deg - start_left >= config.start_progress_deg ||
         value.right_deg - start_right >= config.start_progress_deg || error <= config.tolerance_deg))
      startup_active = false;
    if (std::fabs(error) <= config.tolerance_deg && stopped()) {
      if (settle_started < 0)
        settle_started = now;
      value.left_mV = value.right_mV = 0;
      value.status = "SETTLING";
      pid.reset();
      if (now - settle_started >= config.settle_ms) {
        value.result = state::done;
        value.status = "DONE";
      }
    } else {
      settle_started = -1;
      value.status = "MOVING";
      const double rate = 0.5 * data.left_deg_s + 0.5 * data.right_deg_s;
      const bool integrate =
          error > 0 ? value.left_deg < config.maximum_deg && value.right_deg < config.maximum_deg
                    : value.left_deg > config.minimum_deg && value.right_deg > config.minimum_deg;
      const double raw = startup_active ? config.start_up_mV : pid.update(error, rate, dt, config, integrate);
      if (!std::isfinite(raw)) {
        fail("CONTROL");
        return;
      }
      const int up_limit = startup_active ? config.start_up_mV : config.maximum_up_mV;
      if (startup_active)
        value.status = "STARTUP";
      value.left_mV = value.right_mV = voltage(raw, up_limit);
    }
    if (now - progress_time >= config.stall_ms) {
      if ((std::abs(value.left_mV) >= config.stall_voltage_mV &&
           std::fabs(value.left_deg - progress_left) < config.stall_progress_deg) ||
          (std::abs(value.right_mV) >= config.stall_voltage_mV &&
           std::fabs(value.right_deg - progress_right) < config.stall_progress_deg)) {
        fail("STALL");
        return;
      }
      progress_time = now;
      progress_left = value.left_deg;
      progress_right = value.right_deg;
    }
    // 截止时刻已满足到位/停稳可DONE，仍在运行则与调度器一致立即撤压。
    if (busy() && value.elapsed_ms >= timeout)
      fail("TIMEOUT");
  }
  bool fail(const char* reason) {
    startup_active = false;
    pid.reset();
    value.result = state::error;
    value.status = reason;
    value.left_mV = value.right_mV = 0;
    return false;
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
    if (busy() && (!clock_valid || !online)) {
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
    return config.voltage_mV > 0 && config.voltage_mV <= 12000 && std::isfinite(config.maximum_duration_ms) &&
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
      value.status = "DONE";
      value.millivolts = 0;
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
        action.volts > 12) {
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
