#pragma once

#include "drive-straight.h"
#include "trapezoid-motion.h"

namespace drive {
enum class pivot_side { none, left, right };  // 固定并制动的一侧。

enum class turn_state { idle, running, done, cancelled, error };

struct turn_config {
  double kP = 0.8;  // 角度外环：rpm/度。
  double kD = 0.2;  // 角速度阻尼：rpm/(度/秒)。
  double acceleration_rpm_s = 120;  // 单侧目标轮速变化上限，非角度梯形规划。
  bool profile_enabled = false;  // 关闭时保留原角度PD，便于与低速基线对照。
  double wheel_diameter_mm = 69.85, motor_revs_per_wheel_rev = 1.2, track_width_mm = 325;
  double angular_acceleration_deg_s2 = 900;  // 规划角加速度，度/秒²。
  double angular_deceleration_deg_s2 = 1200; // 规划角减速度，度/秒²，独立整定。
  double profile_braking_delay_ms = 120;  // 制动预留，覆盖轮速响应和角速度滤波延迟。
  double profile_finish_kP = 0;  // 参考结束后的角度纠偏增益；0沿用kP，不改变规划段跟踪。
  double angle_tolerance_deg = 0.8;
  double rate_tolerance_deg_s = 2.5;
  double wheel_tolerance_rpm = 5;
  double rate_filter_ms = 60;
  double settle_ms = 250;
  double timeout_ms = 12000;
  double max_update_ms = 200;
  double minimum_settle_rpm = 0;  // 实车转向可设置低速回调下限，避免停在静摩擦区。
  double minimum_finish_rpm = 0;  // 非零时，末端回调下限渐变到此值；0 沿用上项。
  double finish_zone_deg = 6;
  double minimum_rate_deg_s = 8;  // 低角速度时全量补偿，其上平滑退去，保留高速制动。
  double correction_fraction = 0.5;  // 转向可单独放宽；直行仍使用原值。
  velocity_pid_config pid;
  feedforward_config left_forward, right_forward, left_reverse, right_reverse;
};

struct turn_info {
  turn_state state = turn_state::idle;
  const char* status = "READY";
  double time_ms = 0, elapsed_ms = 0;
  double angle_deg = 0, target_deg = 0, error_deg = 0, rate_deg_s = 0;
  double target_rpm = 0;  // 逆时针为正，左侧取负、右侧取正。
  pivot_side pivot = pivot_side::none;
  bool profiled = false;
  double reference_deg = 0, reference_rate_deg_s = 0, profile_duration_ms = 0, profile_peak_deg_s = 0;
  int profile_phase = 0;  // 0未启用、1加速、2匀速、3减速、4末端纠偏。
  int left_mV = 0, right_mV = 0;
  velocity_loop_info velocity;
};

// 连续惯导角逆时针为正。只计算输出，全部接口由主线程调用。
class turn_controller {
 public:
  void Configure(const turn_config& value) {
    config = value;
    Velocity.ConfigureForward(value.left_forward, value.right_forward);
    Velocity.ConfigureReverse(value.left_reverse, value.right_reverse);
    Velocity.ConfigurePID(value.pid);
    info = turn_info(); configured = true; have_frame = false;
  }
  void Observe(const straight_feedback& value, bool permitted, double now_ms) {
    frame = value; allowed = permitted; now = now_ms; have_frame = true;
    Velocity.Observe(value.velocity, now_ms);
  }
  const turn_info& Info() const { return info; }
  bool StartTurnRelative(double angle_deg, double maximum_rpm) {
    if (info.state == turn_state::running) return false;
    return Start(frame.heading_deg + angle_deg, maximum_rpm);
  }
  bool StartSwingRelative(double angle_deg, double maximum_rpm, pivot_side fixed) {
    if (info.state == turn_state::running) return false;
    if (fixed != pivot_side::left && fixed != pivot_side::right) return Fail("SWING CONFIG");
    return Start(frame.heading_deg + angle_deg, maximum_rpm, fixed);
  }
  // 绝对朝向按最短方向转，跨越 0/360 正常处理；恰好 180 度选逆时针。
  bool StartTurnToHeading(double heading_deg, double maximum_rpm) {
    if (info.state == turn_state::running) return false;
    if (!std::isfinite(heading_deg) || !std::isfinite(frame.heading_deg)) return Fail("CONFIG");
    double delta = std::fmod(std::fmod(heading_deg, 360.0) -
                             std::fmod(frame.heading_deg, 360.0), 360.0);
    if (delta <= -180) delta += 360;
    if (delta > 180) delta -= 360;
    return Start(frame.heading_deg + delta, maximum_rpm);
  }
  void UpdateTurn() {
    if (info.state != turn_state::running) return;
    if (!CheckFeedback()) return;
    const double dt_ms = now - previous_time;
    if (dt_ms < 0 || dt_ms > config.max_update_ms) { Fail("CLOCK ERR"); return; }
    if (dt_ms == 0) return;
    info.time_ms = now; info.elapsed_ms = now - started_at;
    info.angle_deg = frame.heading_deg; info.error_deg = info.target_deg - info.angle_deg;
    if (info.profiled) {
      const double seconds = info.elapsed_ms / 1000;
      const auto reference = Profile.sample(seconds);
      info.reference_deg = start_angle + reference.position;
      info.reference_rate_deg_s = reference.velocity;
      info.profile_phase = seconds >= Profile.duration() ? 4 :
          seconds < Profile.accelerationTime() ? 1 :
          seconds < Profile.accelerationTime() + Profile.cruiseTime() ? 2 : 3;
    }
    const double measured_rate = (frame.heading_deg - previous_angle) * 1000 / dt_ms;
    if (!std::isfinite(measured_rate)) { Fail("IMU ERROR"); return; }
    info.rate_deg_s += dt_ms / (config.rate_filter_ms + dt_ms) * (measured_rate - info.rate_deg_s);
    previous_time = now; previous_angle = frame.heading_deg;
    if (info.elapsed_ms >= config.timeout_ms) { Fail("TIMEOUT"); return; }
    const bool angle_close = std::fabs(info.error_deg) <= config.angle_tolerance_deg;
    if (angle_close && (!info.profiled || info.profile_phase == 4)) {
      StopOutput();
      if (std::fabs(info.rate_deg_s) <= config.rate_tolerance_deg_s &&
          std::fabs(frame.velocity.left_rpm) <= config.wheel_tolerance_rpm &&
          std::fabs(frame.velocity.right_rpm) <= config.wheel_tolerance_rpm) {
        if (settled_since < 0) settled_since = now;
        if (now - settled_since >= config.settle_ms) {
          info.state = turn_state::done; info.status = "DONE"; return;
        }
      } else settled_since = -1;
      info.status = "SETTLE"; return;
    }
    settled_since = -1;
    double requested = Limit(config.kP * info.error_deg - config.kD * info.rate_deg_s, max_rpm);
    if (info.profiled) {
      // 角速度前馈跟随规划，角度/角速度反馈只负责修正实际运动的落后或超前。
      const double position_kP = info.profile_phase == 4 && config.profile_finish_kP > 0 ?
          config.profile_finish_kP : config.kP;
      const double tracking_rpm = info.reference_rate_deg_s / degrees_per_rpm +
          position_kP * (info.reference_deg - info.angle_deg) +
          config.kD * (info.reference_rate_deg_s - info.rate_deg_s);
      if (!std::isfinite(tracking_rpm)) { Fail("PROFILE ERR"); return; }
      requested = Limit(tracking_rpm, max_rpm);
      // 按实测剩余角度限制可行制动速度，防止时间参考超前或实际轮距偏差导致冲过终点。
      const double deceleration = config.angular_deceleration_deg_s2;
      const double delay = deceleration * config.profile_braking_delay_ms / 1000;
      const double remaining = std::fabs(info.error_deg);
      const double braking_term = delay * delay + 2 * deceleration * remaining;
      if (!std::isfinite(braking_term)) { Fail("PROFILE ERR"); return; }
      const double braking_rate = std::sqrt(braking_term) - delay;
      if (requested * info.error_deg > 0)
        requested = Limit(requested, braking_rate / degrees_per_rpm);
    }
    const double finish_rpm = config.minimum_finish_rpm > 0 ? config.minimum_finish_rpm : config.minimum_settle_rpm;
    const double progress = std::max(0.0, std::min(1.0,
        (std::fabs(info.error_deg) - config.angle_tolerance_deg) /
        (config.finish_zone_deg - config.angle_tolerance_deg)));
    const double rate_weight = std::max(0.0, std::min(1.0,
        2 - std::fabs(info.rate_deg_s) / config.minimum_rate_deg_s));
    const double minimum_rpm = (finish_rpm + (config.minimum_settle_rpm - finish_rpm) * progress) * rate_weight;
    if ((!info.profiled || info.profile_phase == 4) &&
        requested * info.error_deg >= 0 && std::fabs(requested) < minimum_rpm)
      requested = std::copysign(std::min(minimum_rpm, max_rpm), info.error_deg);
    if (info.profiled) {
      info.target_rpm = motion::limitVelocity(info.target_rpm, requested,
          config.angular_acceleration_deg_s2 / degrees_per_rpm,
          config.angular_deceleration_deg_s2 / degrees_per_rpm, dt_ms / 1000);
    } else {
      const double step = config.acceleration_rpm_s * dt_ms / 1000;
      info.target_rpm += Limit(requested - info.target_rpm, step);
    }
    const double left_target = info.pivot == pivot_side::left ? 0 : -info.target_rpm;
    const double right_target = info.pivot == pivot_side::right ? 0 : info.target_rpm;
    if (!Velocity.Feedforward(left_target, right_target, info.left_mV, info.right_mV)) {
      Fail("FF ERROR"); return;
    }
    if (!Velocity.ApplyPID(left_target, right_target, frame.velocity, now,
                           info.left_mV, info.right_mV, config.correction_fraction)) { Fail("SPEED ERR"); return; }
    info.velocity = Velocity.Info();
    // 固定侧始终给零输出，由现有hardware::Apply主动brake；PID不能推着固定侧移动。
    if (info.pivot == pivot_side::left) info.left_mV = 0;
    if (info.pivot == pivot_side::right) info.right_mV = 0;
    info.status = "TURN";
  }
  void CancelTurn() {
    if (info.state == turn_state::running) { info.state = turn_state::cancelled; info.status = "CANCEL"; }
    StopOutput();
  }
  void Reset() { CancelTurn(); info = turn_info(); }

 private:
  static double Limit(double value, double cap) { return std::max(-cap, std::min(cap, value)); }
  static bool Positive(double v) { return std::isfinite(v) && v > 0; }
  static bool Nonnegative(double v) { return std::isfinite(v) && v >= 0; }
  bool Start(double target, double maximum_rpm, pivot_side fixed = pivot_side::none) {
    if (!configured || !std::isfinite(target) || !Positive(maximum_rpm) || maximum_rpm > 600 ||
        !Positive(config.kP) || !Nonnegative(config.kD) || !Positive(config.acceleration_rpm_s) ||
        !Positive(config.angle_tolerance_deg) || !Positive(config.rate_tolerance_deg_s) ||
        !Positive(config.wheel_tolerance_rpm) || !Nonnegative(config.rate_filter_ms) ||
        !Nonnegative(config.settle_ms) || !Positive(config.timeout_ms) || !Positive(config.max_update_ms) ||
        !Nonnegative(config.minimum_settle_rpm) || !Positive(config.minimum_rate_deg_s) ||
        !Nonnegative(config.minimum_finish_rpm) || config.minimum_finish_rpm > config.minimum_settle_rpm ||
        !Positive(config.finish_zone_deg) || config.finish_zone_deg <= config.angle_tolerance_deg ||
        !Positive(config.correction_fraction))
      return Fail("CONFIG");
    if (!CheckFeedback()) return false;
    if (config.profile_enabled && (!Positive(config.wheel_diameter_mm) ||
        !Positive(config.motor_revs_per_wheel_rev) || !Positive(config.track_width_mm) ||
        !Positive(config.angular_acceleration_deg_s2) || !Positive(config.angular_deceleration_deg_s2) ||
        !Nonnegative(config.profile_braking_delay_ms) || !Nonnegative(config.profile_finish_kP)))
      return Fail("CONFIG");
    if (!frame.stopped || std::fabs(frame.velocity.left_rpm) > config.wheel_tolerance_rpm ||
        std::fabs(frame.velocity.right_rpm) > config.wheel_tolerance_rpm) return Fail("MOVING");
    Velocity.Reset(); info = turn_info(); info.state = turn_state::running; info.status = "TURN";
    info.angle_deg = previous_angle = frame.heading_deg; info.target_deg = target;
    info.error_deg = target - frame.heading_deg;
    if (!std::isfinite(info.error_deg)) return Fail("CONFIG");
    info.pivot = fixed;
    start_angle = frame.heading_deg;
    info.profiled = config.profile_enabled;
    info.reference_deg = start_angle;
    if (info.profiled) {
      // 原地转向两侧等速反向：角速度(deg/s) = 电机rpm × 6D/(传动比×轮距)。
      degrees_per_rpm = 6 * config.wheel_diameter_mm /
          config.motor_revs_per_wheel_rev / config.track_width_mm;
      if (fixed != pivot_side::none) degrees_per_rpm /= 2; // 单侧运动的角速度为原地转向的一半。
      if (!Positive(degrees_per_rpm)) return Fail("CONFIG");
      Profile = motion::TrapezoidProfile(info.error_deg, maximum_rpm * degrees_per_rpm,
          config.angular_acceleration_deg_s2, config.angular_deceleration_deg_s2);
      if (!Profile.valid() || !std::isfinite(Profile.duration() * 1000)) return Fail("CONFIG");
      info.profile_duration_ms = Profile.duration() * 1000;
      info.profile_peak_deg_s = Profile.peakVelocity();
    }
    started_at = previous_time = info.time_ms = now; settled_since = -1; max_rpm = maximum_rpm;
    return true;
  }
  bool CheckFeedback() {
    if (!have_frame || !allowed) return Fail("DISABLED");
    if (!Nonnegative(now)) return Fail("CLOCK ERR");
    if (!frame.motors_online) return Fail("MOTOR OFF");
    if (!frame.inertial_ready || !std::isfinite(frame.heading_deg)) return Fail("IMU WAIT");
    if (!Velocity.Fresh(now)) return Fail("SPEED ERR");
    return true;
  }
  void StopOutput() {
    info.left_mV = info.right_mV = 0; info.target_rpm = 0;
    Velocity.Reset(); info.velocity = Velocity.Info();
  }
  bool Fail(const char* status) {
    info.state = turn_state::error; info.status = status; StopOutput(); return false;
  }
  turn_config config;
  straight_feedback frame;
  turn_info info;
  velocity_control Velocity;
  motion::TrapezoidProfile Profile{0, 1, 1, 1};
  bool configured = false, have_frame = false, allowed = false;
  double now = 0, previous_time = 0, previous_angle = 0, started_at = 0;
  double settled_since = -1, max_rpm = 0;
  double start_angle = 0, degrees_per_rpm = 1;
};
}  // 底盘转向控制命名空间
