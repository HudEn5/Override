#pragma once

#include "drive-straight.h"
#include "corner-profile.h"

namespace drive {
enum class corner_state { idle, running, done, cancelled, error };
struct corner_config {
  double wheel_diameter_mm = 0, motor_revs_per_wheel_rev = 1;
  double track_width_mm = 0;  // 左右驱动轮接地中心线间距，需实测。
  double radius_mm = 250;  // 参考曲线的最小曲率半径，不是固定圆半径。
  double corner_rpm = 50;
  double corner_floor_rpm = 0;  // 0 关闭弯速调节；非零时也是入弯中心速度。
  double corner_acceleration_rpm_s = 0;  // 0 沿用普通加速，非零时限制弯内提速。
  double heading_slow_start_deg = 4, heading_slow_full_deg = 10;
  double governor_lookahead_ms = 200, entry_braking_delay_ms = 0;
  bool prediction_enabled = false;
  double prediction_wheel_acceleration_rpm_s = 180, prediction_wheel_deceleration_rpm_s = 120;
  double prediction_margin = 0.85;
  double acceleration_rpm_s = 120, deceleration_rpm_s = 120;
  double heading_kP = 1.6, heading_kD = 0.2;
  double exit_heading_kP = 6.0, exit_heading_kD = 0.8;
  double exit_heading_kI = 2.0, exit_integral_limit_rpm = 12;
  double exit_acceleration_rpm_s = 60;
  double exit_angle_tolerance_deg = 0.8, exit_rate_tolerance_deg_s = 2.5;
  double exit_settle_ms = 100;  // 出口保持弯速前进，方向稳定后才恢复直线加速。
  double rate_filter_ms = 60, max_heading_error_deg = 15;
  double max_yaw_fraction = 0.8;  // 差速不超过中心速度的此比例，两侧保持同向。
  double position_tolerance_mm = 5, angle_tolerance_deg = 0.8;
  double wheel_tolerance_rpm = 5, rate_tolerance_deg_s = 2.5, settle_ms = 250;
  double finish_kP = 0.25, finish_min_rpm = 12, finish_max_rpm = 20;
  double braking_delay_ms = 200;  // 终点减速预留电机跟随滞后，避免带着较高轮速冲入窗口。
  double timeout_ms = 20000, max_update_ms = 200;
  double correction_fraction = 3;
  velocity_pid_config pid;
  feedforward_config left_forward, right_forward, left_reverse, right_reverse;
};
struct corner_info {
  corner_state state = corner_state::idle;
  const char* status = "READY";
  int phase = 0;  // 0 空闲，1 进入直线，2 圆角，3 离开直线，4 终点停稳。
  double time_ms = 0, elapsed_ms = 0, distance_mm = 0, remaining_mm = 0;
  double reference_deg = 0, angle_deg = 0, error_deg = 0, rate_deg_s = 0;
  double center_rpm = 0, left_target_rpm = 0, right_target_rpm = 0;
  double planned_rpm = 0;  // 当前空间速度计划含响应距离预览后的上限，不是实际轮速。
  int left_mV = 0, right_mV = 0;
  velocity_loop_info velocity;
};

// 一条连续路线由一个执行器拥有：中间不清零积分、不等待停稳、不标记提前 DONE。
class corner_controller {
 public:
  void Configure(const corner_config& value) {
    config = value; info = corner_info(); have_frame = false; configured = true;
    Velocity.ConfigureForward(value.left_forward, value.right_forward);
    Velocity.ConfigureReverse(value.left_reverse, value.right_reverse);
    Velocity.ConfigurePID(value.pid);
  }
  void Observe(const straight_feedback& value, bool permitted, double now_ms) {
    frame = value; allowed = permitted; now = now_ms; have_frame = true;
    Velocity.Observe(frame.velocity, now);
  }
  const corner_info& Info() const { return info; }
  // 距离均为直线切点前/后的长度，正角逆时针；最高 rpm 约束单侧目标。
  bool StartDriveCorner(double before_mm, double angle_deg, double after_mm, double maximum_rpm) {
    if (info.state == corner_state::running) return false;
    if (!configured || !Nonnegative(before_mm) || !Nonnegative(after_mm) ||
        !Positive(std::fabs(angle_deg)) || std::fabs(angle_deg) > 180 ||
        !Positive(maximum_rpm) || maximum_rpm > 600 || !ValidConfig()) return Fail("CORNER CONFIG");
    if (!CheckFeedback()) return false;
    if (!frame.stopped || std::fabs(frame.velocity.left_rpm) > config.wheel_tolerance_rpm ||
        std::fabs(frame.velocity.right_rpm) > config.wheel_tolerance_rpm) return Fail("MOVING");
    mm_s_per_rpm = PI * config.wheel_diameter_mm / (60 * config.motor_revs_per_wheel_rev);
    before = before_mm; after = after_mm; angle = angle_deg; max_rpm = maximum_rpm;
    // 三次平滑步进的最大导数是 1.5，保证参考曲率峰值为 1/radius。
    corner_length = 1.5 * std::fabs(angle_deg) * PI / 180 * config.radius_mm;
    total = before + corner_length + after;
    if (!Positive(total) || !Positive(mm_s_per_rpm)) return Fail("CORNER CONFIG");
    if (config.prediction_enabled) {
      corner_profile_config p;
      p.before_mm = before; p.after_mm = after; p.angle_deg = angle; p.radius_mm = config.radius_mm;
      p.track_width_mm = config.track_width_mm; p.mm_s_per_rpm = mm_s_per_rpm; p.maximum_rpm = max_rpm;
      p.corner_rpm = std::min(config.corner_rpm, max_rpm / (1 + config.max_yaw_fraction));
      p.entry_rpm = config.corner_floor_rpm > 0 ? std::min(config.corner_floor_rpm, p.corner_rpm) : p.corner_rpm;
      p.finish_rpm = config.finish_min_rpm;
      p.acceleration_rpm_s = config.acceleration_rpm_s; p.deceleration_rpm_s = config.deceleration_rpm_s;
      p.corner_acceleration_rpm_s = config.corner_acceleration_rpm_s > 0 ?
          config.corner_acceleration_rpm_s : config.acceleration_rpm_s;
      p.exit_acceleration_rpm_s = config.exit_acceleration_rpm_s;
      // 在线每侧目标变化也受 acceleration_rpm_s 约束，预测不能超过这一层。
      p.wheel_acceleration_rpm_s = std::min(config.prediction_wheel_acceleration_rpm_s, config.acceleration_rpm_s);
      p.wheel_deceleration_rpm_s = std::min(config.prediction_wheel_deceleration_rpm_s, config.acceleration_rpm_s);
      p.margin = config.prediction_margin;
      if (!Profile.Build(p)) return Fail("PLAN CONFIG");
    }
    origin = Position();
    if (!std::isfinite(origin)) return Fail("POSITION ERR");
    initial_heading = previous_angle = frame.heading_deg;
    previous_position = origin; previous_time = started_at = now; settled_since = -1;
    center = 0; exit_ready = false; exit_settled_since = -1; exit_integral = 0;
    have_exit_speed = false; exit_speed = 0;
    Velocity.Reset(); info = corner_info();
    info.state = corner_state::running; info.status = "IN";
    info.phase = 1; info.remaining_mm = total; info.angle_deg = info.reference_deg = initial_heading;
    info.time_ms = now;
    return true;
  }
  void UpdateCorner() {
    if (info.state != corner_state::running || !CheckFeedback()) return;
    const double dt_ms = now - previous_time;
    if (dt_ms < 0 || dt_ms > config.max_update_ms) { Fail("CLOCK ERR"); return; }
    if (dt_ms == 0) return;
    const double dt = dt_ms / 1000;
    const double position = Position();
    if (std::fabs(position - previous_position) > 720 * mm_s_per_rpm * dt + 1) { Fail("POSITION JUMP"); return; }
    info.time_ms = now; info.elapsed_ms = now - started_at;
    info.distance_mm = position - origin; info.remaining_mm = total - info.distance_mm;
    info.rate_deg_s += dt_ms / (config.rate_filter_ms + dt_ms) *
        ((frame.heading_deg - previous_angle) / dt - info.rate_deg_s);
    previous_time = now; previous_position = position; previous_angle = frame.heading_deg;
    info.angle_deg = frame.heading_deg;
    if (info.elapsed_ms >= config.timeout_ms) { Fail("TIMEOUT"); return; }
    const double q = Clamp01((info.distance_mm - before) / corner_length);
    info.reference_deg = initial_heading + angle * q * q * (3 - 2 * q);
    info.error_deg = info.reference_deg - info.angle_deg;
    if (!std::isfinite(info.error_deg) || !std::isfinite(info.rate_deg_s)) { Fail("IMU ERROR"); return; }
    if (std::fabs(info.error_deg) > config.max_heading_error_deg) { Fail("HEADING ERR"); return; }
    if (info.distance_mm < before) { info.phase = 1; info.status = "IN"; }
    else if (info.distance_mm < before + corner_length) { info.phase = 2; info.status = "CORNER"; }
    else {
      info.phase = 3; info.status = "OUT";
      if (!have_exit_speed) {
        exit_speed = std::fabs(center); have_exit_speed = true;
      }
      if (!exit_ready) {
        if (std::fabs(info.error_deg) <= config.exit_angle_tolerance_deg &&
            std::fabs(info.rate_deg_s) <= config.exit_rate_tolerance_deg_s) {
          if (exit_settled_since < 0) exit_settled_since = now;
          if (now - exit_settled_since >= config.exit_settle_ms) exit_ready = true;
        } else exit_settled_since = -1;
        if (!exit_ready) info.status = "OUT ALIGN";
      }
    }
    if (std::fabs(info.remaining_mm) <= config.position_tolerance_mm) {
      info.phase = 4; info.status = "SETTLE"; center = 0; StopOutput();
      info.reference_deg = initial_heading + angle;
      info.error_deg = info.reference_deg - info.angle_deg;
      // 仅终点允许停车；若此处方向误差仍过大，不伪报路线完成。
      if (std::fabs(info.error_deg) > config.angle_tolerance_deg) { Fail("END HEADING"); return; }
      if (std::fabs(frame.velocity.left_rpm) <= config.wheel_tolerance_rpm &&
          std::fabs(frame.velocity.right_rpm) <= config.wheel_tolerance_rpm &&
          std::fabs(info.rate_deg_s) <= config.rate_tolerance_deg_s) {
        if (settled_since < 0) settled_since = now;
        if (now - settled_since >= config.settle_ms) {
          info.state = corner_state::done; info.status = "DONE";
        }
      } else settled_since = -1;
      return;
    }
    settled_since = -1;
    const double remaining = std::fabs(info.remaining_mm);
    const double actual_center = (frame.velocity.left_rpm + frame.velocity.right_rpm) / 2;
    const double curvature = angle * PI / 180 / corner_length * 6 * q * (1 - q);
    const double reference_rate = actual_center * mm_s_per_rpm * curvature * 180 / PI;
    const double budget = std::max(0.0, remaining - std::fabs(actual_center) * mm_s_per_rpm * config.braking_delay_ms / 1000);
    double request = std::min(max_rpm, std::max(config.finish_min_rpm,
        std::sqrt(2 * config.deceleration_rpm_s * budget / mm_s_per_rpm)));
    const double corner_speed = std::min(config.corner_rpm, max_rpm / (1 + config.max_yaw_fraction));
    const double floor_speed = config.corner_floor_rpm > 0 ? std::min(config.corner_floor_rpm, corner_speed) : corner_speed;
    if (info.distance_mm < before) {
      const double entry_remaining = std::max(0.0, before - info.distance_mm -
          std::fabs(actual_center) * mm_s_per_rpm * config.entry_braking_delay_ms / 1000);
      request = std::min(request, std::sqrt(floor_speed * floor_speed +
          2 * config.deceleration_rpm_s * entry_remaining / mm_s_per_rpm));
    } else if (info.distance_mm < before + corner_length) {
      double allowed_corner = corner_speed;
      if (config.corner_floor_rpm > 0) {
        // 用航向及角速度跟随误差预测短时偏差，在硬保护之前主动降低前进速度。
        const double predicted_error = std::fabs(info.error_deg +
            (reference_rate - info.rate_deg_s) * config.governor_lookahead_ms / 1000);
        const double slow = Clamp01((predicted_error - config.heading_slow_start_deg) /
            (config.heading_slow_full_deg - config.heading_slow_start_deg));
        allowed_corner -= (corner_speed - floor_speed) * slow;
        if (slow > 0) info.status = "CORNER SLOW";
      }
      request = std::min(request, allowed_corner);
    } else if (!exit_ready) request = std::min(request, std::min(corner_speed, exit_speed));
    // 全路线末尾才使用低速位置回调，反向仅用于终点过冲恢复。
    if (remaining < 50) request = std::min(request, std::max(config.finish_min_rpm,
        std::min(config.finish_max_rpm, remaining * config.finish_kP)));
    info.planned_rpm = 0;
    if (config.prediction_enabled) {
      info.planned_rpm = Profile.LimitAt(info.distance_mm,
          std::fabs(actual_center) * mm_s_per_rpm * config.entry_braking_delay_ms / 1000);
      // 规划限速优先于人为弯速下限；反馈降速仍可要求更低速度。
      request = std::min(request, info.planned_rpm);
    }
    request = std::copysign(request, info.remaining_mm);
    const double acceleration = info.phase == 3 ? config.exit_acceleration_rpm_s :
        info.phase == 2 && config.corner_acceleration_rpm_s > 0 ? config.corner_acceleration_rpm_s : config.acceleration_rpm_s;
    const double slew = (std::fabs(request) > std::fabs(center) && request * center >= 0 ?
        acceleration : config.deceleration_rpm_s) * dt;
    center += Limit(request - center, slew);
    const double yaw_feedforward = center * curvature * config.track_width_mm / 2;
    const double kp = info.phase == 3 ? config.exit_heading_kP : config.heading_kP;
    const double kd = info.phase == 3 ? config.exit_heading_kD : config.heading_kD;
    const double base_yaw = yaw_feedforward + kp * info.error_deg + kd * (reference_rate - info.rate_deg_s);
    const double yaw_limit = config.max_yaw_fraction * std::fabs(center);
    if (info.phase == 3) {
      const double candidate = Limit(exit_integral + config.exit_heading_kI * info.error_deg * dt,
                                      config.exit_integral_limit_rpm);
      const double command = base_yaw + candidate;
      if (std::fabs(command) <= yaw_limit || (command > yaw_limit && info.error_deg < 0) ||
          (command < -yaw_limit && info.error_deg > 0)) exit_integral = candidate;
    }
    const double yaw = Limit(base_yaw + (info.phase == 3 ? exit_integral : 0), yaw_limit);
    double left = center - yaw, right = center + yaw;
    const double scale = std::max(1.0, std::max(std::fabs(left), std::fabs(right)) / max_rpm);
    left /= scale; right /= scale;
    const double wheel_step = config.acceleration_rpm_s * dt;
    info.left_target_rpm += Limit(left - info.left_target_rpm, wheel_step);
    info.right_target_rpm += Limit(right - info.right_target_rpm, wheel_step);
    info.center_rpm = (info.left_target_rpm + info.right_target_rpm) / 2;
    if (!Velocity.Feedforward(info.left_target_rpm, info.right_target_rpm, info.left_mV, info.right_mV) ||
        !Velocity.ApplyPID(info.left_target_rpm, info.right_target_rpm, frame.velocity, now,
                          info.left_mV, info.right_mV, config.correction_fraction)) { Fail("SPEED ERR"); return; }
    info.velocity = Velocity.Info();
  }
  void CancelCorner() {
    if (info.state == corner_state::running) { info.state = corner_state::cancelled; info.status = "CANCEL"; }
    center = 0; StopOutput();
  }
  void Reset() { CancelCorner(); info = corner_info(); }
 private:
  static constexpr double PI = 3.14159265358979323846;
  static bool Positive(double v) { return std::isfinite(v) && v > 0; }
  static bool Nonnegative(double v) { return std::isfinite(v) && v >= 0; }
  static double Limit(double v, double cap) { return std::max(-cap, std::min(cap, v)); }
  static double Clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }
  bool ValidConfig() const {
    return Positive(config.wheel_diameter_mm) && Positive(config.motor_revs_per_wheel_rev) &&
        Positive(config.track_width_mm) && Positive(config.radius_mm) && config.radius_mm > config.track_width_mm / 2 &&
        Positive(config.corner_rpm) && Positive(config.acceleration_rpm_s) && Positive(config.deceleration_rpm_s) &&
        Nonnegative(config.corner_floor_rpm) && config.corner_floor_rpm <= config.corner_rpm &&
        Nonnegative(config.corner_acceleration_rpm_s) && Nonnegative(config.entry_braking_delay_ms) &&
        (!config.prediction_enabled || (Positive(config.prediction_wheel_acceleration_rpm_s) &&
          Positive(config.prediction_wheel_deceleration_rpm_s) && Positive(config.prediction_margin) &&
          config.prediction_margin <= 1)) &&
        Nonnegative(config.governor_lookahead_ms) && (config.corner_floor_rpm == 0 ||
          (Positive(config.heading_slow_start_deg) && config.heading_slow_full_deg > config.heading_slow_start_deg &&
           config.heading_slow_full_deg < config.max_heading_error_deg)) &&
        Nonnegative(config.heading_kP) && Nonnegative(config.heading_kD) && Nonnegative(config.rate_filter_ms) &&
        Nonnegative(config.exit_heading_kP) && Nonnegative(config.exit_heading_kD) &&
        Nonnegative(config.exit_heading_kI) && Nonnegative(config.exit_integral_limit_rpm) &&
        Positive(config.exit_acceleration_rpm_s) && Positive(config.exit_angle_tolerance_deg) &&
        Positive(config.exit_rate_tolerance_deg_s) && Nonnegative(config.exit_settle_ms) &&
        Positive(config.max_heading_error_deg) && Positive(config.max_yaw_fraction) && config.max_yaw_fraction < 1 &&
        config.track_width_mm / (2 * config.radius_mm) < config.max_yaw_fraction &&
        Positive(config.position_tolerance_mm) && Positive(config.angle_tolerance_deg) &&
        Positive(config.wheel_tolerance_rpm) && Positive(config.rate_tolerance_deg_s) && Nonnegative(config.settle_ms) &&
        Positive(config.finish_kP) && Positive(config.finish_min_rpm) && Positive(config.finish_max_rpm) &&
        config.finish_min_rpm <= config.finish_max_rpm && Positive(config.timeout_ms) &&
        Nonnegative(config.braking_delay_ms) &&
        Positive(config.max_update_ms) && Positive(config.correction_fraction);
  }
  double Position() const {
    return (frame.position.left_deg / 2 + frame.position.right_deg / 2) * (mm_s_per_rpm / 6);
  }
  bool CheckFeedback() {
    if (!have_frame || !allowed) return Fail("DISABLED");
    if (!Nonnegative(now)) return Fail("CLOCK ERR");
    if (!frame.motors_online) return Fail("MOTOR OFF");
    if (!frame.inertial_ready || !std::isfinite(frame.heading_deg)) return Fail("IMU WAIT");
    if (!Velocity.Fresh(now)) return Fail("SPEED ERR");
    if (!frame.position.valid || !std::isfinite(frame.position.left_deg) ||
        !std::isfinite(frame.position.right_deg)) return Fail("POSITION ERR");
    return true;
  }
  void StopOutput() {
    info.left_mV = info.right_mV = 0;
    info.left_target_rpm = info.right_target_rpm = info.center_rpm = 0;
    info.planned_rpm = 0;
    Velocity.Reset(); info.velocity = Velocity.Info();
  }
  bool Fail(const char* status) {
    info.state = corner_state::error; info.status = status; center = 0; StopOutput(); return false;
  }
  corner_config config;
  straight_feedback frame;
  corner_info info;
  velocity_control Velocity;
  corner_profile Profile;
  bool configured = false, have_frame = false, allowed = false;
  double now = 0, started_at = 0, previous_time = 0, previous_angle = 0, previous_position = 0;
  double before = 0, after = 0, angle = 0, corner_length = 0, total = 0, max_rpm = 0;
  double mm_s_per_rpm = 0, origin = 0, initial_heading = 0, center = 0, settled_since = -1;
  bool exit_ready = false;
  double exit_settled_since = -1;
  double exit_integral = 0;
  bool have_exit_speed = false;
  double exit_speed = 0;
};
}  // 连续圆角控制命名空间
