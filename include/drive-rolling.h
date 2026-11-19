#pragma once

#include "drive-corner.h"
#include "drive-turn.h"

namespace drive {
// 连续移动转向的独立快照；距离余量只针对当前直线段，不能当作全路线余量。
using rolling_state = motion_state;
struct rolling_info {
  rolling_state state = rolling_state::idle;
  const char* status = "READY";
  int phase = 0;  // 0 空闲，1 进入直线，2 移动转向，3 后续直线，4 终点停稳。
  unsigned completed_turns = 0, total_turns = 0;
  double time_ms = 0, elapsed_ms = 0, distance_mm = 0, segment_remaining_mm = 0;
  double reference_deg = 0, angle_deg = 0, error_deg = 0, rate_deg_s = 0;
  double center_rpm = 0, left_target_rpm = 0, right_target_rpm = 0;
  int left_mV = 0, right_mV = 0;
  velocity_loop_info velocity;
};

// 每项表示前直线、一次移动转向、后直线；相邻项的后/前直线相加。
struct rolling_leg {
  double before_mm = 0, angle_deg = 0, after_mm = 0;
  double maximum_rpm = 0, entry_mm_s = 0, timeout_ms = 0;
};
constexpr unsigned MAX_ROLLING_TURNS = 16;

// 通用连续路线：一次拥有全部转角，保持速度环状态，仅整条路线终点停稳。
class rolling_controller {
 public:
  void Configure(const straight_config& s, const turn_config& t, const corner_config& c) {
    straight = s;
    turn = t;
    config = c;
    Velocity.ConfigureForward(s.left_forward, s.right_forward);
    Velocity.ConfigureReverse(s.left_reverse, s.right_reverse);
    Velocity.ConfigurePID(c.pid);
    info = rolling_info();
  }
  const rolling_info& Info() const {
    return info;
  }
  void Observe(const straight_feedback& f, bool permission, double time) {
    frame = f;
    allowed = permission;
    now = time;
    Velocity.Observe(f.velocity, time);
  }
  bool StartDriveTurnDrive(double before_mm, double angle_deg, double after_mm, double maximum_rpm,
                           double entry_mm_s, double timeout_ms) {
    rolling_leg leg;
    leg.before_mm = before_mm;
    leg.angle_deg = angle_deg;
    leg.after_mm = after_mm;
    leg.maximum_rpm = maximum_rpm;
    leg.entry_mm_s = entry_mm_s;
    leg.timeout_ms = timeout_ms;
    return StartPath(&leg, 1);
  }
  bool StartPath(const rolling_leg* legs, unsigned count) {
    if (info.state == rolling_state::running)
      return false;
    info = rolling_info();
    if (!legs || count == 0 || count > MAX_ROLLING_TURNS || !Positive(straight.motion.wheel_diameter_mm) ||
        !Positive(straight.motion.motor_revs_per_wheel_rev) || !Positive(turn.track_width_mm) ||
        !Positive(turn.angular_acceleration_deg_s2) || !Positive(turn.angular_deceleration_deg_s2) ||
        !Positive(straight.motion.acceleration_rpm_s) || !Positive(straight.motion.deceleration_rpm_s) ||
        !Positive(config.position_tolerance_mm) || !Positive(config.angle_tolerance_deg) ||
        !Positive(config.wheel_tolerance_rpm) || !Positive(config.rate_tolerance_deg_s) ||
        !Positive(config.timeout_ms) || !Positive(config.max_update_ms) || !std::isfinite(config.settle_ms) ||
        config.settle_ms < 0)
      return Fail("ROLL CONFIG");
    const double nonnegative[] = {turn.kP,
                                  turn.kD,
                                  turn.profile_finish_kP,
                                  turn.minimum_finish_rpm,
                                  turn.rate_filter_ms,
                                  config.exit_heading_kP,
                                  config.exit_heading_kD,
                                  config.braking_delay_ms,
                                  config.finish_kP};
    for (double value : nonnegative)
      if (!std::isfinite(value) || value < 0)
        return Fail("ROLL CONFIG");
    if (!Positive(config.finish_min_rpm) || !Positive(config.finish_max_rpm) ||
        config.finish_max_rpm < config.finish_min_rpm || !Positive(config.correction_fraction) ||
        !Positive(config.max_heading_error_deg))
      return Fail("ROLL CONFIG");
    mm_s_per_rpm = PI * straight.motion.wheel_diameter_mm / (60 * straight.motion.motor_revs_per_wheel_rev);
    deg_s_per_rpm = 6 * straight.motion.wheel_diameter_mm / straight.motion.motor_revs_per_wheel_rev /
                    turn.track_width_mm;
    if (!Positive(mm_s_per_rpm) || !Positive(deg_s_per_rpm))
      return Fail("ROLL CONFIG");
    turn_count = count;
    for (unsigned i = 0; i < count; ++i) {
      const auto& leg = legs[i];
      if (!std::isfinite(leg.before_mm) || leg.before_mm < 0 || !std::isfinite(leg.after_mm) ||
          leg.after_mm < 0 || !Positive(std::fabs(leg.angle_deg)) || std::fabs(leg.angle_deg) > 180 ||
          !Positive(leg.maximum_rpm) || leg.maximum_rpm > 600 || !Positive(leg.entry_mm_s) ||
          !Positive(leg.timeout_ms))
        return Fail("ROLL CONFIG");
      path[i] = leg;
    }
    // 先检查整条路线；最后必须有足够距离刹停，不能在开跑后才发现后续非法参数。
    if (path[count - 1].after_mm <= config.position_tolerance_mm)
      return Fail("ROLL END DIST");
    for (unsigned i = 0; i < count; ++i) {
      entries[i] = path[i].entry_mm_s / mm_s_per_rpm;
      if (!Positive(entries[i]) || entries[i] >= TurnMaximum(i) || !std::isfinite(LineDistance(i)))
        return Fail("ROLL CONFIG");
    }
    // 从终点向前传播减速预算；短中间直线会提前压低前一个弯速。
    for (unsigned end = count; end > 0; --end) {
      const unsigned i = end - 1;
      const double next = i + 1 < count ? entries[i + 1] : 0;
      const double distance = std::max(0., LineDistance(i + 1) - config.position_tolerance_mm);
      const double decel = straight.motion.deceleration_rpm_s;
      const double delay = config.braking_delay_ms / 1000;
      const double bound =
          std::sqrt(decel * decel * delay * delay + next * next + 2 * decel * distance / mm_s_per_rpm) -
          decel * delay;
      entries[i] = std::min(entries[i], std::max(next, bound));
      if (!Positive(entries[i]))
        return Fail("ROLL END DIST");
    }
    // 正向传播加速预算，避免短段末端突然抬高下一弯的前进目标。
    for (unsigned i = 1; i < count; ++i)
      entries[i] = std::min(
          entries[i], std::sqrt(entries[i - 1] * entries[i - 1] +
                                2 * straight.motion.acceleration_rpm_s * LineDistance(i) / mm_s_per_rpm));
    for (unsigned i = 0; i < count; ++i) {
      const auto profile =
          motion::trapezoid_profile(path[i].angle_deg, (TurnMaximum(i) - entries[i]) * deg_s_per_rpm,
                                    turn.angular_acceleration_deg_s2, turn.angular_deceleration_deg_s2);
      if (!profile.Valid())
        return Fail("ROLL CONFIG");
    }
    if (!Check())
      return false;
    if (!std::isfinite(Position()))
      return Fail("POSITION ERR");
    if (!frame.stopped || std::fabs(frame.velocity.left_rpm) > config.wheel_tolerance_rpm ||
        std::fabs(frame.velocity.right_rpm) > config.wheel_tolerance_rpm)
      return Fail("MOVING");
    info.state = rolling_state::running;
    info.status = "ROLL IN";
    info.phase = 1;
    initial_heading = previous_angle = frame.heading_deg;
    info.angle_deg = info.reference_deg = initial_heading;
    origin = phase_origin = previous_position = Position();
    completed_heading = initial_heading;
    previous_time = started_at = section_started = now;
    settled_since = -1;
    turns = timed_leg = 0;
    leg_started_at = now;
    info.total_turns = turn_count;
    turning = false;
    center = yaw = 0;
    segment = LineDistance(0);
    max_rpm = LineMaximum(0);
    entry = entries[0];
    Velocity.ResetControl();
    return true;
  }

 public:
  void Update() {
    if (info.state != rolling_state::running || !Check())
      return;
    const double dt_ms = now - previous_time;
    if (dt_ms < 0 || dt_ms > config.max_update_ms) {
      Fail("CLOCK ERR");
      return;
    }
    if (dt_ms == 0)
      return;
    const double dt = dt_ms / 1000, position = Position();
    if (std::fabs(position - previous_position) > 720 * mm_s_per_rpm * dt + 1) {
      Fail("POSITION JUMP");
      return;
    }
    info.time_ms = now;
    info.elapsed_ms = now - started_at;
    info.distance_mm = position - origin;
    info.angle_deg = frame.heading_deg;
    info.rate_deg_s +=
        dt_ms / (turn.rate_filter_ms + dt_ms) * ((frame.heading_deg - previous_angle) / dt - info.rate_deg_s);
    previous_time = now;
    previous_position = position;
    previous_angle = frame.heading_deg;
    if (!CheckDeadline(position))
      return;
    if (now - section_started >= config.timeout_ms) {
      Fail("TIMEOUT");
      return;
    }
    if (!turning && turns < turn_count)
      entry = entries[turns];
    const double actual_center = (frame.velocity.left_rpm + frame.velocity.right_rpm) / 2;
    const double remaining = segment - (position - phase_origin);
    info.segment_remaining_mm = remaining;
    // 入角既看剩余距离，也检查实际前进速度；不带着高速直接启动角速度规划。
    if (!turning && turns < turn_count && remaining <= config.position_tolerance_mm &&
        std::fabs(actual_center) <= entry * 1.15) {
      turning = true;
      turn_started = now;
      max_rpm = TurnMaximum(turns);
      Profile = motion::trapezoid_profile(path[turns].angle_deg, (max_rpm - entry) * deg_s_per_rpm,
                                          turn.angular_acceleration_deg_s2, turn.angular_deceleration_deg_s2);
    }
    double request = 0, yaw_request = 0;
    if (turning) {
      const double elapsed = (now - turn_started) / 1000;
      const auto reference = Profile.Sample(elapsed);
      info.reference_deg = completed_heading + reference.position;
      info.error_deg = info.reference_deg - frame.heading_deg;
      const double final_error = completed_heading + path[turns].angle_deg - frame.heading_deg;
      info.phase = 2;
      info.status = "ROLL TURN";
      request = entry;
      yaw_request = reference.velocity / deg_s_per_rpm +
                    (elapsed >= Profile.Duration() ? turn.profile_finish_kP : turn.kP) * info.error_deg +
                    turn.kD * (reference.velocity - info.rate_deg_s);
      if (elapsed >= Profile.Duration() && std::fabs(final_error) > config.angle_tolerance_deg &&
          std::fabs(yaw_request) < turn.minimum_finish_rpm)
        yaw_request = std::copysign(turn.minimum_finish_rpm, final_error);
      if (elapsed >= Profile.Duration() && std::fabs(final_error) <= config.angle_tolerance_deg) {
        completed_heading += path[turns].angle_deg;
        ++turns;
        info.completed_turns = turns;
        turning = false;
        phase_origin = position;
        segment = LineDistance(turns);
        section_started = now;
        if (!CheckDeadline(position))
          return;
      }
    } else {
      max_rpm = LineMaximum(turns);
      entry = turns < turn_count ? entries[turns] : 0;
      info.reference_deg = completed_heading;
      info.error_deg = info.reference_deg - frame.heading_deg;
      info.phase = turns == 0 ? 1 : 3;
      info.status = "ROLL STR";
      const double exit = turns == turn_count ? 0 : entry;
      const double budget =
          std::max(0., remaining - std::fabs(actual_center) * mm_s_per_rpm * config.braking_delay_ms / 1000);
      request = std::min(
          max_rpm, std::sqrt(exit * exit + 2 * straight.motion.deceleration_rpm_s * budget / mm_s_per_rpm));
      yaw_request = config.exit_heading_kP * info.error_deg - config.exit_heading_kD * info.rate_deg_s;
      if (std::fabs(info.error_deg) > config.max_heading_error_deg) {
        Fail("HEADING ERR");
        return;
      }
      if (turns == turn_count && std::fabs(remaining) <= config.position_tolerance_mm) {
        info.phase = 4;
        info.status = "SETTLE";
        Stop();
        if (std::fabs(info.error_deg) > config.angle_tolerance_deg) {
          Fail("END HEADING");
          return;
        }
        if (std::fabs(frame.velocity.left_rpm) <= config.wheel_tolerance_rpm &&
            std::fabs(frame.velocity.right_rpm) <= config.wheel_tolerance_rpm &&
            std::fabs(info.rate_deg_s) <= config.rate_tolerance_deg_s) {
          if (settled_since < 0)
            settled_since = now;
          if (now - settled_since >= config.settle_ms) {
            info.state = rolling_state::done;
            info.status = "DONE";
          }
        } else
          settled_since = -1;
        return;
      }
      if (turns == turn_count && remaining < 50)
        request =
            std::copysign(std::max(config.finish_min_rpm,
                                   std::min(config.finish_max_rpm, std::fabs(remaining) * config.finish_kP)),
                          remaining);
    }
    settled_since = -1;
    info.segment_remaining_mm = turns == turn_count && !turning ? remaining : std::max(0., remaining);
    if (!std::isfinite(yaw_request) || !std::isfinite(request)) {
      Fail("ROLL CONFIG");
      return;
    }
    center = motion::LimitVelocity(center, request, straight.motion.acceleration_rpm_s,
                                   straight.motion.deceleration_rpm_s, dt);
    yaw = motion::LimitVelocity(yaw, yaw_request, turn.angular_acceleration_deg_s2 / deg_s_per_rpm,
                                turn.angular_deceleration_deg_s2 / deg_s_per_rpm, dt);
    const double left = center - yaw, right = center + yaw;
    const double scale = std::max(1., std::max(std::fabs(left), std::fabs(right)) / max_rpm);
    info.left_target_rpm = left / scale;
    info.right_target_rpm = right / scale;
    info.center_rpm = (info.left_target_rpm + info.right_target_rpm) / 2;
    if (!Velocity.Feedforward(info.left_target_rpm, info.right_target_rpm, info.left_mV, info.right_mV) ||
        !Velocity.ApplyPID(info.left_target_rpm, info.right_target_rpm, frame.velocity, now, info.left_mV,
                           info.right_mV, config.correction_fraction)) {
      Fail("SPEED ERR");
      return;
    }
    info.velocity = Velocity.Info();
  }
  void Cancel() {
    if (info.state == rolling_state::running) {
      info.state = rolling_state::cancelled;
      info.status = "CANCEL";
    }
    Stop();
  }
  void Reset() {
    Cancel();
    info = rolling_info();
  }

 private:
  static constexpr double PI = 3.14159265358979323846;
  static bool Positive(double x) {
    return std::isfinite(x) && x > 0;
  }
  bool CheckDeadline(double position) {
    // 相邻项共享直段；走完上一项after才开始下一项before的预算。
    // 必须先检查旧项期限，迟到不能通过推进计时项绕过超时。
    if (now - leg_started_at >= path[timed_leg].timeout_ms)
      return Fail("TIMEOUT");
    while (timed_leg + 1 < turn_count && turns > timed_leg &&
           position - phase_origin >= std::max(0., path[timed_leg].after_mm - config.position_tolerance_mm)) {
      ++timed_leg;
      leg_started_at = now;
    }
    return true;
  }
  double LineDistance(unsigned index) const {
    if (index == 0)
      return path[0].before_mm;
    return path[index - 1].after_mm + (index < turn_count ? path[index].before_mm : 0);
  }
  double LineMaximum(unsigned index) const {
    if (index == 0)
      return path[0].maximum_rpm;
    return index == turn_count ? path[index - 1].maximum_rpm
                               : std::min(path[index - 1].maximum_rpm, path[index].maximum_rpm);
  }
  double TurnMaximum(unsigned index) const {
    return index + 1 == turn_count ? path[index].maximum_rpm
                                   : std::min(path[index].maximum_rpm, path[index + 1].maximum_rpm);
  }
  double Position() const {
    return (frame.position.left_deg / 2 + frame.position.right_deg / 2) * mm_s_per_rpm / 6;
  }
  bool Check() {
    if (!allowed)
      return Fail("DISABLED");
    if (!std::isfinite(now) || now < 0)
      return Fail("CLOCK ERR");
    if (!frame.motors_online)
      return Fail("MOTOR OFF");
    if (!frame.inertial_ready || !std::isfinite(frame.heading_deg))
      return Fail("IMU WAIT");
    if (!frame.position.valid || !std::isfinite(frame.position.left_deg) ||
        !std::isfinite(frame.position.right_deg))
      return Fail("POSITION ERR");
    if (!Velocity.Fresh(now))
      return Fail("SPEED ERR");
    return true;
  }
  void Stop() {
    info.left_mV = info.right_mV = 0;
    info.left_target_rpm = info.right_target_rpm = info.center_rpm = 0;
    center = yaw = 0;
    Velocity.ResetControl();
    info.velocity = Velocity.Info();
  }
  bool Fail(const char* s) {
    info.state = rolling_state::error;
    info.status = s;
    Stop();
    return false;
  }
  straight_config straight;
  turn_config turn;
  corner_config config;
  straight_feedback frame;
  rolling_info info;
  velocity_control Velocity;
  motion::trapezoid_profile Profile{0, 1, 1, 1};
  bool allowed = false, turning = false;
  std::array<rolling_leg, MAX_ROLLING_TURNS> path;
  std::array<double, MAX_ROLLING_TURNS> entries;
  unsigned turns = 0, turn_count = 0, timed_leg = 0;
  double now = 0, previous_time = 0, started_at = 0, previous_angle = 0, previous_position = 0, origin = 0;
  double mm_s_per_rpm = 0, deg_s_per_rpm = 0, max_rpm = 0, entry = 0, segment = 0, phase_origin = 0;
  double completed_heading = 0, section_started = 0, leg_started_at = 0;
  double initial_heading = 0, turn_started = 0, center = 0, yaw = 0, settled_since = -1;
};

}  // 连续移动转向命名空间
