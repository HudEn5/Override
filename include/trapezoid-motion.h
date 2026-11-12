#pragma once

#include "trapezoid-profile.h"
#include "velocity-control.h"
#include <algorithm>

namespace drive {
struct position_feedback {
  bool valid = false;
  double left_deg = 0;
  double right_deg = 0;
};

struct trapezoid_config {
  double wheel_diameter_mm = 0;
  double motor_revs_per_wheel_rev = 1;
  double distance_mm = 500;
  double acceleration_rpm_s = 120;
  double deceleration_rpm_s = 120;
  double position_kP = 0.25;  // rpm/mm，与原距离测试的比例增益一致。
  double max_position_correction_rpm = 20;
  double position_tolerance_mm = 5;
  double velocity_tolerance_rpm = 5;
  double settle_ms = 250;
  double timeout_margin_ms = 3000;
  double max_update_ms = 200;
};

struct trapezoid_info {
  bool running = false;
  double time_ms = 0;
  double elapsed_ms = 0;
  double duration_ms = 0;
  double reference_mm = 0;
  double reference_rpm = 0;
  double distance_mm = 0;
  double remaining_mm = 0;
  double target_rpm = 0;
  int phase = 0;  // 0 空闲，1 加速，2 匀速，3 减速，4 停稳，5 完成，6 取消，7 故障。
  const char* status = "READY";
};

// 同事的纯梯形规划提供位置/速度参考；本层仅输出中心目标 rpm，不写电机。
class trapezoid_motion {
 public:
  void Configure(const trapezoid_config& value) { config = value; info = trapezoid_info(); }
  const trapezoid_info& Info() const { return info; }
  void Cancel() { if (info.running) Finish("CANCEL", 6); }

  bool Start(int direction, double max_rpm, const position_feedback& position,
             const velocity_feedback& speed, double now, bool stopped) {
    info = trapezoid_info();
    if (!Positive(config.wheel_diameter_mm) || !Positive(config.motor_revs_per_wheel_rev) ||
        !Positive(config.distance_mm) || !Positive(config.acceleration_rpm_s) ||
        !Positive(config.deceleration_rpm_s) || !Nonnegative(config.position_kP) ||
        !Positive(config.max_position_correction_rpm) || !Positive(config.position_tolerance_mm) ||
        !Positive(config.velocity_tolerance_rpm) || !Nonnegative(config.settle_ms) ||
        !Positive(config.timeout_margin_ms) || !Positive(config.max_update_ms) ||
        !Positive(max_rpm) || max_rpm > 600 || (direction != 1 && direction != -1) ||
        !Nonnegative(now)) return Fail("TP CONFIG");
    if (!Valid(position, speed)) return Fail("POSITION ERR");
    if (!stopped || std::fabs(speed.left_rpm) > config.velocity_tolerance_rpm ||
        std::fabs(speed.right_rpm) > config.velocity_tolerance_rpm) return Fail("MOVING");
    mm_per_degree = 3.14159265358979323846 * config.wheel_diameter_mm /
                    (360.0 * config.motor_revs_per_wheel_rev);
    mm_s_per_rpm = mm_per_degree * 6.0;
    if (!Positive(mm_per_degree) || !Positive(mm_s_per_rpm)) return Fail("TP CONFIG");
    profile = motion::TrapezoidProfile(direction * config.distance_mm,
        max_rpm * mm_s_per_rpm, config.acceleration_rpm_s * mm_s_per_rpm,
        config.deceleration_rpm_s * mm_s_per_rpm);
    if (!profile.valid()) return Fail("TP CONFIG");
    origin_left = position.left_deg; origin_right = position.right_deg;
    maximum_rpm = max_rpm;
    goal = direction * config.distance_mm;
    started_at = previous_time = now;
    settled_at = -1;
    info.running = true; info.phase = 1; info.status = "ACCEL";
    info.time_ms = now; info.duration_ms = profile.duration() * 1000;
    info.remaining_mm = goal;
    return true;
  }

  bool Update(const position_feedback& position, const velocity_feedback& speed, double now) {
    if (!info.running) return info.phase == 5;
    if (!Valid(position, speed)) return Fail("POSITION ERR");
    if (!Nonnegative(now) || now < previous_time || now - previous_time > config.max_update_ms)
      return Fail("CLOCK ERR");
    const double dt = (now - previous_time) / 1000;
    previous_time = now;
    info.time_ms = now; info.elapsed_ms = now - started_at;
    if (info.elapsed_ms >= info.duration_ms + config.timeout_margin_ms) return Fail("TIMEOUT");
    const motion::ProfileState reference = profile.sample(info.elapsed_ms / 1000);
    info.reference_mm = reference.position;
    info.reference_rpm = reference.velocity / mm_s_per_rpm;
    info.distance_mm = ((position.left_deg - origin_left) / 2.0 +
                        (position.right_deg - origin_right) / 2.0) * mm_per_degree;
    info.remaining_mm = goal - info.distance_mm;
    if (!std::isfinite(info.distance_mm) || !std::isfinite(info.remaining_mm)) return Fail("POSITION ERR");
    const bool near = std::fabs(info.remaining_mm) <= config.position_tolerance_mm;
    const bool reference_done = info.elapsed_ms >= info.duration_ms;
    const bool settled = reference_done && near &&
        std::fabs(speed.left_rpm) <= config.velocity_tolerance_rpm &&
        std::fabs(speed.right_rpm) <= config.velocity_tolerance_rpm;
    if (!settled) settled_at = -1;
    else if (settled_at < 0) settled_at = now;
    if (settled && now - settled_at >= config.settle_ms) { Finish("DONE", 5); return true; }
    info.phase = reference_done ? 4 : (info.elapsed_ms < profile.accelerationTime() * 1000 ? 1 :
        (info.elapsed_ms < (profile.accelerationTime() + profile.cruiseTime()) * 1000 ? 2 : 3));
    info.status = info.phase == 1 ? "ACCEL" : info.phase == 2 ? "CRUISE" :
                  info.phase == 3 ? "DECEL" : "SETTLE";
    const double correction = Clamp(config.position_kP * (reference.position - info.distance_mm),
                                     config.max_position_correction_rpm);
    const double requested = Clamp(info.reference_rpm + correction, maximum_rpm);
    // 到达位置窗口后直接制动，等实际速度也稳定；故障和松手同样立即制动。
    info.target_rpm = reference_done && near ? 0 : motion::limitVelocity(info.target_rpm,
        requested, config.acceleration_rpm_s, config.deceleration_rpm_s, dt);
    if (!std::isfinite(info.target_rpm)) return Fail("TP CONFIG");
    return true;
  }

 private:
  static bool Positive(double v) { return std::isfinite(v) && v > 0; }
  static bool Nonnegative(double v) { return std::isfinite(v) && v >= 0; }
  static double Clamp(double v, double limit) { return std::max(-limit, std::min(limit, v)); }
  static bool Valid(const position_feedback& p, const velocity_feedback& v) {
    return p.valid && v.valid && std::isfinite(p.left_deg) && std::isfinite(p.right_deg) &&
        std::isfinite(v.left_rpm) && std::isfinite(v.right_rpm);
  }
  void Finish(const char* status, int phase) {
    info.running = false; info.target_rpm = 0; info.status = status; info.phase = phase;
  }
  bool Fail(const char* status) { Finish(status, 7); return false; }
  trapezoid_config config;
  trapezoid_info info;
  motion::TrapezoidProfile profile{0, 1, 1, 1};
  double origin_left = 0, origin_right = 0, mm_per_degree = 0, mm_s_per_rpm = 0;
  double maximum_rpm = 0, goal = 0, started_at = 0, previous_time = 0, settled_at = -1;
};
}  // 底盘轨迹控制命名空间
