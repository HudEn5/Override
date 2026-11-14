#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace drive {
struct corner_profile_config {
  double before_mm = 0, after_mm = 0, angle_deg = 0, radius_mm = 0, track_width_mm = 0;
  double mm_s_per_rpm = 0, maximum_rpm = 0, corner_rpm = 0, entry_rpm = 0, finish_rpm = 0;
  double acceleration_rpm_s = 0, deceleration_rpm_s = 0;
  double corner_acceleration_rpm_s = 0, exit_acceleration_rpm_s = 0;
  double wheel_acceleration_rpm_s = 0, wheel_deceleration_rpm_s = 0;
  double margin = 0.85;  // 预留航向修正和实车响应余量，不代表已标定的机械极限。
};
struct corner_profile_point { double distance_mm = 0, rpm = 0; };

// 路径按切点分段，曲线内的速度平方线性插值。固定容量，无堆分配、无线程共享。
class corner_profile {
 public:
  bool Build(const corner_profile_config& value) {
    ready = false; count = 0; config = value;
    if (!Valid()) return false;
    length = 1.5 * std::fabs(config.angle_deg) * PI / 180 * config.radius_mm;
    total = config.before_mm + length + config.after_mm;
    if (!Positive(length) || !Positive(total)) return false;
    points[count++].distance_mm = 0;
    if (!Append(config.before_mm, 32) || !Append(length, 128) || !Append(config.after_mm, 32)) return false;
    for (std::size_t i = 0; i < count; ++i) {
      const double s = points[i].distance_mm;
      double factor = 0, derivative = 0;
      Shape(s, s >= config.before_mm && s <= config.before_mm + length, factor, derivative);
      points[i].rpm = std::min(config.maximum_rpm / (1 + std::fabs(factor)),
          s >= config.before_mm && s <= config.before_mm + length ? config.corner_rpm : config.maximum_rpm);
      if (s == config.before_mm) points[i].rpm = std::min(points[i].rpm, config.entry_rpm);
    }
    // 邻段中最大的曲率导数限定匀速可行值，给前后向求解提供可行的二分下界。
    const double wheel_budget = std::min(config.wheel_acceleration_rpm_s,
                                         config.wheel_deceleration_rpm_s) * config.margin;
    for (std::size_t i = 0; i + 1 < count; ++i) {
      double f = 0, d0 = 0, d1 = 0;
      const bool curve = IsCurve(i);
      Shape(points[i].distance_mm, curve, f, d0);
      double maximum_factor = std::fabs(f);
      Shape(points[i + 1].distance_mm, curve, f, d1);
      maximum_factor = std::max(maximum_factor, std::fabs(f));
      if (curve && points[i].distance_mm <= config.before_mm + length / 2 &&
          points[i + 1].distance_mm >= config.before_mm + length / 2)
        maximum_factor = config.track_width_mm / (2 * config.radius_mm);
      const double wheel_cap = config.maximum_rpm / (1 + maximum_factor);
      points[i].rpm = std::min(points[i].rpm, wheel_cap);
      points[i + 1].rpm = std::min(points[i + 1].rpm, wheel_cap);
      const double derivative = std::max(std::fabs(d0), std::fabs(d1));
      if (derivative > 0) {
        const double cap = std::sqrt(wheel_budget / (config.mm_s_per_rpm * derivative));
        points[i].rpm = std::min(points[i].rpm, cap);
        points[i + 1].rpm = std::min(points[i + 1].rpm, cap);
      }
    }
    // 起点实际从静止按时间加速；小的非零规划种子避免空间速度在零位置锁死。
    points[0].rpm = std::min(points[0].rpm, config.finish_rpm);
    points[count - 1].rpm = std::min(points[count - 1].rpm, config.finish_rpm);
    bool converged = false;
    for (unsigned int pass = 0; pass < 64; ++pass) {
      bool changed = false;
      // 前向限制能加到的速度；后向限制到下一个限速点之前能否减下来。
      for (std::size_t i = 0; i + 1 < count; ++i)
        if (points[i + 1].rpm > points[i].rpm && !Feasible(i, points[i].rpm, points[i + 1].rpm))
          changed |= Reduce(i, false);
      for (std::size_t i = count - 1; i > 0; --i)
        if (points[i - 1].rpm > points[i].rpm && !Feasible(i - 1, points[i - 1].rpm, points[i].rpm))
          changed |= Reduce(i - 1, true);
      if (!changed) { converged = true; break; }
    }
    if (!converged) return false;
    for (std::size_t i = 0; i + 1 < count; ++i)
      if (!Feasible(i, points[i].rpm, points[i + 1].rpm)) return false;
    ready = true;
    return true;
  }
  // 向前查看响应距离内的最低限速，只能提前减速，不能提前采用更高速度。
  double LimitAt(double distance_mm, double response_mm = 0) const {
    if (!ready || !std::isfinite(distance_mm) || !std::isfinite(response_mm)) return 0;
    double cap = SpeedAt(distance_mm);
    const double end = distance_mm + std::max(0.0, response_mm);
    cap = std::min(cap, SpeedAt(end));
    for (std::size_t i = 0; i < count; ++i)
      if (points[i].distance_mm > distance_mm && points[i].distance_mm < end)
        cap = std::min(cap, points[i].rpm);
    return cap;
  }
  std::size_t Size() const { return ready ? count : 0; }
  const corner_profile_point& Point(std::size_t i) const { return points[i]; }
 private:
  static constexpr double PI = 3.14159265358979323846;
  static bool Positive(double v) { return std::isfinite(v) && v > 0; }
  bool Valid() const {
    return std::isfinite(config.before_mm) && config.before_mm >= 0 &&
        std::isfinite(config.after_mm) && config.after_mm >= 0 &&
        Positive(std::fabs(config.angle_deg)) && std::fabs(config.angle_deg) <= 180 &&
        Positive(config.radius_mm) && Positive(config.track_width_mm) &&
        config.radius_mm > config.track_width_mm / 2 && Positive(config.mm_s_per_rpm) &&
        Positive(config.maximum_rpm) && config.maximum_rpm <= 600 &&
        Positive(config.corner_rpm) && Positive(config.entry_rpm) && Positive(config.finish_rpm) &&
        Positive(config.acceleration_rpm_s) && Positive(config.deceleration_rpm_s) &&
        Positive(config.corner_acceleration_rpm_s) && Positive(config.exit_acceleration_rpm_s) &&
        Positive(config.wheel_acceleration_rpm_s) && Positive(config.wheel_deceleration_rpm_s) &&
        Positive(config.margin) && config.margin <= 1;
  }
  bool Append(double distance, unsigned int steps) {
    if (distance == 0) return true;
    const double start = points[count - 1].distance_mm;
    for (unsigned int j = 1; j <= steps; ++j) {
      const double s = start + distance * (static_cast<double>(j) / steps);
      if (!Positive(s) || s <= points[count - 1].distance_mm || count >= points.size()) return false;
      points[count++].distance_mm = s;
    }
    return true;
  }
  bool IsCurve(std::size_t i) const {
    const double middle = (points[i].distance_mm + points[i + 1].distance_mm) / 2;
    return middle > config.before_mm && middle < config.before_mm + length;
  }
  void Shape(double s, bool curve, double& factor, double& derivative) const {
    factor = derivative = 0;
    if (!curve) return;
    const double q = std::max(0.0, std::min(1.0, (s - config.before_mm) / length));
    const double k = 3 * config.track_width_mm * (config.angle_deg * PI / 180) / length;
    factor = k * q * (1 - q);  // 右侧相对中心的速度增量系数，左侧取负。
    derivative = k * (1 - 2 * q) / length;
  }
  bool Feasible(std::size_t i, double from, double to) const {
    const double ds = points[i + 1].distance_mm - points[i].distance_mm;
    const double x = from * from, y = to * to;
    const double a = (y - x) * config.mm_s_per_rpm / (2 * ds);
    const bool curve = IsCurve(i);
    const double acceleration = curve ? config.corner_acceleration_rpm_s :
        points[i].distance_mm >= config.before_mm + length ? config.exit_acceleration_rpm_s : config.acceleration_rpm_s;
    if (a > acceleration + 1e-7 || a < -config.deceleration_rpm_s - 1e-7) return false;
    for (int sign : {-1, 1}) {
      const auto rate = [&](double t) {
        double f = 0, derivative = 0;
        Shape(points[i].distance_mm + ds * t, curve, f, derivative);
        return a * (1 + sign * f) + sign * config.mm_s_per_rpm * (x + (y - x) * t) * derivative;
      };
      const double r0 = rate(0), rm = rate(0.5), r1 = rate(1);
      double lo = std::min(r0, r1), hi = std::max(r0, r1);
      // 曲率为二次函数，段内轮加速度也为二次函数，额外检查内点极值。
      const double qa = 2 * (r1 + r0 - 2 * rm), qb = r1 - r0 - qa;
      if (std::fabs(qa) > 1e-12) {
        const double t = -qb / (2 * qa);
        if (t > 0 && t < 1) { lo = std::min(lo, rate(t)); hi = std::max(hi, rate(t)); }
      }
      if (hi > config.wheel_acceleration_rpm_s * config.margin + 1e-7 ||
          lo < -config.wheel_deceleration_rpm_s * config.margin - 1e-7) return false;
    }
    return true;
  }
  bool Reduce(std::size_t i, bool backward) {
    double low = backward ? points[i + 1].rpm : points[i].rpm;
    double high = backward ? points[i].rpm : points[i + 1].rpm;
    for (unsigned int j = 0; j < 28; ++j) {
      const double mid = (low + high) / 2;
      if (Feasible(i, backward ? mid : points[i].rpm, backward ? points[i + 1].rpm : mid)) low = mid;
      else high = mid;
    }
    double& value = backward ? points[i].rpm : points[i + 1].rpm;
    const bool changed = value > low;
    value = low;
    return changed;
  }
  double SpeedAt(double s) const {
    if (s <= 0) return points[0].rpm;
    for (std::size_t i = 0; i + 1 < count; ++i) {
      if (s <= points[i + 1].distance_mm) {
        const double q = (s - points[i].distance_mm) / (points[i + 1].distance_mm - points[i].distance_mm);
        return std::sqrt(std::max(0.0, (1 - q) * points[i].rpm * points[i].rpm +
                                                   q * points[i + 1].rpm * points[i + 1].rpm));
      }
    }
    return points[count - 1].rpm;
  }
  corner_profile_config config;
  std::array<corner_profile_point, 193> points;
  std::size_t count = 0;
  double length = 0, total = 0;
  bool ready = false;
};
}  // 圆角速度规划命名空间
