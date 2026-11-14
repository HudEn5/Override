#include "corner-profile.h"
#include <cassert>
#include <cstdio>
#include <limits>

drive::corner_profile_config Settings(double angle) {
  drive::corner_profile_config c;
  c.before_mm = c.after_mm = 500; c.angle_deg = angle;
  c.radius_mm = 250; c.track_width_mm = 325;
  c.mm_s_per_rpm = 69.85 * 3.141592653589793 / 72;
  c.maximum_rpm = 600; c.corner_rpm = 140; c.entry_rpm = 130; c.finish_rpm = 12;
  c.acceleration_rpm_s = 180; c.deceleration_rpm_s = 120;
  c.corner_acceleration_rpm_s = c.exit_acceleration_rpm_s = 60;
  c.wheel_acceleration_rpm_s = 180; c.wheel_deceleration_rpm_s = 120;
  return c;
}
double Wheel(const drive::corner_profile_config& c, const drive::corner_profile& p, double s, int sign) {
  const double length = 1.5 * std::fabs(c.angle_deg) * 3.141592653589793 / 180 * c.radius_mm;
  const double q = std::max(0.0, std::min(1.0, (s - c.before_mm) / length));
  const double curvature = c.angle_deg * 3.141592653589793 / 180 / length * 6 * q * (1 - q);
  return p.LimitAt(s) * (1 + sign * c.track_width_mm / 2 * curvature);
}
void Check(const drive::corner_profile_config& c) {
  drive::corner_profile p; assert(p.Build(c)); assert(p.Size() >= 129 && p.Size() <= 193);
  const double length = 1.5 * std::fabs(c.angle_deg) * 3.141592653589793 / 180 * c.radius_mm;
  assert(p.LimitAt(c.before_mm) <= c.entry_rpm + 1e-6);
  // 用独立的轮速差/经过时间核对每段内部，而非只核对离散节点。
  for (std::size_t i = 0; i + 1 < p.Size(); ++i) {
    const double ds = p.Point(i + 1).distance_mm - p.Point(i).distance_mm;
    for (int j = 1; j < 100; ++j) {
      const double s = p.Point(i).distance_mm + ds * j / 100;
      const double h = ds / 10000;
      const double dt = 4 * h / (c.mm_s_per_rpm * (p.LimitAt(s - h) + p.LimitAt(s + h)));
      for (int sign : {-1, 1}) {
        const double rpm = Wheel(c, p, s, sign);
        const double acceleration = (Wheel(c, p, s + h, sign) - Wheel(c, p, s - h, sign)) / dt;
        assert(rpm > 0 && rpm <= c.maximum_rpm + 1e-5);
        assert(acceleration <= c.wheel_acceleration_rpm_s * c.margin + 0.001);
        assert(acceleration >= -c.wheel_deceleration_rpm_s * c.margin - 0.001);
      }
      if (s > c.before_mm && s < c.before_mm + length) assert(p.LimitAt(s) <= c.corner_rpm + 1e-6);
      assert(p.LimitAt(s, 100) <= p.LimitAt(s) + 1e-6);
    }
  }
  assert(p.LimitAt(-1) > 0 && p.LimitAt(1e8) <= c.finish_rpm + 1e-6);
}
int main() {
  for (double angle : {0.1, 5., 30., 60., 90., 120., 150., 180.}) {
    auto cfg = Settings(angle); Check(cfg); cfg.angle_deg = -angle; Check(cfg);
    drive::corner_profile positive, negative;
    assert(positive.Build(Settings(angle)) && negative.Build(cfg));
    for (std::size_t i = 0; i < positive.Size(); ++i)
      assert(std::fabs(positive.Point(i).rpm - negative.Point(i).rpm) < 1e-6);
  }
  for (double cap : {20., 60., 150.}) { auto cfg = Settings(30); cfg.maximum_rpm = cap; Check(cfg); }
  for (int zero = 0; zero < 3; ++zero) {
    auto cfg = Settings(30); if (zero != 1) cfg.before_mm = 0; if (zero != 0) cfg.after_mm = 0; Check(cfg);
  }
  drive::corner_profile small, large; assert(small.Build(Settings(30)) && large.Build(Settings(150)));
  assert(small.LimitAt(500) < large.LimitAt(500));
  assert(small.LimitAt(450) < large.LimitAt(450)); // 限速已经反向传到入口之前。
  assert(small.LimitAt(500) < 100); // 不会被人为的 130 rpm 偏好顶回去。
  auto weaker = Settings(30); weaker.wheel_deceleration_rpm_s = 60;
  drive::corner_profile weak; assert(weak.Build(weaker));
  assert(weak.LimitAt(500) < small.LimitAt(500));
  Check(weaker);
  for (int fault = 0; fault < 6; ++fault) {
    auto cfg = Settings(30);
    if (fault == 0) cfg.angle_deg = 0;
    if (fault == 1) cfg.margin = 1.1;
    if (fault == 2) cfg.wheel_deceleration_rpm_s = 0;
    if (fault == 3) cfg.before_mm = std::numeric_limits<double>::infinity();
    if (fault == 4) cfg.maximum_rpm = 601;
    if (fault == 5) cfg.radius_mm = 160;
    assert(!small.Build(cfg) && small.Size() == 0 && small.LimitAt(500) == 0);
  }
  std::puts("Prediction checks passed: signed angles, independent wheel acceleration/velocity checks, anticipatory braking, lower limits and invalid configs.");
}
