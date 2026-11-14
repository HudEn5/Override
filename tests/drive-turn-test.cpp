#include "drive-turn.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

drive::turn_config Settings() {
  drive::turn_config c;
  c.left_forward.kS = 400; c.left_forward.kV = 16; c.left_forward.calibrated = true;
  c.right_forward = c.left_reverse = c.right_reverse = c.left_forward;
  return c;
}
drive::straight_feedback Feedback(double now, double angle = 0, double left = 0, double right = 0) {
  drive::straight_feedback f;
  f.motors_online = f.inertial_ready = f.velocity.valid = true;
  f.stopped = std::fabs(left) < 5 && std::fabs(right) < 5;
  f.heading_deg = angle;
  f.velocity.time_ms = now; f.velocity.sequence = static_cast<unsigned int>(now / 20 + 1);
  f.velocity.left_rpm = left; f.velocity.right_rpm = right;
  return f;
}
double Motor(int voltage) {
  return std::fabs(static_cast<double>(voltage)) <= 400 ? 0 :
      (voltage > 0 ? voltage - 400 : voltage + 400) / 16.0;
}
drive::turn_info LoadedTurn(bool tuned, double delta, double breakaway, double friction) {
  auto cfg = Settings();
  if (tuned) {
    cfg.kP = 1.6; cfg.acceleration_rpm_s = 240; cfg.minimum_settle_rpm = 18;
    cfg.minimum_finish_rpm = 12; cfg.minimum_rate_deg_s = 16;
    cfg.correction_fraction = 3;
    cfg.pid.kP = 20; cfg.pid.kI = 20;
    cfg.pid.max_correction = 2000; cfg.pid.max_integral = 1200;
  }
  drive::turn_controller c; c.Configure(cfg);
  c.Observe(Feedback(0), true, 0); assert(c.StartTurnRelative(delta, tuned ? 100 : 60));
  double angle = 0, left = 0, right = 0, previous_target = 0;
  const auto motor = [&](int mV, double speed) {
    const double v = std::fabs(static_cast<double>(mV));
    if (std::fabs(speed) < 0.5 && v < breakaway) return 0.0;
    return std::copysign(std::max(0.0, v - friction) / 16, mV);
  };
  for (int i = 1; i <= 600; ++i) {
    auto old = c.Info();
    left += (motor(old.left_mV, left) - left) * 0.2;
    right += (motor(old.right_mV, right) - right) * 0.2;
    angle += (right - left) * 0.6 * 0.02;
    c.Observe(Feedback(i * 20, angle, left, right), true, i * 20); c.UpdateTurn();
    const auto v = c.Info();
    assert(std::abs(v.left_mV) <= 12000 && std::abs(v.right_mV) <= 12000);
    if (std::strcmp(v.status, "TURN") == 0)
      assert(std::fabs(v.target_rpm - previous_target) <= cfg.acceleration_rpm_s * 0.02 + 1e-6);
    previous_target = v.target_rpm;
    if (v.state != drive::turn_state::running) break;
  }
  return c.Info();
}
void Simulate(double delta) {
  drive::turn_controller c; c.Configure(Settings());
  double angle = 350, left = 0, right = 0, previous_target = 0;
  c.Observe(Feedback(0, angle), true, 0);
  assert(c.StartTurnRelative(delta, 60));
  for (int i = 1; i <= 600; ++i) {
    const auto old = c.Info();
    // 电压驱动一阶负载，模拟左右差异；角速度来自两侧实际速度。
    left += (Motor(old.left_mV) * 0.95 - left) * 0.2;
    right += (Motor(old.right_mV) * 1.03 - right) * 0.2;
    angle += (right - left) * 0.6 * 0.02;
    c.Observe(Feedback(i * 20, angle, left, right), true, i * 20); c.UpdateTurn();
    const auto v = c.Info();
    assert(std::abs(v.left_mV) <= 12000 && std::abs(v.right_mV) <= 12000);
    assert(std::fabs(v.target_rpm) <= 60);
    // 进入精度区直接 brake；其余周期轮速目标遵守加减速上限。
    if (std::strcmp(v.status, "TURN") == 0)
      assert(std::fabs(v.target_rpm - previous_target) <= 2.400001);
    previous_target = v.target_rpm;
    assert(v.state != drive::turn_state::error);
    if (v.state == drive::turn_state::done) break;
  }
  assert(c.Info().state == drive::turn_state::done);
  assert(std::fabs(angle - (350 + delta)) <= 0.8);
  assert(c.Info().left_mV == 0 && c.Info().right_mV == 0);
  for (int i = 601; i < 620; ++i) {
    c.Observe(Feedback(i * 20, angle + 2), true, i * 20); c.UpdateTurn();
    assert(c.Info().state == drive::turn_state::done && c.Info().left_mV == 0);
  }
}

int main() {
  const auto stalled = LoadedTurn(false, 90, 1550, 1100);
  assert(stalled.state == drive::turn_state::error && std::strcmp(stalled.status, "TIMEOUT") == 0);
  for (double friction : {900.0, 1100.0, 1300.0})
    for (double delta : {-180.0, -90.0, -5.0, 5.0, 90.0, 180.0}) {
      const auto r = LoadedTurn(true, delta, friction + 450, friction);
      if (r.state != drive::turn_state::done)
        std::printf("Loaded turn failed: friction %.0f delta %.0f error %.2f status %s\n", friction, delta, r.error_deg, r.status);
      assert(r.state == drive::turn_state::done && std::fabs(r.error_deg) <= 0.8);
    }
  for (double delta : {0.0, 5.0, 45.0, 90.0, 180.0, 270.0, 360.0, -5.0, -90.0, -180.0, -360.0}) Simulate(delta);
  drive::turn_controller c; c.Configure(Settings());
  assert(!c.StartTurnRelative(90, 60));
  c.Observe(Feedback(0, 350), true, 0); assert(c.StartTurnToHeading(10, 60));
  assert(c.Info().target_deg == 370 && c.Info().error_deg == 20);
  assert(!c.StartTurnRelative(-90, 60) && c.Info().target_deg == 370);
  c.CancelTurn(); assert(c.Info().state == drive::turn_state::cancelled && c.Info().left_mV == 0);
  c.Observe(Feedback(20, 10), true, 20); assert(c.StartTurnToHeading(350, 60));
  assert(c.Info().target_deg == -10);
  c.CancelTurn(); c.Observe(Feedback(40, 180), true, 40); assert(c.StartTurnToHeading(0, 60));
  assert(c.Info().error_deg == 180);
  c.CancelTurn(); assert(!c.StartTurnRelative(std::numeric_limits<double>::quiet_NaN(), 60));
  assert(!c.StartTurnRelative(90, 0)); assert(!c.StartTurnRelative(90, 601));
  for (int fault = 0; fault < 7; ++fault) {
    drive::turn_controller x; auto cfg = Settings(); cfg.timeout_ms = 80; x.Configure(cfg);
    auto f = Feedback(0); x.Observe(f, true, 0); assert(x.StartTurnRelative(90, 60));
    double now = 20;
    f = Feedback(now);
    if (fault == 0) f.motors_online = false;
    if (fault == 1) f.inertial_ready = false;
    if (fault == 2) f.velocity.valid = false;
    if (fault == 3) f.heading_deg = std::numeric_limits<double>::quiet_NaN();
    if (fault == 4) now = 240;  // 主循环异常长间隔。
    if (fault == 5) now = 80;   // 惯导角不变也必须超时停止。
    x.Observe(f, fault != 6, now); x.UpdateTurn();
    assert(x.Info().state == drive::turn_state::error && x.Info().left_mV == 0 && x.Info().right_mV == 0);
    x.Observe(Feedback(260), true, 260); x.UpdateTurn();
    assert(x.Info().state == drive::turn_state::error); // 故障恢复不会自行重启。
  }
  drive::turn_controller stale; stale.Configure(Settings());
  auto f = Feedback(0); stale.Observe(f, true, 0); assert(stale.StartTurnRelative(90, 60));
  stale.Observe(f, true, 120); stale.UpdateTurn(); assert(stale.Info().state == drive::turn_state::error);
  drive::turn_controller moving; moving.Configure(Settings());
  moving.Observe(Feedback(0, 0, 20, 20), true, 0); assert(!moving.StartTurnRelative(90, 60));
  std::puts("Turn checks passed: voltage-driven bidirectional simulations, wrap, full rotations, slew, settle, busy and fault stops.");
}
