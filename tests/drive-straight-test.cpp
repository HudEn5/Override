#include "drive-straight.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

drive::straight_config Config() {
  drive::straight_config c;
  c.motion.wheel_diameter_mm = 69.85;
  c.motion.motor_revs_per_wheel_rev = 36.0 / 30.0;
  c.left_forward.kS = 400; c.left_forward.kV = 16; c.left_forward.calibrated = true;
  c.right_forward = c.left_reverse = c.right_reverse = c.left_forward;
  return c;
}
drive::straight_feedback Frame() {
  drive::straight_feedback f;
  f.motors_online = f.inertial_ready = f.stopped = f.velocity.valid = f.position.valid = true;
  f.velocity.sequence = 1;
  return f;
}

int main() {
  drive::straight_controller c;
  c.Configure(Config());
  assert(!c.StartDriveStraight(500, 100)); // 不能借用旧手柄状态，在没有快照时起步。
  auto f = Frame();
  f.heading_deg = 10;
  c.Observe(f, true, 0);
  assert(c.StartDriveStraight(0, 100) && c.Info().state == drive::straight_state::done);
  assert(c.Info().left_mV == 0 && c.Info().right_mV == 0);
  assert(c.StartDriveStraight(123, 100));
  assert(c.Info().motion.remaining_mm == 123 && c.Info().heading.target == 10);
  assert(!c.StartDriveStraight(-400, 200)); // 忙时拒绝覆盖，不悄悄替换已有动作。
  assert(c.Info().state == drive::straight_state::running && c.Info().motion.remaining_mm == 123);
  c.CancelDriveStraight();
  assert(c.Info().state == drive::straight_state::cancelled && c.Info().left_mV == 0);
  f.heading_deg = 40; f.velocity.sequence++; f.velocity.time_ms = 20;
  c.Observe(f, true, 20);
  assert(c.StartDriveStraight(-321, 100));
  assert(c.Info().motion.remaining_mm == -321 && c.Info().heading.target == 40);
  c.Observe(f, true, 121); c.UpdateDriveStraight();
  assert(c.Info().state == drive::straight_state::error && std::strcmp(c.Info().status, "SPEED ERR") == 0);
  assert(c.Info().left_mV == 0 && c.Info().right_mV == 0);
  for (int fault = 0; fault < 4; ++fault) {
    drive::straight_controller x; x.Configure(Config());
    auto value = Frame();
    x.Observe(value, true, 0); assert(x.StartDriveStraight(100, 100));
    value.velocity.sequence++; value.velocity.time_ms = 20;
    if (fault == 0) value.motors_online = false;
    if (fault == 1) value.inertial_ready = false;
    if (fault == 2) value.position.valid = false;
    x.Observe(value, fault != 3, 20); x.UpdateDriveStraight();
    assert(x.Info().state == drive::straight_state::error);
    assert(x.Info().left_mV == 0 && x.Info().right_mV == 0);
    value = Frame(); value.velocity.sequence = 3; value.velocity.time_ms = 40;
    x.Observe(value, true, 40); x.UpdateDriveStraight();
    assert(x.Info().state == drive::straight_state::error); // 反馈恢复不自行重启。
  }
  c.Observe(Frame(), true, 0);
  assert(!c.StartDriveStraight(std::numeric_limits<double>::quiet_NaN(), 100));
  assert(!c.StartDriveStraight(500, 0));
  assert(!c.StartDriveStraight(500, 601));
  std::puts("Straight API checks passed: two-argument start, signed distance, zero, busy, heading reset and faults.");
}
