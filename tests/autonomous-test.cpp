#include "autonomous.h"
#include <cassert>
#include <cstdio>
#include <cstring>

// 用测试路线替代 src/route.cpp，验证真实调度器和公共直行层；无 VEX 设备或手柄依赖。
void autonomous_program::RunRoutine() {
  switch (info.step) {
    case 0:
      if (!StartDriveStraight(200, 100)) { FinishRoutine("START ERROR"); return; }
      info.step = 1; break;
    case 1:
      if (DriveStraightState() == drive::straight_state::done) {
        if (!StartDriveStraight(-150, 80)) { FinishRoutine("START ERROR"); return; }
        info.step = 2;
      }
      break;
    case 2:
      if (DriveStraightState() == drive::straight_state::done) FinishRoutine();
      break;
  }
}

drive::straight_config Config() {
  drive::straight_config c;
  c.motion.wheel_diameter_mm = 69.85; c.motion.motor_revs_per_wheel_rev = 36.0 / 30.0;
  c.left_forward.kS = 400; c.left_forward.kV = 16; c.left_forward.calibrated = true;
  c.right_forward = c.left_reverse = c.right_reverse = c.left_forward;
  return c;
}
drive::straight_feedback Frame() {
  drive::straight_feedback f;
  f.position.valid = f.velocity.valid = f.motors_online = f.stopped = f.inertial_ready = true;
  return f;
}
int main() {
  autonomous_program p; p.Configure(Config());
  auto f = Frame();
  f.inertial_ready = false;
  p.Update(true, f, 0);
  assert(p.Info().active && p.Info().step == 0 && std::strcmp(p.Info().status, "WAIT READY") == 0);
  f.inertial_ready = true;
  double left = 0, right = 0;
  for (int tick = 1; tick < 2000 && !p.Info().finished; ++tick) {
    const auto last = p.Info().drive;
    const auto rpm = [](int mV) { return std::abs(mV) < 400 ? 0 : (mV - (mV > 0 ? 400.0 : -400.0)) / 16; };
    left += 0.2 * (rpm(last.left_mV) - left); right += 0.2 * (rpm(last.right_mV) - right);
    f.position.left_deg += left * 0.06; f.position.right_deg += right * 0.06;
    f.velocity.left_rpm = left; f.velocity.right_rpm = right;
    f.velocity.sequence++; f.velocity.time_ms = tick * 10;
    f.stopped = std::fabs(left) < 5 && std::fabs(right) < 5;
    p.Update(true, f, tick * 10);
  }
  assert(p.Info().finished && p.Info().step == 2 && std::strcmp(p.Info().status, "DONE") == 0);
  assert(p.Info().drive.left_mV == 0 && p.Info().drive.right_mV == 0);
  const unsigned int run = p.Info().run;
  for (int i = 0; i < 100; ++i) p.Update(true, f, 30000 + i * 10);
  assert(p.Info().run == run && !p.Info().active); // 同一自动时段仅执行一次。
  p.Update(false, f, 40000);
  f = Frame(); f.velocity.sequence = 10000; f.velocity.time_ms = 40010;
  p.Update(true, f, 40010);
  assert(p.Info().active && p.Info().run == run + 1);
  p.Update(false, f, 40020);
  assert(!p.Info().active && p.Info().drive.left_mV == 0 && p.Info().drive.right_mV == 0);
  std::puts("Autonomous checks passed: ready wait, shared straight sequence, once-per-period and disable cancellation.");
}
