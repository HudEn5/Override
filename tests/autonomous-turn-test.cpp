#include "autonomous.h"
#include <cassert>
#include <cstdio>
#include <cstring>

void autonomous_program::RunRoutine() {
  switch (info.step) {
    case 0:
      assert(StartDriveStraight(100, 60));
      assert(!StartTurnRelative(90, 60)); // 两类运动不能同时拥有底盘。
      info.step = 1; break;
    case 1:
      if (DriveStraightState() == drive::straight_state::done) {
        assert(StartTurnRelative(90, 60));
        assert(!StartDriveStraight(100, 60));
        info.step = 2;
      }
      break;
    case 2:
      if (TurnState() == drive::turn_state::done) {
        assert(StartDriveStraight(-100, 60)); info.step = 3;
      }
      break;
    case 3:
      if (DriveStraightState() == drive::straight_state::done) {
        assert(StartTurnToHeading(0, 60)); info.step = 4;
      }
      break;
    case 4:
      if (TurnState() == drive::turn_state::done) FinishRoutine();
      break;
  }
}
int main() {
  drive::straight_config c; c.motion.wheel_diameter_mm = 69.85;
  c.left_forward.kS = 400; c.left_forward.kV = 16; c.left_forward.calibrated = true;
  c.right_forward = c.left_reverse = c.right_reverse = c.left_forward;
  for (int scenario = 0; scenario < 3; ++scenario) {
    autonomous_program p; p.Configure(c);
    drive::straight_feedback f;
    f.position.valid = f.velocity.valid = f.motors_online = f.inertial_ready = f.stopped = true;
    double left = 0, right = 0;
    bool interrupted = false;
    for (int tick = 0; tick < 4000 && !p.Info().finished; ++tick) {
      const auto v = p.Info();
      const auto rpm = [](int mV) { return std::abs(mV) <= 400 ? 0 : (mV - (mV > 0 ? 400.0 : -400.0)) / 16; };
      left += 0.2 * (rpm(v.left_mV) - left); right += 0.2 * (rpm(v.right_mV) - right);
      f.position.left_deg += left * 0.06; f.position.right_deg += right * 0.06;
      f.heading_deg += (right - left) * 0.6 * 0.01;
      f.velocity.left_rpm = left; f.velocity.right_rpm = right;
      f.velocity.sequence++; f.velocity.time_ms = tick * 10;
      f.stopped = std::fabs(left) < 5 && std::fabs(right) < 5;
      if (scenario && v.motion == autonomous_motion::turn && std::fabs(v.turn.target_rpm) > 5) {
        interrupted = true;
        if (scenario == 1) f.inertial_ready = false;
        p.Update(scenario != 2, f, tick * 10);
        assert(!p.Info().active && p.Info().left_mV == 0 && p.Info().right_mV == 0);
        break;
      }
      p.Update(true, f, tick * 10);
      if (p.Info().step == 3) assert(std::fabs(p.Info().drive.heading.target - 90) <= 0.8);
    }
    if (scenario) assert(interrupted);
    else {
      assert(p.Info().finished && p.Info().step == 4 && std::strcmp(p.Info().status, "DONE") == 0);
      assert(std::fabs(f.heading_deg) <= 0.8 && p.Info().left_mV == 0 && p.Info().right_mV == 0);
    }
  }
  std::puts("Mixed route checks passed: straight/turn/straight/heading, exclusive ownership, IMU fault and disable.");
}
