#include "autonomous.h"
#include <cassert>
#include <cstdio>
#include <limits>
int scenario = 0;
void autonomous_program::RunRoutine() {
  using namespace route;
  if (scenario == 2) RunRoute({PushFor(std::numeric_limits<double>::quiet_NaN()), DriveStraight(0)});
  else if (scenario == 3) RunRoute({PushFor(0), Wait(0), DriveStraight(0)});
  else RunRoute({PushFor(1000), Wait(100), DriveStraight(0)});
  if (info.active && info.motion == autonomous_motion::push && PushState() == drive::push_state::running) {
    assert(!StartDriveStraight(50, 100) && !StartTurnRelative(90, 100));
    assert(!StartPushFor(500));
  }
}
int main() {
  for (int which = 0; which < 4; ++which) {
    scenario = which; autonomous_program p;
    drive::straight_config s; s.motion.wheel_diameter_mm = 69.85;
    p.Configure(s);
    drive::straight_feedback f;
    f.motors_online = f.inertial_ready = f.position.valid = f.velocity.valid = f.stopped = true;
    int powered = 0;
    for (int time = 0; time < 2000 && !p.Info().finished; time += 20) {
      f.velocity.sequence++; f.velocity.time_ms = time;
      p.Update(!(which == 1 && time >= 400), f, time);
      if (p.Info().left_mV) { assert(p.Info().left_mV == 4000 && p.Info().right_mV == 4000); ++powered; }
      if (time >= 1000) assert(p.Info().left_mV == 0 && p.Info().right_mV == 0);
    }
    assert(p.Info().finished);
    if (which == 0) assert(p.Info().result == 2 && p.Info().step == 3 && powered == 50);
    if (which == 1) assert(p.Info().result == 3 && p.Info().step == 0 && powered == 20);
    if (which == 2) assert(p.Info().result == 4 && p.Info().step == 0 && powered == 0);
    if (which == 3) assert(p.Info().result == 2 && p.Info().step == 3 && powered == 0);
    assert(p.Info().left_mV == 0 && p.Info().right_mV == 0);
  }
  std::puts("Autonomous PushFor checks passed: cold-start observe/reset, one-second stalled push, follow-on Wait/drive, exclusive ownership, disable and invalid/zero duration.");
}
