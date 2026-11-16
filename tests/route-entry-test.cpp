#include "autonomous.h"
#include "motion-presets.h"
#include <cassert>
#include <cstdio>
int main() {
  autonomous_program p;
  drive::straight_config s; s.motion.wheel_diameter_mm = 69.85; s.motion.motor_revs_per_wheel_rev = 1.2;
  s.left_forward.kS = 400; s.left_forward.kV = 16; s.left_forward.calibrated = true;
  s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
  drive::corner_config c; c.wheel_diameter_mm = 69.85; c.motor_revs_per_wheel_rev = 1.2; c.track_width_mm = 325;
  c.left_forward = c.right_forward = c.left_reverse = c.right_reverse = s.left_forward;
  p.Configure(config::HighSpeedStraightControl(s), drive::turn_config(), config::HighSpeedCornerControl(c));
  drive::straight_feedback frame;
  frame.position.valid = frame.velocity.valid = frame.motors_online = frame.stopped = frame.inertial_ready = true;
  p.Update(false, frame, 0); assert(p.Info().run == 0 && !p.Info().active);
  p.Update(true, frame, 10);
  assert(p.Info().active && p.Info().result == 1 && p.Info().run == 1);
  p.Update(false, frame, 20);
  assert(!p.Info().active && p.Info().left_mV == 0 && p.Info().right_mV == 0);
  for (int i = 0; i < 20; ++i) p.Update(false, frame, 30 + i * 10);
  assert(p.Info().run == 1);
  std::puts("Route entry checks passed: actual route builds, explicit enable required and disable stops.");
}
