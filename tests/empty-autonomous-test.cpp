#include "autonomous.h"
#include <cassert>
#include <cstdio>
#include <cstring>

int main() {
  autonomous_program p;
  drive::straight_config config;
  p.Configure(config);
  drive::straight_feedback f;
  f.position.valid = f.velocity.valid = f.motors_online = f.stopped = f.inertial_ready = true;
  p.Update(true, f, 0);
  assert(p.Info().finished && !p.Info().active && std::strcmp(p.Info().status, "EMPTY") == 0);
  assert(p.Info().drive.left_mV == 0 && p.Info().drive.right_mV == 0);
  for (int i = 1; i < 100; ++i) p.Update(true, f, i * 10);
  assert(p.Info().run == 1);
  std::puts("Default match route checks passed: EMPTY, zero voltage and no automatic test motion.");
}
