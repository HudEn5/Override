#include "test-config.h"
#include <cassert>
#include <cstdio>
#include <cstring>

struct fixture {
  test_control Test;
  manual::Input input;
  manual::Output output;
  drive::straight_feedback frame;
  double left = 0, right = 0;
  int now = 0;
  explicit fixture(bool warm = true) {
    drive::straight_config s; s.motion.wheel_diameter_mm = 69.85; s.motion.motor_revs_per_wheel_rev = 1.2;
    s.left_forward.kS = 400; s.left_forward.kV = 16; s.left_forward.calibrated = true;
    s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
    drive::turn_config t; t.left_forward = t.right_forward = t.left_reverse = t.right_reverse = s.left_forward;
    drive::corner_config c; c.wheel_diameter_mm = 69.85; c.motor_revs_per_wheel_rev = 1.2; c.track_width_mm = 325;
    c.left_forward = c.right_forward = c.left_reverse = c.right_reverse = s.left_forward;
    c.prediction_enabled = true; c.corner_rpm = 140; c.corner_floor_rpm = 100;
    c.angle_tolerance_deg = 1.5; c.exit_angle_tolerance_deg = 1.5;
    c.pid.kP = 20; c.pid.kI = 20; c.pid.max_correction = 2000; c.pid.max_integral = 1200;
    Test.Configure(config::TestSettings(), s, t, c);
    if (warm) Step();
  }
  bool Step(bool allowed = true, int manual_left = 0, int fault = 0) {
    const auto motor = [](int mv) { return std::copysign(std::max(0.0, std::fabs(static_cast<double>(mv)) - 400) / 16, mv); };
    left += (motor(output.L_voltage) - left) * 0.25;
    right += (motor(output.R_voltage) - right) * 0.25;
    frame.position.left_deg += left * 0.12; frame.position.right_deg += right * 0.12;
    frame.heading_deg += (right - left) * (69.85 * 3.141592653589793 / 72) / 325 * 180 / 3.141592653589793 * 0.02;
    now += 20;
    frame.motors_online = fault != 1; frame.inertial_ready = fault != 2;
    frame.position.valid = fault != 3; frame.velocity.valid = fault != 4;
    frame.velocity.sequence++; frame.velocity.time_ms = now;
    frame.velocity.left_rpm = left; frame.velocity.right_rpm = right;
    frame.stopped = std::fabs(left) < 5 && std::fabs(right) < 5;
    output = manual::Output(); output.L_voltage = manual_left;
    output.arm_enabled = true; output.arm_voltage = 1234; output.claw_voltage = 5678; output.toggle_piston = true;
    const bool owns = Test.Update(input, output, allowed, now, frame);
    assert(output.arm_enabled && output.arm_voltage == 1234 && output.claw_voltage == 5678 && output.toggle_piston);
    return owns;
  }
  void Select(test_motion selected, int direction = 1) {
    while (Test.Info().selected != selected) {
      input.test_next = true; Step(); input.test_next = false; Step();
    }
    if (direction < 0) { input.test_negative = true; Step(); input.test_negative = false; Step(); }
  }
};
int main() {
  fixture navigation;
  navigation.input.test_next = true;
  for (int i = 0; i < 100; ++i) navigation.Step();
  assert(navigation.Test.Info().selected == test_motion::turn && !navigation.Test.Info().running);
  navigation.input.test_next = false; navigation.Step();
  navigation.input.test_next = true; navigation.Step();
  assert(navigation.Test.Info().selected == test_motion::corner);
  navigation.input.test_next = false; navigation.Step();
  navigation.input.test_previous = true; navigation.Step();
  assert(navigation.Test.Info().selected == test_motion::turn);
  navigation.input = manual::Input(); navigation.Step();
  navigation.input.test_negative = true; navigation.Step();
  assert(navigation.Test.Info().request_deg == -config::TEST_TURN_DEG);
  navigation.input.test_positive = true; navigation.Step(); // 左右同时按住不改变方向。
  assert(navigation.Test.Info().direction == -1);
  for (test_motion selected : {test_motion::straight, test_motion::turn, test_motion::corner})
    for (int direction : {-1, 1}) {
      fixture f; f.Select(selected, direction); f.input.test_start = true;
      for (int i = 0; i < 1500; ++i) {
        f.Step();
        assert(std::abs(f.output.L_voltage) <= 12000 && std::abs(f.output.R_voltage) <= 12000);
        if (!std::strcmp(f.Test.Info().status, "DONE")) break;
      }
      if (std::strcmp(f.Test.Info().status, "DONE")) std::printf("task %d direction %d failed: %s\n", static_cast<int>(selected), direction, f.Test.Info().status);
      assert(!f.Test.Info().running && !std::strcmp(f.Test.Info().status, "DONE"));
      assert(f.output.L_voltage == 0 && f.output.R_voltage == 0);
      for (int i = 0; i < 30; ++i) { f.Step(); assert(!f.Test.Info().running); } // 按住 A 不重复运行。
      if (selected == test_motion::straight)
        assert(std::fabs(f.Test.Info().straight.motion.distance_mm - direction * config::TEST_STRAIGHT_MM) <= 5);
      else assert(std::fabs(f.frame.heading_deg - direction * (selected == test_motion::turn ? config::TEST_TURN_DEG : config::TEST_CORNER_DEG)) <= 1.5);
    }
  for (int interruption = 0; interruption < 7; ++interruption) {
    fixture f; f.input.test_start = true; f.Step(); f.Step(); assert(f.Test.Info().running);
    if (interruption == 0) { f.input.test_cancel = true; assert(f.Step()); }
    else if (interruption == 1) assert(!f.Step(false));
    else if (interruption == 2) { assert(!f.Step(true, 500)); assert(f.output.L_voltage == 500); }
    else f.Step(true, 0, interruption - 2);
    assert(!f.Test.Info().running);
    if (interruption != 2) assert(f.output.L_voltage == 0 && f.output.R_voltage == 0);
    for (int i = 0; i < 20; ++i) f.Step();
    assert(!f.Test.Info().running); // 恢复后仍按住 A，不能自行重启。
  }
  fixture startup(false); startup.input.test_start = true;
  for (int i = 0; i < 10; ++i) startup.Step();
  assert(!startup.Test.Info().running);
  startup.input.test_start = false; startup.Step(); startup.input.test_start = true; startup.Step();
  assert(startup.Test.Info().running);
  startup.input.test_next = true; startup.input.test_negative = true; startup.Step();
  assert(startup.Test.Info().selected == test_motion::straight && startup.Test.Info().direction == 1);
  fixture simultaneous; simultaneous.input.test_start = simultaneous.input.test_cancel = true; simultaneous.Step();
  assert(!simultaneous.Test.Info().running);
  fixture selecting; selecting.input.test_start = selecting.input.test_next = true; selecting.Step();
  assert(!selecting.Test.Info().running && selecting.Test.Info().selected == test_motion::turn);
  selecting.input.test_next = false; selecting.Step(); assert(!selecting.Test.Info().running);
  std::puts("Unified tests passed: three signed actions, DONE latch, edge selection, startup hold, B priority, stick takeover, fault/disable stops and unchanged mechanism commands.");
}
