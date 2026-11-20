#include "test-config.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

struct fixture {
  high_speed_control Test;
  manual::input input;
  manual::output output;
  drive::straight_feedback frame;
  double left = 0, right = 0, motor_scale = 1;
  int now = 0;
  explicit fixture(bool warm = true) {
    drive::straight_config s;
    s.motion.wheel_diameter_mm = 69.85;
    s.motion.motor_revs_per_wheel_rev = 1.2;
    s.left_forward.kS = 400;
    s.left_forward.kV = 16;
    s.left_forward.calibrated = true;
    s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
    drive::turn_config t;
    t.left_forward = t.right_forward = t.left_reverse = t.right_reverse = s.left_forward;
    drive::corner_config c;
    c.wheel_diameter_mm = 69.85;
    c.motor_revs_per_wheel_rev = 1.2;
    c.track_width_mm = 325;
    c.left_forward = c.right_forward = c.left_reverse = c.right_reverse = s.left_forward;
    c.prediction_enabled = true;
    c.corner_rpm = 140;
    c.corner_floor_rpm = 100;
    c.angle_tolerance_deg = 1.5;
    c.exit_angle_tolerance_deg = 1.5;
    c.pid.kP = 20;
    c.pid.kI = 20;
    c.pid.max_correction_mV = 2000;
    c.pid.max_integral_mV = 1200;
    Test.Configure(config::HIGH_SPEED_STAGES, s);
    if (warm)
      Step();
  }
  bool Step(bool allowed = true, int manual_left = 0, int fault = 0) {
    const auto motor = [](int mv) {
      return std::copysign(std::max(0.0, std::fabs(static_cast<double>(mv)) - 400) / 16, mv);
    };
    left += (motor(output.left_mV) * motor_scale - left) * 0.25;
    right += (motor(output.right_mV) * motor_scale - right) * 0.25;
    frame.position.left_deg += left * 0.12;
    frame.position.right_deg += right * 0.12;
    frame.heading_deg +=
        (right - left) * (69.85 * 3.141592653589793 / 72) / 325 * 180 / 3.141592653589793 * 0.02;
    now += 20;
    frame.motors_online = fault != 1;
    frame.inertial_ready = fault != 2;
    frame.position.valid = fault != 3;
    frame.velocity.valid = fault != 4;
    frame.velocity.sequence++;
    frame.velocity.time_ms = now;
    frame.velocity.left_rpm = left;
    frame.velocity.right_rpm = right;
    frame.stopped = std::fabs(left) < 5 && std::fabs(right) < 5;
    output = manual::output();
    output.left_mV = manual_left;
    output.arm_enabled = true;
    output.arm_mV = 1234;
    output.claw_mV = 5678;
    output.toggle_piston = true;
    const bool owns = Test.Update(input, output, allowed, now, frame);
    assert(output.arm_enabled && output.arm_mV == 1234 && output.claw_mV == 5678 && output.toggle_piston);
    return owns;
  }
  void Stage(unsigned int stage, int direction) {
    while (Test.Metrics().stage < stage) {
      input.test_next = true;
      Step();
      input.test_next = false;
      Step();
    }
    if (direction < 0) {
      input.test_negative = true;
      Step();
      input.test_negative = false;
      Step();
    }
  }
};
int main() {
  fixture offline;
  offline.frame.motors_online = offline.frame.velocity.valid = offline.frame.stopped = false;
  offline.input.test_start = true;
  offline.Test.Update(offline.input, offline.output, true, offline.now, offline.frame);
  assert(!std::strcmp(offline.Test.Info().status, "MOTOR OFF"));
  assert(!offline.Test.Info().running && offline.output.left_mV == 0 && offline.output.right_mV == 0);
  offline.Step();
  assert(!offline.Test.Info().running);
  for (unsigned int stage = 0; stage < 4; ++stage)
    for (int direction : {-1, 1}) {
      fixture f;
      f.Stage(stage, direction);
      f.input.test_start = true;
      for (int tick = 0; tick < 1000; ++tick) {
        f.Step();
        if (f.Test.Metrics().run && !f.Test.Info().running)
          break;
      }
      const auto m = f.Test.Metrics();
      std::printf("H%u direction %d: %s peak %.1f rpm, hold %.0f ms, error %.2f mm, time %.0f ms\n",
                  stage + 1, direction, f.Test.Info().status, m.peak_rpm, m.at_speed_ms,
                  f.Test.Info().straight.motion.remaining_mm, m.elapsed_ms);
      assert(m.state == drive::motion_state::done && m.qualified && m.at_speed_ms >= 300);
      assert(std::fabs(f.Test.Info().straight.motion.remaining_mm) <= 5);
      assert(m.peak_rpm >= config::HIGH_SPEED_STAGES[stage].maximum_rpm * .9);
      assert(std::isfinite(m.speed_rms_rpm) && m.peak_heading_deg < 1.5);
      assert(f.output.left_mV == 0 && f.output.right_mV == 0);
      for (int i = 0; i < 50; ++i)
        f.Step();
      assert(f.Test.Metrics().run == m.run && f.Test.Metrics().elapsed_ms == m.elapsed_ms &&
             !f.Test.Info().running);
    }
  for (int fault = 0; fault < 7; ++fault) {
    fixture f;
    f.input.test_start = true;
    f.Step();
    f.Step();
    assert(f.Test.Info().running);
    if (fault == 0) {
      f.input.test_cancel = true;
      f.Step();
    } else if (fault == 1)
      f.Step(false);
    else if (fault == 2) {
      f.Step(true, 500);
      assert(f.output.left_mV == 500);
    } else
      f.Step(true, 0, fault - 2);
    assert(!f.Test.Info().running && !f.Test.Metrics().qualified);
    for (int i = 0; i < 30; ++i)
      f.Step();
    assert(!f.Test.Info().running && f.Test.Metrics().run == 1);
  }
  fixture startup(false);
  startup.input.test_start = true;
  for (int i = 0; i < 10; ++i)
    startup.Step();
  assert(!startup.Test.Info().running && startup.Test.Metrics().run == 0);
  fixture keys;
  keys.input.test_next = true;
  for (int i = 0; i < 50; ++i)
    keys.Step();
  assert(keys.Test.Metrics().stage == 1);
  keys.input.test_next = false;
  keys.Step();
  keys.input.test_start = keys.input.test_cancel = true;
  keys.Step();
  assert(!keys.Test.Info().running);
  fixture lock;
  lock.input.test_start = true;
  lock.Step();
  lock.input.test_next = true;
  lock.Step();
  assert(lock.Test.Metrics().stage == 0);
  // 一次瞬时高读数不能冒充持续达速；中断后指标不能报 qualified。
  fixture spike;
  spike.input.test_start = true;
  spike.Step();
  spike.left = spike.right = 240;
  spike.Step();
  assert(!spike.Test.Metrics().qualified && spike.Test.Metrics().at_speed_ms < 300);
  spike.input.test_cancel = true;
  spike.Step();
  assert(!spike.Test.Metrics().qualified);
  fixture slow;
  slow.motor_scale = .45;
  slow.input.test_start = true;
  for (int i = 0; i < 1000; ++i) {
    slow.Step();
    if (slow.Test.Metrics().run && !slow.Test.Info().running)
      break;
  }
  assert(!slow.Test.Info().running && !slow.Test.Metrics().qualified &&
         slow.Test.Metrics().at_speed_ms < 300);
  fixture invalid;
  tuning_stage bad[4] = {
      {1500, 601, 240, 240}, {1500, 265, 320, 320}, {1800, 330, 320, 320}, {1800, 330, 400, 400}};
  drive::straight_config base;
  invalid.Test.Configure(bad, base);
  invalid.Step();
  invalid.input.test_start = true;
  invalid.Step();
  assert(!invalid.Test.Info().running && !std::strcmp(invalid.Test.Info().status, "HS CONFIG"));
  // 位置纠偏配置无效时拒绝起步，不能将坏参数送到电机输出。
  for (double scale : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    fixture bad_gain;
    tuning_stage stages[4];
    for (unsigned int i = 0; i < 4; ++i)
      stages[i] = config::HIGH_SPEED_STAGES[i];
    stages[0].position_gain_scale = scale;
    bad_gain.Test.Configure(stages, base);
    bad_gain.Step();
    bad_gain.input.test_start = true;
    bad_gain.Step();
    assert(!bad_gain.Test.Info().running && !std::strcmp(bad_gain.Test.Info().status, "HS CONFIG"));
    assert(bad_gain.output.left_mV == 0 && bad_gain.output.right_mV == 0);
    stages[0] = config::HIGH_SPEED_STAGES[0];
    stages[0].position_correction_scale = scale;
    bad_gain.input.test_start = false;
    bad_gain.Test.Configure(stages, base);
    bad_gain.Step();
    bad_gain.input.test_start = true;
    bad_gain.Step();
    assert(!bad_gain.Test.Info().running && !std::strcmp(bad_gain.Test.Info().status, "HS CONFIG"));
    assert(bad_gain.output.left_mV == 0 && bad_gain.output.right_mV == 0);
  }
  std::puts(
      "High-speed checks passed: four signed stages, independent qualification, frozen terminal statistics, "
      "startup/interlock/cancel/disable/faults and unchanged mechanisms.");
}
