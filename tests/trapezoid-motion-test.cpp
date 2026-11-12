#include "voltage-test.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

drive::feedforward_config Model() {
  drive::feedforward_config result;
  result.kS = 400; result.kV = 16; result.calibrated = true;
  return result;
}

struct fixture {
  int volts[3] = {2000, 3000, 4000};
  int targets[3] = {100, 150, 200};
  voltage_test test;
  manual::Input input;
  manual::Output output;
  drive::velocity_feedback speed;
  drive::position_feedback position;
  double now = 0, heading = 0;
  bool enabled = true, online = true, imu = true;
  fixture(double distance = 500) : test(volts, 3000) {
    test.ConfigureFeedforward(targets, Model(), Model(), 1000);
    test.ConfigureVelocityPID(drive::velocity_pid_config());
    test.ConfigureHeading(drive::heading_config());
    drive::trapezoid_config config;
    config.wheel_diameter_mm = 69.85; config.motor_revs_per_wheel_rev = 36.0 / 30.0;
    config.distance_mm = distance; test.ConfigureTrapezoid(config);
    speed.valid = position.valid = true;
    position.left_deg = 1234; position.right_deg = -567; // 不依赖编码器清零。
    Step(); Switch(); Switch(); Switch();
    assert(test.Info().trapezoid && test.Info().speed_control);
  }
  void Step(double dt = 10, bool new_sample = true, bool joystick = false) {
    now += dt;
    if (new_sample) { ++speed.sequence; speed.time_ms = now; }
    output = manual::Output();
    output.arm_enabled = true; output.arm_voltage = 12000; output.claw_voltage = -6000;
    if (joystick) output.L_voltage = output.R_voltage = 1000;
    test.Update(input, output, enabled, online, now,
        std::fabs(speed.left_rpm) < 5 && std::fabs(speed.right_rpm) < 5,
        imu, heading, speed, position);
    assert(output.arm_voltage == 12000 && output.claw_voltage == -6000);
  }
  void Switch() {
    input = manual::Input(); Step();
    input.voltage_down = true; Step(); Step(1000);
    input.voltage_down = false; Step();
  }
};

double PlantTarget(int voltage) {
  return std::fabs(static_cast<double>(voltage)) < 400 ? 0 :
      (voltage - (voltage > 0 ? 400.0 : -400.0)) / 16.0;
}

void Simulate(int direction, int level, double distance) {
  fixture f(distance);
  for (int i = 0; i < level; ++i) {
    f.input.voltage_up = true; f.Step(); f.input.voltage_up = false; f.Step();
  }
  f.input.voltage_forward = direction > 0; f.input.voltage_reverse = direction < 0;
  f.Step(); assert(f.test.Info().running && f.output.L_voltage == 0);
  double left = 0, right = 0, peak = 0;
  bool decelerated = false;
  for (int i = 0; i < 1500 && f.test.Info().running; ++i) {
    left += 0.2 * (PlantTarget(f.output.L_voltage) - left);
    right += 0.2 * (PlantTarget(f.output.R_voltage) - right);
    f.position.left_deg += left * 6 * 0.01;
    f.position.right_deg += right * 6 * 0.01;
    const bool sample = i % 2 == 0;
    if (sample) {
      f.speed.left_rpm += (left - f.speed.left_rpm) / 3.0;
      f.speed.right_rpm += (right - f.speed.right_rpm) / 3.0;
    }
    f.Step(10, sample);
    peak = std::fmax(peak, std::fabs(f.test.Info().motion.reference_rpm));
    decelerated |= f.test.Info().motion.phase == 3;
    assert(std::abs(f.output.L_voltage) <= 12000 && std::abs(f.output.R_voltage) <= 12000);
    assert(std::fabs(f.test.Info().target_rpm) <= f.targets[level] + 1e-8);
  }
  if (std::strcmp(f.test.Info().status, "DONE") != 0)
    std::printf("Simulation failed: dir=%d rpm=%d distance=%.0f status=%s remaining=%.2f\n",
                direction, f.targets[level], distance, f.test.Info().status,
                f.test.Info().motion.remaining_mm);
  assert(std::strcmp(f.test.Info().status, "DONE") == 0);
  assert(std::fabs(f.test.Info().motion.remaining_mm) <= 5);
  assert(f.output.L_voltage == 0 && f.output.R_voltage == 0 && decelerated);
  assert(peak <= f.targets[level] && peak > 0);
  if (distance == 1000) assert(f.test.Info().motion.elapsed_ms > 3000);
  const double last = f.test.Info().motion.distance_mm;
  f.Step(); assert(!f.test.Info().running && f.output.L_voltage == 0);
  assert(f.test.Info().motion.distance_mm == last); // 持续按住也不重复执行。
}

int main() {
  for (int direction : {-1, 1})
    for (int level = 0; level < 3; ++level) Simulate(direction, level, 500);
  Simulate(1, 0, 1000); // 动作超时由规划时长决定，不继承 SP 的固定 3 秒。
  fixture f;
  f.input.telemetry = true; f.Step(); assert(!f.test.Info().running); // Y 只回传。
  f.input.voltage_forward = true; f.Step(); f.Step();
  assert(f.test.Info().running);
  f.input.voltage_forward = false; f.Step();
  assert(f.output.L_voltage == 0 && f.test.Info().motion.phase == 6);
  f.input.voltage_forward = true; f.Step(); f.Step();
  f.input.voltage_forward = false; f.input.voltage_reverse = true; f.Step();
  assert(!f.test.Info().running && f.output.L_voltage == 0);
  f.input.voltage_reverse = false; f.Step();
  f.input.voltage_reverse = true; f.Step();
  f.Step(101, false);
  assert(!f.test.Info().running && f.output.L_voltage == 0);
  assert(std::strcmp(f.test.Info().status, "SPEED ERR") == 0);
  f.Step(); assert(!f.test.Info().running);
  f.input.voltage_reverse = false; f.Step();
  f.input.voltage_forward = true; f.Step();
  f.position.valid = false; f.Step();
  assert(f.output.L_voltage == 0 && std::strcmp(f.test.Info().status, "POSITION ERR") == 0);
  f.position.valid = true; f.input.voltage_forward = false; f.Step();
  f.speed.left_rpm = 10; f.input.voltage_forward = true; f.Step();
  assert(f.output.L_voltage == 0 && std::strcmp(f.test.Info().status, "MOVING") == 0);
  f.speed.left_rpm = 0; f.input.voltage_forward = false; f.Step();
  f.input.voltage_forward = true; f.Step(); f.Step(201);
  assert(f.output.L_voltage == 0 && std::strcmp(f.test.Info().status, "CLOCK ERR") == 0);

  for (int fault = 0; fault < 5; ++fault) {
    fixture cancel;
    cancel.input.voltage_forward = true; cancel.Step(); cancel.Step();
    if (fault == 0) cancel.enabled = false;
    if (fault == 1) cancel.online = false;
    if (fault == 2) cancel.imu = false;
    if (fault == 3) cancel.input.turn_test = true;
    cancel.Step(10, true, fault == 4);
    assert(!cancel.test.Info().running && !cancel.test.Info().motion.running);
    assert(cancel.output.L_voltage == (fault == 4 ? 1000 : 0));
    if (fault < 3) assert(cancel.test.Info().visible); // 主循环据测试视图继续 brake。
  }
  fixture stalled;
  stalled.input.voltage_forward = true; stalled.Step();
  for (int i = 0; i < 1000 && stalled.test.Info().running; ++i) stalled.Step();
  assert(std::strcmp(stalled.test.Info().status, "TIMEOUT") == 0 && stalled.output.L_voltage == 0);
  fixture invalid(0);
  invalid.input.voltage_forward = true; invalid.Step();
  assert(std::strcmp(invalid.test.Info().status, "TP CONFIG") == 0 && invalid.output.L_voltage == 0);
  fixture yaw;
  yaw.input.voltage_forward = true; yaw.Step();
  yaw.heading = 1; yaw.Step();
  assert(yaw.test.Info().target_rpm > 0 && yaw.test.Info().target_rpm < 5);
  assert(yaw.test.Info().velocity.left_target > 0 && yaw.test.Info().velocity.right_target > 0);
  assert(std::fabs(yaw.test.Info().heading.correction) <=
         yaw.test.Info().target_rpm * 16 * 0.25 + 1e-9);
  f.Switch(); assert(!f.test.Info().feedforward && !f.test.Info().trapezoid); // TP -> VTEST。
  std::puts("TP checks passed: closed-loop 500 mm forward/reverse at three caps, triangle, long move, release, faults and mode cycle.");
}
