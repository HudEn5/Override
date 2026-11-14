#include "calibration-control.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

drive::feedforward_config Model() {
  drive::feedforward_config model;
  model.kS = 400; model.kV = 16; model.calibrated = true;
  return model;
}

struct fixture {
  int volts[3] = {2000, 3000, 4000};
  int targets[3] = {100, 150, 200};
  calibration_control test;
  manual::Input input;
  manual::Output output;
  drive::velocity_feedback speed;
  double heading = 0;
  double now = 0;
  bool online = true;
  bool enabled = true;
  fixture() : test(volts, 3000) {
    test.ConfigureFeedforward(targets, Model(), Model(), 1000);
    drive::heading_config yaw; yaw.kD = 0;
    test.ConfigureHeading(yaw);
    test.ConfigureVelocityPID(drive::velocity_pid_config());
    speed.valid = true;
    Step(0);
    Switch(); // 固定电压 -> FH。
  }
  void Step(double time, bool new_sample = true, bool joystick = false) {
    now = time;
    if (new_sample) { ++speed.sequence; speed.time_ms = time; }
    output = manual::Output();
    if (joystick) output.L_voltage = output.R_voltage = 1000;
    test.Update(input, output, enabled, online, time, true, true, heading, speed);
  }
  void Switch() {
    input = manual::Input(); Step(now + 10);
    input.test_previous = true; Step(now + 10); Step(now + 1000);
    input.test_previous = false; Step(now + 10);
  }
};

int main() {
  drive::velocity_pid pid;
  drive::velocity_pid_config config;
  config.kP = 6; config.kI = 2;
  pid.Configure(config);
  double u = 0;
  assert(pid.Update(100, 90, 0, 1000, u) && u == 60);
  assert(pid.Update(100, 90, 0.02, 1000, u) && std::fabs(u - 60.4) < 1e-9);
  pid.Reset();
  assert(pid.Update(-100, -90, 0, 1000, u) && u == -60);
  // 长期饱和不能积累到解除饱和后还持续加压。
  pid.Reset();
  for (int i = 0; i < 1000; ++i) {
    assert(pid.Update(1000, 0, i ? 0.02 : 0, 100, u) && u == 100);
  }
  assert(pid.Update(0, 0, 0.02, 100, u) && u == 0);
  assert(!pid.Update(10, 0, 0, 100, u));
  assert(!pid.Update(10, 0, 0.3, 100, u));
  assert(!pid.Update(10, std::numeric_limits<double>::quiet_NaN(), 0.02, 100, u));
  config.kP = 0; config.kI = 0; config.kD = 1;
  pid.Configure(config);
  assert(pid.Update(100, 10, 0, 1000, u) && u == 0);
  assert(pid.Update(200, 10, 0.02, 1000, u) && u == 0); // 目标阶跃不冲击 D 项。
  assert(pid.Update(200, 12, 0.02, 1000, u) && u == -100);

  fixture f;
  assert(f.test.Info().feedforward && !f.test.Info().speed_control);
  f.input.test_positive = true; f.Step(f.now + 10);
  assert(f.output.L_voltage == 2000); // FH 仍是原前馈。
  f.input.test_positive = false; f.Step(f.now + 10);
  f.Switch();
  assert(f.test.Info().speed_control && f.test.Info().feedforward);
  f.speed.left_rpm = 90; f.speed.right_rpm = 110;
  f.input.test_positive = true; f.Step(f.now + 10);
  assert(f.output.L_voltage == 2060 && f.output.R_voltage == 1940);
  const double correction = f.test.Info().velocity.left_correction;
  f.Step(f.now + 10, false);
  assert(f.test.Info().velocity.left_correction == correction); // 同一测速不重复积分。
  f.Step(f.now + 10);
  assert(f.test.Info().velocity.left_correction > correction);
  f.Step(f.now + 101, false);
  assert(f.output.L_voltage == 0 && f.output.R_voltage == 0 && !f.test.Info().running);
  assert(std::strcmp(f.test.Info().status, "SPEED ERR") == 0);
  f.Step(f.now + 10); assert(f.output.L_voltage == 0); // 恢复新数据也需松开。
  f.input.test_positive = false; f.Step(f.now + 10);
  assert(!f.test.Info().velocity.active);
  f.speed.left_rpm = -90; f.speed.right_rpm = -110;
  f.input.test_negative = true; f.Step(f.now + 10);
  assert(f.output.L_voltage == -2060 && f.output.R_voltage == -1940);
  f.input.test_negative = false; f.Step(f.now + 10);
  f.heading = 2;
  f.speed.left_rpm = -90; f.speed.right_rpm = -110;
  f.input.test_negative = true; f.Step(f.now + 10);
  // 偏左 -> 顺时针修正；外环差速目标为左 -90、右 -110，内环不能抵消它。
  assert(f.test.Info().velocity.left_target == -90 && f.test.Info().velocity.right_target == -110);
  assert(f.output.L_voltage == -1840 && f.output.R_voltage == -2160);
  f.speed.valid = false; f.Step(f.now + 10);
  assert(f.output.L_voltage == 0 && !f.test.Info().running);
  f.speed.valid = true; f.Step(f.now + 10);
  assert(f.output.L_voltage == 0);
  f.input.test_negative = false; f.Step(f.now + 10);
  f.input.test_positive = true; f.Step(f.now + 10);
  f.Step(f.now + 3000);
  assert(f.output.L_voltage == 0 && std::strcmp(f.test.Info().status, "TIMEOUT") == 0);
  f.input.test_positive = false; f.Step(f.now + 10);
  f.Switch(); assert(!f.test.Info().feedforward && !f.test.Info().speed_control);
  f.Switch(); f.Switch();
  f.input.test_positive = true; f.Step(f.now + 10);
  f.Step(f.now + 10, true, true);
  assert(!f.test.Info().velocity.active && f.output.L_voltage == 1000);
  f.Step(f.now + 10); assert(f.output.L_voltage == 0);
  f.input.test_positive = false; f.Step(f.now + 10);
  f.input.test_negative = true; f.Step(f.now + 10);
  f.online = false; f.Step(f.now + 10);
  assert(f.output.L_voltage == 0 && !f.test.Info().velocity.active);

  // 合成一阶负载，验证闭环可修正前馈模型偏差；不代表实车精度。
  for (int sign = -1; sign <= 1; sign += 2) {
    pid.Configure(drive::velocity_pid_config());
    double rpm = 0;
    for (int i = 0; i < 3000; ++i) {
      assert(pid.Update(sign * 100, rpm, i ? 0.02 : 0, 1000, u));
      rpm += 0.1 * ((sign * 1800 + u) / 20.0 - rpm);
    }
    assert(std::fabs(rpm - sign * 100) < 0.1);
  }
  std::puts("Velocity PID checks passed: signs, anti-windup, derivative, mode cycle, stale/duplicate samples, heading cascade and cancellation.");
}
