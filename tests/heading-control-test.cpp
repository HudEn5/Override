#include "voltage-test.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

drive::feedforward_config Model() {
  drive::feedforward_config config;
  config.kS = 400;
  config.kV = 16;
  config.calibrated = true;
  return config;
}

struct fixture {
  int voltages[3] = {2000, 3000, 4000};
  int targets[3] = {100, 150, 200};
  voltage_test test;
  manual::Input input;
  manual::Output output;
  fixture() : test(voltages, 3000) {
    test.ConfigureFeedforward(targets, Model(), Model(), 1000);
    drive::heading_config heading;
    heading.kD = 0;  // 单独验证比例修正符号；下面另测微分。
    test.ConfigureHeading(heading);
    Update(0);
    input.voltage_down = true; Update(10); Update(1010);
    input.voltage_down = false; Update(1020);
  }
  void Update(double now, double angle = 10, bool ready = true, bool enabled = true,
              bool joystick = false) {
    output = manual::Output();
    if (joystick) output.L_voltage = output.R_voltage = 1000;
    test.Update(input, output, enabled, true, now, true, ready, angle);
  }
};

int main() {
  fixture f;
  assert(!f.test.Info().heading.locked);
  f.input.voltage_forward = true; f.Update(1030);
  assert(f.output.L_voltage == 2000 && f.output.R_voltage == 2000);
  assert(f.test.Info().heading.target == 10);
  f.Update(1040, 12); // 车头偏左，修正应顺时针：左加、右减。
  assert(f.output.L_voltage == 2160 && f.output.R_voltage == 1840);
  f.input.voltage_forward = false; f.Update(1050, 13);
  assert(f.output.L_voltage == 0 && f.output.R_voltage == 0);
  assert(f.test.Info().heading.locked && !f.test.Info().heading.active);
  f.Update(1060, 14); // 停车偏转不能改写参考角，更不能主动旋转。
  assert(f.test.Info().heading.target == 10 && f.output.L_voltage == 0);
  f.input.telemetry = true; f.Update(1065, 14);
  assert(f.test.Info().heading.target == 10); // 回传开关不能重新锁定方向。
  f.input.telemetry = false;
  f.input.voltage_reverse = true; f.Update(1070, 14);
  assert(f.output.L_voltage == -1680 && f.output.R_voltage == -2320);
  assert(f.test.Info().heading.target == 10); // 后退沿用同一转向符号。
  f.input.voltage_reverse = false; f.Update(1080, 9);
  f.input.voltage_up = true; f.Update(1090, 9);
  f.input.voltage_up = false; f.Update(1100, 9);
  f.input.voltage_forward = true; f.Update(1110, 9);
  assert(f.output.L_voltage == 2720 && f.output.R_voltage == 2880);
  assert(f.test.Info().heading.target == 10); // 换速度档也保留方向。
  f.Update(1120, 0);
  assert(f.output.L_voltage == 2300 && f.output.R_voltage == 3300); // 限幅 500 mV。
  f.Update(1130, -6);
  assert(f.output.L_voltage == 0 && !f.test.Info().running);
  assert(std::strcmp(f.test.Info().status, "HDG ERROR") == 0);
  assert(f.test.Info().heading.target == 10); // 超限不能自动重定向掩盖偏差。
  f.Update(1140, 10); assert(f.output.L_voltage == 0); // 必须松开再试。
  f.input.voltage_forward = false; f.Update(1150);
  f.input.voltage_forward = true; f.Update(1160);
  assert(f.test.Info().running);
  f.Update(4160); assert(f.output.L_voltage == 0); // 3 秒限时不受航向修正影响。
  f.input.voltage_forward = false; f.Update(4170);
  f.Update(4180, 20, true, true, true);
  assert(!f.test.Info().heading.locked && !f.test.Info().visible);
  f.input.voltage_reverse = true; f.Update(4190, 20);
  assert(f.output.L_voltage == 0); // 摇杆接管后仍需松开运行键。
  f.input.voltage_reverse = false; f.Update(4200, 20);
  f.input.voltage_reverse = true; f.Update(4210, 20);
  assert(f.test.Info().heading.target == 20);
  f.Update(4220, 20, false);
  assert(f.output.L_voltage == 0 && !f.test.Info().heading.locked);
  f.Update(4230, 20); assert(f.output.L_voltage == 0); // 惯导恢复不自动重启。
  f.input.voltage_reverse = false; f.Update(4240);
  f.input.voltage_reverse = true; f.Update(4250);
  assert(f.test.Info().running);
  f.Update(4260, 10, true, false);
  assert(!f.test.Info().heading.locked);
  f.Update(4270); assert(f.output.L_voltage == 0);

  fixture modes;
  modes.input.voltage_forward = true; modes.Update(1030);
  modes.input.voltage_forward = false; modes.Update(1040);
  modes.input.voltage_down = true; modes.Update(1050); modes.Update(2050);
  assert(!modes.test.Info().feedforward && !modes.test.Info().heading.locked);
  modes.input.voltage_down = false; modes.Update(2060);
  modes.input.voltage_forward = true; modes.Update(2070, 30, false);
  assert(modes.output.L_voltage == 2000); // 固定电压仍不依赖惯导。

  drive::heading_hold pd;
  int left = 2000, right = 2000;
  assert(pd.Update(true, true, 0, 0, left, right));
  left = right = 2000;
  assert(pd.Update(true, true, 1, 10, left, right));
  assert(left > 2080 && right < 1920); // 微分抑制继续向左转动。
  assert(left <= 2500 && right >= 1500);
  left = right = -2000;
  assert(pd.Update(true, true, 1, 20, left, right));
  assert(left > -2000 && right < -2000);
  assert(!pd.Update(true, true, std::numeric_limits<double>::quiet_NaN(), 30, left, right));
  assert(!pd.Info().ready && !pd.Info().locked);
  std::puts("Heading checks passed: persistent forward/reverse reference, passive stop, PD signs, limits, timeout and recovery.");
}
