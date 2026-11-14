#include "calibration-control.h"
#include <cassert>
#include <cstdio>
#include <cstring>

drive::feedforward_config Model(double kS, double kV) {
  drive::feedforward_config value;
  value.kS = kS;
  value.kV = kV;
  value.calibrated = true;
  return value;
}

struct fixture {
  int voltages[3] = {2000, 3000, 4000};
  int targets[3] = {100, 150, 200};
  calibration_control test;
  manual::Input input;
  manual::Output output;
  fixture() : test(voltages, 3000) {
    test.ConfigureFeedforward(targets, Model(340.5243, 17.516864),
                              Model(325.9312, 17.067421), 1000);
  }
  void Update(double now, bool stopped = true, bool allowed = true, bool online = true) {
    output = manual::Output();
    test.Update(input, output, allowed, online, now, stopped);
  }
  void Switch(double now) {
    input = manual::Input(); Update(now);
    input.test_previous = true; Update(now + 10);
    Update(now + 1010);
    input.test_previous = false; Update(now + 1020);
  }
};

int main() {
  fixture f;
  f.Update(0);
  f.input.test_previous = true; f.Update(10); f.Update(1009);
  assert(!f.test.Info().feedforward && f.output.L_voltage == 0);
  f.Update(1010);
  assert(f.test.Info().feedforward && f.output.L_voltage == 0);
  f.Update(4010);  // 长按只切换一次。
  assert(f.test.Info().feedforward);
  f.input.test_previous = false; f.Update(4020);
  assert(f.test.Info().selected_rpm == 100 && f.test.Info().millivolts == 2000);
  f.input.test_positive = true; f.Update(4030);
  assert(f.output.L_voltage == 2092 && f.output.R_voltage == 2033);
  assert(f.test.Info().target_rpm == 100);
  f.input.test_previous = true; f.Update(4040); f.Update(5500);
  assert(f.test.Info().feedforward && f.test.Info().target_rpm == 100);
  assert(f.output.L_voltage == 2092 && f.output.R_voltage == 2033);
  f.input.test_positive = f.input.test_previous = false; f.Update(5510);
  assert(f.output.L_voltage == 0 && f.output.R_voltage == 0 && f.test.Info().target_rpm == 0);

  f.input.test_next = true; f.Update(5520);
  assert(f.test.Info().selected_rpm == 150);
  f.input.test_next = false; f.Update(5530);
  f.input.test_negative = true; f.Update(5540);
  assert(f.output.L_voltage == -2968 && f.output.R_voltage == -2886);
  assert(f.test.Info().target_rpm == -150);
  f.input.test_negative = false; f.Update(5550);
  f.input.test_next = true; f.Update(5560);
  assert(f.test.Info().selected_rpm == 200);
  f.input.test_next = false; f.Update(5570);
  f.input.test_positive = true; f.Update(5580);
  assert(f.output.L_voltage == 3844 && f.output.R_voltage == 3739);
  f.Update(8580);
  assert(!f.test.Info().running && f.test.Info().target_rpm == 0);
  assert(f.output.L_voltage == 0 && std::strcmp(f.test.Info().status, "TIMEOUT") == 0);
  f.Update(9000); assert(f.output.L_voltage == 0);
  f.input.test_positive = false; f.Update(9010);
  f.input.test_previous = true; f.Update(9020);
  assert(f.test.Info().selected_rpm == 200);
  f.input.test_previous = false; f.Update(9030);
  assert(f.test.Info().selected_rpm == 150);
  f.Switch(9040);
  assert(!f.test.Info().feedforward && f.test.Info().millivolts == 2000);

  // 同时按上下不切模式，也不换档。
  f.input.test_next = f.input.test_previous = true; f.Update(11000); f.Update(13000);
  f.input.test_next = f.input.test_previous = false; f.Update(13010);
  assert(!f.test.Info().feedforward && f.test.Info().millivolts == 2000);

  fixture moving;
  moving.Update(0);
  moving.input.test_previous = true; moving.Update(10, false); moving.Update(2010, false);
  moving.Update(3010, true); // 按住时车停下来也不能突然切模式。
  assert(!moving.test.Info().feedforward);
  moving.input.test_previous = false; moving.Update(3020);
  moving.input.test_previous = true; moving.Update(3030);
  moving.Update(3530, false); moving.Update(4530);
  assert(!moving.test.Info().feedforward); // 中途移动取消长按手势。

  fixture startup;
  startup.input.test_previous = true; startup.Update(0); startup.Update(2000);
  assert(!startup.test.Info().feedforward);
  startup.Switch(2010);
  assert(startup.test.Info().feedforward);
  startup.input.test_positive = true; startup.Update(3040);
  startup.Update(3050, true, false);
  startup.Update(3060);
  assert(startup.output.L_voltage == 0); // 禁用后恢复不能自动启动。

  fixture conflict;
  conflict.Update(0);
  conflict.input.test_previous = true; conflict.Update(10); conflict.Update(1010);
  conflict.input.test_positive = true; conflict.Update(1020);
  assert(conflict.output.L_voltage == 0);

  fixture invalid;
  invalid.test.ConfigureFeedforward(invalid.targets, Model(1, -1), Model(1, 1), 1000);
  invalid.Switch(0);
  invalid.input.test_positive = true; invalid.Update(1030);
  assert(invalid.output.L_voltage == 0 && invalid.output.R_voltage == 0);
  assert(std::strcmp(invalid.test.Info().status, "FF ERROR") == 0);
  fixture saturated;
  saturated.test.ConfigureFeedforward(saturated.targets, Model(1000, 200), Model(1, 1), 1000);
  saturated.Switch(0);
  saturated.input.test_positive = true; saturated.Update(1030);
  assert(saturated.output.L_voltage == 0 && saturated.test.Info().target_rpm == 0);

  // 当前硬件的第二版参数：每档检查前后不同模型、符号及整数电压。
  fixture retuned;
  retuned.test.ConfigureFeedforward(retuned.targets, Model(372.573735, 17.900544),
                                    Model(408.162564, 16.537990), 1000);
  retuned.test.ConfigureReverseFeedforward(Model(547.829766, 16.939618),
                                           Model(537.033135, 15.870378));
  retuned.Switch(0);
  const int forward_left[3] = {2163, 3058, 3953};
  const int forward_right[3] = {2062, 2889, 3716};
  const int reverse_left[3] = {-2242, -3089, -3936};
  const int reverse_right[3] = {-2124, -2918, -3711};
  double now = 1030;
  for (int i = 0; i < 3; ++i) {
    retuned.input.test_positive = true; retuned.Update(now);
    assert(retuned.output.L_voltage == forward_left[i] && retuned.output.R_voltage == forward_right[i]);
    retuned.input.test_positive = false; retuned.Update(now + 10);
    retuned.input.test_negative = true; retuned.Update(now + 20);
    assert(retuned.output.L_voltage == reverse_left[i] && retuned.output.R_voltage == reverse_right[i]);
    retuned.input.test_negative = false; retuned.Update(now + 30);
    retuned.input.test_next = true; retuned.Update(now + 40);
    retuned.input.test_next = false; retuned.Update(now + 50);
    now += 60;
  }
  std::puts("Feedforward mode checks passed: long press, stop detection, targets, polarity, timeout and faults.");
}
