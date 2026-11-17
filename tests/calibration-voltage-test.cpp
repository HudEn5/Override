#include "calibration-control.h"
#include <cassert>
#include <cstring>
#include <limits>
#include <cstdio>

struct test_update {
  manual::input& input;
  manual::output& output;
  calibration_control& test;
  bool operator()(double time, bool allowed = true, bool online = true, int stick = 0) {
    output = manual::output();
    output.left_mV = output.right_mV = stick;
    output.arm_enabled = true;
    output.arm_mV = 7000;
    output.claw_mV = -6000;
    output.toggle_piston = true;
    const bool owns = test.Update(input, output, allowed, online, time);
    assert(output.arm_enabled && output.arm_mV == 7000);
    assert(output.claw_mV == -6000 && output.toggle_piston);
    return owns;
  }
};

int main() {
  const int levels[3] = {2000, 3000, 4000};
  calibration_control test(levels, 3000);
  manual::input input;
  manual::output output;
  test_update update = {input, output, test};

  // 默认不接管驾驶，启动时按住运行键也不启动。
  assert(!update(0));
  calibration_control held_at_start(levels, 3000);
  input.test_positive = true;
  assert(held_at_start.Update(input, output, true, true, 0));
  assert(output.left_mV == 0 && !held_at_start.Info().running);
  input = manual::input();
  update(10);
  input.test_positive = true;
  assert(update(20));
  assert(output.left_mV == 2000 && output.right_mV == 2000);
  for (int t = 30; t < 3020; t += 10) {
    update(t);
    assert(output.left_mV == 2000 && output.right_mV == 2000);
  }
  update(3020);
  assert(output.left_mV == 0 && !test.Info().running);
  assert(std::strcmp(test.Info().status, "TIMEOUT") == 0);
  update(4000);
  assert(output.left_mV == 0);  // 超时后保持按住不能重新启动。
  input.test_positive = false;
  update(4010);

  // 上下换档仅触发一次，档位限制在 2/3/4 V。
  input.test_next = true;
  update(4020);
  assert(test.Info().millivolts == 3000);
  update(4030);
  assert(test.Info().millivolts == 3000);
  input.test_next = false;
  update(4040);
  input.test_next = true;
  update(4050);
  assert(test.Info().millivolts == 4000);
  input.test_next = false;
  update(4060);
  input.test_next = true;
  update(4070);
  assert(test.Info().millivolts == 4000);
  input.test_next = false;
  update(4080);
  input.test_negative = true;
  update(4090);
  assert(output.left_mV == -4000 && output.right_mV == -4000);
  input.test_previous = true;
  update(4100);
  assert(test.Info().millivolts == 4000 && output.left_mV == -4000);
  input.test_previous = false;
  input.test_negative = false;
  update(4110);
  assert(output.left_mV == 0 && !test.Info().running);

  input.test_previous = true;
  update(4120);
  assert(test.Info().millivolts == 4000);
  input.test_previous = false;
  update(4130);
  assert(test.Info().millivolts == 3000);
  input.test_previous = true;
  update(4140);
  assert(test.Info().millivolts == 3000);
  input.test_previous = false;
  update(4150);
  assert(test.Info().millivolts == 2000);
  input.test_previous = true;
  update(4160);
  assert(test.Info().millivolts == 2000);
  input.test_previous = false;
  update(4170);
  input.test_next = input.test_previous = true;
  update(4180);
  assert(test.Info().millivolts == 2000);
  input.test_next = input.test_previous = false;
  update(4190);

  // 两个运行键同时按下，以及不经松手直接换向，都停止并要求释放。
  input.test_positive = input.test_negative = true;
  update(4200);
  assert(output.left_mV == 0);
  input.test_negative = false;
  update(4210);
  assert(output.left_mV == 0);
  input.test_positive = false;
  update(4220);
  input.test_positive = true;
  update(4230);
  assert(output.left_mV == 2000);
  input.test_positive = false;
  input.test_negative = true;
  update(4240);
  assert(output.left_mV == 0);
  input.test_negative = false;
  update(4250);

  // 场控接入、禁用、掉线、时钟异常都停止；恢复后必须重新松手。
  input.test_positive = true;
  update(4260);
  assert(output.left_mV == 2000);
  assert(!update(4270, false));
  assert(output.left_mV == 0);
  update(4280);
  assert(output.left_mV == 0);
  input.test_positive = false;
  update(4290);
  input.test_positive = true;
  update(4300);
  update(4310, true, false);
  assert(output.left_mV == 0);
  update(4320);
  assert(output.left_mV == 0);
  input.test_positive = false;
  update(4330);
  input.test_positive = true;
  update(4340);
  update(std::numeric_limits<double>::quiet_NaN());
  assert(output.left_mV == 0);
  input.test_positive = false;
  update(4350);
  input.test_positive = true;
  update(4360);
  update(4359);
  assert(output.left_mV == 0);
  input.test_positive = false;
  update(4370);

  // 摇杆优先，松开摇杆不会恢复仍被按住的测试键。
  input.test_positive = true;
  update(4380);
  assert(!update(4390, true, true, 1000));
  assert(output.left_mV == 1000 && !test.Info().visible);
  update(4400);
  assert(output.left_mV == 0);
  input.test_positive = false;
  update(4410);
  input.test_positive = true;
  update(4420);
  input.test_start = true;
  assert(!update(4430));
  assert(output.left_mV == 0 && !test.Info().visible);
  input.test_start = false;
  update(4440);
  assert(output.left_mV == 0);
  std::puts("Fixed voltage checks passed: levels, hold, timeout, release, arbitration and faults.");
}
