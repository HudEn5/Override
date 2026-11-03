#include "arm-pid.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
  const manual::ArmPIDConfig settings = {600, 40, 20, 6000, 12000, 1500, 5, .08};
  manual::arm_pid PID(settings);
  PID.Reset(0);
  double previous = 0;
  // 不同周期和正反向误差下，输出和变化率都不能超限。
  for (int i = 0; i < 1000; ++i) {
    const double dt = i % 2 ? .02 : .035;
    const double voltage = PID.Update(i < 500 ? 100 : -100, 0, dt);
    assert(std::isfinite(voltage));
    assert(std::fabs(voltage) <= 6000);
    assert(std::fabs(voltage - previous) <= 12000 * dt + 1e-8);
    previous = voltage;
  }
  assert(previous < 0);
  assert(PID.Update(0, 0, 0) == previous);

  // 重置清空积分、微分和上次输出，重新使能时从零平滑起步。
  PID.Reset(40);
  assert(PID.Update(40, 40, .02) == 0);
  assert(PID.Update(45, 40, .02) > 0);
  PID.Reset(40);
  assert(PID.Update(35, 40, .02) < 0);

  // 微分只跟随测量值，改变目标不会产生微分冲击。
  manual::arm_pid D_PID({0, 0, 20, 6000, 1000000, 0, 0, .08});
  D_PID.Reset(10);
  assert(D_PID.Update(90, 10, .02) == 0);
  assert(D_PID.Update(90, 11, .02) < 0);

  // 积分有上限，输出饱和时不继续积累；解除饱和后能够反向响应。
  manual::arm_pid I_PID({0, 1000, 0, 1000, 1000000, 500, 100, 0});
  I_PID.Reset(0);
  for (int i = 0; i < 100; ++i) previous = I_PID.Update(10, 0, .02);
  assert(previous == 500);
  for (int i = 0; i < 100; ++i) previous = I_PID.Update(-10, 0, .02);
  assert(previous == -500);
  manual::arm_pid Saturated({1000, 1000, 0, 1000, 1000000, 500, 100, 0});
  Saturated.Reset(0);
  for (int i = 0; i < 1000; ++i) Saturated.Update(10, 0, .02);
  assert(Saturated.Update(0, 0, .02) == 0);

  // 两侧独立测量但跟踪同一目标，各自纠正本侧误差。
  manual::arm_pid Left(settings), Right(settings);
  Left.Reset(29);
  Right.Reset(31);
  assert(Left.Update(30, 29, .02) > 0);
  assert(Right.Update(30, 31, .02) < 0);

  // 目标滤波不跳变、不超调，并限制每周期的目标移动量。
  manual::arm_target_filter Target;
  Target.Reset(0);
  previous = 0;
  for (int i = 0; i < 500; ++i) {
    const double value = Target.Update(90, .02, 30, .12);
    assert(value >= previous && value <= 90);
    assert(value - previous <= .6 + 1e-8);
    previous = value;
  }
  assert(previous > 89.99);
  for (int i = 0; i < 500; ++i) {
    const double value = Target.Update(0, .02, 30, .12);
    assert(value <= previous && value >= 0);
    assert(previous - value <= .6 + 1e-8);
    previous = value;
  }
  assert(previous < .01);
  Target.Reset(150);
  assert(Target.Update(150, .02, 30, .12) == 150);
  assert(Target.Update(0, 0, 30, .12) == 150);
  // 提速参数下仍保持单调、无超调，正反向每周期最多移动 1.2°。
  for (int direction = -1; direction <= 1; direction += 2) {
    Target.Reset(0);
    previous = 0;
    for (int i = 0; i < 500; ++i) {
      const double value = Target.Update(direction * 90, .02, 60, .06);
      const double progress = (value - previous) * direction;
      assert(progress >= 0 && progress <= 1.2 + 1e-8);
      assert(value * direction <= 90);
      previous = value;
    }
    assert(std::fabs(previous - direction * 90) < .01);
  }
  std::puts("Arm PID checks passed: filtering, derivative, anti-windup, limits and reset.");
}
