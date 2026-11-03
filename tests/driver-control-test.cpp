#include "driver-control-logic.h"
#include "arm-pid.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
  manual::driver_control DriverControl(false);
  manual::Input input;
  auto update = [&]() { return DriverControl.Update(input, true, 5, 12000, 0.6, 5000, 0.0, 90.0); };

  // 检查直线前进、后退，以及顺时针和逆时针原地转向。
  input.y_axis = 100;
  auto output = update();
  assert(output.L_voltage == 12000 && output.R_voltage == 12000);
  input.y_axis = -100;
  output = update();
  assert(output.L_voltage == -12000 && output.R_voltage == -12000);
  input.y_axis = 0;
  input.angular_axis = 100;
  output = update();
  assert(output.L_voltage == 12000 && output.R_voltage == -12000);
  input.angular_axis = -100;
  output = update();
  assert(output.L_voltage == -12000 && output.R_voltage == 12000);

  // 同时前进和转向时，即使一侧达到上限，也应保持转向比例。
  input.y_axis = 100;
  input.angular_axis = 100;
  output = update();
  assert(output.L_voltage == 12000 && output.R_voltage == 0);
  input.angular_axis = 50;
  output = update();
  assert(output.L_voltage == 12000);
  assert(output.R_voltage > 4000 && output.R_voltage < 4500);
  input.y_axis = -100;
  input.angular_axis = -100;
  output = update();
  assert(output.L_voltage == -12000 && output.R_voltage == 0);

  // 检查死区、输入限幅，以及所有摇杆组合的输出范围。
  input.y_axis = 5;
  input.angular_axis = -5;
  output = update();
  assert(output.L_voltage == 0 && output.R_voltage == 0);
  assert(manual::AxisVoltage(6, 5, 12000) > 0);
  assert(manual::AxisVoltage(150, 5, 12000) == 12000);
  for (int y_axis = -100; y_axis <= 100; ++y_axis) {
    for (int angular_axis = -100; angular_axis <= 100; ++angular_axis) {
      input.y_axis = y_axis;
      input.angular_axis = angular_axis;
      output = update();
      assert(output.L_voltage >= -12000 && output.L_voltage <= 12000);
      assert(output.R_voltage >= -12000 && output.R_voltage <= 12000);
    }
  }

  // 按住增减目标，松开或同时按下保持目标，夹爪独立运行。
  input = manual::Input();
  input.arm_up = input.claw_close = true;
  output = update();
  assert(output.arm_enabled && output.arm_target_angle > 0.59 && output.arm_target_angle < 0.61);
  assert(output.claw_voltage == 5000);
  const double first_target = output.arm_target_angle;
  input.arm_up = false;
  input.arm_angle = 0.1;
  output = update();
  assert(output.arm_target_angle == first_target);
  input.arm_up = input.arm_down = true;
  output = update();
  assert(output.arm_target_angle == first_target);
  input.arm_up = false;
  input.claw_close = false;
  input.claw_open = true;
  output = update();
  assert(output.arm_target_angle == 0.0 && output.claw_voltage == -5000);
  input.claw_close = true;
  output = update();
  assert(output.claw_voltage == 0);

  // 持续按住不会越过 0 到 90 度；目标不跟随每次反馈重新归零。
  // 模拟编码器跟随目标移动，否则堵转保护会按预期冻结目标。
  input = manual::Input();
  input.arm_up = true;
  for (int i = 0; i < 200; ++i) {
    output = update();
    input.arm_angle = output.arm_target_angle;
  }
  assert(output.arm_target_angle == 90.0);
  input.arm_up = false;
  input.arm_down = true;
  for (int i = 0; i < 200; ++i) {
    output = update();
    input.arm_angle = output.arm_target_angle;
  }
  assert(output.arm_target_angle == 0.0);

  // 禁用停止位置运动；重连后跟随当前位置，不继续追赶旧目标。
  input = manual::Input();
  input.arm_angle = 35.0;
  input.y_axis = input.angular_axis = 100;
  input.arm_up = input.claw_close = true;
  output = DriverControl.Update(input, false, 5, 12000, 0.6, 5000, 0, 90);
  assert(!output.arm_enabled && output.arm_target_angle == 35.0);
  assert(output.L_voltage == 0 && output.R_voltage == 0 && output.claw_voltage == 0);
  input.arm_angle = 20.0;
  input.arm_up = false;
  output = update();
  assert(output.arm_enabled && output.arm_target_angle == 20.0);

  // 起始反馈超出软件范围时，目标仍限于设置的范围。
  manual::driver_control Startup(false);
  input = manual::Input();
  input.arm_angle = 120;
  output = Startup.Update(input, true, 5, 12000, 0.6, 5000, 0, 90);
  assert(output.arm_target_angle == 90);
  output = Startup.Update(input, false, 5, 12000, 0.6, 5000, 0, 90);
  input.arm_angle = -20;
  output = Startup.Update(input, true, 5, 12000, 0.6, 5000, 0, 90);
  assert(output.arm_target_angle == 0);

  // 自定义角度上限和按住速率同样生效。
  manual::driver_control Custom(false);
  input = manual::Input();
  input.arm_up = true;
  for (int i = 0; i < 100; ++i) {
    output = Custom.Update(input, true, 5, 12000, 2.0, 5000, 0, 45);
    input.arm_angle = output.arm_target_angle;
  }
  assert(output.arm_target_angle == 45);

  // 关闭上限后目标可以超过 90 度，松开和重连时也不会被截回 90 度。
  manual::driver_control Unbounded(false);
  input = manual::Input();
  input.arm_up = true;
  for (int i = 0; i < 200; ++i) {
    output = Unbounded.Update(input, true, 5, 12000, 1.0, 5000, 0, 90, false);
    input.arm_angle = output.arm_target_angle;
  }
  assert(output.arm_target_angle == 200);
  assert(manual::LimitAngle(output.arm_target_angle, 0, 90, false) == 200);
  input.arm_up = false;
  output = Unbounded.Update(input, true, 5, 12000, 1.0, 5000, 0, 90, false);
  assert(output.arm_target_angle == 200);
  input.arm_angle = 135;
  output = Unbounded.Update(input, false, 5, 12000, 1.0, 5000, 0, 90, false);
  assert(output.arm_target_angle == 135 && !output.arm_enabled);
  output = Unbounded.Update(input, true, 5, 12000, 1.0, 5000, 0, 90, false);
  assert(output.arm_target_angle == 135);
  input.arm_down = true;
  for (int i = 0; i < 200; ++i) {
    output = Unbounded.Update(input, true, 5, 12000, 1.0, 5000, 0, 90, false);
    input.arm_angle = output.arm_target_angle;
  }
  assert(output.arm_target_angle == 0);

  // 堵转保护：编码器不动时目标最多领先实测 5°，随后冻结在实测位置。
  manual::driver_control Jammed(false);
  input = manual::Input();
  input.arm_up = true;  // 实测始终为 0，模拟手臂被卡住
  for (int i = 0; i < 200; ++i) {
    output = Jammed.Update(input, true, 5, 12000, 0.6, 5000, 0, 90, false);
    assert(output.arm_target_angle <= manual::DEFAULT_ARM_MAX_LEAD);
  }
  assert(output.arm_target_angle == 0.0);
  // 松开后重新按下：堵转锁存清除，从实测位置继续。
  input.arm_up = false;
  output = Jammed.Update(input, true, 5, 12000, 0.6, 5000, 0, 90, false);
  assert(output.arm_target_angle == 0.0);
  input.arm_up = true;
  output = Jammed.Update(input, true, 5, 12000, 0.6, 5000, 0, 90, false);
  assert(output.arm_target_angle > 0.0 && output.arm_target_angle < 0.61);

  // 累计方向位移：每周期仅移动 0.02°，连续升降也不应误触发堵转。
  for (int direction = -1; direction <= 1; direction += 2) {
    manual::driver_control Slow(false);
    input = manual::Input();
    input.arm_angle = 50;
    input.arm_up = direction > 0;
    input.arm_down = direction < 0;
    for (int i = 0; i < 200; ++i) {
      input.arm_angle += direction * .02;
      output = Slow.Update(input, true, 5, 12000, 1.2, 5000, 0, 90, false, 10);
      assert((output.arm_target_angle - input.arm_angle) * direction > .5);
      assert(std::fabs(output.arm_target_angle - input.arm_angle) <= 10 + 1e-8);
    }
  }
  // 反向移动不能算有效进展；堵转锁存后目标固定，不随实测继续漂移。
  manual::driver_control WrongDirection(false);
  input = manual::Input();
  input.arm_up = true;
  input.arm_angle = 30;
  for (int i = 0; i < 50; ++i) {
    input.arm_angle -= .1;
    output = WrongDirection.Update(input, true, 5, 12000, 1.2, 5000, 0, 90, false, 10);
  }
  const double stalled_target = output.arm_target_angle;
  assert(stalled_target == input.arm_angle);
  input.arm_angle -= 1;
  output = WrongDirection.Update(input, true, 5, 12000, 1.2, 5000, 0, 90, false, 10);
  assert(output.arm_target_angle == stalled_target);
  input.arm_up = false;
  input.arm_down = true;
  output = WrongDirection.Update(input, true, 5, 12000, 1.2, 5000, 0, 90, false, 10);
  assert(std::fabs(output.arm_target_angle - input.arm_angle + 1.2) < 1e-8);

  // 假设编码器准确跟随上一周期的平滑目标，检查完整目标链可双向达到 60°/s。
  // 此处验证软件限制，不模拟机械负载，不能据此认定实机速度已翻倍。
  for (int direction = -1; direction <= 1; direction += 2) {
    manual::driver_control Fast(false);
    manual::arm_target_filter Target;
    input = manual::Input();
    input.arm_angle = 120;
    input.arm_up = direction > 0;
    input.arm_down = direction < 0;
    Target.Reset(input.arm_angle);
    for (int i = 0; i < 80; ++i) {
      output = Fast.Update(input, true, 5, 12000, 60 * .02, 5000, 0, 90, false, 10);
      const double next = Target.Update(output.arm_target_angle, .02, 10 * 6.0, .06);
      const double step = (next - input.arm_angle) * direction;
      assert(step > 0 && step <= 1.2 + 1e-8);
      if (i > 60) assert(std::fabs(step - 1.2) < 1e-6);
      input.arm_angle = next;
    }
  }

  // 单侧掉线或两侧位置差超限时手臂停止；恢复后跟随实测位置重新起步。
  manual::driver_control Ready(false);
  input = manual::Input();
  input.arm_angle = 40.0;
  input.arm_ready = false;
  input.arm_up = true;
  output = Ready.Update(input, true, 5, 12000, 0.6, 5000, 0, 90);
  assert(!output.arm_enabled && output.arm_target_angle == 40.0);
  input.arm_ready = true;
  input.arm_angle = 37.0;
  input.arm_up = false;
  output = Ready.Update(input, true, 5, 12000, 0.6, 5000, 0, 90);
  assert(output.arm_enabled && output.arm_target_angle == 37.0);

  // X 按下沿切换，禁用和重新启用期间保持气动状态。
  input = manual::Input();
  update();
  input.toggle = true;
  output = update();
  assert(output.toggle_piston);
  output = update();
  assert(output.toggle_piston);
  output = DriverControl.Update(input, false, 5, 12000, 0.6, 5000, 0, 90);
  assert(output.toggle_piston && !output.arm_enabled);
  output = update();
  assert(output.toggle_piston);
  input.toggle = false;
  update();
  input.toggle = true;
  output = update();
  assert(!output.toggle_piston);
  manual::driver_control InitialOn(true);
  output = InitialOn.Update(input, false, 5, 12000, 0.6, 5000, 0, 90);
  assert(output.toggle_piston);

  // 夹爪反向参数不影响手臂目标角度。
  input = manual::Input();
  input.claw_close = true;
  output = DriverControl.Update(input, true, 5, 12000, 0.6, -5000, 0, 90);
  assert(output.claw_voltage == -5000);
  std::puts("Angle control checks passed (including 40,401 drive combinations).");
}
