#include "driver-control-logic.h"

#include <cassert>

#include <cstdio>

int main() {
  manual::driver_control DriverControl(false);
  manual::Input input;
  auto update = [&]() { return DriverControl.Update(input, true, 5, 12000, 6000, 4000, 5000); };

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

  // 首次按下、直接换向和松手都在一次 Update 内生效，不存在电压渐变。
  input = manual::Input();
  input.arm_up = true;
  output = update();
  assert(output.arm_enabled && output.arm_voltage == 6000);
  input.arm_up = false;
  input.arm_down = true;
  output = update();
  assert(output.arm_voltage == -4000);
  input.arm_down = false;
  output = update();
  assert(output.arm_voltage == 0);
  input.arm_up = input.arm_down = true;
  output = update();
  assert(output.arm_voltage == 0);
  // 长按输出恒定，不受编码器角度、目标领先或堵转计数影响。
  input.arm_down = false;
  for (int i = 0; i < 1000; ++i) {
    output = update();
    assert(output.arm_voltage == 6000);
  }
  // 手臂掉线时停止，底盘和夹爪继续独立运行。
  input.arm_ready = false;
  input.y_axis = 100;
  input.claw_close = true;
  output = update();
  assert(!output.arm_enabled && output.arm_voltage == 0);
  assert(output.L_voltage == 12000 && output.R_voltage == 12000);
  assert(output.claw_voltage == 5000);
  input.arm_ready = true;
  output = update();
  assert(output.arm_enabled && output.arm_voltage == 6000);
  // 禁用立即归零所有电机指令，再启用时按当前按键恢复输出。
  output = DriverControl.Update(input, false, 5, 12000, 6000, 4000, 5000);
  assert(!output.arm_enabled && output.arm_voltage == 0);
  assert(output.L_voltage == 0 && output.R_voltage == 0 && output.claw_voltage == 0);
  output = update();
  assert(output.arm_voltage == 6000);
  input.claw_close = false;
  input.claw_open = true;
  output = update();
  assert(output.claw_voltage == -5000);
  input.claw_close = true;
  output = update();
  assert(output.claw_voltage == 0);
  // 气动按下沿、重连及初始状态保持原有语义。
  input = manual::Input();
  update();
  input.toggle = true;
  assert(update().toggle_piston);
  assert(update().toggle_piston);
  output = DriverControl.Update(input, false, 5, 12000, 6000, 4000, 5000);
  assert(output.toggle_piston);
  assert(update().toggle_piston);
  input.toggle = false;
  update();
  input.toggle = true;
  assert(!update().toggle_piston);
  manual::driver_control InitialOn(true);
  output = InitialOn.Update(input, false, 5, 12000, 6000, 4000, 5000);
  assert(output.toggle_piston);
  output = DriverControl.Update(input, true, 5, 12000, 6000, 4000, -5000);
  input.claw_close = true;
  output = DriverControl.Update(input, true, 5, 12000, 6000, 4000, -5000);
  assert(output.claw_voltage == -5000);
  // 当前配置的 12 V 上升和 6 V 下降均应原值传递，无限幅或渐变残留。
  input = manual::Input();
  input.arm_up = true;
  output = DriverControl.Update(input, true, 5, 12000, 12000, 6000, 6000);
  assert(output.arm_voltage == 12000);
  input.arm_up = false;
  input.arm_down = true;
  output = DriverControl.Update(input, true, 5, 12000, 12000, 6000, 6000);
  assert(output.arm_voltage == -6000);
  std::puts("Voltage control checks passed (including 40,401 drive combinations).");
}
