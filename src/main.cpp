#include "control-config.h"
#include "robot-config.h"
#include <stdio.h>

namespace {
struct controller_info {
  manual::Input input;
  manual::Output output;
  bool configured = false;
  bool enabled = false;
  unsigned int tick = 0;
  double dt = 0;
};

controller_info ControllerInfo;
vex::mutex controller_display_mutex;

void ControllerInfoDisplay() {
  while (true) {
    // 显示和串口通信独立运行，避免阻塞按键到电机的输出路径。
    controller_display_mutex.lock();
    const controller_info info = ControllerInfo;
    controller_display_mutex.unlock();
    const double L_angle = ArmL ? ArmL->position(vex::deg) * config::ARM_UP_DIRECTION : 0;
    const double R_angle = ArmR ? ArmR->position(vex::deg) * config::ARM_UP_DIRECTION : 0;
    const bool L_installed = ArmL && ArmL->installed();
    const bool R_installed = ArmR && ArmR->installed();
    const double L_current = ArmL ? ArmL->current(vex::amp) : 0;
    const double R_current = ArmR ? ArmR->current(vex::amp) : 0;
    // BMV 的第五个字段是指令电压 mV，与旧 BM5 的目标角字段明确区分。
    printf("BMV,%u,%d,%d,%d,%d,%.2f,%.2f,%d,%d,%.2f,%.2f\n",
           info.tick, info.input.arm_up, info.input.arm_down, info.output.arm_enabled,
           info.output.arm_voltage, L_angle, R_angle,
           L_installed, R_installed, L_current, R_current);
    fflush(stdout);
    if (Controller.installed()) {
      char lines[3][20];
      snprintf(lines[0], sizeof(lines[0]), "U%+4.1fL%5.1fR%5.1f",
               info.output.arm_voltage / 1000.0, L_angle, R_angle);
      snprintf(lines[1], sizeof(lines[1]), "1:%d 2:%d E:%d C:%d %u",
               info.input.arm_up, info.input.arm_down, info.output.arm_enabled,
               info.configured, (info.tick / 20) % 10);
      snprintf(lines[2], sizeof(lines[2]), "%s M%d%d I%.1f/%.1f",
               config::CONTROLLER_VERSION, L_installed, R_installed, L_current, R_current);
      for (int row = 0; row < 3; ++row) {
        Controller.Screen.setCursor(row + 1, 1);
        Controller.Screen.print("%-19s", lines[row]);
      }
    }
    int left_count, right_count;
    hardware::GetDriveMotorCount(left_count, right_count);
    // 保留初始化错误所在的第二行。
    Brain.Screen.printAt(10, 60, "CFG=%d PAD=%d RUN=%d             ",
                         info.configured, Controller.installed(), info.enabled);
    Brain.Screen.printAt(10, 85, "A3=%4d A1=%4d Tick=%u         ",
                         info.input.y_axis, info.input.angular_axis, info.tick);
    Brain.Screen.printAt(10, 110, "L=%6d R=%6d mV              ",
                         info.output.L_voltage, info.output.R_voltage);
    Brain.Screen.printAt(10, 135, "FLD=%d SW=%d EN=%d DRV=%d AUTO=%d    ",
                         Competition.isFieldControl(), Competition.isCompetitionSwitch(),
                         Competition.isEnabled(), Competition.isDriverControl(),
                         Competition.isAutonomous());
    Brain.Screen.printAt(10, 160, "Motors L=%d/4 R=%d/4         ", left_count, right_count);
    Brain.Screen.printAt(10, 185, "U=%5d AL=%5.1f AR=%5.1f      ",
                         info.output.arm_voltage, L_angle, R_angle);
    Brain.Screen.printAt(10, 210, "R1=%d R2=%d ARM=%d dt=%4.0fms     ",
                         info.input.arm_up, info.input.arm_down, info.output.arm_enabled, info.dt);
    Brain.Screen.printAt(10, 235, true, "VOLTAGE MODE  %s             ",
                         info.output.arm_voltage ? "RUN" : "HOLD");
    vex::wait(200, vex::msec);
  }
}
}  // 匿名命名空间

static_assert(config::YAxis >= 1 && config::YAxis <= 4, "Forward axis must be 1..4");
static_assert(config::AngularAxis >= 1 && config::AngularAxis <= 4, "Steering axis must be 1..4");
static_assert(config::JOYSTICK_DEADBAND >= 0 && config::JOYSTICK_DEADBAND < 100,
              "Joystick deadband must be 0..99");
static_assert(config::MAX_VOLTAGE > 0 && config::MAX_VOLTAGE <= 12000,
              "Drive voltage must be 1..12000 mV");
static_assert(config::UPDATE_TIME > 0, "Control period must be positive");
static_assert(config::ARM_UP_VOLTAGE > 0 && config::ARM_UP_VOLTAGE <= 12000 &&
              config::ARM_DOWN_VOLTAGE > 0 && config::ARM_DOWN_VOLTAGE <= 12000,
              "Arm voltages must be 1..12000 mV");
static_assert(config::CLAW_VOLTAGE > 0 && config::CLAW_VOLTAGE <= 12000,
              "Claw voltage must be 1..12000 mV");
static_assert(config::ARM_UP_DIRECTION == 1 || config::ARM_UP_DIRECTION == -1,
              "Arm direction must be 1 or -1");
static_assert(config::CLAW_CLOSE_DIRECTION == 1 || config::CLAW_CLOSE_DIRECTION == -1,
              "Claw direction must be 1 or -1");

int main() {
  Brain.Screen.print("Bartmoss v%s", config::PROGRAM_VERSION);
  const bool configured = hardware::Initialize();
  if (!configured) {
    Controller.Screen.print("Configure ports");
  } else {
    Brain.Screen.clearScreen();
    Brain.Screen.print("Bartmoss v%s", config::PROGRAM_VERSION);
    Controller.Screen.print("Voltage control ready");
  }
  manual::driver_control DriverControl(config::PISTON_INITIAL_STATE);
  vex::thread controller_display(ControllerInfoDisplay);
  vex::timer ControlTimer;
  double pre_time = ControlTimer.time(vex::msec);
  unsigned int loop_count = 0;
  while (true) {
    const double current_time = ControlTimer.time(vex::msec);
    const double dt = current_time - pre_time;
    pre_time = current_time;
    const bool competition_connected =
        Competition.isFieldControl() || Competition.isCompetitionSwitch();
    const bool driver_period = !competition_connected ||
        (Competition.isEnabled() && Competition.isDriverControl());
    const bool enabled = configured && driver_period && Controller.installed();
    const manual::Input input = hardware::ReadInput();
    const manual::Output output = DriverControl.Update(
        input, enabled, config::JOYSTICK_DEADBAND, config::MAX_VOLTAGE,
        config::ARM_UP_VOLTAGE, config::ARM_DOWN_VOLTAGE,
        config::CLAW_VOLTAGE * config::CLAW_CLOSE_DIRECTION);
    hardware::Apply(output);
    // 控制循环仅发布快照，不执行屏幕打印和串口等待。
    controller_display_mutex.lock();
    ControllerInfo.input = input;
    ControllerInfo.output = output;
    ControllerInfo.configured = configured;
    ControllerInfo.enabled = enabled;
    ControllerInfo.tick = loop_count;
    ControllerInfo.dt = dt;
    controller_display_mutex.unlock();
    ++loop_count;
    vex::wait(config::UPDATE_TIME, vex::msec);
  }
}
