#include "control-config.h"
#include "robot-config.h"
#include <stdio.h>

namespace {
struct controller_info {
  manual::Input input;
  manual::Output output;
  bool configured = false;
  unsigned int tick = 0;
};

controller_info ControllerInfo;
vex::mutex controller_display_mutex;

void ControllerInfoDisplay() {
  while (true) {
    // 只在复制数据时持锁；屏幕和串口通信都在独立线程中完成。
    controller_display_mutex.lock();
    const controller_info info = ControllerInfo;
    controller_display_mutex.unlock();

    const double L_angle = ArmL ? ArmL->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0;
    const double R_angle = ArmR ? ArmR->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0;
    const bool L_installed = ArmL && ArmL->installed();
    const bool R_installed = ArmR && ArmR->installed();
    const double L_current = ArmL ? ArmL->current(vex::amp) : 0.0;
    const double R_current = ArmR ? ArmR->current(vex::amp) : 0.0;
    // 回传主控程序实际采样的数据，而不是电脑游戏手柄接口的数据。
    printf("BM5,%u,%d,%d,%d,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f\n",
           info.tick, info.input.arm_up, info.input.arm_down, info.output.arm_enabled,
           info.output.arm_target_angle, L_angle, R_angle,
           L_installed, R_installed, L_current, R_current);
    fflush(stdout);

    if (Controller.installed()) {
      // 每行最多 19 个字符，固定宽度覆盖旧内容，不反复清屏。
      char lines[3][20];
      snprintf(lines[0], sizeof(lines[0]), "T%4.1fL%5.1fR%5.1f",
                    info.output.arm_target_angle, L_angle, R_angle);
      snprintf(lines[1], sizeof(lines[1]), "1:%d 2:%d E:%d C:%d %u",
                    info.input.arm_up, info.input.arm_down, info.output.arm_enabled,
                    info.configured, (info.tick / 10) % 10);
      snprintf(lines[2], sizeof(lines[2]), "%s M%d%d I%.1f/%.1f",
                    config::CONTROLLER_VERSION, L_installed, R_installed, L_current, R_current);
      for (int row = 0; row < 3; ++row) {
        Controller.Screen.setCursor(row + 1, 1);
        Controller.Screen.print("%-19s", lines[row]);
      }
    }
    vex::wait(200, vex::msec);
  }
}
}  // 匿名命名空间

static_assert(config::YAxis >= 1 && config::YAxis <= 4,
              "Forward joystick axis must be 1..4");
static_assert(config::AngularAxis >= 1 && config::AngularAxis <= 4,
              "Steering joystick axis must be 1..4");
static_assert(config::JOYSTICK_DEADBAND >= 0 && config::JOYSTICK_DEADBAND < 100,
              "Joystick deadband must be 0..99");
static_assert(config::MAX_VOLTAGE > 0 && config::MAX_VOLTAGE <= 12000,
              "Drive voltage must be 1..12000 mV");
static_assert(config::UPDATE_TIME > 0, "Control period must be positive");
static_assert(config::ARM_MIN_ANGLE == 0 &&
              (!config::ARM_UPPER_LIMIT_ENABLED || config::ARM_MAX_ANGLE > config::ARM_MIN_ANGLE),
              "Arm range must start at zero and have a positive upper bound");
static_assert(config::ARM_TARGET_SPEED > 0, "Arm target speed must be positive");
static_assert(config::ARM_MAX_VELOCITY > 0,
              "Filtered arm target velocity must be positive");
static_assert(config::ARM_TARGET_FILTER_TIME >= 0, "Target filter time cannot be negative");
static_assert(config::ARM_PID.kp >= 0 && config::ARM_PID.ki >= 0 && config::ARM_PID.kd >= 0,
              "Arm PID gains cannot be negative");
static_assert(config::ARM_PID.max_voltage > 0 && config::ARM_PID.max_voltage <= 12000,
              "Arm PID voltage must be 1..12000 mV");
static_assert(config::ARM_PID.voltage_slew > 0 && config::ARM_PID.integral_voltage_max >= 0 &&
              config::ARM_PID.integral_zone >= 0 && config::ARM_PID.D_filter_time >= 0,
              "Invalid arm PID limits or filter settings");
static_assert(config::ARM_MAX_LEAD > 0, "Arm max lead must be positive");
static_assert(config::ARM_STALL_UPDATES > 0, "Arm stall threshold must be positive");
static_assert(config::ARM_MAX_DIFF > 0, "Arm max position difference must be positive");
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
    Controller.Screen.print("Driver control ready");
  }

  manual::driver_control DriverControl(config::PISTON_INITIAL_STATE);
  // 启动后遥控器屏幕由显示线程统一写入，避免通信拖慢手控循环。
  vex::thread controller_display(ControllerInfoDisplay);
  unsigned int loop_count = 0;
  vex::timer ControlTimer;
  double pre_time = ControlTimer.time(vex::msec);
  // 用同一个循环统一管理所有电机和气动输出，禁用时也持续更新。
  // 不使用独立回调或自动线程，避免旧指令在模式切换后继续执行。
  while (true) {
    const double current_time = ControlTimer.time(vex::msec);
    double dt = current_time - pre_time;
    pre_time = current_time;
    // 首次循环使用一个周期；长时间调试暂停后限制单次目标跳变量。
    if (loop_count == 0) dt = config::UPDATE_TIME;
    if (dt > 100) dt = 100;
    const bool field_control = Competition.isFieldControl();
    const bool competition_switch = Competition.isCompetitionSwitch();
    const bool competition_enabled = Competition.isEnabled();
    const bool driver_period = Competition.isDriverControl();
    const bool autonomous_period = Competition.isAutonomous();
    const bool controller_connected = Controller.installed();
    const bool competition_connected = field_control || competition_switch;
    const bool driver_control_period = !competition_connected ||
        (competition_enabled && driver_period);
    const bool enabled = configured && driver_control_period && controller_connected;
    const manual::Input input = hardware::ReadInput();
    const manual::Output output = DriverControl.Update(
        input, enabled, config::JOYSTICK_DEADBAND, config::MAX_VOLTAGE,
        config::ARM_TARGET_SPEED * dt / 1000.0,
        config::CLAW_VOLTAGE * config::CLAW_CLOSE_DIRECTION,
        config::ARM_MIN_ANGLE, config::ARM_MAX_ANGLE, config::ARM_UPPER_LIMIT_ENABLED,
        config::ARM_MAX_LEAD, config::ARM_STALL_UPDATES);
    hardware::Apply(output, dt / 1000.0);

    // 每约 200 ms 更新诊断，不改变输出的启用条件。
    // 保留配置错误所在的第二行，方便辨认初始化是否成功。
    if (loop_count % 10 == 0) {
      controller_display_mutex.lock();
      ControllerInfo.input = input;
      ControllerInfo.output = output;
      ControllerInfo.configured = configured;
      ControllerInfo.tick = loop_count;
      controller_display_mutex.unlock();
      int left_count, right_count;
      hardware::GetDriveMotorCount(left_count, right_count);
      const char* status = !configured ? "CONFIG ERROR" :
          !controller_connected ? "NO CONTROLLER" :
          !driver_control_period ? "MATCH BLOCKED" : "MANUAL ACTIVE";
      Brain.Screen.printAt(10, 60, "CFG=%d PAD=%d RUN=%d   %s        ",
                           configured, controller_connected, enabled, status);
      Brain.Screen.printAt(10, 85, "A3=%4d A1=%4d  Tick=%u        ",
                           input.y_axis, input.angular_axis, loop_count);
      Brain.Screen.printAt(10, 110, "L=%6d R=%6d mV       ",
                           output.L_voltage, output.R_voltage);
      Brain.Screen.printAt(10, 135, "FLD=%d SW=%d EN=%d DRV=%d AUTO=%d     ",
                           field_control, competition_switch, competition_enabled,
                           driver_period, autonomous_period);
      Brain.Screen.printAt(10, 160, "Motors L=%d/4 R=%d/4       ", left_count, right_count);
      Brain.Screen.printAt(10, 185, "Tgt=%5.1f AL=%5.1f AR=%5.1f deg   ",
                           output.arm_target_angle,
                           ArmL ? ArmL->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0,
                           ArmR ? ArmR->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0);
      // 直接显示按键采样和手臂使能，区分没有收到输入与电机未跟随。
      Brain.Screen.printAt(10, 210, "R1=%d R2=%d ARM=%d dt=%4.0fms      ",
                           input.arm_up, input.arm_down, output.arm_enabled, dt);
      double filtered_target, L_pid_voltage, R_pid_voltage;
      hardware::GetArmPIDInfo(filtered_target, L_pid_voltage, R_pid_voltage);
      Brain.Screen.printAt(10, 235, "S=%.1f VL=%.0f VR=%.0f mV       ",
                           filtered_target, L_pid_voltage, R_pid_voltage);
    }
    ++loop_count;
    vex::wait(config::UPDATE_TIME, vex::msec);
  }
}
