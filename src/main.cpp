#include "control-config.h"
#include "robot-config.h"
#include "turn-test.h"
#include "drive-sampling.h"
#include "voltage-test.h"
#include <stdio.h>

namespace {
struct controller_info {
  manual::Input input;
  manual::Output output;
  bool configured = false;
  bool enabled = false;
  bool telemetry = false;
  unsigned int tick = 0;
  double dt = 0;
  turn_test_info turn;
  voltage_test_info voltage;
};

controller_info ControllerInfo;
vex::mutex controller_display_mutex;

void DriveTelemetry() {
  while (true) {
    drive::sample value;
    unsigned int dropped;
    if (drive::PopSample(value, dropped)) {
      drive::PrintSample(value, dropped);
      controller_display_mutex.lock();
      const voltage_test_info voltage = ControllerInfo.voltage;
      controller_display_mutex.unlock();
      if (voltage.visible && voltage.feedforward && voltage.heading_enabled) {
        const drive::heading_info& heading = voltage.heading;
        // 航向来自主循环快照；时间戳是惯导读取时刻，不冒充速度采样时刻。
        printf("DHD,%u,%.1f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d\n", value.sequence,
               heading.time_ms, heading.angle, heading.target, heading.error,
               heading.correction, heading.locked, heading.ready, heading.active);
        fflush(stdout);
      }
      if (voltage.visible && voltage.speed_control) {
        const drive::velocity_loop_info& velocity = voltage.velocity;
        printf("DVL,%u,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d\n", value.sequence,
               velocity.time_ms, velocity.left_target, velocity.right_target,
               velocity.left_error, velocity.right_error, velocity.left_correction,
               velocity.right_correction, velocity.active);
        fflush(stdout);
      }
      if (config::ARM_SERIAL_LOG) {
        controller_display_mutex.lock();
        const controller_info info = ControllerInfo;
        controller_display_mutex.unlock();
        // 所有串口输出放在同一线程；手臂日志也只在 Y 开启回传时输出。
        printf("BMV,%u,%d,%d,%d,%d,%.2f,%.2f,%d,%d,%.2f,%.2f\n",
               info.tick, info.input.arm_up, info.input.arm_down, info.output.arm_enabled,
               info.output.arm_voltage,
               ArmL ? ArmL->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0,
               ArmR ? ArmR->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0,
               ArmL && ArmL->installed(), ArmR && ArmR->installed(),
               ArmL ? ArmL->current(vex::amp) : 0.0,
               ArmR ? ArmR->current(vex::amp) : 0.0);
        fflush(stdout);
      }
    }
    vex::wait(10, vex::msec);
  }
}

void ControllerInfoDisplay() {
  while (true) {
    // 显示、串口和主控制循环分别运行，互不等待慢速外设。
    controller_display_mutex.lock();
    const controller_info info = ControllerInfo;
    controller_display_mutex.unlock();
    const double L_angle = ArmL ? ArmL->position(vex::deg) * config::ARM_UP_DIRECTION : 0;
    const double R_angle = ArmR ? ArmR->position(vex::deg) * config::ARM_UP_DIRECTION : 0;
    const bool L_installed = ArmL && ArmL->installed();
    const bool R_installed = ArmR && ArmR->installed();
    drive::sample speed;
    const bool speed_available = drive::ReadLatestSample(speed);
    if (Controller.installed()) {
      char lines[3][20];
      snprintf(lines[0], sizeof(lines[0]), "U%+4.1fL%5.1fR%5.1f",
               info.output.arm_voltage / 1000.0, L_angle, R_angle);
      snprintf(lines[1], sizeof(lines[1]), "1:%d 2:%d E:%d C:%d %u",
               info.input.arm_up, info.input.arm_down, info.output.arm_enabled,
               info.configured, (info.tick / 20) % 10);
      snprintf(lines[2], sizeof(lines[2]), "%s LOG:%s Y M%d%d",
               config::CONTROLLER_VERSION, info.telemetry ? "ON" : "OFF", L_installed, R_installed);
      if (info.voltage.visible) {
        snprintf(lines[0], sizeof(lines[0]), "VTEST %.1fV %s",
                 info.voltage.millivolts / 1000.0, info.voltage.status);
        snprintf(lines[1], sizeof(lines[1]), "L%+.1fV R%+.1fV",
                 info.output.L_voltage / 1000.0, info.output.R_voltage / 1000.0);
        if (info.voltage.feedforward) {
          snprintf(lines[0], sizeof(lines[0]), "%s %drpm %s",
                   info.voltage.speed_control ? "SP" : "FH",
                   info.voltage.running ? static_cast<int>(info.voltage.target_rpm) :
                   info.voltage.selected_rpm, info.voltage.status);
          char left[8] = "--", right[8] = "--";
          if (speed_available && speed.left.velocity_valid)
            snprintf(left, sizeof(left), "%.1f", speed.left.rpm);
          if (speed_available && speed.right.velocity_valid)
            snprintf(right, sizeof(right), "%.1f", speed.right.rpm);
          snprintf(lines[1], sizeof(lines[1]), "L%s R%s", left, right);
          snprintf(lines[2], sizeof(lines[2]), "H%+.1f %s LOG:%s",
                   info.voltage.heading.error, info.voltage.heading.locked ? "LOCK" : "WAIT",
                   info.telemetry ? "ON" : "OFF");
        }
      }
      if (info.turn.running) {
        snprintf(lines[0], sizeof(lines[0]), "S%u D%.0f A%.1f",
                 info.turn.side, info.turn.distance_mm, info.turn.angle_deg);
        snprintf(lines[1], sizeof(lines[1]), "%s E%+.1f",
                 info.turn.status, info.turn.error_deg);
      }
      for (int row = 0; row < 3; ++row) {
        Controller.Screen.setCursor(row + 1, 1);
        Controller.Screen.print("%-19s", lines[row]);
      }
    }
    int left_count, right_count;
    hardware::GetDriveMotorCount(left_count, right_count);
    // 保留初始化错误所在的第二行。
    Brain.Screen.printAt(10, 60, "CFG=%d PAD=%d RUN=%d LOG=%s       ",
                         info.configured, Controller.installed(), info.enabled,
                         info.telemetry ? "ON" : "OFF");
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
    if (info.voltage.visible) {
      if (info.voltage.feedforward) {
        Brain.Screen.printAt(10, 235, true, "%s %drpm %s                       ",
                             info.voltage.speed_control ? "SP" : "FH",
                             info.voltage.selected_rpm, info.voltage.status);
      } else Brain.Screen.printAt(10, 235, true, "VTEST %.1fV %s                     ",
                           info.voltage.millivolts / 1000.0, info.voltage.status);
    } else {
      Brain.Screen.printAt(10, 235, true, "%s S%u D%.0f A%.1f E%+.1f          ",
                           info.turn.status, info.turn.side, info.turn.distance_mm,
                           info.turn.angle_deg, info.turn.error_deg);
    }
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
static_assert(config::TELEMETRY_INTERVAL_MS >= 20, "Telemetry interval must be at least 20 ms");
static_assert(config::DRIVE_TEST_VOLTAGES[0] > 0 &&
              config::DRIVE_TEST_VOLTAGES[0] < config::DRIVE_TEST_VOLTAGES[1] &&
              config::DRIVE_TEST_VOLTAGES[1] < config::DRIVE_TEST_VOLTAGES[2] &&
              config::DRIVE_TEST_VOLTAGES[2] <= config::MAX_VOLTAGE &&
              config::DRIVE_TEST_TIMEOUT_MS > 0,
              "Invalid fixed voltage test settings");
static_assert(config::FF_TEST_TARGETS[0] > 0 &&
              config::FF_TEST_TARGETS[0] < config::FF_TEST_TARGETS[1] &&
              config::FF_TEST_TARGETS[1] < config::FF_TEST_TARGETS[2] &&
              config::FF_TEST_TARGETS[2] <= 600 && config::TEST_MODE_HOLD_MS > 0,
              "Invalid feedforward test settings");
static_assert(config::INERTIAL_PORT >= 1 && config::INERTIAL_PORT <= 21,
              "Inertial smart port must be 1..21");
static_assert(config::DRIVE_WHEEL_DIAMETER_MM >= 0 &&
              config::DRIVE_MOTOR_REVS_PER_WHEEL_REV > 0,
              "Wheel diameter must be nonnegative and external reduction must be positive");
static_assert(config::AUTO_DRIVE_RPM > 0 && config::AUTO_DRIVE_RPM <= 100 &&
              config::AUTO_TURN_RPM > 0 && config::AUTO_TURN_RPM <= 100,
              "Auto test speeds must be within (0, 100] rpm");
static_assert(config::AUTO_TURN_FAST_RPM >= config::AUTO_TURN_RPM &&
              config::AUTO_TURN_FAST_RPM <= 100 && config::AUTO_TURN_SLOW_ANGLE > 0,
              "Invalid fast turn speed or precision zone");
static_assert(config::SQUARE_SIDE_MM > 0 && config::AUTO_ANGLE_TOLERANCE > 0 &&
              config::AUTO_TURN_KP > 0 && config::AUTO_TURN_KD >= 0,
              "Invalid square test control settings");
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
  turn_test TurnTest;
  voltage_test VoltageTest(config::DRIVE_TEST_VOLTAGES, config::DRIVE_TEST_TIMEOUT_MS);
  VoltageTest.ConfigureFeedforward(config::FF_TEST_TARGETS, config::FF_TEST_LEFT,
                                  config::FF_TEST_RIGHT, config::TEST_MODE_HOLD_MS);
  VoltageTest.ConfigureReverseFeedforward(config::FF_TEST_LEFT_REVERSE, config::FF_TEST_RIGHT_REVERSE);
  VoltageTest.ConfigureHeading(config::FF_HEADING);
  VoltageTest.ConfigureVelocityPID(config::DRIVE_VELOCITY_PID);
  drive::telemetry_switch Telemetry;
  vex::thread drive_sampling(drive::SamplingTask);
  vex::thread drive_telemetry(DriveTelemetry);
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
    const bool telemetry = Telemetry.Update(input.telemetry, Controller.installed());
    drive::SetTelemetryEnabled(telemetry);
    manual::Output output = DriverControl.Update(
        input, enabled, config::JOYSTICK_DEADBAND, config::MAX_VOLTAGE,
        config::ARM_UP_VOLTAGE, config::ARM_DOWN_VOLTAGE,
        config::CLAW_VOLTAGE * config::CLAW_CLOSE_DIRECTION);
    // 自动测试仅用于无场控的手柄调试，场控接入时立即让出底盘。
    // 标记整个输出阶段，包含自动测试内部的电机写入；不持锁执行慢速设备操作。
    drive::BeginCommandUpdate();
    const bool voltage_control = VoltageTest.Info().running ||
                                 input.voltage_forward || input.voltage_reverse;
    const bool test_owns_drive = TurnTest.Update(input, output,
        enabled && !competition_connected, current_time, voltage_control);
    bool motors_online = true;
    if (voltage_control || VoltageTest.Info().visible || input.voltage_up || input.voltage_down) {
      int left_count, right_count;
      hardware::GetDriveMotorCount(left_count, right_count);
      motors_online = left_count == 4 && right_count == 4;
    }
    drive::sample speed;
    const bool speed_available = drive::ReadLatestSample(speed);
    const bool stopped = speed_available && speed.left.velocity_valid &&
        speed.right.velocity_valid && std::fabs(speed.left.raw_average_rpm) < 5.0 &&
        std::fabs(speed.right.raw_average_rpm) < 5.0;
    double heading = 0.0;
    // 惯导状态只由主循环读取；显示和串口线程只消费带互斥锁的快照。
    const bool inertial_ready =
        hardware::ReadInertial(heading) == hardware::InertialState::ready;
    drive::velocity_feedback velocity;
    velocity.valid = speed_available && speed.left.velocity_valid && speed.right.velocity_valid;
    velocity.sequence = speed.sequence;
    velocity.time_ms = speed.time_ms;
    velocity.left_rpm = speed.left.rpm;
    velocity.right_rpm = speed.right.rpm;
    const bool voltage_owns_drive = VoltageTest.Update(input, output,
        enabled && !competition_connected && !test_owns_drive, motors_online, current_time,
        stopped, inertial_ready, heading, velocity);
    hardware::Apply(output, !test_owns_drive, voltage_owns_drive);
    const int mode = test_owns_drive ? 3 : (VoltageTest.Info().visible ?
        (VoltageTest.Info().speed_control ? 5 : (VoltageTest.Info().feedforward ? 4 : 1)) : 0);
    drive::PublishCommand(output.L_voltage, output.R_voltage, enabled, !test_owns_drive,
                          mode, VoltageTest.Info().target_rpm);
    // 控制循环仅发布快照，不执行屏幕打印和串口等待。
    controller_display_mutex.lock();
    ControllerInfo.input = input;
    ControllerInfo.output = output;
    ControllerInfo.configured = configured;
    ControllerInfo.enabled = enabled;
    ControllerInfo.telemetry = telemetry;
    ControllerInfo.tick = loop_count;
    ControllerInfo.dt = dt;
    ControllerInfo.turn = TurnTest.Info();
    ControllerInfo.voltage = VoltageTest.Info();
    controller_display_mutex.unlock();
    ++loop_count;
    vex::wait(config::UPDATE_TIME, vex::msec);
  }
}
