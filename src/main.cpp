#include "control-config.h"
#include "robot-config.h"
#include "drive-test.h"
#include "drive-test-route.h"
#include <stdio.h>

namespace {
struct controller_info {
  manual::Input input;
  manual::Output output;
  bool configured = false;
  bool enabled = false;
  unsigned int tick = 0;
  double dt = 0;
  motion::MoveResult drive_result = motion::MoveResult::Idle;
  bool drive_test_view = false;
  bool drive_test_active = false;
  unsigned int drive_test_run = 0;
  motion::DriveTelemetry drive;
  bool drive_route_view = false;
  motion::DriveRouteStage route_stage = motion::DriveRouteStage::Idle;
};

controller_info ControllerInfo;
vex::mutex controller_display_mutex;

// 独立于较慢的屏幕刷新，约 20 Hz 输出测试曲线；主控制线程不等待串口。
void DriveTelemetryOutput() {
  unsigned int previous_tick = 0;
  unsigned int previous_run = 0;
  motion::MoveResult previous_result = motion::MoveResult::Idle;
  while (true) {
    controller_display_mutex.lock();
    const controller_info info = ControllerInfo;
    controller_display_mutex.unlock();
    if (info.drive_test_view && info.drive_test_run && info.tick != previous_tick &&
        (info.drive.result == motion::MoveResult::Running ||
         info.drive_test_run != previous_run || info.drive.result != previous_result)) {
      const motion::DriveTelemetry& drive = info.drive;
      printf("BMD,%u,%u,%s,%s,%d,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
             info.drive_test_run, info.tick, motion::ResultName(drive.result), drive.phase,
             drive.feedbackValid, drive.elapsedSec, drive.targetPosition,
             drive.reference.position, drive.leftPosition, drive.rightPosition,
             drive.reference.velocity / 6, drive.command.leftRpm, drive.command.rightRpm,
             drive.leftVelocity / 6, drive.rightVelocity / 6);
      fflush(stdout);
      previous_run = info.drive_test_run;
      previous_result = info.drive.result;
    }
    previous_tick = info.tick;
    vex::wait(50, vex::msec);
  }
}

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
      if (info.drive_test_view) {
        snprintf(lines[0], sizeof(lines[0]), "%s %-8s %3.1fs",
                 info.drive_test_active ? "TEST" : "LAST",
                 motion::ResultName(info.drive.result), info.drive.elapsedSec);
        snprintf(lines[1], sizeof(lines[1]), "RPM L%+5.0f R%+5.0f",
                 info.drive.leftVelocity / 6, info.drive.rightVelocity / 6);
        snprintf(lines[2], sizeof(lines[2]), "%s Y/B:STOP", info.drive.phase);
        if (info.drive_route_view) {
          snprintf(lines[0], sizeof(lines[0]), "ROUTE %s", motion::RouteStageName(info.route_stage));
          snprintf(lines[2], sizeof(lines[2]), "%s Y:START/STOP", motion::ResultName(info.drive.result));
        }
      }
      for (int row = 0; row < 3; ++row) {
        Controller.Screen.setCursor(row + 1, 1);
        Controller.Screen.print("%-19s", lines[row]);
      }
    }
    int left_count, right_count;
    hardware::GetDriveMotorCount(left_count, right_count);
    if (info.drive_test_view) {
      const motion::DriveTelemetry& drive = info.drive;
      Brain.Screen.printAt(10, 60, true, "%s %s %s M=%d/4,%d/4       ",
                           info.drive_route_view ? "ROUTE" : info.drive_test_active ? "TEST" : "LAST TEST",
                           info.drive_route_view ? motion::RouteStageName(info.route_stage) : motion::ResultName(drive.result),
                           drive.phase, left_count, right_count);
      Brain.Screen.printAt(10, 85, "t=%.2f / %.2fs feedback=%d         ",
                           drive.elapsedSec, drive.durationSec, drive.feedbackValid);
      Brain.Screen.printAt(10, 110, "REF %+.0f / %+.0f deg V%+.1f RPM     ",
                           drive.reference.position, drive.targetPosition, drive.reference.velocity / 6);
      Brain.Screen.printAt(10, 135, "POS L%+.1f R%+.1f deg              ",
                           drive.leftPosition, drive.rightPosition);
      Brain.Screen.printAt(10, 160, "RPM L%+.1f R%+.1f                 ",
                           drive.leftVelocity / 6, drive.rightVelocity / 6);
      Brain.Screen.printAt(10, 185, "CMD L%+.1f R%+.1f RPM             ",
                           drive.command.leftRpm, drive.command.rightRpm);
      Brain.Screen.printAt(10, 210, "ERR L%+.1f R%+.1f deg             ",
                           drive.targetPosition - drive.leftPosition,
                           drive.rightTargetPosition - drive.rightPosition);
      Brain.Screen.printAt(10, 235, true, "Y: %s  B/STICK:STOP               ",
                           info.drive_test_active ? "STOP ROUTE" : "START ROUTE");
      vex::wait(200, vex::msec);
      continue;
    }
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
    Brain.Screen.printAt(10, 235, true, "%s AUTO %s Y:START ROUTE          ",
                         info.output.arm_voltage ? "RUN" : "HOLD",
                         motion::ResultName(info.drive_result));
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
    Controller.Screen.print("Y: start/stop route");
  }
  manual::driver_control DriverControl(config::PISTON_INITIAL_STATE);
  motion::DriveTestSession DriveTest(config::DRIVE_TEST_LONG_MOTOR_REVS);
  motion::DriveTestRoute DriveRoute(config::DRIVE_TEST_ROUTE_PAUSE_SEC);
  vex::thread controller_display(ControllerInfoDisplay);
  vex::thread drive_telemetry(DriveTelemetryOutput);
  vex::timer ControlTimer;
  double pre_time = ControlTimer.time(vex::msec);
  unsigned int loop_count = 0;
  bool previous_automatic = false;
  bool drive_test_view = false;
  bool drive_test_units = false;
  unsigned int drive_test_run = 0;
  bool drive_route_view = false;
  bool test_brake_latched = false;
  while (true) {
    const double current_time = ControlTimer.time(vex::msec);
    const double dt = current_time - pre_time;
    pre_time = current_time;
    const bool competition_connected =
        Competition.isFieldControl() || Competition.isCompetitionSwitch();
    const bool driver_period = !competition_connected ||
        (Competition.isEnabled() && Competition.isDriverControl());
    const bool automatic = configured && competition_connected &&
        Competition.isEnabled() && Competition.isAutonomous();
    const bool enabled = configured && driver_period && !automatic && Controller.installed();
    const manual::Input input = hardware::ReadInput();
    motion::DriveTestInput test_input;
    test_input.permitted = enabled && !competition_connected;
    test_input.startStop = Controller.ButtonY.pressing();
    test_input.stop = Controller.ButtonB.pressing();
    test_input.neutral = std::abs(input.y_axis) <= config::JOYSTICK_DEADBAND &&
        std::abs(input.angular_axis) <= config::JOYSTICK_DEADBAND &&
        !input.arm_up && !input.arm_down && !input.claw_close && !input.claw_open && !input.toggle;
    const motion::DriveTestAction test = DriveTest.Update(test_input,
        hardware::GetDriveMotionResult() == motion::MoveResult::Running || DriveRoute.active());
    if (test.ownsDrive && !automatic) drive_test_view = true;
    if (test.cancel || (previous_automatic && !automatic)) {
      DriveRoute.Cancel();
      hardware::CancelDriveMotion();
      test_brake_latched = true;
    }
    if (automatic) {
      drive_test_view = false;
      drive_test_units = false;
      drive_route_view = false;
      DriveRoute.Cancel();
      test_brake_latched = false;
    }
    const manual::Output output = DriverControl.Update(
        input, enabled && !test.ownsDrive, config::JOYSTICK_DEADBAND, config::MAX_VOLTAGE,
        config::ARM_UP_VOLTAGE, config::ARM_DOWN_VOLTAGE,
        config::CLAW_VOLTAGE * config::CLAW_CLOSE_DIRECTION);
    // 取消或完成后中性手控保持制动；主动推动摇杆才恢复原手控输出。
    if (!test.ownsDrive && enabled && (output.L_voltage || output.R_voltage))
      test_brake_latched = false;
    hardware::Apply(output, driver_period && !automatic && !test.ownsDrive && !test_brake_latched);
    if (automatic) {
      if (!previous_automatic && config::AUTONOMOUS_TRAPEZOID_ENABLED) {
        hardware::StartDriveMotion(config::AUTONOMOUS_DISTANCE_CM);
      }
      hardware::UpdateDriveMotion();
    } else if (test.ownsDrive) {
      if (test.start) {
        ++drive_test_run;
        test_brake_latched = false;
        drive_test_units = true;
        drive_route_view = true;
        const motion::DriveRouteAction route = DriveRoute.Start(current_time / 1000.0);
        if (route.startStraight) hardware::StartDriveTest(test.motorRevolutions);
      }
      // 完成或故障后持续刹车；取消周期也不能由手控覆盖。
      hardware::UpdateDriveMotion();
      const motion::DriveRouteAction route = DriveRoute.Step(
          hardware::GetDriveMotionResult(), current_time / 1000.0,
          test_input.permitted && test_input.neutral && !test_input.stop && !test.cancel);
      if (route.startTurn) {
        ++drive_test_run;
        hardware::StartDriveTurn(config::DRIVE_TEST_TURN_MOTOR_REVS);
        hardware::UpdateDriveMotion();
      }
      if (DriveRoute.stage() == motion::DriveRouteStage::Failed) hardware::CancelDriveMotion();
      if (DriveRoute.stage() == motion::DriveRouteStage::Done ||
          DriveRoute.stage() == motion::DriveRouteStage::Failed) test_brake_latched = true;
    } else if (!driver_period || test_brake_latched) {
      hardware::CancelDriveMotion();
    }
    previous_automatic = automatic;
    // 控制循环仅发布快照，不执行屏幕打印和串口等待。
    controller_display_mutex.lock();
    ControllerInfo.input = input;
    ControllerInfo.output = output;
    ControllerInfo.configured = configured;
    ControllerInfo.enabled = enabled;
    ControllerInfo.tick = loop_count;
    ControllerInfo.dt = dt;
    ControllerInfo.drive_result = hardware::GetDriveMotionResult();
    ControllerInfo.drive_test_view = drive_test_view;
    ControllerInfo.drive_test_active = DriveRoute.active();
    ControllerInfo.drive_test_run = drive_test_units ? drive_test_run : 0;
    ControllerInfo.drive_route_view = drive_route_view;
    ControllerInfo.route_stage = DriveRoute.stage();
    ControllerInfo.drive = drive_test_view && drive_test_units ?
        hardware::GetDriveMotionTelemetry() : motion::DriveTelemetry();
    controller_display_mutex.unlock();
    ++loop_count;
    vex::wait(config::UPDATE_TIME, vex::msec);
  }
}
