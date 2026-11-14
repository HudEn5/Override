#include "control-config.h"
#include "runtime-info.h"
#include "robot-config.h"
#include "drive-sampling.h"
#include <cstdio>

namespace {
const char* Environment() {
  return config::PROGRAM_MODE == runtime::program_mode::competition ? "COMP" :
      config::PROGRAM_MODE == runtime::program_mode::calibration ? "CAL" : "TEST";
}
void TestLines(const test_info& test, bool log, char (&lines)[3][20]) {
  const bool straight = test.selected == test_motion::straight, turn = test.selected == test_motion::turn;
  snprintf(lines[0], sizeof(lines[0]), "%s %+.0f %.0frpm", straight ? "STR" : turn ? "TURN" : "CNR",
           straight ? test.request_mm : test.request_deg, test.maximum_rpm);
  const double distance = straight ? test.straight.motion.distance_mm : test.corner.distance_mm;
  const double error = straight ? test.straight.heading.error : turn ? test.turn.error_deg : test.corner.error_deg;
  snprintf(lines[1], sizeof(lines[1]), "%s E%+.1f", test.status, error);
  snprintf(lines[2], sizeof(lines[2]), "A:GO B:STOP LOG:%s", log ? "ON" : "OFF");
  if (test.running) {
    if (turn) snprintf(lines[0], sizeof(lines[0]), "TURN %.1f/%.0f", test.turn.angle_deg, test.turn.target_deg);
    else snprintf(lines[0], sizeof(lines[0]), "%s D%.0f E%+.1f", straight ? "STR" : "CNR", distance, error);
    if (!straight && !turn)
      snprintf(lines[2], sizeof(lines[2]), "V%.0f P%.0f LOG:%s", test.corner.center_rpm, test.corner.planned_rpm, log ? "ON" : "OFF");
  }
}
}

void ControllerInfoDisplay() {
  while (true) {
    const runtime_info info = ReadRuntimeInfo();
    const double left = ArmL ? ArmL->position(vex::deg) * config::ARM_UP_DIRECTION : 0;
    const double right = ArmR ? ArmR->position(vex::deg) * config::ARM_UP_DIRECTION : 0;
    if (Controller.installed()) {
      char lines[3][20];
      snprintf(lines[0], sizeof(lines[0]), "U%+.1f L%.0f R%.0f", info.output.arm_voltage / 1000.0, left, right);
      snprintf(lines[1], sizeof(lines[1]), "%s CFG%d ARM%d", Environment(), info.configured, info.output.arm_enabled);
      snprintf(lines[2], sizeof(lines[2]), "LOG:%s Y %u", info.telemetry ? "ON" : "OFF", (info.tick / 20) % 10);
      if (info.test.visible) TestLines(info.test, info.telemetry, lines);
#if BARTMOSS_CALIBRATION
      if (info.calibration.visible) {
        const auto& c = info.calibration;
        snprintf(lines[0], sizeof(lines[0]), "CAL %s %s", c.feedforward ? c.speed_control ? "SP" : "FH" : "VTEST", c.status);
        if (c.feedforward) snprintf(lines[1], sizeof(lines[1]), "%drpm H%+.1f", c.selected_rpm, c.heading.error);
        else snprintf(lines[1], sizeof(lines[1]), "%.1fV", c.millivolts / 1000.0);
        snprintf(lines[2], sizeof(lines[2]), "LOG:%s Y", info.telemetry ? "ON" : "OFF");
      }
#endif
      if (info.autonomous_period) {
        snprintf(lines[0], sizeof(lines[0]), "AUTO %s", info.automatic.status);
        snprintf(lines[1], sizeof(lines[1]), "STEP %u", info.automatic.step);
        snprintf(lines[2], sizeof(lines[2]), "COMP %s", info.automatic.motion == autonomous_motion::corner ?
            info.automatic.corner.status : info.automatic.motion == autonomous_motion::turn ?
            info.automatic.turn.status : info.automatic.drive.status);
      }
      for (int row = 0; row < 3; ++row) {
        Controller.Screen.setCursor(row + 1, 1); Controller.Screen.print("%-19s", lines[row]);
      }
    }
    int l = 0, r = 0;
    hardware::GetDriveMotorCount(l, r);
    // 第二行留给初始化错误；其余显示只读取快照，不调用运动控制器。
    Brain.Screen.printAt(10, 60, true, "%s CFG=%d PAD=%d RUN=%d LOG=%s          ", Environment(), info.configured,
                         Controller.installed(), info.enabled, info.telemetry ? "ON" : "OFF");
    Brain.Screen.printAt(10, 85, "A3=%4d A1=%4d Tick=%u          ", info.input.y_axis, info.input.angular_axis, info.tick);
    Brain.Screen.printAt(10, 110, "L=%6d R=%6d mV           ", info.output.L_voltage, info.output.R_voltage);
    Brain.Screen.printAt(10, 135, "FLD=%d SW=%d EN=%d CB=%d:%u       ", Competition.isFieldControl(),
                         Competition.isCompetitionSwitch(), Competition.isEnabled(), static_cast<int>(info.phase.stage), info.phase.generation);
    Brain.Screen.printAt(10, 160, "Motors L=%d/4 R=%d/4        ", l, r);
    Brain.Screen.printAt(10, 185, "U=%5d AL=%5.1f AR=%5.1f      ", info.output.arm_voltage, left, right);
    Brain.Screen.printAt(10, 210, "R1=%d R2=%d ARM=%d dt=%4.0fms      ", info.input.arm_up, info.input.arm_down, info.output.arm_enabled, info.dt);
    Brain.Screen.printAt(10, 235, true, "%s %s                                  ", Environment(),
                         info.autonomous_period ? info.automatic.status : info.test.visible ? info.test.status : "MANUAL");
    vex::wait(200, vex::msec);
  }
}
