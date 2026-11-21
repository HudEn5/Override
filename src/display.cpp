#include "control-config.h"
#include "runtime-info.h"
#include "robot-config.h"
#include "drive-sampling.h"
#include <cstdio>
#include <cstring>

namespace {
const char* Environment() {
#if BARTMOSS_HIGH_SPEED
  return "HS";
#endif
  return config::PROGRAM_MODE == runtime::program_mode::competition   ? "COMP"
         : config::PROGRAM_MODE == runtime::program_mode::calibration ? "CAL"
                                                                      : "TEST";
}
void TestLines(const test_info& test, bool log, char (&lines)[3][20]) {
  const bool straight = test.selected == test_motion::straight, turn = test.selected == test_motion::turn;
  snprintf(lines[0], sizeof(lines[0]), "%s %+.0f %.0frpm",
           straight ? "STR"
           : turn   ? "TURN"
                    : "CNR",
           straight ? test.request_mm : test.request_deg, test.maximum_rpm);
  const double distance = straight ? test.straight.motion.distance_mm : test.corner.distance_mm;
  const double error = straight ? test.straight.heading.error_deg
                       : turn   ? test.turn.error_deg
                                : test.corner.error_deg;
  snprintf(lines[1], sizeof(lines[1]), "%s E%+.1f", test.status, error);
  snprintf(lines[2], sizeof(lines[2]), "A:GO B:STOP LOG:%s", log ? "ON" : "OFF");
  if (test.running) {
    if (turn)
      snprintf(lines[0], sizeof(lines[0]), "TURN %.1f/%.0f", test.turn.angle_deg, test.turn.target_deg);
    else
      snprintf(lines[0], sizeof(lines[0]), "%s D%.0f E%+.1f", straight ? "STR" : "CNR", distance, error);
    if (!straight && !turn)
      snprintf(lines[2], sizeof(lines[2]), "V%.0f P%.0f LOG:%s", test.corner.center_rpm,
               test.corner.planned_rpm, log ? "ON" : "OFF");
  }
}
}

void ControllerInfoDisplay() {
  bool lift_panel_was_visible = false;
#if BARTMOSS_HIGH_SPEED
  char previous[3][20] = {};
#endif
  while (true) {
    const runtime_info info = ReadRuntimeInfo();
    const double left = ArmL ? ArmL->position(vex::deg) * config::ARM_UP_DIRECTION : 0;
    const double right = ArmR ? ArmR->position(vex::deg) * config::ARM_UP_DIRECTION : 0;
    if (Controller.installed()) {
      char lines[3][20];
      snprintf(lines[0], sizeof(lines[0]), "U%+.1f L%.0f R%.0f", info.output.arm_mV / 1000.0, left, right);
      snprintf(lines[1], sizeof(lines[1]), "%s CFG%d ARM%d", Environment(), info.configured,
               info.output.arm_enabled);
#if BARTMOSS_CALIBRATION
      snprintf(lines[2], sizeof(lines[2]), "LOG:%s Y", info.telemetry ? "ON" : "OFF");
#else
      snprintf(lines[2], sizeof(lines[2]), "Y:LIFT LOG:%s", info.telemetry ? "ON" : "OFF");
#endif
      if (info.test.visible)
        TestLines(info.test, info.telemetry, lines);
#if BARTMOSS_HIGH_SPEED
      if (info.test.visible && info.high_speed_stages) {
        snprintf(lines[0], sizeof(lines[0]), "H%u %+.0f %.0frpm", info.tuning.stage + 1, info.test.request_mm,
                 info.test.maximum_rpm);
        snprintf(lines[1], sizeof(lines[1]), "%s P%.0f E%.1f", info.test.status, info.tuning.peak_rpm,
                 info.tuning.peak_heading_deg);
        snprintf(lines[2], sizeof(lines[2]), "%s LOG:%s",
                 info.tuning.state == drive::motion_state::done
                     ? (info.tuning.qualified ? "SPEED OK" : "LOW SPEED")
                     : "A:GO B:STOP",
                 info.telemetry ? "ON" : "OFF");
      }
#endif
#if BARTMOSS_CALIBRATION
      if (info.calibration.visible) {
        const auto& c = info.calibration;
        snprintf(lines[0], sizeof(lines[0]), "CAL %s %s",
                 c.feedforward ? c.speed_control ? "SP" : "FH" : "VTEST", c.status);
        if (c.feedforward)
          snprintf(lines[1], sizeof(lines[1]), "%drpm H%+.1f", c.selected_rpm, c.heading.error_deg);
        else
          snprintf(lines[1], sizeof(lines[1]), "%.1fV", c.millivolts / 1000.0);
        snprintf(lines[2], sizeof(lines[2]), "LOG:%s Y", info.telemetry ? "ON" : "OFF");
      }
#endif
      if (info.local_selection) {
        if (info.local_mode == runtime::local_mode::select) {
          snprintf(lines[0], sizeof(lines[0]), "A:AUTO B:MANUAL");
          snprintf(lines[1], sizeof(lines[1]), "X:TEST LOG:%s", info.telemetry ? "ON" : "OFF");
          snprintf(lines[2], sizeof(lines[2]), "%s",
                   mechanisms::LIFT_WITH_ROUTE && !info.lift.homed ? "WAIT POWER ZERO"
                   : info.auto_ready ? "Release keys/choose" : "AUTO: WAIT READY");
        } else if (!info.controls_armed) {
          snprintf(lines[0], sizeof(lines[0]), "%s SELECTED",
                   info.local_mode == runtime::local_mode::automatic ? "AUTO"
                   : info.local_mode == runtime::local_mode::manual  ? "MANUAL"
                                                                     : "TEST");
          snprintf(lines[1], sizeof(lines[1]), "Release A/B/X");
          snprintf(lines[2], sizeof(lines[2]), "Center sticks");
        } else if (info.local_mode == runtime::local_mode::manual) {
          snprintf(lines[1], sizeof(lines[1]), "MANUAL %s",
                   info.automatic.run ? info.automatic.status : "READY");
          snprintf(lines[2], sizeof(lines[2]), "Y:LIFT B1s:MENU");
        } else if (info.local_mode == runtime::local_mode::automatic && !info.autonomous_period) {
          snprintf(lines[0], sizeof(lines[0]), "AUTO WAIT READY");
          snprintf(lines[2], sizeof(lines[2]), "B:STOP LOG:%s", info.telemetry ? "ON" : "OFF");
        }
      }
      if (info.autonomous_period) {
        snprintf(lines[0], sizeof(lines[0]), "AUTO %s", info.automatic.status);
        snprintf(lines[1], sizeof(lines[1]), "STEP %u", info.automatic.step);
        snprintf(lines[2], sizeof(lines[2]), "%s LOG:%s",
                 info.automatic.waiting                                ? "WAIT"
                 : info.automatic.motion == autonomous_motion::rolling ? info.automatic.rolling.status
                 : info.automatic.motion == autonomous_motion::corner  ? info.automatic.corner.status
                 : info.automatic.motion == autonomous_motion::push    ? info.automatic.push.status
                 : info.automatic.motion == autonomous_motion::turn    ? info.automatic.turn.status
                                                                       : info.automatic.drive.status,
                 info.telemetry ? "ON" : "OFF");
        if (info.returning_to_manual)
          snprintf(lines[2], sizeof(lines[2]), "MANUAL IN %.1fs", info.return_remaining_ms / 1000);
      }
      if (info.initializing) {
        snprintf(lines[0], sizeof(lines[0]), "INITIALIZING");
        snprintf(lines[1], sizeof(lines[1]), "IMU CALIBRATING");
        snprintf(lines[2], sizeof(lines[2]), "KEEP ROBOT STILL");
      }
      for (int row = 0; row < 3; ++row) {
#if BARTMOSS_HIGH_SPEED
        if (std::strcmp(previous[row], lines[row]) == 0)
          continue;
        snprintf(previous[row], sizeof(previous[row]), "%s", lines[row]);
#endif
        Controller.Screen.setCursor(row + 1, 1);
        Controller.Screen.print("%-19s", lines[row]);
      }
    }
    int l = 0, r = 0;
    hardware::GetDriveMotorCount(l, r);
    if (lift_panel_was_visible != info.lift_panel_enabled) {
      Brain.Screen.setFillColor(vex::black);
      Brain.Screen.setPenColor(vex::black);
      Brain.Screen.drawRectangle(0, 65, 480, 175);
      Brain.Screen.setPenColor(vex::white);
      lift_panel_was_visible = info.lift_panel_enabled;
    }
    // 第二行留给初始化错误；其余显示只读取快照，不调用运动控制器。
    Brain.Screen.printAt(10, 60, true, "%s CFG=%d PAD=%d RUN=%d LOG=%s          ", Environment(),
                         info.configured, Controller.installed(), info.enabled,
                         info.telemetry ? "ON" : "OFF");
    Brain.Screen.printAt(10, 85, "X=%7.0f Y=%7.0f mm          ", info.pose.x_mm, info.pose.y_mm);
    Brain.Screen.printAt(10, 110, "L=%6d R=%6d mV           ", info.output.left_mV, info.output.right_mV);
    Brain.Screen.printAt(10, 135, "FLD=%d SW=%d EN=%d CB=%d:%u       ", Competition.isFieldControl(),
                         Competition.isCompetitionSwitch(), Competition.isEnabled(),
                         static_cast<int>(info.phase.stage), info.phase.generation);
    Brain.Screen.printAt(10, 160, "H=%7.1f %s L%d R%d       ", info.pose.heading_deg,
                         info.pose.needs_reset ? "RESET"
                         : info.pose.valid     ? "OK"
                                               : "WAIT",
                         l, r);
    Brain.Screen.printAt(10, 185, "U=%5d AL=%5.1f AR=%5.1f      ", info.output.arm_mV, left, right);
    Brain.Screen.printAt(10, 210, "R1=%d R2=%d ARM=%d dt=%4.0fms      ", info.input.arm_up,
                         info.input.arm_down, info.output.arm_enabled, info.dt);
    Brain.Screen.printAt(10, 235, true, "%s %s                                  ", Environment(),
                         info.local_selection && info.local_mode == runtime::local_mode::select
                             ? "A:AUTO B:MANUAL X:TEST"
                         : info.autonomous_period ? info.automatic.status
                         : info.test.visible      ? info.test.status
                                                  : "MANUAL");
    if (info.lift_auto_blocked || (info.local_selection && info.local_mode == runtime::local_mode::select &&
                                 mechanisms::LIFT_WITH_ROUTE && !info.lift.homed))
      Brain.Screen.printAt(10, 235, "AUTO BLOCKED: WAIT POWER ZERO       ");
    if (info.lift_panel_enabled) {
      Brain.Screen.printAt(10, 85, true, "CLAW %s %dmV %.0f/%.0fms           ", info.claw.status,
                           info.claw.millivolts, info.claw.elapsed_ms, info.claw.duration_ms);
      Brain.Screen.printAt(10, 110, true, "CMD %s X%d H%d                   ", info.lift.command_status,
                           info.input.toggle, info.output.toggle_piston);
      Brain.Screen.printAt(10, 135, true, "%s ZERO%d T%.1fdeg dt=%.0fms       ", info.lift.status,
                           info.lift.homed, info.lift.target_deg, info.lift.control_dt_ms);
      Brain.Screen.printAt(10, 160, true, "LIFT %.1f L%.1f R%.1f V%d/%d      ", info.lift.position_deg(),
                           info.lift.left_deg, info.lift.right_deg, info.lift.left_mV, info.lift.right_mV);
      const char* labels[6] = {"CLAW OPEN", "CLAW CLOSE", "CANCEL CLAW", "ZERO LIFT", "LIFT TEST", "CANCEL LIFT"};
      Brain.Screen.setFillColor(vex::black);
      for (int button = 0; button < 6; ++button) {
        const int x = button % 3 * mechanisms::BUTTON_WIDTH;
        const int y = button < 3 ? mechanisms::CLAW_PANEL_TOP : mechanisms::PANEL_TOP;
        Brain.Screen.drawRectangle(x, y, mechanisms::BUTTON_WIDTH, mechanisms::PANEL_HEIGHT);
        Brain.Screen.printAt(x + 8, y + 22, true, "%s", labels[button]);
      }
    } else if (info.autonomous_period) {
      Brain.Screen.printAt(10, 185, true, "LIFT %s %.1f/%.1f V%d/%d       ", info.lift.status,
                           info.lift.position_deg(), info.lift.target_deg, info.lift.left_mV, info.lift.right_mV);
      Brain.Screen.printAt(10, 210, true, "ZERO%d CMD %s dt=%.0fms          ", info.lift.homed,
                           info.lift.command_status, info.lift.control_dt_ms);
    }
#if BARTMOSS_HIGH_SPEED
    vex::wait(500, vex::msec);
#else
    vex::wait(200, vex::msec);
#endif
  }
}
