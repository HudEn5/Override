#include "control-config.h"
#include "runtime-info.h"
#include "drive-sampling.h"
#include "robot-config.h"
#include <cstdio>

namespace {
void PrintVelocity(unsigned int sequence, const drive::velocity_loop_info& v) {
  printf("DVL,%u,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d\n", sequence, v.time_ms,
         v.left_target, v.right_target, v.left_error, v.right_error, v.left_correction, v.right_correction, v.active);
}
void PrintHeading(unsigned int sequence, const drive::heading_info& h) {
  printf("DHD,%u,%.1f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d\n", sequence,
         h.time_ms, h.angle, h.target, h.error, h.correction, h.locked, h.ready, h.active);
}
void PrintMotion(unsigned int sequence, const test_info& test) {
  if (!test.visible) return;
  if (test.selected == test_motion::straight) {
    const auto& m = test.straight.motion;
    printf("DTP,%u,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d\n", sequence,
           m.time_ms, m.elapsed_ms, m.reference_mm, m.reference_rpm, m.distance_mm, m.target_rpm,
           m.remaining_mm, m.duration_ms, m.running, m.phase);
    PrintHeading(sequence, test.straight.heading); PrintVelocity(sequence, test.straight.velocity);
  } else if (test.selected == test_motion::turn) {
    const auto& t = test.turn;
    printf("DTN,%u,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\n", sequence,
           t.time_ms, t.elapsed_ms, t.angle_deg, t.target_deg, t.error_deg, t.rate_deg_s,
           t.target_rpm, t.left_mV, t.right_mV, static_cast<int>(t.state));
    if (t.profiled) printf("DTA,%u,%.1f,%.1f,%.2f,%.2f,%.1f,%.2f,%d,%.2f\n", sequence,
        t.time_ms, t.elapsed_ms, t.reference_deg, t.reference_rate_deg_s, t.profile_duration_ms,
        t.profile_peak_deg_s, t.profile_phase, t.reference_deg - t.angle_deg);
    PrintVelocity(sequence, t.velocity);
  } else {
    const auto& c = test.corner;
    printf("DCR,%u,%.1f,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
           sequence, c.time_ms, c.elapsed_ms, c.phase, static_cast<int>(c.state), c.distance_mm,
           c.remaining_mm, c.angle_deg, c.reference_deg, c.error_deg, c.rate_deg_s, c.center_rpm,
           c.left_target_rpm, c.right_target_rpm, c.left_mV, c.right_mV);
    PrintVelocity(sequence, c.velocity);
  }
}
}

// 唯一串口打印线程。锁内只复制状态，所有格式化和外设访问都在锁外完成。
void DriveTelemetry() {
  while (true) {
    drive::sample value;
    unsigned int dropped = 0;
    if (drive::PopSample(value, dropped)) {
#if !BARTMOSS_HIGH_SPEED
      drive::PrintSample(value, dropped);
#endif
      const runtime_info info = ReadRuntimeInfo();
      printf("DPO,%u,%.1f,%.2f,%.2f,%.2f,%d,%d\n", value.sequence, info.pose.time_ms,
          info.pose.x_mm, info.pose.y_mm, info.pose.heading_deg, info.pose.valid, info.pose.needs_reset);
      PrintMotion(value.sequence, info.test);
      const bool route_terminal = info.local_selection && info.local_mode != runtime::local_mode::test && info.automatic.run;
      if (info.route_debug || info.autonomous_period || route_terminal) {
        const auto& a = info.automatic;
        printf("DRA,%u,%.1f,%u,%u,%d,%d,%d,%d,%d\n", value.sequence, a.time_ms, a.run, a.step,
               static_cast<int>(a.motion), a.active, a.finished, a.waiting, a.result);
        test_info motion;
        const bool moving_route = (info.route_debug || info.autonomous_period) && a.run;
        if (moving_route && a.motion == autonomous_motion::push) {
          const auto& p = a.push;
          printf("DPS,%u,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.2f,%.2f\n", value.sequence,
                 p.time_ms, p.elapsed_ms, p.duration_ms, p.left_mV, p.right_mV,
                 static_cast<int>(p.state), p.settling, p.left_rpm, p.right_rpm);
        }
        motion.visible = moving_route && a.motion != autonomous_motion::none && a.motion != autonomous_motion::push;
        motion.selected = a.motion == autonomous_motion::corner ? test_motion::corner :
                          a.motion == autonomous_motion::turn ? test_motion::turn : test_motion::straight;
        motion.straight = a.drive; motion.turn = a.turn; motion.corner = a.corner;
        PrintMotion(value.sequence, motion);
      }
#if BARTMOSS_HIGH_SPEED
      const auto& h = info.tuning;
      if (info.test.visible && h.run) printf("DHS,%u,%.1f,%u,%u,%d,%d,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d\n",
          value.sequence, h.time_ms, h.run, h.stage + 1, h.direction, h.state, h.elapsed_ms,
          info.test.straight.motion.distance_mm, info.test.straight.motion.remaining_mm,
          h.peak_rpm, h.peak_heading_deg, h.speed_rms_rpm, h.high_voltage_ms, h.at_speed_ms, h.qualified);
#endif
#if BARTMOSS_CALIBRATION
      if (info.calibration.visible && info.calibration.feedforward)
        PrintHeading(value.sequence, info.calibration.heading);
      if (info.calibration.visible && info.calibration.speed_control)
        PrintVelocity(value.sequence, info.calibration.velocity);
#endif
      if (config::ARM_SERIAL_LOG) {
        printf("BMV,%u,%d,%d,%d,%d,%.2f,%.2f,%d,%d,%.2f,%.2f\n", info.tick,
               info.input.arm_up, info.input.arm_down, info.output.arm_enabled, info.output.arm_voltage,
               ArmL ? ArmL->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0,
               ArmR ? ArmR->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0,
               ArmL && ArmL->installed(), ArmR && ArmR->installed(),
               ArmL ? ArmL->current(vex::amp) : 0.0, ArmR ? ArmR->current(vex::amp) : 0.0);
      }
      fflush(stdout);
    }
    vex::wait(10, vex::msec);
  }
}
