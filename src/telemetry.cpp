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
      drive::PrintSample(value, dropped);
      const runtime_info info = ReadRuntimeInfo();
      PrintMotion(value.sequence, info.test);
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
