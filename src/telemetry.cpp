#include "control-config.h"
#include "runtime-info.h"
#include "drive-sampling.h"
#include "robot-config.h"
#include <cstdio>

namespace {
void PrintVelocity(unsigned int sequence, const drive::velocity_loop_info& v) {
  printf("DVL,%u,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d\n", sequence, v.time_ms, v.left_target_rpm,
         v.right_target_rpm, v.left_error_rpm, v.right_error_rpm, v.left_correction_mV, v.right_correction_mV,
         v.active);
}
void PrintHeading(unsigned int sequence, const drive::heading_info& h) {
  printf("DHD,%u,%.1f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d\n", sequence, h.time_ms, h.angle_deg, h.target_deg,
         h.error_deg, h.correction_mV, h.locked, h.ready, h.active);
}
// 独立协议：余量是当前直线段，末两列为已完成/计划转向次数；不复用DCR整程余量。
void PrintRolling(unsigned int sequence, const drive::rolling_info& r) {
  printf("DRL,%u,%.1f,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%u,%u\n", sequence,
         r.time_ms, r.elapsed_ms, r.phase, static_cast<int>(r.state), r.distance_mm, r.segment_remaining_mm,
         r.angle_deg, r.reference_deg, r.error_deg, r.rate_deg_s, r.center_rpm, r.left_target_rpm,
         r.right_target_rpm, r.left_mV, r.right_mV, r.completed_turns, r.total_turns);
  PrintVelocity(sequence, r.velocity);
}
void PrintMotion(unsigned int sequence, const test_info& test) {
  if (!test.visible)
    return;
  if (test.selected == test_motion::straight) {
    const auto& m = test.straight.motion;
    printf("DTP,%u,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d\n", sequence, m.time_ms, m.elapsed_ms,
           m.reference_mm, m.reference_rpm, m.distance_mm, m.target_rpm, m.remaining_mm, m.duration_ms,
           m.running, m.phase);
    PrintHeading(sequence, test.straight.heading);
    PrintVelocity(sequence, test.straight.velocity);
  } else if (test.selected == test_motion::turn) {
    const auto& t = test.turn;
    printf("DTN,%u,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\n", sequence, t.time_ms, t.elapsed_ms,
           t.angle_deg, t.target_deg, t.error_deg, t.rate_deg_s, t.target_rpm, t.left_mV, t.right_mV,
           static_cast<int>(t.state));
    if (t.profiled)
      printf("DTA,%u,%.1f,%.1f,%.2f,%.2f,%.1f,%.2f,%d,%.2f\n", sequence, t.time_ms, t.elapsed_ms,
             t.reference_deg, t.reference_rate_deg_s, t.profile_duration_ms, t.profile_peak_deg_s,
             t.profile_phase, t.reference_deg - t.angle_deg);
    PrintVelocity(sequence, t.velocity);
  } else {
    const auto& c = test.corner;
    printf("DCR,%u,%.1f,%.1f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\n", sequence, c.time_ms,
           c.elapsed_ms, c.phase, static_cast<int>(c.state), c.distance_mm, c.remaining_mm, c.angle_deg,
           c.reference_deg, c.error_deg, c.rate_deg_s, c.center_rpm, c.left_target_rpm, c.right_target_rpm,
           c.left_mV, c.right_mV);
    PrintVelocity(sequence, c.velocity);
  }
}
}

// 唯一串口打印线程。锁内只复制状态，所有格式化和外设访问都在锁外完成。
void DriveTelemetry() {
  unsigned teach_batch = 0, teach_index = 0;
  unsigned reported_route = 0;
  unsigned reported_arm_pulse = 0;
  vex::timer teach_poll;
  while (true) {
    if (teach_poll.time(vex::msec) >= 100) {
      teach_poll.reset();
      const auto point = ReadTeachPacket();
      if (point.index && (teach_batch != point.batch || teach_index != point.index)) {
        printf("DWP,%u,%u,%u,%.1f,%.1f,%.1f\n", point.batch, point.index, point.total,
               point.point.x_mm, point.point.y_mm, point.point.heading_deg);
        fflush(stdout);
        teach_batch = point.batch;
        teach_index = point.index;
      }
      const auto info = ReadRuntimeInfo();
      const auto& arm = info.arm_tuning;
      if (info.telemetry && arm.visible && arm.run && arm.run != reported_arm_pulse) {
        // 时间、试次、脉冲耗时、指令电压、左右电机角度与角速度、状态。
        printf("DAT,%.1f,%u,%.1f,%d,%.2f,%.2f,%.2f,%.2f,%s\n", arm.time_ms, arm.run,
               arm.elapsed_ms, info.output.arm_mV, info.lift.left_deg, info.lift.right_deg,
               arm.left_deg_s, arm.right_deg_s, arm.status);
        if (info.output.arm_mV == 0)
          reported_arm_pulse = arm.run;
        fflush(stdout);
      }
    }
    drive::sample value;
    unsigned int dropped = 0;
    if (drive::PopSample(value, dropped)) {
#if !BARTMOSS_HIGH_SPEED
      drive::PrintSample(value, dropped);
#endif
      const runtime_info info = ReadRuntimeInfo();
      printf("DPO,%u,%.1f,%.2f,%.2f,%.2f,%d,%d\n", value.sequence, info.pose.time_ms, info.pose.x_mm,
             info.pose.y_mm, info.pose.heading_deg, info.pose.valid, info.pose.needs_reset);
      PrintMotion(value.sequence, info.test);
      const bool route_terminal =
          info.local_selection && info.local_mode != runtime::local_mode::test && info.automatic.run;
      if (info.route_debug || info.autonomous_period || route_terminal) {
        const auto& a = info.automatic;
        printf("DRA,%u,%.1f,%u,%u,%d,%d,%d,%d,%d\n", value.sequence, a.time_ms, a.run, a.step,
               static_cast<int>(a.motion), a.active, a.finished, a.waiting, static_cast<int>(a.result));
        const bool new_terminal = a.finished && a.run != reported_route;
        const auto& lift = info.lift;
        const auto& arm = a.mechanisms.channels[static_cast<int>(mechanism::channel::arm)];
        if (info.local_selection && a.run && arm.claimed &&
            ((a.active && lift.result == mechanisms::state::running) || new_terminal)) {
          // 记录同一主循环快照中的左右角度、目标和实际发布的电压指令。
          printf("DAL,%u,%u,%u,%.1f,%.2f,%.2f,%.2f,%d,%s,%.1f,%.1f\n", value.sequence, a.run, a.step,
                 lift.elapsed_ms, lift.left_deg, lift.right_deg, lift.target_deg, info.output.arm_mV,
                 lift.status, lift.reference_deg_s, lift.measured_deg_s);
        }
        const auto& claw = a.mechanisms.channels[static_cast<int>(mechanism::channel::claw)];
        if (info.local_selection && claw.claimed && (a.active || new_terminal)) {
          printf("DCL,%u,%u,%u,%d,%s\n", value.sequence, a.run, a.step, info.output.claw_mV,
                 info.claw.status);
        }
        if (new_terminal) {
          // 每轮只在原始日志记一次退出原因，沿用现有 LOG 开关与 DRA 格式。
          printf("DRE,%u,%u,%u,%d,%s\n", value.sequence, a.run, a.step, static_cast<int>(a.result), a.status);
          reported_route = a.run;
        }
        test_info motion;
        const bool moving_route = (info.route_debug || info.autonomous_period) && a.run;
        if (moving_route && a.motion == autonomous_motion::push) {
          const auto& p = a.push;
          printf("DPS,%u,%.1f,%.1f,%.1f,%d,%d,%d,%d,%.2f,%.2f\n", value.sequence, p.time_ms, p.elapsed_ms,
                 p.duration_ms, p.left_mV, p.right_mV, static_cast<int>(p.state), p.settling, p.left_rpm,
                 p.right_rpm);
        }
        if (moving_route && a.motion == autonomous_motion::rolling)
          PrintRolling(value.sequence, a.rolling);
        motion.visible = moving_route && a.motion != autonomous_motion::none &&
                         a.motion != autonomous_motion::push && a.motion != autonomous_motion::rolling;
        motion.selected = a.motion == autonomous_motion::corner ? test_motion::corner
                          : a.motion == autonomous_motion::turn ? test_motion::turn
                                                                : test_motion::straight;
        motion.straight = a.drive;
        motion.turn = a.turn;
        motion.corner = a.corner;
        PrintMotion(value.sequence, motion);
      }
#if BARTMOSS_HIGH_SPEED
      const auto& h = info.tuning;
      if (info.test.visible && h.run)
        printf("DHS,%u,%.1f,%u,%u,%d,%d,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d\n", value.sequence,
               h.time_ms, h.run, h.stage + 1, h.direction, static_cast<int>(h.state), h.elapsed_ms,
               info.test.straight.motion.distance_mm, info.test.straight.motion.remaining_mm, h.peak_rpm,
               h.peak_heading_deg, h.speed_rms_rpm, h.high_voltage_ms, h.at_speed_ms, h.qualified);
#endif
#if BARTMOSS_CALIBRATION
      if (info.calibration.visible && info.calibration.feedforward)
        PrintHeading(value.sequence, info.calibration.heading);
      if (info.calibration.visible && info.calibration.speed_control)
        PrintVelocity(value.sequence, info.calibration.velocity);
#endif
      if (config::ARM_SERIAL_LOG) {
        printf("BMV,%u,%d,%d,%d,%d,%.2f,%.2f,%d,%d,%.2f,%.2f\n", info.tick, info.input.arm_up,
               info.input.arm_down, info.output.arm_enabled, info.output.arm_mV,
               ArmL ? ArmL->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0,
               ArmR ? ArmR->position(vex::deg) * config::ARM_UP_DIRECTION : 0.0, ArmL && ArmL->installed(),
               ArmR && ArmR->installed(), ArmL ? ArmL->current(vex::amp) : 0.0,
               ArmR ? ArmR->current(vex::amp) : 0.0);
      }
      fflush(stdout);
    }
    vex::wait(10, vex::msec);
  }
}
