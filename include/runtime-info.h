#pragma once

#include "driver-control-logic.h"
#include "test-control.h"
#include "autonomous.h"
#include "competition-control.h"
#include "startup-mode.h"
#include "mechanisms.h"
#if BARTMOSS_CALIBRATION
#include "calibration-control.h"
#endif

#if BARTMOSS_HIGH_SPEED
#include "high-speed-control.h"
#endif

// 主循环发布整包状态，显示和回传线程只读取副本，不共享控制器实例。
struct runtime_info {
  drive::pose pose;
  runtime::teach_info teaching;
  manual::input input;
  manual::output output;
  bool configured = false, enabled = false, telemetry = false, autonomous_period = false;
  bool route_debug = false;
  bool local_selection = false, controls_armed = false, auto_ready = false;
  const char* auto_status = "WAIT READY";  // 静态字符串，通过同一mutex快照发布。
  bool high_speed_stages = false;
  bool initializing = false, returning_to_manual = false;
  double return_remaining_ms = 0;
  runtime::local_mode local_mode = runtime::local_mode::select;
  unsigned int tick = 0;
  double dt = 0;
  test_info test;
  autonomous_info automatic;
  mechanisms::lift_info lift;
  mechanisms::arm_tuning_info arm_tuning;
  mechanisms::claw_info claw;
  competition_request phase;
#if BARTMOSS_HIGH_SPEED
  tuning_metrics tuning;
#endif
#if BARTMOSS_CALIBRATION
  calibration_info calibration;
#endif
};
void PublishRuntimeInfo(const runtime_info& value);
runtime_info ReadRuntimeInfo();
runtime::teach_packet ReadTeachPacket();
void DriveTelemetry();
void ControllerInfoDisplay();
