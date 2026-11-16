#pragma once

#include "driver-control-logic.h"
#include "test-control.h"
#include "autonomous.h"
#include "competition-control.h"
#include "startup-mode.h"
#if BARTMOSS_CALIBRATION
#include "calibration-control.h"
#endif

#if BARTMOSS_HIGH_SPEED
#include "high-speed-control.h"
#endif

// 主循环发布整包状态，显示和回传线程只读取副本，不共享控制器实例。
struct runtime_info {
  manual::Input input;
  manual::Output output;
  bool configured = false, enabled = false, telemetry = false, autonomous_period = false;
  bool route_debug = false;
  bool local_selection = false, controls_armed = false, auto_ready = false;
  bool high_speed_stages = false;
  bool initializing = false, returning_to_manual = false;
  double return_remaining_ms = 0;
  runtime::local_mode local_mode = runtime::local_mode::select;
  unsigned int tick = 0;
  double dt = 0;
  test_info test;
  autonomous_info automatic;
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
void DriveTelemetry();
void ControllerInfoDisplay();
