#pragma once

#include "driver-control-logic.h"

struct turn_test_info {
  const char* status = "IMU WAIT";
  unsigned int side = 0;
  double distance_mm = 0.0;
  double angle_deg = 0.0;
  double error_deg = 0.0;
  bool running = false;
};

class turn_test {
 public:
  bool Update(const manual::Input& input, const manual::Output& output,
              bool enabled, double now_ms, bool voltage_control = false);
  const turn_test_info& Info() const { return info; }

 private:
  enum class Stage { idle, drive, turn };
  void Finish(const char* status);
  void BeginStage(Stage next, double now_ms, double left, double right);
  Stage stage = Stage::idle;
  turn_test_info info;
  bool previous_a = false;
  bool previous_b = false;
  bool previous_ready = false;
  bool square = false;
  double initial_heading = 0.0;
  double target_heading = 0.0;
  double stage_origin = 0.0;
  double started_at = 0.0;
  double settled_since = -1.0;
  double previous_time = 0.0;
  double previous_heading = 0.0;
  double previous_position = 0.0;
  double angular_rate = 0.0;
  double linear_rate = 0.0;
  double forward_rpm = 0.0;
};
