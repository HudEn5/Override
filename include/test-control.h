#pragma once

#include "driver-control-logic.h"
#include "drive-straight.h"
#include "drive-turn.h"
#include "drive-corner.h"

enum class test_motion { straight, turn, corner };
struct test_settings {
  double straight_mm = 0, straight_rpm = 0;
  double turn_deg = 0, turn_rpm = 0;
  double before_mm = 0, corner_deg = 0, after_mm = 0, corner_rpm = 0;
};
struct test_info {
  test_motion selected = test_motion::straight;
  int direction = 1;
  bool visible = false, running = false;
  const char* status = "READY";
  double request_mm = 0, request_deg = 0, maximum_rpm = 0, target_rpm = 0;
  drive::straight_info straight;
  drive::turn_info turn;
  drive::corner_info corner;
};

// 日常测试只编排公共运动控制器，A 开始、B 取消；电机仍由主循环统一写入。
class test_control {
 public:
  void Configure(const test_settings& value, const drive::straight_config& straight,
                 const drive::turn_config& turn, const drive::corner_config& corner) {
    settings = value; Straight.Configure(straight); Turn.Configure(turn); Corner.Configure(corner);
    info = test_info(); UpdateSelection();
  }
  bool Update(const manual::Input& input, manual::Output& output, bool allowed,
              double now, const drive::straight_feedback& frame);
  const test_info& Info() const { return info; }
 private:
  void UpdateSelection();
  void ResetMotion();
  void Cancel();
  void Publish();
  test_settings settings;
  test_info info;
  drive::straight_controller Straight;
  drive::turn_controller Turn;
  drive::corner_controller Corner;
  bool previous_start = false, previous_next = false, previous_previous = false;
  bool previous_positive = false, previous_negative = false, previous_allowed = false;
};
