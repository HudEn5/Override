#pragma once

#include "motion-presets.h"
#include "drive-turn.h"

namespace config {
// 实验提速档。false恢复core-v2.0.0的速度与控制参数，保留基线函数原样。
constexpr bool EXPERIMENTAL_FAST_ROUTE = true;
// 用户要求轮端450rpm；电机输出轴经30:36外部减速，代码需540rpm。
constexpr double ROUTE_DRIVE_RPM = EXPERIMENTAL_FAST_ROUTE ? 540 : 330;
// 原地转向同样使用540电机rpm上限；角速度规划会按角度缩短加速段。
constexpr double ROUTE_TURN_RPM = EXPERIMENTAL_FAST_ROUTE ? 540 : 140;
constexpr bool TURN_ANGULAR_PROFILE = true;  // false单独恢复原540rpm上限的角度PD方案。
constexpr bool TURN_FAST_FINISH = true;  // false恢复首轮turn1补角，保留角速度规划。
// 正式自动到位且实际速度满足零速门槛即接下一动作，不额外驻停；X单项仍250ms。
// 改为100恢复route2确认时间；改为250恢复route1。
constexpr double AUTO_SETTLE_MS = EXPERIMENTAL_FAST_ROUTE ? 0 : 250;
inline drive::straight_config RouteStraightControl(const drive::straight_config& baseline) {
  auto result = HighSpeedStraightControl(baseline);
  if (EXPERIMENTAL_FAST_ROUTE) {
    result.motion.acceleration_rpm_s = result.motion.deceleration_rpm_s = 400;
    result.motion.minimum_settle_rpm = 18;
    // 高速轨迹结束时可能仍有位置落后，避免用原20rpm上限长时间爬行补足。
    result.motion.position_kP = .6; result.motion.max_position_correction_rpm = 60;
  }
  return result;
}
inline drive::turn_config RouteTurnControl(const drive::turn_config& baseline) {
  auto result = baseline;
  if (EXPERIMENTAL_FAST_ROUTE) {
    result.kP = 2.0; result.kD = .25;
    result.acceleration_rpm_s = 640;
    result.profile_enabled = TURN_ANGULAR_PROFILE;
    if (TURN_ANGULAR_PROFILE) {
      result.kP = 3.0; result.kD = .10;  // 跟踪规划角度/角速度，末端继续纠偏。
      result.angular_acceleration_deg_s2 = 900;
      result.angular_deceleration_deg_s2 = 1200;
      result.profile_braking_delay_ms = 120;
      result.profile_finish_kP = TURN_FAST_FINISH ? 6.0 : 0;
      // 补角最低转速在32°/s以下完整保留，64°/s以上退出；只在参考结束后生效。
      result.minimum_rate_deg_s = TURN_FAST_FINISH ? 32 : 16;
      result.minimum_settle_rpm = TURN_FAST_FINISH ? 50 : 30;
      result.minimum_finish_rpm = TURN_FAST_FINISH ? 30 : 24;
    }
  }
  return result;
}
inline drive::straight_config AutoStraightControl(const drive::straight_config& baseline) {
  auto result = RouteStraightControl(baseline);
  result.motion.settle_ms = AUTO_SETTLE_MS;
  return result;
}
inline drive::turn_config AutoTurnControl(const drive::turn_config& baseline) {
  auto result = RouteTurnControl(baseline);
  result.settle_ms = AUTO_SETTLE_MS;
  return result;
}
inline drive::corner_config RouteCornerControl(const drive::corner_config& baseline) {
  auto result = HighSpeedCornerControl(baseline);
  if (EXPERIMENTAL_FAST_ROUTE) {
    // 不增加弯内速度或放宽角度窗口，只加快末端位置补足。
    result.finish_min_rpm = 15; result.finish_max_rpm = 40; result.finish_kP = .8;
  }
  return result;
}
}  // 实验性能配置命名空间
