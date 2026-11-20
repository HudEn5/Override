#pragma once

#include "drive-straight.h"
#include "drive-corner.h"
#include "drive-turn.h"

namespace config {
inline drive::point_config PointControl() {
  drive::point_config result;
  result.position_tolerance_mm = 5;
  result.pose_stale_ms = 200;
  result.turn_finish_speed_scale = 1.5;  // 设1恢复原点位补角下限，普通Turn不受影响。
  return result;
}

// 路线与高速测试共用同一套参数，普通低速配置不受影响。
constexpr double HIGH_SPEED_STRAIGHT_ACCELERATION = 320;
inline drive::straight_config HighSpeedStraightControl(const drive::straight_config& baseline) {
  auto result = baseline;
  result.motion.acceleration_rpm_s = HIGH_SPEED_STRAIGHT_ACCELERATION;
  result.motion.deceleration_rpm_s = HIGH_SPEED_STRAIGHT_ACCELERATION;
  return result;  // H3 基线，不采用尚有异常测速的新 H4 位置增益。
}
constexpr double HIGH_SPEED_CORNER_ACCELERATION = 320;
constexpr double HIGH_SPEED_CORNER_DECELERATION = 320;
constexpr double HIGH_SPEED_CORNER_WHEEL_ACCELERATION = 320;  // 预测预算，并非实测机械极限。
constexpr double HIGH_SPEED_CORNER_WHEEL_DECELERATION = 320;
constexpr double HIGH_SPEED_CORNER_CRUISE_RPM = 180;
constexpr double HIGH_SPEED_CORNER_ENTRY_RPM = 130;
constexpr double HIGH_SPEED_CORNER_CURVE_ACCELERATION = 120;
constexpr double HIGH_SPEED_CORNER_EXIT_ACCELERATION = 240;
inline drive::corner_config HighSpeedCornerControl(const drive::corner_config& baseline) {
  auto result = baseline;
  result.acceleration_rpm_s = HIGH_SPEED_CORNER_ACCELERATION;
  result.deceleration_rpm_s = HIGH_SPEED_CORNER_DECELERATION;
  result.prediction_enabled = true;
  result.prediction_wheel_acceleration_rpm_s = HIGH_SPEED_CORNER_WHEEL_ACCELERATION;
  result.prediction_wheel_deceleration_rpm_s = HIGH_SPEED_CORNER_WHEEL_DECELERATION;
  result.corner_rpm = HIGH_SPEED_CORNER_CRUISE_RPM;
  result.corner_floor_rpm = HIGH_SPEED_CORNER_ENTRY_RPM;
  result.corner_acceleration_rpm_s = HIGH_SPEED_CORNER_CURVE_ACCELERATION;
  result.exit_acceleration_rpm_s = HIGH_SPEED_CORNER_EXIT_ACCELERATION;
  return result;
}
}  // 运动配置命名空间

namespace config {
// 实验提速档。false恢复core-v2.0.0的速度与控制参数，保留基线函数原样。
constexpr bool EXPERIMENTAL_FAST_ROUTE = true;
// 用户要求轮端450rpm；电机输出轴经30:36外部减速，代码需540rpm。
constexpr double ROUTE_DRIVE_RPM = EXPERIMENTAL_FAST_ROUTE ? 540 : 330;
// 直行斜率单位为电机rpm/s；原400/400，当前放缓到240/240以减小动态跟踪差异。
// 最高速度仍为540电机rpm；短距离会因加减速距离不足而达不到该上限。
constexpr double ROUTE_STRAIGHT_ACCELERATION = 240;
constexpr double ROUTE_STRAIGHT_DECELERATION = 240;
// 原地转向同样使用540电机rpm上限；角速度规划会按角度缩短加速段。
constexpr double ROUTE_TURN_RPM = EXPERIMENTAL_FAST_ROUTE ? 540 : 140;
constexpr bool TURN_ANGULAR_PROFILE = true;  // false单独恢复原540rpm上限的角度PD方案。
constexpr bool TURN_FAST_FINISH = true;      // false恢复首轮turn1补角，保留角速度规划。
// 正式自动到位且实际速度满足零速门槛即接下一动作，不额外驻停；X单项仍250ms。
// 改为100恢复route2确认时间；改为250恢复route1。
constexpr double AUTO_SETTLE_MS = EXPERIMENTAL_FAST_ROUTE ? 0 : 250;
inline drive::straight_config RouteStraightControl(const drive::straight_config& baseline) {
  auto result = HighSpeedStraightControl(baseline);
  if (EXPERIMENTAL_FAST_ROUTE) {
    result.motion.acceleration_rpm_s = ROUTE_STRAIGHT_ACCELERATION;
    result.motion.deceleration_rpm_s = ROUTE_STRAIGHT_DECELERATION;
    result.motion.minimum_settle_rpm = 18;
    // 高速轨迹结束时可能仍有位置落后，避免用原20rpm上限长时间爬行补足。
    result.motion.position_kP = .6;
    result.motion.max_position_correction_rpm = 60;
  }
  return result;
}
inline drive::turn_config RouteTurnControl(const drive::turn_config& baseline,
                                           bool experimental = EXPERIMENTAL_FAST_ROUTE) {
  auto result = baseline;
  if (experimental) {
    result.kP = 2.0;
    result.kD = .25;
    result.acceleration_rpm_s = 640;
    result.profile_enabled = TURN_ANGULAR_PROFILE;
    if (TURN_ANGULAR_PROFILE) {
      result.kP = 3.0;
      result.kD = .10;  // 跟踪规划角度/角速度，末端继续纠偏。
      // 原900/1200配120ms预留容易提前掉速；让减速更早、更连续，减少尾段再起步。
      result.angular_acceleration_deg_s2 = 1200;
      result.angular_deceleration_deg_s2 = 1000;
      result.profile_braking_delay_ms = 100;
      result.rate_filter_ms = 20;  // 原60ms会延迟减速反馈与零速识别，保留短滤波抑制量化。
      result.profile_finish_kP = TURN_FAST_FINISH ? 6.0 : 0;
      // 原32°/s门槛会在减速中先撤掉低速补偿、随后再加回；扩大平滑过渡避免尾段掉速。
      result.minimum_rate_deg_s = TURN_FAST_FINISH ? 48 : 16;
      result.minimum_settle_rpm = TURN_FAST_FINISH ? 60 : 30;
      result.minimum_finish_rpm = TURN_FAST_FINISH ? 30 : 24;
    }
  } else
    result.acceleration_rpm_s = 240;  // core-v2.0.0原地转向斜率，不继承后续基础配置的360。
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
// 单侧Swing独立整定：加强追赶与角速度阻尼，不提高末端最低轮速。
constexpr double SWING_POSITION_FEEDBACK_SCALE = 4.0;
constexpr double SWING_RATE_DAMPING = 1.4;
inline drive::turn_config SwingControl(const drive::turn_config& baseline) {
  auto result = baseline;
  if (EXPERIMENTAL_FAST_ROUTE && result.profile_enabled) {
    result.kP *= SWING_POSITION_FEEDBACK_SCALE;
    result.profile_finish_kP *= SWING_POSITION_FEEDBACK_SCALE;
    result.kD = SWING_RATE_DAMPING;
  }
  return result;  // 普通Turn不经过此函数；加减速/制动预留/补角下限/精度沿用原值。
}
inline drive::corner_config RouteCornerControl(const drive::corner_config& baseline) {
  auto result = HighSpeedCornerControl(baseline);
  if (EXPERIMENTAL_FAST_ROUTE) {
    // 不增加弯内速度或放宽角度窗口，只加快末端位置补足。
    result.finish_min_rpm = 15;
    result.finish_max_rpm = 40;
    result.finish_kP = .8;
  }
  return result;
}
}  // 实验性能配置命名空间
