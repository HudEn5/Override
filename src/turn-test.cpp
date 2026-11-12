#include "turn-test.h"
#include "control-config.h"
#include "robot-config.h"
#include <cmath>

namespace {
constexpr double PI = 3.141592653589793;
constexpr double SETTLE_MS = 250.0;
constexpr double TIMEOUT_MS = 12000.0;
constexpr double DISTANCE_TOLERANCE_MM = 5.0;

double Limit(double value, double magnitude) {
  if (value > magnitude) return magnitude;
  if (value < -magnitude) return -magnitude;
  return value;
}
}  // 匿名命名空间

void turn_test::Finish(const char* status) {
  hardware::StopDriveTurn();
  stage = Stage::idle;
  info.running = false;
  info.status = status;
}

void turn_test::BeginStage(Stage next, double now_ms, double left, double right) {
  hardware::StopDriveTurn();
  stage = next;
  stage_origin = (left + right) / 2.0;
  started_at = now_ms;
  settled_since = -1.0;
  forward_rpm = 0.0;
}

bool turn_test::Update(const manual::Input& input, const manual::Output& output,
                       bool enabled, double now_ms, bool voltage_control) {
  double heading = 0.0;
  const hardware::InertialState imu = hardware::ReadInertial(heading);
  const bool ready = enabled && imu == hardware::InertialState::ready;
  const bool a_edge = input.turn_test && !previous_a;
  const bool b_edge = input.square_test && !previous_b;
  const bool start_a = ready && previous_ready && a_edge && !voltage_control;
  const bool start_b = ready && previous_ready && b_edge && !voltage_control;
  const bool became_ready = ready && !previous_ready;
  previous_a = input.turn_test;
  previous_b = input.square_test;
  previous_ready = ready;
  const bool sticks = output.L_voltage != 0 || output.R_voltage != 0;

  // 校准期间底盘静止，避免手柄输入破坏零偏校准。
  if (imu == hardware::InertialState::calibrating) {
    if (stage != Stage::idle) Finish("IMU CAL");
    hardware::StopDriveTurn();
    info.status = "IMU CAL";
    return true;
  }
  if (stage != Stage::idle && (!ready || sticks || a_edge || b_edge || voltage_control)) {
    Finish(imu == hardware::InertialState::ready ? "CANCEL" : "IMU LOST");
    return !sticks;
  }
  if (!ready) {
    info.status = imu == hardware::InertialState::offline ? "NO IMU" :
                  (imu == hardware::InertialState::fault ? "RESTART IMU" : "TEST DISABLED");
    return false;
  }
  if (stage == Stage::idle) {
    if (became_ready) info.status = "READY";
    if (!start_a && !start_b) return false;
    if (sticks || (start_a && start_b)) {
      info.status = "CENTER STICKS";
      return false;
    }
    if (start_b && (config::DRIVE_WHEEL_DIAMETER_MM <= 0 ||
                    config::DRIVE_MOTOR_REVS_PER_WHEEL_REV <= 0)) {
      info.status = "SET WHEEL";
      return false;
    }
    double left, right;
    if (!hardware::ReadDrivePosition(left, right)) {
      info.status = "DRIVE OFFLINE";
      return false;
    }
    square = start_b;
    initial_heading = heading;
    target_heading = heading + (square ? 0.0 : 90.0);
    previous_time = now_ms;
    previous_heading = heading;
    previous_position = (left + right) / 2.0;
    angular_rate = linear_rate = 0.0;
    info.side = square ? 1 : 0;
    info.distance_mm = 0.0;
    info.angle_deg = 0.0;
    info.error_deg = square ? 0.0 : 90.0;
    info.running = true;
    info.status = square ? "DRIVE" : "TURN";
    BeginStage(square ? Stage::drive : Stage::turn, now_ms, left, right);
    return true;
  }

  double left, right;
  if (!hardware::ReadDrivePosition(left, right)) {
    Finish("DRIVE OFFLINE");
    return true;
  }
  const double position = (left + right) / 2.0;
  const double mm_per_degree = config::DRIVE_WHEEL_DIAMETER_MM * PI /
                              (360.0 * config::DRIVE_MOTOR_REVS_PER_WHEEL_REV);
  const double dt = (now_ms - previous_time) / 1000.0;
  if (dt <= 0 || dt > 0.25) {
    previous_time = now_ms;
    previous_heading = heading;
    previous_position = position;
    angular_rate = linear_rate = 0.0;
    settled_since = -1.0;
    hardware::StopDriveTurn();
    if (now_ms - started_at >= TIMEOUT_MS) Finish("TIMEOUT");
    return true;
  }
  const double alpha = dt / (0.06 + dt);
  angular_rate += alpha * ((heading - previous_heading) / dt - angular_rate);
  linear_rate += alpha * ((position - previous_position) * mm_per_degree / dt - linear_rate);
  previous_time = now_ms;
  previous_heading = heading;
  previous_position = position;
  info.angle_deg = heading - initial_heading;
  info.error_deg = target_heading - heading;

  if (now_ms - started_at >= TIMEOUT_MS) {
    Finish("TIMEOUT");
    return true;
  }
  if (info.angle_deg < -10.0) {
    Finish("CHECK DIRECTION");
    return true;
  }

  const double distance = (position - stage_origin) * mm_per_degree;
  const double remaining = config::SQUARE_SIDE_MM - distance;
  if (stage == Stage::drive) info.distance_mm = distance;
  const bool angle_close = std::fabs(info.error_deg) <= config::AUTO_ANGLE_TOLERANCE;
  const bool distance_close = stage == Stage::turn ||
                              std::fabs(remaining) <= DISTANCE_TOLERANCE_MM;
  if (stage == Stage::drive && distance < -20.0) {
    Finish("CHECK DIRECTION");
    return true;
  }
  if (angle_close && distance_close) {
    hardware::StopDriveTurn();
    forward_rpm = 0.0;
    // 误差和速度同时满足，连续稳定后才进入下一段。
    if (std::fabs(angular_rate) <= 2.5 &&
        (stage == Stage::turn || std::fabs(linear_rate) <= 15.0)) {
      if (settled_since < 0) settled_since = now_ms;
      if (now_ms - settled_since >= SETTLE_MS) {
        if (stage == Stage::drive) {
          // 方形每次转弯都以最初朝向为基准，目标依次为 90/180/270/360 度。
          target_heading = initial_heading + 90.0 * info.side;
          info.status = "TURN";
          BeginStage(Stage::turn, now_ms, left, right);
        } else if (!square || info.side == 4) {
          Finish("DONE");
        } else {
          ++info.side;
          info.status = "DRIVE";
          info.distance_mm = 0.0;
          BeginStage(Stage::drive, now_ms, left, right);
        }
      }
    } else {
      settled_since = -1.0;
    }
    return true;
  }
  settled_since = -1.0;
  info.status = stage == Stage::drive ? "DRIVE" : "TURN";
  // 距离外环生成目标转速，靠近终点降速。SDK 在电机内部跟踪转速。
  double requested_forward = 0.0;
  if (stage == Stage::drive && !distance_close) {
    requested_forward = Limit(remaining * 0.25, config::AUTO_DRIVE_RPM);
    if (std::fabs(requested_forward) < 3.0)
      requested_forward = remaining > 0 ? 3.0 : -3.0;
  }
  forward_rpm += Limit(requested_forward - forward_rpm, 45.0 * dt);
  double turn_kp = config::AUTO_TURN_KP;
  double turn_limit = config::AUTO_TURN_RPM;
  if (stage == Stage::turn) {
    // 远处加快，进入精调区前平滑减小增益与转速上限。
    double fast_blend = (std::fabs(info.error_deg) - config::AUTO_TURN_SLOW_ANGLE) /
                        config::AUTO_TURN_SLOW_ANGLE;
    if (fast_blend < 0.0) fast_blend = 0.0;
    if (fast_blend > 1.0) fast_blend = 1.0;
    turn_limit += (config::AUTO_TURN_FAST_RPM - config::AUTO_TURN_RPM) * fast_blend;
    turn_kp *= 1.0 + (config::AUTO_TURN_FAST_RPM / config::AUTO_TURN_RPM - 1.0) * fast_blend;
  }
  double turn_rpm = Limit(turn_kp * info.error_deg -
                           config::AUTO_TURN_KD * angular_rate, turn_limit);
  if (angle_close) turn_rpm = 0.0;
  else if (std::fabs(angular_rate) < 2.5 && std::fabs(turn_rpm) < 3.0)
    turn_rpm = info.error_deg > 0 ? 3.0 : -3.0;
  const double left_rpm = forward_rpm - turn_rpm;
  const double right_rpm = forward_rpm + turn_rpm;
  hardware::SetDriveVelocity(left_rpm, right_rpm);
  return true;
}
