#pragma once

#include "trapezoid-profile.h"

namespace motion {
struct TrapezoidOptions {
  double wheelDiameterCm = 0;  // 必须填写实测轮径。
  double motorRevsPerWheelRev = 1;
  double maxMotorRpm = 600;    // 硬件层按八台电机中最低的齿轮箱转速设置。
  double maxSpeedCmPerSec = 30;
  double accelerationCmPerSec2 = 30;
  double decelerationCmPerSec2 = 30;
  double positionKp = 2;
  double synchronizationKp = 1;
  double positionToleranceCm = 0.5;
  double velocityToleranceCmPerSec = 1;
  double settleTimeSec = 0.10;
  double timeoutSec = 0;  // 0 = 规划时间 + 3 秒。
  double maxUpdateIntervalSec = 0.25;
};

enum class MoveResult {
  Idle, Running, Completed, InvalidParameters, TimedOut,
  MotorDisconnected, InvalidFeedback, Interrupted, NotStationary
};

inline const char* ResultName(MoveResult result) {
  switch (result) {
    case MoveResult::Idle: return "IDLE";
    case MoveResult::Running: return "RUN";
    case MoveResult::Completed: return "DONE";
    case MoveResult::InvalidParameters: return "CONFIG";
    case MoveResult::TimedOut: return "TIMEOUT";
    case MoveResult::MotorDisconnected: return "MOTOR";
    case MoveResult::InvalidFeedback: return "FEEDBACK";
    case MoveResult::Interrupted: return "CANCEL";
    case MoveResult::NotStationary: return "MOVING";
  }
  return "UNKNOWN";
}

struct MotorFeedback {
  bool online = false;
  double positionDeg = 0;
  double velocityRpm = 0;
};

// 前四台为左侧 LF/LM/LBD/LBU，后四台为右侧 RF/RM/RBD/RBU。
struct DriveFeedback { MotorFeedback motors[8]; };
struct DriveCommand {
  double leftRpm = 0;
  double rightRpm = 0;
};

// 位置/线速度与规划使用相同单位；输出轴测试中分别为 deg、deg/s。
struct DriveTelemetry {
  MoveResult result = MoveResult::Idle;
  DriveCommand command;
  ProfileState reference = {0, 0};
  double targetPosition = 0;
  double rightTargetPosition = 0;
  double leftPosition = 0, rightPosition = 0;
  double leftVelocity = 0, rightVelocity = 0;
  double elapsedSec = 0, durationSec = 0;
  const char* phase = "IDLE";
  bool feedbackValid = false;
};

// 从四电机工程移植为八电机、非阻塞执行；所有状态由主控制线程更新。
class DriveMotion {
 public:
  MoveResult Start(double distanceCm, const TrapezoidOptions& options,
                   const DriveFeedback& feedback, double now, bool permitted) {
    command_ = DriveCommand();
    result_ = MoveResult::Idle;
    telemetry_ = DriveTelemetry();
    options_ = options;
    profile_ = TrapezoidProfile(distanceCm, options.maxSpeedCmPerSec,
        options.accelerationCmPerSec2, options.decelerationCmPerSec2);
    if (!permitted) return Finish(MoveResult::Interrupted);
    if (!profile_.valid() || !Positive(options.wheelDiameterCm) ||
        !Positive(options.motorRevsPerWheelRev) || !Positive(options.maxMotorRpm) ||
        !Nonnegative(options.positionKp) || !Nonnegative(options.synchronizationKp) ||
        !Positive(options.positionToleranceCm) || !Positive(options.velocityToleranceCmPerSec) ||
        !Nonnegative(options.settleTimeSec) || !Nonnegative(options.timeoutSec) ||
        !Positive(options.maxUpdateIntervalSec) || !Nonnegative(now)) {
      return Finish(MoveResult::InvalidParameters);
    }
    const double circumference = 3.14159265358979323846 * options.wheelDiameterCm;
    cmPerDegree_ = circumference / (360 * options.motorRevsPerWheelRev);
    cmPerSecPerRpm_ = circumference / (60 * options.motorRevsPerWheelRev);
    const double peakRpm = options.maxSpeedCmPerSec / cmPerSecPerRpm_;
    timeout_ = options.timeoutSec > 0 ? options.timeoutSec : profile_.duration() + 3;
    if (!Positive(cmPerDegree_) || !Positive(cmPerSecPerRpm_) ||
        !Positive(peakRpm) || peakRpm > options.maxMotorRpm || !Positive(timeout_)) {
      return Finish(MoveResult::InvalidParameters);
    }
    telemetry_.targetPosition = distanceCm;
    telemetry_.rightTargetPosition = distanceCm;
    telemetry_.durationSec = profile_.duration();
    if (!ReadValid(feedback)) return result_;
    telemetry_.feedbackValid = true;
    for (int i = 0; i < 8; ++i) {
      starts_[i] = feedback.motors[i].positionDeg;
      if (i < 4) telemetry_.leftVelocity += feedback.motors[i].velocityRpm * cmPerSecPerRpm_ / 4;
      else telemetry_.rightVelocity += feedback.motors[i].velocityRpm * cmPerSecPerRpm_ / 4;
      if (std::fabs(feedback.motors[i].velocityRpm * cmPerSecPerRpm_) >
          options.velocityToleranceCmPerSec) return Finish(MoveResult::NotStationary);
    }
    distance_ = distanceCm;
    started_ = previous_ = now;
    settledSince_ = -1;
    telemetry_.phase = distanceCm == 0 ? "SETTLE" : "ACCEL";
    return Finish(distanceCm == 0 ? MoveResult::Completed : MoveResult::Running);
  }

  MoveResult Step(const DriveFeedback& feedback, double now, bool permitted) {
    if (result_ != MoveResult::Running) return result_;
    if (!permitted) return Finish(MoveResult::Interrupted);
    if (!std::isfinite(now) || now < previous_) return Finish(MoveResult::InvalidFeedback);
    const double dt = now - previous_;
    if (dt > options_.maxUpdateIntervalSec) return Finish(MoveResult::Interrupted);
    const double elapsed = now - started_;
    telemetry_.elapsedSec = elapsed;
    telemetry_.reference = profile_.sample(elapsed);
    telemetry_.phase = elapsed < profile_.accelerationTime() ? "ACCEL" :
        elapsed < profile_.accelerationTime() + profile_.cruiseTime() ? "CRUISE" :
        elapsed < profile_.duration() ? "DECEL" : "SETTLE";
    if (elapsed >= timeout_) return Finish(MoveResult::TimedOut);
    if (!ReadValid(feedback)) return result_;
    double position[2] = {}, velocity[2] = {};
    bool motorsStopped = true;
    for (int i = 0; i < 8; ++i) {
      position[i / 4] += (feedback.motors[i].positionDeg - starts_[i]) * cmPerDegree_ / 4;
      const double speed = feedback.motors[i].velocityRpm * cmPerSecPerRpm_;
      velocity[i / 4] += speed / 4;
      if (std::fabs(speed) > options_.velocityToleranceCmPerSec) motorsStopped = false;
    }
    if (!std::isfinite(position[0]) || !std::isfinite(position[1]) ||
        !std::isfinite(velocity[0]) || !std::isfinite(velocity[1])) {
      telemetry_.feedbackValid = false;
      return Finish(MoveResult::InvalidFeedback);
    }
    telemetry_.feedbackValid = true;
    telemetry_.leftPosition = position[0];
    telemetry_.rightPosition = position[1];
    telemetry_.leftVelocity = velocity[0];
    telemetry_.rightVelocity = velocity[1];
    const bool settled = elapsed >= profile_.duration() && motorsStopped &&
        std::fabs(distance_ - position[0]) <= options_.positionToleranceCm &&
        std::fabs(distance_ - position[1]) <= options_.positionToleranceCm;
    if (!settled) settledSince_ = -1;
    else if (settledSince_ < 0) settledSince_ = now;
    if (settled && now - settledSince_ >= options_.settleTimeSec) {
      return Finish(MoveResult::Completed);
    }
    const ProfileState reference = profile_.sample(elapsed);
    const double correction = options_.synchronizationKp * (position[0] - position[1]);
    double left = reference.velocity + options_.positionKp * (reference.position - position[0]) - correction;
    double right = reference.velocity + options_.positionKp * (reference.position - position[1]) + correction;
    if (!std::isfinite(left) || !std::isfinite(right)) return Finish(MoveResult::InvalidFeedback);
    const double largest = std::fmax(std::fabs(left), std::fabs(right));
    if (largest > options_.maxSpeedCmPerSec) {
      const double scale = options_.maxSpeedCmPerSec / largest;
      left *= scale;
      right *= scale;
    }
    command_.leftRpm = limitVelocity(command_.leftRpm * cmPerSecPerRpm_, left,
        options_.accelerationCmPerSec2, options_.decelerationCmPerSec2, dt) / cmPerSecPerRpm_;
    command_.rightRpm = limitVelocity(command_.rightRpm * cmPerSecPerRpm_, right,
        options_.accelerationCmPerSec2, options_.decelerationCmPerSec2, dt) / cmPerSecPerRpm_;
    if (!std::isfinite(command_.leftRpm) || !std::isfinite(command_.rightRpm)) {
      return Finish(MoveResult::InvalidFeedback);
    }
    previous_ = now;
    return result_;
  }

  MoveResult Cancel() {
    if (result_ == MoveResult::Running) Finish(MoveResult::Interrupted);
    command_ = DriveCommand();
    return result_;
  }
  MoveResult result() const { return result_; }
  DriveCommand command() const { return command_; }
  DriveTelemetry telemetry() const {
    DriveTelemetry value = telemetry_;
    value.result = result_;
    value.command = command_;
    return value;
  }

 private:
  static bool Positive(double value) { return std::isfinite(value) && value > 0; }
  static bool Nonnegative(double value) { return std::isfinite(value) && value >= 0; }
  bool ReadValid(const DriveFeedback& feedback) {
    for (int i = 0; i < 8; ++i) {
      if (!feedback.motors[i].online) {
        telemetry_.feedbackValid = false;
        Finish(MoveResult::MotorDisconnected);
        return false;
      }
      if (!std::isfinite(feedback.motors[i].positionDeg) ||
          !std::isfinite(feedback.motors[i].velocityRpm)) {
        telemetry_.feedbackValid = false;
        Finish(MoveResult::InvalidFeedback);
        return false;
      }
    }
    return true;
  }
  MoveResult Finish(MoveResult result) {
    result_ = result;
    command_ = DriveCommand();
    return result_;
  }
  TrapezoidOptions options_;
  TrapezoidProfile profile_{0, 1, 1, 1};
  DriveCommand command_;
  DriveTelemetry telemetry_;
  MoveResult result_ = MoveResult::Idle;
  double starts_[8] = {};
  double distance_ = 0, started_ = 0, previous_ = 0, settledSince_ = -1;
  double timeout_ = 0, cmPerDegree_ = 0, cmPerSecPerRpm_ = 0;
};
}  // namespace motion
