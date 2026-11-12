#include "robot-config.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

using motion::MoveResult;
const double PI = 3.14159265358979323846;
const double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
const double INF_VALUE = std::numeric_limits<double>::infinity();

static void near(double actual, double expected, double tolerance = 1e-8) {
  assert(std::isfinite(actual));
  assert(std::fabs(actual - expected) <= tolerance);
}
static motion::TrapezoidOptions options() {
  motion::TrapezoidOptions value;
  value.wheelDiameterCm = 10;
  return value;
}
static motion::DriveFeedback feedback() {
  motion::DriveFeedback value;
  for (int i = 0; i < 8; ++i) {
    value.motors[i].online = true;
    value.motors[i].positionDeg = i * 120 - 300;
  }
  return value;
}
static void zeroCommand(const motion::DriveMotion& controller) {
  near(controller.command().leftRpm, 0);
  near(controller.command().rightRpm, 0);
}

static void profileMath() {
  motion::TrapezoidProfile known(100, 40, 40, 40);
  assert(known.valid());
  near(known.duration(), 3.5);
  near(known.sample(0.5).position, 5);
  near(known.sample(0.5).velocity, 20);
  near(known.sample(3).position, 95);
  near(known.sample(3).velocity, 20);
  motion::TrapezoidProfile triangle(10, 40, 40, 40);
  near(triangle.peakVelocity(), 20);
  near(triangle.cruiseTime(), 0);
  near(triangle.duration(), 1);
  motion::TrapezoidProfile asymmetric(80, 40, 80, 20);
  near(asymmetric.duration(), 3.25);
  near(asymmetric.accelerationTime(), 0.5);
  near(asymmetric.decelerationTime(), 2);
  near(motion::TrapezoidProfile(40, 40, 40, 40).cruiseTime(), 0);
  near(motion::limitVelocity(10, -10, 20, 10, 1.25), -5);
  near(motion::limitVelocity(-10, 10, 20, 10, 1.25), 5);
  near(motion::limitVelocity(20, 0, 10, 5, 0.1), 19.5);
  int count = 0;
  for (double distance : {0.001, 0.1, 1., 5., 20., 100., 1000.})
  for (double maximum : {0.5, 10., 50., 100.})
  for (double acceleration : {1., 10., 70.})
  for (double deceleration : {2., 30., 100.})
  for (double sign : {-1., 1.}) {
    motion::TrapezoidProfile curve(sign * distance, maximum, acceleration, deceleration);
    assert(curve.valid());
    const double dt = curve.duration() / 2000;
    auto previous = curve.sample(0);
    double integral = 0;
    for (int i = 1; i <= 2000; ++i) {
      const auto state = curve.sample(i * dt);
      assert(sign * (state.position - previous.position) >= -1e-8);
      assert(std::fabs(state.velocity) <= maximum + 1e-8);
      const double change = sign * (state.velocity - previous.velocity);
      assert(change <= acceleration * dt + 1e-8);
      assert(change >= -deceleration * dt - 1e-8);
      integral += (previous.velocity + state.velocity) * dt / 2;
      previous = state;
    }
    near(previous.position, sign * distance, 1e-7);
    near(previous.velocity, 0);
    near(integral, sign * distance, distance * 0.0006 + 1e-8);
    ++count;
  }
  assert(count == 504);
  for (double invalid : {0., -1., NAN_VALUE, INF_VALUE}) {
    assert(!motion::TrapezoidProfile(10, invalid, 20, 20).valid());
    assert(!motion::TrapezoidProfile(10, 20, invalid, 20).valid());
    assert(!motion::TrapezoidProfile(10, 20, 20, invalid).valid());
  }
  assert(!motion::TrapezoidProfile(NAN_VALUE, 20, 20, 20).valid());
  assert(!motion::TrapezoidProfile(INF_VALUE, 20, 20, 20).valid());
  assert(!motion::TrapezoidProfile(10, 1e300, 20, 20).valid());
}

static void eightMotorFollower() {
  for (double distance : {3., 100., -60.}) {
    auto config = options();
    config.motorRevsPerWheelRev = 2;
    config.decelerationCmPerSec2 = 20;
    auto state = feedback();
    const auto initial = state;
    motion::DriveMotion controller;
    double now = 40;
    assert(controller.Start(distance, config, state, now, true) == MoveResult::Running);
    const double scale = PI * config.wheelDiameterCm / (60 * config.motorRevsPerWheelRev);
    for (int step = 0; controller.result() == MoveResult::Running && step < 4000; ++step) {
      const double dt = step % 2 ? 0.017 : 0.010;
      const auto previous = controller.command();
      for (int i = 0; i < 8; ++i) {
        auto& motor = state.motors[i];
        motor.velocityRpm = (i < 4 ? previous.leftRpm : previous.rightRpm) * (0.85 + i * 0.02);
        motor.positionDeg += motor.velocityRpm * 6 * dt;
      }
      now += dt;
      controller.Step(state, now, true);
      if (controller.result() != MoveResult::Running) break;
      const auto command = controller.command();
      const double before[] = {previous.leftRpm * scale, previous.rightRpm * scale};
      const double after[] = {command.leftRpm * scale, command.rightRpm * scale};
      for (int side = 0; side < 2; ++side) {
        assert(std::fabs(after[side]) <= config.maxSpeedCmPerSec + 1e-8);
        if (before[side] * after[side] < 0) {
          assert(std::fabs(before[side]) / config.decelerationCmPerSec2 +
                 std::fabs(after[side]) / config.accelerationCmPerSec2 <= dt + 1e-8);
        } else {
          const double rate = std::fabs(after[side]) > std::fabs(before[side])
              ? config.accelerationCmPerSec2 : config.decelerationCmPerSec2;
          assert(std::fabs(after[side] - before[side]) <= rate * dt + 1e-8);
        }
      }
    }
    assert(controller.result() == MoveResult::Completed);
    zeroCommand(controller);
    for (int side = 0; side < 2; ++side) {
      double position = 0;
      for (int i = side * 4; i < side * 4 + 4; ++i)
        position += (state.motors[i].positionDeg - initial.motors[i].positionDeg) * scale / 24;
      near(position, distance, config.positionToleranceCm);
    }
    // 完成后不重启；禁用和后续 Step 不恢复速度输出。
    assert(controller.Step(state, now + 1, false) == MoveResult::Completed);
    zeroCommand(controller);
  }
}

static void invalidConfiguration() {
  double motion::TrapezoidOptions::*positive[] = {
    &motion::TrapezoidOptions::wheelDiameterCm, &motion::TrapezoidOptions::motorRevsPerWheelRev,
    &motion::TrapezoidOptions::maxMotorRpm, &motion::TrapezoidOptions::maxSpeedCmPerSec,
    &motion::TrapezoidOptions::accelerationCmPerSec2, &motion::TrapezoidOptions::decelerationCmPerSec2,
    &motion::TrapezoidOptions::positionToleranceCm, &motion::TrapezoidOptions::velocityToleranceCmPerSec,
    &motion::TrapezoidOptions::maxUpdateIntervalSec
  };
  motion::DriveMotion controller;
  const auto state = feedback();
  for (auto field : positive) for (double value : {0., -1., NAN_VALUE, INF_VALUE}) {
    auto config = options(); config.*field = value;
    assert(controller.Start(10, config, state, 0, true) == MoveResult::InvalidParameters);
    zeroCommand(controller);
  }
  double motion::TrapezoidOptions::*nonnegative[] = {
    &motion::TrapezoidOptions::positionKp, &motion::TrapezoidOptions::synchronizationKp,
    &motion::TrapezoidOptions::settleTimeSec, &motion::TrapezoidOptions::timeoutSec
  };
  for (auto field : nonnegative) for (double value : {-1., NAN_VALUE, INF_VALUE}) {
    auto config = options(); config.*field = value;
    assert(controller.Start(10, config, state, 0, true) == MoveResult::InvalidParameters);
  }
  auto config = options(); config.maxSpeedCmPerSec = 1000;
  assert(controller.Start(10, config, state, 0, true) == MoveResult::InvalidParameters);
  assert(controller.Start(NAN_VALUE, options(), state, 0, true) == MoveResult::InvalidParameters);
  assert(controller.Start(10, options(), state, NAN_VALUE, true) == MoveResult::InvalidParameters);
  assert(controller.Start(0, options(), state, 0, true) == MoveResult::Completed);
  zeroCommand(controller);
}

static void faultsAndInterruption() {
  for (int index = 0; index < 8; ++index) {
    motion::DriveMotion controller;
    auto state = feedback();
    assert(controller.Start(10, options(), state, 0, true) == MoveResult::Running);
    state.motors[index].online = false;
    assert(controller.Step(state, 0.01, true) == MoveResult::MotorDisconnected);
    zeroCommand(controller);
    state = feedback(); state.motors[index].positionDeg = NAN_VALUE;
    assert(controller.Start(10, options(), state, 0, true) == MoveResult::InvalidFeedback);
    state = feedback();
    controller.Start(10, options(), state, 0, true);
    state.motors[index].velocityRpm = INF_VALUE;
    assert(controller.Step(state, 0.01, true) == MoveResult::InvalidFeedback);
  }
  auto state = feedback();
  motion::DriveMotion controller;
  controller.Start(100, options(), state, 0, true);
  assert(controller.Step(state, 0.1, false) == MoveResult::Interrupted);
  zeroCommand(controller);
  controller.Start(100, options(), state, 0, true);
  assert(controller.Step(state, 0.3, true) == MoveResult::Interrupted);
  controller.Start(100, options(), state, 1, true);
  assert(controller.Step(state, 0.9, true) == MoveResult::InvalidFeedback);
  auto config = options(); config.timeoutSec = 0.3;
  controller.Start(100, config, state, 0, true);
  for (int i = 1; i < 40 && controller.result() == MoveResult::Running; ++i)
    controller.Step(state, i * 0.01, true);
  assert(controller.result() == MoveResult::TimedOut);
  zeroCommand(controller);
  state.motors[5].velocityRpm = 30;
  assert(controller.Start(10, options(), state, 0, true) == MoveResult::NotStationary);
}

static void controllerTelemetry() {
  motion::DriveMotion controller;
  auto telemetry = controller.telemetry();
  assert(telemetry.result == MoveResult::Idle && !telemetry.feedbackValid);
  near(telemetry.command.leftRpm, 0);
  near(telemetry.command.rightRpm, 0);

  auto config = options();
  config.motorRevsPerWheelRev = 2;
  auto state = feedback();
  const auto initial = state;
  assert(controller.Start(100, config, state, 10, true) == MoveResult::Running);
  telemetry = controller.telemetry();
  assert(telemetry.result == MoveResult::Running && telemetry.feedbackValid);
  near(telemetry.targetPosition, 100);
  near(telemetry.rightTargetPosition, 100);
  near(telemetry.leftPosition, 0);
  near(telemetry.rightPosition, 0);
  near(telemetry.elapsedSec, 0);
  assert(std::strcmp(telemetry.phase, "ACCEL") == 0);

  // 每个电机的起点、位移及速度都不同，确认遥测使用八电机相对起点的侧均值。
  double expectedPosition[2] = {}, expectedVelocity[2] = {};
  const double cmPerDegree = PI * config.wheelDiameterCm / (360 * config.motorRevsPerWheelRev);
  for (int i = 0; i < 8; ++i) {
    state.motors[i].positionDeg = initial.motors[i].positionDeg + 2 + i;
    state.motors[i].velocityRpm = 4 + i;
    expectedPosition[i / 4] += (2 + i) * cmPerDegree / 4;
    expectedVelocity[i / 4] += (4 + i) * 6 * cmPerDegree / 4;
  }
  assert(controller.Step(state, 10.1, true) == MoveResult::Running);
  telemetry = controller.telemetry();
  near(telemetry.leftPosition, expectedPosition[0]);
  near(telemetry.rightPosition, expectedPosition[1]);
  near(telemetry.leftVelocity, expectedVelocity[0]);
  near(telemetry.rightVelocity, expectedVelocity[1]);
  near(telemetry.elapsedSec, 0.1);
  near(telemetry.reference.position, 0.15);
  near(telemetry.reference.velocity, 3);
  near(telemetry.command.leftRpm, controller.command().leftRpm);
  near(telemetry.command.rightRpm, controller.command().rightRpm);
  assert(telemetry.command.leftRpm != 0 && telemetry.command.rightRpm != 0);

  assert(controller.Cancel() == MoveResult::Interrupted);
  const auto cancelled = controller.telemetry();
  assert(cancelled.result == MoveResult::Interrupted && cancelled.feedbackValid);
  near(cancelled.command.leftRpm, 0);
  near(cancelled.command.rightRpm, 0);
  near(cancelled.leftPosition, telemetry.leftPosition);
  near(cancelled.rightPosition, telemetry.rightPosition);
  near(cancelled.elapsedSec, telemetry.elapsedSec);
  assert(controller.Step(feedback(), 11, true) == MoveResult::Interrupted);
  near(controller.telemetry().leftPosition, cancelled.leftPosition);

  state = feedback();
  controller.Start(100, config, state, 20, true);
  state.motors[6].online = false;
  assert(controller.Step(state, 20.01, true) == MoveResult::MotorDisconnected);
  assert(!controller.telemetry().feedbackValid);
  near(controller.telemetry().command.leftRpm, 0);
  state = feedback();
  state.motors[1].positionDeg = NAN_VALUE;
  assert(controller.Start(100, config, state, 20, true) == MoveResult::InvalidFeedback);
  assert(!controller.telemetry().feedbackValid);
}

static vex::motor* driveMotor(int i) {
  vex::motor* motors[] = {LF, LM, LBD, LBU, RF, RM, RBD, RBU};
  return motors[i];
}
static void resetHardware() {
  hardware::CancelDriveMotion();
  vex::simulation() = vex::Simulation();
  for (int i = 0; i < 8; ++i) {
    auto* motor = driveMotor(i);
    motor->encoder = i * 100; motor->commandRpm = 0;
    motor->responseScale = 1; motor->connected = true; motor->badFeedback = false;
  }
}
static void hardwareStopped() {
  for (int i = 0; i < 8; ++i) {
    near(driveMotor(i)->commandRpm, 0);
    assert(driveMotor(i)->lastStop == vex::brake);
  }
}
static void hardwareIntegration() {
  assert(hardware::Initialize());
  resetHardware();
  assert(hardware::StartDriveMotion(10) == MoveResult::InvalidParameters); // 未填轮径。
  hardwareStopped();
  for (double distance : {10., -10.}) {
    resetHardware();
    assert(hardware::StartDriveMotion(distance, options()) == MoveResult::Running);
    for (int step = 0; hardware::GetDriveMotionResult() == MoveResult::Running && step < 2000; ++step) {
      for (int i = 0; i < 8; ++i) driveMotor(i)->encoder += driveMotor(i)->commandRpm * 6 * 0.01;
      vex::simulation().now += 0.01;
      hardware::UpdateDriveMotion();
      near(LF->commandRpm, LM->commandRpm); near(LF->commandRpm, LBD->commandRpm);
      near(LF->commandRpm, LBU->commandRpm); near(RF->commandRpm, RM->commandRpm);
      near(RF->commandRpm, RBD->commandRpm); near(RF->commandRpm, RBU->commandRpm);
      manual::Output manual;
      manual.L_voltage = 12000; manual.R_voltage = -12000;
      const double previous = LF->commandRpm;
      hardware::Apply(manual, false);
      near(LF->commandRpm, previous); // 手控更新不能覆盖自动底盘输出。
    }
    assert(hardware::GetDriveMotionResult() == MoveResult::Completed);
    hardwareStopped();
    for (int i = 0; i < 8; ++i) {
      near((driveMotor(i)->encoder - i * 100) * PI * 10 / 360, distance, 0.5);
      assert(driveMotor(i)->resetCalls == 0);
    }
    hardware::UpdateDriveMotion(); hardwareStopped();
  }
  for (int index = 0; index < 8; ++index) {
    resetHardware(); hardware::StartDriveMotion(10, options());
    driveMotor(index)->connected = false;
    assert(hardware::UpdateDriveMotion() == MoveResult::MotorDisconnected);
    hardwareStopped();
  }
  resetHardware();
  vex::simulation().field = true;
  hardware::StartDriveMotion(10, options());
  vex::simulation().enabled = false;
  assert(hardware::UpdateDriveMotion() == MoveResult::Interrupted);
  hardwareStopped();
  vex::simulation().enabled = true; vex::simulation().autonomous = false;
  assert(hardware::StartDriveMotion(10, options()) == MoveResult::Interrupted);
  hardwareStopped();
  resetHardware();
  for (double value : {NAN_VALUE, INF_VALUE}) {
    auto config = options(); config.maxMotorRpm = value;
    assert(hardware::StartDriveMotion(10, config) == MoveResult::InvalidParameters);
    hardwareStopped();
  }
  auto overspeed = options();
  overspeed.maxMotorRpm = 1000000;
  overspeed.maxSpeedCmPerSec = 400; // 对应约 764 RPM，不能通过调高请求上限绕过 600 RPM 齿轮箱限制。
  assert(hardware::StartDriveMotion(10, overspeed) == MoveResult::InvalidParameters);
  hardwareStopped();
  manual::Output manual;
  manual.L_voltage = 3000; manual.R_voltage = -2000;
  manual.arm_enabled = true; manual.arm_voltage = 12000;
  manual.claw_voltage = -6000; manual.toggle_piston = true;
  hardware::Apply(manual);
  for (int i = 0; i < 8; ++i) assert(driveMotor(i)->voltage == (i < 4 ? 3000 : -2000));
  assert(ArmL->voltage == 12000 && ArmR->voltage == 12000);
  assert(Claw->voltage == -6000 && TogglePiston->value);
  hardware::Apply(manual::Output());
  assert(ArmL->lastStop == vex::hold && ArmR->lastStop == vex::hold);
}

static void driveTestIntegration() {
  // 普通厘米接口仍要求轮径；输出轴测试在默认未标定的配置下可直接运行。
  for (double revolutions : {3., -3., 0.25, -0.25}) {
    resetHardware();
    assert(hardware::StartDriveMotion(10) == MoveResult::InvalidParameters);
    assert(hardware::StartDriveTest(revolutions) == MoveResult::Running);
    const double distance = revolutions * 360;
    const bool longMove = std::fabs(revolutions) == 3;
    const motion::TrapezoidProfile profile(distance, 720, 720, 720);
    auto telemetry = hardware::GetDriveMotionTelemetry();
    near(telemetry.targetPosition, distance);
    near(telemetry.rightTargetPosition, distance);
    near(telemetry.durationSec, longMove ? 2.5 : std::sqrt(0.5));
    assert(telemetry.feedbackValid && telemetry.result == MoveResult::Running);
    bool accelerationSeen = false, cruiseSeen = false, decelerationSeen = false, settleSeen = false;
    double peakReferenceRpm = 0;
    for (int step = 0; hardware::GetDriveMotionResult() == MoveResult::Running && step < 1000; ++step) {
      const double previousLeft = LF->commandRpm, previousRight = RF->commandRpm;
      for (int i = 0; i < 8; ++i) driveMotor(i)->encoder += driveMotor(i)->commandRpm * 6 * 0.01;
      vex::simulation().now += 0.01;
      hardware::UpdateDriveMotion();
      telemetry = hardware::GetDriveMotionTelemetry();
      assert(telemetry.result == hardware::GetDriveMotionResult());
      assert(telemetry.feedbackValid);
      near(telemetry.elapsedSec, vex::simulation().now);
      near(telemetry.command.leftRpm, LF->commandRpm);
      near(telemetry.command.rightRpm, RF->commandRpm);
      // 反馈在写入本周期命令前采集，因此速度对应上周期输出，单位为 deg/s。
      near(telemetry.leftVelocity, previousLeft * 6);
      near(telemetry.rightVelocity, previousRight * 6);
      double position[2] = {};
      for (int i = 0; i < 8; ++i) {
        position[i / 4] += (driveMotor(i)->encoder - i * 100) / 4;
        near(driveMotor(i)->commandRpm, i < 4 ? telemetry.command.leftRpm : telemetry.command.rightRpm);
        assert(driveMotor(i)->resetCalls == 0);
      }
      near(telemetry.leftPosition, position[0]);
      near(telemetry.rightPosition, position[1]);
      const auto reference = profile.sample(telemetry.elapsedSec);
      near(telemetry.reference.position, reference.position);
      near(telemetry.reference.velocity, reference.velocity);
      assert(distance * telemetry.reference.position >= 0);
      assert(distance * telemetry.reference.velocity >= 0);
      peakReferenceRpm = std::fmax(peakReferenceRpm, std::fabs(reference.velocity) / 6);
      accelerationSeen |= std::strcmp(telemetry.phase, "ACCEL") == 0;
      cruiseSeen |= std::strcmp(telemetry.phase, "CRUISE") == 0;
      decelerationSeen |= std::strcmp(telemetry.phase, "DECEL") == 0;
      settleSeen |= std::strcmp(telemetry.phase, "SETTLE") == 0;
      if (telemetry.result == MoveResult::Running) {
        assert(std::fabs(telemetry.command.leftRpm) <= 120 + 1e-8);
        assert(std::fabs(telemetry.command.rightRpm) <= 120 + 1e-8);
        assert(std::fabs(telemetry.command.leftRpm - previousLeft) <= 1.2 + 1e-8);
        assert(std::fabs(telemetry.command.rightRpm - previousRight) <= 1.2 + 1e-8);
      }
    }
    assert(telemetry.result == MoveResult::Completed);
    assert(accelerationSeen && decelerationSeen && settleSeen);
    assert(cruiseSeen == longMove);
    if (longMove) near(peakReferenceRpm, 120);
    else assert(peakReferenceRpm > 40 && peakReferenceRpm < 43);
    near(telemetry.leftPosition, distance, 3);
    near(telemetry.rightPosition, distance, 3);
    near(telemetry.command.leftRpm, 0);
    near(telemetry.command.rightRpm, 0);
    hardwareStopped();
    const auto completed = telemetry;
    vex::simulation().now += 1;
    assert(hardware::UpdateDriveMotion() == MoveResult::Completed);
    telemetry = hardware::GetDriveMotionTelemetry();
    near(telemetry.leftPosition, completed.leftPosition);
    near(telemetry.rightPosition, completed.rightPosition);
    near(telemetry.elapsedSec, completed.elapsedSec);
    hardwareStopped();
  }

  resetHardware();
  hardware::StartDriveTest(3);
  for (int step = 0; step < 10; ++step) {
    for (int i = 0; i < 8; ++i) driveMotor(i)->encoder += driveMotor(i)->commandRpm * 6 * 0.01;
    vex::simulation().now += 0.01;
    hardware::UpdateDriveMotion();
  }
  const auto moving = hardware::GetDriveMotionTelemetry();
  assert(moving.leftPosition > 0 && moving.command.leftRpm > 0);
  assert(hardware::CancelDriveMotion() == MoveResult::Interrupted);
  auto stopped = hardware::GetDriveMotionTelemetry();
  assert(stopped.result == MoveResult::Interrupted);
  near(stopped.leftPosition, moving.leftPosition);
  near(stopped.rightPosition, moving.rightPosition);
  near(stopped.command.leftRpm, 0);
  near(stopped.command.rightRpm, 0);
  hardwareStopped();

  resetHardware();
  hardware::StartDriveTest(3);
  driveMotor(7)->connected = false;
  assert(hardware::UpdateDriveMotion() == MoveResult::MotorDisconnected);
  assert(!hardware::GetDriveMotionTelemetry().feedbackValid);
  hardwareStopped();

  // 场控已连接时，即使处于自动时段，测试入口也不能启动或保留现有输出。
  resetHardware();
  hardware::StartDriveTest(3);
  vex::simulation().now += 0.01;
  hardware::UpdateDriveMotion();
  assert(LF->commandRpm > 0);
  vex::simulation().field = true;
  assert(hardware::StartDriveTest(3) == MoveResult::Interrupted);
  hardwareStopped();
}

static void advanceHardware(double dt = 0.01) {
  for (int i = 0; i < 8; ++i) driveMotor(i)->encoder += driveMotor(i)->commandRpm * 6 * dt;
  vex::simulation().now += dt;
  hardware::UpdateDriveMotion();
}

static void driveTurnIntegration() {
  for (double revolutions : {0.75, -0.75}) {
    resetHardware();
    assert(hardware::StartDriveTurn(revolutions) == MoveResult::Running);
    const double distance = revolutions * 360;
    const motion::TrapezoidProfile profile(distance, 360, 360, 360);
    auto telemetry = hardware::GetDriveMotionTelemetry();
    near(telemetry.targetPosition, distance);
    near(telemetry.rightTargetPosition, -distance);
    near(telemetry.durationSec, std::sqrt(3.0));
    bool oppositeCommandsSeen = false;
    for (int step = 0; hardware::GetDriveMotionResult() == MoveResult::Running && step < 1000; ++step) {
      const double previousLeft = LF->commandRpm, previousRight = RF->commandRpm;
      advanceHardware();
      telemetry = hardware::GetDriveMotionTelemetry();
      assert(telemetry.feedbackValid);
      assert(telemetry.result == hardware::GetDriveMotionResult());
      near(telemetry.command.leftRpm, LF->commandRpm);
      near(telemetry.command.rightRpm, RF->commandRpm);
      near(telemetry.leftVelocity, previousLeft * 6);
      near(telemetry.rightVelocity, previousRight * 6);
      near(telemetry.command.leftRpm, -telemetry.command.rightRpm);
      const auto reference = profile.sample(telemetry.elapsedSec);
      near(telemetry.reference.position, reference.position);
      near(telemetry.reference.velocity, reference.velocity);
      double positions[2] = {};
      for (int i = 0; i < 8; ++i) {
        positions[i / 4] += (driveMotor(i)->encoder - i * 100) / 4;
        near(driveMotor(i)->commandRpm, i < 4 ? telemetry.command.leftRpm : telemetry.command.rightRpm);
        assert(driveMotor(i)->resetCalls == 0);
      }
      near(telemetry.leftPosition, positions[0]);
      near(telemetry.rightPosition, positions[1]);
      near(telemetry.leftPosition, -telemetry.rightPosition);
      if (std::fabs(telemetry.command.leftRpm) > 1) {
        assert(telemetry.command.leftRpm * telemetry.command.rightRpm < 0);
        oppositeCommandsSeen = true;
      }
      if (telemetry.result == MoveResult::Running) {
        assert(std::fabs(telemetry.command.leftRpm) <= 60 + 1e-8);
        assert(std::fabs(telemetry.command.rightRpm) <= 60 + 1e-8);
        assert(std::fabs(telemetry.command.leftRpm - previousLeft) <= 0.6 + 1e-8);
        assert(std::fabs(telemetry.command.rightRpm - previousRight) <= 0.6 + 1e-8);
      }
    }
    assert(telemetry.result == MoveResult::Completed && oppositeCommandsSeen);
    near(telemetry.leftPosition, distance, 3);
    near(telemetry.rightPosition, -distance, 3);
    near(telemetry.reference.position, distance);
    near(telemetry.reference.velocity, 0);
    assert(telemetry.leftPosition * distance > 0 && telemetry.rightPosition * distance < 0);
    hardwareStopped();
    const auto completed = telemetry;
    assert(hardware::UpdateDriveMotion() == MoveResult::Completed);
    near(hardware::GetDriveMotionTelemetry().rightPosition, completed.rightPosition);

    // 转弯之后调用两种直线接口，右侧目标与实际输出都必须恢复为正向。
    assert(hardware::StartDriveTest(0.25) == MoveResult::Running);
    near(hardware::GetDriveMotionTelemetry().rightTargetPosition, 90);
    advanceHardware();
    assert(LF->commandRpm > 0 && RF->commandRpm > 0);
    hardware::CancelDriveMotion();
    assert(hardware::StartDriveTurn(0.75) == MoveResult::Running);
    advanceHardware();
    assert(LF->commandRpm > 0 && RF->commandRpm < 0);
    hardware::CancelDriveMotion();
    assert(hardware::StartDriveMotion(10, options()) == MoveResult::Running);
    near(hardware::GetDriveMotionTelemetry().rightTargetPosition, 10);
    advanceHardware();
    assert(LF->commandRpm > 0 && RF->commandRpm > 0);
    hardware::CancelDriveMotion();
    hardwareStopped();
  }

  for (int i = 0; i < 8; ++i) {
    resetHardware();
    driveMotor(i)->connected = false;
    assert(hardware::StartDriveTurn(0.75) == MoveResult::MotorDisconnected);
    assert(!hardware::GetDriveMotionTelemetry().feedbackValid);
    hardwareStopped();
    resetHardware();
    assert(hardware::StartDriveTurn(0.75) == MoveResult::Running);
    advanceHardware();
    driveMotor(i)->connected = false;
    assert(hardware::UpdateDriveMotion() == MoveResult::MotorDisconnected);
    assert(!hardware::GetDriveMotionTelemetry().feedbackValid);
    hardwareStopped();
  }
  for (double invalid : {NAN_VALUE, INF_VALUE}) {
    resetHardware();
    assert(hardware::StartDriveTurn(invalid) == MoveResult::InvalidParameters);
    hardwareStopped();
  }
  resetHardware();
  vex::simulation().field = true;
  assert(hardware::StartDriveTurn(0.75) != MoveResult::Running);
  hardwareStopped();
  resetHardware();
  assert(hardware::StartDriveTurn(0.75) == MoveResult::Running);
  advanceHardware();
  vex::simulation().field = true;
  assert(hardware::UpdateDriveMotion() == MoveResult::Interrupted);
  hardwareStopped();
  resetHardware();
  assert(hardware::StartDriveTest(3) == MoveResult::Running);
  advanceHardware();
  vex::simulation().field = true;
  assert(hardware::UpdateDriveMotion() == MoveResult::Interrupted);
  hardwareStopped();
}

int main() {
  profileMath();
  eightMotorFollower();
  invalidConfiguration();
  faultsAndInterruption();
  controllerTelemetry();
  hardwareIntegration();
  driveTestIntegration();
  driveTurnIntegration();
  std::puts("Trapezoid checks passed: 504 profiles, 8-motor forward/reverse, limits, faults, telemetry, straight and turn tests.");
}
