#include "robot-config.h"
#include "control-config.h"

vex::brain Brain;
vex::controller Controller(vex::primary);
vex::competition Competition;

vex::motor* LF = nullptr;
vex::motor* LM = nullptr;
vex::motor* LBD = nullptr;
vex::motor* LBU = nullptr;
vex::motor* RF = nullptr;
vex::motor* RM = nullptr;
vex::motor* RBD = nullptr;
vex::motor* RBU = nullptr;
vex::motor* ArmL = nullptr;
vex::motor* ArmR = nullptr;
vex::motor* Claw = nullptr;
vex::digital_out* TogglePiston = nullptr;

namespace {
// 统一遍历具名设备；通过指针地址表写回各设备变量。
vex::motor** const LeftDrive[4] = {&LF, &LM, &LBD, &LBU};
vex::motor** const RightDrive[4] = {&RF, &RM, &RBD, &RBU};
bool arm_moving = false;
bool drive_turning = false;
bool drive_test_motion = false;
motion::DriveMotion drive_motion;
vex::timer drive_motion_clock;

bool MotionPermitted() {
  const bool connected = Competition.isFieldControl() || Competition.isCompetitionSwitch();
  if (drive_test_motion) return !connected;
  return !connected || (Competition.isEnabled() && Competition.isAutonomous());
}

motion::DriveFeedback ReadDriveFeedback() {
  motion::DriveFeedback feedback;
  for (int i = 0; i < 8; ++i) {
    vex::motor* motor = i < 4 ? *LeftDrive[i] : *RightDrive[i - 4];
    feedback.motors[i].online = motor && motor->installed();
    if (feedback.motors[i].online) {
      feedback.motors[i].positionDeg = motor->position(vex::deg);
      feedback.motors[i].velocityRpm = motor->velocity(vex::rpm);
      if (drive_turning && i >= 4) {
        feedback.motors[i].positionDeg *= -1;
        feedback.motors[i].velocityRpm *= -1;
      }
    }
  }
  return feedback;
}

double GearMaxRpm(vex::gearSetting gear) {
  switch (gear) {
    case vex::gearSetting::ratio6_1: return 600;
    case vex::gearSetting::ratio18_1: return 200;
    case vex::gearSetting::ratio36_1: return 100;
    default: return 0;
  }
}

void ApplyDriveMotion() {
  const motion::DriveCommand command = drive_motion.command();
  for (int i = 0; i < 8; ++i) {
    vex::motor* motor = i < 4 ? *LeftDrive[i] : *RightDrive[i - 4];
    if (!motor) continue;
    if (drive_motion.result() != motion::MoveResult::Running) motor->stop(vex::brake);
    else {
      const double rpm = i < 4 ? command.leftRpm : command.rightRpm * (drive_turning ? -1 : 1);
      motor->spin(rpm < 0 ? vex::directionType::rev : vex::directionType::fwd,
                  std::fabs(rpm), vex::rpm);
    }
  }
}

motion::MoveResult StartMotion(double distance, const motion::TrapezoidOptions& requested,
                               bool turning, bool test_motion) {
  drive_turning = turning;
  drive_test_motion = test_motion;
  motion::TrapezoidOptions options = requested;
  // fmin 会吞掉 NaN；保留非有限请求，让规划器明确拒绝非法参数。
  if (std::isfinite(options.maxMotorRpm)) {
    for (int i = 0; i < 4; ++i) {
      options.maxMotorRpm = std::fmin(options.maxMotorRpm, GearMaxRpm(config::LEFT_DRIVE[i].gear));
      options.maxMotorRpm = std::fmin(options.maxMotorRpm, GearMaxRpm(config::RIGHT_DRIVE[i].gear));
    }
  }
  const motion::DriveFeedback feedback = ReadDriveFeedback();
  const motion::MoveResult result = drive_motion.Start(
      distance, options, feedback, drive_motion_clock.time(vex::sec), MotionPermitted());
  ApplyDriveMotion();
  return result;
}

motion::TrapezoidOptions DriveTestOptions(double max_rpm, double acceleration, double deceleration) {
  // circumference=360、传动比=1，使每个编码器度对应一个规划单位。
  // 这里的等效轮径仅用于单位换算，不作为实际厘米距离。
  motion::TrapezoidOptions options;
  options.wheelDiameterCm = 360.0 / 3.14159265358979323846;
  options.motorRevsPerWheelRev = 1;
  options.maxSpeedCmPerSec = max_rpm * 6;
  options.accelerationCmPerSec2 = acceleration * 6;
  options.decelerationCmPerSec2 = deceleration * 6;
  options.positionToleranceCm = 3;
  options.velocityToleranceCmPerSec = 6;
  return options;
}

bool RegisterPort(int port, bool required, bool (&used)[22]) {
  if (port == 0 && !required) return true;
  if (port < 1 || port > 21) {
    Brain.Screen.print("Assign smart ports 1-21");
    return false;
  }
  if (used[port]) {
    Brain.Screen.print("Duplicate smart port: %d", port);
    return false;
  }
  used[port] = true;
  return true;
}

vex::motor* CreateMotor(const config::Motor& setting, vex::brakeType brake) {
  if (setting.port == 0) return nullptr;
  // SDK 端口索引从 0 开始；配置文件使用主控上标注的端口编号。
  vex::motor* motor = new vex::motor(setting.port - 1, setting.gear, setting.reversed);
  motor->setStopping(brake);
  motor->stop();
  return motor;
}

vex::triport::port& ThreeWirePort(char port) {
  switch (port) {
    case 'A': return Brain.ThreeWirePort.A;
    case 'B': return Brain.ThreeWirePort.B;
    case 'C': return Brain.ThreeWirePort.C;
    case 'D': return Brain.ThreeWirePort.D;
    case 'E': return Brain.ThreeWirePort.E;
    case 'F': return Brain.ThreeWirePort.F;
    case 'G': return Brain.ThreeWirePort.G;
    default: return Brain.ThreeWirePort.H;  // 仅在端口校验通过后调用。
  }
}

int ReadAxis(int axis) {
  switch (axis) {
    case 1: return Controller.Axis1.position();
    case 2: return Controller.Axis2.position();
    case 3: return Controller.Axis3.position();
    case 4: return Controller.Axis4.position();
    default: return 0;
  }
}

bool Pressed(config::Button button) {
  switch (button) {
    case config::Button::R1: return Controller.ButtonR1.pressing();
    case config::Button::R2: return Controller.ButtonR2.pressing();
    case config::Button::L1: return Controller.ButtonL1.pressing();
    case config::Button::L2: return Controller.ButtonL2.pressing();
    case config::Button::A: return Controller.ButtonA.pressing();
    case config::Button::B: return Controller.ButtonB.pressing();
    case config::Button::X: return Controller.ButtonX.pressing();
    case config::Button::Y: return Controller.ButtonY.pressing();
    case config::Button::Up: return Controller.ButtonUp.pressing();
    case config::Button::Down: return Controller.ButtonDown.pressing();
    case config::Button::Left: return Controller.ButtonLeft.pressing();
    case config::Button::Right: return Controller.ButtonRight.pressing();
  }
  return false;
}

void SetVoltage(vex::motor* motor, int millivolts) {
  if (!motor) return;
  if (millivolts == 0) motor->stop();
  else motor->spin(vex::fwd, millivolts, vex::voltageUnits::mV);
}
}  // 匿名命名空间

namespace hardware {
bool Initialize() {
  // 先检查全部配置，再创建设备或写入输出。
  bool used[22] = {};
  Brain.Screen.setCursor(2, 1);
  for (int i = 0; i < 4; ++i) {
    if (!RegisterPort(config::LEFT_DRIVE[i].port, true, used) ||
        !RegisterPort(config::RIGHT_DRIVE[i].port, true, used)) return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (!RegisterPort(config::STRUCTURE_MOTORS[i].port, true, used)) return false;
  }
  if (!RegisterPort(config::INERTIAL_PORT, false, used)) return false;

  if (config::TogglePiston < 'A' || config::TogglePiston > 'H') {
    Brain.Screen.print("Invalid three-wire port");
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    *LeftDrive[i] = CreateMotor(config::LEFT_DRIVE[i], config::BRAKE_MODE);
    *RightDrive[i] = CreateMotor(config::RIGHT_DRIVE[i], config::BRAKE_MODE);
  }
  ArmL = CreateMotor(config::ArmL, config::ARM_BRAKE_MODE);
  ArmR = CreateMotor(config::ArmR, config::ARM_BRAKE_MODE);
  // 编码器归零仅用于观察位移，不参与电压控制，也不要求从机械起点启动。
  if (ArmL) {
    ArmL->setPosition(0, vex::deg);
  }
  if (ArmR) {
    ArmR->setPosition(0, vex::deg);
  }
  Claw = CreateMotor(config::Claw, config::CLAW_BRAKE_MODE);
  TogglePiston = new vex::digital_out(ThreeWirePort(config::TogglePiston));
  TogglePiston->set(config::PISTON_INITIAL_STATE);
  return true;
}

manual::Input ReadInput() {
  manual::Input input;
  input.y_axis = ReadAxis(config::YAxis);
  input.angular_axis = ReadAxis(config::AngularAxis);
  input.arm_ready = ArmL && ArmR && ArmL->installed() && ArmR->installed();
  input.arm_up = Pressed(config::ArmUp);
  input.arm_down = Pressed(config::ArmDown);
  input.claw_close = Pressed(config::ClawClose);
  input.claw_open = Pressed(config::ClawOpen);
  input.toggle = Pressed(config::Toggle);
  return input;
}

void Apply(const manual::Output& output, bool manual_drive) {
  if (manual_drive) {
    for (int i = 0; i < 4; ++i) {
      SetVoltage(*LeftDrive[i], output.L_voltage);
      SetVoltage(*RightDrive[i], output.R_voltage);
    }
  }
  const bool arm_ready = ArmL && ArmR && ArmL->installed() && ArmR->installed();
  const int voltage = output.arm_enabled && arm_ready ? output.arm_voltage : 0;
  if (voltage != 0) {
    // 两台电机同周期接收电压，左右安装方向由各自的反转配置处理。
    ArmL->spin(vex::fwd, voltage * config::ARM_UP_DIRECTION, vex::voltageUnits::mV);
    ArmR->spin(vex::fwd, voltage * config::ARM_UP_DIRECTION, vex::voltageUnits::mV);
    arm_moving = true;
  } else if (arm_moving) {
    // 仅在停止的瞬间记录内置 hold 位置，避免循环改写保持点。
    if (ArmL) ArmL->stop(config::ARM_BRAKE_MODE);
    if (ArmR) ArmR->stop(config::ARM_BRAKE_MODE);
    arm_moving = false;
  }
  SetVoltage(Claw, output.claw_voltage);
  if (TogglePiston) TogglePiston->set(output.toggle_piston);
}
void GetDriveMotorCount(int& left_count, int& right_count) {
  left_count = 0;
  right_count = 0;
  for (int i = 0; i < 4; ++i) {
    if (*LeftDrive[i] && (*LeftDrive[i])->installed()) ++left_count;
    if (*RightDrive[i] && (*RightDrive[i])->installed()) ++right_count;
  }
}

motion::MoveResult StartDriveMotion(double distance_cm) {
  motion::TrapezoidOptions options;
  options.wheelDiameterCm = config::DRIVE_WHEEL_DIAMETER_CM;
  options.motorRevsPerWheelRev = config::DRIVE_MOTOR_REVS_PER_WHEEL_REV;
  options.maxSpeedCmPerSec = config::DRIVE_MAX_SPEED_CM_PER_SEC;
  options.accelerationCmPerSec2 = config::DRIVE_ACCELERATION_CM_PER_SEC2;
  options.decelerationCmPerSec2 = config::DRIVE_DECELERATION_CM_PER_SEC2;
  return StartDriveMotion(distance_cm, options);
}

motion::MoveResult StartDriveMotion(double distance_cm, const motion::TrapezoidOptions& requested) {
  return StartMotion(distance_cm, requested, false, false);
}

motion::MoveResult StartDriveTest(double motor_revolutions) {
  drive_turning = false;
  if (Competition.isFieldControl() || Competition.isCompetitionSwitch()) {
    return CancelDriveMotion();
  }
  const auto options = DriveTestOptions(config::DRIVE_TEST_MAX_RPM,
      config::DRIVE_TEST_ACCEL_RPM_PER_SEC, config::DRIVE_TEST_DECEL_RPM_PER_SEC);
  return StartMotion(motor_revolutions * 360, options, false, true);
}

motion::MoveResult StartDriveTurn(double motor_revolutions) {
  if (Competition.isFieldControl() || Competition.isCompetitionSwitch()) {
    return CancelDriveMotion();
  }
  const auto options = DriveTestOptions(config::DRIVE_TEST_TURN_MAX_RPM,
      config::DRIVE_TEST_TURN_ACCEL_RPM_PER_SEC, config::DRIVE_TEST_TURN_DECEL_RPM_PER_SEC);
  return StartMotion(motor_revolutions * 360, options, true, true);
}

motion::MoveResult UpdateDriveMotion() {
  if (drive_motion.result() == motion::MoveResult::Running) {
    const motion::DriveFeedback feedback = ReadDriveFeedback();
    drive_motion.Step(feedback, drive_motion_clock.time(vex::sec), MotionPermitted());
  }
  ApplyDriveMotion();
  return drive_motion.result();
}

motion::MoveResult CancelDriveMotion() {
  const motion::MoveResult result = drive_motion.Cancel();
  ApplyDriveMotion();
  return result;
}

motion::MoveResult GetDriveMotionResult() { return drive_motion.result(); }
motion::DriveTelemetry GetDriveMotionTelemetry() {
  motion::DriveTelemetry telemetry = drive_motion.telemetry();
  if (drive_turning) {
    telemetry.rightTargetPosition *= -1;
    telemetry.rightPosition *= -1;
    telemetry.rightVelocity *= -1;
    telemetry.command.rightRpm *= -1;
  }
  return telemetry;
}
}  // 硬件接口命名空间
