#include "robot-config.h"
#include "control-config.h"
#include <cmath>

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
vex::inertial* Inertial = nullptr;
vex::timer InertialTimer;
bool inertial_calibration_started = false;

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
  if (config::INERTIAL_PORT != 0) {
    Inertial = new vex::inertial(config::INERTIAL_PORT - 1, vex::turnType::left);
    if (Inertial->installed()) {
      Inertial->calibrate();
      InertialTimer.reset();
      inertial_calibration_started = true;
    }
  }
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
  input.turn_test = Pressed(config::TurnTest);
  input.square_test = Pressed(config::SquareTest);
  input.telemetry = Pressed(config::Telemetry);
  input.voltage_up = Pressed(config::VoltageUp);
  input.voltage_down = Pressed(config::VoltageDown);
  input.voltage_forward = Pressed(config::VoltageForward);
  input.voltage_reverse = Pressed(config::VoltageReverse);
  return input;
}

void Apply(const manual::Output& output, bool manual_drive, bool brake_drive) {
  if (manual_drive) {
    for (int i = 0; i < 4; ++i) {
      // 测试停车直接刹车，避免先按普通手控停止方式再发一次刹车。
      if (brake_drive && output.L_voltage == 0) {
        if (*LeftDrive[i]) (*LeftDrive[i])->stop(vex::brake);
      } else SetVoltage(*LeftDrive[i], output.L_voltage);
      if (brake_drive && output.R_voltage == 0) {
        if (*RightDrive[i]) (*RightDrive[i])->stop(vex::brake);
      } else SetVoltage(*RightDrive[i], output.R_voltage);
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

drive::feedback ReadDriveFeedback() {
  drive::feedback result;
  for (int i = 0; i < 4; ++i) {
    vex::motor* motors[2] = {*LeftDrive[i], *RightDrive[i]};
    drive::motor_feedback* values[2] = {&result.left[i], &result.right[i]};
    for (int side = 0; side < 2; ++side) {
      if (motors[side] && motors[side]->installed()) {
        values[side]->online = true;
        values[side]->rpm = motors[side]->velocity(vex::rpm);
        values[side]->millivolts = motors[side]->voltage(vex::voltageUnits::mV);
        // 八台均读取位置，供 TP 速度差分和 SDK 异常转速替代使用。
        values[side]->position_deg = motors[side]->position(vex::deg);
        values[side]->position_valid = std::isfinite(values[side]->position_deg);
        if (side == 0 && i < 2) {
          // 保留 LM 与同侧 LF 的独立位置、电流读数，便于和 SDK 转速交叉比较。
          values[side]->current_A = motors[side]->current(vex::amp);
          values[side]->diagnostic_valid = values[side]->position_valid &&
                                           std::isfinite(values[side]->current_A);
        }
      }
    }
  }
  return result;
}

bool ReadDrivePosition(double& left_deg, double& right_deg) {
  double left_sum = 0.0;
  double right_sum = 0.0;
  for (int i = 0; i < 4; ++i) {
    vex::motor* left = *LeftDrive[i];
    vex::motor* right = *RightDrive[i];
    if (!left || !right || !left->installed() || !right->installed()) return false;
    left_sum += left->position(vex::deg);
    right_sum += right->position(vex::deg);
  }
  if (!std::isfinite(left_sum) || !std::isfinite(right_sum)) return false;
  left_deg = left_sum / 4.0;
  right_deg = right_sum / 4.0;
  return true;
}

void SetDriveVelocity(double left_rpm, double right_rpm) {
  // 每个电机使用 SDK 的转速控制。零速时主动刹车。
  for (int i = 0; i < 4; ++i) {
    vex::motor* left = *LeftDrive[i];
    vex::motor* right = *RightDrive[i];
    if (left_rpm == 0) left->stop(vex::brake);
    else left->spin(vex::forward, left_rpm, vex::rpm);
    if (right_rpm == 0) right->stop(vex::brake);
    else right->spin(vex::forward, right_rpm, vex::rpm);
  }
}

void StopDriveTurn() {
  for (int i = 0; i < 4; ++i) {
    vex::motor* left = *LeftDrive[i];
    vex::motor* right = *RightDrive[i];
    if (left) left->stop(vex::brake);
    if (right) right->stop(vex::brake);
  }
}

InertialState ReadInertial(double& rotation) {
  if (!Inertial || !Inertial->installed()) {
    inertial_calibration_started = false;
    return InertialState::offline;
  }
  // 掉线或启动时未识别到传感器，需要重启程序完成静止校准。
  if (!inertial_calibration_started) return InertialState::fault;
  const double elapsed = InertialTimer.time(vex::msec);
  if (Inertial->isCalibrating())
    return elapsed < 8000 ? InertialState::calibrating : InertialState::fault;
  if (elapsed < 500) return InertialState::calibrating;
  rotation = Inertial->rotation(vex::deg);
  return std::isfinite(rotation) ? InertialState::ready : InertialState::fault;
}
}  // 硬件接口命名空间
