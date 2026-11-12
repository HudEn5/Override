#pragma once

#include "driver-control-logic.h"
#include "drive-motion.h"
#include "vex.h"

extern vex::brain Brain;
extern vex::controller Controller;
extern vex::competition Competition;

// 设备初始化后赋值；名称与本赛季提供的配置保持一致。
extern vex::motor* LF;
extern vex::motor* LM;
extern vex::motor* LBD;
extern vex::motor* LBU;
extern vex::motor* RF;
extern vex::motor* RM;
extern vex::motor* RBD;
extern vex::motor* RBU;
extern vex::motor* ArmL;
extern vex::motor* ArmR;
extern vex::motor* Claw;
extern vex::digital_out* TogglePiston;

namespace hardware {
bool Initialize();
manual::Input ReadInput();
void Apply(const manual::Output& output, bool manual_drive = true);
void GetDriveMotorCount(int& left_count, int& right_count);
// 非阻塞接口，仅由主控制线程调用；自动时段 Apply(..., false) 避免覆盖 RPM 输出。
motion::MoveResult StartDriveMotion(double distance_cm);
motion::MoveResult StartDriveMotion(double distance_cm, const motion::TrapezoidOptions& options);
// 输出轴角度作为规划单位，复用相同规划/跟随算法，无需轮径标定；仅限无场控。
motion::MoveResult StartDriveTest(double motor_revolutions);
// 正数为左侧前进、右侧后退的原地右转；单位为电机输出轴圈，仅限无场控。
motion::MoveResult StartDriveTurn(double motor_revolutions);
motion::MoveResult UpdateDriveMotion();
motion::MoveResult CancelDriveMotion();
motion::MoveResult GetDriveMotionResult();
motion::DriveTelemetry GetDriveMotionTelemetry();
}  // 硬件接口命名空间
