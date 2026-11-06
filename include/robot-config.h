#pragma once

#include "driver-control-logic.h"
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
void Apply(const manual::Output& output);
void GetDriveMotorCount(int& left_count, int& right_count);
}  // 硬件接口命名空间
