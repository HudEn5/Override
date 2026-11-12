#pragma once

#include "program-mode.h"

struct competition_request {
  runtime::competition_stage stage = runtime::competition_stage::none;
  unsigned int generation = 0;
};

// 与 Tungsten 相同的 VEX 比赛回调名；只发布阶段，不在回调线程写电机。
void autonomous();
void usercontrol();
void RegisterCompetitionCallbacks();
competition_request ReadCompetitionRequest();
