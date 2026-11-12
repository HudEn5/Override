#include "autonomous.h"

// 对应 Tungsten 的 route.cpp：这里只编排比赛动作，不处理手柄、采样或直接写电机。
void autonomous_program::RunRoutine() {
  // 正式路线尚未指定，默认保持停车。
  // 用法示例见 README：StartDriveStraight(距离 mm, 最高 rpm)，等 DONE 再进下一步。
  FinishRoutine("EMPTY");
}
