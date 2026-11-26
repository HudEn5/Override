#include "autonomous.h"

void autonomous_program::RunRoutine() {
  using namespace route;
  using mechanism::channel;
  static const std::initializer_list<command> path = {
      // 放预装：夹紧，前推与抬臂同周期启动；两者完成就下降，降到放料高度再松爪。
      ClawClose(750, 12, 4.0, 4000),  // 闭合 2 秒后以 4 V 持续夹紧，直到开爪。
      WaitMechanism(channel::claw, 4000),
      LiftTo(45, 4000),
      PushFor(700, 4.0, 4000),
      WaitMechanism(channel::arm, 4000),
      LiftTo(15, 4000),
      WaitMechanism(channel::arm, 4000),
      ClawOpen(300, 12, 1000),
      WaitMechanism(channel::claw, 1000),

      // 降到零位，沿原方向倒离蓝座，转到车尾正对墙，再贴墙并直行翻 Toggle。
      LiftTo(0, 4000),
      WaitMechanism(channel::arm, 4000),
      DriveStraight(-300, 200, 3000),
      TurnToHeading(-45, 200, 3000),
      PushFor(700, -3.5, 2500),
      DriveStraight(120, 200, 2500),

      // 翻完 Toggle 后按上轮实测航向和行程绕过蓝座。
      TurnToHeading(-35, 250, 3000),
      DriveStraight(565.3, 300, 6000),
      // 从上轮绕行落点 (466.7, -352.6) 去当前目标 (750, -286)。
      TurnToHeading(-5, 250, 3000),
      DriveStraight(270, 300, 4000),
      DriveStraight(45, 20, 2500),
      ClawClose(500, 12, 4.0, 1500),
      WaitMechanism(channel::claw, 1500),

      // // 在抓取位原地转向蓝座预备位，同时抬臂。转向和抬臂都完成后再前进。
      LiftTo(50, 4000),
      TurnToHeading(115, 250, 3000),
      WaitMechanism(channel::arm, 4000),
      DriveStraight(300, 180, 4000),

      // // 到蓝座预备位后对准限位，推靠后降到放料高度再松爪。
      //TurnToHeading(135, 250, 3000),
      PushFor(380, 4.0, 2500),
      LiftTo(10, 4000),
      WaitMechanism(channel::arm, 4000),
      ClawOpen(300, 12, 1000),
      WaitMechanism(channel::claw, 1000),
      DriveStraight(-300, 180, 4000)
  };
  RunRoute(path, pace::sprint);
}
