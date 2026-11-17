#include "autonomous.h"

// 调路线只改下面的动作表，按顺序执行；默认速度与加减速由高速配置统一提供。
// 距离 mm；Turn 为相对转角（正数逆时针），TurnToHeading 为绝对朝向；Wait 单位毫秒。
void autonomous_program::RunRoutine() {
  using namespace route;
  RunRoute({
      DriveStraight(1500),  // 前进1.5m，当前实验档最高540电机rpm，到位停稳后立即进入转向。
      Turn(90),             // 原地逆时针90°，当前540rpm上限，使用角速度规划与快速补角。
      DriveStraight(1500),  // 按转弯后的朝向再前进1.5m，三段之间不额外插入Wait。
      // DriveCorner(500, 90, 500), // 需要连续圆角时，可用这一条替换上面的三个动作。
      // MoveToPoint(500, 500),   // 全局X前/Y左，先转向目标再直行；可另传直行/转向rpm。
      // Swing(90, drive::pivot_side::left), // 左侧制动，右侧前进，逆时针转90度。
      // Swing(-90, drive::pivot_side::right), // 右侧制动，左侧前进，顺时针转90度。
      // Wait(200),              // 停车等待 200 毫秒，控制与场控检查仍正常运行。
      // PushFor(1000),          // 到定位机构附近后，以默认4V前推1秒，停稳后继续。
      // PushFor(600, 3.0),      // 可选指定前推电压V；不会自动判断卡入或清零坐标。
      // DriveStraight(500, 200), // 可为某个动作单独指定较低速度。
      // TurnToHeading(0),       // 按最短方向转到惯导绝对朝向 0°。
  });
}
