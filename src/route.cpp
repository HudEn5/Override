#include "autonomous.h"

// 调路线只改下面的动作表，按顺序执行；默认速度与加减速由高速配置统一提供。
// 距离 mm；Turn 为相对转角（正数逆时针），TurnToHeading 为绝对朝向；Wait 单位毫秒。
void autonomous_program::RunRoutine() {
  using namespace route;
  RunRoute({
      DriveCorner(500, 90, 500),  // 已完成一次实车的连续路线，中途不停稳。
      // DriveStraight(500),     // 独立直行，默认最高 330 rpm；负数后退。
      // Wait(200),              // 停车等待 200 毫秒，控制与场控检查仍正常运行。
      // PushFor(1000),          // 到定位机构附近后，以默认4V前推1秒，停稳后继续。
      // PushFor(600, 3.0),      // 可选指定前推电压V；不会自动判断卡入或清零坐标。
      // Turn(90),               // 原地相对左转 90°，默认单侧上限 140 rpm。
      // DriveStraight(500, 200), // 可为某个动作单独指定较低速度。
      // TurnToHeading(0),       // 按最短方向转到惯导绝对朝向 0°。
  });
}
