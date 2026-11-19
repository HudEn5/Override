#include "autonomous.h"

namespace {
// 只改这个编号选择路线，重新编译下载；一次只运行一组，不会连续跑完所有测试。
// 0 原点位；1 直行往返；2 左L；3 右L；4 多弯；5 原地转角；6 三点闭合。
// 7 单点MoveToPoint；8 左侧固定Swing；9 右侧固定Swing，一次只测试一个动作。
constexpr int TEST_ROUTE = 7;
// 仅4号路线：true为降速转向，false恢复原来的直行→原地转向。
constexpr bool ROLLING_ROUTE = true;
static_assert(TEST_ROUTE >= 0 && TEST_ROUTE <= 9, "Invalid TEST_ROUTE");
}  // 命名空间

// 距离/坐标为mm，转角为逆时针正度数；速度为电机输出轴rpm。
// 新测试均用当前高速上限540rpm，沿用原加减速；短距离通常达不到速度上限。
// 下述范围仅为理想底盘中心轨迹，摆车时还需给车身、原地旋转留空间。
void autonomous_program::RunRoutine() {
  using namespace route;
  switch (TEST_ROUTE) {
    case 0:  // 保留当前测试：起点至(600,600)，最高直行400/转向300rpm。
      RunRoute({MoveToPoint(600, 600, 400, 300, 10000)});
      return;

    case 1:  // 前进800mm再倒退800mm，检查加减速、反向与回程误差。
      RunRoute({DriveStraight(800, 540, 5000), DriveStraight(-800, 540, 5000)});
      return;

    case 2:  // 左L：600mm→原地左转90度→600mm，中心范围600×600mm。
      RunRoute({DriveStraight(600, 540, 5000), Turn(90, 540, 3000), DriveStraight(600, 540, 5000)});
      return;

    case 3:  // 右L：与左L镜像，便于对照左右转向，中心范围600×600mm。
      RunRoute({DriveStraight(600, 540, 5000), Turn(-90, 540, 3000), DriveStraight(600, 540, 5000)});
      return;

    case 4:  // 四弯路线用通用组合显式编排；直线长度不含移动转向的位移。
      // 相邻rolling自动衔接；下一项前直线填0，沿用上一项后直线。
      if (ROLLING_ROUTE)
        RunRoute({DriveTurnDrive(250, 90, 500, turn_mode::rolling, 150, 540, 5000),
                  DriveTurnDrive(0, 90, 500, turn_mode::rolling, 150, 540, 5000),
                  DriveTurnDrive(0, 90, 500, turn_mode::rolling, 150, 540, 5000),
                  DriveTurnDrive(0, 90, 250, turn_mode::rolling, 150, 540, 5000)});
      else
        RunRoute({DriveStraight(500, 540, 5000), Turn(90, 540, 3000), DriveStraight(500, 540, 5000),
                  Turn(90, 540, 3000), DriveStraight(500, 540, 5000), Turn(90, 540, 3000),
                  DriveStraight(500, 540, 5000), Turn(90, 540, 3000)});
      return;

    case 5:  // 原地正反转30/90/120/150度，每对转角后回到原朝向，不平移。
      RunRoute({Turn(30, 540, 3000), Wait(200, 1000), Turn(-30, 540, 3000), Wait(200, 1000),
                Turn(90, 540, 3000), Wait(200, 1000), Turn(-90, 540, 3000), Wait(200, 1000),
                Turn(120, 540, 3000), Wait(200, 1000), Turn(-120, 540, 3000), Turn(150, 540, 3000),
                Wait(200, 1000), Turn(-150, 540, 3000)});
      return;

    case 6:  // 三点闭合：全局500×500mm范围，每段仍是先原地转向、再直行。
      // 此组会检查位姿终点容差；原高速POINT ERROR问题仍待整定。
      // 最后朝向为返回原点的方向，闭合位置不代表恢复初始朝向。
      RunRoute({MoveToPoint(500, 0, 540, 540, 10000), MoveToPoint(500, 500, 540, 540, 10000),
                MoveToPoint(0, 0, 540, 540, 10000)});
      return;

    case 7:  // 首测单点：前方300、左方300mm；观察最终位置与约45度朝向。
      RunRoute({MoveToPoint(300, 300, 400, 300, 10000)});
      return;

    case 8:  // 左侧制动、右侧驱动，逆时针90度；不接其他动作，单独观察终态。
      RunRoute({Swing(90, drive::pivot_side::left, 300, 5000)});
      return;

    case 9:  // 右侧制动、左侧驱动，顺时针90度；与8号路线镜像对照。
      RunRoute({Swing(-90, drive::pivot_side::right, 300, 5000)});
      return;
  }
}
