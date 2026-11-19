#include "autonomous.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

static double entry_mm_s = 150;
static bool followup = false;
static int route_mode = 0;
static double turn_deg = 90;
static bool corrupt_owner = false;
static int invalid_leg = 0;
void autonomous_program::RunRoutine() {
  using namespace route;
  if (route_mode == 11 || route_mode == 12)
    RunRoute({DriveTurnDrive(100, 30, 100, turn_mode::rolling, 100, 100, route_mode == 12 ? 100 : 100000),
              DriveTurnDrive(0, -30, 100, turn_mode::rolling, 100, 100, 100)});
  else if (route_mode == 13)
    RunRoute({DriveTurnDrive(0, 30, 2000, turn_mode::rolling, 100, 100, 2500)});
  else if (route_mode == 8 || route_mode == 9) {
    const auto c = DriveTurnDrive(0, 30, 100, turn_mode::rolling, 100, 100, 100000);
    if (route_mode == 8)
      RunRoute({c, c, c, c, c, c, c, c, c, c, c, c, c, c, c, c, c});
    else
      RunRoute({c, c, c, c, c, c, c, c, c, c, c, c, c, c, c, c});
  } else if (route_mode == 10)
    RunRoute({DriveTurnDrive(100, 90, 200, turn_mode::rolling, 150, 540, 100000),
              DriveTurnDrive(100, -90, 300, turn_mode::rolling, 150, 540, 100000)});
  else if (route_mode == 3)
    RunRoute({DriveTurnDrive(120, 30, 120, turn_mode::rolling, 150, 100, 100000),
              DriveCorner(100, -30, 100, 100, 100000), Turn(90, 100, 100000),
              DriveTurnDrive(120, -90, 120, turn_mode::rolling, 150, 100, 100000),
              DriveStraight(100, 100, 100000)});
  else if (route_mode == 4)
    RunRoute({DriveTurnDrive(300, 30, 200, turn_mode::rolling, 150, 540, 100000),
              DriveTurnDrive(0, -120, 200, turn_mode::rolling, 150, 540, 100000),
              DriveTurnDrive(0, 150, 300, turn_mode::rolling, 150, 540, 100000)});
  else if (route_mode == 5)
    RunRoute({DriveTurnDrive(300, 90, 20, turn_mode::rolling, 200, 200, 100000),
              DriveTurnDrive(0, -30, 20, turn_mode::rolling, 100, 100, 100000),
              DriveTurnDrive(0, -60, 300, turn_mode::rolling, 180, 540, 100000)});
  else if (route_mode == 6)
    RunRoute({DriveTurnDrive(150, 90, 150, turn_mode::rolling, 150, 540, 100000), Wait(80, 100000),
              DriveTurnDrive(0, -90, 150, turn_mode::rolling, 150, 540, 100000),
              DriveTurnDrive(0, 30, 150, turn_mode::rolling, 150, 540, 100000)});
  else if (route_mode == 7)
    RunRoute({DriveTurnDrive(0, 30, 0, turn_mode::rolling, 150, 540, 100000),
              DriveTurnDrive(0, -30, 200, turn_mode::rolling, 150, 540, 100000)});
  else if (route_mode != 0)
    RunRoute({DriveTurnDrive(500, turn_deg, 500, route_mode == 1 ? turn_mode::stop : turn_mode::rolling, 150,
                             540, 100000)});
  else if (followup)
    RunRoute({DriveTurnDrive(250, 90, 500, turn_mode::rolling, entry_mm_s, 540, 100000),
              DriveTurnDrive(0, 90, 500, turn_mode::rolling, entry_mm_s, 540, 100000),
              DriveTurnDrive(0, 90, 500, turn_mode::rolling, entry_mm_s, 540, 100000),
              DriveTurnDrive(0, 90, 250, turn_mode::rolling, entry_mm_s, 540, 100000),
              DriveStraight(100, 100, 100000)});
  else
    RunRoute({DriveTurnDrive(250, 90, 500, turn_mode::rolling, entry_mm_s, 540, 100000),
              DriveTurnDrive(0, invalid_leg == 12 ? std::numeric_limits<double>::quiet_NaN() : 90,
                             invalid_leg == 13 ? -1 : 500, turn_mode::rolling,
                             invalid_leg == 14 ? 2000 : entry_mm_s, 540,
                             invalid_leg == 17   ? 0
                             : invalid_leg == 18 ? std::numeric_limits<double>::quiet_NaN()
                                                 : 100000),
              DriveTurnDrive(0, 90, 500, turn_mode::rolling, entry_mm_s, 540, 100000),
              DriveTurnDrive(0, 90,
                             invalid_leg == 15   ? 0
                             : invalid_leg == 16 ? 5
                                                 : 250,
                             turn_mode::rolling, entry_mm_s, 540, 100000)});
  // 故意制造动作标记错误，验证运行时会拒绝错误的控制器归属。
  if (corrupt_owner && RollingState() == drive::rolling_state::running)
    info.motion = autonomous_motion::corner;
}
void Run(double response, double right_response, int fault = 0) {
  followup = fault == 8;
  corrupt_owner = fault == 11;
  invalid_leg = fault;
  drive::straight_config base;
  base.motion.wheel_diameter_mm = 69.85;
  base.motion.motor_revs_per_wheel_rev = 1.2;
  base.left_forward.kS = 400;
  base.left_forward.kV = 16;
  base.left_forward.calibrated = true;
  base.left_reverse = base.right_forward = base.right_reverse = base.left_forward;
  auto s = config::AutoStraightControl(base);
  auto t = config::AutoTurnControl(drive::turn_config());
  t.left_forward = t.left_reverse = t.right_forward = t.right_reverse = base.left_forward;
  t.pid.kP = t.pid.kI = 20;
  t.pid.max_correction_mV = 2000;
  t.pid.max_integral_mV = 1200;
  t.correction_fraction = 3;
  drive::corner_config c;
  c.wheel_diameter_mm = 69.85;
  c.motor_revs_per_wheel_rev = 1.2;
  c.track_width_mm = 325;
  c.left_forward = c.left_reverse = c.right_forward = c.right_reverse = base.left_forward;
  c.pid.kP = c.pid.kI = 20;
  c.pid.max_correction_mV = 2000;
  c.pid.max_integral_mV = 1200;
  c.angle_tolerance_deg = 1.5;
  c.rate_tolerance_deg_s = 4;
  c.settle_ms = 0;
  c.timeout_ms = 30000;
  autonomous_program p;
  p.Configure(s, t, config::RouteCornerControl(c));
  drive::straight_feedback f;
  f.position.valid = f.velocity.valid = f.motors_online = f.inertial_ready = f.stopped = true;
  p.Update(true, f, 0);
  double l = 0, r = 0, x = 0, y = 0, lo_x = 0, hi_x = 0, lo_y = 0, hi_y = 0;
  int previous = 0, turns = 0, exits = 0;
  bool joined_distance_checked = false;
  for (int tick = 1; tick < 5000 && p.Info().active; ++tick) {
    const auto v = p.Info();
    const auto motor = [](int mv) { return std::copysign(std::max(0., std::abs(mv) - 400.) / 16, mv); };
    l += (motor(v.left_mV) - l) * response;
    r += (motor(v.right_mV) - r) * right_response;
    const double dh = (r - l) * 6 * 69.85 / 1.2 / 325 / 2 * .02;
    const double ds = (l + r) / 2 * 3.141592653589793 * 69.85 / (60 * 1.2) * .02;
    x += ds * std::cos((f.heading_deg + dh / 2) * 3.141592653589793 / 180);
    y += ds * std::sin((f.heading_deg + dh / 2) * 3.141592653589793 / 180);
    lo_x = std::min(lo_x, x);
    hi_x = std::max(hi_x, x);
    lo_y = std::min(lo_y, y);
    hi_y = std::max(hi_y, y);
    f.heading_deg += dh;
    f.position.left_deg += l * .12;
    f.position.right_deg += r * .12;
    f.velocity.left_rpm = l;
    f.velocity.right_rpm = r;
    f.velocity.time_ms = tick * 20;
    ++f.velocity.sequence;
    f.stopped = std::fabs(l) < 5 && std::fabs(r) < 5;
    if (fault == 1 && tick == 70)
      f.inertial_ready = false;
    if (fault == 3 && tick == 70)
      f.motors_online = false;
    if (fault == 4 && tick == 70)
      f.position.valid = false;
    if (fault == 5 && tick == 70)
      f.velocity.valid = false;
    if ((fault == 9 || fault == 10) && tick == 70) {
      assert(p.Info().motion == autonomous_motion::rolling);
      if (fault == 9) {
        p.CancelRolling();
        assert(p.Info().left_mV == 0 && p.Info().right_mV == 0);
      } else {
        // 忙时所有其他动作都不得抢占；取消旧圆角不应影响当前移动转向。
        assert(!p.StartDriveStraight(100, 100));
        assert(!p.StartTurnRelative(90, 100));
        assert(!p.StartDriveCorner(100, 90, 100, 100));
        assert(!p.StartRollingPath(nullptr, 0));
        assert(!p.StartRollingDriveTurnDrive(100, 90, 100, 100, 150, 1000));
        assert(!p.StartPushFor(100));
        p.CancelCorner();
        assert(p.Info().left_mV == v.left_mV && p.Info().right_mV == v.right_mV);
        assert(p.RollingState() == drive::rolling_state::running);
      }
    }
    if (fault == 6 && tick == 70) {
      p.CancelRoutine();
      break;
    }
    p.Update(!(fault == 2 && tick >= 70), f, tick * 20);
    const auto& info = p.Info().rolling;
    assert(p.Info().corner.state != drive::corner_state::running ||
           p.Info().motion == autonomous_motion::corner);
    assert(info.state != drive::rolling_state::running || p.Info().motion == autonomous_motion::rolling);
    if (p.Info().motion != autonomous_motion::rolling) {
      previous = 0;
      continue;
    }
    if (route_mode == 10 && !joined_distance_checked && info.completed_turns == 1 && info.phase == 3) {
      assert(info.segment_remaining_mm > 285 && info.segment_remaining_mm <= 300);
      joined_distance_checked = true;
    }
    assert(std::abs(p.Info().left_mV) <= 12000 && std::abs(p.Info().right_mV) <= 12000);
    assert(std::fabs(info.left_target_rpm) <= 540 + 1e-6 && std::fabs(info.right_target_rpm) <= 540 + 1e-6);
    assert(info.completed_turns <= info.total_turns);
    if ((route_mode == 0 || route_mode == 4 || route_mode == 5 || route_mode >= 7) && p.Info().active)
      assert(p.Info().step == 0);  // 整批完成才推进命令表，中间不另启控制器。
    if (info.completed_turns < info.total_turns && info.distance_mm > 50 &&
        info.state == drive::rolling_state::running)
      assert((f.velocity.left_rpm + f.velocity.right_rpm) / 2 > 1);
    if (previous != 2 && info.phase == 2) {
      ++turns;
      assert(info.center_rpm > 0);
    }
    if (previous == 2 && info.phase == 3) {
      ++exits;
      assert(info.center_rpm > 0);
    }
    if (info.phase == 2 && info.state == drive::rolling_state::running)
      assert(info.center_rpm > 0 && (info.distance_mm <= 5 || !f.stopped));
    previous = info.phase;
  }
  std::printf(
      "Rolling route%d lag%.2f/%.2f fault%d: %s turns%d exits%d t%.0f pose(%.1f,%.1f,%.2f) size%.1f/%.1f\n",
      route_mode, response, right_response, fault, p.Info().status, turns, exits, p.Info().time_ms, x, y,
      f.heading_deg, hi_x - lo_x, hi_y - lo_y);
  if ((!fault || fault == 8 || fault == 10) && route_mode != 8 && route_mode < 11) {
    assert(p.Info().result == route_result::done);
    const unsigned expected = route_mode == 9                      ? 16
                              : route_mode == 10                   ? 2
                              : route_mode == 0                    ? 4
                              : route_mode == 1                    ? 0
                              : route_mode == 2                    ? 1
                              : route_mode == 3 || route_mode == 7 ? 2
                                                                   : 3;
    if (route_mode != 1 && route_mode != 3 && route_mode != 6)
      assert(p.Info().rolling.completed_turns == expected && p.Info().rolling.total_turns == expected);
    if (route_mode != 7)
      assert(turns == static_cast<int>(expected) && exits == turns);
    const double heading = route_mode == 9                      ? 480
                           : route_mode == 10                   ? 0
                           : route_mode == 0                    ? 360
                           : route_mode == 4                    ? 60
                           : route_mode == 6                    ? 30
                           : route_mode == 1 || route_mode == 2 ? turn_deg
                                                                : 0;
    // 独立动作按起步实际朝向取相对角；只有同一连续块共享累加目标。
    const double final_reference = route_mode == 3   ? p.Info().drive.heading.target_deg
                                   : route_mode == 6 ? p.Info().rolling.reference_deg
                                                     : heading;
    assert(std::fabs(f.heading_deg - final_reference) <= 1.5);
    assert(f.stopped);
    if (route_mode == 0 || route_mode == 4 || route_mode == 5 || route_mode >= 7)
      assert(p.Info().step == expected + (followup ? 1 : 0));
    if (route_mode == 10)
      assert(joined_distance_checked);
  } else {
    assert(p.Info().finished && p.Info().result != route_result::done);
  }
  assert(p.Info().left_mV == 0 && p.Info().right_mV == 0);
  if (route_mode >= 11) {
    assert(std::strcmp(p.Info().status, "TIMEOUT") == 0);
    if (route_mode == 12)
      assert(p.Info().rolling.elapsed_ms == 100 && p.Info().rolling.completed_turns == 0);
    else
      assert(p.Info().rolling.elapsed_ms > 100 && p.Info().rolling.completed_turns == 1);
    if (route_mode == 13)
      assert(p.Info().rolling.elapsed_ms == 2500);
  }
  if (route_mode == 8) {
    assert(p.Info().time_ms == 0 && std::strcmp(p.Info().status, "ROLL TOO LONG") == 0);
  }
  if (fault >= 12) {
    assert(p.Info().time_ms == 0 && p.Info().rolling.distance_mm == 0);
    assert(std::strcmp(p.Info().status, (fault == 15 || fault == 16) ? "ROLL END DIST" : "ROLL CONFIG") == 0);
  }
  if (fault == 11)
    assert(std::strcmp(p.Info().status, "CONTROL OWNER") == 0);
}
int main() {
  for (double speed : {150., 200.}) {
    entry_mm_s = speed;
    for (double a : {.12, .25})
      for (double b : {.12, .25})
        Run(a, b);
  }
  Run(.25, .25, 1);
  Run(.25, .25, 2);
  for (int fault : {3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18})
    Run(.25, .25, fault);
  for (int mode : {1, 2}) {
    route_mode = mode;
    for (double angle : {-150., -120., -90., -30., 30., 90., 120., 150.}) {
      turn_deg = angle;
      Run(.25, .25);
    }
  }
  for (int mode : {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}) {
    route_mode = mode;
    for (double a : {.12, .25})
      for (double b : {.12, .25})
        Run(a, b);
  }
}
