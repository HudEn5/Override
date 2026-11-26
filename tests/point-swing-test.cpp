#include "autonomous.h"
#include <cassert>
#include <cstdio>
#include <limits>

static route::command requested = route::MoveToPoint(0, 300, 100, 100, 100000);
void autonomous_program::RunRoutine() {
  RunRoute({requested, route::Wait(0, 100000)});
}

drive::straight_config Straight() {
  drive::straight_config s;
  s.motion.wheel_diameter_mm = 69.85;
  s.motion.motor_revs_per_wheel_rev = 1.2;
  s.motion.settle_ms = 0;
  s.left_forward.kS = 400;
  s.left_forward.kV = 16;
  s.left_forward.calibrated = true;
  s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
  return s;
}
drive::straight_feedback Frame(double now = 0) {
  drive::straight_feedback f;
  f.velocity.valid = f.position.valid = f.motors_online = f.inertial_ready = f.stopped = true;
  f.heading_deg = 350;
  f.velocity.time_ms = now;
  f.velocity.sequence = static_cast<unsigned int>(now / 20 + 1);
  return f;
}
drive::pose Pose(double now = 0) {
  drive::pose p;
  p.valid = true;
  p.time_ms = now;
  return p;
}
double CheckPointCommand(const route::command& command, double shift_mm, unsigned realignments,
                         bool expect_done, double straight_offset_mm = 0) {
  requested = command;
  const double goal_x = command.x_mm, goal_y = command.y_mm;
  const auto s = Straight();
  auto t = config::AutoTurnControl(drive::turn_config());
  t.left_forward = t.left_reverse = t.right_forward = t.right_reverse = s.left_forward;
  autonomous_program p;
  p.Configure(s, t);
  auto point_settings = config::PointControl();
  point_settings.max_heading_realignments = realignments;
  p.ConfigurePoint(point_settings);
  bool shifted = false, driving_started = false;
  auto f = Frame();
  auto pose = Pose();
  p.ObservePose(pose);
  p.Update(true, f, 0);
  const double direction = command.reverse_point ? -1 : 1;
  const double expected_heading = 350 + std::atan2(direction * goal_y, direction * goal_x) *
                                            180 / 3.14159265358979323846;
  assert(std::fabs(std::remainder(p.Info().turn.target_deg - expected_heading, 360.0)) < 1e-6);
  double l = 0, r = 0, x = 0, y = 0;
  const double mm_deg = 3.14159265358979323846 * 69.85 / 360 / 1.2;
  for (int tick = 1; tick < 1200 && p.Info().active; ++tick) {
    const auto motor = [](int mv) { return std::copysign(std::max(0., std::abs(mv) - 400.) / 16, mv); };
    l += (motor(p.Info().left_mV) - l) * .25;
    r += (motor(p.Info().right_mV) - r) * .25;
    const double dh = (r - l) * 6 * 69.85 / 1.2 / 325 / 2 * .02;
    const double mid = (f.heading_deg - 350 + dh / 2) * 3.14159265358979323846 / 180;
    const double ds = (l + r) / 2 * 6 * .02 * mm_deg;
    x += ds * std::cos(mid);
    y += ds * std::sin(mid);
    f.heading_deg += dh;
    f.position.left_deg += l * 6 * .02;
    f.position.right_deg += r * 6 * .02;
    f.velocity.left_rpm = l;
    f.velocity.right_rpm = r;
    f.velocity.time_ms = tick * 20;
    ++f.velocity.sequence;
    f.stopped = std::fabs(l) < 5 && std::fabs(r) < 5;
    // 单次转向中心漂移的外部位姿扰动，隔离复现“旧方位/新距离”几何问题。
    if (!shifted && shift_mm != 0 && p.Info().motion == autonomous_motion::turn &&
        std::fabs(p.Info().turn.error_deg) < 10) {
      x += shift_mm;
      shifted = true;
    }
    if (!driving_started && p.Info().motion == autonomous_motion::straight)
      y += straight_offset_mm;
    if (p.Info().motion == autonomous_motion::straight)
      driving_started = true;
    if (driving_started)
      assert(p.Info().motion != autonomous_motion::turn);  // 不在终点再次转向/绕回。
    pose.x_mm = x;
    pose.y_mm = y;
    pose.heading_deg = f.heading_deg - 350;
    pose.time_ms = tick * 20;
    p.ObservePose(pose);
    p.Update(true, f, tick * 20);
  }
  const double error = std::hypot(x - goal_x, y - goal_y);
  if (expect_done) {
    if (p.Info().result != route_result::done)
      std::printf("Point failed %s at %.2f %.2f shift%.1f cap%.0f\n", p.Info().status, x, y, shift_mm,
                  command.maximum_rpm);
    const double exit_tolerance = command.point_exit_tolerance_mm > 0
                                      ? command.point_exit_tolerance_mm : point_settings.position_tolerance_mm;
    assert(p.Info().result == route_result::done && p.Info().step == 2 && error <= exit_tolerance);
  } else {
    assert(p.Info().finished);
    if (straight_offset_mm != 0)
      assert(p.Info().result == route_result::error && p.Info().step == 0 &&
             !std::strcmp(p.Info().status, "POINT ERROR"));
  }
  assert(p.Info().left_mV == 0 && p.Info().right_mV == 0);
  return error;
}

double CheckPoint(double goal_x, double goal_y, double shift_mm = 0, unsigned realignments = 2,
                  bool expect_done = true, double maximum_rpm = 100, bool reverse = false) {
  const auto command = reverse ? route::ReverseToPoint(goal_x, goal_y, maximum_rpm, 100, 100000)
                               : route::MoveToPoint(goal_x, goal_y, maximum_rpm, 100, 100000);
  return CheckPointCommand(command, shift_mm, realignments, expect_done);
}

drive::feedforward_config PointTurnModel(double ks, double kv) {
  drive::feedforward_config f;
  f.kS = ks;
  f.kV = kv;
  f.calibrated = true;
  return f;
}
struct point_turn_result {
  bool done = false;
  double elapsed = 0, error = 0, tail = 0;
};
point_turn_result CheckPointTurn(double scale, double distance_mm, double angle, double friction,
                                 double response, double right_response, double cap) {
  const double PI = 3.14159265358979323846;
  requested = route::MoveToPoint(distance_mm * std::cos(angle * PI / 180),
                                 distance_mm * std::sin(angle * PI / 180), 540, cap, 100000);
  drive::straight_config base;
  base.motion.wheel_diameter_mm = 69.85;
  base.motion.motor_revs_per_wheel_rev = 1.2;
  base.left_forward = PointTurnModel(372.573735, 17.900544);
  base.right_forward = PointTurnModel(408.162564, 16.537990);
  base.left_reverse = PointTurnModel(547.829766, 16.939618);
  base.right_reverse = PointTurnModel(537.033135, 15.870378);
  auto s = config::AutoStraightControl(base);
  drive::turn_config raw;
  raw.pid.kP = raw.pid.kI = 20;
  raw.pid.max_correction_mV = 2000;
  raw.pid.max_integral_mV = 1200;
  raw.correction_fraction = 3;
  raw.left_forward = base.left_forward;
  raw.right_forward = base.right_forward;
  raw.left_reverse = base.left_reverse;
  raw.right_reverse = base.right_reverse;
  const auto t = config::AutoTurnControl(raw);
  autonomous_program p;
  p.Configure(s, t);
  auto settings = config::PointControl();
  settings.turn_finish_speed_scale = scale;
  p.ConfigurePoint(settings);
  drive::straight_feedback f;
  f.position.valid = f.velocity.valid = f.motors_online = f.inertial_ready = f.stopped = true;
  drive::pose pose;
  pose.valid = true;
  p.ObservePose(pose);
  p.Update(true, f, 0);
  double l = 0, r = 0, first = -1;
  point_turn_result out;
  const auto motor = [&](int mv, bool left) {
    const auto m = left ? (mv < 0 ? base.left_reverse : base.left_forward)
                        : (mv < 0 ? base.right_reverse : base.right_forward);
    return std::copysign(std::max(0., std::abs(mv) - m.kS - friction) / m.kV, mv);
  };
  for (int tick = 1; tick < 650; ++tick) {
    const auto before = p.Info();
    l += (motor(before.left_mV, true) - l) * response;
    r += (motor(before.right_mV, false) - r) * right_response;
    f.heading_deg += (r - l) * 6 * 69.85 / 1.2 / 325 / 2 * .02;
    f.velocity.left_rpm = l;
    f.velocity.right_rpm = r;
    f.velocity.time_ms = tick * 20;
    ++f.velocity.sequence;
    f.stopped = std::fabs(l) < 5 && std::fabs(r) < 5;
    pose.heading_deg = f.heading_deg;
    pose.time_ms = tick * 20;
    p.ObservePose(pose);
    p.Update(true, f, tick * 20);
    assert(std::abs(p.Info().left_mV) <= 12000 && std::abs(p.Info().right_mV) <= 12000);
    out.elapsed = tick * 20;
    out.error = angle - f.heading_deg;
    if (std::fabs(out.error) <= .8 && first < 0)
      first = out.elapsed;
    if (p.Info().motion == autonomous_motion::straight && p.Info().active) {
      out.done = true;
      break;
    }
    if (!p.Info().active)
      break;
  }
  out.tail = first >= 0 ? out.elapsed - first : out.elapsed;
  return out;
}

// 保留严格角度窗口，验证轻负载/静摩擦、短点位/远点位和左右响应差异下的补角。
void CheckPointTurnSpeed() {
  unsigned count = 0;
  double before_time = 0, after_time = 0;
  for (double distance : {500., 707., 1500., 2121.})
    for (double angle : {-150., -135., -90., -30., 30., 90., 135., 150.})
      for (double friction : {0., 600., 900.})
        for (int lag = 0; lag < 3; ++lag)
          for (double cap : {100., 540.}) {
            const auto before =
                CheckPointTurn(1, distance, angle, friction, lag == 1 ? .12 : .25, lag == 2 ? .12 : .25, cap);
            const auto after = CheckPointTurn(config::PointControl().turn_finish_speed_scale, distance, angle,
                                              friction, lag == 1 ? .12 : .25, lag == 2 ? .12 : .25, cap);
            const double tolerance = std::atan2(2.5, distance) * 180 / 3.14159265358979323846;
            assert(before.done && after.done && std::fabs(after.error) <= tolerance);
            before_time += before.elapsed;
            after_time += after.elapsed;
            ++count;
          }
  if (config::PointControl().turn_finish_speed_scale > 1)
    assert(after_time < before_time);
  std::printf("Point turn speed: %u paired models, mean %.0f -> %.0fms, unchanged angle/stop conditions.\n",
              count, before_time / count, after_time / count);
}

void CheckPointRealignment() {
  unsigned old_failures = 0;
  double max_before = 0, max_after = 0;
  for (double x : {-300., 300.})
    for (double y : {-300., 300.})
      for (double shift : {-8., 8.})
        for (double cap : {100., 400.}) {
          const double before = CheckPoint(x, y, shift, 0, false, cap);
          const double after = CheckPoint(x, y, shift, 2, true, cap);
          old_failures += before > 5;
          max_before = std::max(max_before, before);
          max_after = std::max(max_after, after);
        }
  assert(old_failures > 0 && max_after <= 5);
  std::printf("Point realignment: 16 paired center-shift models old failures%u max%.2fmm, new max%.2fmm.\n",
              old_failures, max_before, max_after);
}

double CheckAlignmentGuard(double deadline = 100000) {
  requested = route::MoveToPoint(300, 300, 400, 300, deadline);
  const auto s = Straight();
  auto t = config::AutoTurnControl(drive::turn_config());
  t.left_forward = t.left_reverse = t.right_forward = t.right_reverse = s.left_forward;
  autonomous_program p;
  p.Configure(s, t);
  auto f = Frame();
  auto pose = Pose();
  p.ObservePose(pose);
  p.Update(true, f, 0);
  unsigned starts = 0;
  double target = -10000, first_realign = -1;
  for (int tick = 1; tick < 500 && p.Info().active; ++tick) {
    assert(p.Info().motion == autonomous_motion::turn);
    if (p.Info().turn.target_deg != target) {
      target = p.Info().turn.target_deg;
      ++starts;
      pose.x_mm += 8;  // 每次转向后目标方位都继续变化，验证不能无休止重试。
      if (starts == 2)
        first_realign = p.Info().time_ms;
    }
    f = Frame(tick * 20);
    f.heading_deg = target;
    pose.heading_deg = target - 350;
    pose.time_ms = tick * 20;
    p.ObservePose(pose);
    p.Update(true, f, tick * 20);
  }
  assert(p.Info().finished && p.Info().result == route_result::error && p.Info().step == 0);
  assert(p.Info().left_mV == 0 && p.Info().right_mV == 0);
  if (deadline == 100000) {
    assert(starts == 3 && !std::strcmp(p.Info().status, "POINT ALIGN"));
  } else {
    assert(first_realign >= 0 && !std::strcmp(p.Info().status, "TIMEOUT"));
    assert(p.Info().time_ms == deadline);
  }
  return first_realign;
}

int main() {
  CheckPointRealignment();
  const double first_realign = CheckAlignmentGuard();
  CheckAlignmentGuard(first_realign + 40);
  CheckPointTurnSpeed();
  {
    autonomous_program invalid;
    invalid.Configure(Straight(), config::AutoTurnControl(drive::turn_config()));
    auto c = config::PointControl();
    c.max_heading_realignments = 3;
    invalid.ConfigurePoint(c);
    invalid.ObservePose(Pose());
    invalid.Update(true, Frame(), 0);
    assert(invalid.Info().result == route_result::error &&
           !std::strcmp(invalid.Info().status, "POINT CONFIG"));
  }
  for (double invalid :
       {0., -1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    autonomous_program bad;
    bad.Configure(Straight(), config::AutoTurnControl(drive::turn_config()));
    auto settings = config::PointControl();
    settings.turn_finish_speed_scale = invalid;
    bad.ConfigurePoint(settings);
    bad.ObservePose(Pose());
    bad.Update(true, Frame(), 0);
    assert(bad.Info().result == route_result::error && !std::strcmp(bad.Info().status, "POINT CONFIG") &&
           bad.Info().left_mV == 0 && bad.Info().right_mV == 0);
  }
  const auto s = Straight();
  drive::turn_config t = config::AutoTurnControl(drive::turn_config());
  t.left_forward = t.left_reverse = t.right_forward = t.right_reverse = s.left_forward;
  // 路线Swing使用独立配置，普通Turn仍使用原配置，防止调参泄漏。
  for (bool swing_action : {true, false}) {
    requested =
        swing_action ? route::Swing(90, drive::pivot_side::left, 300, 5000) : route::Turn(90, 300, 5000);
    autonomous_program p;
    p.Configure(s, t);
    drive::turn_controller direct;
    direct.Configure(swing_action ? config::SwingControl(t) : t);
    auto f = Frame();
    direct.Observe(f, true, 0);
    assert(swing_action ? direct.StartSwingRelative(90, 300, drive::pivot_side::left)
                        : direct.StartTurnRelative(90, 300));
    p.Update(true, f, 0);
    f = Frame(20);
    direct.Observe(f, true, 20);
    direct.UpdateTurn();
    p.Update(true, f, 20);
    assert(p.Info().left_mV == direct.Info().left_mV && p.Info().right_mV == direct.Info().right_mV);
    p.CancelRoutine();
  }
  for (auto fixed : {drive::pivot_side::left, drive::pivot_side::right})
    for (double angle : {-150., -120., -90., -30., 30., 90., 120., 150.})
      for (double cap : {100., 540.}) {
        drive::turn_controller c;
        c.Configure(t);
        auto f = Frame();
        c.Observe(f, true, 0);
        assert(c.StartSwingRelative(angle, cap, fixed));
        assert(c.Info().pivot == fixed);
        const double factor = 6 * 69.85 / 1.2 / 325 / 2;
        assert(c.Info().profile_peak_deg_s <= cap * factor + 1e-6);
        double l = 0, r = 0;
        for (int tick = 1; tick < 600; ++tick) {
          const auto v = c.Info();
          assert((fixed == drive::pivot_side::left ? v.left_mV : v.right_mV) == 0);
          const auto motor = [](int mv) { return std::copysign(std::max(0., std::abs(mv) - 400.) / 16, mv); };
          l += (motor(v.left_mV) - l) * .25;
          r += (motor(v.right_mV) - r) * .25;
          f.heading_deg += (r - l) * factor * .02;
          f.velocity.left_rpm = l;
          f.velocity.right_rpm = r;
          f.velocity.time_ms = tick * 20;
          ++f.velocity.sequence;
          f.stopped = std::fabs(l) < 5 && std::fabs(r) < 5;
          c.Observe(f, true, tick * 20);
          c.UpdateTurn();
          if (c.Info().state != drive::turn_state::running)
            break;
        }
        assert(c.Info().state == drive::turn_state::done && std::fabs(c.Info().error_deg) <= .8);
        c.Observe(Frame(13000), true, 13000);
        assert(c.StartTurnRelative(90, 100));
        assert(c.Info().pivot == drive::pivot_side::none);
        c.CancelTurn();
        assert(c.Info().left_mV == 0 && c.Info().right_mV == 0);
      }
  CheckPoint(0, 300);
  CheckPoint(0, 1500);
  CheckPoint(-1500, 0);
  CheckPoint(1500, 1500);
  CheckPoint(650, -600, 0, 2, true, 540, true);
  // 同一段横向偏差默认应拒绝，只有显式放宽的绕行点允许继续。
  const auto strict_point = route::MoveToPoint(560, 0, 300, 250, 6000);
  const auto waypoint = route::MoveToWaypoint(560, 0, 300, 250, 10, 6000);
  assert(CheckPointCommand(strict_point, 0, 2, false, 6.3) > 5);
  assert(CheckPointCommand(waypoint, 0, 2, true, 6.3) <= 10);
  assert(CheckPointCommand(waypoint, 0, 2, false, 11) > 10);
  for (int fault = 0; fault < 4; ++fault) {
    autonomous_program bad;
    bad.Configure(s, t);
    auto pos = Pose();
    auto frame = Frame();
    bad.ObservePose(pos);
    bad.Update(true, frame, 0);
    if (fault == 0)
      pos.valid = false;
    if (fault == 1)
      pos.needs_reset = true;
    if (fault == 2)
      pos.x_mm = std::numeric_limits<double>::quiet_NaN();
    pos.time_ms = fault == 3 ? 0 : 220;
    frame.velocity.time_ms = 220;
    ++frame.velocity.sequence;
    bad.ObservePose(pos);
    bad.Update(true, frame, 220);
    assert(bad.Info().result == route_result::error && bad.Info().left_mV == 0 && bad.Info().right_mV == 0);
  }
  for (double x : {-300., 300.})
    for (double y : {-300., 300.}) {
      requested = route::MoveToPoint(x, y, 100, 100, 100000);
      autonomous_program direction;
      direction.Configure(s, t);
      auto pos = Pose();
      pos.heading_deg = 720;
      direction.ObservePose(pos);
      direction.Update(true, Frame(), 0);
      double expected = std::atan2(y, x) * 180 / 3.14159265358979323846;
      assert(std::fabs(direction.Info().turn.target_deg - (350 + expected)) < 1e-6);
      direction.CancelRoutine();
      direction.Update(false, Frame(20), 20);
      assert(direction.Info().result == route_result::cancelled && direction.Info().left_mV == 0 &&
             direction.Info().right_mV == 0);
    }
  for (int fault = 0; fault < 3; ++fault) {
    requested = route::MoveToPoint(fault == 0 ? std::numeric_limits<double>::infinity() : 0, 300,
                                   fault == 1 ? 601 : 100, route::DEFAULT_TURN_RPM, 100000);
    autonomous_program invalid;
    invalid.Configure(s, t);
    auto pos = Pose();
    if (fault == 2)
      pos.valid = false;
    invalid.ObservePose(pos);
    invalid.Update(true, Frame(), 0);
    assert(invalid.Info().result == route_result::error && invalid.Info().left_mV == 0 &&
           invalid.Info().right_mV == 0);
  }
  requested = route::MoveToPoint(0, 0, route::DEFAULT_DRIVE_RPM, route::DEFAULT_TURN_RPM, 100000);
  autonomous_program zero;
  zero.Configure(s, t);
  zero.ObservePose(Pose());
  zero.Update(true, Frame(), 0);
  zero.ObservePose(Pose(20));
  zero.Update(true, Frame(20), 20);
  assert(zero.Info().result == route_result::done);
  requested = route::Swing(90, drive::pivot_side::left, route::DEFAULT_TURN_RPM, 100000);
  autonomous_program swing;
  swing.Configure(s, t);
  swing.Update(true, Frame(), 0);
  swing.Update(true, Frame(20), 20);
  assert(swing.Info().turn.pivot == drive::pivot_side::left && swing.Info().left_mV == 0);
  swing.Update(false, Frame(40), 40);
  assert(swing.Info().result == route_result::cancelled && swing.Info().left_mV == 0 &&
         swing.Info().right_mV == 0);
  std::puts(
      "Point/Swing passed: global frame, coupled sequence, endpoint, pose loss/stale, zero goal, signed "
      "pivots and disable.");
}
