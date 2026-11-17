#include "autonomous.h"
#include <cassert>
#include <cstdio>
#include <limits>

static route::command requested = route::MoveToPoint(0, 300, 100, 100);
void autonomous_program::RunRoutine() {
  RunRoute({requested, route::Wait(0)});
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
void CheckPoint(double goal_x, double goal_y) {
  requested = route::MoveToPoint(goal_x, goal_y, 100, 100);
  const auto s = Straight();
  auto t = config::AutoTurnControl(drive::turn_config());
  t.left_forward = t.left_reverse = t.right_forward = t.right_reverse = s.left_forward;
  autonomous_program p;
  p.Configure(s, t);
  auto f = Frame();
  auto pose = Pose();
  p.ObservePose(pose);
  p.Update(true, f, 0);
  assert(std::fabs(p.Info().turn.target_deg -
                   (350 + std::atan2(goal_y, goal_x) * 180 / 3.14159265358979323846)) < 1e-6);
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
    pose.x_mm = x;
    pose.y_mm = y;
    pose.heading_deg = f.heading_deg - 350;
    pose.time_ms = tick * 20;
    p.ObservePose(pose);
    p.Update(true, f, tick * 20);
  }
  if (p.Info().result != route_result::done)
    std::printf("Point failed %s at %.2f %.2f\n", p.Info().status, x, y);
  assert(p.Info().result == route_result::done && std::hypot(x - goal_x, y - goal_y) <= 5);
}

int main() {
  const auto s = Straight();
  drive::turn_config t = config::AutoTurnControl(drive::turn_config());
  t.left_forward = t.left_reverse = t.right_forward = t.right_reverse = s.left_forward;
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
      requested = route::MoveToPoint(x, y, 100, 100);
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
                                   fault == 1 ? 601 : 100);
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
  requested = route::MoveToPoint(0, 0);
  autonomous_program zero;
  zero.Configure(s, t);
  zero.ObservePose(Pose());
  zero.Update(true, Frame(), 0);
  zero.ObservePose(Pose(20));
  zero.Update(true, Frame(20), 20);
  assert(zero.Info().result == route_result::done);
  requested = route::Swing(90, drive::pivot_side::left);
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
