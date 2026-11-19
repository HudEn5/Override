#include "autonomous.h"
#include <cassert>
#include <cstdio>

static route::command requested = route::DriveStraight(500, 100, 100000);
void autonomous_program::RunRoutine() {
  RunRoute({requested});
}
drive::straight_config Settings() {
  drive::straight_config s;
  s.motion.wheel_diameter_mm = 69.85;
  s.left_forward.calibrated = true;
  s.left_forward.kS = 400;
  s.left_forward.kV = 16;
  s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
  return s;
}
drive::straight_feedback Frame(double now) {
  drive::straight_feedback f;
  f.stopped = f.position.valid = f.velocity.valid = f.inertial_ready = f.motors_online = true;
  f.velocity.sequence = static_cast<unsigned int>(now / 20 + 1);
  f.velocity.time_ms = now;
  return f;
}
int main() {
  autonomous_program p;
  p.Configure(Settings());
  for (int tick = 0; tick < 3; ++tick)
    p.Update(true, Frame(tick * 20), tick * 20);
  assert(p.Info().active && p.Info().left_mV != 0);
  p.CancelRoutine();
  assert(!p.Info().active && p.Info().left_mV == 0 && p.Info().right_mV == 0);
  assert(p.Info().result == route_result::cancelled && p.Info().drive.left_mV == 0);
  drive::velocity_control v;
  drive::velocity_pid_config cfg;
  v.ConfigurePID(cfg);
  drive::velocity_feedback speed;
  speed.valid = true;
  speed.sequence = 1;
  speed.time_ms = 1000;
  v.Observe(speed, 1000);
  assert(v.Fresh(1000));
  v.ResetControl();
  assert(v.Fresh(1000));
  v.ConfigurePID(cfg);
  assert(!v.Fresh(1000));
  speed.time_ms = 0;
  v.Observe(speed, 0);
  assert(v.Fresh(0));
  assert(!v.Fresh(101));
  const auto swing = route::Swing(-90, drive::pivot_side::right, 200, 100000);
  assert(swing.fixed_side == drive::pivot_side::right && swing.angle_deg == -90 && swing.maximum_rpm == 200);
  const auto push = route::PushFor(1000, 4, 100000);
  assert(push.duration_ms == 1000 && push.volts == 4);
  requested = route::MoveToPoint(0, 1500, 100, 100, 100000);
  drive::turn_config t;
  t.left_forward = t.left_reverse = t.right_forward = t.right_reverse = Settings().left_forward;
  autonomous_program point;
  point.Configure(Settings(), t);
  drive::pose pos;
  pos.valid = true;
  point.ObservePose(pos);
  point.Update(true, Frame(0), 0);
  pos.heading_deg = 89.3;
  pos.time_ms = 20;
  auto f = Frame(20);
  f.heading_deg = 89.3;
  point.ObservePose(pos);
  point.Update(true, f, 20);
  for (int now = 40; now <= 1000; now += 20) {
    f = Frame(now);
    f.heading_deg = 89.3;
    pos.time_ms = now;
    point.ObservePose(pos);
    point.Update(true, f, now);
  }
  // 0.7度虽满足普通转向0.8度，但不足以完成1.5m/5mm点位，不能提前直行。
  assert(point.Info().motion == autonomous_motion::turn);
  drive::point_config navigation;
  navigation.position_tolerance_mm = 50;
  navigation.pose_stale_ms = 20;
  autonomous_program stale;
  stale.Configure(Settings(), t);
  stale.ConfigurePoint(navigation);
  drive::pose start;
  start.valid = true;
  stale.ObservePose(start);
  stale.Update(true, Frame(0), 0);
  stale.Update(true, Frame(40), 40);
  assert(stale.Info().result == route_result::error);
  auto low = Settings(), high = Settings();
  high.max_heading_speed_fraction = .5;
  drive::straight_controller a, b;
  a.Configure(low);
  b.Configure(high);
  a.Observe(Frame(0), true, 0);
  b.Observe(Frame(0), true, 0);
  assert(a.StartDriveStraight(500, 100) && b.StartDriveStraight(500, 100));
  auto offset = Frame(20);
  offset.heading_deg = 1;
  a.Observe(offset, true, 20);
  b.Observe(offset, true, 20);
  a.UpdateDriveStraight();
  b.UpdateDriveStraight();
  assert(std::fabs(b.Info().heading.correction_mV) > std::fabs(a.Info().heading.correction_mV) * 1.9);
  std::puts(
      "Consistency checks passed: immediate cancel, warm reset/cold configure, typed units and "
      "distance-aware point heading.");
}
