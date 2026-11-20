#include "performance-config.h"
#include <cassert>
#include <cstdio>

struct result {
  bool done = false;
  double elapsed = 0, peak = 0, heading = 0, remaining = 0;
  const char* status = "";
};
drive::feedforward_config Model(double ks, double kv) {
  drive::feedforward_config f;
  f.kS = ks;
  f.kV = kv;
  f.calibrated = true;
  return f;
}
result Sim(bool experimental, int direction, double la, double ra, double drag, double distance) {
  drive::straight_config base;
  base.motion.wheel_diameter_mm = 69.85;
  base.motion.motor_revs_per_wheel_rev = 1.2;
  base.left_forward = Model(372.573735, 17.900544);
  base.right_forward = Model(408.162564, 16.537990);
  base.left_reverse = Model(547.829766, 16.939618);
  base.right_reverse = Model(537.033135, 15.870378);
  const auto cfg = experimental ? config::RouteStraightControl(base) : config::HighSpeedStraightControl(base);
  drive::straight_controller c;
  c.Configure(cfg);
  drive::straight_feedback f;
  f.velocity.valid = f.position.valid = f.motors_online = f.inertial_ready = f.stopped = true;
  c.Observe(f, true, 0);
  assert(c.StartDriveStraight(direction * distance, experimental ? config::ROUTE_DRIVE_RPM : 330));
  double l = 0, r = 0;
  result out;
  const double mm = 69.85 * 3.141592653589793 / (60 * 1.2);
  for (int tick = 1; tick <= 1000; ++tick) {
    const auto old = c.Info();
    const auto motor = [&](int mv, bool left) {
      const auto model = left ? (mv < 0 ? cfg.left_reverse : cfg.left_forward)
                              : (mv < 0 ? cfg.right_reverse : cfg.right_forward);
      return std::copysign(std::max(0.0, std::fabs(static_cast<double>(mv)) - model.kS - drag) / model.kV,
                           mv);
    };
    l += (motor(old.left_mV, true) * .97 - l) * la;
    r += (motor(old.right_mV, false) * 1.03 - r) * ra;
    f.position.left_deg += l * .12;
    f.position.right_deg += r * .12;
    f.heading_deg += (r - l) * mm / 325 * 180 / 3.141592653589793 * .02;
    f.velocity.left_rpm = l;
    f.velocity.right_rpm = r;
    f.velocity.time_ms = tick * 20;
    f.velocity.sequence++;
    f.stopped = std::fabs(l) < 5 && std::fabs(r) < 5;
    c.Observe(f, true, tick * 20);
    c.UpdateDriveStraight();
    const auto v = c.Info();
    assert(std::abs(v.left_mV) <= 12000 && std::abs(v.right_mV) <= 12000);
    out.elapsed = v.motion.elapsed_ms;
    out.peak = std::max(out.peak, direction * (l + r) / 2);
    out.heading = std::max(out.heading, std::fabs(f.heading_deg));
    out.remaining = v.motion.remaining_mm;
    out.status = v.status;
    if (v.state != drive::straight_state::running) {
      out.done = v.state == drive::straight_state::done;
      break;
    }
  }
  return out;
}
int main() {
  drive::turn_config turn;
  turn.acceleration_rpm_s = 360;
  const auto legacy_turn = config::RouteTurnControl(turn, false);
  assert(legacy_turn.acceleration_rpm_s == 240 && !legacy_turn.profile_enabled);
  assert(legacy_turn.kP == turn.kP && legacy_turn.kD == turn.kD);
  const auto fast_turn = config::RouteTurnControl(turn, true);
  assert(fast_turn.acceleration_rpm_s == 640 && fast_turn.profile_enabled == config::TURN_ANGULAR_PROFILE);
  unsigned int count = 0;
  double before_time = 0, after_time = 0, max_error = 0, max_heading = 0;
  for (int direction : {-1, 1})
    for (int response = 0; response < 3; ++response)
      for (double drag : {0., 200.}) {
        const double la = response == 1 ? .1 : .25, ra = response == 2 ? .1 : (response == 1 ? .35 : .25);
        const auto before = Sim(false, direction, la, ra, drag, 3000);
        const auto after = Sim(true, direction, la, ra, drag, 3000);
        if (!before.done || !after.done)
          std::printf(
              "Performance straight failed: direction %d response %d drag %.0f base %s fast %s error %.3f\n",
              direction, response, drag, before.status, after.status, after.remaining);
        std::fflush(stdout);
        assert(before.done && after.done && std::fabs(after.remaining) <= 5);
        before_time += before.elapsed;
        after_time += after.elapsed;
        max_error = std::max(max_error, std::fabs(after.remaining));
        max_heading = std::max(max_heading, after.heading);
        ++count;
      }
  const auto nominal_before = Sim(false, 1, .25, .25, 0, 3000);
  const auto nominal_after = Sim(true, 1, .25, .25, 0, 3000);
  std::printf(
      "Performance straight: %u paired 3000mm cases, mean %.0f -> %.0f ms, nominal %.0f -> %.0f ms, peak "
      "%.1f motor rpm / %.1f wheel rpm, endpoint %.3f mm, heading %.3f deg.\n",
      count, before_time / count, after_time / count, nominal_before.elapsed, nominal_after.elapsed,
      nominal_after.peak, nominal_after.peak / 1.2, max_error, max_heading);
  if (config::EXPERIMENTAL_FAST_ROUTE)
    assert(nominal_after.elapsed < nominal_before.elapsed);
  const auto short_run = Sim(true, 1, .25, .25, 0, 500);
  assert(short_run.done && short_run.peak < config::ROUTE_DRIVE_RPM);
  std::printf(
      "Short 500mm case: actual model peak %.1f motor rpm / %.1f wheel rpm, %.0f ms; requested limit need "
      "not be reached.\n",
      short_run.peak, short_run.peak / 1.2, short_run.elapsed);
}
