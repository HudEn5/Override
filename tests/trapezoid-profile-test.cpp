#include "trapezoid-motion.h"
#include <initializer_list>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

const double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
const double INF_VALUE = std::numeric_limits<double>::infinity();

static void near(double actual, double expected, double tolerance = 1e-8) {
  assert(std::isfinite(actual));
  assert(std::fabs(actual - expected) <= tolerance);
}
static void profileMath() {
  motion::trapezoid_profile known(100, 40, 40, 40);
  assert(known.Valid());
  near(known.Duration(), 3.5);
  near(known.Sample(0.5).position, 5);
  near(known.Sample(0.5).velocity, 20);
  near(known.Sample(3).position, 95);
  near(known.Sample(3).velocity, 20);
  motion::trapezoid_profile triangle(10, 40, 40, 40);
  near(triangle.PeakVelocity(), 20);
  near(triangle.CruiseTime(), 0);
  near(triangle.Duration(), 1);
  motion::trapezoid_profile asymmetric(80, 40, 80, 20);
  near(asymmetric.Duration(), 3.25);
  near(asymmetric.AccelerationTime(), 0.5);
  near(asymmetric.DecelerationTime(), 2);
  near(motion::trapezoid_profile(40, 40, 40, 40).CruiseTime(), 0);
  near(motion::LimitVelocity(10, -10, 20, 10, 1.25), -5);
  near(motion::LimitVelocity(-10, 10, 20, 10, 1.25), 5);
  near(motion::LimitVelocity(20, 0, 10, 5, 0.1), 19.5);
  int count = 0;
  for (double distance : {0.001, 0.1, 1., 5., 20., 100., 1000.})
    for (double maximum : {0.5, 10., 50., 100.})
      for (double acceleration : {1., 10., 70.})
        for (double deceleration : {2., 30., 100.})
          for (double sign : {-1., 1.}) {
            motion::trapezoid_profile curve(sign * distance, maximum, acceleration, deceleration);
            assert(curve.Valid());
            const double dt = curve.Duration() / 2000;
            auto previous = curve.Sample(0);
            double integral = 0;
            for (int i = 1; i <= 2000; ++i) {
              const auto state = curve.Sample(i * dt);
              assert(sign * (state.position - previous.position) >= -1e-8);
              assert(std::fabs(state.velocity) <= maximum + 1e-8);
              const double change = sign * (state.velocity - previous.velocity);
              assert(change <= acceleration * dt + 1e-8);
              assert(change >= -deceleration * dt - 1e-8);
              integral += (previous.velocity + state.velocity) * dt / 2;
              previous = state;
            }
            near(previous.position, sign * distance, 1e-7);
            near(previous.velocity, 0);
            near(integral, sign * distance, distance * 0.0006 + 1e-8);
            ++count;
          }
  assert(count == 504);
  for (double invalid : {0., -1., NAN_VALUE, INF_VALUE}) {
    assert(!motion::trapezoid_profile(10, invalid, 20, 20).Valid());
    assert(!motion::trapezoid_profile(10, 20, invalid, 20).Valid());
    assert(!motion::trapezoid_profile(10, 20, 20, invalid).Valid());
  }
  assert(!motion::trapezoid_profile(NAN_VALUE, 20, 20, 20).Valid());
  assert(!motion::trapezoid_profile(INF_VALUE, 20, 20, 20).Valid());
  assert(!motion::trapezoid_profile(10, 1e300, 20, 20).Valid());
}

int main() {
  profileMath();
  std::puts("Trapezoid math checks passed: 504 profiles, reverse, triangular, asymmetric ramps and limits.");
}
