#include "trapezoid-profile.h"
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
  motion::TrapezoidProfile known(100, 40, 40, 40);
  assert(known.valid());
  near(known.duration(), 3.5);
  near(known.sample(0.5).position, 5);
  near(known.sample(0.5).velocity, 20);
  near(known.sample(3).position, 95);
  near(known.sample(3).velocity, 20);
  motion::TrapezoidProfile triangle(10, 40, 40, 40);
  near(triangle.peakVelocity(), 20);
  near(triangle.cruiseTime(), 0);
  near(triangle.duration(), 1);
  motion::TrapezoidProfile asymmetric(80, 40, 80, 20);
  near(asymmetric.duration(), 3.25);
  near(asymmetric.accelerationTime(), 0.5);
  near(asymmetric.decelerationTime(), 2);
  near(motion::TrapezoidProfile(40, 40, 40, 40).cruiseTime(), 0);
  near(motion::limitVelocity(10, -10, 20, 10, 1.25), -5);
  near(motion::limitVelocity(-10, 10, 20, 10, 1.25), 5);
  near(motion::limitVelocity(20, 0, 10, 5, 0.1), 19.5);
  int count = 0;
  for (double distance : {0.001, 0.1, 1., 5., 20., 100., 1000.})
  for (double maximum : {0.5, 10., 50., 100.})
  for (double acceleration : {1., 10., 70.})
  for (double deceleration : {2., 30., 100.})
  for (double sign : {-1., 1.}) {
    motion::TrapezoidProfile curve(sign * distance, maximum, acceleration, deceleration);
    assert(curve.valid());
    const double dt = curve.duration() / 2000;
    auto previous = curve.sample(0);
    double integral = 0;
    for (int i = 1; i <= 2000; ++i) {
      const auto state = curve.sample(i * dt);
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
    assert(!motion::TrapezoidProfile(10, invalid, 20, 20).valid());
    assert(!motion::TrapezoidProfile(10, 20, invalid, 20).valid());
    assert(!motion::TrapezoidProfile(10, 20, 20, invalid).valid());
  }
  assert(!motion::TrapezoidProfile(NAN_VALUE, 20, 20, 20).valid());
  assert(!motion::TrapezoidProfile(INF_VALUE, 20, 20, 20).valid());
  assert(!motion::TrapezoidProfile(10, 1e300, 20, 20).valid());
}

int main() {
  profileMath();
  std::puts("Trapezoid math checks passed: 504 profiles, reverse, triangular, asymmetric ramps and limits.");
}
