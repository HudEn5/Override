#include "competition-control.h"
#include "robot-config.h"
#include <atomic>
#include <cassert>
#include <cstdio>
#include <thread>

bool vex::competition::bStopAllTasksBetweenModes = true;
bool vex::competition::bStopTasksBetweenModes = true;
vex::competition Competition;

int main() {
  RegisterCompetitionCallbacks();
  assert(Competition.auto_handler == autonomous && Competition.driver_handler == usercontrol);
  assert(!vex::competition::bStopAllTasksBetweenModes && !vex::competition::bStopTasksBetweenModes);
  Competition.auto_handler();
  assert(ReadCompetitionRequest().stage == runtime::competition_stage::autonomous);
  Competition.driver_handler();
  assert(ReadCompetitionRequest().stage == runtime::competition_stage::driver);
  std::atomic<int> done(0);
  std::thread a([&] { for (int i = 0; i < 20000; ++i) autonomous(); ++done; });
  std::thread d([&] { for (int i = 0; i < 20000; ++i) usercontrol(); ++done; });
  unsigned int previous = 0;
  while (done.load() != 2) {
    const auto r = ReadCompetitionRequest();
    assert(r.generation >= previous);
    assert(r.stage == runtime::competition_stage::autonomous || r.stage == runtime::competition_stage::driver);
    previous = r.generation;
  }
  a.join(); d.join();
  assert(ReadCompetitionRequest().generation == 40002);
  std::puts("Competition callback checks passed: VEX registration, notification-only callbacks and mutex snapshots.");
}
