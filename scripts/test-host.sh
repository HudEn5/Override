#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -p build/host-tests build/logs
log_file="build/logs/host-tests-$(date +%Y%m%d-%H%M%S)-$$.log"
exec > >(tee "$log_file") 2>&1
printf 'Host test log: %s\n' "$log_file"
compiler="${CXX:-g++}"
run() {
  local name="$1"
  shift
  printf 'Checking %s\n' "$name"
  "$compiler" -O2 -std=c++11 -Wall -Wextra -Werror -pedantic "$@" -I include -o "build/host-tests/$name"
  "build/host-tests/$name"
}
run driver tests/driver-control-test.cpp
run sampling-math tests/drive-control-test.cpp
run odometry src/odometry.cpp tests/odometry-test.cpp
run velocity tests/velocity-control-test.cpp
run heading tests/heading-control-test.cpp
run trapezoid-profile tests/trapezoid-profile-test.cpp
run trapezoid-motion tests/trapezoid-motion-test.cpp
run straight tests/drive-straight-test.cpp
run turn tests/drive-turn-test.cpp
run turn-profile tests/turn-profile-test.cpp
run corner tests/drive-corner-test.cpp
run corner-profile tests/corner-profile-test.cpp
run corner-models tests/corner-speed-test.cpp
run test-control src/test-control.cpp tests/test-control-test.cpp
run calibration-voltage tests/calibration-voltage-test.cpp
run calibration-feedforward tests/calibration-feedforward-test.cpp
run environments tests/program-mode-test.cpp
run autonomous src/autonomous.cpp tests/autonomous-test.cpp
run api-consistency src/autonomous.cpp tests/api-consistency-test.cpp
run point-swing src/autonomous.cpp tests/point-swing-test.cpp
run autonomous-turn src/autonomous.cpp tests/autonomous-turn-test.cpp
run autonomous-corner src/autonomous.cpp tests/autonomous-corner-test.cpp
run rolling-route src/autonomous.cpp tests/rolling-route-test.cpp
run route-entry src/autonomous.cpp tests/route-entry-test.cpp
run current-route -DBARTMOSS_CURRENT_ROUTE=1 src/autonomous.cpp src/route.cpp tests/route-entry-test.cpp
run route-sequence src/autonomous.cpp tests/route-sequence-test.cpp
run route-timeout src/autonomous.cpp tests/route-timeout-test.cpp
run mechanism-route src/autonomous.cpp tests/mechanism-route-test.cpp
run mechanism-backend tests/mechanism-backend-test.cpp
run direct-mechanism src/autonomous.cpp tests/direct-mechanism-test.cpp
run timed-push tests/drive-push-test.cpp
run autonomous-push src/autonomous.cpp tests/autonomous-push-test.cpp
run startup-mode tests/startup-mode-test.cpp
run startup-route src/autonomous.cpp tests/startup-route-test.cpp
run competition -pthread -I tests/competition-mocks src/competition-control.cpp tests/competition-control-test.cpp
run sampling-concurrency -pthread -I tests/sampling-mocks src/drive-sampling.cpp tests/drive-sampling-test.cpp
run runtime-concurrency -pthread -I tests/sampling-mocks src/runtime-info.cpp tests/runtime-info-test.cpp
run calibration-concurrency -pthread -DBARTMOSS_CALIBRATION=1 -I tests/sampling-mocks src/runtime-info.cpp tests/runtime-info-test.cpp

run high-speed tests/high-speed-test.cpp
run performance tests/performance-test.cpp
run high-speed-concurrency -pthread -DBARTMOSS_HIGH_SPEED=1 -I tests/sampling-mocks src/runtime-info.cpp tests/runtime-info-test.cpp

# 旧最大参数个数必须报缺少timeout，禁止默默给出默认超时。
for kind in {0..16}; do
  if "$compiler" -std=c++11 -fsyntax-only -I include -DMISSING_ROUTE_TIMEOUT="$kind" tests/route-timeout-test.cpp > build/host-tests/missing-timeout.log 2>&1; then
    printf 'Missing timeout unexpectedly compiled: %s\n' "$kind"
    exit 1
  fi
  if grep -q 'too few arguments' build/host-tests/missing-timeout.log; then
    continue
  fi
  # ClawClose 有重载，GCC 将缺参报为没有匹配的函数，再列出候选参数数量。
  if [[ "$kind" == 14 ]] && grep -q 'candidate expects 3 arguments, 2 provided' build/host-tests/missing-timeout.log; then
    continue
  fi
  cat build/host-tests/missing-timeout.log
  exit 1
done
printf 'All seventeen route factories reject missing timeout at compile time.\n'
