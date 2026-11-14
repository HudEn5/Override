#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
mkdir -p build/host-tests
compiler="${CXX:-g++}"
run() {
  local name="$1"
  shift
  "$compiler" -O2 -std=c++11 -Wall -Wextra -Werror -pedantic "$@" -I include -o "build/host-tests/$name"
  "build/host-tests/$name"
}
run driver tests/driver-control-test.cpp
run sampling-math tests/drive-control-test.cpp
run velocity tests/velocity-control-test.cpp
run heading tests/heading-control-test.cpp
run trapezoid-profile tests/trapezoid-profile-test.cpp
run trapezoid-motion tests/trapezoid-motion-test.cpp
run straight tests/drive-straight-test.cpp
run turn tests/drive-turn-test.cpp
run corner tests/drive-corner-test.cpp
run corner-profile tests/corner-profile-test.cpp
run corner-models tests/corner-speed-test.cpp
run test-control src/test-control.cpp tests/test-control-test.cpp
run calibration-voltage tests/calibration-voltage-test.cpp
run calibration-feedforward tests/calibration-feedforward-test.cpp
run environments tests/program-mode-test.cpp
run autonomous src/autonomous.cpp tests/autonomous-test.cpp
run autonomous-turn src/autonomous.cpp tests/autonomous-turn-test.cpp
run autonomous-corner src/autonomous.cpp tests/autonomous-corner-test.cpp
run empty-route src/autonomous.cpp src/route.cpp tests/empty-autonomous-test.cpp
run competition -pthread -I tests/competition-mocks src/competition-control.cpp tests/competition-control-test.cpp
run sampling-concurrency -pthread -I tests/sampling-mocks src/drive-sampling.cpp tests/drive-sampling-test.cpp
run runtime-concurrency -pthread -I tests/sampling-mocks src/runtime-info.cpp tests/runtime-info-test.cpp
run calibration-concurrency -pthread -DBARTMOSS_CALIBRATION=1 -I tests/sampling-mocks src/runtime-info.cpp tests/runtime-info-test.cpp
