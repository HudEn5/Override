#include "drive-test-route.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

using motion::DriveRouteAction;
using motion::DriveRouteStage;
using motion::DriveTestRoute;
using motion::MoveResult;

static void noAction(const DriveRouteAction& action) {
    assert(!action.startStraight && !action.startTurn);
}

static void startStraight(const DriveRouteAction& action) {
    assert(action.startStraight && !action.startTurn);
}

static void startTurn(const DriveRouteAction& action) {
    assert(!action.startStraight && action.startTurn);
}

static void completeRouteAndSingleTriggers() {
    DriveTestRoute route(0.5);
    assert(!route.active() && route.stage() == DriveRouteStage::Idle);
    noAction(route.Step(MoveResult::Completed, 10, true));
    startStraight(route.Start(10));
    assert(route.active() && route.stage() == DriveRouteStage::Straight);
    noAction(route.Step(MoveResult::Running, 10, true));
    noAction(route.Step(MoveResult::Running, 10.5, true));
    noAction(route.Step(MoveResult::Completed, 11, true));
    assert(route.active() && route.stage() == DriveRouteStage::Pause);
    // The previous segment retains Completed throughout the pause.
    noAction(route.Step(MoveResult::Completed, 11.25, true));
    assert(route.stage() == DriveRouteStage::Pause);
    startTurn(route.Step(MoveResult::Completed, 11.5, true));
    assert(route.active() && route.stage() == DriveRouteStage::Turning);
    noAction(route.Step(MoveResult::Running, 11.5, true));
    noAction(route.Step(MoveResult::Running, 12, true));
    noAction(route.Step(MoveResult::Completed, 12.5, true));
    assert(!route.active() && route.stage() == DriveRouteStage::Done);
    noAction(route.Step(MoveResult::Completed, 20, true));
    noAction(route.Step(MoveResult::Running, 21, true));
    route.Cancel();
    assert(route.stage() == DriveRouteStage::Done);
    // A new run may restart its clock and must begin with the straight segment.
    startStraight(route.Start(0));
    assert(route.stage() == DriveRouteStage::Straight && route.active());
}

static void pauseAndCompletionBoundaries() {
    DriveTestRoute route;
    startStraight(route.Start(0));
    noAction(route.Step(MoveResult::Completed, 1, true));
    noAction(route.Step(MoveResult::Completed, 1.59, true));
    startTurn(route.Step(MoveResult::Completed, 1.61, true));
    noAction(route.Step(MoveResult::Completed, 1.62, true));
    assert(route.stage() == DriveRouteStage::Done);

    DriveTestRoute immediate(0);
    startStraight(immediate.Start(0));
    noAction(immediate.Step(MoveResult::Completed, 0, true));
    assert(immediate.stage() == DriveRouteStage::Pause);
    startTurn(immediate.Step(MoveResult::Completed, 0, true));
    assert(immediate.stage() == DriveRouteStage::Turning);
}

static void segmentFailures() {
    const MoveResult failures[] = {MoveResult::Idle, MoveResult::InvalidParameters,
        MoveResult::TimedOut, MoveResult::MotorDisconnected, MoveResult::InvalidFeedback,
        MoveResult::Interrupted, MoveResult::NotStationary};
    for (unsigned int i = 0; i < sizeof(failures) / sizeof(failures[0]); ++i) {
        for (int segment = 0; segment < 2; ++segment) {
            DriveTestRoute route(0.5);
            startStraight(route.Start(0));
            if (segment) {
                noAction(route.Step(MoveResult::Completed, 1, true));
                startTurn(route.Step(MoveResult::Completed, 1.5, true));
            }
            noAction(route.Step(failures[i], 2, true));
            assert(route.stage() == DriveRouteStage::Failed && !route.active());
            noAction(route.Step(MoveResult::Completed, 3, true));
            route.Cancel();
            assert(route.stage() == DriveRouteStage::Failed);
            startStraight(route.Start(0));
        }
    }
}

static void cancellationAtEveryStage() {
    for (int stage = 0; stage < 3; ++stage) {
        for (int explicitCancel = 0; explicitCancel < 2; ++explicitCancel) {
            DriveTestRoute route(0.5);
            startStraight(route.Start(0));
            if (stage >= 1) noAction(route.Step(MoveResult::Completed, 1, true));
            if (stage >= 2) startTurn(route.Step(MoveResult::Completed, 1.5, true));
            if (explicitCancel) route.Cancel();
            else noAction(route.Step(MoveResult::Completed, 2, false));
            assert(route.stage() == DriveRouteStage::Cancelled && !route.active());
            noAction(route.Step(MoveResult::Completed, 4, true));
            route.Cancel();
            assert(route.stage() == DriveRouteStage::Cancelled);
            startStraight(route.Start(0));
            assert(route.active());
        }
    }
    DriveTestRoute idle;
    idle.Cancel();
    noAction(idle.Step(MoveResult::Running, 0, false));
    assert(idle.stage() == DriveRouteStage::Idle);
}

static void invalidTimesAndPause() {
    const double invalid[] = {-1, std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()};
    for (unsigned int i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        DriveTestRoute badPause(invalid[i]);
        noAction(badPause.Start(0));
        assert(badPause.stage() == DriveRouteStage::Failed && !badPause.active());
        DriveTestRoute badStart;
        noAction(badStart.Start(invalid[i]));
        assert(badStart.stage() == DriveRouteStage::Failed && !badStart.active());
        startStraight(badStart.Start(0));
        noAction(badStart.Step(MoveResult::Running, invalid[i], true));
        assert(badStart.stage() == DriveRouteStage::Failed && !badStart.active());
    }
    for (int stage = 0; stage < 3; ++stage) {
        DriveTestRoute route(0.5);
        startStraight(route.Start(10));
        if (stage >= 1) noAction(route.Step(MoveResult::Completed, 11, true));
        if (stage >= 2) startTurn(route.Step(MoveResult::Completed, 11.5, true));
        noAction(route.Step(MoveResult::Running, stage == 0 ? 9 : 10, true));
        assert(route.stage() == DriveRouteStage::Failed && !route.active());
    }
}

static void stageNames() {
    const DriveRouteStage stages[] = {DriveRouteStage::Idle, DriveRouteStage::Straight,
        DriveRouteStage::Pause, DriveRouteStage::Turning, DriveRouteStage::Done,
        DriveRouteStage::Failed, DriveRouteStage::Cancelled};
    const char* names[] = {"IDLE", "STRAIGHT", "PAUSE", "TURN", "DONE", "FAILED", "CANCEL"};
    for (unsigned int i = 0; i < sizeof(stages) / sizeof(stages[0]); ++i) {
        assert(std::strcmp(motion::RouteStageName(stages[i]), names[i]) == 0);
    }
}

int main() {
    completeRouteAndSingleTriggers();
    pauseAndCompletionBoundaries();
    segmentFailures();
    cancellationAtEveryStage();
    invalidTimesAndPause();
    stageNames();
    std::puts("Drive test route checks passed (segments, pause, edges, faults, cancellation).");
}
