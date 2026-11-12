#include "drive-test.h"

#include <cassert>
#include <cstdio>
#include <limits>

using motion::DriveTestAction;
using motion::DriveTestInput;
using motion::DriveTestSession;

static void idle(const DriveTestAction& action) {
    assert(!action.ownsDrive && !action.start && !action.cancel);
    assert(action.motorRevolutions == 0);
}

static void owned(const DriveTestAction& action) {
    assert(action.ownsDrive && !action.start && !action.cancel);
    assert(action.motorRevolutions == 0);
}

static void cancelled(const DriveTestAction& action) {
    assert(action.ownsDrive && !action.start && action.cancel);
    assert(action.motorRevolutions == 0);
}

static void started(const DriveTestAction& action, double revolutions = 3.0) {
    assert(action.ownsDrive && action.start && !action.cancel);
    assert(action.motorRevolutions == revolutions);
}

static void startupAndReconnect() {
    DriveTestSession session;
    DriveTestInput input;
    assert(!session.active());
    input.startStop = true;
    idle(session.Update(input, false));
    input.permitted = true;
    idle(session.Update(input, false));
    idle(session.Update(input, false));
    input.startStop = false;
    idle(session.Update(input, false));
    input.startStop = true;
    started(session.Update(input, false));
    assert(session.active());
    input.permitted = false;
    cancelled(session.Update(input, true));
    assert(!session.active());
    idle(session.Update(input, true));
    // Releasing Y while permission is absent does not arm reconnection.
    input.startStop = false;
    idle(session.Update(input, false));
    input.permitted = input.startStop = true;
    idle(session.Update(input, false));
    input.startStop = false;
    idle(session.Update(input, false));
    input.startStop = true;
    started(session.Update(input, false));
}

static void toggleAndHold() {
    DriveTestSession session;
    DriveTestInput input;
    input.permitted = true;
    idle(session.Update(input, false));
    input.startStop = true;
    started(session.Update(input, false));
    // Startup does not complete itself just because the route was not running yet.
    assert(session.active());
    for (int i = 0; i < 20; ++i) owned(session.Update(input, true));
    input.startStop = false;
    owned(session.Update(input, true));
    assert(session.active());
    input.startStop = true;
    cancelled(session.Update(input, true));
    assert(!session.active());
    idle(session.Update(input, false));
    input.startStop = false;
    idle(session.Update(input, false));
    input.startStop = true;
    started(session.Update(input, false));
}

static void completionAndRunningPauses() {
    DriveTestSession session;
    DriveTestInput input;
    input.permitted = true;
    idle(session.Update(input, false));
    input.startStop = true;
    started(session.Update(input, false));
    // The caller reports route activity as true during pauses between segments too.
    owned(session.Update(input, true));
    owned(session.Update(input, true));
    owned(session.Update(input, false));
    assert(!session.active());
    idle(session.Update(input, false));
    idle(session.Update(input, false));
    input.startStop = false;
    idle(session.Update(input, false));
    input.startStop = true;
    started(session.Update(input, false));
    input.startStop = false;
    owned(session.Update(input, true));
    input.startStop = true;
    // A stop press on the completion tick cancels; it never starts a new route.
    cancelled(session.Update(input, false));
    assert(!session.active());
    idle(session.Update(input, false));
}

static void safetyAndRecovery() {
    for (int reason = 0; reason < 3; ++reason) {
        DriveTestSession session;
        DriveTestInput input;
        input.permitted = true;
        idle(session.Update(input, false));
        input.startStop = true;
        started(session.Update(input, false));
        if (reason == 0) input.stop = true;
        if (reason == 1) input.neutral = false;
        if (reason == 2) input.permitted = false;
        cancelled(session.Update(input, true));
        assert(!session.active());
        idle(session.Update(input, false));
        input.stop = false;
        input.neutral = input.permitted = true;
        // Clearing a fault while Y is still held cannot restart the route.
        idle(session.Update(input, false));
        input.startStop = false;
        idle(session.Update(input, false));
        input.startStop = true;
        started(session.Update(input, false));
    }
    // Safety cancellation also wins on the natural completion tick.
    for (int reason = 0; reason < 3; ++reason) {
        DriveTestSession session;
        DriveTestInput input;
        input.permitted = true;
        idle(session.Update(input, false));
        input.startStop = true;
        started(session.Update(input, false));
        if (reason == 0) input.stop = true;
        if (reason == 1) input.neutral = false;
        if (reason == 2) input.permitted = false;
        cancelled(session.Update(input, false));
    }
}

static void invalidStartAndFieldIsolation() {
    for (int reason = 0; reason < 3; ++reason) {
        DriveTestSession session;
        DriveTestInput input;
        input.permitted = true;
        idle(session.Update(input, false));
        input.startStop = true;
        if (reason == 0) input.stop = true;
        if (reason == 1) input.neutral = false;
        if (reason == 2) input.permitted = false;
        idle(session.Update(input, false));
        assert(!session.active());
        input.stop = false;
        input.neutral = input.permitted = true;
        idle(session.Update(input, false));
        input.startStop = false;
        idle(session.Update(input, false));
        input.startStop = true;
        started(session.Update(input, false));
    }
    DriveTestSession session;
    DriveTestInput input;
    // Motion belonging to field control grants this module no ownership or cancel rights.
    input.startStop = input.stop = true;
    input.neutral = false;
    idle(session.Update(input, true));
    idle(session.Update(input, true));
    input.permitted = input.neutral = true;
    input.stop = input.startStop = false;
    idle(session.Update(input, true));
}

static void customAndInvalidDistances() {
    DriveTestInput input;
    input.permitted = true;
    DriveTestSession custom(4.5);
    idle(custom.Update(input, false));
    input.startStop = true;
    started(custom.Update(input, false), 4.5);
    const double invalid[] = {0.0, -1.0,
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()};
    for (unsigned int i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        DriveTestSession bad(invalid[i]);
        input = DriveTestInput();
        input.permitted = true;
        idle(bad.Update(input, false));
        input.startStop = true;
        idle(bad.Update(input, false));
        assert(!bad.active());
    }
}

int main() {
    startupAndReconnect();
    toggleAndHold();
    completionAndRunningPauses();
    safetyAndRecovery();
    invalidStartAndFieldIsolation();
    customAndInvalidDistances();
    std::puts("Drive route button checks passed (Y toggle, completion, interlocks, reconnect).");
}
