#pragma once

#include "drive-motion.h"

#include <cmath>

namespace motion {

enum class DriveRouteStage { Idle, Straight, Pause, Turning, Done, Failed, Cancelled };

inline const char* RouteStageName(DriveRouteStage stage) {
    switch (stage) {
        case DriveRouteStage::Idle: return "IDLE";
        case DriveRouteStage::Straight: return "STRAIGHT";
        case DriveRouteStage::Pause: return "PAUSE";
        case DriveRouteStage::Turning: return "TURN";
        case DriveRouteStage::Done: return "DONE";
        case DriveRouteStage::Failed: return "FAILED";
        case DriveRouteStage::Cancelled: return "CANCEL";
    }
    return "UNKNOWN";
}

struct DriveRouteAction {
    bool startStraight = false;
    bool startTurn = false;
};

// 只协调直行、停车等待和转向；硬件层在动作发出时启动对应运动。
class DriveTestRoute {
public:
    explicit DriveTestRoute(double pauseSec = 0.6) : m_pauseSec(pauseSec) {}

    DriveRouteAction Start(double nowSec) {
        DriveRouteAction action;
        if (!ValidTime(nowSec) || !ValidTime(m_pauseSec)) {
            m_stage = DriveRouteStage::Failed;
            return action;
        }
        m_previousSec = nowSec;
        m_pauseStartedSec = nowSec;
        m_stage = DriveRouteStage::Straight;
        action.startStraight = true;
        return action;
    }

    DriveRouteAction Step(MoveResult result, double nowSec, bool permitted) {
        DriveRouteAction action;
        if (!active()) return action;
        if (!permitted) {
            Cancel();
            return action;
        }
        if (!ValidTime(nowSec) || nowSec < m_previousSec) {
            m_stage = DriveRouteStage::Failed;
            return action;
        }
        m_previousSec = nowSec;

        if (m_stage == DriveRouteStage::Pause) {
            if (nowSec - m_pauseStartedSec >= m_pauseSec) {
                m_stage = DriveRouteStage::Turning;
                action.startTurn = true;
            }
        } else if (result == MoveResult::Completed) {
            if (m_stage == DriveRouteStage::Straight) {
                m_stage = DriveRouteStage::Pause;
                m_pauseStartedSec = nowSec;
            } else {
                m_stage = DriveRouteStage::Done;
            }
        } else if (result != MoveResult::Running) {
            m_stage = DriveRouteStage::Failed;
        }
        return action;
    }

    bool active() const {
        return m_stage == DriveRouteStage::Straight ||
            m_stage == DriveRouteStage::Pause || m_stage == DriveRouteStage::Turning;
    }

    void Cancel() {
        if (active()) m_stage = DriveRouteStage::Cancelled;
    }

    DriveRouteStage stage() const { return m_stage; }

private:
    static bool ValidTime(double value) { return std::isfinite(value) && value >= 0; }

    double m_pauseSec;
    double m_previousSec = 0;
    double m_pauseStartedSec = 0;
    DriveRouteStage m_stage = DriveRouteStage::Idle;
};

}  // namespace motion
