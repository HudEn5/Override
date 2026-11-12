#pragma once

#include <cmath>

namespace motion {

struct DriveTestInput {
    bool permitted = false;  // 无场控、配置完成且手柄在线。
    bool startStop = false;  // Y 当前状态；按下沿开始/结束整套路线。
    bool stop = false;       // B：立即取消。
    bool neutral = true;     // 摇杆在死区且机构、气动键均已松开。
};

struct DriveTestAction {
    bool ownsDrive = false;
    bool start = false;
    bool cancel = false;
    double motorRevolutions = 0;
};

// 仅生成动作；路线段运行及段间暂停均由调用者通过 motionRunning 表示。
class DriveTestSession {
public:
    explicit DriveTestSession(double longRevs = 3.0) : m_longRevs(longRevs) {}

    bool active() const { return m_active; }

    DriveTestAction Update(const DriveTestInput& input, bool motionRunning) {
        DriveTestAction action;
        const bool pressed = input.startStop && !m_previousStartStop;
        m_previousStartStop = input.startStop;

        if (!input.permitted) {
            action.cancel = m_active;
            action.ownsDrive = m_active;
            m_active = false;
            m_readyForPress = false;
            return action;
        }
        // 初次运行或恢复许可时已按住 Y，必须先在许可期间松开一次。
        if (!input.startStop) m_readyForPress = true;

        if (m_active) {
            action.ownsDrive = true;
            // 用户取消优先于自然完成；当周期继续接管，避免手控覆盖刹车。
            if (pressed || input.stop || !input.neutral) {
                action.cancel = true;
                m_active = false;
            } else if (!motionRunning) {
                m_active = false;
            }
            return action;
        }

        if (m_readyForPress && pressed && !input.stop && input.neutral &&
            std::isfinite(m_longRevs) && m_longRevs > 0) {
            action.ownsDrive = true;
            action.start = true;
            action.motorRevolutions = m_longRevs;
            m_active = true;
        }
        return action;
    }

private:
    double m_longRevs;
    bool m_active = false;
    bool m_readyForPress = false;
    bool m_previousStartStop = false;
};

}  // namespace motion
