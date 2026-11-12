#pragma once

#include <cmath>

// 纯数学层：单位可为 cm/s 或 deg/s，但同一次规划必须保持一致。
// 当前实现从静止出发、到静止结束，不依赖 VEX 设备。
namespace motion {

struct ProfileState {
    double position;
    double velocity;
};

class TrapezoidProfile {
public:
    TrapezoidProfile(double distance, double maxVelocity,
                     double acceleration, double deceleration)
        : m_valid(false), m_direction(distance < 0 ? -1.0 : 1.0),
          m_distance(std::fabs(distance)), m_acceleration(acceleration),
          m_deceleration(deceleration), m_peakVelocity(0),
          m_accelerationTime(0), m_cruiseTime(0), m_decelerationTime(0),
          m_accelerationDistance(0), m_totalTime(0) {
        if (!std::isfinite(distance) || !positive(maxVelocity) ||
            !positive(acceleration) || !positive(deceleration)) return;
        if (m_distance == 0) {
            m_valid = true;
            return;
        }

        // 到达最高速度再刹停需要的路程。路程不够则降低峰值，成为三角形。
        const double rampDistance = maxVelocity * maxVelocity *
            (0.5 / acceleration + 0.5 / deceleration);
        if (!std::isfinite(rampDistance)) return;
        m_peakVelocity = m_distance < rampDistance
            ? std::sqrt(m_distance / (0.5 / acceleration + 0.5 / deceleration))
            : maxVelocity;
        m_accelerationTime = m_peakVelocity / acceleration;
        m_decelerationTime = m_peakVelocity / deceleration;
        m_accelerationDistance = 0.5 * m_peakVelocity * m_accelerationTime;
        const double decelerationDistance = 0.5 * m_peakVelocity * m_decelerationTime;
        const double cruiseDistance = std::fmax(0.0,
            m_distance - m_accelerationDistance - decelerationDistance);
        m_cruiseTime = cruiseDistance / m_peakVelocity;
        m_totalTime = m_accelerationTime + m_cruiseTime + m_decelerationTime;
        m_valid = positive(m_peakVelocity) && positive(m_totalTime);
    }

    bool valid() const { return m_valid; }
    double duration() const { return m_totalTime; }          // s
    double peakVelocity() const { return m_peakVelocity; } // 非负峰值
    double accelerationTime() const { return m_accelerationTime; }
    double cruiseTime() const { return m_cruiseTime; }
    double decelerationTime() const { return m_decelerationTime; }

    ProfileState sample(double seconds) const {
        if (!m_valid || !std::isfinite(seconds) || seconds <= 0) return {0, 0};
        if (seconds >= m_totalTime) return {m_direction * m_distance, 0};

        double position, velocity;
        if (seconds < m_accelerationTime) {
            velocity = m_acceleration * seconds;
            position = 0.5 * m_acceleration * seconds * seconds;
        } else if (seconds < m_accelerationTime + m_cruiseTime) {
            velocity = m_peakVelocity;
            position = m_accelerationDistance +
                m_peakVelocity * (seconds - m_accelerationTime);
        } else {
            const double remainingTime = m_totalTime - seconds;
            velocity = m_deceleration * remainingTime;
            position = m_distance - 0.5 * m_deceleration * remainingTime * remainingTime;
        }
        return {m_direction * position, m_direction * velocity};
    }

private:
    static bool positive(double value) { return std::isfinite(value) && value > 0; }
    bool m_valid;
    double m_direction, m_distance, m_acceleration, m_deceleration;
    double m_peakVelocity, m_accelerationTime, m_cruiseTime, m_decelerationTime;
    double m_accelerationDistance, m_totalTime;
};

// 限制反馈修正后的速度指令变化率；反向时先减速到零，再向另一方向加速。
inline double limitVelocity(double current, double target,
                            double acceleration, double deceleration, double dt) {
    if (dt <= 0) return current;
    if ((current > 0 && target < 0) || (current < 0 && target > 0)) {
        const double stopTime = std::fabs(current) / deceleration;
        if (dt <= stopTime) {
            return current + (current > 0 ? -1.0 : 1.0) * deceleration * dt;
        }
        const double magnitude = std::fmin(std::fabs(target), acceleration * (dt - stopTime));
        return target < 0 ? -magnitude : magnitude;
    }
    const double rate = std::fabs(target) > std::fabs(current) ? acceleration : deceleration;
    const double change = rate * dt;
    return current + std::fmax(-change, std::fmin(change, target - current));
}

} // namespace motion
