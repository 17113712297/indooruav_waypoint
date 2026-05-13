#ifndef INDOORUAV_WAYPOINT_SMOOTH_VELOCITY_LIMITER_HPP_
#define INDOORUAV_WAYPOINT_SMOOTH_VELOCITY_LIMITER_HPP_

#include <algorithm>
#include <cmath>

namespace indooruav_waypoint {

// 三阶速率限幅器（cascaded rate limiter）。
//
// 输入：本周期目标速度 v_target、步长 dt
// 输出：本周期实际下发速度 v_cmd
//
// 内部状态变量：
//   v_cmd_：当前已下发速度
//   a_cmd_：当前加速度
//   j_cmd_：当前急动度
//
// 每个周期按下式更新：
//   a_des = (v_target - v_cmd) / dt
//   j_des = (a_des   - a_cmd) / dt
//   s     = clamp((j_des - j_cmd) / dt, ±s_max)
//   j_cmd = clamp(j_cmd + s     * dt,  ±j_max)
//   a_cmd = clamp(a_cmd + j_cmd * dt,  ±a_max)
//   v_cmd = clamp(v_cmd + a_cmd * dt,  ±v_max)
//
// 这样保证下发速度 v_cmd 满足：|a| ≤ a_max、|j| ≤ j_max、|s| ≤ s_max。
// snap_max 设为很大的值（例如 1e6）即可退化为二阶限幅器。
//
// 注意：该类不感知物理量纲，使用者应保证 v/a/j/s 各自单位自洽
// （例如 XY 用 m/s, m/s², m/s³, m/s⁴；yaw 用 rad/s, rad/s², …）。

class SmoothVelocityLimiter {
public:
    struct Limits {
        double v_max = 0.0;  // 速度上限（同时作为下限的绝对值）
        double a_max = 0.0;  // 加速度上限
        double j_max = 0.0;  // 急动度上限
        double s_max = 0.0;  // snap 上限（设大值则等效禁用）
    };

    SmoothVelocityLimiter() = default;

    explicit SmoothVelocityLimiter(const Limits& limits) : limits_(limits) {}

    void SetLimits(const Limits& limits) { limits_ = limits; }
    const Limits& GetLimits() const { return limits_; }

    // 完全清零内部状态：v=a=j=0
    void Reset() {
        v_cmd_ = 0.0;
        a_cmd_ = 0.0;
        j_cmd_ = 0.0;
    }

    // 强制把当前下发速度置为某个值（速度初始化场景），加速度/急动度归零
    void ResetToVelocity(double v) {
        v_cmd_ = ClampMagnitude(v, limits_.v_max);
        a_cmd_ = 0.0;
        j_cmd_ = 0.0;
    }

    double GetVelocity() const { return v_cmd_; }
    double GetAcceleration() const { return a_cmd_; }
    double GetJerk() const { return j_cmd_; }

    // 主更新：给定目标速度和步长，输出本周期下发速度。
    // dt 必须 > 0；若不正常调用者应自行处理（例如 Reset()）。
    double Update(double v_target, double dt) {
        if (dt <= 0.0) {
            return v_cmd_;
        }

        // 先把目标速度先夹到 v_max 以内，避免 v_target 远超 v_max 时
        // 限幅器在末端依然以最大加速度逼近一个永不可达的目标。
        const double v_target_clamped = ClampMagnitude(v_target, limits_.v_max);

        const double a_des = (v_target_clamped - v_cmd_) / dt;
        const double j_des = (a_des - a_cmd_) / dt;
        const double s_des = (j_des - j_cmd_) / dt;
        const double s_applied = ClampMagnitude(s_des, limits_.s_max);

        j_cmd_ = ClampMagnitude(j_cmd_ + s_applied * dt, limits_.j_max);
        a_cmd_ = ClampMagnitude(a_cmd_ + j_cmd_ * dt, limits_.a_max);
        v_cmd_ = ClampMagnitude(v_cmd_ + a_cmd_ * dt, limits_.v_max);

        return v_cmd_;
    }

private:
    static double ClampMagnitude(double value, double limit) {
        if (limit <= 0.0) {
            return 0.0;
        }
        return std::max(-limit, std::min(value, limit));
    }

    Limits limits_;
    double v_cmd_ = 0.0;
    double a_cmd_ = 0.0;
    double j_cmd_ = 0.0;
};

}  // namespace indooruav_waypoint

#endif  // INDOORUAV_WAYPOINT_SMOOTH_VELOCITY_LIMITER_HPP_
