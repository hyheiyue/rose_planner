#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

namespace rose_planner {

struct TrajectoryPoint {
    Eigen::Vector2f p;
    Eigen::Vector2f v;
    float t;
};

inline float segmentLength(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
    return (b - a).norm();
}

inline std::vector<float> computeArcLengths(const std::vector<Eigen::Vector3f>& path) {
    std::vector<float> s(path.size(), 0.0f);
    for (size_t i = 1; i < path.size(); ++i) {
        Eigen::Vector2f p0(path[i - 1].x(), path[i - 1].y());
        Eigen::Vector2f p1(path[i].x(), path[i].y());
        s[i] = s[i - 1] + segmentLength(p0, p1);
    }
    return s;
}

inline float
velocityAtS(float s, float max_vel, float acc, float dec, float s_acc, float s_dec, float s_flat) {
    if (s < s_acc) {
        return std::sqrt(2.0f * acc * s);
    } else if (s < s_acc + s_flat) {
        return max_vel;
    } else {
        float s_remain = (s_acc + s_flat + s_dec) - s;
        return std::sqrt(2.0f * dec * std::max(s_remain, 0.0f));
    }
}

inline Eigen::Vector2f
tangentAtS(float s_query, const std::vector<Eigen::Vector3f>& path, const std::vector<float>& s) {
    if (s_query <= 0) {
        return (Eigen::Vector2f(path[1].x(), path[1].y())
                - Eigen::Vector2f(path[0].x(), path[0].y()))
            .normalized();
    }
    if (s_query >= s.back()) {
        size_t n = path.size();
        return (Eigen::Vector2f(path[n - 1].x(), path[n - 1].y())
                - Eigen::Vector2f(path[n - 2].x(), path[n - 2].y()))
            .normalized();
    }
    for (size_t i = 1; i < s.size(); ++i) {
        if (s_query <= s[i]) {
            Eigen::Vector2f p0(path[i - 1].x(), path[i - 1].y());
            Eigen::Vector2f p1(path[i].x(), path[i].y());
            return (p1 - p0).normalized();
        }
    }
    return Eigen::Vector2f(1, 0);
}

inline std::vector<TrajectoryPoint> sampleTrajectoryTrapezoid(
    const std::vector<Eigen::Vector3f>& path,
    float max_vel,
    float acc,
    float dec,
    float dt,
    float max_yaw_rate,
    const Eigen::Vector2f& init_v = Eigen::Vector2f(0, 0)
) {
    if (path.size() < 2)
        return {};

    auto s = computeArcLengths(path);
    float s_total = s.back();

    float maxv = max_vel;
    float s_acc = (maxv * maxv) / (2.0f * acc);
    float s_dec = (maxv * maxv) / (2.0f * dec);
    float s_flat = 0.0f;

    if (s_acc + s_dec > s_total) {
        maxv = std::sqrt(2.0f * acc * dec * s_total / (acc + dec));
        s_acc = (maxv * maxv) / (2.0f * acc);
        s_dec = (maxv * maxv) / (2.0f * dec);
    }
    s_flat = s_total - s_acc - s_dec;
    s_flat = std::max(s_flat, 0.0f);

    std::vector<TrajectoryPoint> traj;
    traj.reserve(1024);

    float t = 0.0f;
    float s_now = 0.0f;

    // 初始 yaw 仍由 path 几何决定
    float prev_yaw = std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x());

    // 初始速度投影到路径切线方向，作为初始弧长速度
    Eigen::Vector2f init_tan = tangentAtS(0.0f, path, s);
    float v0 = init_v.dot(init_tan);
    v0 = std::clamp(v0, 0.0f, maxv); // 不能反向、不能超过最大速度

    // 根据初始弧长速度计算已等效推进的初始距离（逆向恢复s，使v(0)=v0）
    float s_init = (v0 * v0) / (2.0f * acc);
    s_now = s_init; // 让梯形曲线从这个距离开始推进，就能保证 v(0)=v0

    const int MAX_ITERS = 20000;
    int iter = 0;

    while (s_now <= s_total + 1e-5f) {
        float v = velocityAtS(s_now, maxv, acc, dec, s_acc, s_dec, s_flat);
        v = std::max(v, 0.05f);

        if (++iter > MAX_ITERS)
            break;

        // 计算当前位置
        Eigen::Vector2f tan = tangentAtS(s_now, path, s);

        Eigen::Vector2f pos = Eigen::Vector2f(path.back().x(), path.back().y());
        for (size_t i = 1; i < s.size(); ++i) {
            if (s_now <= s[i]) {
                float s0 = s[i - 1], s1 = s[i];
                Eigen::Vector2f p0(path[i - 1].x(), path[i - 1].y());
                Eigen::Vector2f p1(path[i].x(), path[i].y());
                float ratio = (s_now - s0) / std::max(s1 - s0, 1e-5f);
                pos = p0 + ratio * (p1 - p0);
                break;
            }
        }

        // 计算 yaw 并限制角速度
        float yaw = std::atan2(tan.y(), tan.x());
        float dyaw = yaw - prev_yaw;
        while (dyaw > M_PI)
            dyaw -= 2 * M_PI;
        while (dyaw < -M_PI)
            dyaw += 2 * M_PI;

        float max_dyaw = max_yaw_rate * dt;
        if (std::abs(dyaw) > max_dyaw) {
            yaw = prev_yaw + std::copysign(max_dyaw, dyaw);
        }
        prev_yaw = yaw;

        // 速度方向用受限后的 yaw 旋转
        Eigen::Vector2f v_vec(v * std::cos(yaw), v * std::sin(yaw));

        traj.push_back({ pos, v_vec, t });

        // 推进
        s_now += v * dt;
        t += dt;
    }

    return traj;
}

} // namespace rose_planner
