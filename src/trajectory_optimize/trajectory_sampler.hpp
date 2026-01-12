#pragma once
#include "../common.hpp"
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <vector>

namespace rose_planner {

inline double segmentLength(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return (b - a).norm();
}

inline std::vector<double> computeArcLengths(const std::vector<Eigen::Vector2d>& path) {
    if (path.empty())
        return {};
    std::vector<double> s(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i) {
        s[i] = s[i - 1] + (path[i] - path[i - 1]).norm();
    }
    return s;
}

inline Eigen::Vector2d interpolatePath(
    const std::vector<Eigen::Vector2d>& path,
    const std::vector<double>& s,
    double s_q
) {
    if (s_q <= s.front())
        return path.front();
    if (s_q >= s.back())
        return path.back();

    auto it = std::lower_bound(s.begin(), s.end(), s_q);
    int i = std::distance(s.begin(), it);

    // 保护 i 范围
    if (i <= 0)
        i = 1;
    if (i >= (int)s.size())
        i = s.size() - 1;

    double ds = s[i] - s[i - 1];
    if (ds < 1e-6)
        return path[i]; // 避免除0

    double r = (s_q - s[i - 1]) / ds;
    return path[i - 1] * (1.0 - r) + path[i] * r;
}

inline std::vector<Eigen::Vector2d>
resamplePath(const std::vector<Eigen::Vector2d>& path, double step = 0.05) {
    auto s = computeArcLengths(path);
    if (s.size() < 2)
        return path;

    std::vector<Eigen::Vector2d> resampled;
    double total_s = s.back();
    for (double cur_s = 0.0; cur_s < total_s; cur_s += step) {
        resampled.push_back(interpolatePath(path, s, cur_s));
    }
    resampled.push_back(path.back());
    return resampled;
}


inline std::vector<double> computeVelocityProfile(
    const std::vector<double>& s,
    double expected_vel,
    double max_acc,
    double start_v,
    double end_v
) {
    size_t n = s.size();
    std::vector<double> v(n, expected_vel);

    // 仅限制起点终点速度，不对中间做硬裁剪
    v[0] = std::min(expected_vel, start_v);
    v[n - 1] = std::min(expected_vel, end_v);

    // 速度线性积分传播，更温和
    for (size_t i = 1; i < n; i++) {
        double ds = s[i] - s[i - 1];
        double dv = max_acc * std::sqrt(ds); // 软加速因子（非物理硬限制）
        v[i] = std::min(expected_vel, v[i - 1] + dv);
    }

    for (int i = n - 2; i >= 0; i--) {
        double ds = s[i + 1] - s[i];
        double dv = max_acc * std::sqrt(ds);
        v[i] = std::min(expected_vel, v[i + 1] + dv);
        v[i] = std::max(v[i], 1.0);
    }


    return v;
}

inline std::vector<SampleTrajectoryPoint> sampleTrajectoryTrapezoid(
    const std::vector<Eigen::Vector2d>& path,
    double expected_vel,
    double max_acc,
    double dt,
    const Eigen::Vector2d& init_v = Eigen::Vector2d(0, 0)
) {
    if (path.size() < 2)
        return {};
    auto smooth = resamplePath(path, 0.05);
    auto s_map = computeArcLengths(smooth);

    Eigen::Vector2d start_dir = (smooth[1] - smooth[0]).normalized();
    double start_v = std::max(1.0, init_v.norm());
    double end_v = 0.0;

    auto v_profile = computeVelocityProfile(s_map, expected_vel, max_acc, start_v, end_v);

    std::vector<SampleTrajectoryPoint> traj;
    double t = 0.0, s_now = 0.0, s_total = s_map.back();

    while (s_now < s_total) {
        auto it = std::lower_bound(s_map.begin(), s_map.end(), s_now);
        int i = std::distance(s_map.begin(), it);

        if (i >= (int)v_profile.size())
            i = v_profile.size() - 1;
        if (i < 0)
            i = 0;

        double v_cmd = v_profile[i];

        if (!traj.empty()) {
            double v_prev = traj.back().v.norm();
            double dv = max_acc * dt;
            v_cmd = std::clamp(v_cmd, v_prev - dv, v_prev + dv);
        }

        Eigen::Vector2d p = interpolatePath(smooth, s_map, s_now);
        Eigen::Vector2d p2 = interpolatePath(smooth, s_map, s_now + 0.01);
        Eigen::Vector2d dir = (p2 - p).normalized();

        traj.push_back({ p, dir * v_cmd, t });
        s_now += v_cmd * dt;
        t += dt;

        if (v_cmd <= 0.0 && s_now < s_total)
            break;
    }

    traj.push_back({ smooth.back(), { 0, 0 }, t });
    return traj;
}

} // namespace rose_planner
