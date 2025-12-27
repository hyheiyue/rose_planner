#pragma once
#include "common.hpp"
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace rose_planner {

inline float segmentLength(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
    return (b - a).norm();
}

inline std::vector<float> computeArcLengths(const std::vector<Eigen::Vector3f>& path) {
    if (path.empty())
        return {};
    std::vector<float> s(path.size(), 0.0f);
    for (size_t i = 1; i < path.size(); ++i) {
        s[i] = s[i - 1] + (path[i].head<2>() - path[i - 1].head<2>()).norm();
    }
    return s;
}

inline Eigen::Vector3f
interpolatePath(const std::vector<Eigen::Vector3f>& path, const std::vector<float>& s, float s_q) {
    if (s_q <= s.front())
        return path.front();
    if (s_q >= s.back())
        return path.back();
    auto it = std::lower_bound(s.begin(), s.end(), s_q);
    int i = std::distance(s.begin(), it);
    float r = (s_q - s[i - 1]) / (s[i] - s[i - 1]);
    return path[i - 1] * (1.0f - r) + path[i] * r;
}

inline std::vector<Eigen::Vector3f>
resamplePath(const std::vector<Eigen::Vector3f>& path, float step = 0.05f) {
    auto s = computeArcLengths(path);
    if (s.size() < 2)
        return path;

    std::vector<Eigen::Vector3f> resampled;
    float total_s = s.back();
    for (float cur_s = 0.0f; cur_s < total_s; cur_s += step) {
        resampled.push_back(interpolatePath(path, s, cur_s));
    }
    resampled.push_back(path.back());
    return resampled;
}

// 使用 5 点局部最小二乘拟合 y = ax^2 + bx + c
inline std::vector<float> computeOptimizedCurvatures(const std::vector<Eigen::Vector3f>& path) {
    size_t n = path.size();
    std::vector<float> kappa(n, 0.0f);
    if (n < 5)
        return kappa;

    for (int i = 0; i < (int)n; ++i) {
        int start = std::max(0, i - 2);
        int end = std::min((int)n - 1, i + 2);
        int pts_count = end - start + 1;

        Eigen::Vector2f p_ref = path[i].head<2>();
        Eigen::MatrixXf A(pts_count, 3);
        Eigen::VectorXf B_val(pts_count);

        for (int j = start; j <= end; ++j) {
            float dx = path[j].x() - p_ref.x();
            float dy = path[j].y() - p_ref.y();
            int row = j - start;
            A(row, 0) = dx * dx;
            A(row, 1) = dx;
            A(row, 2) = 1.0f;
            B_val(row) = dy;
        }

        // 求解最小二乘拟合系数
        Eigen::Vector3f coeffs = A.colPivHouseholderQr().solve(B_val);
        float a = coeffs[0];
        float b = coeffs[1];

        // 曲率公式 k = |2a| / (1 + b^2)^1.5
        float denom = std::pow(1.0f + b * b, 1.5f);
        kappa[i] = std::abs(2.0f * a) / std::max(denom, 1e-6f);
    }
    return kappa;
}

inline std::vector<float> computeVelocityProfile(
    const std::vector<float>& s,
    const std::vector<float>& kappa,
    float max_vel,
    float max_acc,
    float start_v,
    float end_v
) {
    size_t n = s.size();
    std::vector<float> v_limit(n);

    // 1. 静态几何限速 (合加速度约束基础)
    for (size_t i = 0; i < n; ++i) {
        float v_k = (kappa[i] > 1e-5f) ? std::sqrt(max_acc / kappa[i]) : max_vel;
        v_limit[i] = std::min(max_vel, v_k);
    }
    v_limit[0] = std::min(v_limit[0], start_v);
    v_limit[n - 1] = std::min(v_limit[n - 1], end_v);

    auto get_a_long = [&](float v, float k) {
        float a_lat = v * v * k;
        return (a_lat >= max_acc) ? 0.0f : std::sqrt(max_acc * max_acc - a_lat * a_lat);
    };

    // 2. 双向扫描处理加减速
    for (size_t i = 1; i < n; ++i) {
        float ds = s[i] - s[i - 1];
        float a = get_a_long(v_limit[i - 1], kappa[i - 1]);
        v_limit[i] =
            std::min(v_limit[i], std::sqrt(v_limit[i - 1] * v_limit[i - 1] + 2.0f * a * ds));
    }
    for (int i = (int)n - 2; i >= 0; --i) {
        float ds = s[i + 1] - s[i];
        float a = get_a_long(v_limit[i + 1], kappa[i + 1]);
        v_limit[i] =
            std::min(v_limit[i], std::sqrt(v_limit[i + 1] * v_limit[i + 1] + 2.0f * a * ds));
    }
    return v_limit;
}

inline std::vector<SampleTrajectoryPoint> sampleTrajectoryTrapezoid(
    const std::vector<Eigen::Vector3f>& path,
    float max_vel,
    float max_acc,
    float dt,
    const Eigen::Vector2f& init_v = Eigen::Vector2f(0, 0)
) {
    if (path.size() < 2)
        return {};

    std::vector<Eigen::Vector2f> bspline_pts;
    const float B[4][4] = { { -1, 3, -3, 1 }, { 3, -6, 3, 0 }, { -3, 0, 3, 0 }, { 1, 4, 1, 0 } };
    if (path.size() >= 4) {
        for (size_t i = 0; i + 3 < path.size(); ++i) {
            for (int j = 0; j < 10; ++j) {
                float u = j / 10.0f;
                Eigen::Vector4f U(u * u * u, u * u, u, 1.0f);
                for (int k = 0; k < 2; ++k) {
                    Eigen::Vector4f G(path[i][k], path[i + 1][k], path[i + 2][k], path[i + 3][k]);
                    float val =
                        (U.transpose()
                         * (Eigen::Matrix4f() << -1, 3, -3, 1, 3, -6, 3, 0, -3, 0, 3, 0, 1, 4, 1, 0)
                               .finished()
                         * G / 6.0f)
                            .value();
                    if (k == 0)
                        bspline_pts.push_back({ val, 0 });
                    else
                        bspline_pts.back().y() = val;
                }
            }
        }
    } else {
        for (auto& p: path)
            bspline_pts.push_back(p.head<2>());
    }

    std::vector<Eigen::Vector3f> smooth_path;
    for (auto& p: bspline_pts)
        smooth_path.push_back({ p.x(), p.y(), path.back().z() });

    auto resampled_path = resamplePath(smooth_path, 0.05f);
    auto s_map = computeArcLengths(resampled_path);
    auto kappa_map = computeOptimizedCurvatures(resampled_path);

    Eigen::Vector2f start_dir =
        (resampled_path[1].head<2>() - resampled_path[0].head<2>()).normalized();
    float start_v = std::max(0.0f, init_v.dot(start_dir));
    auto v_profile = computeVelocityProfile(s_map, kappa_map, max_vel, max_acc, start_v, 0.0f);

    std::vector<SampleTrajectoryPoint> traj;
    float t = 0.0f, s_now = 0.0f, s_total = s_map.back();

    while (s_now < s_total) {
        auto it = std::lower_bound(s_map.begin(), s_map.end(), s_now);
        int i = std::distance(s_map.begin(), it);
        float v = (i == 0) ? v_profile[0] : v_profile[i - 1];

        float v_cmd = (v < 0.02f && s_total - s_now > 0.05f) ? 0.02f : v;

        Eigen::Vector3f p_3d = interpolatePath(resampled_path, s_map, s_now);
        Eigen::Vector3f p_next = interpolatePath(resampled_path, s_map, s_now + 0.01f);
        Eigen::Vector2f dir = (p_next.head<2>() - p_3d.head<2>()).normalized();

        traj.push_back({ p_3d.head<2>(), dir * v_cmd, t });

        s_now += v_cmd * dt;
        t += dt;
        if (v_cmd <= 0.0f && s_now < s_total)
            break;
    }

    traj.push_back({ resampled_path.back().head<2>(), { 0, 0 }, t });
    return traj;
}

} // namespace rose_planner