#pragma once
#include "common.hpp"
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
    std::vector<double> s(path.size(), 0.0f);
    for (size_t i = 1; i < path.size(); ++i) {
        s[i] = s[i - 1] + (path[i].head<2>() - path[i - 1].head<2>()).norm();
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
    double r = (s_q - s[i - 1]) / (s[i] - s[i - 1]);
    return path[i - 1] * (1.0 - r) + path[i] * r;
}

inline std::vector<Eigen::Vector2d>
resamplePath(const std::vector<Eigen::Vector2d>& path, double step = 0.05f) {
    auto s = computeArcLengths(path);
    if (s.size() < 2)
        return path;

    std::vector<Eigen::Vector2d> resampled;
    double total_s = s.back();
    for (double cur_s = 0.0f; cur_s < total_s; cur_s += step) {
        resampled.push_back(interpolatePath(path, s, cur_s));
    }
    resampled.push_back(path.back());
    return resampled;
}

// 使用 5 点局部最小二乘拟合 y = ax^2 + bx + c
inline std::vector<double> computeOptimizedCurvatures(const std::vector<Eigen::Vector2d>& path) {
    size_t n = path.size();
    std::vector<double> kappa(n, 0.0f);
    if (n < 5)
        return kappa;

    for (int i = 0; i < (int)n; ++i) {
        int start = std::max(0, i - 2);
        int end = std::min((int)n - 1, i + 2);
        int pts_count = end - start + 1;

        Eigen::Vector2d p_ref = path[i].head<2>();
        Eigen::MatrixXd A(pts_count, 3);
        Eigen::VectorXd B_val(pts_count);

        for (int j = start; j <= end; ++j) {
            double dx = path[j].x() - p_ref.x();
            double dy = path[j].y() - p_ref.y();
            int row = j - start;
            A(row, 0) = dx * dx;
            A(row, 1) = dx;
            A(row, 2) = 1.0f;
            B_val(row) = dy;
        }

        // 求解最小二乘拟合系数
        Eigen::Vector3d coeffs = A.colPivHouseholderQr().solve(B_val);
        double a = coeffs[0];
        double b = coeffs[1];

        // 曲率公式 k = |2a| / (1 + b^2)^1.5
        double denom = std::pow(1.0 + b * b, 1.5);
        kappa[i] = std::abs(2.0 * a) / std::max(denom, 1e-6);
    }
    return kappa;
}

inline std::vector<double> computeVelocityProfile(
    const std::vector<double>& s,
    const std::vector<double>& kappa,
    double max_vel,
    double max_acc,
    double start_v,
    double end_v
) {
    size_t n = s.size();
    std::vector<double> v_limit(n);

    // 1. 静态几何限速 (合加速度约束基础)
    for (size_t i = 0; i < n; ++i) {
        double v_k = (kappa[i] > 1e-5f) ? std::sqrt(max_acc / kappa[i]) : max_vel;
        v_limit[i] = std::min(max_vel, v_k);
    }
    v_limit[0] = std::min(v_limit[0], start_v);
    v_limit[n - 1] = std::min(v_limit[n - 1], end_v);

    auto get_a_long = [&](double v, double k) {
        double a_lat = v * v * k;
        return (a_lat >= max_acc) ? 0.0f : std::sqrt(max_acc * max_acc - a_lat * a_lat);
    };

    // 2. 双向扫描处理加减速
    for (size_t i = 1; i < n; ++i) {
        double ds = s[i] - s[i - 1];
        double a = get_a_long(v_limit[i - 1], kappa[i - 1]);
        v_limit[i] =
            std::min(v_limit[i], std::sqrt(v_limit[i - 1] * v_limit[i - 1] + 2.0f * a * ds));
    }
    for (int i = (int)n - 2; i >= 0; --i) {
        double ds = s[i + 1] - s[i];
        double a = get_a_long(v_limit[i + 1], kappa[i + 1]);
        v_limit[i] =
            std::min(v_limit[i], std::sqrt(v_limit[i + 1] * v_limit[i + 1] + 2.0f * a * ds));
    }
    return v_limit;
}

inline std::vector<SampleTrajectoryPoint> sampleTrajectoryTrapezoid(
    const std::vector<Eigen::Vector2d>& path,
    double max_vel,
    double max_acc,
    double dt,
    const Eigen::Vector2d& init_v = Eigen::Vector2d(0, 0)
) {
    if (path.size() < 2)
        return {};

    std::vector<Eigen::Vector2d> bspline_pts;
    const double B[4][4] = { { -1, 3, -3, 1 }, { 3, -6, 3, 0 }, { -3, 0, 3, 0 }, { 1, 4, 1, 0 } };
    if (path.size() >= 4) {
        for (size_t i = 0; i + 3 < path.size(); ++i) {
            for (int j = 0; j < 10; ++j) {
                double u = j / 10.0f;
                Eigen::Vector4d U(u * u * u, u * u, u, 1.0f);
                for (int k = 0; k < 2; ++k) {
                    Eigen::Vector4d G(path[i][k], path[i + 1][k], path[i + 2][k], path[i + 3][k]);
                    double val =
                        (U.transpose()
                         * (Eigen::Matrix4d() << -1, 3, -3, 1, 3, -6, 3, 0, -3, 0, 3, 0, 1, 4, 1, 0)
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

    std::vector<Eigen::Vector2d> smooth_path;
    for (auto& p: bspline_pts)
        smooth_path.push_back({ p.x(), p.y() });

    auto resampled_path = resamplePath(smooth_path, 0.05);
    auto s_map = computeArcLengths(resampled_path);
    auto kappa_map = computeOptimizedCurvatures(resampled_path);

    Eigen::Vector2d start_dir =
        (resampled_path[1].head<2>() - resampled_path[0].head<2>()).normalized();
    double start_v = std::max(0.0, init_v.dot(start_dir));
    auto v_profile = computeVelocityProfile(s_map, kappa_map, max_vel, max_acc, start_v, 0.0f);

    std::vector<SampleTrajectoryPoint> traj;
    double t = 0.0f, s_now = 0.0f, s_total = s_map.back();

    while (s_now < s_total) {
        auto it = std::lower_bound(s_map.begin(), s_map.end(), s_now);
        int i = std::distance(s_map.begin(), it);
        double v = (i == 0) ? v_profile[0] : v_profile[i - 1];

        double v_cmd = (v < 0.02 && s_total - s_now > 0.05) ? 0.02 : v;

        Eigen::Vector2d p_2d = interpolatePath(resampled_path, s_map, s_now);
        Eigen::Vector2d p_next = interpolatePath(resampled_path, s_map, s_now + 0.01);
        Eigen::Vector2d dir = (p_next - p_2d).normalized();

        traj.push_back({ p_2d, dir * v_cmd, t });

        s_now += v_cmd * dt;
        t += dt;
        if (v_cmd <= 0.0f && s_now < s_total)
            break;
    }

    traj.push_back({ resampled_path.back(), { 0, 0 }, t });
    return traj;
}

} // namespace rose_planner