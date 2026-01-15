#pragma once
#include "../common.hpp"
#include "../parameters.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

namespace rose_planner {

class TrajectorySampler2D {
public:
    using Ptr = std::shared_ptr<TrajectorySampler2D>;
    explicit TrajectorySampler2D(const Parameters& params) {
        min_speed = params.resampler_params_.min_speed;
        expected_speed = params.resampler_params_.expected_speed;
        max_acc = params.resampler_params_.max_acc;
        dt = params.resampler_params_.dt;
    }
    static Ptr create(const Parameters& params) {
        return std::make_shared<TrajectorySampler2D>(params);
    }

    std::vector<SampleTrajectoryPoint> sampleTrapezoid(
        const std::vector<Eigen::Vector2d>& path,
        const Eigen::Vector2d& init_v = Eigen::Vector2d::Zero()
    ) const {
        if (path.size() < 2)
            return {};

        auto smooth = resamplePath(path);
        auto s_map = computeArcLengths(smooth);

        double start_v = init_v.norm();
        double end_v = 0.0;

        auto v_profile = computeVelocityProfile(s_map, expected_speed, max_acc, start_v, end_v);

        return sampleByTime(smooth, s_map, v_profile, max_acc, dt);
    }
    static double segmentLength(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return (b - a).norm();
    }
    static std::vector<double> computeArcLengths(const std::vector<Eigen::Vector2d>& path) {
        std::vector<double> s(path.size(), 0.0);
        for (size_t i = 1; i < path.size(); ++i)
            s[i] = s[i - 1] + (path[i] - path[i - 1]).norm();
        return s;
    }

private:
    static Eigen::Vector2d interpolatePath(
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

        i = std::clamp(i, 1, static_cast<int>(s.size()) - 1);

        double ds = s[i] - s[i - 1];
        if (ds < 1e-6)
            return path[i];

        double r = (s_q - s[i - 1]) / ds;
        return path[i - 1] * (1.0 - r) + path[i] * r;
    }

    std::vector<Eigen::Vector2d> resamplePath(const std::vector<Eigen::Vector2d>& path) const {
        auto s = computeArcLengths(path);
        if (s.size() < 2)
            return path;

        std::vector<Eigen::Vector2d> resampled;
        double total_s = s.back();

        for (double cur = 0.0; cur < total_s; cur += resample_step)
            resampled.push_back(interpolatePath(path, s, cur));

        resampled.push_back(path.back());
        return resampled;
    }

    std::vector<double> computeVelocityProfile(
        const std::vector<double>& s,
        double expected_vel,
        double max_acc,
        double start_v,
        double end_v
    ) const {
        size_t n = s.size();
        std::vector<double> v(n, expected_vel);

        v.front() = std::min(expected_vel, start_v);
        v.back() = std::min(expected_vel, end_v);

        // forward
        for (size_t i = 1; i < n; i++) {
            double ds = s[i] - s[i - 1];
            double dv = max_acc * std::sqrt(ds);
            v[i] = std::min(expected_vel, v[i - 1] + dv);
        }

        // backward
        for (int i = n - 2; i >= 0; i--) {
            double ds = s[i + 1] - s[i];
            double dv = max_acc * std::sqrt(ds);
            v[i] = std::min(v[i], v[i + 1] + dv);
            v[i] = std::max(v[i], min_speed);
        }

        return v;
    }

    std::vector<SampleTrajectoryPoint> sampleByTime(
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<double>& s_map,
        const std::vector<double>& v_profile,
        double max_acc,
        double dt
    ) const {
        std::vector<SampleTrajectoryPoint> traj;

        double t = 0.0;
        double s_now = 0.0;
        double s_total = s_map.back();

        while (s_now < s_total) {
            auto it = std::lower_bound(s_map.begin(), s_map.end(), s_now);
            int i = std::clamp(
                static_cast<int>(std::distance(s_map.begin(), it)),
                0,
                static_cast<int>(v_profile.size()) - 1
            );

            double v_cmd = v_profile[i];

            if (!traj.empty()) {
                double v_prev = traj.back().v.norm();
                double dv = max_acc * dt;
                v_cmd = std::clamp(v_cmd, v_prev - dv, v_prev + dv);
            }

            Eigen::Vector2d p = interpolatePath(path, s_map, s_now);
            Eigen::Vector2d p2 = interpolatePath(path, s_map, s_now + 0.01);
            Eigen::Vector2d dir = (p2 - p).normalized();

            traj.push_back({ p, dir * v_cmd, t });

            s_now += v_cmd * dt;
            t += dt;

            if (v_cmd <= 0.0 && s_now < s_total)
                break;
        }

        traj.push_back({ path.back(), Eigen::Vector2d::Zero(), t });
        return traj;
    }

private:
    double resample_step = 0.05; // 路径重采样间隔
    double min_speed = 1.0; // 最小速度保护
    double expected_speed = 3.0;
    double max_acc = 2.0;
    double dt = 0.1;
};

} // namespace rose_planner
