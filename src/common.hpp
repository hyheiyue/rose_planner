#pragma once
#include "trajectory_optimize/trajectory.hpp"
#include <Eigen/Dense>

namespace rose_planner {
struct SampleTrajectoryPoint {
    Eigen::Vector2d p;
    Eigen::Vector2d v;
    double t;
};
using TrajType = Trajectory<5, 2>;
static inline bool smoothedL1(const double& x, const double& mu, double& f, double& df) {
    if (x < 0.0) {
        return false;
    } else if (x > mu) {
        f = x - 0.5 * mu;
        df = 1.0;
        return true;
    } else {
        const double xdmu = x / mu;
        const double sqrxdmu = xdmu * xdmu;
        const double mumxd2 = mu - 0.5 * x;
        f = mumxd2 * sqrxdmu * xdmu;
        df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
        return true;
    }
}
static inline double kahanSum(double& sum, double& c, const double& val) {
    double y = val - c;
    double t = sum + y;
    c = (t - sum) - y;
    sum = t;
    return sum;
}
} // namespace rose_planner