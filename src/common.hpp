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
} // namespace rose_planner