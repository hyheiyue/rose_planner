#pragma once
#include <Eigen/Dense>

namespace rose_planner {
struct SampleTrajectoryPoint {
    Eigen::Vector2d p;
    Eigen::Vector2d v;
    double t;
};

} // namespace rose_planner