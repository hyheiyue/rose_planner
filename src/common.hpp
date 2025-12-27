#pragma once
#include <Eigen/Dense>

namespace rose_planner {
struct SampleTrajectoryPoint {
    Eigen::Vector2f p;
    Eigen::Vector2f v;
    float t;
};

} // namespace rose_planner