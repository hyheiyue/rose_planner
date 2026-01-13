#pragma once

#include <Eigen/Dense>
namespace rose_planner {
class MPCState {
public:
    double x = 0;
    double y = 0;
    double vx = 0;
    double vy = 0;
};

class TrajPoint {
public:
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    Eigen::Vector2d acc;
};
} // namespace rose_planner