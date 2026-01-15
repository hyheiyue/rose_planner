#pragma once

#include <Eigen/Dense>
namespace rose_planner {
class MPCState {
public:
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    double yaw;
};

class TrajPoint {
public:
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    Eigen::Vector2d acc;
};
} // namespace rose_planner