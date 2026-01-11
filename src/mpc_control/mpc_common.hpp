#pragma once

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
    double x = 0;
    double y = 0;
    double vx = 0;
    double vy = 0;
    double ax = 0;
    double ay = 0;
};
} // namespace rose_planner