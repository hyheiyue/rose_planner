#pragma once
#include "../common.hpp"
#include "../parameters.hpp"
#include "control/type.hpp"

#include "rose_map/rose_map.hpp"

namespace rose_planner {

class TrajectoryOpt {
public:
    using Ptr = std::shared_ptr<TrajectoryOpt>;
    TrajectoryOpt(rose_map::RoseMap::Ptr rose_map, Parameters params);
    ~TrajectoryOpt();
    static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
        return std::make_shared<TrajectoryOpt>(rose_map, params);
    }

    void setPath(const std::vector<Eigen::Vector2d>& path, RoboState now);
    TrajType getTrajectory();
    void optimize();
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace rose_planner
