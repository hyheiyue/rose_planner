#pragma once
#include "common.hpp"
#include "parameters.hpp"

#include "rose_map/rose_map.hpp"

namespace rose_planner {
class PlannerManager {
public:
    enum FSMSTATE : int {
        INIT,
        WAIT_GOAL,
        SEARCH_PATH,
        REPLAN,
    };
    using Ptr = std::unique_ptr<PlannerManager>;
    PlannerManager(
        rclcpp::Node& node,
        const Parameters& params,
        Robo::Ptr robo,
        rose_map::RoseMap::Ptr rose_map
    );
    ~PlannerManager();
    static Ptr create(
        rclcpp::Node& node,
        const Parameters& params,
        Robo::Ptr robo,
        rose_map::RoseMap::Ptr rose_map
    ) {
        return std::make_unique<PlannerManager>(node, params, robo, rose_map);
    }
    std::optional<TrajType> getTrajectory();
    FSMSTATE getState() const;
    void setNewGoal(const Goal& g);
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace rose_planner