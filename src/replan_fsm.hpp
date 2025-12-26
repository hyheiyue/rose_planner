#pragma once

namespace rose_planner {
class ReplanFSM {
public:
    ReplanFSM()

    {}
    enum STATE : int {
        INIT,
        WAIT_GOAL,
        SEARCH_PATH,
        REPLAN,
    } state_;
};
} // namespace rose_planner