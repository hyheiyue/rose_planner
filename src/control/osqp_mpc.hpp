#pragma once
#include "../parameters.hpp"
#include "../trajectory_optimize/trajectory.hpp"
#include "control/type.hpp"
#include "rose_map/rose_map.hpp"
namespace rose_planner {
namespace control {

    class OsqpMpc {
    public:
        using Ptr = std::shared_ptr<OsqpMpc>;

        OsqpMpc(rose_map::RoseMap::Ptr rose_map, const Parameters& params);
        ~OsqpMpc();

        static Ptr create(rose_map::RoseMap::Ptr rose_map, const Parameters& params) {
            return std::make_shared<OsqpMpc>(rose_map, params);
        }

        void setTrajectory(const TrajType& traj);

        void setCurrent(const RoboState& c);
        Output getOutput();

        bool solve();

        struct Impl;
        std::unique_ptr<Impl> _impl;
    };
} // namespace control
} // namespace rose_planner