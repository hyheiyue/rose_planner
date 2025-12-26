#pragma once
#include "parameters.hpp"
#include "rose_map/rose_map.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rose_planner {

class PathSearch {
public:
    using Path = std::vector<Eigen::Vector3f>;
    using Ptr = std::shared_ptr<PathSearch>;

    explicit PathSearch(rose_map::RoseMap::Ptr rose_map, Parameters params):
        rose_map_(rose_map),
        params_(params) {
        auto robot_size = params_.path_search_params_.robot_size;
        robot_radius_ =
            0.5f * std::sqrt(robot_size.x() * robot_size.x() + robot_size.y() * robot_size.y());
    }

    static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
        return std::make_shared<PathSearch>(rose_map, params);
    }

    bool checkStartGoalSafe(rose_map::VoxelKey2D& start, rose_map::VoxelKey2D& goal) {
        if (isOccupied(start)) {
            if (!findNearestSafe(start, start))
                return false;
        }
        if (isOccupied(goal)) {
            if (!findNearestSafe(goal, goal))
                return false;
        }

        return true;
    }
    bool findNearestSafe(const Eigen::Vector2f& raw, Eigen::Vector2f& safe) const {
        rose_map::VoxelKey2D a = rose_map_->worldToKey2D(raw);
        rose_map::VoxelKey2D b;
        if (findNearestSafe(a, b)) {
            auto w = rose_map_->key2DToWorld(b);
            safe = Eigen::Vector2f(w.x(), w.y());
            return true;
        }
        return false;
    }
    bool isOccupied(const rose_map::VoxelKey2D& k) const {
        int idx = rose_map_->key2DToIndex2D(k);
        if (idx < 0)
            return false;
        return rose_map_->esdf_[idx] < robot_radius_;
    }

    bool findNearestSafe(const rose_map::VoxelKey2D& seed, rose_map::VoxelKey2D& out) const {
        std::queue<rose_map::VoxelKey2D> q;
        std::unordered_set<int64_t> vis;
        auto hash = [&](const rose_map::VoxelKey2D& k) -> int64_t {
            return (int64_t(k.x) << 32) | uint32_t(k.y);
        };

        q.push(seed);
        vis.insert(hash(seed));
        int limit = 500;

        static const int dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
        static const int dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

        while (!q.empty() && limit--) {
            auto c = q.front();
            q.pop();
            if (!isOccupied(c)) {
                out = c;
                return true;
            }
            for (int i = 0; i < 8; i++) {
                rose_map::VoxelKey2D nb { c.x + dx[i], c.y + dy[i] };
                auto h = hash(nb);
                if (!vis.count(h)) {
                    vis.insert(h);
                    q.push(nb);
                }
            }
        }
        return false;
    }

    rose_map::RoseMap::Ptr rose_map_;
    Parameters params_;
    float robot_radius_;
};

} // namespace rose_planner
