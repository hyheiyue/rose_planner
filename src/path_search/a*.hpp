#pragma once
#include "path_search.hpp"
#include "rose_map/rose_map.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <vector>

namespace rose_planner {

class AStar: public PathSearch {
public:
    using Ptr = std::shared_ptr<AStar>;

    explicit AStar(rose_map::RoseMap::Ptr rose_map, Parameters params):
        PathSearch(rose_map, params) {
        nodes_.reserve(4096);
        succ_buffer_.reserve(8);
    }

    static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
        return std::make_shared<AStar>(rose_map, params);
    }

    struct Node {
        rose_map::VoxelKey2D key;
        float g = 0.0f;
        float f = 0.0f;
        int parent = -1;
        float esdf = 0.0f; // cached clearance value
    };

    struct PQItem {
        int id;
        float f;
        float h;
    };

    struct PQComp {
        bool operator()(const PQItem& a, const PQItem& b) const {
            constexpr float EPS = 1e-4f;
            if (std::abs(a.f - b.f) < EPS) {
                return a.h > b.h;
            }
            return a.f > b.f;
        }
    };

    SearchState search(const Eigen::Vector2f& start_w, const Eigen::Vector2f& goal_w, Path& path);

private:
    std::vector<Node> nodes_;
    mutable std::vector<rose_map::VoxelKey2D> succ_buffer_;

    inline float
    heuristicCached(float esdf_val, const rose_map::VoxelKey2D& a, const rose_map::VoxelKey2D& b)
        const {
        float dx = float(a.x - b.x);
        float dy = float(a.y - b.y);
        float dist = std::sqrt(dx * dx + dy * dy);
        float clearance =
            params_.path_search_params_.a_star.clearance_weight * (1.0f / (esdf_val + 0.1f));
        return dist + clearance;
    }

    inline void
    getSuccessors(const rose_map::VoxelKey2D& cur, std::vector<rose_map::VoxelKey2D>& succ) const {
        succ.clear();
        static const int dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
        static const int dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
        for (int i = 0; i < 8; i++) {
            rose_map::VoxelKey2D nb { cur.x + dx[i], cur.y + dy[i] };
            if (!isOccupied(nb))
                succ.push_back(nb);
        }
    }
};

} // namespace rose_planner
