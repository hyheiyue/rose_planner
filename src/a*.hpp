#pragma once
#include "path_search.hpp"
#include "rose_map/rose_map.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

namespace rose_planner {

class AStar: public PathSearch {
public:
    using Ptr = std::shared_ptr<AStar>;

    explicit AStar(rose_map::RoseMap::Ptr rose_map, Parameters params):
        PathSearch(rose_map, params) {
        // 构造时预留空间
        nodes_.reserve(4096);
    }

    static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
        return std::make_shared<AStar>(rose_map, params);
    }

    struct Node {
        rose_map::VoxelKey2D key;
        float g = 0.0f;
        float f = 0.0f;
        int parent = -1;
    };

    struct PQItem {
        int id;
        float f;
        float h;
    };

    struct PQComp {
        bool operator()(const PQItem& a, const PQItem& b) const {
            constexpr float EPSILON = 1e-4f;
            // Tie-breaking: F相等时，优先扩展H小的节点（更靠近目标的）
            if (std::abs(a.f - b.f) < EPSILON) {
                return a.h > b.h;
            }
            return a.f > b.f;
        }
    };

    SearchState search(const Eigen::Vector2f& start_w, const Eigen::Vector2f& goal_w, Path& path) {
        auto start = rose_map_->worldToKey2D(start_w);
        auto goal = rose_map_->worldToKey2D(goal_w);

        if (!PathSearch::checkStartGoalSafe(start, goal)) {
            return SearchState::NO_PATH;
        }

        nodes_.clear();
        if (nodes_.capacity() < 4096)
            nodes_.reserve(4096);

        std::priority_queue<PQItem, std::vector<PQItem>, PQComp> open;
        std::unordered_map<int64_t, int> visited_index_map;

        auto hash = [&](const rose_map::VoxelKey2D& k) -> int64_t {
            return (int64_t(k.x) << 32) | uint32_t(k.y);
        };
        Node sn;
        sn.key = start;
        sn.g = 0.0f;
        float h_start = clearanceHeuristic(start, goal);
        sn.f = sn.g + h_start;
        sn.parent = -1;

        nodes_.push_back(sn);
        open.push({ 0, sn.f, h_start });
        visited_index_map[hash(start)] = 0;

        const float heu_weight = params_.path_search_params_.a_star.heuristic_weight;
        const int max_iters = 10000000;
        int iters = 0;
        auto t_start = std::chrono::steady_clock::now();
        const float max_search_time = 1.0f; // 1秒
        while (!open.empty()) {
            float elapsed =
                std::chrono::duration<float>(std::chrono::steady_clock::now() - t_start).count();
            if (elapsed > max_search_time) {
                std::cout << "A* terminated: reached max search time 1.0s!" << std::endl;
                return SearchState::TIMEOUT;
            }
            iters++;
            if (iters > max_iters) {
                std::cout << "A* terminated: reached max search iterations!" << std::endl;
                return SearchState::TIMEOUT;
            }
            PQItem current_item = open.top();
            open.pop();

            int cid = current_item.id;

            if (current_item.f > nodes_[cid].f + 1e-4f) {
                continue;
            }
            const auto ck = nodes_[cid].key;

            if (ck.x == goal.x && ck.y == goal.y) {
                path.clear();
                for (int id = cid; id >= 0; id = nodes_[id].parent) {
                    auto w = rose_map_->key2DToWorld(nodes_[id].key);
                    path.push_back({ w.x(), w.y(), 0 });
                }
                std::reverse(path.begin(), path.end());
                return SearchState::SUCCESS;
            }

            std::vector<rose_map::VoxelKey2D> succ;
            getSuccessors(ck, succ);

            for (const auto& nbk: succ) {
                int64_t h_key = hash(nbk);

                float step_cost = stepCostWithClearance(ck, nbk);
                float ng = nodes_[cid].g + step_cost;
                float nh = clearanceHeuristic(nbk, goal) * heu_weight;
                float nf = ng + nh;

                auto it = visited_index_map.find(h_key);
                if (it != visited_index_map.end()) {
                    int exist_id = it->second;

                    if (ng >= nodes_[exist_id].g - 1e-4f) {
                        continue;
                    }
                    nodes_[exist_id].g = ng;
                    nodes_[exist_id].f = nf;
                    nodes_[exist_id].parent = cid;

                    open.push({ exist_id, nf, nh });
                } else {
                    Node nn;
                    nn.key = nbk;
                    nn.g = ng;
                    nn.f = nf;
                    nn.parent = cid;

                    int nid = (int)nodes_.size();
                    nodes_.push_back(nn);
                    visited_index_map[h_key] = nid;

                    open.push({ nid, nf, nh });
                }
            }
        }
        return SearchState::NO_PATH;
    }

private:
    std::vector<Node> nodes_;

    float getEsdfValue(const rose_map::VoxelKey2D& k) const {
        int idx = rose_map_->key2DToIndex2D(k);
        if (idx < 0 || idx >= (int)rose_map_->esdf_.size()) {
            return 0.0f;
        }
        return rose_map_->esdf_[idx];
    }

    float clearanceHeuristic(const rose_map::VoxelKey2D& a, const rose_map::VoxelKey2D& b) const {
        float dx = float(a.x - b.x);
        float dy = float(a.y - b.y);
        float dist = std::sqrt(dx * dx + dy * dy);

        float d = getEsdfValue(a);
        float clearance_term =
            params_.path_search_params_.a_star.clearance_weight * (1.0f / (d + 0.1f));
        return dist + clearance_term;
    }

    float
    stepCostWithClearance(const rose_map::VoxelKey2D& a, const rose_map::VoxelKey2D& b) const {
        float base = (std::abs(a.x - b.x) + std::abs(a.y - b.y) == 1) ? 1.0f : 1.41421356f;

        float d = getEsdfValue(b);
        float penalty =
            params_.path_search_params_.a_star.obstacle_penalty_weight * (1.0f / (d + 0.1f));
        return base + penalty;
    }

    void
    getSuccessors(const rose_map::VoxelKey2D& cur, std::vector<rose_map::VoxelKey2D>& succ) const {
        succ.clear();
        succ.reserve(8);
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