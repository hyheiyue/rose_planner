#pragma once
#include "path_search.hpp"
#include "rose_map/rose_map.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <queue>
#include <vector>

namespace rose_planner {

class AStar : public PathSearch {
public:
    using Ptr = std::shared_ptr<AStar>;

    explicit AStar(rose_map::RoseMap::Ptr rose_map, Parameters params)
        : PathSearch(rose_map, params) {
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
        int idx = -1;        // ESDF index
        float esdf = 0.0f;   // cached clearance value
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

    SearchState search(const Eigen::Vector2f& start_w, const Eigen::Vector2f& goal_w, Path& path) {
        auto start = rose_map_->worldToKey2D(start_w);
        auto goal  = rose_map_->worldToKey2D(goal_w);

        if (!PathSearch::checkStartGoalSafe(start, goal)) {
            return SearchState::NO_PATH;
        }

        nodes_.clear();
        const int esdf_size = static_cast<int>(rose_map_->esdf_.size());
        if (esdf_size <= 0) return SearchState::NO_PATH;

        // visited lookup table instead of unordered_map
        std::vector<int> index_map(esdf_size, -1);

        std::priority_queue<PQItem, std::vector<PQItem>, PQComp> open;

        int start_idx = rose_map_->key2DToIndex2D(start);
        int goal_idx  = rose_map_->key2DToIndex2D(goal);
        if (start_idx < 0 || goal_idx < 0) return SearchState::NO_PATH;

        Node sn;
        sn.key = start;
        sn.g = 0.0f;
        sn.idx = start_idx;
        sn.esdf = rose_map_->esdf_[start_idx];
        float h_start = heuristicCached(sn.esdf, start, goal);
        sn.f = sn.g + h_start;
        sn.parent = -1;

        nodes_.push_back(sn);
        index_map[start_idx] = 0;
        open.push({0, sn.f, h_start});

        const float heu_weight = params_.path_search_params_.a_star.heuristic_weight;
        const int max_iters = 10000000;
        int iters = 0;
        auto t0 = std::chrono::steady_clock::now();
        const float max_time = 1.0f;
        const int time_check_interval = 256;

        while (!open.empty()) {
            iters++;
            if ((iters & (time_check_interval - 1)) == 0) {
                float dt = std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
                if (dt > max_time || iters > max_iters) {
                    return SearchState::TIMEOUT;
                }
            }

            PQItem top = open.top();
            open.pop();
            int cid = top.id;
            if (cid < 0 || cid >= (int)nodes_.size()) continue;
            if (top.f > nodes_[cid].f + 1e-4f) continue;

            // goal reached
            if (nodes_[cid].idx == goal_idx) {
                path.clear();
                for (int id = cid; id >= 0; id = nodes_[id].parent) {
                    auto w = rose_map_->key2DToWorld(nodes_[id].key);
                    path.push_back({w.x(), w.y(), 0});
                }
                std::reverse(path.begin(), path.end());
                return SearchState::SUCCESS;
            }

            getSuccessors(nodes_[cid].key, succ_buffer_);

            for (const auto& nbk : succ_buffer_) {
                int nb_idx = rose_map_->key2DToIndex2D(nbk);
                if (nb_idx < 0 || nb_idx >= esdf_size) continue;

                float base = (std::abs(nodes_[cid].key.x - nbk.x) +
                              std::abs(nodes_[cid].key.y - nbk.y) == 1) ? 1.0f : 1.41421356f;
                float d = rose_map_->esdf_[nb_idx];
                float step_cost = base + params_.path_search_params_.a_star.obstacle_penalty_weight * (1.0f / (d + 0.1f));

                float ng = nodes_[cid].g + step_cost;
                float nh = heuristicCached(d, nbk, goal) * heu_weight;
                float nf = ng + nh;

                int exist_id = index_map[nb_idx];
                if (exist_id != -1) {
                    if (ng >= nodes_[exist_id].g - 1e-4f) continue;
                    nodes_[exist_id].g = ng;
                    nodes_[exist_id].f = nf;
                    nodes_[exist_id].parent = cid;
                    open.push({exist_id, nf, nh});
                } else {
                    Node nn;
                    nn.key = nbk;
                    nn.g = ng;
                    nn.f = nf;
                    nn.parent = cid;
                    nn.idx = nb_idx;
                    nn.esdf = d;
                    int nid = (int)nodes_.size();
                    nodes_.push_back(nn);
                    index_map[nb_idx] = nid;
                    open.push({nid, nf, nh});
                }
            }
        }

        return SearchState::NO_PATH;
    }

private:
    std::vector<Node> nodes_;
    mutable std::vector<rose_map::VoxelKey2D> succ_buffer_;

    float heuristicCached(float esdf_val, const rose_map::VoxelKey2D& a, const rose_map::VoxelKey2D& b) const {
        float dx = float(a.x - b.x);
        float dy = float(a.y - b.y);
        float dist = std::sqrt(dx * dx + dy * dy);
        float clearance = params_.path_search_params_.a_star.clearance_weight * (1.0f / (esdf_val + 0.1f));
        return dist + clearance;
    }


    void getSuccessors(const rose_map::VoxelKey2D& cur, std::vector<rose_map::VoxelKey2D>& succ) const {
        succ.clear();
        static const int dx[8] = {1,-1,0,0,1,1,-1,-1};
        static const int dy[8] = {0,0,1,-1,1,-1,1,-1};
        for (int i=0;i<8;i++) {
            rose_map::VoxelKey2D nb{cur.x + dx[i], cur.y + dy[i]};
            if (!isOccupied(nb)) succ.push_back(nb);
        }
    }
};

} // namespace rose_planner
