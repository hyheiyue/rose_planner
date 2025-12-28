#include "a*.hpp"
namespace rose_planner {
SearchState
AStar::search(const Eigen::Vector2f& start_w, const Eigen::Vector2f& goal_w, Path& path) {
    auto start = rose_map_->worldToKey2D(start_w);
    auto goal = rose_map_->worldToKey2D(goal_w);

    if (!PathSearch::checkStartGoalSafe(start, goal)) {
        return SearchState::NO_PATH;
    }

    nodes_.clear();
    const int esdf_size = static_cast<int>(rose_map_->esdf_.size());
    if (esdf_size <= 0)
        return SearchState::NO_PATH;

    // visited lookup table instead of unordered_map
    std::vector<int> index_map(esdf_size, -1);

    std::priority_queue<PQItem, std::vector<PQItem>, PQComp> open;

    int start_idx = rose_map_->key2DToIndex2D(start);
    int goal_idx = rose_map_->key2DToIndex2D(goal);
    if (start_idx < 0 || goal_idx < 0)
        return SearchState::NO_PATH;

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
    open.push({ 0, sn.f, h_start });

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
        if (cid < 0 || cid >= (int)nodes_.size())
            continue;
        if (top.f > nodes_[cid].f + 1e-4f)
            continue;

        // goal reached
        if (nodes_[cid].idx == goal_idx) {
            path.clear();
            for (int id = cid; id >= 0; id = nodes_[id].parent) {
                auto w = rose_map_->key2DToWorld(nodes_[id].key);
                path.push_back({ w.x(), w.y() });
            }
            std::reverse(path.begin(), path.end());
            return SearchState::SUCCESS;
        }

        getSuccessors(nodes_[cid].key, succ_buffer_);

        for (const auto& nbk: succ_buffer_) {
            int nb_idx = rose_map_->key2DToIndex2D(nbk);
            if (nb_idx < 0 || nb_idx >= esdf_size)
                continue;

            float base =
                (std::abs(nodes_[cid].key.x - nbk.x) + std::abs(nodes_[cid].key.y - nbk.y) == 1)
                ? 1.0f
                : 1.41421356f;
            float d = rose_map_->esdf_[nb_idx];
            float step_cost = base
                + params_.path_search_params_.a_star.obstacle_penalty_weight * (1.0f / (d + 0.1f));

            float ng = nodes_[cid].g + step_cost;
            float nh = heuristicCached(d, nbk, goal) * heu_weight;
            float nf = ng + nh;

            int exist_id = index_map[nb_idx];
            if (exist_id != -1) {
                if (ng >= nodes_[exist_id].g - 1e-4f)
                    continue;
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
                nn.idx = nb_idx;
                nn.esdf = d;
                int nid = (int)nodes_.size();
                nodes_.push_back(nn);
                index_map[nb_idx] = nid;
                open.push({ nid, nf, nh });
            }
        }
    }

    return SearchState::NO_PATH;
}

} // namespace rose_planner