#pragma once
#include "path_search.hpp"
#include "rose_map/rose_map.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

namespace rose_planner {

class Jps: public PathSearch {
public:
    using Ptr = std::shared_ptr<Jps>;

    explicit Jps(rose_map::RoseMap::Ptr rose_map, Parameters params):
        PathSearch(rose_map, params) {}

    static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
        return std::make_shared<Jps>(rose_map, params);
    }

    bool search(const Eigen::Vector2f& start_w, const Eigen::Vector2f& goal_w, Path& path) {
        path.clear();

        auto start = rose_map_->worldToKey2D(start_w);
        auto goal = rose_map_->worldToKey2D(goal_w);
        if (!PathSearch::checkStartGoalSafe(start, goal)) {
            return false;
        }

        nodes_.clear();
        nodes_.reserve(4096);

        struct PQItem {
            int id;
            float f;
            float g;
        };
        struct PQComp {
            bool operator()(const PQItem& a, const PQItem& b) const {
                return a.f > b.f;
            }
        };

        std::priority_queue<PQItem, std::vector<PQItem>, PQComp> open;
        std::unordered_map<int64_t, int> visited; // hash -> best node id

        auto hash = [&](const rose_map::VoxelKey2D& k) -> int64_t {
            return (int64_t(k.x) << 32) | uint32_t(k.y);
        };

        Node sn;
        sn.key = start;
        sn.g = 0.0f;
        sn.f = heuristic(start, goal);
        sn.parent = -1;
        nodes_.push_back(sn);
        open.push({ 0, sn.f, sn.g });
        visited[hash(start)] = 0;

        while (!open.empty()) {
            auto cur_item = open.top();
            open.pop();
            int cid = cur_item.id;
            float g_in_queue = cur_item.g;

            if (cid < 0 || cid >= (int)nodes_.size())
                continue;
            if (g_in_queue > nodes_[cid].g + 1e-5f)
                continue; // 过期节点跳过

            const auto& cur = nodes_[cid].key;
            if (cur.x == goal.x && cur.y == goal.y) {
                int id = cid;
                while (id >= 0) {
                    auto w = rose_map_->key2DToWorld(nodes_[id].key);
                    path.push_back({ w.x(), w.y(), 0 });
                    id = nodes_[id].parent;
                }
                std::reverse(path.begin(), path.end());
                return true;
            }

            std::vector<std::pair<int, int>> dirs;
            getPrunedDirs(
                cur,
                (nodes_[cid].parent >= 0) ? nodes_[nodes_[cid].parent].key
                                          : rose_map::VoxelKey2D { INT32_MAX, INT32_MAX },
                dirs
            );

            for (auto& d: dirs) {
                rose_map::VoxelKey2D jp;
                if (!jump(cur, d.first, d.second, goal, jp))
                    continue;
                int64_t h = hash(jp);

                float dx = float(jp.x - cur.x);
                float dy = float(jp.y - cur.y);
                float dist = std::sqrt(dx * dx + dy * dy);
                float new_g = nodes_[cid].g + dist;

                if (visited.count(h)) {
                    int vid = visited[h];
                    if (new_g < nodes_[vid].g) {
                        nodes_[vid].g = new_g;
                        nodes_[vid].f = new_g + heuristic(jp, goal);
                        nodes_[vid].parent = cid;
                        open.push({ vid, nodes_[vid].f, new_g });
                    }
                    continue;
                }

                Node nn;
                nn.key = jp;
                nn.g = new_g;
                nn.f = new_g + heuristic(jp, goal);
                nn.parent = cid;
                int nid = nodes_.size();
                nodes_.push_back(nn);
                visited[h] = nid;
                open.push({ nid, nn.f, new_g });
            }
        }

        return false;
    }

private:
    struct Node {
        rose_map::VoxelKey2D key;
        float g = 0.0f;
        float f = 0.0f;
        int parent = -1;
    };

    bool jump(
        const rose_map::VoxelKey2D& prev,
        int dx,
        int dy,
        const rose_map::VoxelKey2D& goal,
        rose_map::VoxelKey2D& out
    ) const {
        const int MAX_STEPS = 512; // 恢复为合理范围
        int steps = 0;
        rose_map::VoxelKey2D n = prev;

        while (++steps < MAX_STEPS) {
            n.x += dx;
            n.y += dy;

            if (!isWalkable(n))
                return false;
            if (n.x == goal.x && n.y == goal.y) {
                out = n;
                return true;
            }

            if (isForced(n, dx, dy, prev)) { // 传入prev，而不是cur
                out = n;
                return true;
            }

            if (dx && dy) {
                rose_map::VoxelKey2D tmp;
                if (jump(n, dx, 0, goal, tmp) || jump(n, 0, dy, goal, tmp)) {
                    out = n;
                    return true;
                }
            }
        }
        return false;
    }

    bool isForced(const rose_map::VoxelKey2D& n, int dx, int dy, const rose_map::VoxelKey2D& prev)
        const {
        if (dx && dy) {
            if (!isWalkable({ n.x - dx, n.y }) && isWalkable({ n.x - dx, n.y + dy }))
                return true;
            if (!isWalkable({ n.x, n.y - dy }) && isWalkable({ n.x + dx, n.y - dy }))
                return true;
        } else if (dx) {
            if (!isWalkable({ n.x, n.y + 1 }) && isWalkable({ n.x + dx, n.y + 1 }))
                return true;
            if (!isWalkable({ n.x, n.y - 1 }) && isWalkable({ n.x + dx, n.y - 1 }))
                return true;
        } else if (dy) {
            if (!isWalkable({ n.x + 1, n.y }) && isWalkable({ n.x + 1, n.y + dy }))
                return true;
            if (!isWalkable({ n.x - 1, n.y }) && isWalkable({ n.x - 1, n.y + dy }))
                return true;
        }
        return false;
    }

    bool isWalkable(const rose_map::VoxelKey2D& k) const {
        return !isOccupied(k);
    }
    bool isOccupied(const rose_map::VoxelKey2D& k) const {
        int idx = rose_map_->key2DToIndex2D(k);
        if (idx < 0)
            return false;
        return rose_map_->esdf_[idx] <= 0;
    }
    float heuristic(const rose_map::VoxelKey2D& a, const rose_map::VoxelKey2D& b) const {
        float dx = float(a.x - b.x);
        float dy = float(a.y - b.y);
        float dist = std::sqrt(dx * dx + dy * dy);
        return dist;
    }

    void getPrunedDirs(
        const rose_map::VoxelKey2D& cur,
        const rose_map::VoxelKey2D& parent,
        std::vector<std::pair<int, int>>& dirs
    ) const {
        dirs.clear();
        static const int DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
        static const int DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

        if (parent.x == INT32_MAX) {
            for (int i = 0; i < 8; i++) {
                if (isWalkable({ cur.x + DX[i], cur.y + DY[i] })) {
                    if (std::abs(DX[i]) + std::abs(DY[i]) == 2) {
                        if (!isWalkable({ cur.x + DX[i], cur.y })
                            || !isWalkable({ cur.x, cur.y + DY[i] }))
                            continue;
                    }
                    dirs.push_back({ DX[i], DY[i] });
                }
            }
            return;
        }

        int pdx = cur.x - parent.x;
        int pdy = cur.y - parent.y;
        for (int i = 0; i < 8; i++) {
            if (DX[i] * pdx < 0 || DY[i] * pdy < 0)
                continue;
            if (isWalkable({ cur.x + DX[i], cur.y + DY[i] })) {
                if (std::abs(DX[i]) + std::abs(DY[i]) == 2) {
                    if (!isWalkable({ cur.x + DX[i], cur.y })
                        || !isWalkable({ cur.x, cur.y + DY[i] }))
                        continue;
                }
                dirs.push_back({ DX[i], DY[i] });
            }
        }
    }

    std::vector<Node> nodes_;
};

} // namespace rose_planner
