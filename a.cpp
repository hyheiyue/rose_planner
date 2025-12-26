// #pragma once
// #include "parameters.hpp"
// #include "rose_map/rose_map.hpp"
// #include <algorithm>
// #include <cmath>
// #include <cstdint>
// #include <queue>
// #include <unordered_map>
// #include <unordered_set>
// #include <vector>

// namespace rose_planner {

// class PathSearch {
// public:
//     using Path = std::vector<Eigen::Vector3f>;
//     using Ptr = std::shared_ptr<PathSearch>;

//     explicit PathSearch(rose_map::RoseMap::Ptr rose_map, Parameters params):
//         rose_map_(rose_map),
//         params_(params) {
//         auto robot_size = params_.path_search_params_.robot_size;
//         robot_radius_ =
//             0.5f * std::sqrt(robot_size.x() * robot_size.x() + robot_size.y() * robot_size.y());
//     }

//     static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
//         return std::make_shared<PathSearch>(rose_map, params);
//     }

//     bool search(const Eigen::Vector2f& start_w, const Eigen::Vector2f& goal_w, Path& path) {
//         path.clear();

//         auto start = rose_map_->worldToKey2D(start_w);
//         auto goal = rose_map_->worldToKey2D(goal_w);
//         if (isOccupied(start)) {
//             if (!findNearestSafe(start, start))
//                 return false;
//         }
//         if (isOccupied(goal)) {
//             if (!findNearestSafe(goal, goal))
//                 return false;
//         }

//         nodes_.clear();
//         nodes_.reserve(4096);

//         std::priority_queue<PQItem, std::vector<PQItem>, PQComp> open;
//         std::unordered_map<int64_t, int> visited;

//         auto hash = [&](const rose_map::VoxelKey2D& k) -> int64_t {
//             return (int64_t(k.x) << 32) | uint32_t(k.y);
//         };

//         Node sn;
//         sn.key = start;
//         sn.g = 0.0f;
//         sn.f = clearanceHeuristic(start, goal);
//         sn.parent = -1;
//         nodes_.push_back(sn);
//         open.push({ 0, sn.f });
//         visited[hash(start)] = 0;

//         while (!open.empty()) {
//             int cid = open.top().id;
//             open.pop();

//             const auto& ck = nodes_[cid].key;

//             if (ck.x == goal.x && ck.y == goal.y) {
//                 for (int id = cid; id >= 0; id = nodes_[id].parent) {
//                     auto w = rose_map_->key2DToWorld(nodes_[id].key);
//                     path.push_back({ w.x(), w.y(), 0 });
//                 }
//                 std::reverse(path.begin(), path.end());
//                 return true;
//             }

//             std::vector<rose_map::VoxelKey2D> succ;
//             getSuccessors(ck, succ);

//             for (const auto& nbk: succ) {
//                 int64_t h = hash(nbk);
//                 if (visited.count(h))
//                     continue;

//                 float ng = nodes_[cid].g + stepCostWithClearance(ck, nbk);
//                 float nf = ng + clearanceHeuristic(nbk, goal);

//                 Node nn;
//                 nn.key = nbk;
//                 nn.g = ng;
//                 nn.f = nf;
//                 nn.parent = cid;

//                 int nid = (int)nodes_.size();
//                 nodes_.push_back(nn);
//                 visited[h] = nid;
//                 open.push({ nid, nf });
//             }
//         }

//         return false;
//     }
//     bool findNearestSafe(const Eigen::Vector2f& raw, Eigen::Vector2f& safe) const {
//         rose_map::VoxelKey2D a = rose_map_->worldToKey2D(raw);
//         rose_map::VoxelKey2D b;
//         if (findNearestSafe(a, b)) {
//             auto w = rose_map_->key2DToWorld(b);
//             safe = Eigen::Vector2f(w.x(), w.y());
//             return true;
//         }
//         return false;
//     }

// private:
//     struct Node {
//         rose_map::VoxelKey2D key;
//         float g = 0.0f;
//         float f = 0.0f;
//         int parent = -1;
//     };

//     struct PQItem {
//         int id;
//         float f;
//     };
//     struct PQComp {
//         bool operator()(const PQItem& a, const PQItem& b) const {
//             return a.f > b.f;
//         }
//     };

//     bool isOccupied(const rose_map::VoxelKey2D& k) const {
//         int idx = rose_map_->key2DToIndex2D(k);
//         if (idx < 0)
//             return true;
//         return rose_map_->esdf_[idx] < robot_radius_;
//     }

//     float clearanceHeuristic(const rose_map::VoxelKey2D& a, const rose_map::VoxelKey2D& b) const {
//         float dx = float(a.x - b.x);
//         float dy = float(a.y - b.y);
//         float dist = std::sqrt(dx * dx + dy * dy);

//         int idx = rose_map_->key2DToIndex2D(a);
//         float d = (idx < 0) ? 0.0f : rose_map_->esdf_[idx];

//         float clearance_term = params_.path_search_params_.clearance_weight * (1.0f / (d + 1.0f));
//         return dist + clearance_term;
//     }

//     float
//     stepCostWithClearance(const rose_map::VoxelKey2D& a, const rose_map::VoxelKey2D& b) const {
//         float base = (std::abs(a.x - b.x) + std::abs(a.y - b.y) == 1) ? 1.0f : std::sqrt(2.0f);

//         int idx = rose_map_->key2DToIndex2D(b);
//         float d = (idx < 0) ? 0.0f : rose_map_->esdf_[idx];
//         float penalty = params_.path_search_params_.obstacle_penalty_weight * (1.0f / (d + 0.3f));
//         return base + penalty;
//     }

//     void
//     getSuccessors(const rose_map::VoxelKey2D& cur, std::vector<rose_map::VoxelKey2D>& succ) const {
//         succ.clear();
//         static const int dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
//         static const int dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

//         for (int i = 0; i < 8; i++) {
//             rose_map::VoxelKey2D nb { cur.x + dx[i], cur.y + dy[i] };
//             if (!isOccupied(nb))
//                 succ.push_back(nb);
//         }
//     }

//     bool findNearestSafe(const rose_map::VoxelKey2D& seed, rose_map::VoxelKey2D& out) const {
//         std::queue<rose_map::VoxelKey2D> q;
//         std::unordered_set<int64_t> vis;
//         auto hash = [&](const rose_map::VoxelKey2D& k) -> int64_t {
//             return (int64_t(k.x) << 32) | uint32_t(k.y);
//         };

//         q.push(seed);
//         vis.insert(hash(seed));
//         int limit = 500;

//         static const int dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
//         static const int dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

//         while (!q.empty() && limit--) {
//             auto c = q.front();
//             q.pop();
//             if (!isOccupied(c)) {
//                 out = c;
//                 return true;
//             }
//             for (int i = 0; i < 8; i++) {
//                 rose_map::VoxelKey2D nb { c.x + dx[i], c.y + dy[i] };
//                 auto h = hash(nb);
//                 if (!vis.count(h)) {
//                     vis.insert(h);
//                     q.push(nb);
//                 }
//             }
//         }
//         return false;
//     }

// private:
//     rose_map::RoseMap::Ptr rose_map_;
//     std::vector<Node> nodes_;
//     Parameters params_;
//     float robot_radius_;
// };

// } // namespace rose_planner

// #pragma once
// #include "parameters.hpp"
// #include "rose_map/rose_map.hpp"

// #include <algorithm>
// #include <cmath>
// #include <cstdint>
// #include <iostream>
// #include <limits>
// #include <memory>
// #include <queue>
// #include <unordered_map>
// #include <unordered_set>
// #include <vector>

// namespace rose_planner {

// struct TrajPoint {
//     float t;
//     Eigen::Vector2f pos;
//     Eigen::Vector2f vel;
// };

// class PathSearch {
// public:
//     using Path = std::vector<TrajPoint>;
//     using Ptr = std::shared_ptr<PathSearch>;

//     explicit PathSearch(rose_map::RoseMap::Ptr rose_map, Parameters params):
//         rose_map_(rose_map),
//         params_(params) {
//         auto robot_size = params_.path_search_params_.robot_size;
//         robot_radius_ =
//             0.5f * std::sqrt(robot_size.x() * robot_size.x() + robot_size.y() * robot_size.y());
//     }

//     static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
//         return std::make_shared<PathSearch>(rose_map, params);
//     }

//     bool search(const Eigen::Vector2f& start_w, const Eigen::Vector2f& goal_w, Path& path) {
//         path.clear();

//         if (!rose_map_) {
//             std::cerr << "[PathSearch] Map pointer is null!\n";
//             return false;
//         }
//         if (rose_map_->esdf_.empty()) {
//             std::cerr << "[PathSearch] ESDF is empty!\n";
//             return false;
//         }

//         auto start = rose_map_->worldToKey2D(start_w);
//         auto goal = rose_map_->worldToKey2D(goal_w);

//         if (isOccupied(start) && !findNearestSafe(start, start)) {
//             std::cerr << "[PathSearch] No safe start found!\n";
//             return false;
//         }
//         if (isOccupied(goal) && !findNearestSafe(goal, goal)) {
//             std::cerr << "[PathSearch] No safe goal found!\n";
//             return false;
//         }

//         nodes_.clear();
//         nodes_.reserve(4096);

//         struct PQItem {
//             int id;
//             float f;
//         };
//         struct PQComp {
//             bool operator()(const PQItem& a, const PQItem& b) const {
//                 return a.f > b.f;
//             }
//         };
//         std::priority_queue<PQItem, std::vector<PQItem>, PQComp> open;
//         std::unordered_map<int64_t, int> visited;

//         auto validNum = [&](float x) { return std::isfinite(x); };

//         auto hash4d = [&](const rose_map::VoxelKey2D& k, const Eigen::Vector2f& v) -> int64_t {
//             if (!validNum(v.x()) || !validNum(v.y()))
//                 return 0;
//             int16_t vx = (int16_t)std::round(v.x() / 0.1f);
//             int16_t vy = (int16_t)std::round(v.y() / 0.1f);
//             int64_t hx = std::clamp((int64_t)k.x, (int64_t)-1000000LL, (int64_t)1000000LL);
//             int64_t hy = std::clamp((int64_t)k.y, (int64_t)-1000000LL, (int64_t)1000000LL);
//             return (hx << 32) ^ (hy << 16) ^ (uint32_t(vx) << 1) ^ (uint32_t(vy) << 8);
//         };

//         Node s;
//         s.key = start;
//         s.vel = Eigen::Vector2f::Zero();
//         s.g = 0.0f;
//         s.f = 2.5f * heuristicNode(s, goal);
//         s.parent = -1;
//         if (!validNum(s.f))
//             s.f = 1e6;
//         nodes_.push_back(s);

//         open.push({ 0, s.f });
//         visited[hash4d(start, s.vel)] = 0;

//         const float dt = params_.path_search_params_.dt;
//         const float vmax = params_.path_search_params_.max_vel;
//         const float amax = params_.path_search_params_.max_acc;
//         const int MAX_NODES = 20000;

//         while (!open.empty()) {
//             int cid = open.top().id;
//             open.pop();

//             if (cid < 0 || cid >= (int)nodes_.size())
//                 continue;
//             const Node& cur = nodes_[cid];

//             if (cur.key.x == goal.x && cur.key.y == goal.y) {
//                 reconstructTrajectory(cid, path, dt);
//                 return !path.empty();
//             }

//             if ((int)nodes_.size() > MAX_NODES) {
//                 std::cerr << "[PathSearch] Node limit exceeded!\n";
//                 return false;
//             }

//             std::vector<Eigen::Vector2f> acc_inputs = { { 1, 0 },
//                                                         { -1, 0 },
//                                                         { 0, 1 },
//                                                         { 0, -1 },
//                                                         { 0.707f, 0.707f },
//                                                         { -0.707f, 0.707f },
//                                                         { 0.707f, -0.707f },
//                                                         { -0.707f, -0.707f } };

//             Eigen::Vector2f wa = rose_map_->key2DToWorld(cur.key).head<2>();
//             Eigen::Vector2f wg = rose_map_->key2DToWorld(goal).head<2>();
//             if (!validNum(wa.x()) || !validNum(wa.y()))
//                 continue;
//             if (!validNum(wg.x()) || !validNum(wg.y()))
//                 continue;
//             Eigen::Vector2f dir = (wg - wa).normalized();

//             for (auto acc: acc_inputs) {
//                 if (!validNum(acc.x()) || !validNum(acc.y()))
//                     continue;
//                 if (acc.dot(dir) < 0.0f)
//                     continue;
//                 if (acc.norm() > 1.0f)
//                     acc.normalize();
//                 acc *= amax;
//                 if (acc.norm() > amax)
//                     acc = acc.normalized() * amax;

//                 Node next;
//                 motionModel(cur, acc, dt, next, vmax, amax);
//                 if (!validNum(next.vel.x()) || !validNum(next.vel.y()))
//                     continue;

//                 float g_new = cur.g + cost(cur, next, acc, dt);
//                 if (!validNum(g_new))
//                     continue;
//                 float h_val = heuristicNode(next, goal);
//                 if (!validNum(h_val))
//                     continue;
//                 if (h_val > cur.f)
//                     continue;
//                 if (isCollisionTraj(cur, next))
//                     continue;

//                 int64_t h = hash4d(next.key, next.vel);
//                 if (visited.count(h) && g_new >= nodes_[visited[h]].g)
//                     continue;

//                 int nid = nodes_.size();
//                 next.parent = cid;
//                 next.g = g_new;
//                 next.f = next.g + 2.0f * h_val;
//                 if (!validNum(next.f))
//                     next.f = 1e6;

//                 nodes_.push_back(next);
//                 visited[h] = nid;
//                 open.push({ nid, next.f });
//             }
//         }

//         return false;
//     }

//     bool findNearestSafe(const Eigen::Vector2f& raw, Eigen::Vector2f& safe) const {
//         if (!rose_map_)
//             return false;
//         auto a = rose_map_->worldToKey2D(raw);
//         rose_map::VoxelKey2D b;
//         if (findNearestSafe(a, b)) {
//             auto w = rose_map_->key2DToWorld(b);
//             if (!std::isfinite(w.x()) || !std::isfinite(w.y()))
//                 return false;
//             safe = { w.x(), w.y() };
//             return true;
//         }
//         return false;
//     }

// private:
//     struct Node {
//         rose_map::VoxelKey2D key;
//         Eigen::Vector2f vel;
//         float g = 0;
//         float f = 0;
//         int parent = -1;
//     };

//     bool isOccupied(const rose_map::VoxelKey2D& k) const {
//         if (!rose_map_)
//             return false;
//         int idx = rose_map_->key2DToIndex2D(k);
//         if (idx < 0 || idx >= (int)rose_map_->esdf_.size())
//             return false;
//         return rose_map_->esdf_[idx] < robot_radius_;
//     }

//     void motionModel(
//         const Node& cur,
//         const Eigen::Vector2f& acc,
//         float dt,
//         Node& next,
//         float vmax,
//         float amax
//     ) const {
//         if (!rose_map_)
//             return;
//         float vs = rose_map_->voxel_size_;
//         if (vs <= 1e-6f)
//             return;

//         Eigen::Vector2f v = cur.vel + acc * dt;
//         if (!std::isfinite(v.x()) || !std::isfinite(v.y()))
//             return;
//         if (v.norm() > vmax && v.norm() > 1e-6f)
//             v = v.normalized() * vmax;
//         if (acc.norm() > amax)
//             v = cur.vel + acc.normalized() * amax * dt;
//         if (!std::isfinite(v.x()) || !std::isfinite(v.y()))
//             return;
//         next.vel = v;

//         Eigen::Vector2f v_key = cur.vel / vs;
//         Eigen::Vector2f a_key = acc / vs;
//         if (!std::isfinite(v_key.x()) || !std::isfinite(v_key.y()))
//             return;
//         if (!std::isfinite(a_key.x()) || !std::isfinite(a_key.y()))
//             return;

//         Eigen::Vector2f p_key((float)cur.key.x, (float)cur.key.y);
//         Eigen::Vector2f p_next_key = p_key + v_key * dt + 0.5f * a_key * dt * dt;
//         if (!std::isfinite(p_next_key.x()) || !std::isfinite(p_next_key.y()))
//             return;

//         next.key.x = (int)std::round(p_next_key.x());
//         next.key.y = (int)std::round(p_next_key.y());
//     }

//     bool isCollisionTraj(const Node& a, const Node& b) const {
//         if (!rose_map_)
//             return true;
//         auto wa = rose_map_->key2DToWorld(a.key);
//         auto wb = rose_map_->key2DToWorld(b.key);
//         if (!std::isfinite(wa.x()) || !std::isfinite(wa.y()))
//             return true;
//         if (!std::isfinite(wb.x()) || !std::isfinite(wb.y()))
//             return true;
//         Eigen::Vector2f pa(wa.x(), wa.y());
//         Eigen::Vector2f pb(wb.x(), wb.y());

//         const int N = 10;
//         for (int i = 0; i <= N; i++) {
//             float s = (float)i / N;
//             Eigen::Vector2f pi = pa * (1 - s) + pb * s;
//             if (!std::isfinite(pi.x()) || !std::isfinite(pi.y()))
//                 continue;
//             auto k = rose_map_->worldToKey2D(pi);
//             int idx = rose_map_->key2DToIndex2D(k);
//             if (idx < 0 || idx >= (int)rose_map_->esdf_.size())
//                 continue;
//             if (rose_map_->esdf_[idx] < robot_radius_)
//                 return true;
//         }
//         return false;
//     }

//     bool findNearestSafe(const rose_map::VoxelKey2D& seed, rose_map::VoxelKey2D& out) const {
//         if (!rose_map_ || rose_map_->esdf_.empty())
//             return false;
//         std::queue<rose_map::VoxelKey2D> q;
//         std::unordered_set<int64_t> vis;
//         auto hsh = [&](auto& k) { return (int64_t(k.x) << 32) | uint32_t(k.y); };
//         q.push(seed);
//         vis.insert(hsh(seed));
//         int limit = 300;
//         while (!q.empty() && limit--) {
//             auto c = q.front();
//             q.pop();
//             if (!isOccupied(c)) {
//                 out = c;
//                 return true;
//             }
//             for (int dx = -1; dx <= 1; dx++)
//                 for (int dy = -1; dy <= 1; dy++) {
//                     if (!dx && !dy)
//                         continue;
//                     rose_map::VoxelKey2D nb { c.x + dx, c.y + dy };
//                     auto h = hsh(nb);
//                     if (!vis.count(h)) {
//                         vis.insert(h);
//                         q.push(nb);
//                     }
//                 }
//         }
//         return false;
//     }

//     float cost(const Node& a, const Node& b, const Eigen::Vector2f& acc, float dt) const {
//         if (!rose_map_)
//             return 1e6;
//         return dt + params_.path_search_params_.control_weight * acc.squaredNorm();
//     }

//     float heuristicNode(const Node& a, const rose_map::VoxelKey2D& gk) const {
//         if (!rose_map_)
//             return 1e6;
//         auto wa = rose_map_->key2DToWorld(a.key);
//         auto wg = rose_map_->key2DToWorld(gk);
//         if (!std::isfinite(wa.x()) || !std::isfinite(wa.y()))
//             return 1e6;
//         if (!std::isfinite(wg.x()) || !std::isfinite(wg.y()))
//             return 1e6;
//         return Eigen::Vector2f(wg.x() - wa.x(), wg.y() - wa.y()).norm();
//     }

//     void reconstructTrajectory(int goal_id, Path& traj, float dt) const {
//         if (!rose_map_)
//             return;
//         if (goal_id < 0 || goal_id >= (int)nodes_.size())
//             return;
//         std::vector<int> cids;
//         int id = goal_id, guard = 10000;
//         while (id >= 0 && guard--) {
//             if (id >= (int)nodes_.size())
//                 break;
//             cids.push_back(id);
//             id = nodes_[id].parent;
//         }
//         if (guard <= 0) {
//             std::cerr << "[PathSearch] parent loop!\n";
//             return;
//         }
//         std::reverse(cids.begin(), cids.end());
//         float t = 0;
//         for (int cid: cids) {
//             if (cid < 0 || cid >= (int)nodes_.size())
//                 continue;
//             auto p3 = rose_map_->key2DToWorld(nodes_[cid].key);
//             if (!std::isfinite(p3.x()) || !std::isfinite(p3.y()))
//                 continue;
//             traj.push_back({ t, { p3.x(), p3.y() }, nodes_[cid].vel });
//             t += dt;
//         }
//     }

// private:
//     rose_map::RoseMap::Ptr rose_map_;
//     Parameters params_;
//     float robot_radius_;
//     std::vector<Node> nodes_;
// };

// } // namespace rose_planner

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
        PathSearch(rose_map, params) {}

    static Ptr create(rose_map::RoseMap::Ptr rose_map, Parameters params) {
        return std::make_shared<AStar>(rose_map, params);
    }

    // ==========================================
    // 数据结构定义
    // ==========================================
    struct Node {
        rose_map::VoxelKey2D key;
        float g = 0.0f;
        float f = 0.0f;
        int parent = -1;
    };

    struct PQItem {
        int id; // 指向 nodes_ 中的索引
        float f; // 总代价 F = G + H
        float h; // 启发代价 H (用于 Tie-Breaking)
    };

    // 优化 1: Tie-Breaking 比较器
    // 当 F 值相同时，优先选择 H 值更小（更靠近目标）的节点
    struct PQComp {
        bool operator()(const PQItem& a, const PQItem& b) const {
            constexpr float EPSILON = 1e-4f;
            // 如果 F 值非常接近
            if (std::abs(a.f - b.f) < EPSILON) {
                // H 越小越好 (注意 priority_queue 默认是最大堆，所以用 >)
                return a.h > b.h;
            }
            // 否则 F 越小越好
            return a.f > b.f;
        }
    };

    // ==========================================
    // 核心搜索函数
    // ==========================================
    bool search(const Eigen::Vector2f& start_w, const Eigen::Vector2f& goal_w, Path& path) {
        auto start = rose_map_->worldToKey2D(start_w);
        auto goal = rose_map_->worldToKey2D(goal_w);

        // 检查起终点安全性
        if (!PathSearch::checkStartGoalSafe(start, goal)) {
            return false;
        }

        // 初始化容器
        nodes_.clear();
        nodes_.reserve(4096); // 预分配内存避免频繁 reallocation

        std::priority_queue<PQItem, std::vector<PQItem>, PQComp> open;
        // Key: 哈希值, Value: nodes_ 中的索引
        std::unordered_map<int64_t, int> visited_index_map;

        auto hash = [&](const rose_map::VoxelKey2D& k) -> int64_t {
            return (int64_t(k.x) << 32) | uint32_t(k.y);
        };

        // 初始化起点 Node
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

        while (!open.empty()) {
            PQItem current_item = open.top();
            open.pop();
            int cid = current_item.id;

            if (current_item.f > nodes_[cid].f + 1e-4f) {
                continue;
            }

            const auto& ck = nodes_[cid].key;

            // 到达目标
            if (ck.x == goal.x && ck.y == goal.y) {
                path.clear();
                for (int id = cid; id >= 0; id = nodes_[id].parent) {
                    auto w = rose_map_->key2DToWorld(nodes_[id].key);
                    path.push_back({ w.x(), w.y(), 0 });
                }
                std::reverse(path.begin(), path.end());
                return true;
            }

            // 扩展邻居
            std::vector<rose_map::VoxelKey2D> succ;
            getSuccessors(ck, succ);

            for (const auto& nbk: succ) {
                int64_t h_key = hash(nbk);

                float step_cost = stepCostWithClearance(ck, nbk);
                float ng = nodes_[cid].g + step_cost;
                float nh = clearanceHeuristic(nbk, goal) * heu_weight;
                float nf = ng + nh;

                // 检查节点是否已访问
                auto it = visited_index_map.find(h_key);

                if (it != visited_index_map.end()) {
                    int exist_id = it->second;

                    // 优化 3: 严格的 G 值检查 (松弛操作)
                    // 如果新路径代价 >= 旧路径代价，直接剪枝
                    if (ng >= nodes_[exist_id].g) {
                        continue;
                    }

                    // 否则，发现了更优路径：更新旧节点信息
                    nodes_[exist_id].g = ng;
                    nodes_[exist_id].f = nf;
                    nodes_[exist_id].parent = cid;

                    // 将更新后的节点重新加入堆中
                    open.push({ exist_id, nf, nh });
                } else {
                    // 全新节点：创建并加入
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
        return false;
    }

private:
    std::vector<Node> nodes_;

    // 启发式函数：欧几里得距离 + 障碍物距离势场
    float clearanceHeuristic(const rose_map::VoxelKey2D& a, const rose_map::VoxelKey2D& b) const {
        float dx = float(a.x - b.x);
        float dy = float(a.y - b.y);
        float dist = std::sqrt(dx * dx + dy * dy);

        int idx = rose_map_->key2DToIndex2D(a);
        float d = (idx < 0) ? 0 : rose_map_->esdf_[idx];

        // 避免除以零，加一个常数
        float clearance_term =
            params_.path_search_params_.a_star.clearance_weight * (1.0f / (d + 0.1f));
        return dist + clearance_term;
    }

    // 移动代价函数：距离代价 + 靠近障碍物的惩罚
    float
    stepCostWithClearance(const rose_map::VoxelKey2D& a, const rose_map::VoxelKey2D& b) const {
        // 直走代价 1.0，斜走代价 sqrt(2)
        float base = (std::abs(a.x - b.x) + std::abs(a.y - b.y) == 1) ? 1.0f : 1.41421356f;

        int idx = rose_map_->key2DToIndex2D(b);
        float d = (idx < 0) ? 0 : rose_map_->esdf_[idx];

        float penalty =
            params_.path_search_params_.a_star.obstacle_penalty_weight * (1.0f / (d + 0.1f));
        return base + penalty;
    }

    // 获取 8 邻域
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