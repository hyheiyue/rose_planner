#include "planner_manager.hpp"
#include "path_search/a*.hpp"
#include "trajectory_optimize/trajectory_opt.hpp"
namespace rose_planner {
struct PlannerManager::Impl {
public:
    using Ptr = std::unique_ptr<PlannerManager>;
    Impl(
        rclcpp::Node& node,
        const Parameters& params,
        Robo::Ptr robo,
        rose_map::RoseMap::Ptr rose_map
    ) {
        node_ = &node;
        params_ = params;
        robo_ = robo;
        rose_map_ = rose_map;
        path_search_ = AStar::create(rose_map_, params_);
        traj_opt_ = TrajectoryOpt::create(rose_map_, params_);
        state_ = FSMSTATE::INIT;
        timer_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                auto start = std::chrono::steady_clock::now();
                timerCallback();
                auto end = std::chrono::steady_clock::now();
                auto cost =
                    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                if (cost < 100)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100 - cost));
            }
        });
        raw_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("raw_path", 10);
        opt_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("opt_path", 10);
        opt_marker_pub_ =
            node.create_publisher<visualization_msgs::msg::Marker>("/opt_traj_marker", 10);
    }
    static Ptr create(
        rclcpp::Node& node,
        const Parameters& params,
        Robo::Ptr robo,
        rose_map::RoseMap::Ptr rose_map
    ) {
        return std::make_unique<PlannerManager>(node, params, robo, rose_map);
    }
    void timerCallback() {
        bool change_to_wait = false;
        switch (state_) {
            case FSMSTATE::INIT: {
                break;
            }

            case FSMSTATE::WAIT_GOAL: {
                break;
            }

            case FSMSTATE::REPLAN: {
                Eigen::Vector2d goal_pos = goal_.pos.front();
                auto current = robo_->getNowState();
                double dist_to_goal = (goal_pos - current.pos).norm();

                if (dist_to_goal < 0.5) {
                    if (goal_.pos.empty()) {
                        state_ = FSMSTATE::WAIT_GOAL;
                        change_to_wait = true;
                        break;
                    }
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Goal reached (dist=%.2f m), waiting for new goal",
                        dist_to_goal
                    );
                    goal_.pos.pop_front();
                    if (goal_.pos.empty()) {
                        state_ = FSMSTATE::WAIT_GOAL;
                        change_to_wait = true;
                        break;
                    } else {
                        state_ = FSMSTATE::SEARCH_PATH;
                    }
                    break;
                }

                removeOldTraj();
                removeOldPath();
                auto unsafe_points = checkSafePath(current_raw_path_);
                if (!unsafe_points.empty()) {
                    traj_valid_ = false;
                    localReplan(unsafe_points, goal_pos);
                } else {
                    unsafe_points = checkSafePath(current_traj_.toPointVector(0.05), 0.05, 3.0);
                    double t_cur = current_traj_.getTimeByPos(current.pos, 0.5);
                    if (!unsafe_points.empty()
                            && current_raw_path_.size() > (1 / rose_map_->esdf_->esdf_->voxel_size)
                        || t_cur < 0.0)
                    {
                        traj_valid_ = false;
                        resampleAndOpt(current_raw_path_);
                    }
                }

                if (state_ == FSMSTATE::WAIT_GOAL) {
                    change_to_wait = true;
                }
                break;
            }

            case FSMSTATE::SEARCH_PATH: {
                if (goal_.pos.empty()) {
                    state_ = FSMSTATE::WAIT_GOAL;
                    change_to_wait = true;
                    break;
                }
                traj_valid_ = false;
                searchOnce(goal_);
                if (state_ == FSMSTATE::WAIT_GOAL) {
                    change_to_wait = true;
                }
                break;
            }
        }
    }
    void localReplan(const std::vector<int>& unsafe_points, const Eigen::Vector2d& goal_w) {
        if (current_raw_path_.size() < 2) {
            RCLCPP_WARN(node_->get_logger(), "Raw path too short for local replan.");
            return;
        }
        auto current = robo_->getNowState();
        const int Num = static_cast<int>(current_raw_path_.size());
        Eigen::Vector2d start_w = current.pos;
        int local_end_idx = unsafe_points.back() + (1 / rose_map_->esdf_->esdf_->voxel_size);

        local_end_idx = std::clamp(local_end_idx, 0, Num - 1);

        int next_idx = local_end_idx + 1;
        if (next_idx >= Num) {
            RCLCPP_WARN(node_->get_logger(), "Local replan end is last point, skipping.");
            state_ = FSMSTATE::WAIT_GOAL;
            return;
        }
        std::vector<Eigen::Vector2d> path_after(
            current_raw_path_.begin() + next_idx,
            current_raw_path_.end()
        );
        auto local_path = search(start_w, current_raw_path_[next_idx]);
        if (local_path.empty()) {
            state_ = FSMSTATE::WAIT_GOAL;
        }
        std::vector<Eigen::Vector2d> new_raw_path;
        new_raw_path.reserve(local_path.size() + path_after.size());

        for (const auto& p: local_path) {
            new_raw_path.emplace_back(p.x(), p.y());
        }
        new_raw_path.insert(new_raw_path.end(), path_after.begin(), path_after.end());

        current_raw_path_ = new_raw_path;
        resampleAndOpt(current_raw_path_);
    }
    std::vector<int> checkSafePath(const std::vector<Eigen::Vector2d>& path) const noexcept {
        std::vector<int> unsafe_points;

        if (path.size() < 2)
            return unsafe_points;

        if (!rose_map_ || !rose_map_->esdf_ || !rose_map_->esdf_->esdf_)
            return unsafe_points;

        const auto& esdf = rose_map_->esdf_;

        const double step = std::min(esdf->esdf_->voxel_size * 0.5, params_.robot_radius * 0.5);

        if (step <= 1e-6)
            return unsafe_points;

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const Eigen::Vector2d p0 = path[i];
            const Eigen::Vector2d p1 = path[i + 1];

            const double seg_len = (p1 - p0).norm();
            const int n = std::max(1, (int)std::ceil(seg_len / step));

            bool unsafe = false;

            for (int k = 0; k <= n; ++k) {
                double alpha = (double)k / n;
                Eigen::Vector2d p = p0 + alpha * (p1 - p0);

                auto key = esdf->worldToKey(p.cast<float>());
                int idx = esdf->keyToIndex(key);

                if (idx < 0)
                    continue;

                if (esdf->getEsdf(idx) < params_.robot_radius) {
                    unsafe = true;
                    break;
                }
            }

            if (unsafe)
                unsafe_points.push_back(i);
        }

        return unsafe_points;
    }

    std::vector<int>
    checkSafePath(const std::vector<Eigen::Vector2d>& path, double sample_dt, double horizon)
        const noexcept {
        std::vector<Eigen::Vector2d> path_cut = path;

        int target_size = static_cast<int>(horizon / sample_dt);
        if (target_size < 0) {
            target_size = 0;
        }

        if (static_cast<int>(path_cut.size()) > target_size) {
            path_cut.resize(target_size);
        }

        return checkSafePath(path_cut);
    }
    void removeOldTraj() {
        auto current = robo_->getNowState();
        double t_cur = current_traj_.getTimeByPos(current.pos, 0.5);
        if (t_cur < 0) {
            return;
        }
        current_traj_.truncateBeforeTime(t_cur);
    }
    void searchOnce(const Goal& goal) {
        if (goal.pos.empty()) {
            return;
        }
        auto current = robo_->getNowState();
        Eigen::Vector2d start_w = current.pos;
        std::vector<Eigen::Vector2d> path;
        Eigen::Vector2d local_start = start_w;
        // for (const auto& goal_pos: goal.pos) {
        //     auto local = search(local_start, goal_pos);
        //     if (local.empty()) {
        //         state_ = FSMSTATE::WAIT_GOAL;
        //         return;
        //     }
        //     local_start = local.back();
        //     path.insert(path.end(), local.begin(), local.end());
        // }
        path = search(local_start, goal.pos.front());
        current_raw_path_ = path;
        resampleAndOpt(current_raw_path_);
        state_ = FSMSTATE::REPLAN;
    }
    std::vector<Eigen::Vector2d> search(const Eigen::Vector2d& start, const Eigen::Vector2d& goal) {
        PathSearch::Path path;
        SearchState search_state = SearchState::NO_PATH;

        try {
            search_state = path_search_->search(start, goal, path);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "A* search exception: %s", e.what());
        }
        if (search_state == SearchState::SUCCESS) {
            return path;
        } else if (search_state == SearchState::NO_PATH) {
            RCLCPP_WARN(node_->get_logger(), "No path found by A*");

        } else if (search_state == SearchState::TIMEOUT) {
            RCLCPP_WARN(node_->get_logger(), "A* search timeout");
        }
        return path;
    }
    void removeOldPath() {
        if (current_raw_path_.empty())
            return;

        auto current = robo_->getNowState();

        int best_target_index = -1;
        double best_dist2 = std::numeric_limits<double>::infinity();

        for (int i = 0; i < static_cast<int>(current_raw_path_.size()); ++i) {
            double dx = current_raw_path_[i].x() - current.pos.x();
            double dy = current_raw_path_[i].y() - current.pos.y();
            double dist2 = dx * dx + dy * dy;

            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                best_target_index = i;
            }
        }
        if (best_target_index > 0) {
            current_raw_path_.erase(
                current_raw_path_.begin(),
                current_raw_path_.begin() + best_target_index
            );
        }
    }
    void resampleAndOpt(const std::vector<Eigen::Vector2d>& path) noexcept {
        nav_msgs::msg::Path raw_path_msg;
        raw_path_msg.header.stamp = node_->now();
        raw_path_msg.header.frame_id = params_.target_frame;
        nav_msgs::msg::Path opt_path_msg;
        opt_path_msg.header.stamp = node_->now();
        opt_path_msg.header.frame_id = params_.target_frame;
        auto current = robo_->getNowState();
        Eigen::Vector2d start_v = current.vel;
        // Publish raw path
        if (raw_path_pub_->get_subscription_count() > 0) {
            for (int i = 0; i < (int)path.size(); i = i + 3) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header = raw_path_msg.header;
                pose_msg.pose.position.x = path[i].x();
                pose_msg.pose.position.y = path[i].y();
                pose_msg.pose.position.z = 0.0;

                raw_path_msg.poses.push_back(pose_msg);
            }
            raw_path_pub_->publish(raw_path_msg);
        }
        traj_opt_->setPath(path, current);
        traj_opt_->optimize();
        auto opt_traj = traj_opt_->getTrajectory();
        if (opt_traj.getPieceNum() > 1 && opt_traj.getPieceNum() < 1000) {
            current_traj_ = opt_traj;
            traj_valid_ = true;
        }
        // Publish optimized path + marker spheres
        if ((opt_path_pub_->get_subscription_count() > 0
             || opt_marker_pub_->get_subscription_count() > 0)
            && opt_traj.getPieceNum() > 1)
        {
            double totalDur = opt_traj.getTotalDuration();
            double dt = 0.1;
            int sampleNum = static_cast<int>(totalDur / dt) + 2;

            double t_cur = 0.0;
            opt_path_msg.poses.clear();

            visualization_msgs::msg::Marker marker;
            marker.header = opt_path_msg.header;
            marker.ns = "opt_traj";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.scale.x = 0.2; // 球半径（直径）
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.r = 0.1f;
            marker.color.g = 0.1f;
            marker.color.b = 0.9f;
            marker.color.a = 1.0f;

            marker.points.clear();

            for (int i = 0; i < sampleNum; ++i) {
                Eigen::VectorXd pos = opt_traj.getPos(t_cur);
                Eigen::VectorXd vel = opt_traj.getVel(t_cur);
                if (pos.size() < 2 || vel.size() < 2)
                    break;

                double yaw =
                    std::hypot(vel.x(), vel.y()) > 1e-3 ? std::atan2(vel.y(), vel.x()) : 0.0;
                geometry_msgs::msg::PoseStamped p;
                p.header = opt_path_msg.header;
                p.pose.position.x = pos.x();
                p.pose.position.y = pos.y();
                p.pose.position.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                q.normalize();
                p.pose.orientation = tf2::toMsg(q);

                opt_path_msg.poses.push_back(p);
                geometry_msgs::msg::Point mp;
                mp.x = pos.x();
                mp.y = pos.y();
                mp.z = p.pose.position.z;
                marker.points.push_back(mp);

                t_cur = std::min(t_cur + dt, totalDur);
            }

            // 发布
            opt_path_pub_->publish(opt_path_msg);
            opt_marker_pub_->publish(marker);
        }
    }
    std::optional<TrajType> getTrajectory() {
        if (!traj_valid_) {
            return std::nullopt;
        }
        return current_traj_;
    }
    void setNewGoal(const Goal& g) {
        goal_ = g;
        state_ = FSMSTATE::SEARCH_PATH;
    }
    FSMSTATE state_;
    rclcpp::Node* node_;
    Parameters params_;
    std::thread timer_thread_;
    AStar::Ptr path_search_;
    rose_map::RoseMap::Ptr rose_map_;
    TrajectoryOpt::Ptr traj_opt_;
    Robo::Ptr robo_;
    Goal goal_;
    std::vector<Eigen::Vector2d> current_raw_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr opt_marker_pub_;
    TrajType current_traj_;
    bool traj_valid_ = false;
};
PlannerManager::PlannerManager(
    rclcpp::Node& node,
    const Parameters& params,
    Robo::Ptr robo,
    rose_map::RoseMap::Ptr rose_map
) {
    _impl = std::make_unique<Impl>(node, params, robo, rose_map);
}
PlannerManager::~PlannerManager() {
    _impl.reset();
}

std::optional<TrajType> PlannerManager::getTrajectory() {
    return _impl->getTrajectory();
}
void PlannerManager::setNewGoal(const Goal& g) {
    _impl->setNewGoal(g);
}
PlannerManager::FSMSTATE PlannerManager::getState() const {
    return _impl->state_;
}
} // namespace rose_planner