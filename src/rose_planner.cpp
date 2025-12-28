#include "rose_planner.hpp"
#include "a*.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "parameters.hpp"
#include "path_search.hpp"
#include "replan_fsm.hpp"
#include "rose_map/rose_map.hpp"
#include "trajectory_opt.hpp"
#include "trajectory_sampler.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
namespace rose_planner {
struct RosePlanner::Impl {
public:
    using SearchType = AStar;

    Impl(rclcpp::Node& node) {
        node_ = &node;
        parameters_.load(node);

        rose_map_ = std::make_shared<rose_map::RoseMap>(node);
        path_search_ = SearchType::create(rose_map_, parameters_);
        traj_opt_ = TrajectoryOpt::create(rose_map_, parameters_);

        std::string odom_topic = node.declare_parameter<std::string>("odom_topic", "");
        odometry_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&RosePlanner::Impl::odomCallback, this, std::placeholders::_1)
        );

        std::string goal_topic = node.declare_parameter<std::string>("goal_topic", "");
        goal_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic,
            10,
            std::bind(&RosePlanner::Impl::goalPoseCallback, this, std::placeholders::_1)
        );

        goal_point_sub_ = node.create_subscription<geometry_msgs::msg::PointStamped>(
            goal_topic,
            10,
            std::bind(&RosePlanner::Impl::goalPointCallback, this, std::placeholders::_1)
        );

        raw_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("raw_path", 10);
        opt_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("opt_path", 10);

        timer_ = node.create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RosePlanner::Impl::timerCallback, this)
        );

        replan_fsm_.state_ = ReplanFSM::INIT;
        RCLCPP_INFO(node_->get_logger(), "RosePlanner initialized");
    }

    void timerCallback() {
        static auto last_fps_time = node_->now();
        static int frame_count = 0;

        ++frame_count;

        auto current_time = node_->now();
        double elapsed_sec = (current_time - last_fps_time).seconds();

        if (elapsed_sec >= 1.0) {
            double fps = frame_count / elapsed_sec;
            RCLCPP_INFO(
                node_->get_logger(),
                "Planner callback frequency: %.2f Hz (%d frames in %.1f s)",
                fps,
                frame_count,
                elapsed_sec
            );

            frame_count = 0;
            last_fps_time = current_time;
        }

        switch (replan_fsm_.state_) {
            case ReplanFSM::INIT:
                RCLCPP_DEBUG(node_->get_logger(), "FSM: INIT");
                break;

            case ReplanFSM::WAIT_GOAL:
                RCLCPP_DEBUG(node_->get_logger(), "FSM: WAIT_GOAL");
                break;

            case ReplanFSM::REPLAN: {
                Eigen::Vector2f goal_pos(
                    current_goal_.pose.position.x,
                    current_goal_.pose.position.y
                );
                Eigen::Vector2f current_pos(current_pose_.position.x, current_pose_.position.y);
                double dist_to_goal = (goal_pos - current_pos).norm();

                if (dist_to_goal < 0.5) {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Goal reached (dist=%.2f m), waiting for new goal",
                        dist_to_goal
                    );
                    replan_fsm_.state_ = ReplanFSM::WAIT_GOAL;
                    break;
                }

                auto unsafe_points = checkSafePath(current_path_);
                if (unsafe_points.empty()) {
                    RCLCPP_DEBUG(
                        node_->get_logger(),
                        "Current path safe, performing optimization only"
                    );
                    onlyOpt();
                } else {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "Detected %zu unsafe point(s), triggering local replan",
                        unsafe_points.size()
                    );
                    localReplan(unsafe_points, goal_pos);
                }
                break;
            }

            case ReplanFSM::SEARCH_PATH: {
                Eigen::Vector2f goal_w(
                    current_goal_.pose.position.x,
                    current_goal_.pose.position.y
                );
                RCLCPP_INFO(node_->get_logger(), "New goal received, starting global search");
                replan_fsm_.state_ = ReplanFSM::REPLAN;
                searchOnce(goal_w);
                break;
            }
        }
    }

    std::vector<int> checkSafePath(const std::vector<Eigen::Vector2d>& path) {
        std::vector<int> unsafe_points;
        for (int i = 0; i < path.size(); ++i) {
            auto key = rose_map_->worldToKey2D(path[i].cast<float>());
            int idx = rose_map_->key2DToIndex2D(key);
            if (idx < 0)
                continue;
            if (rose_map_->esdf_[idx] < parameters_.robot_radius) {
                unsafe_points.push_back(i);
            }
        }
        return unsafe_points;
    }

    int findClosestPointIndex(
        const Eigen::Vector2d& start_w,
        const std::vector<Eigen::Vector2d>& current_path
    ) {
        if (current_path.empty())
            return -1;

        constexpr double MATCH_DIST2 = 1.0 * 1.0;
        int best_index = 0;
        double best_dist2 = std::numeric_limits<double>::infinity();
        int first_forward_match = -1;

        for (int i = 0; i < (int)current_path.size(); ++i) {
            double dx = current_path[i].x() - start_w.x();
            double dy = current_path[i].y() - start_w.y();
            double dist2 = dx * dx + dy * dy;

            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                best_index = i;
            }

            if (first_forward_match == -1 && dist2 < MATCH_DIST2) {
                first_forward_match = i;
                break;
            }
        }

        return (first_forward_match != -1) ? first_forward_match : best_index;
    }

    void localReplan(const std::vector<int>& unsafe_points, const Eigen::Vector2f& goal_w) {
        if (unsafe_points.empty() || current_raw_path_.empty())
            return;

        nav_msgs::msg::Path raw_path_msg, opt_path_msg;
        raw_path_msg.header.stamp = opt_path_msg.header.stamp = node_->now();
        raw_path_msg.header.frame_id = opt_path_msg.header.frame_id = "map";

        int local_start_idx = unsafe_points.front();
        int local_end_idx = unsafe_points.back();
        int N = (int)current_raw_path_.size();

        local_start_idx = std::clamp(local_start_idx, 0, N - 1);
        local_end_idx = std::clamp(local_end_idx, 0, N - 1);
        if (local_start_idx > local_end_idx)
            return;

        std::vector<Eigen::Vector2d> path_after(
            current_raw_path_.begin() + local_end_idx + 1,
            current_raw_path_.end()
        );

        PathSearch::Path local_path;
        SearchState search_state = SearchState::NO_PATH;

        Eigen::Vector2d start_w(current_pose_.position.x, current_pose_.position.y);

        try {
            search_state = path_search_->search(
                start_w.cast<float>(),
                current_raw_path_[local_end_idx].cast<float>(),
                local_path
            );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Local A* exception: %s", e.what());
            search_state = SearchState::NO_PATH;
        }

        if (search_state != SearchState::SUCCESS) {
            RCLCPP_WARN(node_->get_logger(), "Local replan failed, falling back to global search");
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
            return;
        }

        std::vector<Eigen::Vector2d> new_raw_path;
        new_raw_path.reserve(local_path.size() + path_after.size());
        for (const auto& p: local_path) {
            new_raw_path.emplace_back(p.x(), p.y());
        }
        new_raw_path.insert(new_raw_path.end(), path_after.begin(), path_after.end());

        current_raw_path_.swap(new_raw_path);
        RCLCPP_INFO(
            node_->get_logger(),
            "Local replan succeeded (%zu new points), optimizing trajectory",
            local_path.size()
        );

        resampleAndOpt(current_raw_path_);
    }

    void onlyOpt() {
        Eigen::Vector2d start_w(current_pose_.position.x, current_pose_.position.y);
        int idx = findClosestPointIndex(start_w, current_raw_path_);

        if (idx == -1) {
            RCLCPP_WARN(
                node_->get_logger(),
                "No valid point on current path, triggering global search"
            );
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
            return;
        }

        if (idx != 0) {
            current_raw_path_.erase(current_raw_path_.begin(), current_raw_path_.begin() + idx);
            RCLCPP_DEBUG(node_->get_logger(), "Path trimmed from index %d to start", idx);
        }

        current_raw_path_[0] = start_w;
        resampleAndOpt(current_raw_path_);
    }

    void searchOnce(const Eigen::Vector2f& goal_w) {
        Eigen::Vector2d start_w(current_pose_.position.x, current_pose_.position.y);
        current_path_.clear();
        current_raw_path_.clear();

        RCLCPP_INFO(
            node_->get_logger(),
            "Global planning | Start: [%.2f, %.2f] → Goal: [%.2f, %.2f]",
            start_w.x(),
            start_w.y(),
            goal_w.x(),
            goal_w.y()
        );

        using Clock = std::chrono::system_clock;
        using Ms = std::chrono::milliseconds;
        auto t0_total = Clock::now();
        auto t1_search_start = Clock::now();

        PathSearch::Path path;
        SearchState search_state = SearchState::NO_PATH;

        try {
            search_state = path_search_->search(start_w.cast<float>(), goal_w, path);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "A* search exception: %s", e.what());
        }

        auto t1_search_end = Clock::now();
        auto search_ms = std::chrono::duration_cast<Ms>(t1_search_end - t1_search_start).count();

        if (search_state == SearchState::SUCCESS) {
            auto t2_opt_start = Clock::now();
            current_raw_path_ = path;
            resampleAndOpt(current_raw_path_);
            auto t2_opt_end = Clock::now();

            auto opt_ms = std::chrono::duration_cast<Ms>(t2_opt_end - t2_opt_start).count();
            auto total_ms = std::chrono::duration_cast<Ms>(t2_opt_end - t0_total).count();

            RCLCPP_INFO(
                node_->get_logger(),
                "Planning success | A*: %ld ms | Opt: %ld ms | Total: %ld ms | Raw: %zu pts | Opt: %zu pts",
                search_ms,
                opt_ms,
                total_ms,
                current_raw_path_.size(),
                current_path_.size()
            );

        } else if (search_state == SearchState::NO_PATH) {
            RCLCPP_WARN(
                node_->get_logger(),
                "No path found by A*, waiting for new goal or map update"
            );
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;

        } else if (search_state == SearchState::TIMEOUT) {
            RCLCPP_WARN(node_->get_logger(), "A* search timeout, abandoning current goal");
            replan_fsm_.state_ = ReplanFSM::WAIT_GOAL;
        }
    }

    void resampleAndOpt(const std::vector<Eigen::Vector2d>& path) {
        nav_msgs::msg::Path raw_path_msg;
        raw_path_msg.header.stamp = node_->now();
        raw_path_msg.header.frame_id = "map";

        nav_msgs::msg::Path opt_path_msg;
        opt_path_msg.header.stamp = node_->now();
        opt_path_msg.header.frame_id = "map";

        Eigen::Vector2d start_v(
            current_odom_.twist.twist.linear.x,
            current_odom_.twist.twist.linear.y
        );

        auto traj = sampleTrajectoryTrapezoid(
            path,
            parameters_.path_search_params_.resampler.acc,
            parameters_.path_search_params_.resampler.max_vel,
            parameters_.path_search_params_.resampler.dt,
            start_v
        );

        traj_opt_->setSampledPath(
            traj,
            parameters_.path_search_params_.resampler.dt,
            start_v.cast<double>()
        );

        traj_opt_->optimize();
        auto opt_traj = traj_opt_->getTrajectory();
        current_path_ = opt_traj;

        // Publish raw path
        if (raw_path_pub_->get_subscription_count() > 0) {
            for (const auto& point: traj) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header = raw_path_msg.header;
                pose_msg.pose.position.x = point.p.x();
                pose_msg.pose.position.y = point.p.y();
                pose_msg.pose.position.z = current_goal_.pose.position.z;

                float yaw = std::atan2(point.v.y(), point.v.x());
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                q.normalize();
                pose_msg.pose.orientation.x = q.x();
                pose_msg.pose.orientation.y = q.y();
                pose_msg.pose.orientation.z = q.z();
                pose_msg.pose.orientation.w = q.w();

                raw_path_msg.poses.push_back(pose_msg);
            }
            raw_path_pub_->publish(raw_path_msg);
        }

        // Publish optimized path
        if (opt_path_pub_->get_subscription_count() > 0) {
            for (const auto& point: opt_traj) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header = opt_path_msg.header;
                pose_msg.pose.position.x = point.x();
                pose_msg.pose.position.y = point.y();
                pose_msg.pose.position.z = current_goal_.pose.position.z;
                opt_path_msg.poses.push_back(pose_msg);
            }
            opt_path_pub_->publish(opt_path_msg);
        }
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(
            node_->get_logger(),
            "Received PoseStamped goal [%.2f, %.2f]",
            msg->pose.position.x,
            msg->pose.position.y
        );
        current_goal_ = *msg;
        replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
    }

    void goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        RCLCPP_INFO(
            node_->get_logger(),
            "Received PointStamped goal [%.2f, %.2f]",
            msg->point.x,
            msg->point.y
        );

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = msg->point.x;
        pose.pose.position.y = msg->point.y;
        pose.pose.position.z = msg->point.z;
        pose.pose.orientation.w = 1.0;

        current_goal_ = pose;
        replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rose_map_->setOrigin(Eigen::Vector3f(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        ));
        current_pose_ = msg->pose.pose;
        current_odom_ = *msg;
    }

private:
    Parameters parameters_;
    rclcpp::Node* node_;

    SearchType::Ptr path_search_;
    rose_map::RoseMap::Ptr rose_map_;
    TrajectoryOpt::Ptr traj_opt_;

    geometry_msgs::msg::Pose current_pose_;
    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::PoseStamped current_goal_;

    ReplanFSM replan_fsm_;

    std::vector<Eigen::Vector2d> current_path_;
    std::vector<Eigen::Vector2d> current_raw_path_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_pub_;
};

RosePlanner::RosePlanner(rclcpp::Node& node) {
    _impl = std::make_unique<Impl>(node);
}
RosePlanner::~RosePlanner() {
    _impl.reset();
}
} // namespace rose_planner