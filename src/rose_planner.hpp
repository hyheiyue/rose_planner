#pragma once
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

class RosePlanner {
public:
    using Ptr = std::unique_ptr<RosePlanner>;
    using SearchType = AStar;
    explicit RosePlanner(rclcpp::Node& node) {
        node_ = &node;
        parameters_.load(node);

        rose_map_ = std::make_shared<rose_map::RoseMap>(node);
        path_search_ = SearchType::create(rose_map_, parameters_);
        traj_opt_ = TrajectoryOpt::create(rose_map_, parameters_);
        std::string odom_topic = node.declare_parameter<std::string>("odom_topic", "");
        odometry_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&RosePlanner::odomCallback, this, std::placeholders::_1)
        );

        std::string goal_topic = node.declare_parameter<std::string>("goal_topic", "");
        goal_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic,
            10,
            std::bind(&RosePlanner::goalPoseCallback, this, std::placeholders::_1)
        );

        goal_point_sub_ = node.create_subscription<geometry_msgs::msg::PointStamped>(
            goal_topic,
            10,
            std::bind(&RosePlanner::goalPointCallback, this, std::placeholders::_1)
        );

        raw_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("raw_path", 10);
        opt_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("opt_path", 10);
        timer_ = node.create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RosePlanner::timerCallback, this)
        );

        replan_fsm_.state_ = ReplanFSM::INIT;
    }

    static Ptr create(rclcpp::Node& node) {
        return std::make_unique<RosePlanner>(node);
    }

    void timerCallback() {
        switch (replan_fsm_.state_) {
            case ReplanFSM::INIT:
                break;

            case ReplanFSM::WAIT_GOAL:
                break;

            case ReplanFSM::REPLAN: {
                Eigen::Vector2f goal_pos(
                    current_goal_.pose.position.x,
                    current_goal_.pose.position.y
                );
                Eigen::Vector2f current_pos(current_pose_.position.x, current_pose_.position.y);
                if ((goal_pos - current_pos).norm() < 0.5) {
                    replan_fsm_.state_ = ReplanFSM::WAIT_GOAL;
                    break;
                }
                auto unsafe_points = checkSafePath(current_path_);
                if (unsafe_points.size() < 1) {
                    onlyOpt();
                } else {
                    localReplan(unsafe_points, goal_pos);
                    std::cout << "unsafe points: " << unsafe_points.size() << std::endl;
                }

                break;
            }
            case ReplanFSM::SEARCH_PATH: {
                Eigen::Vector2f goal_w(
                    current_goal_.pose.position.x,
                    current_goal_.pose.position.y
                );
                replan_fsm_.state_ = ReplanFSM::REPLAN;
                searchOnce(goal_w);
                break;
            }
        }
    }
    std::vector<int> checkSafePath(const std::vector<Eigen::Vector2d>& path) {
        std::vector<int> unsafe_points;
        for (int i = 0; i < path.size(); i++) {
            auto key = rose_map_->worldToKey2D(path[i].cast<float>());
            int idx = rose_map_->key2DToIndex2D(key);
            if (idx < 0) {
                continue;
            }
            if (rose_map_->esdf_[idx] < parameters_.robot_radius) {
                unsafe_points.push_back(i);
                //RCLCPP_ERROR_STREAM(node_->get_logger(), "Unsafe point: " << point.transpose());
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

        for (int i = 0; i < (int)current_path.size(); i++) {
            double dx = current_path[i].x() - start_w.x();
            double dy = current_path[i].y() - start_w.y();
            double dist2 = dx * dx + dy * dy;

            // 记录全局最近点（用于 fallback）
            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                best_index = i;
            }

            // 记录前向方向上第一个足够近的点
            if (first_forward_match == -1 && dist2 < MATCH_DIST2) {
                first_forward_match = i;
                break; // 关键：遇到第一个前向可匹配点就停止
            }
        }

        return (first_forward_match != -1) ? first_forward_match : best_index;
    }

    void localReplan(const std::vector<int>& unsafe_points, const Eigen::Vector2f& goal_w) {
        if (unsafe_points.empty() || current_raw_path_.empty())
            return;

        nav_msgs::msg::Path opt_path_msg, raw_path_msg;
        opt_path_msg.header.stamp = raw_path_msg.header.stamp = node_->now();
        opt_path_msg.header.frame_id = raw_path_msg.header.frame_id = "map";

        int local_start_idx = unsafe_points.front();
        int local_end_idx = unsafe_points.back();
        int N = (int)current_raw_path_.size();

        local_start_idx = std::clamp(local_start_idx, 0, N - 1);
        local_end_idx = std::clamp(local_end_idx, 0, N - 1);
        if (local_start_idx > local_end_idx)
            return;

        std::vector<Eigen::Vector2d> path_after;
        path_after.insert(
            path_after.end(),
            current_raw_path_.begin() + local_end_idx + 1,
            current_raw_path_.end()
        );
        PathSearch::Path local_path;
        SearchState search_state = SearchState::NO_PATH;
        Eigen::Vector2d vel2d(
            current_odom_.twist.twist.linear.x,
            current_odom_.twist.twist.linear.y
        );
        Eigen::Vector2d start_w(current_pose_.position.x, current_pose_.position.y);
        try {
            search_state = path_search_->search(
                start_w.cast<float>(),
                current_raw_path_[local_end_idx].cast<float>(),
                local_path
            );
        } catch (std::exception& e) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "A* exception: " << e.what());
            search_state = SearchState::NO_PATH;
        }

        if (search_state != SearchState::SUCCESS) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Local replan failed");
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
            return;
        }

        std::vector<Eigen::Vector2d> new_raw_path;
        new_raw_path.reserve(local_path.size() + path_after.size() + 1);
        for (const auto& p: local_path) {
            new_raw_path.emplace_back(p.x(), p.y());
        }
        new_raw_path.insert(new_raw_path.end(), path_after.begin(), path_after.end());
        current_raw_path_.swap(new_raw_path);
        resampleAndOpt(current_raw_path_);
    }

    void onlyOpt() {
        nav_msgs::msg::Path opt_path_msg;
        opt_path_msg.header.stamp = node_->now();
        opt_path_msg.header.frame_id = "map";
        nav_msgs::msg::Path raw_path_msg;
        raw_path_msg.header.stamp = node_->now();
        raw_path_msg.header.frame_id = "map";
        Eigen::Vector2d start_w(current_pose_.position.x, current_pose_.position.y);
        int idx = findClosestPointIndex(start_w, current_raw_path_);
        current_path_.clear();
        if (idx == -1) {
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
            return;
        }
        if (idx != 0) {
            current_raw_path_.erase(current_raw_path_.begin(), current_raw_path_.begin() + idx);
        }
        if (current_raw_path_.empty()) {
            current_raw_path_.push_back(start_w);
        }
        current_raw_path_[0] = start_w;
        resampleAndOpt(current_raw_path_);
    }

    void searchOnce(const Eigen::Vector2f& goal_w) {
        Eigen::Vector2d start_w(current_pose_.position.x, current_pose_.position.y);
        current_path_.clear();
        current_raw_path_.clear();
        nav_msgs::msg::Path raw_path_msg;
        raw_path_msg.header.stamp = node_->now();
        raw_path_msg.header.frame_id = "map";
        nav_msgs::msg::Path opt_path_msg;
        opt_path_msg.header.stamp = node_->now();
        opt_path_msg.header.frame_id = "map";
        RCLCPP_INFO_STREAM(
            node_->get_logger(),
            "Start: " << start_w.transpose() << " Goal: " << goal_w.transpose()
        );
        using Clock = std::chrono::system_clock;
        using Ms = std::chrono::milliseconds;
        auto t0_total = Clock::now();
        auto t1_search_start = Clock::now();
        PathSearch::Path path;
        SearchState search_state = SearchState::NO_PATH;
        try {
            search_state = path_search_->search(start_w.cast<float>(), goal_w, path);
        } catch (std::exception& e) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
        }
        auto t1_search_end = Clock::now();
        if (search_state == SearchState::SUCCESS) {
            auto t2_sampleopt_start = Clock::now();
            current_raw_path_ = path;
            resampleAndOpt(current_raw_path_);
            auto t2_sampleopt_end = Clock::now();

            auto t_end_total = Clock::now();
            RCLCPP_INFO_STREAM(
                node_->get_logger(),
                "Planning Time (ms) | A*: "
                    << std::chrono::duration_cast<Ms>(t1_search_end - t1_search_start).count()
                    << " | Resample and Optimize: "
                    << std::chrono::duration_cast<Ms>(t2_sampleopt_start - t2_sampleopt_end).count()
                    << " | Total: "
                    << std::chrono::duration_cast<Ms>(t_end_total - t0_total).count()
            );

        } else if (search_state == SearchState::NO_PATH) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "No path found");
            replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
        } else if (search_state == SearchState::TIMEOUT) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Search timeout, stop this goal plan");
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
            current_raw_path_,
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
        if (raw_path_pub_->get_subscription_count() > 0) {
            for (auto& point: traj) {
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
        if (opt_path_pub_->get_subscription_count() > 0) {
            for (auto& point: opt_traj) {
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
        replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
        current_goal_ = *msg;
    }

    void goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = msg->point.x;
        pose.pose.position.y = msg->point.y;
        pose.pose.position.z = msg->point.z;

        // 方向信息不存在，设为默认无旋转
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        replan_fsm_.state_ = ReplanFSM::SEARCH_PATH;
        current_goal_ = pose;
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

} // namespace rose_planner
