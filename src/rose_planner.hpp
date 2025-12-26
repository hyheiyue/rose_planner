#pragma once
#include "a*.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jps.hpp"
#include "parameters.hpp"
#include "path_search.hpp"
#include "replan_fsm.hpp"
#include "rose_map/rose_map.hpp"
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
                searchOnce(goal_pos);
                break;
            }

            case ReplanFSM::SEARCH_PATH: {
                Eigen::Vector2f goal_w(
                    current_goal_.pose.position.x,
                    current_goal_.pose.position.y
                );
                searchOnce(goal_w);
                replan_fsm_.state_ = ReplanFSM::REPLAN;
                break;
            }
        }
    }

    void searchOnce(const Eigen::Vector2f& goal_w) {
        Eigen::Vector2f start_w(current_pose_.position.x, current_pose_.position.y);

        nav_msgs::msg::Path raw_path_msg;
        raw_path_msg.header.stamp = node_->now();
        raw_path_msg.header.frame_id = "map";

        RCLCPP_INFO_STREAM(
            node_->get_logger(),
            "Start: " << start_w.transpose() << " Goal: " << goal_w.transpose()
        );

        auto start_time = std::chrono::system_clock::now();
        PathSearch::Path path;
        bool success = false;
        try {
            success = path_search_->search(start_w, goal_w, path);
        } catch (std::exception& e) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
        }
        if (success) {
            auto end_time = std::chrono::system_clock::now();
            RCLCPP_INFO_STREAM(
                node_->get_logger(),
                "Path found: " << path.size() << " in "
                               << std::chrono::duration_cast<std::chrono::milliseconds>(
                                      end_time - start_time
                                  )
                                      .count()
                               << " ms"
            );

            if (raw_path_pub_->get_subscription_count() > 0) {
                for (auto& point: path) {
                    geometry_msgs::msg::PoseStamped pose_msg;
                    pose_msg.header = raw_path_msg.header;
                    pose_msg.pose.position.x = point.x();
                    pose_msg.pose.position.y = point.y();
                    pose_msg.pose.position.z = current_goal_.pose.position.z;
                    raw_path_msg.poses.push_back(pose_msg);
                }
                raw_path_pub_->publish(raw_path_msg);
            }
        } else {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "No path found");
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
    }

private:
    Parameters parameters_;
    rclcpp::Node* node_;
    SearchType::Ptr path_search_;
    rose_map::RoseMap::Ptr rose_map_;

    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::PoseStamped current_goal_;
    ReplanFSM replan_fsm_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
};

} // namespace rose_planner
