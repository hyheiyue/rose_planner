#pragma once

#include "common.hpp"
#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
namespace rose_planner {
namespace control {

    class TrajPoint {
    public:
        Eigen::Vector2d pos;
        Eigen::Vector2d vel;
        Eigen::Vector2d acc;
        double yaw;
        double w;
    };
    class Output {
    public:
        Eigen::Vector2d vel;
        double w;
        std::vector<RoboState> pred_states;
        void fillPath(nav_msgs::msg::Path& predict_path, const RoboState& current) const {
            geometry_msgs::msg::PoseStamped pose_msg;
            for (int i = 0; i < pred_states.size(); ++i) {
                pose_msg.header = predict_path.header;
                pose_msg.pose.position.x = pred_states[i].pos.x();
                pose_msg.pose.position.y = pred_states[i].pos.y();
                pose_msg.pose.position.z = 0.0;
                double yaw = std::hypot(pred_states[i].vel.x(), pred_states[i].vel.y()) > 1e-3
                    ? std::atan2(pred_states[i].vel.y(), pred_states[i].vel.x())
                    : 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                q.normalize();
                pose_msg.pose.orientation = tf2::toMsg(q);
                predict_path.poses.push_back(pose_msg);
            }
        }
        void
        fillVelocityArrow(visualization_msgs::msg::Marker& marker, const RoboState& current) const {
            marker.ns = "mpc_velocity";
            marker.id = 0;

            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            auto pos = current.pos.cast<float>();
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = pos.x();
            p_start.y = pos.y();
            p_start.z = 0.0;
            double scale = 3.0;
            auto vel_normalized = vel.normalized();
            p_end.x = pos.x() + scale * vel_normalized.x();
            p_end.y = pos.y() + scale * vel_normalized.y();
            p_end.z = 0.0;

            marker.points.push_back(p_start);
            marker.points.push_back(p_end);

            marker.scale.x = 0.1; // shaft diameter
            marker.scale.y = 0.2; // head diameter
            marker.scale.z = 0.3; // head length

            marker.color.r = 0.9f;
            marker.color.g = 0.1f;
            marker.color.b = 0.1f;
            marker.color.a = 1.0f;

            marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        }
    };
} // namespace control

} // namespace rose_planner