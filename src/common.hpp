#pragma once
#include "angles.h"
#include "rose_map/rose_map.hpp"
#include "trajectory_optimize/trajectory.hpp"
#include <Eigen/Dense>
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace rose_planner {
template<class Tag>
inline double orientationToYaw(const geometry_msgs::msg::Quaternion& q) noexcept {
    // Get armor yaw
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    // Make yaw change continuous (-pi~pi to -inf~inf)
    static double last_yaw_ = 0.0;
    yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;
    return yaw;
}
class RoboState {
public:
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    double yaw;
};
class Robo {
public:
    using Ptr = std::shared_ptr<Robo>;
    Robo(rclcpp::Node& node) {
        node_ = &node;
    }
    static Ptr create(rclcpp::Node& node) {
        return std::make_shared<Robo>(node);
    }
    void setCurrentOdom(const nav_msgs::msg::Odometry& current_odom) {
        current_odom_ = current_odom;
    }
    RoboState getNowState() noexcept {
        RoboState now_state;
        auto current = getCurrentPose();
        now_state.pos.x() = current.pose.position.x;
        now_state.pos.y() = current.pose.position.y;
        auto vel = getCurrentVel();
        now_state.vel = vel;
        struct Tag {};
        double now_yaw = orientationToYaw<Tag>(current.pose.orientation);
        now_state.yaw = now_yaw;
        return now_state;
    }
    Eigen::Vector2d getCurrentVel() const noexcept {
        Eigen::Vector2d v(current_odom_.twist.twist.linear.x, current_odom_.twist.twist.linear.y);
        return v;
    }
    geometry_msgs::msg::PoseStamped getCurrentPose() const noexcept {
        geometry_msgs::msg::PoseStamped predicted_pose;
        predicted_pose.header.stamp = node_->get_clock()->now();
        predicted_pose.header.frame_id = current_odom_.header.frame_id;

        double px = current_odom_.pose.pose.position.x;
        double py = current_odom_.pose.pose.position.y;
        double pz = current_odom_.pose.pose.position.z;

        Eigen::Quaterniond q(
            current_odom_.pose.pose.orientation.w,
            current_odom_.pose.pose.orientation.x,
            current_odom_.pose.pose.orientation.y,
            current_odom_.pose.pose.orientation.z
        );

        Eigen::Vector3d v(
            current_odom_.twist.twist.linear.x,
            current_odom_.twist.twist.linear.y,
            current_odom_.twist.twist.linear.z
        );

        Eigen::Vector3d w(
            current_odom_.twist.twist.angular.x,
            current_odom_.twist.twist.angular.y,
            current_odom_.twist.twist.angular.z
        );

        static rclcpp::Time last_time = rclcpp::Time(current_odom_.header.stamp, RCL_ROS_TIME);
        double dt = (node_->now() - last_time).seconds();
        last_time = node_->now();
        Eigen::Quaterniond dq = Eigen::Quaterniond(Eigen::AngleAxisd(
            dt * w.norm(),
            (w.norm() > 1e-6) ? w.normalized() : Eigen::Vector3d::UnitZ()
        ));

        Eigen::Quaterniond q_pred = q * dq;
        q_pred.normalize();
        Eigen::Vector3d p_pred(px, py, pz);
        p_pred += v * dt;
        predicted_pose.pose.position.x = p_pred.x();
        predicted_pose.pose.position.y = p_pred.y();
        predicted_pose.pose.position.z = p_pred.z();

        predicted_pose.pose.orientation.w = q_pred.w();
        predicted_pose.pose.orientation.x = q_pred.x();
        predicted_pose.pose.orientation.y = q_pred.y();
        predicted_pose.pose.orientation.z = q_pred.z();

        return predicted_pose;
    }
    rclcpp::Node* node_;
    nav_msgs::msg::Odometry current_odom_;
};
using TrajType = Trajectory<5, 2>;
struct Goal {
    std::deque<Eigen::Vector2d> pos;
};
static inline bool smoothedL1(const double& x, const double& mu, double& f, double& df) {
    if (x < 0.0) {
        return false;
    } else if (x > mu) {
        f = x - 0.5 * mu;
        df = 1.0;
        return true;
    } else {
        const double xdmu = x / mu;
        const double sqrxdmu = xdmu * xdmu;
        const double mumxd2 = mu - 0.5 * x;
        f = mumxd2 * sqrxdmu * xdmu;
        df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
        return true;
    }
}
static inline double kahanSum(double& sum, double& c, const double& val) {
    double y = val - c;
    double t = sum + y;
    c = (t - sum) - y;
    sum = t;
    return sum;
}

} // namespace rose_planner