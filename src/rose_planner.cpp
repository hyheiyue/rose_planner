#include "rose_planner.hpp"
#include "control/osqp_mpc.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "parameters.hpp"
#include "path_search/a*.hpp"
#include "planner_manager.hpp"
#include "rose_map/rose_map.hpp"
#include <angles.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <sentry_interfase/msg/detail/nav_state__struct.hpp>
#include <sentry_interfase/msg/nav_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
//clang-format off
#include "control/acado_mpc.hpp"
//clang-format on
#include <std_srvs/srv/trigger.hpp>
namespace rose_planner {
struct RosePlanner::Impl {
public:
    using SearchType = AStar;
    using MpcType = control::OsqpMpc;
    double default_wz = 0.0;
    ~Impl() {
        if (control_timer_thread_.joinable())
            control_timer_thread_.join();
    }
    enum class GoalMode : u_int {
        SINGALE = 0,
        MULTI = 1,
    } goal_mode_ = GoalMode::SINGALE;

    Impl(rclcpp::Node& node) {
        node_ = &node;
        tf_ = TF::create(node);
        params_.load(node);
        robo_ = Robo::create(node);
        rose_map_ = std::make_shared<rose_map::RoseMap>(node);
        planner_manager_ = PlannerManager::create(node, params_, robo_, rose_map_);
        mpc_ = MpcType::create(rose_map_, params_);
        default_wz = node.declare_parameter<double>("default_wz", 0.0);
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
        nav_state_pub_ = node.create_publisher<sentry_interfase::msg::NavState>("rose_state", 10);
        raw_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("raw_path", 10);
        opt_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("opt_path", 10);
        predict_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("predict_path", 10);
        cmd_vel_pub_ = node.create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        cmd_vel_norm_pub_ = node.create_publisher<std_msgs::msg::Float64>("/cmd_vel_norm", 10);
        vel_marker_pub_ = node.create_publisher<visualization_msgs::msg::Marker>("/vel_marker", 10);
        opt_marker_pub_ =
            node.create_publisher<visualization_msgs::msg::Marker>("/opt_traj_marker", 10);
        goal_mode_trigger_ = node_->create_service<std_srvs::srv::Trigger>(
            "change_goal_mode",
            std::bind(
                &RosePlanner::Impl::changeGoalMode,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        multi_goal_commit_trigger_ = node_->create_service<std_srvs::srv::Trigger>(
            "multi_goal_commit",
            std::bind(
                &RosePlanner::Impl::commitMulitGoal,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        control_timer_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                auto start = std::chrono::steady_clock::now();
                mpcCallback();
                auto end = std::chrono::steady_clock::now();
                auto cost =
                    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                if (cost < 1000 / params_.mpc_params.fps)
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(1000 / params_.mpc_params.fps - cost)
                    );
            }
        });
        use_control_output_ = node.declare_parameter<bool>("use_control_output", false);
        RCLCPP_INFO(node_->get_logger(), "RosePlanner initialized");
    }
    void commitMulitGoal(
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr /*res*/
    ) {
        if (goal_mode_ == GoalMode::MULTI) {
            if (!goal_.pos.empty()) {
                RCLCPP_INFO(node_->get_logger(), "multi goal commit");
                planner_manager_->setNewGoal(goal_);
                goal_.pos.clear();
            }
        }
    }
    void changeGoalMode(
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr /*res*/
    ) {
        switch (goal_mode_) {
            case GoalMode::SINGALE: {
                goal_mode_ = GoalMode::MULTI;
                RCLCPP_INFO(node_->get_logger(), "Switching to mult goal mode");
                break;
            }

            case GoalMode::MULTI: {
                goal_mode_ = GoalMode::SINGALE;
                RCLCPP_INFO(node_->get_logger(), "Switching to single goal mode");
                break;
            }
        }
    }
    void mpcCallback() noexcept {
        auto current_traj = planner_manager_->getTrajectory();
        static auto last_state = PlannerManager::FSMSTATE::INIT;
        if (planner_manager_->getState() != PlannerManager::FSMSTATE::REPLAN && use_control_output_)
        {
            static rclcpp::Time replan_start_time;
            static bool timer_active = false;

            auto now = node_->get_clock()->now();
            if (last_state == PlannerManager::FSMSTATE::REPLAN) {
                replan_start_time = now;
                timer_active = true;
            }

            last_state = planner_manager_->getState();

            if (timer_active) {
                if ((now - replan_start_time).seconds() < 1.0) {
                    if (use_control_output_) {
                        geometry_msgs::msg::Twist cmd;
                        cmd.linear.x = 0.0;
                        cmd.linear.y = 0.0;
                        cmd.angular.z = 0.0;
                        cmd_vel_pub_->publish(cmd);
                    }

                } else {
                    timer_active = false;
                }
            }
            return;
        }
        if (!current_traj.has_value()) {
            if (use_control_output_) {
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.angular.z = default_wz;
                cmd_vel_pub_->publish(cmd);
            }
            last_state = planner_manager_->getState();
            return;
        }
        last_state = planner_manager_->getState();
        mpc_->setTrajectory(current_traj.value());

        auto now_state = robo_->getNowState();
        mpc_->setCurrent(now_state);
        if (mpc_->solve()) {
            mpc_->solve();
            auto output = mpc_->getOutput();
            if (use_control_output_) {
                double yaw = now_state.yaw;
                double vx_world = output.vel.x();
                double vy_world = output.vel.y();
                double vx_body = std::cos(yaw) * vx_world + std::sin(yaw) * vy_world;
                double vy_body = -std::sin(yaw) * vx_world + std::cos(yaw) * vy_world;
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = vx_body;
                cmd.linear.y = vy_body;

                cmd.angular.z = default_wz;
                cmd_vel_pub_->publish(cmd);
                std_msgs::msg::Float64 cmd_norm;
                cmd_norm.data = std::hypot(vx_world, vy_world);
                cmd_vel_norm_pub_->publish(cmd_norm);
            }
            visualization_msgs::msg::Marker vel_marker;

            vel_marker.header.frame_id = params_.target_frame;
            vel_marker.header.stamp = node_->get_clock()->now();
            output.fillVelocityArrow(vel_marker, now_state);

            vel_marker_pub_->publish(vel_marker);

            nav_msgs::msg::Path predict_path;
            predict_path.header.frame_id = params_.target_frame;
            predict_path.header.stamp = rclcpp::Clock().now();
            output.fillPath(predict_path, now_state);
            predict_path_pub_->publish(predict_path);
        }
        sentry_interfase::msg::NavState nav_state;
        nav_state.pose_in_map.position.x = now_state.pos.x();
        nav_state.pose_in_map.position.z = 0.0;
        nav_state.pose_in_map.position.y = now_state.pos.y();
        bool reached = last_state == PlannerManager::FSMSTATE::WAIT_GOAL;
        nav_state.reached_goal = reached;
        nav_state_pub_->publish(nav_state);
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto T = tf_->getTransform(params_.target_frame, msg->header.frame_id, msg->header.stamp);
        if (!T.has_value()) {
            return;
        }
        Eigen::Vector4f p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, 1.0f);
        p = T.value() * p;
        setSingleGoal(p.cast<double>().head<2>());
    }

    void goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        auto T = tf_->getTransform(params_.target_frame, msg->header.frame_id, msg->header.stamp);
        if (!T.has_value()) {
            return;
        }
        Eigen::Vector4f p(msg->point.x, msg->point.y, msg->point.z, 1.0f);
        p = T.value() * p;
        setSingleGoal(p.cast<double>().head<2>());
    }
    Goal goal_;
    void setSingleGoal(const Eigen::Vector2d& p) {
        switch (goal_mode_) {
            case GoalMode::SINGALE: {
                goal_.pos.push_back(p);
                planner_manager_->setNewGoal(goal_);
                goal_.pos.clear();
                break;
            }

            case GoalMode::MULTI: {
                goal_.pos.push_back(p);
                break;
            }
        }
    }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rose_map_->odomCallback(msg);

        const auto& odom_in = *msg;

        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();

        auto T_opt =
            tf_->getTransform(params_.target_frame, odom_in.header.frame_id, odom_in.header.stamp);

        if (T_opt.has_value()) {
            T = *T_opt;
        } else {
            RCLCPP_WARN(node_->get_logger(), "[TF] odomCallback transform failed → identity");
            return;
        }

        Eigen::Vector3f p(
            odom_in.pose.pose.position.x,
            odom_in.pose.pose.position.y,
            odom_in.pose.pose.position.z
        );

        p = T * p;
        Eigen::Quaternionf q_in(
            odom_in.pose.pose.orientation.w,
            odom_in.pose.pose.orientation.x,
            odom_in.pose.pose.orientation.y,
            odom_in.pose.pose.orientation.z
        );

        Eigen::Quaternionf q_out(T.rotation() * q_in);
        q_out.normalize();
        geometry_msgs::msg::PoseStamped pose_out;
        pose_out.header = odom_in.header;
        pose_out.header.frame_id = params_.target_frame;

        pose_out.pose.position.x = p.x();
        pose_out.pose.position.y = p.y();
        pose_out.pose.position.z = p.z();
        pose_out.pose.orientation.x = q_out.x();
        pose_out.pose.orientation.y = q_out.y();
        pose_out.pose.orientation.z = q_out.z();
        pose_out.pose.orientation.w = q_out.w();
        Eigen::Matrix3f R = T.rotation();

        Eigen::Vector3f vlin(
            odom_in.twist.twist.linear.x,
            odom_in.twist.twist.linear.y,
            odom_in.twist.twist.linear.z
        );

        Eigen::Vector3f vang(
            odom_in.twist.twist.angular.x,
            odom_in.twist.twist.angular.y,
            odom_in.twist.twist.angular.z
        );

        vlin = R * vlin;
        vang = R * vang;

        nav_msgs::msg::Odometry odom_out = odom_in;
        odom_out.header.frame_id = params_.target_frame;
        odom_out.pose.pose = pose_out.pose;

        odom_out.twist.twist.linear.x = vlin.x();
        odom_out.twist.twist.linear.y = vlin.y();
        odom_out.twist.twist.linear.z = vlin.z();

        odom_out.twist.twist.angular.x = vang.x();
        odom_out.twist.twist.angular.y = vang.y();
        odom_out.twist.twist.angular.z = vang.z();

        robo_->setCurrentOdom(odom_out);
    }

    Parameters params_;
    rclcpp::Node* node_;
    TF::Ptr tf_;
    rose_map::RoseMap::Ptr rose_map_;
    PlannerManager::Ptr planner_manager_;
    Robo::Ptr robo_;
    bool use_control_output_ = false;
    std::thread control_timer_thread_;
    MpcType::Ptr mpc_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_vel_norm_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr opt_marker_pub_;
    rclcpp::Publisher<sentry_interfase::msg::NavState>::SharedPtr nav_state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goal_mode_trigger_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr multi_goal_commit_trigger_;
};

RosePlanner::RosePlanner(rclcpp::Node& node) {
    _impl = std::make_unique<Impl>(node);
}
RosePlanner::~RosePlanner() {
    _impl.reset();
}
} // namespace rose_planner