#pragma once
#include <Eigen/Dense>
#include <rclcpp/node.hpp>
namespace rose_planner {

class Parameters {
public:
    void load(rclcpp::Node& node) {
        path_search_params_.load(node);
        opt_params.load(node);
        mpc_params.load(node);
        std::vector<double> robot_size_vec =
            node.declare_parameter("robot_size", std::vector<double> { 0.5, 0.5 });
        robot_size = Eigen::Vector2f(robot_size_vec[0], robot_size_vec[1]);
        robot_radius =
            0.5f * std::sqrt(robot_size.x() * robot_size.x() + robot_size.y() * robot_size.y());
    }
    Eigen::Vector2f robot_size = Eigen::Vector2f(0.5f, 0.5f);
    double robot_radius;
    struct OptParams {
        double smooth_weight = 1.0;
        double obstacle_weight = 1.0;
        double time_weight = 1.0;
        double base_scale = 1.0;
        void load(rclcpp::Node& node) {
            smooth_weight =
                node.declare_parameter<double>("trajectory_opt.smooth_weight", smooth_weight);
            obstacle_weight =
                node.declare_parameter<double>("trajectory_opt.obstacle_weight", obstacle_weight);
            time_weight = node.declare_parameter<double>("trajectory_opt.time_weight", time_weight);
            base_scale = node.declare_parameter<double>("trajectory_opt.base_scale", base_scale);
        }
    } opt_params;
    struct MpcParams {
        double dt = 0.1;
        int max_iter = 100;
        int predict_steps = 30;
        double max_speed = 2.0;
        double min_speed = 0.5;
        double max_accel = 1.0;
        int delay_num = 1;
        std::vector<double> Q = { 15.0, 15.0, 0.5, 0.5 };

        // 控制变化正则/参考权重 [vx_cmd, vy_cmd]
        std::vector<double> Rd = { 1.0, 0.05 };

        // 控制输入正则权重（不能为0，否则QP不稳定）
        std::vector<double> R = { 0.1, 0.1 };
        void load(rclcpp::Node& node) {
            dt = node.declare_parameter<double>("mpc_control.dt", dt);
            max_iter = node.declare_parameter<int>("mpc_control.max_iter", max_iter);
            predict_steps = node.declare_parameter<int>("mpc_control.predict_steps", predict_steps);
            max_speed = node.declare_parameter<double>("mpc_control.max_speed", max_speed);
            min_speed = node.declare_parameter<double>("mpc_control.min_speed", min_speed);
            max_accel = node.declare_parameter<double>("mpc_control.max_accel", max_accel);
            delay_num = node.declare_parameter<int>("mpc_control.delay_num", delay_num);
            Q = node.declare_parameter<std::vector<double>>("mpc_control.Q", Q);
            Rd = node.declare_parameter<std::vector<double>>("mpc_control.Rd", Rd);
            R = node.declare_parameter<std::vector<double>>("mpc_control.R", R);
        }
    } mpc_params;
    struct PathSearchParams {
        struct AStar {
            float clearance_weight = 5.0f;
            float obstacle_penalty_weight = 20.0f;
            float heuristic_weight = 1.0f;
            void load(rclcpp::Node& node) {
                clearance_weight = node.declare_parameter<float>(
                    "path_search.a_star.clearance_weight",
                    clearance_weight
                );
                obstacle_penalty_weight = node.declare_parameter<float>(
                    "path_search.a_star.obstacle_penalty_weight",
                    obstacle_penalty_weight
                );
                heuristic_weight = node.declare_parameter<float>(
                    "path_search.a_star.heuristic_weight",
                    heuristic_weight
                );
            }
        } a_star;

        struct ReSampler {
            float max_vel = 3.0;
            float acc = 2.0;
            float dt = 0.1;
            void load(rclcpp::Node& node) {
                max_vel = node.declare_parameter<float>("path_search.resampler.max_vel", max_vel);
                acc = node.declare_parameter<float>("path_search.resampler.acc", acc);
                dt = node.declare_parameter<float>("path_search.resampler.dt", dt);
            }
        } resampler;
        void load(rclcpp::Node& node) {
            a_star.load(node);
            resampler.load(node);
        }
    } path_search_params_;
};
} // namespace rose_planner