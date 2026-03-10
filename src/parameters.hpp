#pragma once
#include "rose_map/utils/rclcpp_parameter_helper.hpp"
#include <Eigen/Dense>
#include <cmath>
namespace rose_planner {

class Parameters {
public:
    void load(rclcpp::Node& node) {
        ParamHelper root(node);

        path_search_params_.load(root.sub("path_search"));
        opt_params.load(root.sub("trajectory_opt"));
        mpc_params.load(root.sub("mpc_control"));

        robot_size = root.getEigenVector<2, float>("robot_size");
        target_frame = root.declare<std::string>("target_frame");
        robot_radius =
            0.5f * std::sqrt(robot_size.x() * robot_size.x() + robot_size.y() * robot_size.y());
    }
    std::string target_frame;

    Eigen::Vector2f robot_size = Eigen::Vector2f(0.5f, 0.5f);
    double robot_radius { 0.0 };

    struct OptParams {
        double smooth_weight = 1.0;
        double obstacle_weight = 1.0;
        double sample_ds = 0.3;
        double expected_speed = 2.5;
        void load(const ParamHelper& ph) {
            smooth_weight = ph.declare<double>("smooth_weight");
            obstacle_weight = ph.declare("obstacle_weight", obstacle_weight);
            expected_speed = ph.declare("expected_speed", expected_speed);
            sample_ds = ph.declare("sample_ds", sample_ds);
        }
    } opt_params;

    struct MpcParams {
        int fps = 100;
        double dt = 0.1;
        int max_iter = 100;
        int predict_steps = 30;
        double max_speed = 2.0;
        double max_accel = 1.0;
        double delay_time = 0.0;
        double blind_radius;
        std::vector<double> Q = { 15.0, 15.0, 0.5, 0.5 };
        std::vector<double> Rd = { 1.0, 0.05 };
        std::vector<double> R = { 0.1, 0.1 };

        void load(const ParamHelper& ph) {
            fps = ph.declare<int>("fps");
            dt = ph.declare<double>("dt");
            max_iter = ph.declare<int>("max_iter");
            predict_steps = ph.declare<int>("predict_steps");
            max_speed = ph.declare<double>("max_speed");
            max_accel = ph.declare<double>("max_accel");
            delay_time = ph.declare<double>("delay_time");
            blind_radius = ph.declare<double>("blind_radius");
            Q = ph.declare<std::vector<double>>("Q");
            Rd = ph.declare<std::vector<double>>("Rd");
            R = ph.declare<std::vector<double>>("R");
        }
    } mpc_params;

    struct PathSearchParams {
        struct AStar {
            float clearance_weight = 5.0f;
            float obstacle_penalty_weight = 20.0f;
            float heuristic_weight = 1.0f;

            void load(const ParamHelper& ph) {
                clearance_weight = ph.declare("clearance_weight", clearance_weight);
                obstacle_penalty_weight =
                    ph.declare("obstacle_penalty_weight", obstacle_penalty_weight);
                heuristic_weight = ph.declare("heuristic_weight", heuristic_weight);
            }
        } a_star;

        void load(const ParamHelper& ph) {
            a_star.load(ph.sub("a_star"));
        }
    } path_search_params_;
};

} // namespace rose_planner