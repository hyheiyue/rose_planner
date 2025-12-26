#pragma once
#include <Eigen/Dense>
#include <rclcpp/node.hpp>
namespace rose_planner {

class Parameters {
public:
    void load(rclcpp::Node& node) {
        path_search_params_.load(node);
    }
    struct PathSearchParams {
        Eigen::Vector2f robot_size = Eigen::Vector2f(0.5f, 0.5f);
        struct AStar {
            float clearance_weight = 5.0f;
            float obstacle_penalty_weight = 20.0f;
            float heuristic_weight = 1.0f;
            void load(rclcpp::Node& node) {
                node.get_parameter_or<float>(
                    "path_search.a_star.clearance_weight",
                    clearance_weight,
                    clearance_weight
                );
                node.get_parameter_or<float>(
                    "path_search.a_star.obstacle_penalty_weight",
                    obstacle_penalty_weight,
                    obstacle_penalty_weight
                );
            }
        } a_star;

        void load(rclcpp::Node& node) {
            a_star.load(node);
            std::vector<double> robot_size_vec =
                node.declare_parameter("robot_size", std::vector<double> { 0.5, 0.5 });
            robot_size = Eigen::Vector2f(robot_size_vec[0], robot_size_vec[1]);
        }
    } path_search_params_;
};
} // namespace rose_planner