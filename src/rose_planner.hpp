#pragma once

#include <memory>
namespace rclcpp {
class Node;
}
namespace rose_planner {
class RosePlanner {
public:
    using Ptr = std::unique_ptr<RosePlanner>;
    RosePlanner(rclcpp::Node& node);
    ~RosePlanner();
    static Ptr create(rclcpp::Node& node) {
        return std::make_unique<RosePlanner>(node);
    }
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace rose_planner