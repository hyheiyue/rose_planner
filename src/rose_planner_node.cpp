#include "rclcpp/rclcpp.hpp"
#include "rose_planner.hpp"
namespace rose_planner {
class RosePlannerNode: public rclcpp::Node {
public:
    RosePlannerNode(const rclcpp::NodeOptions& options): Node("rose_planner_node", options) {
        rose_planner = RosePlanner::create(*this);
    }
    RosePlanner::Ptr rose_planner;
};
} // namespace rose_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rose_planner::RosePlannerNode)