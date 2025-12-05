#include <rclcpp/rclcpp.hpp>
#include "k100_motion_planning/k100_motion_planning.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<k100_motion_planning::K100MotionPlanningNode>();
    
    if (node->initialize()) {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    
    rclcpp::shutdown();
    return 0;
}
