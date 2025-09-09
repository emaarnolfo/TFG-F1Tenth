#include "reactive_follow_gap_node.hpp"
#include "utility.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ReactiveFollowGapNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
} 