#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

namespace gp
{
    void find_max_gap(const std::vector<float>& ranges, int& start_i, int& end_i);
    void find_best_point(const std::vector<float>& ranges, int& start_i, int& end_i, int& best_index, int max_jump);
    int get_best_point(const std::vector<float>& ranges, int start_i, int end_i);
    
} // namespace gp
