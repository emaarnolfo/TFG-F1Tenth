#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


namespace lp
{
    std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, int truncated_start_index, int truncated_end_index, int smoothing_filter_size);
    int find_closest_point(std::vector<float>& ranges);
    void apply_bubble(std::vector<float>& ranges, int closest_index, float angle_min, float angle_increment, float bubble_radius);
    void zero_out_safety_bubble(std::vector<float>* input_vector, const size_t center_index, const double bubble_indices);

} // namespace lp
