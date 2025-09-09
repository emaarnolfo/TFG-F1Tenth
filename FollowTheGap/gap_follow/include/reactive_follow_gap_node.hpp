#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

const float truncation_angle_coverage = M_PI; // max steering angle in radians 

class ReactiveFollowGapNode : public rclcpp::Node
{
public:
    ReactiveFollowGapNode();

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void publish_drive_msg(double steering_angle);
    double get_steering_angle_from_range_index(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, const size_t best_point_index, const double closest_value);
    //void apply_bubble(std::vector<float>& ranges, int closest_index, float angle_min, float angle_increment, float bubble_radius);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    bool truncated;
    std::map<std::string, double> error_based_velocities_;
    int smoothing_filter_size;
    int truncated_start_index, truncated_end_index;
    int max_jump;
    float smoothing_steering_factor_;
    float bubble_radius;
};
