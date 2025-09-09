#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "cmath"
#include <fstream>
#include <vector>
#include <algorithm>
#include "utility.hpp"
#include "reactive_follow_gap_node.hpp"
#include "lidar_processing.hpp"
#include "gap_finding.hpp"

// Librerias para publicar la velocidad
#include <vesc_msgs/msg/vesc_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

ReactiveFollowGapNode::ReactiveFollowGapNode() : Node("reactive_node"),
              truncated(false),
              error_based_velocities_({
                //   {"low", 5.0},
                  {"low", 0.8},
                //   {"medium", 2.5},
                  {"medium", 0.8},
                  {"high", 0.8}
                  // Valores minimos para pruebas iniciales.
              }),
              smoothing_filter_size(5),
              max_jump(10),
              smoothing_steering_factor_(0.6),
              bubble_radius(0.2)
        {
            lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                lidarscan_topic, 10, std::bind(&ReactiveFollowGapNode::lidar_callback, this, std::placeholders::_1));
            drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                drive_topic, 10);
            RCLCPP_INFO(this->get_logger(), "Reactive Follow Gap Node has been started");
        }
        // const float max_gap = 10.0; // maximum gap size in meters
        // const int bubble_indices = 400;

    
        double ReactiveFollowGapNode::get_steering_angle_from_range_index(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
                const size_t best_point_index,
                const double closest_value)
        {
            //const size_t best_point_index_input_scan_frame = truncated_start_index + best_point_index;
            double best_point_steering_angle;
    
            // Case When Steering Angle is to be negative
            if(best_point_index < scan_msg->ranges.size()/2)
            {
                best_point_steering_angle = - scan_msg->angle_increment*
                static_cast<double>(scan_msg->ranges.size()/2.0 - best_point_index);
            }
            // Case When Steering Angle is to be negative
            else
            {
                best_point_steering_angle = scan_msg->angle_increment*
                static_cast<double>(best_point_index - scan_msg->ranges.size()/2.0);
            }
            const auto distance_compensated_steering_angle =
            std::clamp((best_point_steering_angle*smoothing_steering_factor_)/static_cast<double>(closest_value), -1.57, 1.57);
            return distance_compensated_steering_angle;
        }
    
        void ReactiveFollowGapNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
        {   
            // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
            if(!truncated)
            {
                //RCLCPP_INFO(this->get_logger(), "Received LiDAR scan with %d points", (int)ranges.size());
                auto truncated_indices = utils::find_truncated_indices(scan_msg, truncation_angle_coverage);
                truncated_start_index = truncated_indices.first;
                truncated_end_index = truncated_indices.second;
                truncated = true;
                RCLCPP_INFO(this->get_logger(), "Truncated indices: %d, %d", truncated_start_index, truncated_end_index);
            } 
            
            // 1. Preprocess the LiDAR scan
            std::vector<float> filtered_ranges = lp::preprocess_lidar(scan_msg, truncated_start_index, truncated_end_index, smoothing_filter_size);
    
            // 2. Find the closest point
            int closest_index = lp::find_closest_point(filtered_ranges); 
            auto closest_range = filtered_ranges[closest_index];
            //RCLCPP_INFO(this->get_logger(), "Closest index: %d, distance: %.2f", closest_index, filtered_ranges[closest_index]);
    
            // 3. Apply bubble
            lp::apply_bubble(filtered_ranges, closest_index, 
                        scan_msg->angle_min, 
                        scan_msg->angle_increment, bubble_radius);
    
            // 4. Find max length gap
            int gap_start = 0, gap_end = 0;
            gp::find_max_gap(filtered_ranges, gap_start, gap_end);
            
            // 5. Find the best point in the gap
            int best_index = gap_start;
            gp::find_best_point(filtered_ranges, gap_start, gap_end, best_index, max_jump);
    
            // Obtiene el punto medio del gap
            // int best_index = gp::get_best_point(filtered_ranges, gap_start, gap_end);
    
            // 6. Calcular el ángulo global del mejor punto
            const double steering_angle = get_steering_angle_from_range_index(scan_msg, (best_index + truncated_start_index), closest_range);
            RCLCPP_INFO(this->get_logger(), "Steering angle: %.f deg", steering_angle * 180 / M_PI);
    
            // 7. Publish Drive message
            publish_drive_msg(steering_angle);
            
        }

        void ReactiveFollowGapNode::publish_drive_msg(double steering_angle)
        {
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = this->get_clock()->now();
            drive_msg.header.frame_id = "laser";
            drive_msg.drive.steering_angle = steering_angle;
            if(abs(steering_angle) > 0.349)
            {
                drive_msg.drive.speed = error_based_velocities_["high"];
            }
            else if(abs(steering_angle) > 0.174)
            {
                drive_msg.drive.speed = error_based_velocities_["medium"];
            }
            else
            {
                drive_msg.drive.speed = error_based_velocities_["low"];
            }
            drive_pub_->publish(drive_msg);
        }