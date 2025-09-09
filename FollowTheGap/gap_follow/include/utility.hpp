#pragma once

// Standard library includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <fstream>

// Namespace declaration (if needed)
namespace utils {

// Function declarations or utility definitions
//double calculateDistance(double x1, double y1, double x2, double y2);
//std::vector<double> filterData(const std::vector<double>& data, double threshold);
void guardar_en_archivo(const std::vector<float>& datos, const std::string& nombre_archivo);
std::pair<int, int> find_truncated_indices(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, float truncation_angle_coverage);


} // namespace utils
