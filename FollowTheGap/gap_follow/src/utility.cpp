#include "utility.hpp"
#include <cmath>
#include <algorithm>

namespace utils
{
    void guardar_en_archivo(const std::vector<float>& datos, const std::string& nombre_archivo)
    {
        std::ofstream archivo(nombre_archivo, std::ios::app); // modo append (no sobreescribe)
        if (!archivo.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("guardar_en_archivo"), "No se pudo abrir el archivo para escribir.");
            return;
        }

        for (const auto& valor : datos)
        {
            archivo << valor << " ";
        }
        archivo << std::endl; // salto de línea después del array

        archivo.close();
    }

    std::pair<int, int> find_truncated_indices(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, float truncation_angle_coverage)
    {
        int start_index = 0;
        int end_index = 0;
        
        int truncated_range_size = static_cast<int>(truncation_angle_coverage / scan_msg->angle_increment);
        start_index = (scan_msg->ranges.size()/2) - (truncated_range_size/2);
        end_index = (scan_msg->ranges.size()/2) + (truncated_range_size/2);
        
        return std::make_pair(start_index, end_index);
    }
}