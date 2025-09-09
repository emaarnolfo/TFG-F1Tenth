#include "lidar_processing.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lp
{
    /**
    * Method to preprocess the LiDAR scan array. Expert implementation includes:
    * 1. Setting each value to the mean over a window of 5 values
    * 2. Rejecting high values
    */
    std::vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, 
                                        int truncated_start_index, 
                                        int truncated_end_index,
                                        int smoothing_filter_size)
    {   
        std::vector<float> ranges_filtered;
        for(int i = truncated_start_index; i < truncated_end_index; i++)
        {
            float sum = 0.0;
            for(int j = -smoothing_filter_size; j <= smoothing_filter_size; j++)
            {
                sum += scan_msg->ranges[i + j];
            }
            float mean = sum / (2 * smoothing_filter_size + 1);
            ranges_filtered.push_back(mean);
        }
        
        return ranges_filtered;
    }

    int find_closest_point(std::vector<float>& ranges)
    {   
        int index = 0;
        // Find the closest point in ranges
        for(int i = 0; i < ranges.size(); i++)
        {
            if(ranges[i] < ranges[index])
                index = i;
        }
        return index;
    }

    void apply_bubble(std::vector<float>& ranges, int closest_index, float angle_min, float angle_increment, float bubble_radius)
    {   
        float r = ranges[closest_index];

        // Si el punta está demasiado cerca, burbuja maxima
        if(r < bubble_radius)
            r = bubble_radius;

        float theta = std::asin(bubble_radius / r);
        int n = static_cast<int>(theta / angle_increment);

        int start = std::max(0, closest_index - n);
        int end = std::min((int)ranges.size() - 1, closest_index + n);

        for (int i = start; i < end; i++)
        {
            ranges[i] = 0.0f;
        }
        //RCLCPP_INFO(this->get_logger(), "Burbuja aplicada a %d indices", end - start);
        return;
    }

    void zero_out_safety_bubble(std::vector<float>* input_vector, const size_t center_index, const double bubble_indices)    
    {
        const double center_point_distance =  input_vector->at(center_index);
        input_vector->at(center_index) = 0.0;

        for (size_t i = center_index - bubble_indices; i < center_index + bubble_indices; i++)
        {
            if(i < 0 || i >= input_vector->size())
                continue;
            input_vector->at(i) = 0.0;
        }
    }
}