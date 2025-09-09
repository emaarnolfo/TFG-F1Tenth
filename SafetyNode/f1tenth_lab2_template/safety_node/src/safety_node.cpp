#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>


class Safety : public rclcpp::Node {
// The class that handles emergency braking 

public:
    Safety() : Node("safety_node")
    {
        // Suscriptor al LiDAR
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        // Suscriptor a la odometría
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&Safety::odom_callback, this, std::placeholders::_1));

        // Publicador para frenar el coche
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
           "/drive", 10);

        RCLCPP_INFO(this->get_logger(), "Nodo SafetyNode iniciado.");        
    }

private:
    double current_speed;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
    

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        double vehicle_speed = current_speed;
        double min_distance = scan_msg->range_max;
        double min_tcc = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
            // Calcula el angulo para la medición actual
            float angle = scan_msg->angle_min + i*scan_msg->angle_increment;
            
            // Corrige el ángulo para que sea respecto al eje x
            float corrected_angle = angle - M_PI/2;
            
            // Filtra el sector de interés de -30 a 30 grados
            if(corrected_angle >= -0.5236 && corrected_angle <= 0.5236){
                double range = scan_msg->ranges[i];

                // Calcula la velocidad en la direccion de la medicion
                double effective_speed = vehicle_speed * cos(corrected_angle);

                // Verifica que la velocidad efectiva sea suficiente para calcular el TTC
                if(effective_speed > 0.2){
                    double ttc = (range / effective_speed);
                    if(ttc < min_tcc){
                        min_tcc = ttc;
                    }
                }
            }
        }

        double ttc_threshold = std::max(0.2, 2.0 / (1 + vehicle_speed));

        RCLCPP_INFO(this->get_logger(), "TTC mínimo calculado: %.2f", min_tcc);

        if(min_tcc < ttc_threshold){
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = 0.0;
            this->drive_pub->publish(drive_msg);
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        current_speed = msg->twist.twist.linear.x;
    }


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
