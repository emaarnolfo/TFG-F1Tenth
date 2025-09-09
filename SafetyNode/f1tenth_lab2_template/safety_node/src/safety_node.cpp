#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

#include "std_msgs/msg/float64.hpp"

// Librerias para publicar la velocidad
#include <vesc_msgs/msg/vesc_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>


class Safety : public rclcpp::Node {
// The class that handles emergency braking 

public:
    Safety() : Node("safety_node")
    {
        // Suscriptor al LiDAR
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        // Suscriptor a la odometría
        //odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        //    "/ego_racecar/odom", 10, std::bind(&Safety::odom_callback, this, std::placeholders::_1));

        // Publicador para frenar el coche
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
           "/drive", 10);
        
        // Suscriptor para obtener la velocidad del coche
        speed_sub_ = create_subscription<std_msgs::msg::Float64>(
            "commands/motor/speed", rclcpp::QoS{10}, std::bind(&Safety::speedCallback, this, std::placeholders::_1)); 

        RCLCPP_INFO(this->get_logger(), "Nodo SafetyNode iniciado.");        
    }

private:
    float current_speed;
    /// Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
    rclcpp::SubscriptionBase::SharedPtr speed_sub_;
    

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        float vehicle_speed = current_speed / 10000;
        RCLCPP_INFO(get_logger(), "Velocidad: %.4f | Valor de calculo: %.4f\n", current_speed, vehicle_speed);
        double min_distance = scan_msg->range_max;
        double min_tcc = std::numeric_limits<double>::max();

        // RCLCPP_INFO(
        //     rclcpp::get_logger("laser_callback"),
        //     "Received LaserScan:\n"
        //     "  angle_min: %.2f\n"
        //     "  angle_max: %.2f\n"
        //     "  angle_increment: %.4f\n"
        //     "  time_increment: %.6f\n"
        //     "  scan_time: %.4f\n"
        //     "  range_min: %.2f\n"
        //     "  range_max: %.2f\n"
        //     "  ranges size: %zu\n"
        //     "  middle range: %.2f",
        //     scan_msg->angle_min,
        //     scan_msg->angle_max,
        //     scan_msg->angle_increment,
        //     scan_msg->time_increment,
        //     scan_msg->scan_time,
        //     scan_msg->range_min,
        //     scan_msg->range_max,
        //     scan_msg->ranges.size(),
        //     scan_msg->ranges.empty() ? -1.0 : scan_msg->ranges[540]
        // );

        
        for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
            // Calcula el angulo para la medición actual
            float angle = scan_msg->angle_min + i*scan_msg->angle_increment;
            
            // Corrige el ángulo para que sea respecto al eje x
            float corrected_angle = angle - M_PI/2;
            
            // Filtra el sector de interés de -15 a 15 grados
            if(corrected_angle >= -0.2618 && corrected_angle <= 0.2618){
                double range = scan_msg->ranges[i];

                //RCLCPP_INFO(this->get_logger(), "Lectura Lidar: %.d", range);

                // Calcula la velocidad en la direccion de la medicion
                double effective_speed = vehicle_speed * cos(corrected_angle);

                // Verifica que la velocidad efectiva sea suficiente para calcular el TTC
                if(effective_speed > 0.01){
                    double ttc = (range / effective_speed);
                    if(ttc < min_tcc){
                        min_tcc = ttc;
                    }
                }
            }
        }

        // double ttc_threshold = std::max(0.2, 1.0 / (1 + vehicle_speed));
        double ttc_threshold = 1; // Valor fijo para el umbral de TTC
        min_tcc = min_tcc > 100 ? 100 : min_tcc; // Limita el valor máximo de min_tcc para evitar valores muy altos

        RCLCPP_INFO(this->get_logger(), "TTC mínimo calculado: %.2f | TTc threshold: %.2f", min_tcc, ttc_threshold);

        if(min_tcc < ttc_threshold){
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = 0.0;
            this->drive_pub->publish(drive_msg);
        }
    }

    void speedCallback(const std_msgs::msg::Float64::SharedPtr speed)
    {
        current_speed = speed->data;
        //RCLCPP_INFO(get_logger(), "Velocidad %.2f", speed->data);
    }


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
