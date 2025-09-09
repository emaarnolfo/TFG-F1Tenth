#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <fstream> // Para imprimir los logs en un archivo

#include "std_msgs/msg/float64.hpp"

// Inclusiones del vesc
#include <vesc_msgs/msg/vesc_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // Tiempo de inicio del nodo
        start_time_ = this->get_clock()->now();


        // Abrir archivo de log
        log_file_.open("/home/orin/f1tenth_ws/wall_follow_log.txt", std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo de log.");
        }

        // Lidar scan subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
        
        // Drive publisher
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
    }

private:
    std::ofstream log_file_; // Archivo para logs
    rclcpp::Time start_time_; // Tiempo de inicio del nodo
    
    // PID CONTROL PARAMS
    double kp = 0.8;
    double kd = 0.5;
    double ki = 0.2;

    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    double dist_deseada = 0.8; // Desired distance to the wall
    double dist_L = 0.05; // Constanste para el instante siguiente
    double g_dist_CD;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/ackermann_cmd";

    /// Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;


    // Funcion para log dial (consola + archivo)
    void log_info(const std::string &message) {
        // RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        auto now = this->get_clock()->now();
        auto relative_time = now - start_time_;

        double relative_time_milis = relative_time.seconds() * 1000.0;
        if (log_file_.is_open()) {
            log_file_ << relative_time_milis << "; " << message << std::endl;
        }
    }
    /*
    * Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.
    * Args:
    *     range_data: single range array from the LiDAR
    *     angle: between angle_min and angle_max of the LiDAR
    * 
    * Returns:
    *     range: range measurement in meters at the given angle
    */
    double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle)
    {

        // Angulo en grados a radianes
        double angle_rad = angle * M_PI / 180.0;

        // Verificamos que esté dentro del rango del lidar
        if (angle_rad < scan_msg->angle_min || angle_rad > scan_msg->angle_max) {
            RCLCPP_WARN(this->get_logger(), "Angle out of range");
            return std::numeric_limits<double>::quiet_NaN();
        }

        // Calculo del indice
        int index = static_cast<int>((angle_rad - scan_msg->angle_min) / scan_msg->angle_increment);

        // Verificamos que el indice esté dentro del rango
        if (index < 0 || index >= scan_msg->ranges.size()) {
            RCLCPP_WARN(this->get_logger(), "Index out of range");
            return std::numeric_limits<double>::quiet_NaN();
        }

        // Obtenemos la distancia
        double range = scan_msg->ranges[index];

        return range;
    }

    double get_error(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double dist_deseada)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

    	// double dist_a = get_range(scan_msg, 60.0); // <- Sigue la pared izquierda
        // double dist_b = get_range(scan_msg, 90.0); // <- Sigue la pared izquierda

    	double dist_a = get_range(scan_msg, -60.0); // <- Sigue la pared derecha
        double dist_b = get_range(scan_msg, -90.0); // <- Sigue la pared derecha
        double theta_rad = 30 * M_PI / 180.0;
        double alpha = atan2(dist_a*cos(theta_rad) - dist_b, dist_a*sin(theta_rad));

        double dist_AB = dist_b * cos(alpha);
        double dist_CD = dist_AB + dist_L * sin(alpha);
        g_dist_CD = dist_CD;

        double dist_error = dist_CD - dist_deseada;
        // double dist_error = dist_deseada - dist_CD;

        return dist_error;
    }

    void pid_control(double error, double velocity)
    {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        this->error = error;

        double pid = kp * error + kd * (error - prev_error);
        // double angle = 1 * pid; // <- Sigue la pared izquierda
        double angle = -1 * pid; // <- Sigue la pared derecha

        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;

        // Publish the drive message
        drive_publisher_->publish(drive_msg);

        // imprimir calculo de pid para debug
        // RCLCPP_INFO(this->get_logger(), "PID: %f", pid);
        // RCLCPP_INFO(this->get_logger(), "angle: %f", angle);
        // log_info("PID: " + std::to_string(pid) + "; angle: " + std::to_string(angle) + "; error: " + std::to_string(error) + "; velocity: " + std::to_string(velocity));
        // log_info(std::to_string(pid) + "; " + std::to_string(angle) + "; " + std::to_string(error) + "; " + std::to_string(velocity) + "; " + std::to_string(g_dist_CD) /*Distancia a la pared*/);
        log_info(std::to_string(angle) + "; " + std::to_string(error) + "; " + std::to_string(velocity) + "; " + std::to_string(g_dist_CD) /*Distancia a la pared*/);
        
        // RCLCPP_INFO(this->get_logger(), "Velovidad: %f", velocity);

        // Guardar el error para la siguiente iteracion
        prev_error = error;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        static double last_time = this->get_clock()->now().seconds();
        double current_time = this->get_clock()->now().seconds();
        double dt = current_time - last_time;
        
        double target_velocity = 1.5; // Velocidad objetivo
        static double current_velocity = 0.0; // Velocidad actual inicializada en 0
        
        if(dt > 0.1) {
            // Aceleración suave hacia la velocidad objetivo
            if(current_velocity < target_velocity) {
                current_velocity += 0.5 * dt; // Aumenta la velocidad en 0.5 m/s²
                if(current_velocity > target_velocity) {
                    current_velocity = target_velocity; // No exceder la velocidad objetivo
                }
            } else if(current_velocity > target_velocity) {
                current_velocity -= 0.5 * dt; // Disminuye la velocidad en 0.5 m/s²
                if(current_velocity < target_velocity) {
                    current_velocity = target_velocity; // No caer por debajo de la velocidad objetivo
                }
            }
            last_time = current_time;
        }

        RCLCPP_INFO(this->get_logger(), "current velocity: %f", current_velocity);
        RCLCPP_INFO(this->get_logger(), "dt : %f", dt);
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        double error = get_error(scan_msg, dist_deseada); // TODO: replace with error calculated by get_error()
        double velocity = 1.50; // TODO: calculate desired car velocity based on error
        // double velocity = 0; // TODO: calculate desired car velocity based on error
        
        // TODO: actuate the car with PID
        pid_control(error, current_velocity);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
