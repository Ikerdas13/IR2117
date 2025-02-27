#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

double x_ini;
double y_ini;
double theta_ini;

double x;
double y; 
double theta;

double calculate_yaw(nav_msgs::msg::Odometry message) {
    double qw = message.pose.pose.orientation.w;
    double qx = message.pose.pose.orientation.x;
    double qy = message.pose.pose.orientation.y;
    double qz = message.pose.pose.orientation.z;

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp); 
}

void topic_callback(nav_msgs::msg::Odometry message) {
    x = message.pose.pose.position.x;
    y = message.pose.pose.position.y;
    theta = calculate_yaw(message);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("square_odom");
    auto subscriber = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, topic_callback);
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    
    node->declare_parameter("linear_speed", 0.1);
    node->declare_parameter("angular_speed", 0.1);
    node->declare_parameter("square_length", 1);
    double target_angle = M_PI / 2;
    
    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    int square_length = node->get_parameter("square_length").get_parameter_value().get<int>();


     

    rclcpp::WallRate loop_rate(10ms);

    geometry_msgs::msg::Twist msg;

    for (int j = 0; j < 4; j++) {
        // Avanzar
        float distance_travelled = 0.0;
        x_ini = x;
        y_ini = y;
        theta_ini = theta;

        while (rclcpp::ok() && distance_travelled < square_length) {
            float delta_x = x - x_ini;
            float delta_y = y - y_ini;
            distance_travelled = std::sqrt(delta_x * delta_x + delta_y * delta_y);
            msg.linear.x = linear_speed;  
            msg.angular.z = 0.0;  
            publisher->publish(msg);
            rclcpp::spin_some(node); 
            loop_rate.sleep();
        }

        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher->publish(msg);
        rclcpp::spin_some(node); 
        loop_rate.sleep();

        // Girar 90 grados
        float current_angle = theta; // Guardamos el ángulo actual al inicio de la rotación
        while (rclcpp::ok()) {
            float angle_diff = target_angle - (theta - current_angle); // Diferencia con el ángulo actual
            // Ajuste para el caso en que el ángulo se pase de -π a π
            if (angle_diff > M_PI) {
                angle_diff -= 2 * M_PI;
            } else if (angle_diff < -M_PI) {
                angle_diff += 2 * M_PI;
            }

            // Si la diferencia de ángulo es suficientemente pequeña, paramos
            if (fabs(angle_diff) < 0.01) {
                break;
            }

            // Control del movimiento
            msg.linear.x = 0.0;  
            msg.angular.z = (angle_diff > 0) ? angular_speed : -angular_speed;  
            publisher->publish(msg);
            rclcpp::spin_some(node); 
            loop_rate.sleep();
        }

        // Detenerse una vez completado el giro
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher->publish(msg);
        rclcpp::spin_some(node); 
        loop_rate.sleep();

        // Actualizamos el ángulo de referencia para el siguiente giro
        current_angle = theta;  // Ahora el ángulo actual es la nueva referencia
    }

    rclcpp::shutdown();
    return 0;
}

