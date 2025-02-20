#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

float x_ini = 0.0;
float y_ini = 0.0;
float 𝜃_ini = 0.0;

float x;
float y; 
float 𝜃;

double calculate_yaw(nav_msgs::msg::Odometry message) {
    float qw = message.pose.pose.orientation.w;
    float qx = message.pose.pose.orientation.x;
    float qy = message.pose.pose.orientation.y;
    float qz = message.pose.pose.orientation.z;

    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp); 
}

void topic_callback(nav_msgs::msg::Odometry message) {
    if (x_ini == 0.0 && y_ini == 0.0 && 𝜃_ini == 0.0) {
        x_ini = message.pose.pose.position.x;
        y_ini = message.pose.pose.position.y;
        𝜃_ini = calculate_yaw(message); 
    }

    x = message.pose.pose.position.x;
    y = message.pose.pose.position.y;
    𝜃 = calculate_yaw(message);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("square_odom");
    auto subscriber = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, topic_callback);

    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    float linear_speed = 0.2;  
    float angular_speed = M_PI / 20;  
    float target_distance = 1.0;  // Lado del cuadrado
    float target_angle = M_PI / 2;  // 90 grados (π / 2)

    rclcpp::WallRate loop_rate(10ms);

    geometry_msgs::msg::Twist msg;

    // Realizar 4 lados del cuadrado
    for (int j = 0; j < 4; j++) {
        
        
        float distance_travelled = 0.0;
        while (rclcpp::ok() && distance_travelled < target_distance) {
            float delta_x = x - x_ini;
            float delta_y = y - y_ini;
            distance_travelled = std::sqrt(delta_x * delta_x + delta_y * delta_y);
	    std::cout << distance_travelled << std::endl;
	    
            msg.linear.x = linear_speed;  
            msg.angular.z = 0.0;  
            publisher->publish(msg);
            rclcpp::spin_some(node); 
            loop_rate.sleep();
        }

        // Detener el robot después de avanzar
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher->publish(msg);
        rclcpp::spin_some(node); 
        loop_rate.sleep();

        
        float angle_diff = target_angle - 𝜃;
        while (rclcpp::ok() && std::fabs(angle_diff) > 0.01) { 
            angle_diff = target_angle - 𝜃;
            
            
            std::cout << angle_diff << std::endl;

            
            if (angle_diff > M_PI) {
                angle_diff -= 2 * M_PI;
                
            } else if (angle_diff < -M_PI) {
                angle_diff += 2 * M_PI;
            }

            msg.linear.x = 0.0;  
            msg.angular.z = (angle_diff > 0) ? angular_speed : -angular_speed;  
            publisher->publish(msg);
            rclcpp::spin_some(node); 
            loop_rate.sleep();
        }
        
        
        
        

        // Detener el robot después de girar
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher->publish(msg);
        rclcpp::spin_some(node); 
        loop_rate.sleep();

        // Actualizar la posición inicial para el siguiente lado
        x_ini = x;
        y_ini = y;
        𝜃_ini = 𝜃;
    }

    rclcpp::shutdown();
    return 0;
}

