#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

using namespace std::chrono_literals;

float x;
float y; 
float 𝜃;

double calculate_yaw(nav_msgs::msg::Odometry message){
    
    float qw = message.pose.pose.orientation.w;
    float qx = message.pose.pose.orientation.x;
    float qy = message.pose.pose.orientation.y;
    float qz = message.pose.pose.orientation.z;

   
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw; 
}

void topic_callback(nav_msgs::msg::Odometry message){
    
    x = message.pose.pose.position.x;
    y = message.pose.pose.position.y;
    
    
    𝜃 = calculate_yaw(message);
    
  
    std::cout << "La coordenada x es: " << x << std::endl;
    std::cout << "La coordenada y es: " << y << std::endl;
    std::cout << "El ángulo (𝜃) es: " << 𝜃 << " radianes" << std::endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("square_odom");
    auto subscriber = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, topic_callback);

    auto publish_count = 0;
    rclcpp::WallRate loop_rate(10ms);

    node->declare_parameter("linear_speed", 0.1);
    node->declare_parameter("angular_speed", M_PI / 20);
    node->declare_parameter("square_length", 2);

    rclcpp::shutdown();
    return 0;
}

