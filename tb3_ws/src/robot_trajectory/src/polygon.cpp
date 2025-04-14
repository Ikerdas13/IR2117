#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using namespace std::chrono_literals;


int main(int argc, char *argv[])
{
rclcpp::init(argc, argv);
auto node = rclcpp::Node::make_shared("polygon");
 auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
 geometry_msgs::msg::Twist message;
rclcpp::WallRate loop_rate(10ms);

     
node->declare_parameter("linear_speed", 0.8);
node->declare_parameter("angular_speed", 0.5);
node->declare_parameter("segment_size", 0.5);
node->declare_parameter("number_segments", 4);
    
    
 double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
double segment_size = node->get_parameter("segment_size").get_parameter_value().get<double>();
int number_segments = node->get_parameter("number_segments").get_parameter_value().get<int>();

    double radianes_cada_giro;
    radianes_cada_giro = (360 / number_segments) * 0.0174532925;
    
    double tiempo;
    tiempo = (radianes_cada_giro / angular_speed)*100;
    
    double iteraciones;
    iteraciones = segment_size / linear_speed * 100;
    
    
    for (int j = 0; j < number_segments; j++) {
      
        int i = 0, n = segment_size * iteraciones;  
        while (rclcpp::ok() && i < n) {
            i++;
            message.linear.x = linear_speed;  
            message.angular.z = 0;
            publisher->publish(message); 
            rclcpp::spin_some(node); 
            loop_rate.sleep();
        }

        
        message.linear.x = 0;
        message.angular.z = 0;
        publisher->publish(message); 
        rclcpp::spin_some(node); 
        loop_rate.sleep();

        
        i = 0; n = tiempo;  
        while (rclcpp::ok() && i < n) {
            i++;
            message.linear.x = 0;
            message.angular.z = angular_speed;  
            publisher->publish(message); 
            rclcpp::spin_some(node); 
            loop_rate.sleep();
        }
    }
    message.linear.x = 0;
        message.angular.z = 0;
        publisher->publish(message); 
        rclcpp::spin_some(node); 
        loop_rate.sleep();

    rclcpp::shutdown();
    return 0;
}

