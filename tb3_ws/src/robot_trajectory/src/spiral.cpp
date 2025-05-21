#include <chrono>     
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;   

int main(int argc, char * argv[])     
{
  rclcpp::init(argc, argv); 
  geometry_msgs::msg::Twist message;    
  auto node = rclcpp::Node::make_shared("spiral");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);     
  
  node->declare_parameter("angular_speed", 0.5);
  node->declare_parameter("distance_between_loops", 1.0);
  node->declare_parameter("number_of_loops", 3);

  double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
  double distance_between_loops = node->get_parameter("distance_between_loops").get_parameter_value().get<double>();
  int number_of_loops = node->get_parameter("number_of_loops").get_parameter_value().get<int>();
rclcpp::WallRate loop_rate(10ms);
  
  
  double angle = 0.0;
  double linear_speed = 0.0;
  double current_loops = 0.0;
  double time = 0.001;
  

  while (rclcpp::ok() && current_loops <= number_of_loops) {   

    angle += angular_speed * time;
    linear_speed = angular_speed * distance_between_loops * angle / 2*M_PI;
    current_loops = angle / 2*M_PI;

    message.linear.x = linear_speed;
    message.angular.z = angular_speed;

    publisher->publish(message);

    rclcpp::spin_some(node);
    loop_rate.sleep();
    
  }
  message.linear.x = 0;
  message.angular.z = 0;

  publisher->publish(message);

  rclcpp::shutdown();   
  return 0;
}
