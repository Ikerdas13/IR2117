#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>
#include "example_interfaces/msg/bool.hpp"
#include <cmath>

std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Bool> > publisher;
void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
example_interfaces::msg:: Bool out_msg; 
out_msg.data = false;
float angle = msg->angle_min;
for (float range: msg->ranges) { 
	if (angle M_PI) angle-=2*M_PI;
	if ((angle >= obs_angle_min) and (angle <= obs_angle_max)) { 
		if (range <= obs_threshold)
			out_msg.data = true;
		}
		angle+msg->angle_increment;
	}
	publisher->publish (out_msg);
}
int main(int argc, char * argv[])
{
  node->declare_parameter("angle_min", 0.2);  
  node->declare_parameter("obs_angle_min", M_PI/8);
  node->declare_parameter("obs_angle_max", -M_PI/8);
    
  double linear_speed = node->get_parameter("angle_min").get_parameter_value().get<double>();
  double angular_speed = node->get_parameter("obs_angle_min").get_parameter_value().get<double>();
  double square_length = node->get_parameter("obs_angle_max").get_parameter_value().get<double>();  
    
    
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("detector");
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, topic_callback);
  publisher = node ->create_publisher<example_interfaces::msg::Bool>("obstacle",10);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
 }
