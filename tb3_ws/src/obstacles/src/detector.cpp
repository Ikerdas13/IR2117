#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "example_interfaces/msg/bool.hpp"
#include <cmath>

std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Bool>> publisher;
double obs_angle_min, obs_angle_max, obs_threshold;

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  example_interfaces::msg::Bool out_msg;
  out_msg.data = false;
  float angle = msg->angle_min;

  for (float range : msg->ranges) {
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    }
    if ((angle >= obs_angle_min) && (angle <= obs_angle_max)) {
      if (range <= obs_threshold and range>0.1) {
        out_msg.data = true;
        break; 
      }
    }
    angle += msg->angle_increment;
  }

  publisher->publish(out_msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("detector");

  
  node->declare_parameter("obs_angle_min", -M_PI / 8);
  node->declare_parameter("obs_angle_max", M_PI / 8);
  node->declare_parameter("obs_threshold", 0.5);

  
  obs_angle_min = node->get_parameter("obs_angle_min").as_double();
  obs_angle_max = node->get_parameter("obs_angle_max").as_double();
  obs_threshold = node->get_parameter("obs_threshold").as_double();

  
  publisher = node->create_publisher<example_interfaces::msg::Bool>("obstacle", 10);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, callback);  

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

