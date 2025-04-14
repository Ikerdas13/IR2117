#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "example_interfaces/msg/bool.hpp"
#include <cmath>

std::shared_ptr<rclcpp::Publisher<example_interfaces::msg::Bool>> publisher;
double person_angle_min, person_angle_max, person_distance_min, person_distance_max;

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  example_interfaces::msg::Bool out_msg;
  out_msg.data = false;
  float angle = msg->angle_min;

  for (float range : msg->ranges) {
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    }
    if ((angle >= person_angle_min) && (angle <= person_angle_max)) {
      if (range > person_distance_min and range < person_distance_max) {
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

  
  node->declare_parameter("person_angle_min", -M_PI / 12);
  node->declare_parameter("person_angle_max", M_PI / 12);
  node->declare_parameter("person_distance_min", 0.5);
  node->declare_parameter("person_distance_max", 1.3);

  
  person_angle_min = node->get_parameter("person_angle_min").as_double();
  person_angle_max = node->get_parameter("person_angle_max").as_double();
  person_distance_min = node->get_parameter("person_distance_min").as_double();
  person_distance_max = node->get_parameter("person_distance_max").as_double();

  
  publisher = node->create_publisher<example_interfaces::msg::Bool>("person", 10);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, callback);  

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

