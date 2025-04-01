
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "example_interfaces/msg/bool.hpp"

using namespace std::chrono_literals;

void call_back_front(const example_msgs::msg::Bool::SharePtr msg)
{
}

void call_back_left(const example_msgs::msg::Bool::SharePtr msg)
{
}

void call_back_right(const example_msgs::msg::Bool::SharePtr msg)
{
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp:: Node::make_shared ("avoidance");
  auto publisher = node->create_publisher<geometry_msgs::msg:: Twist>("cmd_vel", 10); geometry_msgs::msg:: Twist message;
  rclcpp::WallRate loop_rate (50ms);
  auto subs_front = node->create_subscription<example_msgs::msg::Bool>("/front/obstacle", 10, callback_front);
  auto subs_left = node->create_subscription<example_msgs::msg::Bool>("/left/obstacle", 10, callback_front);
  auto subs_right = node->create_subscription<example_msgs::msg::Bool>("/right/obstacle", 10, callback_front); 
  
  
  
  while (rclcpp::ok()) {
    publisher->publish(message);
    rclcpp::spin_some (node); 
    loop_rate.sleep();
  }
  rclcpp:: shutdown();
  return 0;
}
