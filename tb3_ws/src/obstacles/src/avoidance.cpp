#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "example_interfaces/msg/bool.hpp"
#include <cstdlib>
#include <ctime>
#include <iostream>

using namespace std::chrono_literals;

int front = 0;
int left = 0;
int right = 0;

void callback_front(const example_interfaces::msg::Bool::SharedPtr msg)
{
    front = msg->data ? 1 : 0;
}

void callback_left(const example_interfaces::msg::Bool::SharedPtr msg)
{
    left = msg->data ? 1 : 0;
}

void callback_right(const example_interfaces::msg::Bool::SharedPtr msg)
{
    right = msg->data ? 1 : 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("avoidance");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    geometry_msgs::msg::Twist message;
    rclcpp::WallRate loop_rate(10ms);

    auto subs_front = node->create_subscription<example_interfaces::msg::Bool>("/front/obstacle", 10, callback_front);
    auto subs_left = node->create_subscription<example_interfaces::msg::Bool>("/left/obstacle", 10, callback_left);
    auto subs_right = node->create_subscription<example_interfaces::msg::Bool>("/right/obstacle", 10, callback_right);

    std::srand(std::time(0));  

    while (rclcpp::ok()) {
        
        if (front != 1) {
            message.linear.x = 0.2;  
            message.angular.z = 0.0; 
        }
        
        else {
            if (left == 1 && right == 0) {
                message.linear.x = 0.0;  
                message.angular.z = -0.2; 
            }
            else if (left == 0 && right == 1) {
                message.linear.x = 0.0;  
                message.angular.z = 0.2; 
            }
            else if (left == 0 && right == 0) {
                
                message.linear.x = 0.0; 
                message.angular.z = (std::rand() % 2 == 0) ? 0.2 : -0.2; 
            }
            else {
                message.linear.x = 0.0;  
                message.angular.z = 0.2;
            }
        }

        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

