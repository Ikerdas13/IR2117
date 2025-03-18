#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>  
#include <limits>
#include <iostream>

using namespace std::chrono_literals;

float min_left = std::numeric_limits<float>::infinity();  
float min_right = std::numeric_limits<float>::infinity();  

void callback_topic(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::cout << "LaserScan received" << std::endl;

    if (msg->ranges.size() > 0) {
        std::cout << "Ranges size: " << msg->ranges.size() << std::endl;

        if (msg->ranges.size() > 270) {
            min_left = std::numeric_limits<float>::infinity();  
            min_right = std::numeric_limits<float>::infinity();  

            for (int i = 0; i <= 9; ++i) {
                min_left = std::min(min_left, msg->ranges[i]);
            }

            for (int i = 350; i <= 359; ++i) {
                min_right = std::min(min_right, msg->ranges[i]);
            }

            std::cout << "Minimum value on the left (ranges[0..9]): " << min_left << std::endl;
            std::cout << "Minimum value on the right (ranges[350..359]): " << min_right << std::endl;
        } else {
            std::cout << "LaserScan data is not large enough to access these indices." << std::endl;
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("wandering");

    auto subscriber = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, callback_topic);

    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    geometry_msgs::msg::Twist message;

    rclcpp::WallRate loop_rate(10ms);

    while (rclcpp::ok()) {
        if (min_left > 0.5 && min_right > 0.5) {
            message.linear.x = 0.2;  
            message.angular.z = 0.0; 
        } else if (min_left > min_right) {
            message.linear.x = 0.0;
            message.angular.z = 0.2;
        } else {
            message.linear.x = 0.0;
            message.angular.z = -0.2;
        }

        publisher->publish(message);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

