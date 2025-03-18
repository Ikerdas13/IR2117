#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>  // Para std::min

using namespace std::chrono_literals;

void callback_topic(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::cout << "LaserScan received" << std::endl;

    if (msg->ranges.size() > 0) {
        std::cout << "Ranges size: " << msg->ranges.size() << std::endl;

        if (msg->ranges.size() > 270) {
            float min_value = std::numeric_limits<float>::infinity();  

           
            for (int i = 0; i <= 9; ++i) {
                min_value = std::min(min_value, msg->ranges[i]);
            }

           
            for (int i = 350; i <= 359; ++i) {
                min_value = std::min(min_value, msg->ranges[i]);
            }

            
            std::cout << "Minimum value in ranges[0..9] and ranges[350..359]: " << min_value << std::endl;
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
    auto publisher = node->create_publisher<std_msgs::msg::String>("cmd_vel", 10);
    std_msgs::msg::String message;

    rclcpp::WallRate loop_rate(10ms);

    while (rclcpp::ok()) {
        message.data = "0";  
        publisher->publish(message);
        rclcpp::spin(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

