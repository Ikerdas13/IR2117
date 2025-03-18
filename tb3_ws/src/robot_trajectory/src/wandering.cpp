#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

void callback_topic(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::cout << "LaserScan received" << std::endl;

   
    if (msg->ranges.size() > 0) {
        std::cout << "Ranges size: " << msg->ranges.size() << std::endl;  

        if (msg->ranges.size() > 270) {
            std::cout << "Value at index 0: " << msg->ranges[0] << std::endl;
            std::cout << "Value at index 90: " << msg->ranges[90] << std::endl;
            std::cout << "Value at index 180: " << msg->ranges[180] << std::endl;
            std::cout << "Value at index 270: " << msg->ranges[270] << std::endl;
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

