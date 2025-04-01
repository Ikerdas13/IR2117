#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <limits>
#include <iostream>

using namespace std::chrono_literals;

float min_left = std::numeric_limits<float>::infinity();
float min_right = std::numeric_limits<float>::infinity();

bool turn_left = false;
bool turn_right = false;

void callback_topic(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.size() > 270) {
        min_left = std::numeric_limits<float>::infinity();
        min_right = std::numeric_limits<float>::infinity();

        for (int i = 0; i <= 9; ++i) {
            if (std::isfinite(msg->ranges[i])) {
                min_left = std::min(min_left, msg->ranges[i]);
            }
        }
        for (int i = 350; i <= 359; ++i) {
            if (std::isfinite(msg->ranges[i])) {
                min_right = std::min(min_right, msg->ranges[i]);
            }
        }
    }
}

        rclcpp::spin_some(node);  // Solo una llamada
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
