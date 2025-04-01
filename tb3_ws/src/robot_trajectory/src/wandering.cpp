#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

float min_left = std::numeric_limits<float>::infinity();
float min_right = std::numeric_limits<float>::infinity();

void callback_topic(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.size() >= 360) {  // Verificar que hay suficientes datos
        min_left = std::numeric_limits<float>::infinity();
        min_right = std::numeric_limits<float>::infinity();

        for (int i = 0; i <= 9; ++i) { // Sector izquierdo (0° a 9°)
            if (std::isfinite(msg->ranges[i])) {
                min_left = std::min(min_left, msg->ranges[i]);
            }
        }
        for (int i = 350; i <= 359; ++i) { // Sector derecho (350° a 359°)
            if (std::isfinite(msg->ranges[i])) {
                min_right = std::min(min_right, msg->ranges[i]);
            }
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("obstacle_avoidance");

    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, callback_topic);

    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    rclcpp::Rate loop_rate(10ms); // Frecuencia del loop

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        auto cmd_msg = geometry_msgs::msg::Twist();
        float threshold = 0.5; // Umbral de distancia mínima

        if (min_left < threshold && min_right < threshold) {
            cmd_msg.angular.z = 0.5;  // Girar a la izquierda
        } else if (min_left < threshold) {
            cmd_msg.angular.z = -0.5; // Girar a la derecha
        } else if (min_right < threshold) {
            cmd_msg.angular.z = 0.5;  // Girar a la izquierda
        } else {
            cmd_msg.linear.x = 0.2;   // Avanzar
        }

        publisher->publish(cmd_msg);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

