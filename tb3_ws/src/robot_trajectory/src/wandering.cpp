#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

void callback_topic(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::cout << "LaserScan received" << std::endl;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Nodo para suscripción
    auto subscriber_node = rclcpp::Node::make_shared("subscriber");
    auto subscription = subscriber_node->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, callback_topic);

    // Nodo para publicación
    auto publisher_node = rclcpp::Node::make_shared("publisher");
    auto publisher = publisher_node->create_publisher<std_msgs::msg::String>("cmd_vel", 10);
    std_msgs::msg::String message;

    rclcpp::WallRate loop_rate(10ms);

    // Bucle de publicación
    while (rclcpp::ok()) {
        message.data = "0";  
        publisher->publish(message);
        loop_rate.sleep();
    }

    // Espera para el procesamiento de mensajes
    rclcpp::spin(subscriber_node);

    // Cerrar el sistema correctamente
    rclcpp::shutdown();
    return 0;
}

