#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <unordered_map>
#include <vector>

std::vector<float> numeros;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> publisher_mode;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>> publisher_modes;

void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    numeros.push_back(msg->data);
    std::unordered_map<float, int> frequency;
    int max_count = 0;

    // Contar ocurrencias
    for (float num : numeros) {
        frequency[num]++;
        if (frequency[num] > max_count) {
            max_count = frequency[num];
        }
    }

    
    std::vector<float> modes;
    for (const auto& pair : frequency) {
        if (pair.second == max_count) {
            modes.push_back(pair.first);
        }
    }

    // Publicar un único número con la mayor frecuencia
    std_msgs::msg::Float32 mode_msg;
    mode_msg.data = modes.front();  // Publica cualquiera de los modos
    publisher_mode->publish(mode_msg);

    
    std_msgs::msg::Float32MultiArray modes_msg;
    modes_msg.data = modes;
    publisher_modes->publish(modes_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mode");

 
    auto subscription = node->create_subscription<std_msgs::msg::Float32>(
        "topico_median", 10, topic_callback);

  
    publisher_mode = node->create_publisher<std_msgs::msg::Float32>("topico_mode", 10);
    publisher_modes = node->create_publisher<std_msgs::msg::Float32MultiArray>("topico_modes", 10);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
