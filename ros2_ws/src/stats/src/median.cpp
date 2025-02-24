#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

std::vector<float> numeros;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    numeros.push_back(msg->data);
    float median;
    
    std_msgs::msg::Float32 out_msg;
    

    
    std::sort(numeros.begin(), numeros.end());

   
    if (numeros.size() % 2 == 0) {
        median = (numeros[numeros.size()/2 - 1] + numeros[numeros.size()/2]) / 2;
    } else {
        median = numeros[numeros.size()/2];
    }

    out_msg.data = median;  
    publisher->publish(out_msg); 
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("median");

   
    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "number", 10, topic_callback);  

   
    publisher = node->create_publisher<std_msgs::msg::Float32>("topico_median", 10);  

    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}

