#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

std::vector<float> numeros;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    numeros.push_back(msg->data);

    std::cout << numeros << std::endl
    std_msgs::msg::Int32 out_msg;
    

    
   

   
    median; //No lo borro por si quiero publicar más tarde
    out_msg.data = median;  
    publisher->publish(out_msg); 
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("min");

   
    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "number", 10, topic_callback);  

   
    publisher = node->create_publisher<std_msgs::msg::Float32>("topico_median", 10);  

    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}

