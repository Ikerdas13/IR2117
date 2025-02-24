#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

std::vector<float> numeros;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    numeros.push_back(msg->data);

    std::cout << numeros << std::endl
    
    
 
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("min");

   
    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "number", 10, topic_callback);  

   
     

    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}

