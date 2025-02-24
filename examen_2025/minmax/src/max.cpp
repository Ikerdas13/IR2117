#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

std::vector<float> numeros;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  
    numeros.push_back(msg->data);
    int max;
    int longitud = numeros.size();
    std_msgs::msg::Int32 out_msg;
    
    
     std::sort(numeros.begin(), numeros.end());
     	
	max = numeros[longitud - 1];
	
   
    
   out_msg.data = max;  
   publisher->publish(out_msg); 
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("max");

   
    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "number", 10, topic_callback); 
        
       
   publisher = node->create_publisher<std_msgs::msg::Int32>("max", 10);  
 

   
     

    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}

