#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_array.hpp"
#include <iostream>
#include <vector>


std::vector<float> numeros;

std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> publisher;
geometry_msgs::msg::Twist mensaje;

void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  
    numeros.push_back(msg->data);
    int min;
    int max;
    int longitud = numeros.size();
     
    
    
     std::sort(numeros.begin(), numeros.end());	
     max = numeros[longitud - 1];
     min = numeros[0];
     std::cout << min << max <<std::endl;
	
   
    
   mensaje = min, max;
   publisher->publish(mensaje); 
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minmax");

   
    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "number", 10, topic_callback); 
        
       
   publisher = node->create_publisher<geometry_msgs::msg::Twist>("minmax", 10);  
    
 

   
     

    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
}

