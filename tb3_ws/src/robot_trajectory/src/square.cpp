#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;


int main(int argc, char *argv[])
{
rclcpp::init(argc, argv);
auto node = rclcpp::Node::make_shared("square");
 auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
 geometry_msgs::msg::Twist message;
 auto publish_count = 0;
rclcpp::WallRate loop_rate(10ms);

     
node->declare_parameter("linear_speed", 0.1);
node->declare_parameter("angular_speed", 3.1416 /20);
    
    
 double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    
    
    
    
    
    
    
   
    

    for (int j = 0; j < 4; j++) {
      
        int i = 0, n = 1000;  
        while (rclcpp::ok() && i < n) {
            i++;
            message.linear.x = linear_speed;  
            message.angular.z = 0;
            publisher->publish(message); 
            rclcpp::spin_some(node); 
            loop_rate.sleep();
        }

        
        message.linear.x = 0;
        message.angular.z = 0;
        publisher->publish(message); 
        rclcpp::spin_some(node); 
        loop_rate.sleep();

        
        i = 0; n = 1000;  
        while (rclcpp::ok() && i < n) {
            i++;
            message.linear.x = 0;
            message.angular.z = angular_speed;  //0.157 en la simulacion
            publisher->publish(message); 
            rclcpp::spin_some(node); 
            loop_rate.sleep();
        }
    }

    rclcpp::shutdown();
    return 0;
}

