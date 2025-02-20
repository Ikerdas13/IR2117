#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "nav_msgs::msg::odometry.hpp"
#include <cmath>

using namespace std::chrono_literals;

float x;
float y; 
float 𝜃;

void topic_callback(nav_msgs::msg::Odometry message){
	x = message.pose.pose.position.x;
	y = message.pose.pose.position.y;
	std::cout << " La coordenada x es: " << x << std::endl;
	std::cout << " La coordenada y es: " << y << std::endl;
	std::cout << " El anugulo es: " << 𝜃 << std::endl;
	
}

int main(int argc, char *argv[])
{
rclcpp::init(argc, argv);
auto node = rclcpp::Node::make_shared("square_odom");
 auto subscriber = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10,topic_callback);
 
 auto publish_count = 0;
rclcpp::WallRate loop_rate(10ms);

     
node->declare_parameter("linear_speed", 0.1);
node->declare_parameter("angular_speed", M_PI /20);
node->declare_parameter("square_length", 2);
    

    rclcpp::shutdown();
    return 0;
}

