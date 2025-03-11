#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

double x_ini;
double y_ini;
double theta_ini;

double x;
double y;
double theta;

double normalizar_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double calculate_yaw(const nav_msgs::msg::Odometry& message) {
    double qw = message.pose.pose.orientation.w;
    double qx = message.pose.pose.orientation.x;
    double qy = message.pose.pose.orientation.y;
    double qz = message.pose.pose.orientation.z;

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

void topic_callback(const nav_msgs::msg::Odometry::SharedPtr message) {
    x = message->pose.pose.position.x;
    y = message->pose.pose.position.y;
    theta = calculate_yaw(*message);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("square_odom");
    auto subscriber = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, topic_callback);
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    node->declare_parameter("linear_speed", 0.1);
    node->declare_parameter("angular_speed", 0.1);
    node->declare_parameter("square_length", 1.0);
    double target_angle = M_PI / 2;

    double linear_speed = node->get_parameter("linear_speed").get_parameter_value().get<double>();
    double angular_speed = node->get_parameter("angular_speed").get_parameter_value().get<double>();
    double square_length = node->get_parameter("square_length").get_parameter_value().get<double>();

    rclcpp::WallRate loop_rate(10ms);

    geometry_msgs::msg::Twist msg;

    rclcpp::spin_some(node);
    loop_rate.sleep();

    for (int j = 0; j < 4; j++) {
        float distance_travelled = 0.0;
        x_ini = x;
        y_ini = y;
        theta_ini = theta;

        while (rclcpp::ok() && distance_travelled < square_length) {
            msg.linear.x = linear_speed;
            msg.angular.z = 0.0;
            publisher->publish(msg);

            distance_travelled = std::sqrt(std::pow(x - x_ini, 2) + std::pow(y - y_ini, 2));

            rclcpp::spin_some(node);
            loop_rate.sleep();
        }

        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher->publish(msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();

        theta_ini = theta;
        while (rclcpp::ok() && std::abs(normalizar_angle(theta - theta_ini)) < target_angle) {
            msg.linear.x = 0.0;
            msg.angular.z = angular_speed;
            publisher->publish(msg);

            rclcpp::spin_some(node);
            loop_rate.sleep();
        }

        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher->publish(msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

