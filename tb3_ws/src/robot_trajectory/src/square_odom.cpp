#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

using namespace std::chrono_literals;

// Variables globales para almacenar las coordenadas y el ángulo inicial
float x_ini = 0.0;
float y_ini = 0.0;
float 𝜃_ini = 0.0;

float x;
float y; 
float 𝜃;

double calculate_yaw(nav_msgs::msg::Odometry message){
    // Obtener las componentes de la orientación
    float qw = message.pose.pose.orientation.w;
    float qx = message.pose.pose.orientation.x;
    float qy = message.pose.pose.orientation.y;
    float qz = message.pose.pose.orientation.z;

    // Calcular el ángulo de yaw (𝜃)
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    float yaw = std::atan2(siny_cosp, cosy_cosp); // atan2 para obtener el ángulo

    return yaw; // El ángulo en radianes
}

void topic_callback(nav_msgs::msg::Odometry message){
    // Inicializamos las coordenadas y ángulo inicial solo una vez
    if (x_ini == 0.0 && y_ini == 0.0 && 𝜃_ini == 0.0) {
        x_ini = message.pose.pose.position.x;
        y_ini = message.pose.pose.position.y;
        𝜃_ini = calculate_yaw(message); // Calcular yaw inicial
    }
    
    // Actualizamos las coordenadas y el ángulo
    x = message.pose.pose.position.x;
    y = message.pose.pose.position.y;
    𝜃 = calculate_yaw(message);

    // Diferencia en las coordenadas
    float delta_x = x - x_ini;
    float delta_y = y - y_ini;

    // Diferencia en el ángulo (yaw)
    float delta_𝜃 = 𝜃 - 𝜃_ini;

    // Normalizamos la diferencia de ángulo para estar en el rango [-π, π]
    delta_𝜃 = std::atan2(std::sin(delta_𝜃), std::cos(delta_𝜃));

    // Imprimimos las coordenadas y el ángulo
    std::cout << "La coordenada x es: " << x << std::endl;
    std::cout << "La coordenada y es: " << y << std::endl;
    std::cout << "El ángulo (𝜃) es: " << 𝜃 << " radianes" << std::endl;
    std::cout << "La diferencia en X: " << delta_x << " metros" << std::endl;
    std::cout << "La diferencia en Y: " << delta_y << " metros" << std::endl;
    std::cout << "La diferencia en el ángulo (𝜃): " << delta_𝜃 << " radianes" << std::endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("square_odom");
    auto subscriber = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, topic_callback);

    // Se mantiene el nodo corriendo
    rclcpp::WallRate loop_rate(10ms);
    rclcpp::spin(node); // Aquí es donde la función spin correcta mantiene el nodo en ejecución

    rclcpp::shutdown();
    return 0;
}

