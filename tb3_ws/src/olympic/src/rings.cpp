#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class OlympicRings : public rclcpp::Node
{
public:
    OlympicRings()
    : Node("olympic_rings_node")
    {
        set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        clear_client_ = this->create_client<std_srvs::srv::Empty>("/clear");

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(1000ms, std::bind(&OlympicRings::draw_rings, this));
    }

private:
    void draw_rings()
    {
        clear_screen();

        draw_ring(5.0, 8.0, 0, 0, 255);   // Azul
        draw_ring(7.0, 8.0, 0, 0, 0);     // Negro
        draw_ring(9.0, 8.0, 255, 0, 0);   // Rojo
        draw_ring(6.0, 6.5, 255, 255, 0); // Amarillo
        draw_ring(8.0, 6.5, 0, 255, 0);   // Verde
    }

    void draw_ring(float x, float y, int r, int g, int b)
    {
        set_pen(0, 0, 0, 5, true);
        teleport_to_position(x, y);
        set_pen(r, g, b, 5, false);
        draw_circle();
    }

    void clear_screen()
    {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        clear_client_->async_send_request(request);
        rclcpp::sleep_for(500ms);
    }

    void set_pen(int r, int g, int b, int width, bool off)
    {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = r;
        request->g = g;
        request->b = b;
        request->width = width;
        request->off = off;
        set_pen_client_->async_send_request(request);
        rclcpp::sleep_for(100ms);
    }

    void teleport_to_position(float x, float y)
    {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = x;
        request->y = y;
        request->theta = 0.0;
        teleport_client_->async_send_request(request);
        rclcpp::sleep_for(100ms);
    }

    void draw_circle()
    {
        geometry_msgs::msg::Twist message;
        message.linear.x = 1.5;
        message.angular.z = 1.5;
        rclcpp::Rate rate(100);
        for (int i = 0; i < 628; ++i) {
            publisher_->publish(message);
            rate.sleep();
        }
        message.linear.x = 0.0;
        message.angular.z = 0.0;
        publisher_->publish(message);
        rclcpp::sleep_for(300ms);
    }

    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OlympicRings>());
    rclcpp::shutdown();
    return 0;
}

