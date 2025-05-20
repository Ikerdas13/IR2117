#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "olympic_interfaces/action/rings.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/set_pen.hpp"

using Rings = olympic_interfaces::action::Rings;
using GoalHandleRings = rclcpp_action::ServerGoalHandle<Rings>;

// Variables globales para turtlesim
rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub = nullptr;
rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client = nullptr;
rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client = nullptr;

void draw_circle(float radius, int ring_number, const std::shared_ptr<GoalHandleRings> goal_handle)
{
    auto feedback = std::make_shared<Rings::Feedback>();
    auto result = std::make_shared<Rings::Result>();

    // Configurar color del lápiz según el anillo (colores olímpicos)
    auto set_pen_req = std::make_shared<turtlesim::srv::SetPen::Request>();
    set_pen_req->width = 3;
    switch(ring_number) {
        case 1: set_pen_req->r = 0; set_pen_req->g = 0; set_pen_req->b = 255; break; // Azul
        case 2: set_pen_req->r = 255; set_pen_req->g = 255; set_pen_req->b = 0; break; // Amarillo
        case 3: set_pen_req->r = 0; set_pen_req->g = 0; set_pen_req->b = 0; break; // Negro
        case 4: set_pen_req->r = 0; set_pen_req->g = 255; set_pen_req->b = 0; break; // Verde
        case 5: set_pen_req->r = 255; set_pen_req->g = 0; set_pen_req->b = 0; break; // Rojo
    }
    set_pen_client->async_send_request(set_pen_req);

    // Posicionar la tortuga para este anillo
    auto teleport_req = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    teleport_req->x = 5.5 + (ring_number-1) * radius * 1.1;
    teleport_req->y = 5.5;
    teleport_req->theta = 0;
    teleport_client->async_send_request(teleport_req);

    // Dibujar círculo
    for (int angle = 0; angle < 360; angle += 10) {
        if (goal_handle->is_canceling()) {
            return; 
        }

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = radius * 0.1;
        twist.angular.z = 0.1;
        cmd_vel_pub->publish(twist);

        feedback->drawing_ring = ring_number;
        feedback->ring_angle = static_cast<float>(angle);
        goal_handle->publish_feedback(feedback);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
}

rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Rings::Goal> goal)
{
    RCLCPP_INFO(rclcpp::get_logger("server"),
                "Recibido goal con radius %.2f", goal->radius);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRings> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("server"), "Recibida solicitud de cancelación");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(const std::shared_ptr<GoalHandleRings> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("server"), "Ejecutando goal de aros");
    const auto goal = goal_handle->get_goal();
    float radius = goal->radius;

    auto result = std::make_shared<Rings::Result>();
    result->rings_completed = 0; // Inicializar a 0

    for (int ring = 1; ring <= 5; ++ring) {
        if (!rclcpp::ok() || goal_handle->is_canceling()) {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(node->get_logger(), 
                          "Meta cancelada. Anillos completados: %d/5", 
                          result->rings_completed);
            }
            return;
        }

        draw_circle(radius, ring, goal_handle);
        result->rings_completed = ring; // Actualizar contador
    }

    if (rclcpp::ok()) {
        goal_handle->succeed(result);
        RCLCPP_INFO(rclcpp::get_logger("server"), 
                   "Goal completado: %d/5 aros dibujados", 
                   result->rings_completed);
    }
}

void handle_accepted(const std::shared_ptr<GoalHandleRings> goal_handle)
{
    std::thread{execute, goal_handle}.detach();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("rings_action_server");

    
    cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    teleport_client = node->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
    set_pen_client = node->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

    auto action_server = rclcpp_action::create_server<Rings>(
        node,
        "rings",
        handle_goal,
        handle_cancel,
        handle_accepted);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}