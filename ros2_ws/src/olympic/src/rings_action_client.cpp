#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "olympic_interfaces/action/rings.hpp"

using Rings = olympic_interfaces::action::Rings;
using GoalHandleRings = rclcpp_action::ClientGoalHandle<Rings>;

rclcpp::Node::SharedPtr g_node = nullptr;

void feedback_callback(
    GoalHandleRings::SharedPtr,
    const std::shared_ptr<const Rings::Feedback> feedback)
{
    RCLCPP_INFO(
        g_node->get_logger(),
        "Progreso: Anillo %d/5, Ángulo: %.1f° (%.1f%%)",
        feedback->drawing_ring,
        feedback->ring_angle,
        feedback->ring_angle/3.6f);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("rings_action_client");

    // Obtener parámetro de radio (valor por defecto: 1.0)
    float radius = g_node->declare_parameter<float>("radius", 1.0f);

    auto action_client = rclcpp_action::create_client<Rings>(g_node, "rings");

    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(g_node->get_logger(), "Servidor de acciones no disponible.");
        return 1;
    }

    auto goal_msg = Rings::Goal();
    goal_msg.radius = radius;

    RCLCPP_INFO(g_node->get_logger(), "Enviando goal con radius = %.2f", goal_msg.radius);

    auto send_goal_options = rclcpp_action::Client<Rings>::SendGoalOptions();
    send_goal_options.feedback_callback = feedback_callback;

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(g_node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Falló la llamada async_send_goal.");
        return 1;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(g_node->get_logger(), "El servidor rechazó el goal.");
        return 1;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    RCLCPP_INFO(g_node->get_logger(), "Esperando al resultado...");

    if (rclcpp::spin_until_future_complete(g_node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Falló la llamada async_get_result.");
        return 1;
    }

    auto wrapped_result = result_future.get();
    switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(g_node->get_logger(), 
                       "¡Éxito! Anillos completados: %d/5", 
                       wrapped_result.result->rings_completed);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(g_node->get_logger(), "Goal abortado.");
            return 1;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(g_node->get_logger(), 
                       "Goal cancelado. Anillos completados: %d/5", 
                       wrapped_result.result->rings_completed);
            return 1;
        default:
            RCLCPP_ERROR(g_node->get_logger(), "Código de resultado desconocido.");
            return 1;
    }

    rclcpp::shutdown();
    return 0;
}