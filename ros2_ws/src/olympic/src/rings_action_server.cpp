#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "olympic_interfaces/action/rings.hpp"

using Rings = olympic_interfaces::action::Rings;
using GoalHandleRings = rclcpp_action::ServerGoalHandle<Rings>;

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

  auto feedback = std::make_shared<Rings::Feedback>();
  auto result = std::make_shared<Rings::Result>();

  for (int ring = 1; ring <= 5; ++ring) {
    for (int angle = 0; angle < 360; angle += 10) {
      if (goal_handle->is_canceling()) {
        result->rings_completed = ring - 1;
        goal_handle->canceled(result);
        RCLCPP_INFO(rclcpp::get_logger("server"), "Meta cancelada.");
        return;
      }

      feedback->drawing_ring = ring;
      feedback->ring_angle = static_cast<float>(angle);
      goal_handle->publish_feedback(feedback);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    result->rings_completed = ring;
  }

  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal completado: 5 aros dibujados");
  }
}

void handle_accepted(const std::shared_ptr<GoalHandleRings> goal_handle)
{
  std::thread{execute, goal_handle}.detach();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rings_action_server");

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

