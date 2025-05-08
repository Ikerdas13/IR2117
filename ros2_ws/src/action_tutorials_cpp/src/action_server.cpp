#include <inttypes.h>
#include <memory>
#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Alias para la acción Fibonacci
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

// Alias para el manejador de metas de la acción Fibonacci
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

// Manejador de la solicitud de meta (goal)
rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const Fibonacci::Goal> goal)
{
  // Imprime un mensaje indicando que se ha recibido una solicitud de meta
  RCLCPP_INFO(rclcpp::get_logger("server"), 
    "Recibida solicitud de meta con orden %d", goal->order);
  
  // Se ignora el uuid en este ejemplo
  (void)uuid;
  
  // Aceptamos y ejecutamos la meta
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Manejador para la solicitud de cancelación de la meta
rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  // Imprime un mensaje indicando que se ha recibido una solicitud de cancelación
  RCLCPP_INFO(rclcpp::get_logger("server"), 
    "Recibida solicitud para cancelar la meta");
  
  // Aceptamos la cancelación
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// Función para ejecutar la acción
void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  // Imprime un mensaje indicando que se está ejecutando la meta
  RCLCPP_INFO(rclcpp::get_logger("server"), 
    "Ejecutando la meta");

  // Definimos la tasa de ejecución (1 Hz)
  rclcpp::Rate loop_rate(1);

  // Obtenemos el objetivo (goal) desde el manejador
  const auto goal = goal_handle->get_goal();

  // Creamos un mensaje de feedback (retroalimentación)
  auto feedback = std::make_shared<Fibonacci::Feedback>();
  auto & sequence = feedback->partial_sequence;

  // Inicializamos la secuencia con los dos primeros números de Fibonacci
  sequence.push_back(0);
  sequence.push_back(1);

  // Creamos un mensaje para el resultado
  auto result = std::make_shared<Fibonacci::Result>();

  // Calculamos la secuencia de Fibonacci hasta el número de orden especificado
  for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {

    // Si el cliente solicita cancelar la meta, detenemos la ejecución
    if (goal_handle->is_canceling()) {
      result->sequence = sequence;
      goal_handle->canceled(result);
      RCLCPP_INFO(rclcpp::get_logger("server"), 
        "Meta cancelada");
      return;
    }

    // Añadimos el siguiente número en la secuencia de Fibonacci
    sequence.push_back(sequence[i] + sequence[i - 1]);

    // Publicamos la retroalimentación (feedback) para que el cliente lo reciba
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(rclcpp::get_logger("server"), 
      "Publicando retroalimentación");

    // Esperamos para cumplir con la tasa de ejecución (1 Hz)
    loop_rate.sleep();
  }

  // Si la ejecución fue exitosa y no fue cancelada, devolvemos el resultado final
  if (rclcpp::ok()) {
    result->sequence = sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(rclcpp::get_logger("server"), 
      "Meta completada con éxito");
  }
}

// Manejador para las metas aceptadas
void handle_accepted(
  const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
  // Creamos un hilo para ejecutar la acción de manera asíncrona
  std::thread{execute, goal_handle}.detach();
}

// Función principal
int main(int argc, char ** argv)
{
  // Inicializa ROS 2
  rclcpp::init(argc, argv);

  // Creamos un nodo de ROS 2 llamado "action_server"
  auto node = rclcpp::Node::make_shared("action_server");

  // Creamos el servidor de acciones para la acción "Fibonacci"
  auto action_server = 
    rclcpp_action::create_server<Fibonacci>(node,
      "fibonacci",  // Nombre de la acción
      handle_goal,  // Función para manejar la solicitud de meta
      handle_cancel,  // Función para manejar la solicitud de cancelación
      handle_accepted);  // Función para manejar las metas aceptadas

  // Ejecuta el nodo y espera por solicitudes de acciones
  rclcpp::spin(node);

  // Apaga ROS 2 cuando termine
  rclcpp::shutdown();

  return 0;
}

