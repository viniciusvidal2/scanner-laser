#include "ros/ros.h"
#include "std_msgs/String.h"

#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/DynamixelInfo.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

int raw_min = 133, raw_max = 3979;
ros::Subscriber sub;

void chatterCallback(const dynamixel_workbench_msgs::DynamixelStateConstPtr& msg)
{
  ROS_INFO("No momento ca estamos: [%d]", msg->present_position);
  if(abs(msg->present_position - raw_max) < 3){
      sub.shutdown();
      ROS_WARN("Chegamos ao fim de curso.");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mover_continuo");
  ros::NodeHandle nh;

  sleep(2); // Para os motores iniciarem legal

  ROS_INFO("Estamos comecando o no agora.");

  // Cliente que ja manda de uma vez de um lado para o outro
  ros::ServiceClient rodar_continuo = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

  // Mandar motor para o 0 do range
  dynamixel_workbench_msgs::JointCommand comando;
  comando.request.pan_pos  = raw_min;
  comando.request.tilt_pos =       0;
  comando.request.unit     =   "raw";

  if(rodar_continuo.call(comando))
      ROS_INFO("Mandamos a zero para comecar. Estado: %.2f", comando.response.pan_pos);

  sleep(10); // Aguarda aqui um tempo para que chegue ao 0 com tranquilidade

  // Subscriber para ir lendo os angulos do motor
  sub = nh.subscribe("/multi_port/pan_state", 1000, chatterCallback);

  // Mandar o motor andar tudo de um lado para o outro mesmo vamos ver como vai ser
  comando.request.pan_pos  = raw_max;
  comando.request.tilt_pos =       0;
  comando.request.unit     =   "raw";

  if(rodar_continuo.call(comando))
      ROS_INFO("Mandamos o fim de curso aqui. Estado: %.2f", comando.response.pan_pos);

  ros::spin();

  return 0;
}
