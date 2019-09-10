#include "ros/ros.h"
#include "std_msgs/String.h"

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/DynamixelInfo.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace message_filters;
using namespace pcl;

typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> syncPolicy;

// Variaveis referentes ao controle do motor
int raw_min = 133, raw_max = 3979;
double deg_min = 0, deg_max = 360;
int raw_ref, deg_ref;
ros::Subscriber sub;
ros::ServiceClient comando_motor;
bool comecar = false;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void chatterCallback(const dynamixel_workbench_msgs::DynamixelStateConstPtr& msg)
{
  ROS_INFO("No momento ca estamos: [%d]", msg->present_position);
  if(abs(msg->present_position - raw_max) < 3){
      sub.shutdown();
      ROS_WARN("Chegamos ao fim de curso.");
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void callback(const sensor_msgs::LaserScanConstPtr& msg_laser,
              const nav_msgs::OdometryConstPtr& msg_motor){
    // Esperar chegar no 0 - ai liberar flag e começar
    if (abs(msg_motor->pose.pose.position.x) < 2 && !comecar){

    }

    // Começando, converter leitura para nuvem PCL

    // Aplicar transformada de acordo com o angulo

    // Acumular nuvem
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanner");
  ros::NodeHandle nh;

  sleep(2); // Para os motores iniciarem legal

  ROS_INFO("Estamos comecando o no agora.");

  /// Inicia o subscriber sincronizado para as mensagens
  ///
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/scan"                           , 100);
  message_filters::Subscriber<nav_msgs::Odometry>     motor_sub(nh, "/dynamixel_angulos_sincronizados", 100);
  Synchronizer<syncPolicy> sync(syncPolicy(100), laser_sub, motor_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  /// Inicia o serviço - variavel global, para usar dentro do callback
  ///
  comando_motor = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

//  // Cliente que ja manda de uma vez de um lado para o outro
//  ros::ServiceClient rodar_continuo = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

  /// Mandar motor para o 0 do range
  ///
  dynamixel_workbench_msgs::JointCommand comando;
  comando.request.pan_pos  = raw_min;
  comando.request.tilt_pos =       0;
  comando.request.unit     =   "raw";
  if(comando_motor.call(comando))
      ROS_INFO("Mandamos a zero para comecar. Estado: %.2f", comando.response.pan_pos);

//  sleep(10); // Aguarda aqui um tempo para que chegue ao 0 com tranquilidade

  // Subscriber para ir lendo os angulos do motor
//  sub = nh.subscribe("/multi_port/pan_state", 1000, chatterCallback);

  // Mandar o motor andar tudo de um lado para o outro mesmo vamos ver como vai ser
//  comando.request.pan_pos  = raw_max;
//  comando.request.tilt_pos =       0;
//  comando.request.unit     =   "raw";

//  if(rodar_continuo.call(comando))
//      ROS_INFO("Mandamos o fim de curso aqui. Estado: %.2f", comando.response.pan_pos);

  ros::spin();

  return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
