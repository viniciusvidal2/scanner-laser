#include <ros/ros.h>
#include "../include/scanner_gui/registro.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teste_felipe");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");

  Registro* a = new Registro();
  a->init();
  //a.process("/home/grin/Desktop/Arquivos Do Workspace laser_ws/sem_calibracao/", "jpg");

}
