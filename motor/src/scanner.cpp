#include "ros/ros.h"

#include <math.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

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
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace message_filters;
using namespace pcl;
using namespace std;

typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> syncPolicy;

// Variaveis referentes ao controle do motor
double raw_min = 152, raw_max = 3967;
double deg_min = 13, deg_max = 348;
double raw_tilt_hor = 2062;
double deg_raw = (deg_max - deg_min) / (raw_max - raw_min);
double raw_deg = 1.0 / deg_raw;
double raw_ref, deg_ref;
double dentro = 3; // valor raw considerado ok pra estar ja no ponto final do movimento do motor
ros::Subscriber sub;
ros::ServiceClient comando_motor;
bool comecar = false;

// Variaveis para leitura do laser
laser_geometry::LaserProjection projector;

// Variaveis de nuvem
PointCloud<PointXYZ>::Ptr acc;

// Variaveis de tf2 para ver no RViz
geometry_msgs::TransformStamped tf_msg;
tf2::Quaternion q;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double deg2raw(double deg){
    return (deg - deg_min)*raw_deg + raw_min;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double raw2deg(double raw){
    return (raw - raw_min)*deg_raw + deg_min;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f transformFromRaw(double raw){
    // Valor em graus que girou, tirando a referencia, convertido para frame global ODOM:
    // theta_y = -degrees (contrario a mao direita)
    double theta_y = -raw2deg(raw - raw_ref);
    // Prepara a mensagem atual de tf2 para ser transmitida e enviar
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "laser";
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    q.setRPY(0, DEG2RAD(theta_y), 0);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    // Constroi matriz de rotacao
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(               0, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(DEG2RAD(theta_y), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(               0, Eigen::Vector3f::UnitZ());
    // Constroi matriz homgenea e retorna
    Eigen::Vector3f t;
    t << 0, 0, 0;
    Eigen::Matrix4f T;
    T << R, t,
         0, 0, 0, 1;

    return T;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void callback(const sensor_msgs::LaserScanConstPtr& msg_laser,
              const nav_msgs::OdometryConstPtr& msg_motor){
    // Esperar chegar no 0 - ai liberar flag e começar
    if (abs(msg_motor->pose.pose.position.x - raw_min) < dentro && !comecar){
        ROS_INFO("Chegamos no inicio para comecar a aquisicao: %.0f", msg_motor->pose.pose.position.x);
        raw_ref = msg_motor->pose.pose.position.x; // Salva aqui a referencia nova raw
        deg_ref = raw2deg(raw_ref); // Nova referencia em degrees - aqui e o 0

        dynamixel_workbench_msgs::JointCommand comando;
        comando.request.pan_pos  = raw_max;
        comando.request.tilt_pos = raw_tilt_hor;
        comando.request.unit     =   "raw";
        if(comando_motor.call(comando))
            ROS_WARN("INICIO DE CAPTURA. Estado: %.2f graus", raw2deg(comando.response.pan_pos));

        comecar = true;
    } else if (abs(msg_motor->pose.pose.position.x - raw_min) > dentro && !comecar){
        ROS_INFO("Estamos esperando chegar no valor minimo, ainda estamos em %.0f graus", raw2deg(msg_motor->pose.pose.position.x));
    }

    /// DAQUI PRA FRENTE VAI VALER ///
    if(comecar){

        ROS_INFO("Aquisitando ....");
        // Começando, converter leitura para nuvem PCL
        sensor_msgs::PointCloud2 msg_cloud;
        projector.projectLaser(*msg_laser, msg_cloud);
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
        fromROSMsg(msg_cloud, *cloud);
        // Aplicar transformada de acordo com o angulo
        Eigen::Matrix4f T = transformFromRaw(msg_motor->pose.pose.position.x);
        transformPointCloud(*cloud, *cloud, T);
        // Acumular nuvem
        *acc += *cloud;

    } // fim do if comecar, processo principal

    // Finalizar processo se chegar ao fim do curso
    if(abs(msg_motor->pose.pose.position.x - raw_max) < dentro){
        pcl::io::savePLYFileASCII("/home/grin/Desktop/nuvem_laser.ply", *acc);
        ROS_WARN("Conferir por nuvem final na area de trabalho.");
        dynamixel_workbench_msgs::JointCommand finalizar;
        finalizar.request.pan_pos  = raw_max-5;
        finalizar.request.tilt_pos = 1965;
        finalizar.request.unit     = "raw";
        if(comando_motor.call(finalizar))
            ROS_INFO("Baixando de leve o motor para nao machucar aqui...");
        sleep(5);
        ros::shutdown();
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner");
    ros::NodeHandle nh;

    sleep(2); // Para os motores iniciarem legal

    ROS_INFO("Estamos comecando o no agora.");

    /// Aloca o ponteiro da nuvem acumulada
    ///
    acc = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    acc->header.frame_id = "map";

    /// Inicia o subscriber sincronizado para as mensagens
    ///
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/scan"                           , 100);
    message_filters::Subscriber<nav_msgs::Odometry>     motor_sub(nh, "/dynamixel_angulos_sincronizados", 100);
    Synchronizer<syncPolicy> sync(syncPolicy(100), laser_sub, motor_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    comecar = false;

    /// Inicio do publicador da nuvem acumulada e mensagem ros
    ///
    ros::Publisher pub_acc = nh.advertise<sensor_msgs::PointCloud2>("/laser_acumulada", 100);
    sensor_msgs::PointCloud2 msg_acc;
    msg_acc.header.frame_id = "map";

    ROS_INFO("Ligamos subscribers e publishers.");

    /// Aqui o publicador de tf2 e inicio da mensagem
    ///
    tf2_ros::TransformBroadcaster broadcaster;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "laser";
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.x = 0;
    tf_msg.transform.rotation.y = 0;
    tf_msg.transform.rotation.z = 0;
    tf_msg.transform.rotation.w = 1;

    /// Inicia o serviço - variavel global, para usar dentro do callback
    ///
    comando_motor = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

    /// Mandar motor para o 0 do range
    ///
    dynamixel_workbench_msgs::JointCommand comando;
    comando.request.pan_pos  = raw_min;
    comando.request.tilt_pos = raw_tilt_hor;
    comando.request.unit     =   "raw";
    if(comando_motor.call(comando))
        ROS_INFO("Mandamos a zero para comecar. Estado: %.2f", raw2deg(comando.response.pan_pos));

    /// Rodar o ros e o publicador em loop
    ///
    ros::Rate r(1);

    while(ros::ok()){
        // Publicar a nuvem
        toROSMsg(*acc, msg_acc);
        msg_acc.header.stamp = ros::Time::now();
        pub_acc.publish(msg_acc);
        // Publicar a tf
        broadcaster.sendTransform(tf_msg);
        // Dormir
        r.sleep();
        // Rodar o ros - MUITO IMPORTANTE
        ros::spinOnce();
    }

    return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
