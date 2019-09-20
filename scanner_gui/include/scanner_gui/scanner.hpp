#ifndef SCANNER_H
#define SCANNER_H

#endif // SCANNER_H

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

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <math.h>
#include <qobject.h>
#include <QThread>
#include <QMutex>
#include <QString>

namespace scanner_gui{

using namespace message_filters;
using namespace pcl;
using namespace std;

class Scanner : public QThread
{
    Q_OBJECT
public:
    Scanner(int argc, char** argv, QMutex*);
    virtual ~Scanner();
    void init();
    void get_limits(int &minm, int &maxm);
    void set_course(double min, double max);
    void set_acquisition(bool start);
    bool get_acquisition();
    void send_begin_course();
    void start_course();
    bool begin_reached(int &r);
    bool save_cloud();

    QMutex* mutex;

private:

    typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> syncPolicy;
    typedef Synchronizer<syncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    message_filters::Connection conexao_filter;

    /// Funcoes ///
    double deg2raw(double deg);
    double raw2deg(double raw);
    Eigen::Matrix4f transformFromRaw(double raw);
    void callback(const sensor_msgs::LaserScanConstPtr& msg_laser, const nav_msgs::OdometryConstPtr& msg_motor);
    void send_next_point(int end_point);


    /// Variaveis ///
    int init_argc;
    char** init_argv;

    double raw_min, raw_max, deg_min, deg_max;
    double raw_tilt_hor;
    double deg_raw, raw_deg;
    double raw_ref, deg_ref;
    double dentro;
    double inicio_curso, fim_curso; // em unidades RAW
    double raw_atual;

    // Controle do movimento do motor
    ros::ServiceClient comando_motor;
    int intervalo;
    int ponto_final_temp;

    bool comecar;

    // Variaveis para leitura do laser
    laser_geometry::LaserProjection projector;

    // Variaveis de nuvem
    PointCloud<PointXYZ>::Ptr acc;

    // Variaveis de tf2 para ver no RViz
    geometry_msgs::TransformStamped tf_msg;
    tf2::Quaternion q;

};

}
