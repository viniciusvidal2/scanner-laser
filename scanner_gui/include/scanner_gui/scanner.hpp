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
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>

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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

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

#include "saveandwork.hpp"

namespace scanner_gui{

using namespace message_filters;
using namespace pcl;
using namespace std;

class Scanner : public QThread
{
    Q_OBJECT
public:
    typedef PointXYZRGB PointC;

    Scanner(int argc, char** argv, QMutex*);
    virtual ~Scanner();
    void init();
    void get_limits(int &minm, int &maxm);
    void set_course(double min, double max);
    void set_acquisition(bool start);
    bool get_acquisition();
    void send_begin_course();
    void send_to_opposite_edge(int t);
    void start_course();
    bool begin_reached(int &r);
    bool save_data();
    void set_trips(int t);
    void set_overlap(float o);
    int get_current_position();
    int get_current_trip();

    QMutex* mutex;

Q_SIGNALS:
    void new_step();
    void going_to_start_point();
    void saving();

private:
    // Ouve laser e motores, acumula nuvem
    typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> syncPolicy;
    typedef Synchronizer<syncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    message_filters::Connection conexao_filter;

    // Ouve motores e astra para capturar imagem e nuvens posicionadas
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy2;
    typedef Synchronizer<syncPolicy2> Sync2;
    boost::shared_ptr<Sync2> sync2;
    message_filters::Connection conexao_filter2;

    /// Funcoes ///
    double deg2raw(double deg);
    double raw2deg(double raw);
    Eigen::Matrix4f transformFromRaw(double raw);
    void accumulate_parcial_cloud(PointCloud<PointC>::Ptr cloud, double ang_raw);
    void callback(const sensor_msgs::LaserScanConstPtr& msg_laser,
                  const nav_msgs::OdometryConstPtr& msg_motor);
    void callback2(const nav_msgs::OdometryConstPtr& msg_motor,
                   const sensor_msgs::ImageConstPtr& msg_imagem,
                   const sensor_msgs::ImageConstPtr& msg_imagem_zed,
                   const sensor_msgs::PointCloud2ConstPtr& msg_nuvem,
                   const sensor_msgs::PointCloud2ConstPtr& msg_pixels);

    /// Variaveis ///
    int init_argc;
    char** init_argv;

    double raw_min, raw_max, deg_min, deg_max;
    double deg_raw, raw_deg;
    double raw_ref, deg_ref;
    double dentro;
    double inicio_curso, fim_curso; // em unidades RAW
    double raw_atual;

    double FOV_astra;
    float  overlap; // [DEGREES]

    // Vetores de controle de angulos de aquisicao de imagens e nuvens
    vector<float> angulos_captura;
    vector<float> final_nuvens;    // [DEGREES]
    vector<float> inicio_nuvens;
    vector<PointCloud<PointC>> nuvens_parciais; // Nuvens do laser
    vector<cv::Mat> imagens_parciais; // Imagens da astra

    // Controle do movimento do motor
    ros::ServiceClient comando_motor;

    int viagens, viagem_atual;

    bool comecar;

    // Controle de captura de dados da camera
    int capturar_camera;

    // Variaveis para leitura do laser
    laser_geometry::LaserProjection projector;

    // Variaveis de nuvem
    PointCloud<PointC>::Ptr acc;

    // Variaveis de tf2 para ver no RViz
    geometry_msgs::TransformStamped tf_msg;
    tf2::Quaternion q;

    // Objeto da classe de processo e salvar os dados
    SaveAndWork* saw;

};

}
