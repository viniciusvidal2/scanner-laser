// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Diversos
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>

// EIGEN
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

// Messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <astra_calibrada/calib_params_Config.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace tf;
using namespace message_filters;
using namespace nav_msgs;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis e definicoes globais
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definicoes
typedef PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;

// Variaveis
ros::Publisher pub_cloud;
PointCloud<PointT>::Ptr nuvem_acumulada;
// Matriz intrinseca K para Depth cam
Eigen::Matrix3f K1;
float fxd, fyd, Cxd, Cyd;
int contador = 0;
// Matriz intrinseca K para RGB cam
Eigen::Matrix3f K2;
// Calculando Params. Extrisecos
Eigen::MatrixXf RT(3, 4), P;
// Resolucao da nuvem
int resolucao = 1;
// Mutex para parar a publicacao
mutex mut;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Filtro para distâncias
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(pcl::PointCloud<PointT>::Ptr in, std::string field, float min, float max){
    pcl::PassThrough<PointT> ps;
    ps.setInputCloud(in);
    ps.setFilterFieldName(field);
    ps.setFilterLimits(min, max);

    ps.filter(*in);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Voxel Grid
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void voxelgrid_nuvem(PointCloud<PointT>::Ptr in, float lf){
    VoxelGrid<PointT> grid;
    grid.setLeafSize(lf, lf, lf);
    grid.setInputCloud(in);
    grid.filter(*in);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Removendo Outliers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointT>::Ptr in, float mean, float deviation){
    StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(in);
    sor.setMeanK(mean);
    sor.setStddevMulThresh(deviation);
    sor.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para projecao da nuvem
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void callback(const sensor_msgs::ImageConstPtr& msg_rgb,
              const sensor_msgs::ImageConstPtr& msg_depth)
{
    cv_bridge::CvImagePtr cv_ptr_d;
    cv_bridge::CvImagePtr cv_prt_rgb;
    cv_ptr_d   = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    cv_prt_rgb = cv_bridge::toCvCopy(msg_rgb  , sensor_msgs::image_encodings::RGB8      );
    sensor_msgs::PointCloud2 msg_cor;

    PointCloud<PointT>::Ptr nuvem_colorida (new PointCloud<PointT>());

    int depthHeight = cv_ptr_d->image.rows;
    int depthWidth  = cv_ptr_d->image.cols;

    resolucao = resolucao > 0 ? resolucao : 2;
//    #pragma omp parallel for num_threads(30)
    for(int v = 0; v < depthHeight; v=v+resolucao){
//        #pragma omp parallel for num_threads(int(depthWidth/10))
        for(int u = 0; u < depthWidth; u=u+resolucao){

            float x, y, z;
            PointT current_point;
            cv::Vec3b intensity;

            z = cv_ptr_d->image.at<short int>(v, u);
            if(z!=0){ // eliminando pontos de prof. zero
                x = ((u - Cxd)*z)/fxd;
                y = ((v - Cyd)*z)/fyd;

                // Projetando...
                Eigen::MatrixXf X_(4,1);
                X_ << x,
                      y,
                      z,
                      1;
                Eigen::MatrixXf X = P*X_;
                X = X/X(2,0);

                // Adicionando o ponto sem conferencia da matriz fundamental
                if(floor(X(0,0)) >= 0 && floor(X(0,0)) < cv_prt_rgb->image.cols && floor(X(1,0)) >= 0 && floor(X(1,0)) < cv_prt_rgb->image.rows){
                    float s = 1000;
                    current_point.z = z/s;
                    current_point.x = x/s;
                    current_point.y = y/s;

                    intensity = cv_prt_rgb->image.at<cv::Vec3b>(floor(X(1,0)), floor(X(0,0)));
                    current_point.r = intensity.val[0];
                    current_point.g = intensity.val[1];
                    current_point.b = intensity.val[2];

                    nuvem_colorida->push_back(current_point);
                }
            }
        } // fim do for u
    } // fim do for v

    nuvem_colorida->header.frame_id = "camera_rgb_optical_frame";
    // Mensagem Nuvem
    toROSMsg(*nuvem_colorida, msg_cor);
    msg_cor.header.frame_id = nuvem_colorida->header.frame_id;
    msg_cor.header.stamp = ros::Time::now();

    if(contador < 1){
        *nuvem_acumulada += *nuvem_colorida;
    } else {
        savePLYFileASCII("/home/vinicius/Desktop/prova_conceito_artigo.ply", *nuvem_acumulada);
        ros::shutdown();
    }
    contador++;

    // Publicando
    ROS_INFO("Publicando...");
    pub_cloud.publish(msg_cor);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para parametros reconfiguraveis
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void dyn_reconfig_callback(astra_calibrada::calib_params_Config &params, uint32_t level){
  mut.lock();
  RT << 1, 0, 0, params.dx,
        0, 1, 0, params.dy,
        0, 0, 1, 0        ;

  P = K2*RT;

  cout << "\nMatriz de Projecao:\n" << P << endl;
  mut.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "funde_ivensese");

    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    nuvem_acumulada = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    nuvem_acumulada->header.frame_id = "camera_rgb_optical_frame";

    // Servidor e callback de parametros de calibracao reconfiguraveis
    dynamic_reconfigure::Server<astra_calibrada::calib_params_Config> server;
    dynamic_reconfigure::Server<astra_calibrada::calib_params_Config>::CallbackType f;
    f = boost::bind(&dyn_reconfig_callback, _1, _2);
    server.setCallback(f);

    // Variáveis
    K1 <<  622.772216796875,   0.0           , 314.8291931152344, // Camera de profundidade, mensagem depth
             0.0           , 622.772216796875, 233.9505157470703,
             0.0           ,   0.0           ,   1.0            ;

    fxd = K1(0,0);
    fyd = K1(1,1);
    Cxd = K1(0,2);
    Cyd = K1(1,2);

    K2 <<   617.273193359375,   0.0           , 329.1241760253906 , // Camera colorida
              0.0           , 616.837158203125, 235.95721435546875,
              0.0           ,   0.0           ,   1.0             ;

    RT << 1, 0, 0, 10,
          0, 1, 0,  4,
          0, 0, 1,  0;

    P = K2*RT;

    pub_cloud  = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(  nh, "/device_0/sensor_1/Color_0/image/data", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/device_0/sensor_0/Depth_0/image/data", 10);
    Synchronizer<syncPolicy> sync(syncPolicy(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}
