// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Diversos
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
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

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <astra_calibrada/term_params_Config.h>

// Message filter
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace tf;
using namespace message_filters;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis e definicoes globais
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definicoes
typedef PointXYZRGB PointT;
typedef PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> syncPolicy;

// Variaveis
ros::Publisher pub_cloud, pub_base, pub_jet;
PointCloud<PointT>::Ptr nuvem_base, nuvem_termica;
// Matriz intrinseca K para Thermal cam
Eigen::Matrix3f K;
int contador = 0;
// Calculando Params. Extrisecos
Eigen::MatrixXf RT(3, 4), P;
// Resolucao da nuvem
int resolucao = 1;
// Mutex para parar a publicacao
mutex mut;
// Imagem recebida ja em JET
cv::Mat imgCv_;
// Controle de recebimento e processamento
bool recebendo = true;

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
void callback(const sensor_msgs::ImageConstPtr& msg_t,
              const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{
    recebendo = true;
    // Coisas de imagem
    cv::Mat img_g;
    cv::Mat imagem_termica_cor;
    imgCv_ = cv_bridge::toCvShare(msg_t, "rgb8")->image;
    cv::cvtColor(imgCv_, img_g, CV_BGR2GRAY);
    cv::applyColorMap(img_g, imagem_termica_cor, cv::COLORMAP_JET);
    imgCv_ = imagem_termica_cor;

    cv_bridge::CvImage jet_msg;
    jet_msg.image = imgCv_;
    jet_msg.encoding = sensor_msgs::image_encodings::BGR8;

    // Inicia as nuvens e mensagem de saida
    fromROSMsg(*msg_pc, *nuvem_base);
    remove_outlier(nuvem_base, 10, 1);
    nuvem_termica->clear(); nuvem_termica->resize(nuvem_base->size());

    // Rotaciona a nuvem para o frame onde Z esta para frente (se ja estiver, comentar)
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();     // Matriz de transformaçao homogenea
    T.block<3, 3>(0, 0) = R;                             // Adiciona a rotacao onde deve estar
    transformPointCloud(*nuvem_base, *nuvem_base, T);

    recebendo = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para parametros reconfiguraveis
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void dyn_reconfig_callback(astra_calibrada::term_params_Config &params, uint32_t level){
  mut.lock();
  K << params.f,        0, 320,
              0, params.f, 256,
              0,        0,   1;

  RT << 1, 0, 0, params.dx/100,
        0, 1, 0, params.dy/100,
        0, 0, 1,        0      ;

  P = K*RT;

  cout << "\nMatriz de Projecao:\n" << P << endl;

  mut.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Processa e publica
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void processAndPublish(){
    if(!recebendo){
        // Coisas de imagem
        cv_bridge::CvImage jet_msg;
        jet_msg.image = imgCv_;
        jet_msg.encoding = sensor_msgs::image_encodings::BGR8;

        // Inicia as nuvens e mensagem de saida
        nuvem_termica->clear(); nuvem_termica->resize(nuvem_base->size());
        sensor_msgs::PointCloud2 msg_ter, base_msg;

#pragma omp parallel for num_threads(100)
        for(size_t i = 0; i < nuvem_base->size(); i=i+3){

            Eigen::MatrixXf ponto3D(4, 1);
            ponto3D << nuvem_base->points[i].x, nuvem_base->points[i].y, nuvem_base->points[i].z, 1;
            Eigen::MatrixXf ponto2D;

            ponto2D = P*ponto3D;
            ponto2D = ponto2D / ponto2D(2);

            // Verificar se a projecao esta dentro da imagem
            if(ponto2D(0) > 0  && ponto2D(0) < imgCv_.cols && ponto2D(1) > 0 && ponto2D(1) < imgCv_.rows)
            {
                PointT point_termica;
                int b = imgCv_.at<cv::Vec3b>(int(ponto2D(1)), int(ponto2D(0)))[0];
                int g = imgCv_.at<cv::Vec3b>(int(ponto2D(1)), int(ponto2D(0)))[1];
                int r = imgCv_.at<cv::Vec3b>(int(ponto2D(1)), int(ponto2D(0)))[2];

                point_termica.x = ponto3D(0);
                point_termica.y = ponto3D(1);
                point_termica.z = ponto3D(2);
                point_termica.r = r;
                point_termica.g = g;
                point_termica.b = b;

                nuvem_termica->points[i] = point_termica;
            }
        }

        nuvem_termica->header.frame_id = "zed_left_optical";
        // Mensagem Nuvem
        toROSMsg(*nuvem_base   , base_msg);
        toROSMsg(*nuvem_termica, msg_ter );
        msg_ter.header.frame_id  = nuvem_termica->header.frame_id;
        msg_ter.header.stamp     = ros::Time::now();
        base_msg.header.frame_id = "zed_left_optical";
        base_msg.header.stamp    = msg_ter.header.stamp;
        jet_msg.header.stamp     = msg_ter.header.stamp;

        // Publicando
        ROS_INFO("Publicando (nuvem com %zu pontos normais e %zu termicos)...", base_msg.data.size(), msg_ter.data.size());
        pub_cloud.publish(msg_ter);
        pub_base.publish(base_msg);
        pub_jet.publish(jet_msg.toImageMsg());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "projeta_termica");

    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    nuvem_base = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
//    loadPLYFile("/home/vinicius/Desktop/prova_conceito_artigo.ply", *nuvem_base);
    nuvem_base->header.frame_id = "zed_left_optical";

    nuvem_termica = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    nuvem_termica->header.frame_id = "zed_left_optical";

    // Servidor e callback de parametros de calibracao reconfiguraveis
    dynamic_reconfigure::Server<astra_calibrada::term_params_Config> server;
    dynamic_reconfigure::Server<astra_calibrada::term_params_Config>::CallbackType f;
    f = boost::bind(&dyn_reconfig_callback, _1, _2);
    server.setCallback(f);

    K << 1580,    0, 320,
            0, 1580, 256,
            0,    0,   1;

    RT << 1, 0, 0,  0.045,
          0, 1, 0, -0.045,
          0, 0, 1,  0    ;

    P = K*RT;

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10);
    pub_base  = nh.advertise<sensor_msgs::PointCloud2>("/cloud_base"      , 10);
    pub_jet   = nh.advertise<sensor_msgs::Image      >("/ter_jet"         , 10);

//    ros::Subscriber ter_sub = nh.subscribe("/thermal/image_raw", 1000, callback);
    message_filters::Subscriber<sensor_msgs::Image>       subima(nh, "/dados_sync/image_8bits"          , 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subptc(nh, "/zed/point_cloud/cloud_registered", 100);
    Synchronizer<syncPolicy> sync(syncPolicy(100), subima, subptc);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate r(2);
    while(ros::ok()){
        r.sleep();
        processAndPublish();
        ros::spinOnce();
    }

    return 0;
}
