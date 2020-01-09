// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Diversos
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
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
#include <pcl/filters/extract_indices.h>

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
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;

// Variaveis
ros::Publisher pub_cloud, pub_base, pub_jet;
PointCloud<PointT>::Ptr nuvem_base, nuvem_termica, nuvem_acumulada, nuvem_acumulada_termica;
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
/// Processa e publica
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void processAndPublish(Eigen::Matrix4f T, Eigen::Quaternion<float> q, Eigen::Vector3f t){
//    if(!recebendo){
        // Coisas de imagem
        cv_bridge::CvImage jet_msg;
        jet_msg.image = imgCv_;
        jet_msg.encoding = sensor_msgs::image_encodings::RGB8;

        // Inicia as nuvens e mensagem de saida
        nuvem_termica->clear(); nuvem_termica->resize(nuvem_base->size());
        sensor_msgs::PointCloud2 msg_ter, base_msg;

//        PointIndices::Ptr inliers (new PointIndices);
//        ExtractIndices<PointT> extract;
#pragma omp parallel for num_threads(100)
        for(size_t i = 0; i < nuvem_base->size(); i=i+2){

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

//                nuvem_termica->push_back(point_termica);
                nuvem_termica->points[i] = point_termica;

//                inliers->indices.push_back(i);
            }
        }
//        if(inliers->indices.size() > 0){
//            extract.setInputCloud(nuvem_termica);
//            extract.setNegative(false);
//            extract.setIndices(inliers);
//            extract.filter(*nuvem_termica);
//        }

        // Trazer de volta para o frame da ZED, com X para frente da camera
        transformPointCloud(*nuvem_base   , *nuvem_base   , T.inverse());
        transformPointCloud(*nuvem_termica, *nuvem_termica, T.inverse());

        // Colocar no lugar certo segundo Odometria
        transformPointCloud(*nuvem_base   , *nuvem_base   , t, q);
        transformPointCloud(*nuvem_termica, *nuvem_termica, t, q);

        // Acumular nuvens
//        *nuvem_acumulada         += *nuvem_base;
        *nuvem_acumulada_termica += *nuvem_termica;
        ROS_INFO("Tamanho das nuvens acumuladas: %zu   %zu", nuvem_acumulada->size(), nuvem_acumulada_termica->size());

//        loadPLYFile<PointT>("/home/vinicius/Desktop/nuvem_normal_artigo_2.ply", *nuvem_acumulada);

        // Salvar nuvem na hora certa
        if(contador == 8){
            savePLYFileASCII<PointT>("/home/vinicius/Desktop/nuvem_termica_artigo.ply", *nuvem_acumulada_termica);
            savePLYFileASCII<PointT>("/home/vinicius/Desktop/nuvem_normal_artigo.ply" , *nuvem_base        );
            ros::shutdown();
        }
        contador++;

        // Mensagem Nuvem
        nuvem_termica->header.frame_id = nuvem_base->header.frame_id;
        toROSMsg(*nuvem_base        , base_msg);
        toROSMsg(*nuvem_termica, msg_ter );
        msg_ter.header.frame_id  = nuvem_termica->header.frame_id;
        msg_ter.header.stamp     = ros::Time::now();
        base_msg.header.frame_id = nuvem_base->header.frame_id;
        base_msg.header.stamp    = msg_ter.header.stamp;
        jet_msg.header.stamp     = msg_ter.header.stamp;

        // Publicando
        ROS_INFO("Publicando %d ...", contador);
        pub_cloud.publish(msg_ter);
        pub_base.publish(base_msg);
        pub_jet.publish(jet_msg.toImageMsg());
//    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para projecao da nuvem
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void callback(const sensor_msgs::ImageConstPtr&       msg_t ,
              const sensor_msgs::PointCloud2ConstPtr& msg_pc,
              const nav_msgs::OdometryConstPtr&       msg_odo)
{
    recebendo = true;
    ROS_INFO("Recebendo dados sincronizados ...");
    // Coisas de imagem
    cv::Mat img_g;
    cv::Mat imagem_termica_cor;
    imgCv_ = cv_bridge::toCvShare(msg_t, "bgr8")->image;
//    cv::cvtColor(imgCv_, img_g, CV_BGR2GRAY);
//    cv::applyColorMap(img_g, imagem_termica_cor, cv::COLORMAP_JET);
//    imgCv_ = imagem_termica_cor;

    cv_bridge::CvImage jet_msg;
    jet_msg.image = imgCv_;
    jet_msg.encoding = sensor_msgs::image_encodings::BGR8;

    // Recebe Odometria
    Eigen::Quaternion<float> q;
    Eigen::Vector3f          t;
    q.x() = (float)msg_odo->pose.pose.orientation.x;
    q.y() = (float)msg_odo->pose.pose.orientation.y;
    q.z() = (float)msg_odo->pose.pose.orientation.z;
    q.w() = (float)msg_odo->pose.pose.orientation.w;
    t     = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

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

    processAndPublish(T, q, t);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para parametros reconfiguraveis
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void dyn_reconfig_callback(astra_calibrada::term_params_Config &params, uint32_t level){
//  mut.lock();
//  K << params.f,        0, 320,
//              0, params.f, 256,
//              0,        0,   1;

//  RT << 1, 0, 0, params.dx/100,
//        0, 1, 0, params.dy/100,
//        0, 0, 1,        0      ;

//  P = K*RT;

//  cout << "\nMatriz de Projecao:\n" << P << endl;

//  mut.unlock();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "projeta_termica");

    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    nuvem_base              = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    nuvem_acumulada         = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    nuvem_termica           = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    nuvem_acumulada_termica = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    nuvem_base->header.frame_id              = "zed_left_camera";
    nuvem_termica->header.frame_id           = nuvem_base->header.frame_id;
    nuvem_acumulada->header.frame_id         = nuvem_base->header.frame_id;
    nuvem_acumulada_termica->header.frame_id = nuvem_base->header.frame_id;

    // Servidor e callback de parametros de calibracao reconfiguraveis
    dynamic_reconfigure::Server<astra_calibrada::term_params_Config> server;
    dynamic_reconfigure::Server<astra_calibrada::term_params_Config>::CallbackType f;
    f = boost::bind(&dyn_reconfig_callback, _1, _2);
    server.setCallback(f);

    K << 1500,    0, 320,
            0, 1500, 256,
            0,    0,   1;

    RT << 1, 0, 0, -0.075,
          0, 1, 0,  0.045,
          0, 0, 1,  0    ;

    P = K*RT;

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10);
    pub_base  = nh.advertise<sensor_msgs::PointCloud2>("/cloud_base"      , 10);
    pub_jet   = nh.advertise<sensor_msgs::Image      >("/ter_jet"         , 10);

    message_filters::Subscriber<sensor_msgs::Image>       subima(nh, "/dados_sync/image_8bits", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subptc(nh, "/dados_sync/point_cloud", 10);
    message_filters::Subscriber<nav_msgs::Odometry      > subodo(nh, "/dados_sync/odometry"   , 10);
    Synchronizer<syncPolicy> sync(syncPolicy(10), subima, subptc, subodo);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ROS_INFO("Iniciando no ...");

    ros::Rate r(2);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
    }

//    savePLYFileASCII<PointT>("/home/vinicius/Desktop/nuvem_normal_artigo.ply" , *nuvem_acumulada        );
//    savePLYFileASCII<PointT>("/home/vinicius/Desktop/nuvem_termica_artigo.ply", *nuvem_acumulada_termica);

    cout << "\n\nTudo salvo na pasta correta\n\n" << endl;

    return 0;
}
