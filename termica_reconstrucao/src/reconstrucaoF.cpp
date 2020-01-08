/////////////////////////////////////////////////////////////////////
//////////////// Reconstrucao Termica 3D////////////////////////////
////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////


// https://answers.ros.org/question/9705/synchronizer-and-image_transportsubscriber/

// Bibliotecas
#define PCL_NO_PRECOMPILE
#include "ros/ros.h"
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <camera_calibration_parsers/parse.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//using namespace message_filters;
using namespace pcl;
using namespace std;
using namespace message_filters;
using namespace Eigen;

struct PointC
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  float l;
  float o;
  float p;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointC,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, l, l)
                                   (float, o, o)
                                   (float, p, p)
                                   )
typedef pcl::PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::CameraInfo> syncPolicy;




// Classe reconstrucaoTermica
class reconstrucaoTermica
{
protected:
  ros::NodeHandle nh_;
  //image_transport::CameraSubscriber image_sub_;
//  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_felipe;
//  image_transport::ImageTransport it_;
  std::string point_cloud_PLY_;
  ros::Publisher pc_pub_;
  ros::Publisher pc_visual_pub_;
  ros::Publisher pc_completa_pub_;


public:
  ros::Subscriber pc_sub_;
  pcl::PointCloud<PointT> cloud_;
  pcl::PointCloud<PointC> cloudCompleta_;
  sensor_msgs::ImageConstPtr imgMsg_;
  cv::Mat imgCv_;
  cv::Mat imgGray_;
  pcl::PointCloud<PointT> cloudTransformada_;
  image_geometry::PinholeCameraModel camTermicaModelo_;
  camera_info_manager::CameraInfoManager camInfo_;
  sensor_msgs::PointCloud2 completa_out;
  std::string id_;
  int nPontos_;


  // Construtor reconstrucaoTermica
  reconstrucaoTermica(ros::NodeHandle nh_, std::string termicaCalibrationYAML, std::string topico_imagem, std::string topico_pc, std::string topico_out)
    : camInfo_(nh_,"termica",termicaCalibrationYAML)
  {
    //image_sub_ = it_.subscribe(topico_imagem, 1, &reconstrucaoTermica::imageCallback, this);
    //image_sub_ = it_.subscribeCamera(topico_imagem, 1, &reconstrucaoTermica::imageCallback, this);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub_(nh_, "/termica/thermal/camera_info", 200); //
//    message_filters::Subscriber<sensor_msgs::Image>      image_sub_(nh_, topico_imagem, 200); // Vinicius - todo mundo como msg filter pra funcionar
    message_filters::Subscriber<sensor_msgs::Image>      image_sub_(nh_, "termica/thermal/image_raw", 200); // Vinicius - todo mundo como msg filter pra funcionar

    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (topico_out, 1000);
    pc_visual_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("termica/visual_pc", 1000);
    pc_completa_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/completa_pc", 1000);

    //pc_sub_= nh_.subscribe(topico_pc, 1, &reconstrucaoTermica::pointCloudCallback, this);
//    message_filters::Subscriber<sensor_msgs::PointCloud2>  pc_sub_(nh_, topico_pc, 200);
    message_filters::Subscriber<sensor_msgs::PointCloud2>  pc_sub_(nh_, "/stereo/points2", 200);

    camTermicaModelo_.fromCameraInfo(camInfo_.getCameraInfo());

    // Sincronizar geracao da pc termica com a pc visual
    Synchronizer<syncPolicy> sync(syncPolicy(200), pc_sub_, image_sub_, cam_info_sub_);
    sync.registerCallback( boost::bind(&reconstrucaoTermica::imageCallback, this, _1, _2, _3) );
  } //fim reconstrucaoTermica()



  // Callback point cloud
  /*void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
                {
                    id_ = msg->header.frame_id;
                    pcl::fromROSMsg(*msg, cloud_);
                    std::vector<int> indicesNAN;
                    pcl::removeNaNFromPointCloud(cloud_, cloud_, indicesNAN);
                    nPontos_ = int(cloud_.points.size());

                }*/ // fim pointCloudCallback()



  // Callback da imagem termica - Recebe imagem termica
  void imageCallback(const sensor_msgs::PointCloud2ConstPtr& msg_pc, const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ROS_INFO("Entramos no callback dos dados sincronizados.");
    // parte da pc visual
    id_ = msg_pc->header.frame_id;
    pcl::fromROSMsg(*msg_pc, cloud_);
    std::vector<int> indicesNAN;
    pcl::removeNaNFromPointCloud(cloud_, cloud_, indicesNAN);
    nPontos_ = int(cloud_.points.size());
    /////////////////////////////////////////////////////////////


    sensor_msgs::CameraInfo a = *info_msg; //->R[0] = 0;

    a.P[0] = 1400;   // fx (800)
    a.P[1] = 0;     // 0  (0)
    a.P[2] = 330;   // cx (-15)
    a.P[3] = -50;   // Tx (-35)
    a.P[4] = 0;     // 0  (0)
    a.P[5] = 1400;   // fy (800)
    a.P[6] = 270;    // cy (40)
    a.P[7] = -8.16; // Ty (-8.16)
    a.P[8] = 0;     // 0  (0)
    a.P[9] = 0;     // 0  (0)
    a.P[10] = 1;    // 1  (1)
    a.P[11] = 0;    // 0  (0)



    camTermicaModelo_.fromCameraInfo(a);
    cv::Mat img_g;
    cv::Mat imagem_termica_cor;
    imgMsg_ = msg;
    imgCv_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cvtColor(imgCv_, img_g, CV_BGR2GRAY);
    imgGray_ = img_g;
    cv::applyColorMap(img_g, imagem_termica_cor, cv::COLORMAP_JET);
    imgCv_ = imagem_termica_cor;
  } // fim imageCallback()



  // Reconstrucao Termica
  void reconstrucao()
  {

    if(nPontos_ > 0 && nPontos_ < 1000000)
    {
      cloudTransformada_ = cloud_;
      cloudCompleta_.points.resize (nPontos_);

      sensor_msgs::PointCloud2 msg_out;
      sensor_msgs::PointCloud2 visual_out;

      // Projetando os pontos para a imagem
      for(int i = 0; i < nPontos_; i++)
      {
        cv::Point3d ponto3D;
        cv::Point2d pontoProjetado;
        ponto3D.x = cloudTransformada_.points[i].x;
        ponto3D.y = cloudTransformada_.points[i].y;
        ponto3D.z = cloudTransformada_.points[i].z;

        cloudCompleta_.points[i].x = cloudTransformada_.points[i].x;
        cloudCompleta_.points[i].y = cloudTransformada_.points[i].y;
        cloudCompleta_.points[i].z = cloudTransformada_.points[i].z;
        cloudCompleta_.points[i].b = cloudTransformada_.points[i].b;
        cloudCompleta_.points[i].g = cloudTransformada_.points[i].g;
        cloudCompleta_.points[i].r = cloudTransformada_.points[i].r;//*/

        pontoProjetado = camTermicaModelo_.project3dToPixel(ponto3D);
        //std::cout << pontoProjetado << "\n";

        // Verificar se a projecao esta dentro da imagem
        if(pontoProjetado.x > 0  && pontoProjetado.x < imgCv_.cols && pontoProjetado.y > 0 && pontoProjetado.y < imgCv_.rows)
        {
          int b = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[0];
          int g = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[1];
          int r = imgCv_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[2];
          int gray = imgGray_.at<cv::Vec3b>(int(pontoProjetado.y), int(pontoProjetado.x))[0];

          cloudTransformada_.points[i].b = b;
          cloudTransformada_.points[i].g = g;
          cloudTransformada_.points[i].r = r;

          cloudCompleta_.points[i].l = r;
          cloudCompleta_.points[i].o = g;
          cloudCompleta_.points[i].p = b;
        }
        else
        {
          cloudTransformada_.points[i].b = nan("");
          cloudTransformada_.points[i].g = nan("");
          cloudTransformada_.points[i].r = nan("");
          cloudCompleta_.points[i].l = nan("");
          cloudCompleta_.points[i].o = nan("");
          cloudCompleta_.points[i].p = nan("");
        }


      }

      std::vector<int> indicesNAN2;
      pcl::removeNaNFromPointCloud(cloudTransformada_, cloudTransformada_, indicesNAN2);
      pcl::toROSMsg (cloudTransformada_, msg_out);
      pcl::toROSMsg (cloud_, visual_out);
      pcl::toROSMsg (cloudCompleta_, completa_out);


      pc_pub_.publish(msg_out);
      pc_visual_pub_.publish(visual_out);
      completa_out.header.frame_id = id_;
      completa_out.header.stamp = ros::Time::now();
      pc_completa_pub_.publish(completa_out);

    }
  } // fim reconstrucao()



  // Recebe imagem e point cloud, em seguida, processa info
  void spinTermica()
  {
    ros::spinOnce();
    reconstrucao();
  } // fim spinTermica()


}; // fim classe reconstrucaoTermica





// Funcao main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "reconstrucao_tridimensional_termica");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~");
  ROS_INFO_STREAM("Reconstrucao Tridimensional Termica Iniciada.");
  std::string topico_imagem;
  std::string termicaCalibrationYAML;
  std::string topico_pc;
  std::string topico_out;

  n_.getParam("topico_imagem", topico_imagem);
  n_.getParam("topico_out", topico_out);
  n_.getParam("topico_pc", topico_pc);
  n_.getParam("termica_calibration_yaml", termicaCalibrationYAML);
  termicaCalibrationYAML = termicaCalibrationYAML + std::string(".yaml");

//  reconstrucaoTermica rT(nh, termicaCalibrationYAML, topico_imagem, topico_pc, topico_out);
  reconstrucaoTermica rT(n_, termicaCalibrationYAML, topico_imagem, topico_pc, topico_out);

  //    ros::Rate loop_rate(10);
  while ( ros::ok() )
  {
    rT.spinTermica();
    //        loop_rate.sleep();
  }

} // fim main()


