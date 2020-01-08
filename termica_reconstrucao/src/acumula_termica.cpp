//Includes
#define PCL_NO_PRECOMPILE
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/// Namespaces
using namespace pcl;
using namespace std;
using namespace message_filters;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace Eigen;

/// Definitions
typedef PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, Odometry> syncPolicy;

/// Global vars
PointCloud<PointT >::Ptr accumulated_cloud;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(PointCloud<PointT>::Ptr in, std::string field, float min, float max){
  PassThrough<PointT> ps;
  ps.setInputCloud(in);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(min, max);

  ps.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointT>::Ptr in, float mean, float deviation){
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_grid(PointCloud<PointT>::Ptr in, float lf){
  VoxelGrid<PointT> grid;
  grid.setInputCloud(in);
  grid.setLeafSize(lf, lf, lf);
  grid.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void publicar_nuvem_atual(){
  if (accumulated_cloud->size() > 0){
    sensor_msgs::PointCloud2 msg_out;
    toROSMsg(*accumulated_cloud, msg_out);
    msg_out.header.stamp = ros::Time::now();
    msg_out.header.frame_id = accumulated_cloud->header.frame_id;
    ROS_INFO("Publicando nuvem acumulada TERMICA");
    pub->publish(msg_out);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void acumulador_callback(const sensor_msgs::PointCloud2ConstPtr& msg_ptc_ter,
                       const OdometryConstPtr& msg_odo){
  // Declare variables
  PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>());
  PointCloud<PointT>::Ptr cloud_transformed (new PointCloud<PointT>());
  sensor_msgs::PointCloud2 msg_out;

  // Check for incorrect odometry from viso2
  if(msg_odo->pose.covariance.at(0) > 100){
    ROS_WARN("Nao se pode confiar na odometria, movimento rapido");
    return;
  }

  // Convert the ros message to pcl point cloud
  fromROSMsg (*msg_ptc_ter, *cloud);

  // Filter with passthrough filter -> region to see
//  passthrough(cloud, "z",   0, 20);
//  passthrough(cloud, "x", -10, 10);
//  passthrough(cloud, "y", -10, 10);

  // Remove outiliers
//  remove_outlier(cloud, 10, 1);

  /// Obter a odometria da mensagem
  // Rotacao
  Eigen::Quaternion<double> q;
  q.x() = (double)msg_odo->pose.pose.orientation.x;
  q.y() = (double)msg_odo->pose.pose.orientation.y;
  q.z() = (double)msg_odo->pose.pose.orientation.z;
  q.w() = (double)msg_odo->pose.pose.orientation.w;
  // Translacao
  Eigen::Vector3d offset(msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z);

  // Transformar a nuvem
  transformPointCloud<PointT>(*cloud, *cloud_transformed, offset, q);

  // Accumulate the point cloud using the += operator
  (*accumulated_cloud) += (*cloud_transformed);
  ROS_INFO("Tamanho da nuvem acumulada termica = %ld", accumulated_cloud->points.size());

  // Convert the pcl point cloud to ros msg and publish
//  toROSMsg(*accumulated_cloud, msg_out);
//  msg_out.header.stamp = ros::Time::now();
//  pub->publish(msg_out);

  // Reseta as nuvens
  cloud.reset();
  cloud_transformed.reset();

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "acumula_termica");
  ros::NodeHandle nh;

  // Initialize accumulated cloud variable
  accumulated_cloud = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  accumulated_cloud->header.frame_id = "odom";

  // Initialize the point cloud publisher
  pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_termica", 5);

  // Subscriber para a nuvem instantanea e odometria
  message_filters::Subscriber<sensor_msgs::PointCloud2>  subptcter(nh, "/termica/cloud_inst", 100);
  message_filters::Subscriber<Odometry>                  subodo   (nh, "/termica/odometry"  , 100);

  // Sincroniza as leituras dos topicos (sensores e imagem a principio) em um so callback
  Synchronizer<syncPolicy> sync(syncPolicy(100), subptcter, subodo);
  sync.registerCallback(boost::bind(&acumulador_callback, _1, _2));

  ros::Rate rate(1);
  while(ros::ok()){
    publicar_nuvem_atual();

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
