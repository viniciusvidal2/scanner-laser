#ifndef SAVEANDWORK_HPP
#define SAVEANDWORK_HPP

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <QObject>
#include <string>
#include <iostream>
#include <istream>
#include <ostream>
#include <ros/ros.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
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
#include <boost/filesystem.hpp>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef PointXYZRGBNormal PointT;
typedef PointXYZRGB PointC;

class SaveAndWork : public QObject
{
  Q_OBJECT
public:
   SaveAndWork(Eigen::Matrix3f K_, Eigen::Matrix4f Tla);
   virtual ~SaveAndWork();
   void init();

   void process_color_and_save(std::vector<cv::Mat> imagens, std::vector<PointCloud<PointC>> nuvens, std::vector<float> angulos, PointCloud<PointC>::Ptr acumulada_colorida);
   void save_angles_file(std::vector<float> in, std::vector<float> fn, std::vector<float> ac);
   void save_image_and_clouds_partial(cv::Mat imagem, cv::Mat imzed, PointCloud<PointC>::Ptr nuvem_astra, PointCloud<PointXYZ>::Ptr nuvem_pixels, size_t indice);

private:

   std::string pasta;
   Eigen::Matrix3f K_astra;
   Eigen::Matrix4f T_laser_astra;

   void process_and_save_final_cloud(PointCloud<PointT>::Ptr entrada);
   void calculate_normals(PointCloud<PointC> entrada, PointCloud<PointT>::Ptr acc_normal);
   Eigen::Matrix4f calculate_transformation(float thetay_deg);
   void project_cloud_to_image(PointCloud<PointC>::Ptr in, cv::Mat img, Eigen::Matrix4f T);
   std::string write_line_for_nvm(float f, std::string nome, Eigen::Quaternion<float> q);
};

#endif // SAVEANDWORK_HPP
