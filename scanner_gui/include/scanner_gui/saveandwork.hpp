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
   SaveAndWork();
   virtual ~SaveAndWork();
   void init();

   void process_color_and_save(std::vector<cv::Mat> imagens, std::vector<PointCloud<PointXYZ>> nuvens, PointCloud<PointXYZ>::Ptr acumulada_raiz, PointCloud<PointC>::Ptr acumulada_colorida);
   void save_angles_file(std::vector<float> in, std::vector<float> fn, std::vector<float> ac);
   void save_image_and_clouds_partial(cv::Mat imagem, PointCloud<PointC>::Ptr nuvem_astra, PointCloud<PointXYZ>::Ptr nuvem_pixels, size_t indice);

private:

   std::string pasta;

   void process_and_save_final_cloud(PointCloud<PointC>::Ptr entrada);
   void calculate_normals(PointCloud<PointC>::Ptr entrada, PointCloud<PointT>::Ptr acc_normal);

};

#endif // SAVEANDWORK_HPP
