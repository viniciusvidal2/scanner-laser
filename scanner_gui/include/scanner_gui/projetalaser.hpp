#ifndef PROJETALASER_HPP
#define PROJETALASER_HPP

#endif // PROJETALASER_HPP

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>

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

class ProjetaLaser : public QThread
{
    Q_OBJECT
public:
    typedef PointXYZRGB PointT;
    ProjetaLaser(Eigen::Matrix3f K_, Eigen::Matrix4f Tla, std::string p);
    void init();
    virtual ~ProjetaLaser();

    void set_capture_angles(std::vector<float> in);
    void set_cloud_vector(std::vector<PointCloud<PointT>> clouds);

    void process(int indice);



private:

    std::vector<float> angulos;
    std::vector<PointCloud<PointT>> nuvens;
    std::string pasta;

    Eigen::Matrix3f K;
    Eigen::Matrix4f T_eixos;

    Eigen::Matrix4f calculate_transformation(float thetay_deg);
    void color_cloud_depth(PointCloud<PointT>::Ptr cloud);
    float normaldist(float x, float media, float dev);

};

