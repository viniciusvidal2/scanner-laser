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
typedef PointXYZRGBNormal PointT;

class ProjetaLaser : public QThread
{
    Q_OBJECT
public:
    ProjetaLaser(int argc, char **argv);
    void init();
    virtual ~ProjetaLaser();

    void set_angulos(float min, float max);
    void set_nuvem(PointCloud<PointT>::Ptr cloud);

    void process();



private:
    int init_argc;
    char **init_argv;

    float angulo_central;
    float angulo_min;     // ANGULOS EM PAN, GRAUS
    float angulo_max;
    PointCloud<PointT>::Ptr nuvem;

    Eigen::Matrix3f K;
    Eigen::Matrix4f T_eixos;

};

