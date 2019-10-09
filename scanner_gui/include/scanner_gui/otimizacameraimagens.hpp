#ifndef OTIMIZACAMERAIMAGENS_HPP
#define OTIMIZACAMERAIMAGENS_HPP

#include <QObject>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/correspondence_rejection_distance.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ctime>

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

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace pcl::io;
using namespace cv;
using namespace pcl;
using namespace std;

class OtimizaCameraImagens : public QObject
{
  Q_OBJECT
public:
    typedef PointXYZRGB PointC;
    typedef PointXYZRGBNormal PointCN;

    OtimizaCameraImagens();
    virtual ~OtimizaCameraImagens();

    void process_pipeline();
    void init();

private:

    // Estrutura para guardar os dados da camera interessantes aqui apos otimizados por bat
    struct camera{
        camera() {}
        float foco;
        Eigen::Matrix4f T;
    };

    void comparaSift(cv_bridge::CvImagePtr astra, cv_bridge::CvImagePtr zed, PointCloud<PointC>::Ptr cloud);
    void resolveAstraPixeis(PointCloud<PointXYZ>::Ptr pixeis, PointCloud<PointC>::Ptr nuvem_total_bat, cv_bridge::CvImagePtr zed);
    void updateRTFromSolvePNP(std::vector<cv::Point2f> imagePoints, std::vector<cv::Point3f> objectPoints, cv_bridge::CvImagePtr zed);
    void printT(Eigen::Matrix4f T);
    camera bat(std::vector<Point2f> xy_zed, std::vector<Point3f> X_zed, Eigen::Matrix4f T_est, float foco_est, Eigen::Vector2f c_img, bool &valid);
    float  fob(std::vector<Point2f> xy_zed, std::vector<Point3f> X_zed, Eigen::Matrix4f T_est, Eigen::MatrixXf bat, Eigen::Vector2f c_est, Eigen::MatrixXf range);



    float fzed, fastra;
    Eigen::Matrix4f T_projecao_atual;
    Eigen::Matrix4f T_astra_zed, T_astra_zed_original;

    std::vector<cv::KeyPoint> goodKeypointsLeft;  // astra
    std::vector<cv::KeyPoint> goodKeypointsRight; // zed
    // Variaveis para guardar os pontos 2D corretos e os 3D correspondentes
    std::vector<Point2f> imagePointsZed;
    std::vector<Point3f> objectPointsZed;


};

#endif // OTIMIZACAMERAIMAGENS_HPP
