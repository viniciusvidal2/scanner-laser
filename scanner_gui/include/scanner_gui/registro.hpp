#ifndef REGISTRO_HPP
#define REGISTRO_HPP

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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
#include "saveandwork.hpp"


using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace cv;

class Registro : public QObject
{
  Q_OBJECT
public:
   Registro();
   virtual ~Registro();
   void init();
   void run(std::vector<cv::Mat> imagens_zed, std::vector<cv::Mat> imagens_astra,
			           std::vector<PointCloud<PointXYZ>> nuvens_astra, std::vector<PointCloud<PointXYZ>> nuvens_laser,
					   std::vector<float> angulos_captura);
   void process(std::string directory, std::string ext);
					   					   
private:
        // typedef
        typedef PointNormal PointT;
        typedef PointXYZRGBNormal PointF;

	// Variáveis
	Eigen::Matrix3f K_astra; // Matriz de calibração da câmera ASTRA
	Eigen::Matrix3f K_zed; // Matriz de calibração da câmera ZED
	PointCloud<PointXYZ>::Ptr acc; // Ponteiro que faz referência a point cloud acumulada do LASER
	PointCloud<PointXYZRGB>::Ptr acc_cor; // Ponteiro que faz referência a point cloud acumulada colorida do LASER 
	SaveAndWork* saw; // Objeto utilizado para salvar point clouds em formato .ply
        double profundidade_icp;
   
    // Funções privadas
        int count_files(std::string directory, std::string ext);
        std::vector<float> ler_angulos_captura(std::string directory);
        Eigen::Matrix4f transformada_laser_rot(float theta); // Função que gera a matriz de rotação referente a um ângulo THETA
        Eigen::Matrix4f transformada_laser2astra(); // Função que retorna a matriz de transformação entre o LASER e a câmera ASTRA ...
        Eigen::Matrix4f transformada_astra2zed(); // Funcao que retorna a matriz de transformacao entre a camera ASTRA e a camera ZED
        PointCloud<PointXYZRGB> projetar_3d_2_2d(PointCloud<PointXYZ> nuvem_in,   // Gera uma point cloud colorida projetando uma nuvem NUVEM_IN ...
                                                           cv::Mat img, Eigen::Matrix3f K,  // para uma imagem IMG utilizando uma matriz de calibração K ...
                                                           Eigen::Matrix4f T);              // e uma matriz de transformação T.
        void filter_grid(PointCloud<PointT>::Ptr cloud, float leaf_size);                            // Filtragem (funcao 1)
        void filter_grid(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr out, float leaf_size);  // Filtragem (funcao 2)
        void filter_grid(PointCloud<PointF>::Ptr in, PointCloud<PointF>::Ptr out, float leaf_size);  // Filtragem (funcao 3)
        Eigen::Matrix4f icp(const PointCloud<PointT>::Ptr src,  // Aplica a técnica ICP entre uma nuvem SRC e uma nuvem TGT ...
                                      const PointCloud<PointT>::Ptr tgt, // com um chute inicial de transformação T. Retorna a transformação T_icp.
                                      Eigen::Matrix4f T);
};

#endif 
